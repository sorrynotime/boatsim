from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

from boat_sim.model import BoatParameters, DifferentialBoat, GpsSensor, gps_to_local, local_to_gps
from boat_sim.protocol import encode_sensor, encode_target
from boat_sim.route import MAX_WAYPOINTS, RoutePlan, Waypoint
from boat_sim.serial_link import SerialLink
from boat_sim.signal_mapping import map_motor_commands, map_yaw_angle


SIM_DT = 0.01
GPS_PERIOD_S = 0.2


class BoatSimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("差速船闭环仿真器")
        self.root.geometry("1200x760")
        self.root.minsize(900, 600)
        self.boat = DifferentialBoat()
        self.gps = GpsSensor()
        self.link = SerialLink()
        self.route = RoutePlan()
        self.running = False
        self.sim_time = 0.0
        self.accumulator = 0.0
        self.last_tick = time.perf_counter()
        self.last_draw = self.last_tick
        self.last_gps_time = -GPS_PERIOD_S
        self.last_sensor_send = -SIM_DT
        self.sequence = 0
        self.latest_gps = (30.000000, 120.000000)
        self.true_path: list[tuple[float, float]] = []
        self.gps_path: list[tuple[float, float]] = []
        self.ekf_path: list[tuple[float, float]] = []
        self.latest_ekf_gps: tuple[float, float] | None = None
        self.last_ekf_frame: tuple[int, int] | None = None
        self.arrival_since: float | None = None
        self.arrived = False
        self.view_extent = 10.0
        self.vars: dict[str, tk.Variable] = {}
        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(10, self._tick)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(fill="x")
        self.run_button = ttk.Button(toolbar, text="开始", command=self._toggle_running)
        self.run_button.pack(side="left")
        ttk.Button(toolbar, text="复位", command=self._reset).pack(side="left", padx=(6, 12))
        ttk.Label(toolbar, text="串口").pack(side="left")
        self.port_combo = ttk.Combobox(toolbar, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(toolbar, text="刷新", command=self._refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(toolbar, text="连接", command=self._toggle_connection)
        self.connect_button.pack(side="left", padx=(4, 12))
        self.manual_mode = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="本地手动电机", variable=self.manual_mode).pack(side="left")
        self.status_var = tk.StringVar(value="未连接 / 已停止")
        ttk.Label(toolbar, textvariable=self.status_var).pack(side="right")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True)
        controls = ttk.Frame(body, padding=6)
        plot_frame = ttk.Frame(body, padding=(0, 8, 8, 8))
        body.add(controls, weight=0)
        body.add(plot_frame, weight=1)

        control_tabs = ttk.Notebook(controls)
        control_tabs.pack(fill="x")
        operation_tab = ttk.Frame(control_tabs, padding=4)
        setup_tab = ttk.Frame(control_tabs, padding=4)
        control_tabs.add(operation_tab, text="目标与电机")
        control_tabs.add(setup_tab, text="参数与方向")

        params = ttk.LabelFrame(setup_tab, text="船体参数", padding=6)
        params.pack(fill="x")
        self._entry(params, "质量 (kg)", "mass", 20.0)
        self._entry(params, "电机间距 (m)", "spacing", 0.60)
        self._entry(params, "最大速度 (m/s)", "max_speed", 3.0)
        self._entry(params, "最大转向速度 (rad/s)", "max_yaw", 1.2)
        self._entry(params, "单电机最大推力 (N)", "max_thrust", 35.0)
        self._entry(params, "电机响应时间 (s)", "motor_tau", 0.25)
        ttk.Button(params, text="应用参数", command=self._apply_parameters).pack(fill="x", pady=(4, 0))

        location = ttk.LabelFrame(operation_tab, text="坐标与目标", padding=6)
        location.pack(fill="x")
        self._entry(location, "原点纬度", "lat0", 30.0)
        self._entry(location, "原点经度", "lon0", 120.0)
        self._entry(location, "目标东向 (m)", "target_east", 20.0)
        self._entry(location, "目标北向 (m)", "target_north", 30.0)
        self._entry(location, "到达半径 (m)", "arrival_radius", 2.0)
        self._entry(location, "GPS 精度半径 (m)", "gps_accuracy", 1.5)
        target_actions = ttk.Frame(location)
        target_actions.pack(fill="x", pady=(4, 0))
        ttk.Button(target_actions, text="下发目标", command=self._send_target).pack(side="left", fill="x", expand=True)
        ttk.Button(target_actions, text="复制 sim target", command=self._copy_target_coordinates).pack(side="left", fill="x", expand=True, padx=(6, 0))
        self.picked_coordinate_var = tk.StringVar(value="sim target 30.0002695 120.0002075")
        ttk.Label(location, textvariable=self.picked_coordinate_var).pack(fill="x", pady=(4, 0))

        motors = ttk.LabelFrame(operation_tab, text="手动电机 (-10000..10000)", padding=6)
        motors.pack(fill="x", pady=(6, 0))
        self._scale(motors, "左", "manual_left")
        self._scale(motors, "右", "manual_right")

        mapping = ttk.LabelFrame(setup_tab, text="方向映射", padding=6)
        mapping.pack(fill="x", pady=(6, 0))
        self.invert_yaw = tk.BooleanVar(value=False)
        self.swap_motors = tk.BooleanVar(value=True)
        self.invert_left_motor = tk.BooleanVar(value=False)
        self.invert_right_motor = tk.BooleanVar(value=False)
        ttk.Checkbutton(mapping, text="陀螺仪角度方向取反", variable=self.invert_yaw).pack(anchor="w")
        ttk.Checkbutton(mapping, text="交换左右电机", variable=self.swap_motors).pack(anchor="w")
        ttk.Checkbutton(mapping, text="左电机取反", variable=self.invert_left_motor).pack(anchor="w")
        ttk.Checkbutton(mapping, text="右电机取反", variable=self.invert_right_motor).pack(anchor="w")

        self.telemetry_var = tk.StringVar()
        ttk.Label(controls, textvariable=self.telemetry_var, justify="left").pack(fill="x", pady=(6, 0))

        route_panel = ttk.LabelFrame(plot_frame, text=f"航线规划 (最多 {MAX_WAYPOINTS} 点)", padding=8)
        route_panel.pack(side="left", fill="y", padx=(0, 8))
        self.route_list = tk.Listbox(route_panel, width=30, height=16, exportselection=False)
        self.route_list.pack(fill="both", expand=True)
        ttk.Button(route_panel, text="加入当前拾取点", command=self._add_waypoint).pack(fill="x", pady=(6, 0))
        route_edit_actions = ttk.Frame(route_panel)
        route_edit_actions.pack(fill="x", pady=(6, 0))
        ttk.Button(route_edit_actions, text="删除", command=self._delete_waypoint).pack(side="left", fill="x", expand=True)
        ttk.Button(route_edit_actions, text="清空", command=self._clear_route).pack(side="left", fill="x", expand=True, padx=(6, 0))
        route_run_actions = ttk.Frame(route_panel)
        route_run_actions.pack(fill="x", pady=(6, 0))
        ttk.Button(route_run_actions, text="开始航线", command=self._start_route).pack(side="left", fill="x", expand=True)
        ttk.Button(route_run_actions, text="停止", command=self._stop_route).pack(side="left", fill="x", expand=True, padx=(6, 0))
        self.route_status_var = tk.StringVar(value="航线未启动")
        ttk.Label(route_panel, textvariable=self.route_status_var, wraplength=200).pack(fill="x", pady=(8, 0))

        self.canvas = tk.Canvas(plot_frame, background="#f7fafc", highlightthickness=1, highlightbackground="#9ca3af")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw())
        self.canvas.bind("<Button-1>", self._pick_target)

    def _entry(self, parent: ttk.Widget, label: str, key: str, default: float) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text=label, width=23).pack(side="left")
        variable = tk.DoubleVar(value=default)
        self.vars[key] = variable
        ttk.Entry(row, textvariable=variable, width=11).pack(side="right")

    def _scale(self, parent: ttk.Widget, label: str, key: str) -> None:
        variable = tk.IntVar(value=0)
        self.vars[key] = variable
        row = ttk.Frame(parent)
        row.pack(fill="x")
        ttk.Label(row, text=label, width=3).pack(side="left")
        ttk.Scale(row, from_=-10000, to=10000, variable=variable).pack(side="left", fill="x", expand=True)
        ttk.Label(row, textvariable=variable, width=7, anchor="e").pack(side="right")

    def _value(self, key: str) -> float:
        return float(self.vars[key].get())

    def _apply_parameters(self) -> None:
        try:
            values = [self._value(key) for key in ("mass", "spacing", "max_speed", "max_yaw", "max_thrust", "motor_tau")]
            if any(value <= 0 for value in values):
                raise ValueError("参数必须全部大于 0")
            self.boat.parameters = BoatParameters(*values)
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("参数错误", str(exc))

    def _refresh_ports(self) -> None:
        ports = self.link.available_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])

    def _toggle_connection(self) -> None:
        if self.link.connected:
            self.link.disconnect()
            self.connect_button.configure(text="连接")
            return
        if not self.port_combo.get():
            messagebox.showerror("串口", "没有选择串口，或尚未安装 pyserial")
            return
        try:
            self.link.connect(self.port_combo.get())
            self.connect_button.configure(text="断开")
            self._send_target()
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def _toggle_running(self) -> None:
        self.running = not self.running
        self.run_button.configure(text="暂停" if self.running else "开始")
        self.last_tick = time.perf_counter()

    def _reset(self) -> None:
        self.running = False
        self.route.stop()
        self.route_status_var.set("仿真已复位，航线已停止")
        self._refresh_route_list()
        self.run_button.configure(text="开始")
        self.boat.reset()
        self.sim_time = 0.0
        self.accumulator = 0.0
        self.last_gps_time = -GPS_PERIOD_S
        self.last_sensor_send = -SIM_DT
        self.true_path.clear()
        self.gps_path.clear()
        self.ekf_path.clear()
        self.latest_ekf_gps = None
        self.last_ekf_frame = None
        self.arrival_since = None
        self.arrived = False
        self.view_extent = 10.0
        self.vars["manual_left"].set(0)
        self.vars["manual_right"].set(0)
        self._draw()

    def _send_target(self) -> None:
        try:
            latitude, longitude = self._update_picked_coordinate()
            frame = encode_target(self.sequence, int(self.sim_time * 1000), latitude, longitude, self._value("arrival_radius"))
            self.sequence += 1
            self.link.send(frame)
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("目标参数错误", str(exc))

    def _target_gps(self) -> tuple[float, float]:
        return local_to_gps(
            self._value("target_east"), self._value("target_north"), self._value("lat0"), self._value("lon0")
        )

    def _update_picked_coordinate(self) -> tuple[float, float]:
        latitude, longitude = self._target_gps()
        self.picked_coordinate_var.set(self._format_target_command(latitude, longitude))
        return latitude, longitude

    @staticmethod
    def _format_target_command(latitude: float, longitude: float) -> str:
        return f"sim target {latitude:.7f} {longitude:.7f}"

    def _copy_target_coordinates(self) -> None:
        try:
            latitude, longitude = self._update_picked_coordinate()
            self.root.clipboard_clear()
            self.root.clipboard_append(self._format_target_command(latitude, longitude))
            self.status_var.set("sim target 命令已复制")
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("目标参数错误", str(exc))

    def _pick_target(self, event: tk.Event) -> None:
        if self.route.running:
            self.route_status_var.set("请先停止航线，再拾取或编辑航点")
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        scale = getattr(self, "view_scale", 1.0)
        east = (event.x - width / 2) / scale
        north = (height / 2 - event.y) / scale
        self.vars["target_east"].set(round(east, 3))
        self.vars["target_north"].set(round(north, 3))
        self._update_picked_coordinate()
        self._draw()

    def _add_waypoint(self) -> None:
        try:
            self.route.add(self._value("target_east"), self._value("target_north"))
            self._refresh_route_list()
            self.route_list.selection_clear(0, tk.END)
            self.route_list.selection_set(tk.END)
            self.route_list.see(tk.END)
            self.route_status_var.set(f"已规划 {len(self.route.waypoints)} 个航点")
            self._draw()
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("航线规划", str(exc))

    def _delete_waypoint(self) -> None:
        selection = self.route_list.curselection()
        if not selection:
            messagebox.showinfo("航线规划", "请先选择要删除的航点")
            return
        try:
            self.route.delete(selection[0])
            self._refresh_route_list()
            self.route_status_var.set(f"已规划 {len(self.route.waypoints)} 个航点")
            self._draw()
        except ValueError as exc:
            messagebox.showerror("航线规划", str(exc))

    def _clear_route(self) -> None:
        self.route.clear()
        self._refresh_route_list()
        self.route_status_var.set("航线已清空")
        self._draw()

    def _start_route(self) -> None:
        try:
            waypoint = self.route.start()
            self._activate_waypoint(waypoint)
            self.route_status_var.set(f"航线运行中：目标 1/{len(self.route.waypoints)}")
            self._refresh_route_list()
        except ValueError as exc:
            messagebox.showerror("航线规划", str(exc))

    def _stop_route(self) -> None:
        self.route.stop()
        self.route_status_var.set("航线已停止")
        self._refresh_route_list()

    def _activate_waypoint(self, waypoint: Waypoint) -> None:
        self.vars["target_east"].set(waypoint.east_m)
        self.vars["target_north"].set(waypoint.north_m)
        self._send_target()
        self._draw()

    def _refresh_route_list(self) -> None:
        self.route_list.delete(0, tk.END)
        for index, waypoint in enumerate(self.route.waypoints):
            marker = ">" if self.route.active_index == index else " "
            latitude, longitude = local_to_gps(
                waypoint.east_m, waypoint.north_m, self._value("lat0"), self._value("lon0")
            )
            self.route_list.insert(tk.END, f"{marker}{index + 1:02d} {latitude:.7f} {longitude:.7f}")

    def _update_route_progress(self, east_m: float, north_m: float) -> None:
        event = self.route.observe(east_m, north_m, self._value("arrival_radius"))
        if event == "advanced":
            assert self.route.current is not None and self.route.active_index is not None
            self._activate_waypoint(self.route.current)
            self.route_status_var.set(
                f"已到达前一点，目标 {self.route.active_index + 1}/{len(self.route.waypoints)} 已自动下发"
            )
            self._refresh_route_list()
        elif event == "completed":
            self.route_status_var.set(f"航线完成，共 {len(self.route.waypoints)} 个航点")
            self._refresh_route_list()

    def _raw_motor_values(self) -> tuple[int, int, str]:
        if self.manual_mode.get():
            return int(self.vars["manual_left"].get()), int(self.vars["manual_right"].get()), "手动"
        command = self.link.motor_command()
        if command is None:
            return 0, 0, "串口超时，电机归零"
        return command.left, command.right, "MCU"

    def _motor_values(self) -> tuple[int, int, str]:
        left, right, source = self._raw_motor_values()
        left, right = map_motor_commands(
            left,
            right,
            swap=self.swap_motors.get(),
            invert_left=self.invert_left_motor.get(),
            invert_right=self.invert_right_motor.get(),
        )
        return left, right, source

    def _capture_ekf_point(self) -> None:
        position = self.link.ekf_position()
        if position is None:
            return
        frame_key = (position.sequence, position.mcu_time_ms)
        if frame_key == self.last_ekf_frame:
            return
        self.last_ekf_frame = frame_key
        latitude = position.ekf_latitude_deg
        longitude = position.ekf_longitude_deg
        if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
            return
        self.latest_ekf_gps = (latitude, longitude)
        point = gps_to_local(latitude, longitude, self._value("lat0"), self._value("lon0"))
        self._update_route_progress(*point)
        if not self.ekf_path or math.dist(self.ekf_path[-1], point) >= 0.03:
            self.ekf_path.append(point)
            if len(self.ekf_path) > 20_000:
                del self.ekf_path[:4_000]

    def _simulate_step(self) -> None:
        left, right, _ = self._motor_values()
        self.boat.step(left, right, SIM_DT)
        self.sim_time += SIM_DT
        state = self.boat.state
        if not self.true_path or math.dist(self.true_path[-1], (state.east_m, state.north_m)) >= 0.05:
            self.true_path.append((state.east_m, state.north_m))
            if len(self.true_path) > 50_000:
                del self.true_path[:10_000]
        gps_new = self.sim_time - self.last_gps_time >= GPS_PERIOD_S - 1e-9
        if gps_new:
            self.last_gps_time = self.sim_time
            self.latest_gps = self.gps.sample(state, self._value("lat0"), self._value("lon0"), self._value("gps_accuracy"))
            self.gps_path.append(gps_to_local(*self.latest_gps, self._value("lat0"), self._value("lon0")))
        if self.sim_time - self.last_sensor_send >= SIM_DT - 1e-9:
            self.last_sensor_send = self.sim_time
            frame = encode_sensor(
                self.sequence, int(self.sim_time * 1000), gps_new,
                map_yaw_angle(state.yaw_rad, invert=self.invert_yaw.get()),
                state.yaw_rate_rps,
                self.latest_gps[0], self.latest_gps[1], self._value("gps_accuracy"),
            )
            self.sequence += 1
            self.link.send(frame)
        distance = math.hypot(state.east_m - self._value("target_east"), state.north_m - self._value("target_north"))
        if distance <= self._value("arrival_radius"):
            self.arrival_since = self.arrival_since if self.arrival_since is not None else self.sim_time
            self.arrived = self.sim_time - self.arrival_since >= 2.0
        else:
            self.arrival_since = None
            self.arrived = False

    def _tick(self) -> None:
        now = time.perf_counter()
        elapsed = min(0.1, now - self.last_tick)
        self.last_tick = now
        self._capture_ekf_point()
        if self.running:
            self.accumulator += elapsed
            while self.accumulator >= SIM_DT:
                self._simulate_step()
                self.accumulator -= SIM_DT
        if now - self.last_draw >= 0.1:
            self.last_draw = now
            self._draw()
        self.root.after(20, self._tick)

    @staticmethod
    def _display_points(points: list[tuple[float, float]], limit: int = 3000) -> list[tuple[float, float]]:
        if len(points) <= limit:
            return points
        step = math.ceil(len(points) / limit)
        sampled = points[::step]
        if sampled[-1] != points[-1]:
            sampled.append(points[-1])
        return sampled

    def _draw(self) -> None:
        canvas = self.canvas
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        canvas.delete("all")
        target = (self._value("target_east"), self._value("target_north"))
        route_points = [(point.east_m, point.north_m) for point in self.route.waypoints]
        points = self.true_path + self.ekf_path + route_points + [(0.0, 0.0), target]
        required_extent = max(10.0, *(abs(v) for point in points for v in point)) * 1.2
        self.view_extent = max(self.view_extent, required_extent)
        scale = min(width, height) / (2.0 * self.view_extent)
        self.view_scale = scale

        def xy(east: float, north: float) -> tuple[float, float]:
            return width / 2 + east * scale, height / 2 - north * scale

        grid_step = 5 if self.view_extent <= 40 else 10 if self.view_extent <= 100 else 50
        limit = int(self.view_extent // grid_step) * grid_step
        for value in range(-limit, limit + 1, grid_step):
            x, _ = xy(value, 0)
            _, y = xy(0, value)
            canvas.create_line(x, 0, x, height, fill="#e5e7eb")
            canvas.create_line(0, y, width, y, fill="#e5e7eb")
        if len(self.true_path) >= 2:
            display_path = self._display_points(self.true_path)
            canvas.create_line(*(coord for point in display_path for coord in xy(*point)), fill="#2563eb", width=2)
        for point in self.gps_path[-100:]:
            gx, gy = xy(*point)
            canvas.create_oval(gx - 3, gy - 3, gx + 3, gy + 3, fill="#f59e0b", outline="")
        if len(self.ekf_path) >= 2:
            display_ekf = self._display_points(self.ekf_path)
            canvas.create_line(*(coord for point in display_ekf for coord in xy(*point)), fill="#16a34a", width=2)
        for point in self.ekf_path[-200:]:
            ex, ey = xy(*point)
            canvas.create_oval(ex - 2, ey - 2, ex + 2, ey + 2, fill="#16a34a", outline="")
        if len(route_points) >= 2:
            canvas.create_line(*(coord for point in route_points for coord in xy(*point)), fill="#475569", width=2, dash=(5, 3))
        for index, point in enumerate(route_points):
            wx, wy = xy(*point)
            active = self.route.active_index == index
            color = "#dc2626" if active else "#334155"
            radius = 7 if active else 5
            canvas.create_oval(wx - radius, wy - radius, wx + radius, wy + radius, fill=color, outline="")
            canvas.create_text(wx + 9, wy - 9, text=str(index + 1), fill=color, anchor="sw")
        tx, ty = xy(*target)
        radius_px = max(3, self._value("arrival_radius") * scale)
        canvas.create_oval(tx - radius_px, ty - radius_px, tx + radius_px, ty + radius_px, outline="#16a34a", width=2)
        canvas.create_line(tx - 7, ty, tx + 7, ty, fill="#16a34a", width=2)
        canvas.create_line(tx, ty - 7, tx, ty + 7, fill="#16a34a", width=2)
        state = self.boat.state
        bx, by = xy(state.east_m, state.north_m)
        bow_x = bx + math.sin(state.yaw_rad) * 18
        bow_y = by - math.cos(state.yaw_rad) * 18
        left_x = bx + math.sin(state.yaw_rad + 2.45) * 11
        left_y = by - math.cos(state.yaw_rad + 2.45) * 11
        right_x = bx + math.sin(state.yaw_rad - 2.45) * 11
        right_y = by - math.cos(state.yaw_rad - 2.45) * 11
        canvas.create_polygon(bow_x, bow_y, left_x, left_y, right_x, right_y, fill="#dc2626", outline="#7f1d1d")
        distance = math.hypot(state.east_m - target[0], state.north_m - target[1])
        raw_left, raw_right, source = self._raw_motor_values()
        left, right, _ = self._motor_values()
        link_state = "串口已连接" if self.link.connected else "串口未连接"
        run_state = "运行" if self.running else "停止"
        arrived = " / 已到达" if self.arrived else ""
        ekf_text = (
            f"{self.latest_ekf_gps[0]:.7f}, {self.latest_ekf_gps[1]:.7f}"
            if self.latest_ekf_gps is not None else "--"
        )
        self.status_var.set(f"{link_state} / {run_state}{arrived}")
        self.telemetry_var.set(
            f"输入：{source}\n原始左/右：{raw_left} / {raw_right}\n仿真左/右：{left} / {right}\n"
            f"东/北：{state.east_m:.2f} / {state.north_m:.2f} m\n"
            f"航向：{state.yaw_rad:.3f} rad\n速度：{state.speed_mps:.2f} m/s\n"
            f"角速度：{state.yaw_rate_rps:.2f} rad/s\n目标距离：{distance:.2f} m\n"
            f"GPS：{self.latest_gps[0]:.7f}, {self.latest_gps[1]:.7f}\n"
            f"EKF：{ekf_text}\n"
            f"电机帧：{self.link.stats.received_motor_frames}\n"
            f"EKF 帧：{self.link.stats.received_ekf_frames}"
        )

    def _close(self) -> None:
        self.link.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    BoatSimulatorApp(root)
    root.mainloop()
