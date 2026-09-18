from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

from boat_sim.model import BoatParameters, DifferentialBoat, GpsSensor, gps_to_local, local_to_gps
from boat_sim.protocol import encode_sensor, encode_target
from boat_sim.serial_link import SerialLink


SIM_DT = 0.01
GPS_PERIOD_S = 1.0


class BoatSimulatorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("差速船闭环仿真器")
        self.root.geometry("1200x760")
        self.boat = DifferentialBoat()
        self.gps = GpsSensor()
        self.link = SerialLink()
        self.running = False
        self.sim_time = 0.0
        self.accumulator = 0.0
        self.last_tick = time.perf_counter()
        self.last_gps_time = -GPS_PERIOD_S
        self.last_sensor_send = -SIM_DT
        self.sequence = 0
        self.latest_gps = (30.000000, 120.000000)
        self.true_path: list[tuple[float, float]] = []
        self.gps_path: list[tuple[float, float]] = []
        self.arrival_since: float | None = None
        self.arrived = False
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
        controls = ttk.Frame(body, padding=8)
        plot_frame = ttk.Frame(body, padding=(0, 8, 8, 8))
        body.add(controls, weight=0)
        body.add(plot_frame, weight=1)

        params = ttk.LabelFrame(controls, text="船体参数", padding=8)
        params.pack(fill="x")
        self._entry(params, "质量 (kg)", "mass", 20.0)
        self._entry(params, "电机间距 (m)", "spacing", 0.60)
        self._entry(params, "最大速度 (m/s)", "max_speed", 3.0)
        self._entry(params, "最大转向速度 (rad/s)", "max_yaw", 1.2)
        self._entry(params, "单电机最大推力 (N)", "max_thrust", 35.0)
        self._entry(params, "电机响应时间 (s)", "motor_tau", 0.25)
        ttk.Button(params, text="应用参数", command=self._apply_parameters).pack(fill="x", pady=(6, 0))

        location = ttk.LabelFrame(controls, text="坐标与目标", padding=8)
        location.pack(fill="x", pady=(8, 0))
        self._entry(location, "原点纬度", "lat0", 30.0)
        self._entry(location, "原点经度", "lon0", 120.0)
        self._entry(location, "目标东向 (m)", "target_east", 20.0)
        self._entry(location, "目标北向 (m)", "target_north", 30.0)
        self._entry(location, "到达半径 (m)", "arrival_radius", 2.0)
        self._entry(location, "GPS 精度半径 (m)", "gps_accuracy", 1.5)
        ttk.Button(location, text="下发目标", command=self._send_target).pack(fill="x", pady=(6, 0))

        motors = ttk.LabelFrame(controls, text="手动电机 (-10000..10000)", padding=8)
        motors.pack(fill="x", pady=(8, 0))
        self._scale(motors, "左", "manual_left")
        self._scale(motors, "右", "manual_right")

        self.telemetry_var = tk.StringVar()
        ttk.Label(controls, textvariable=self.telemetry_var, justify="left").pack(fill="x", pady=(10, 0))

        self.canvas = tk.Canvas(plot_frame, background="#f7fafc", highlightthickness=1, highlightbackground="#9ca3af")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw())

    def _entry(self, parent: ttk.Widget, label: str, key: str, default: float) -> None:
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
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
        self.run_button.configure(text="开始")
        self.boat.reset()
        self.sim_time = 0.0
        self.accumulator = 0.0
        self.last_gps_time = -GPS_PERIOD_S
        self.last_sensor_send = -SIM_DT
        self.true_path.clear()
        self.gps_path.clear()
        self.arrival_since = None
        self.arrived = False
        self.vars["manual_left"].set(0)
        self.vars["manual_right"].set(0)
        self._draw()

    def _send_target(self) -> None:
        try:
            latitude, longitude = local_to_gps(
                self._value("target_east"), self._value("target_north"), self._value("lat0"), self._value("lon0")
            )
            frame = encode_target(self.sequence, int(self.sim_time * 1000), latitude, longitude, self._value("arrival_radius"))
            self.sequence += 1
            self.link.send(frame)
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("目标参数错误", str(exc))

    def _motor_values(self) -> tuple[int, int, str]:
        if self.manual_mode.get():
            return int(self.vars["manual_left"].get()), int(self.vars["manual_right"].get()), "手动"
        command = self.link.motor_command()
        if command is None:
            return 0, 0, "串口超时，电机归零"
        return max(-10000, min(10000, command.left)), max(-10000, min(10000, command.right)), "MCU"

    def _simulate_step(self) -> None:
        left, right, _ = self._motor_values()
        self.boat.step(left, right, SIM_DT)
        self.sim_time += SIM_DT
        state = self.boat.state
        if not self.true_path or math.dist(self.true_path[-1], (state.east_m, state.north_m)) >= 0.05:
            self.true_path.append((state.east_m, state.north_m))
        gps_new = self.sim_time - self.last_gps_time >= GPS_PERIOD_S - 1e-9
        if gps_new:
            self.last_gps_time = self.sim_time
            self.latest_gps = self.gps.sample(state, self._value("lat0"), self._value("lon0"), self._value("gps_accuracy"))
            self.gps_path.append(gps_to_local(*self.latest_gps, self._value("lat0"), self._value("lon0")))
        if self.sim_time - self.last_sensor_send >= SIM_DT - 1e-9:
            self.last_sensor_send = self.sim_time
            frame = encode_sensor(
                self.sequence, int(self.sim_time * 1000), gps_new, state.yaw_rad,
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
        if self.running:
            self.accumulator += elapsed
            while self.accumulator >= SIM_DT:
                self._simulate_step()
                self.accumulator -= SIM_DT
        self._draw()
        self.root.after(20, self._tick)

    def _draw(self) -> None:
        canvas = self.canvas
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        canvas.delete("all")
        target = (self._value("target_east"), self._value("target_north"))
        points = self.true_path + [(0.0, 0.0), target]
        max_extent = max(10.0, *(abs(v) for point in points for v in point)) * 1.2
        scale = min(width, height) / (2.0 * max_extent)

        def xy(east: float, north: float) -> tuple[float, float]:
            return width / 2 + east * scale, height / 2 - north * scale

        grid_step = 5 if max_extent <= 40 else 10 if max_extent <= 100 else 50
        limit = int(max_extent // grid_step) * grid_step
        for value in range(-limit, limit + 1, grid_step):
            x, _ = xy(value, 0)
            _, y = xy(0, value)
            canvas.create_line(x, 0, x, height, fill="#e5e7eb")
            canvas.create_line(0, y, width, y, fill="#e5e7eb")
        if len(self.true_path) >= 2:
            canvas.create_line(*(coord for point in self.true_path for coord in xy(*point)), fill="#2563eb", width=2)
        for point in self.gps_path[-100:]:
            gx, gy = xy(*point)
            canvas.create_oval(gx - 3, gy - 3, gx + 3, gy + 3, fill="#f59e0b", outline="")
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
        left, right, source = self._motor_values()
        link_state = "串口已连接" if self.link.connected else "串口未连接"
        run_state = "运行" if self.running else "停止"
        arrived = " / 已到达" if self.arrived else ""
        self.status_var.set(f"{link_state} / {run_state}{arrived}")
        self.telemetry_var.set(
            f"输入：{source}\n左/右：{left} / {right}\n"
            f"东/北：{state.east_m:.2f} / {state.north_m:.2f} m\n"
            f"航向：{state.yaw_rad:.3f} rad\n速度：{state.speed_mps:.2f} m/s\n"
            f"角速度：{state.yaw_rate_rps:.2f} rad/s\n目标距离：{distance:.2f} m\n"
            f"GPS：{self.latest_gps[0]:.7f}, {self.latest_gps[1]:.7f}\n"
            f"MCU 帧：{self.link.stats.received_frames}"
        )

    def _close(self) -> None:
        self.link.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    BoatSimulatorApp(root)
    root.mainloop()
