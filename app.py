from __future__ import annotations

import math
import random
import time
import tkinter as tk
from tkinter import messagebox, ttk

from boat_sim.model import BoatParameters, DifferentialBoat, GpsSensor, gps_to_local, local_to_gps
from boat_sim.collision import CircleObstacle, PolygonObstacle, resolve_motion
from boat_sim.protocol import (
    NAV_CMD_PLAN_EXECUTE,
    NAV_CMD_PLAN_RESEND,
    NAV_CMD_STOP,
    NAV_CMD_SURVEY_PLAN,
    encode_nav_config,
    encode_nav_frame,
    encode_nav_task,
    encode_range,
    encode_sensor,
    encode_target,
)
from boat_sim.route import MAX_WAYPOINTS, RoutePlan, Waypoint
from boat_sim.serial_link import SerialLink
from boat_sim.signal_mapping import map_motor_commands, map_yaw_angle
from boat_sim.sonar import SonarConfig, synthesize_sonar, transducer_beams


SIM_DT = 0.01
GPS_DEFAULT_HZ = 5.0
MAP_HALF_EXTENT_M = 40.0


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
        self.route_window: tk.Toplevel | None = None
        # 测距: 世界坐标折点 + 闭合标记 + 橡皮筋光标; 纯地图标注, 不进串口协议
        self.measure_points: list[tuple[float, float]] = []
        self.measure_closed = False
        self.measure_cursor: tuple[float, float] | None = None
        self.measure_status_var = tk.StringVar(value="测距: 左键依次点选, 按钮清除")
        self.obstacles: list[CircleObstacle | PolygonObstacle] = []
        self.shoreline_draft: list[tuple[float, float]] = []
        self.collision_mode_var = tk.StringVar(value="沿边滑行")
        self.obstacle_mode_var = tk.StringVar(value="目标")
        self.obstacle_radius_var = tk.DoubleVar(value=3.0)
        self.default_shore: PolygonObstacle | None = None  # "生成默认岸边"的成果; 重复生成时替换
        self.collision_status = "无碰撞"
        self.running = False
        self.sim_time = 0.0
        self.accumulator = 0.0
        self.last_tick = time.perf_counter()
        self.last_draw = self.last_tick
        self.last_gps_time = -self._gps_period_s()
        self.last_sensor_send = -SIM_DT
        self.sequence = 0
        # --- 导航扩展(0x10~0x13 下发 / 0x83 0x84 接收)的会话与采集状态 ---
        self.nav_session = 0        # 连接时生成的非零会话号(MCU FRAME 帧校验)
        self.nav_frame_id = 1
        self.nav_request = 0        # TASK 请求号, 单调递增(MCU 去重)
        self.last_range_send = -0.1
        self.nav_status = None      # 最近一帧 0x83
        self.plan_points: list[tuple[float, float] | None] = []  # (north, east) 按下标
        self.plan_have: set[int] = set()
        self.plan_total = 0
        self.plan_id = 0
        self.plan_final_seen = False
        self.plan_complete = False
        self.plan_last_resend_s = 0.0
        self.plan_last_segment_s = 0.0
        # --- 声呐可视化: 最近一次波束(画线) + 回波点云(轮廓) ---
        self.last_sonar_beams: list[tuple[tuple[float, float], tuple[float, float], bool]] | None = None
        self.last_sonar_text: tuple[float, float, int] | None = None   # (left_m, right_m, hit)
        self.sonar_points: list[tuple[float, float]] = []
        self.latest_gps = (30.000000, 120.000000)
        self.true_path: list[tuple[float, float]] = []
        self.gps_path: list[tuple[float, float]] = []
        self.ekf_path: list[tuple[float, float]] = []
        self.latest_ekf_gps: tuple[float, float] | None = None
        self.last_ekf_frame: tuple[int, int] | None = None
        self.arrival_since: float | None = None
        self.arrived = False
        self.view_extent = MAP_HALF_EXTENT_M
        # 镜头状态: view_center 为镜头中心的世界坐标(东/北 m), 默认看向原点;
        # follow_boat 开启时每帧把镜头中心对准船体, 拖拽平移会自动退出跟随。
        self.view_center = [0.0, 0.0]
        self.follow_boat = tk.BooleanVar(value=False)
        self._pan_drag: tuple[float, float, float, float] | None = None
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
        self.view_var = tk.StringVar(value="视野 40m")
        ttk.Label(toolbar, textvariable=self.view_var).pack(side="left", padx=(12, 0))
        ttk.Button(toolbar, text="视野复位", command=self._reset_view).pack(side="left", padx=(4, 0))
        ttk.Checkbutton(toolbar, text="镜头跟随船体", variable=self.follow_boat).pack(side="left", padx=(8, 0))
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
        survey_tab = ttk.Frame(control_tabs, padding=4)
        control_tabs.add(operation_tab, text="目标与电机")
        control_tabs.add(setup_tab, text="参数与方向")
        control_tabs.add(survey_tab, text="扫测任务")

        params = ttk.LabelFrame(setup_tab, text="船体参数", padding=6)
        params.pack(fill="x")
        self._entry(params, "质量 (kg)", "mass", 2.0)
        self._entry(params, "电机间距 (m)", "spacing", 0.20)
        self._entry(params, "最大速度 (m/s)", "max_speed", 1.0)
        self._entry(params, "最大转向速度 (rad/s)", "max_yaw", 1.5)
        self._entry(params, "单电机最大推力 (N)", "max_thrust", 4.0)
        self._entry(params, "电机响应时间 (s)", "motor_tau", 0.25)
        self._entry(params, "船体碰撞半径 (m)", "collision_radius", 0.15)
        ttk.Button(params, text="应用参数", command=self._apply_parameters).pack(fill="x", pady=(4, 0))

        location = ttk.LabelFrame(operation_tab, text="坐标与目标", padding=6)
        location.pack(fill="x")
        self._entry(location, "原点纬度", "lat0", 30.0)
        self._entry(location, "原点经度", "lon0", 120.0)
        self._entry(location, "目标东向 (m)", "target_east", 20.0)
        self._entry(location, "目标北向 (m)", "target_north", 30.0)
        self._entry(location, "到达半径 (m)", "arrival_radius", 2.0)
        self._entry(location, "GPS 精度半径 (m)", "gps_accuracy", 1.5)
        gps_row = ttk.Frame(location)
        gps_row.pack(fill="x", pady=1)
        ttk.Label(gps_row, text="GPS 更新频率 (Hz)", width=23).pack(side="left")
        self.gps_rate_var = tk.StringVar(value=str(int(GPS_DEFAULT_HZ)))
        ttk.Combobox(
            gps_row,
            textvariable=self.gps_rate_var,
            values=("1", "5", "10"),
            state="readonly",
            width=9,
        ).pack(side="right")
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
        ttk.Button(motors, text="航线规划…(独立窗口)", command=self._open_route_window).pack(fill="x", pady=(6, 0))

        obstacles = ttk.LabelFrame(operation_tab, text="障碍物与岸线", padding=6)
        obstacles.pack(fill="x", pady=(6, 0))
        obstacle_mode_row = ttk.Frame(obstacles)
        obstacle_mode_row.pack(fill="x", pady=1)
        ttk.Label(obstacle_mode_row, text="地图点击模式", width=15).pack(side="left")
        self.obstacle_mode_buttons: dict[str, tk.Button] = {}
        for mode in ("目标", "石岛", "岸线", "测距"):
            button = tk.Button(obstacle_mode_row, text=mode, relief="raised", bd=2,
                               command=lambda m=mode: self._set_obstacle_mode(m))
            button.pack(side="left", fill="x", expand=True, padx=(0, 4))
            self.obstacle_mode_buttons[mode] = button
        self._refresh_obstacle_mode_buttons()
        ttk.Label(obstacles, textvariable=self.measure_status_var, wraplength=220,
                  justify="left").pack(fill="x", pady=(3, 0))
        ttk.Button(obstacles, text="清除测距 (Esc)", command=self._clear_measure).pack(fill="x", pady=(3, 0))
        obstacle_radius_row = ttk.Frame(obstacles)
        obstacle_radius_row.pack(fill="x", pady=1)
        ttk.Label(obstacle_radius_row, text="石岛半径 (m)", width=15).pack(side="left")
        ttk.Entry(obstacle_radius_row, textvariable=self.obstacle_radius_var, width=11).pack(side="right")
        ttk.Button(obstacles, text="生成默认岸边", command=self._generate_default_shore).pack(fill="x", pady=(3, 0))
        ttk.Button(obstacles, text="完成岸线", command=self._finish_shoreline).pack(fill="x", pady=(3, 0))
        ttk.Button(obstacles, text="删除最后障碍物", command=self._delete_last_obstacle).pack(fill="x", pady=(3, 0))
        ttk.Button(obstacles, text="清空障碍物", command=self._clear_obstacles).pack(fill="x", pady=(3, 0))
        collision_row = ttk.Frame(obstacles)
        collision_row.pack(fill="x", pady=(3, 0))
        ttk.Label(collision_row, text="碰撞响应", width=15).pack(side="left")
        ttk.Combobox(collision_row, textvariable=self.collision_mode_var,
                     values=("停止", "沿边滑行"), state="readonly", width=9).pack(side="right")

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

        sonar = ttk.LabelFrame(setup_tab, text="声呐仿真 (RANGE 10Hz 自动下发)", padding=6)
        sonar.pack(fill="x", pady=(6, 0))
        self._entry(sonar, "量程 (m)", "sonar_range", 5.0)
        self._entry(sonar, "波束全宽 (°)", "sonar_beam_deg", 60.0)
        self._entry(sonar, "双轴夹角 (°)", "sonar_axis_deg", 40.0)
        self._entry(sonar, "前置距离 (m)", "sonar_forward", 0.15)
        self._entry(sonar, "横向偏置 (m)", "sonar_lateral", 0.005)
        ttk.Button(sonar, text="下发 CONFIG", command=self._send_nav_config).pack(fill="x", pady=(4, 0))

        survey = ttk.LabelFrame(survey_tab, text="同心圆扫测 (两阶段: 规划 → 人审 → 执行)", padding=6)
        survey.pack(fill="x")
        self._entry(survey, "圈距 (m)", "survey_step", 3.0)
        self._entry(survey, "点距 (m)", "survey_spacing", 6.0)
        self._entry(survey, "离岸距离 (m)", "survey_launch", 3.0)
        self._entry(survey, "外圈半径 (m)", "survey_radius", 30.0)
        self._entry(survey, "到点容差 (m)", "survey_tol", 1.5)
        survey_actions = ttk.Frame(survey)
        survey_actions.pack(fill="x", pady=(4, 0))
        ttk.Button(survey_actions, text="开始规划", command=self._send_survey_plan).pack(
            side="left", fill="x", expand=True)
        self.execute_button = ttk.Button(survey_actions, text="确认执行",
                                         command=self._confirm_plan_execute, state="disabled")
        self.execute_button.pack(side="left", fill="x", expand=True, padx=(6, 0))
        survey_actions2 = ttk.Frame(survey)
        survey_actions2.pack(fill="x", pady=(4, 0))
        ttk.Button(survey_actions2, text="重传点表", command=self._request_plan_resend).pack(
            side="left", fill="x", expand=True)
        ttk.Button(survey_actions2, text="停止", command=self._send_nav_stop).pack(
            side="left", fill="x", expand=True, padx=(6, 0))
        ttk.Button(survey_tab, text="重建会话 (MCU 复位后先点这个)",
                   command=self._establish_nav_session).pack(fill="x", pady=(6, 0))
        self.survey_status_var = tk.StringVar(value="未规划")
        ttk.Label(survey_tab, textvariable=self.survey_status_var,
                  wraplength=220, justify="left").pack(fill="x", pady=(6, 0))

        self._build_route_window()

        # 遥测栏: 放在地图下方, 横向两行排布(先前在左侧窄栏里, 内容多被整段藏住)
        telemetry_bar = ttk.Frame(plot_frame, padding=(6, 3))
        telemetry_bar.pack(side="bottom", fill="x")
        self.telemetry_row1_var = tk.StringVar()
        self.telemetry_row2_var = tk.StringVar()
        ttk.Label(telemetry_bar, textvariable=self.telemetry_row1_var, anchor="w",
                  font=("TkDefaultFont", 9)).pack(fill="x")
        ttk.Label(telemetry_bar, textvariable=self.telemetry_row2_var, anchor="w",
                  font=("TkDefaultFont", 9)).pack(fill="x")

        self.canvas = tk.Canvas(plot_frame, background="#f7fafc", highlightthickness=1, highlightbackground="#9ca3af")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw())
        self.canvas.bind("<Button-1>", self._pick_target)
        self.canvas.bind("<MouseWheel>", self._on_map_wheel)
        self.canvas.bind("<Double-Button-1>", lambda _event: self._reset_view())
        # 镜头平移: 中键或右键拖拽(左键保留给拾取目标/障碍物)
        for sequence, handler in (
            ("<ButtonPress-2>", self._on_pan_start),
            ("<B2-Motion>", self._on_pan_move),
            ("<ButtonRelease-2>", self._on_pan_end),
            ("<ButtonPress-3>", self._on_pan_start),
            ("<B3-Motion>", self._on_pan_move),
            ("<ButtonRelease-3>", self._on_pan_end),
        ):
            self.canvas.bind(sequence, handler)
        self.canvas.bind("<Motion>", self._on_map_motion)
        self.canvas.bind("<Leave>", self._on_map_leave)
        self.canvas.bind("<Escape>", self._on_map_escape)

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

    MAP_MIN_EXTENT_M = 5.0
    MAP_MAX_EXTENT_M = MAP_HALF_EXTENT_M

    def _reset_view(self) -> None:
        self.view_extent = MAP_HALF_EXTENT_M
        self.view_center[0] = 0.0
        self.view_center[1] = 0.0
        self.follow_boat.set(False)
        self.view_var.set("视野 40m")
        # 不直接 _draw: 让 10Hz 的常规重绘接手, 避免连续操作触发重绘风暴饿死仿真循环

    def _on_map_wheel(self, event: tk.Event) -> None:
        """滚轮缩放视野(半宽 m): 上放大下缩小, 夹在 [5, 40]; 双击/按钮复位。
        只改数值不立即重绘 —— 10Hz 常规重绘会跟上(滚轮连滚会触发重绘风暴,
        曾把仿真循环饿出 600ms+ 声呐断流, MCU 侧 WD2 灭任务)。"""
        if not event.delta:
            return
        factor = 1.15 ** (event.delta / 120.0)
        self.view_extent = max(self.MAP_MIN_EXTENT_M, min(self.MAP_MAX_EXTENT_M, self.view_extent / factor))
        self.view_var.set(f"视野 {self.view_extent:.0f}m")

    def _on_pan_start(self, event: tk.Event) -> None:
        """中键/右键按下: 记录拖拽起点(屏幕 x/y + 当时镜头中心), 并退出跟随模式。"""
        if self.follow_boat.get():
            self.follow_boat.set(False)
        self._pan_drag = (event.x, event.y, self.view_center[0], self.view_center[1])
        self.canvas.configure(cursor="fleur")

    def _on_pan_move(self, event: tk.Event) -> None:
        drag = self._pan_drag
        if drag is None:
            return
        scale = max(1e-6, getattr(self, "view_scale", 1.0))
        # 屏幕右移 = 镜头中心西移(世界东向减小); 屏幕下移 = 镜头中心南移
        self.view_center[0] = drag[2] - (event.x - drag[0]) / scale
        self.view_center[1] = drag[3] + (event.y - drag[1]) / scale

    def _on_pan_end(self, _event: tk.Event) -> None:
        self._pan_drag = None
        self.canvas.configure(cursor="")

    def _gps_period_s(self) -> float:
        try:
            rate_hz = float(self.gps_rate_var.get())
        except (AttributeError, tk.TclError, ValueError):
            rate_hz = GPS_DEFAULT_HZ
        return 1.0 / max(0.1, rate_hz)

    def _apply_parameters(self) -> None:
        try:
            values = [self._value(key) for key in ("mass", "spacing", "max_speed", "max_yaw", "max_thrust", "motor_tau")]
            if any(value <= 0 for value in values):
                raise ValueError("参数必须全部大于 0")
            if self._value("collision_radius") <= 0:
                raise ValueError("船体碰撞半径必须大于 0")
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
            self._establish_nav_session()
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
        self.last_gps_time = -self._gps_period_s()
        self.last_sensor_send = -SIM_DT
        self.last_range_send = -0.1  # 漏了这句: 复位后 sim_time 归零而它保留旧值, RANGE 门控永假(PC 实测一帧不发)
        self.true_path.clear()
        self.gps_path.clear()
        self.ekf_path.clear()
        self.latest_ekf_gps = None
        self.last_ekf_frame = None
        self.arrival_since = None
        self.arrived = False
        self._reset_plan_capture()
        self.last_sonar_beams = None
        self.last_sonar_text = None
        self.sonar_points = []
        self.view_extent = MAP_HALF_EXTENT_M
        self.view_center[0] = 0.0
        self.view_center[1] = 0.0
        if hasattr(self, "view_var"):
            self.view_var.set("视野 40m")
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

    # 地图点击模式按钮配色: 激活=绿, 未激活=灰
    OBSTACLE_MODE_ACTIVE_BG = "#16a34a"
    OBSTACLE_MODE_INACTIVE_BG = "#d1d5db"

    def _set_obstacle_mode(self, mode: str) -> None:
        self.obstacle_mode_var.set(mode)
        if mode != "测距":
            self.measure_cursor = None  # 离开测距模式, 收起橡皮筋
        self._refresh_obstacle_mode_buttons()

    def _refresh_obstacle_mode_buttons(self) -> None:
        active = self.obstacle_mode_var.get()
        for mode, button in self.obstacle_mode_buttons.items():
            if mode == active:
                button.configure(bg=self.OBSTACLE_MODE_ACTIVE_BG, fg="#ffffff", relief="sunken",
                                 activebackground="#15803d", activeforeground="#ffffff")
            else:
                button.configure(bg=self.OBSTACLE_MODE_INACTIVE_BG, fg="#374151", relief="raised",
                                 activebackground="#9ca3af", activeforeground="#111827")

    def _pick_target(self, event: tk.Event) -> None:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        scale = getattr(self, "view_scale", 1.0)
        east = self.view_center[0] + (event.x - width / 2) / scale
        north = self.view_center[1] + (height / 2 - event.y) / scale
        if self.obstacle_mode_var.get() == "石岛":
            try:
                radius = float(self.obstacle_radius_var.get())
                if radius <= 0:
                    raise ValueError("石岛半径必须大于 0")
                self.obstacles.append(CircleObstacle(east, north, radius))
                self.collision_status = f"已添加石岛 ({east:.1f}, {north:.1f})"
                self._draw()
            except (tk.TclError, ValueError) as exc:
                messagebox.showerror("障碍物", str(exc))
            return
        if self.obstacle_mode_var.get() == "岸线":
            self.shoreline_draft.append((east, north))
            self.collision_status = f"岸线点 {len(self.shoreline_draft)}，继续点击或完成岸线"
            self._draw()
            return
        if self.obstacle_mode_var.get() == "测距":
            self._measure_click(east, north, event, width, height, scale)
            return
        if self.route.running:
            self.route_status_var.set("请先停止航线，再拾取或编辑航点")
            return
        self.vars["target_east"].set(round(east, 3))
        self.vars["target_north"].set(round(north, 3))
        self._update_picked_coordinate()
        self._draw()

    def _generate_default_shore(self) -> None:
        """一键生成默认岸边: 位于当前视图最下半部分、横向伸出视野的微波浪岸线;
        船放到岸线法线方向 0.5~1m 处(北/水面侧), 船头朝北。重复点击替换上一条。"""
        extent = self.view_extent
        center_east, center_north = self.view_center
        span = extent * 1.3                       # 岸线横向伸出视野, 看不到尽头
        depth = max(6.0, extent * 0.35)           # 陆地厚度, 伸到视野下方之外
        base_north = center_north - extent * random.uniform(0.5, 0.65)
        amplitude = random.uniform(0.5, 1.0)
        wavelength = extent * random.uniform(1.2, 2.0)
        phase = random.uniform(0.0, 2.0 * math.pi)
        count = 12

        def shore_north_at(east: float) -> float:
            return base_north + amplitude * math.sin(2.0 * math.pi * east / wavelength + phase)

        top = [(-span + 2.0 * span * index / count,
                shore_north_at(-span + 2.0 * span * index / count)) for index in range(count + 1)]
        shore = PolygonObstacle(tuple(top + [(span, base_north - depth), (-span, base_north - depth)]))
        if self.default_shore is not None and self.default_shore in self.obstacles:
            self.obstacles.remove(self.default_shore)
        self.default_shore = shore
        self.obstacles.append(shore)

        # 船位: 按真实折线几何(而非解析曲线)放, 保证到岸边的距离精确
        gap = random.uniform(0.5, 1.0)
        boat_east = center_east + random.uniform(-0.15, 0.15) * extent
        p1, p2 = next((a, b) for a, b in zip(top, top[1:]) if a[0] <= boat_east <= b[0])
        ratio = (boat_east - p1[0]) / (p2[0] - p1[0])
        shore_east, shore_north = boat_east, p1[1] + ratio * (p2[1] - p1[1])
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.hypot(dx, dy) or 1.0
        normal_x, normal_y = -dy / length, dx / length
        if normal_y < 0.0:
            normal_x, normal_y = -normal_x, -normal_y  # 法线指向北(水面一侧)
        self.boat.state.east_m = shore_east + normal_x * gap
        self.boat.state.north_m = shore_north + normal_y * gap
        self.boat.state.yaw_rad = 0.0
        self.boat.state.speed_mps = 0.0
        self.boat.state.yaw_rate_rps = 0.0
        self.boat.left_actual = 0.0
        self.boat.right_actual = 0.0
        # 传送后清掉旧轨迹, 免得旧位置到新位置之间出现跨图长线段
        self.true_path.clear()
        self.gps_path.clear()
        self.ekf_path.clear()
        self.latest_ekf_gps = None
        self.last_ekf_frame = None
        self.sonar_points.clear()
        self.collision_status = f"已生成默认岸边, 船在岸上 {gap:.2f} m 处"
        self._draw()

    def _finish_shoreline(self) -> None:
        if len(self.shoreline_draft) < 3:
            messagebox.showinfo("岸线", "至少需要点击 3 个点")
            return
        self.obstacles.append(PolygonObstacle(tuple(self.shoreline_draft)))
        self.shoreline_draft.clear()
        self.collision_status = "已添加岸线禁行区域"
        self._draw()

    def _delete_last_obstacle(self) -> None:
        if self.shoreline_draft:
            self.shoreline_draft.pop()
            self.collision_status = "已删除最后一个岸线点"
        elif self.obstacles:
            if self.obstacles.pop() is self.default_shore:
                self.default_shore = None
            self.collision_status = "已删除最后一个障碍物"
        else:
            self.collision_status = "没有可删除的障碍物"
        self._draw()

    def _clear_obstacles(self) -> None:
        self.obstacles.clear()
        self.default_shore = None
        self.shoreline_draft.clear()
        self.collision_status = "障碍物已清空"
        self._draw()

    # ------------------------------------------------------------------
    # 测距: 世界坐标折点 + 段长/方位角标注 + 橡皮筋预览; 与障碍物一样
    # 只是 PC 端地图标注, 不改变串口协议。
    # ------------------------------------------------------------------

    def _measure_click(self, east: float, north: float, event: tk.Event,
                       width: int, height: int, scale: float) -> None:
        """测距模式左键: 追加折点; 已有 >=3 点时点在起点 6px 内视为闭合(测周长)。"""
        if self.measure_closed:
            self.measure_status_var.set("已闭合, 按 Esc 或\"清除测距\"后重新测")
            return
        if len(self.measure_points) >= 3:
            first = self.measure_points[0]
            fx = width / 2 + (first[0] - self.view_center[0]) * scale
            fy = height / 2 - (first[1] - self.view_center[1]) * scale
            if math.hypot(event.x - fx, event.y - fy) <= 6.0:
                self.measure_closed = True
                self.measure_cursor = None
                self._refresh_measure_status()
                self._draw()
                return
        self.measure_points.append((east, north))
        self._refresh_measure_status()
        self._draw()

    def _measure_total(self) -> float:
        points = self.measure_points
        if len(points) < 2:
            return 0.0
        total = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if self.measure_closed:
            total += math.dist(points[-1], points[0])
        return total

    def _refresh_measure_status(self) -> None:
        count = len(self.measure_points)
        if self.measure_closed:
            self.measure_status_var.set(f"测距: 已闭合 {count} 点 · 周长 {self._measure_total():.1f} m")
        elif count == 0:
            self.measure_status_var.set("测距: 左键依次点选, 按钮清除")
        elif count == 1:
            self.measure_status_var.set("测距: 已放置起点, 继续左键点选")
        else:
            last = math.dist(self.measure_points[-2], self.measure_points[-1])
            self.measure_status_var.set(
                f"测距: {count} 点 · 末段 {last:.1f} m · 总长 {self._measure_total():.1f} m")

    def _clear_measure(self) -> None:
        self.measure_points.clear()
        self.measure_closed = False
        self.measure_cursor = None
        self._refresh_measure_status()
        self._draw()

    def _on_map_motion(self, event: tk.Event) -> None:
        """测距模式: 只记录光标世界坐标供橡皮筋使用; 不在这里重绘(交给 10Hz 常规重绘)。"""
        if self.obstacle_mode_var.get() != "测距" or not self.measure_points or self.measure_closed:
            self.measure_cursor = None
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        scale = getattr(self, "view_scale", 1.0)
        self.measure_cursor = (self.view_center[0] + (event.x - width / 2) / scale,
                               self.view_center[1] + (height / 2 - event.y) / scale)

    def _on_map_leave(self, _event: tk.Event) -> None:
        self.measure_cursor = None

    def _on_map_escape(self, _event: tk.Event) -> None:
        if self.measure_points or self.measure_closed:
            self._clear_measure()

    def _draw_segment_label(self, canvas: tk.Canvas, s1: tuple[float, float],
                            s2: tuple[float, float], p1: tuple[float, float],
                            p2: tuple[float, float], live: bool = False) -> None:
        """段中点标注: 长度 + 方位角(北为零, 顺时针, 三位度号, 航海习惯)。"""
        length = math.dist(p1, p2)
        bearing = math.degrees(math.atan2(p2[0] - p1[0], p2[1] - p1[1])) % 360.0
        canvas.create_text((s1[0] + s2[0]) / 2, (s1[1] + s2[1]) / 2 - 9,
                           text=f"{length:.1f} m {bearing:03.0f}°",
                           fill="#075985" if live else "#0369a1",
                           font=("TkDefaultFont", 9))

    def _draw_measure(self, canvas: tk.Canvas, xy) -> None:
        """测距标注: 已确认折线(青蓝实线+段长) + 橡皮筋(浅蓝虚线+实时长度)。"""
        points = self.measure_points
        screen = [xy(*point) for point in points]
        for index in range(len(screen) - 1):
            canvas.create_line(*screen[index], *screen[index + 1], fill="#0ea5e9", width=2)
            self._draw_segment_label(canvas, screen[index], screen[index + 1],
                                     points[index], points[index + 1])
        if self.measure_closed and len(screen) >= 3:
            canvas.create_line(*screen[-1], *screen[0], fill="#0ea5e9", width=2)
            self._draw_segment_label(canvas, screen[-1], screen[0], points[-1], points[0])
        if self.measure_cursor is not None and not self.measure_closed:
            x, y = xy(*self.measure_cursor)
            canvas.create_line(*screen[-1], x, y, fill="#7dd3fc", width=2, dash=(4, 3))
            self._draw_segment_label(canvas, screen[-1], (x, y),
                                     points[-1], self.measure_cursor, live=True)
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#7dd3fc", outline="")
        for index, (x, y) in enumerate(screen):
            radius = 5 if index == 0 else 4
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius,
                               fill="#0ea5e9", outline="#075985", width=1)
            canvas.create_text(x + 8, y - 8, text=str(index + 1), fill="#0369a1", anchor="sw")

    def _build_route_window(self) -> None:
        """航线规划独立窗口: 平时 withdraw 隐藏, 由"航线规划…"按钮弹出;
        点窗口的 X 只是隐藏(不销毁), 航线状态保留, 可随时再打开。"""
        window = tk.Toplevel(self.root)
        window.title("航线规划")
        window.geometry("340x520")
        window.protocol("WM_DELETE_WINDOW", window.withdraw)
        self.route_window = window
        panel = ttk.LabelFrame(window, text=f"航线规划 (最多 {MAX_WAYPOINTS} 点)", padding=8)
        panel.pack(fill="both", expand=True, padx=8, pady=8)
        self.route_list = tk.Listbox(panel, width=30, height=16, exportselection=False)
        self.route_list.pack(fill="both", expand=True)
        ttk.Button(panel, text="加入当前拾取点", command=self._add_waypoint).pack(fill="x", pady=(6, 0))
        route_edit_actions = ttk.Frame(panel)
        route_edit_actions.pack(fill="x", pady=(6, 0))
        ttk.Button(route_edit_actions, text="删除", command=self._delete_waypoint).pack(side="left", fill="x", expand=True)
        ttk.Button(route_edit_actions, text="清空", command=self._clear_route).pack(side="left", fill="x", expand=True, padx=(6, 0))
        route_run_actions = ttk.Frame(panel)
        route_run_actions.pack(fill="x", pady=(6, 0))
        ttk.Button(route_run_actions, text="开始航线", command=self._start_route).pack(side="left", fill="x", expand=True)
        ttk.Button(route_run_actions, text="停止", command=self._stop_route).pack(side="left", fill="x", expand=True, padx=(6, 0))
        self.route_status_var = tk.StringVar(value="航线未启动")
        ttk.Label(panel, textvariable=self.route_status_var, wraplength=200).pack(fill="x", pady=(8, 0))
        window.withdraw()  # 平时不显示

    def _open_route_window(self) -> None:
        if self.route_window is None or not self.route_window.winfo_exists():
            self._build_route_window()
        self.route_window.deiconify()
        self.route_window.lift()

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
            if len(self.ekf_path) > 9_000:   # 5Hz ≈ 30 分钟历史
                del self.ekf_path[:3_000]

    # ------------------------------------------------------------------
    # 导航扩展: 会话建立 / CONFIG / RANGE 已在 _simulate_step; 此处是
    # 接收侧(0x83 状态 + 0x84 点表拼装)与两阶段任务的下发按钮。
    # ------------------------------------------------------------------

    def _establish_nav_session(self) -> None:
        """连接后建立导航会话: 生成非零 session, 下发 FRAME(0x10) + CONFIG(0x13)。

        MCU 导航引擎只有认了 FRAME 才收 RANGE/TASK/CONFIG; 每次连接重新生成
        session, 旧会话的任务与重发请求随之全部失效(防跨连接误执行)。"""
        if not self.link.connected:
            messagebox.showerror("导航", "请先连接串口")
            return
        self.nav_session = (int(time.time() * 1000) & 0x7FFFFFFF) or 1
        self.nav_request = 0
        self._reset_plan_capture()
        frame = encode_nav_frame(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                 self._value("lat0"), self._value("lon0"), self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)
        self._send_nav_config()

    def _sonar_config(self) -> SonarConfig:
        return SonarConfig(
            sensor_range=self._value("sonar_range"),
            beam_width=math.radians(self._value("sonar_beam_deg")),
            axis_gap=math.radians(self._value("sonar_axis_deg")),
            sensor_forward=self._value("sonar_forward"),
            sensor_lateral=self._value("sonar_lateral"),
            boat_radius=self._value("collision_radius"),
            margin=0.0,
            max_v=self._value("max_speed"),
            max_w=self._value("max_yaw"),
            acceleration=0.5,
            angular_acceleration=1.0,
            lookahead=1.2,
        )

    def _send_nav_config(self) -> None:
        if not self.nav_session:
            return
        config = self._sonar_config()
        values = (config.sensor_range, config.beam_width, config.axis_gap,
                  config.sensor_forward, config.sensor_lateral, config.boat_radius,
                  config.margin, config.max_v, config.max_w, config.acceleration,
                  config.angular_acceleration, config.lookahead)
        frame = encode_nav_config(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                  values, self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)

    def _reset_plan_capture(self) -> None:
        self.plan_points = []
        self.plan_have = set()
        self.plan_total = 0
        self.plan_id = 0
        self.plan_final_seen = False
        self.plan_complete = False
        self.plan_last_resend_s = 0.0
        self.plan_last_segment_s = 0.0
        self._update_execute_button()

    def _update_execute_button(self) -> None:
        if hasattr(self, "execute_button"):
            self.execute_button.configure(state="normal" if self.plan_complete else "disabled")

    def _capture_nav(self) -> None:
        """每 tick: 收 0x83 状态, 拼装 0x84 点表; 末段已到但拼不齐、或 3s 无新段且不齐 → 重传。"""
        status = self.link.nav_status()
        if status is not None:
            self.nav_status = status
        received_any = False
        for segment in self.link.take_plan_segments():
            received_any = True
            if self.plan_id and segment.plan_id != self.plan_id:
                self._reset_plan_capture()  # 新一次规划到来, 旧表作废
            if not self.plan_id:
                self.plan_id = segment.plan_id
                self.plan_total = segment.total
                self.plan_points = [None] * segment.total
            if segment.total != self.plan_total or segment.total <= 0:
                continue
            for offset, point in enumerate(segment.points):
                index = segment.start_index + offset
                if 0 <= index < self.plan_total:
                    self.plan_points[index] = point
            self.plan_have.add(segment.start_index)
            if segment.flags & 1:
                self.plan_final_seen = True
        if received_any:
            self.plan_last_segment_s = self.sim_time
        if self.plan_total and not self.plan_complete:
            have = sum(1 for point in self.plan_points if point is not None)
            if have >= self.plan_total:
                self.plan_complete = True
                self._update_execute_button()
            elif self.sim_time - max(self.plan_last_segment_s, self.plan_last_resend_s) >= 3.0:
                # 末段丢了收不到 final 标志也照样兜底: 3 秒没有新段且仍不齐 → 整表重传
                self.plan_last_resend_s = self.sim_time
                self._request_plan_resend()
        self._refresh_survey_status()

    def _refresh_survey_status(self) -> None:
        if not hasattr(self, "survey_status_var"):
            return
        status = self.nav_status
        if status is None:
            self.survey_status_var.set("MCU 状态: 尚未收到 0x83 帧")
            return
        have = sum(1 for point in self.plan_points if point is not None) if self.plan_total else 0
        plan_text = f"点表 {have}/{self.plan_total}" if self.plan_total else "点表: 未收到"
        if self.plan_complete:
            plan_text += " 已收齐, 可确认执行"
        self.survey_status_var.set(
            f"MCU 状态: {status.state_name}   ACK: {status.ack_name}\n"
            f"运行: {'是' if status.running else '否'}   当前点: {status.current + 1}/{status.total}\n"
            f"到达 {status.visited} / 判岸 {status.shore} / 跳过 {status.skipped}\n"
            f"{plan_text}\n"
            f"命令拒绝 {status.rejected}   声呐拒绝 {status.range_rejected}   重排 {status.replans}"
        )

    def _send_survey_plan(self) -> None:
        if not self.nav_session:
            messagebox.showerror("扫测", "请先连接串口(自动建立导航会话)")
            return
        try:
            args = [self._value(key) for key in
                    ("survey_step", "survey_spacing", "survey_launch", "survey_radius", "survey_tol")]
        except (tk.TclError, ValueError) as exc:
            messagebox.showerror("扫测参数错误", str(exc))
            return
        self._reset_plan_capture()
        self.nav_request += 1
        frame = encode_nav_task(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                NAV_CMD_SURVEY_PLAN, self.nav_request, args, frame_id=self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)

    def _confirm_plan_execute(self) -> None:
        if not self.plan_complete:
            messagebox.showinfo("扫测", "点表尚未收齐, 不能执行")
            return
        self.nav_request += 1
        frame = encode_nav_task(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                NAV_CMD_PLAN_EXECUTE, self.nav_request, frame_id=self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)

    def _request_plan_resend(self) -> None:
        if not self.nav_session:
            return
        self.nav_request += 1
        frame = encode_nav_task(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                NAV_CMD_PLAN_RESEND, self.nav_request, frame_id=self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)

    def _send_nav_stop(self) -> None:
        if not self.nav_session:
            return
        self.nav_request += 1
        frame = encode_nav_task(self.sequence, self.nav_session, int(self.sim_time * 1000),
                                NAV_CMD_STOP, self.nav_request, frame_id=self.nav_frame_id)
        self.sequence += 1
        self.link.send(frame)

    def _simulate_step(self) -> None:
        left, right, _ = self._motor_values()
        previous_position = (self.boat.state.east_m, self.boat.state.north_m)
        self.boat.step(left, right, SIM_DT)
        state = self.boat.state
        collision = resolve_motion(
            previous_position,
            (state.east_m, state.north_m),
            self._value("collision_radius"),
            self.obstacles,
        )
        if collision is not None:
            state.east_m, state.north_m = collision.position
            if self.collision_mode_var.get() == "沿边滑行":
                velocity = (state.speed_mps * math.sin(state.yaw_rad), state.speed_mps * math.cos(state.yaw_rad))
                normal_speed = velocity[0] * collision.normal[0] + velocity[1] * collision.normal[1]
                tangent = (velocity[0] - normal_speed * collision.normal[0], velocity[1] - normal_speed * collision.normal[1])
                tangent_speed = math.hypot(*tangent)
                if tangent_speed > 1e-4:
                    state.speed_mps = tangent_speed
                    state.yaw_rad = math.atan2(tangent[0], tangent[1])
                else:
                    state.speed_mps = 0.0
                state.yaw_rate_rps = 0.0
            else:
                state.speed_mps = 0.0
                state.yaw_rate_rps = 0.0
            self.collision_status = "碰撞：" + ("沿边滑行" if self.collision_mode_var.get() == "沿边滑行" else "已停止")
        else:
            self.collision_status = "无碰撞"
        self.sim_time += SIM_DT
        if not self.true_path or math.dist(self.true_path[-1], (state.east_m, state.north_m)) >= 0.05:
            self.true_path.append((state.east_m, state.north_m))
            if len(self.true_path) > 18_000:  # 10Hz ≈ 30 分钟历史
                del self.true_path[:6_000]
        gps_period_s = self._gps_period_s()
        gps_new = self.sim_time - self.last_gps_time >= gps_period_s - 1e-9
        if gps_new:
            self.last_gps_time = self.sim_time
            self.latest_gps = self.gps.sample(state, self._value("lat0"), self._value("lon0"), self._value("gps_accuracy"))
            self.gps_path.append(gps_to_local(*self.latest_gps, self._value("lat0"), self._value("lon0")))
            if len(self.gps_path) > 9_000:   # 5Hz ≈ 30 分钟; 补上上限, 消除唯一无界增长
                del self.gps_path[:3_000]
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
        if self.nav_session and self.sim_time - self.last_range_send >= 0.1 - 1e-9:
            # 导航声呐: 每 100ms 对世界障碍物做双波束射线求交, 打成 RANGE(0x11) 下发。
            # 无回波通道按协议约定回 满量程 + hit=0(有效无回波), 不是无效通道。
            self.last_range_send = self.sim_time
            config = self._sonar_config()
            sample = synthesize_sonar(state, config, self.obstacles)
            self.last_sonar_text = (sample.left_m, sample.right_m, sample.hit)
            (left, right) = transducer_beams(state, config)
            beams = []
            for (origin, bearing), distance, hit_bit in ((left, sample.left_m, 1),
                                                         (right, sample.right_m, 2)):
                hit = bool(sample.hit & hit_bit)
                end = (origin[0] + math.sin(bearing) * distance,
                       origin[1] + math.cos(bearing) * distance)
                beams.append((origin, end, hit))
                if hit:
                    # 回波端点入点云: 船边扫过的障碍表面会逐渐连成轮廓
                    if not self.sonar_points or math.dist(self.sonar_points[-1], end) >= 0.05:
                        self.sonar_points.append(end)
                        if len(self.sonar_points) > 18_000:  # 10Hz ≈ 30 分钟回波
                            del self.sonar_points[:6_000]
            self.last_sonar_beams = beams
            frame = encode_range(
                self.sequence, self.nav_session, int(self.sim_time * 1000),
                sample.valid, sample.hit, sample.left_m, sample.right_m,
                sample.max_range, self.nav_frame_id,
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
        self._capture_nav()
        if self.running:
            self.accumulator += elapsed
            while self.accumulator >= SIM_DT:
                self._simulate_step()
                self.accumulator -= SIM_DT
        if now - self.last_draw >= 0.1:
            self.last_draw = now
            self._draw()
        self.root.after(20, self._tick)

    def _cloud_points(
        self,
        points: list[tuple[float, float]],
        to_screen,
        width: int,
        height: int,
        limit: int,
        min_gap_px: float = 3.0,
        window: int = 4,
    ) -> list[tuple[float, float]]:
        """点云(oval 团)的绘制筛选: 视口剔除 + 屏幕空间抽稀 + 数量限幅。

        - 视口剔除: 屏幕坐标落在画布外(留 8px 边距)的点不创建 item
          (Tk 对看不见的 item 一样照收, 纯浪费)
        - 屏幕空间抽稀: 与上一个入选点屏幕距离不足 min_gap_px 的跳过,
          密的区域自动变疏、疏的区域保留细节, 轮廓形状不变
        - 数量限幅: 最多返回 limit 个; 只扫描最新的 window*limit 个点
          (更旧的点基本已驶出视野), 避免每帧全量遍历长缓冲

        缓冲存多久不影响每帧成本: 30 分钟和一分钟的点云, 画出的 oval 一样多。"""
        margin = 8.0
        candidates = points[-(window * limit):] if len(points) > window * limit else points
        kept: list[tuple[float, float]] = []
        last_x = last_y = 0.0
        for east, north in reversed(candidates):  # 新的优先, 越旧越先被舍弃
            x, y = to_screen(east, north)
            if not (-margin <= x <= width + margin and -margin <= y <= height + margin):
                continue
            if kept and math.hypot(x - last_x, y - last_y) < min_gap_px:
                continue
            kept.append((x, y))
            last_x, last_y = x, y
            if len(kept) >= limit:
                break
        return kept

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
        scale = min(width, height) / (2.0 * self.view_extent)
        self.view_scale = scale
        if self.follow_boat.get():
            # 跟随模式: 镜头中心锁定船体位置(船开离原点后不用手动追)
            self.view_center[0] = self.boat.state.east_m
            self.view_center[1] = self.boat.state.north_m

        def xy(east: float, north: float) -> tuple[float, float]:
            return width / 2 + (east - self.view_center[0]) * scale, height / 2 - (north - self.view_center[1]) * scale

        grid_step = (1 if self.view_extent <= 6 else
                     2 if self.view_extent <= 12 else
                     5 if self.view_extent <= 25 else
                     10 if self.view_extent <= 60 else 20)
        # 网格覆盖当前可见世界范围(镜头可平移, 不再以原点为界)
        east_start = math.floor((self.view_center[0] - width / (2.0 * scale)) / grid_step)
        east_end = math.ceil((self.view_center[0] + width / (2.0 * scale)) / grid_step)
        north_start = math.floor((self.view_center[1] - height / (2.0 * scale)) / grid_step)
        north_end = math.ceil((self.view_center[1] + height / (2.0 * scale)) / grid_step)
        for index in range(east_start, east_end + 1):
            x, _ = xy(index * grid_step, 0)
            canvas.create_line(x, 0, x, height, fill="#e5e7eb")
        for index in range(north_start, north_end + 1):
            _, y = xy(0, index * grid_step)
            canvas.create_line(0, y, width, y, fill="#e5e7eb")
        for obstacle in self.obstacles:
            if isinstance(obstacle, CircleObstacle):
                ox, oy = xy(obstacle.east_m, obstacle.north_m)
                radius_px = obstacle.radius_m * scale
                canvas.create_oval(ox - radius_px, oy - radius_px, ox + radius_px, oy + radius_px,
                                   fill="#94a3b8", outline="#475569", width=2)
                canvas.create_oval(ox - radius_px * 0.45, oy - radius_px * 0.25,
                                   ox + radius_px * 0.20, oy + radius_px * 0.10,
                                   fill="#cbd5e1", outline="")
            else:
                polygon = [xy(*point) for point in obstacle.points]
                canvas.create_polygon(*(coord for point in polygon for coord in point),
                                      fill="#64748b", outline="#334155", width=2, stipple="gray25")
        if self.shoreline_draft:
            draft = [xy(*point) for point in self.shoreline_draft]
            if len(draft) >= 2:
                canvas.create_line(*(coord for point in draft for coord in point), fill="#0f766e", width=2, dash=(5, 3))
            for index, point in enumerate(draft):
                canvas.create_oval(point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4, fill="#14b8a6", outline="")
                canvas.create_text(point[0] + 7, point[1] - 7, text=str(index + 1), fill="#0f766e", anchor="sw")
        if len(self.true_path) >= 2:
            display_path = self._display_points(self.true_path, 1000)
            canvas.create_line(*(coord for point in display_path for coord in xy(*point)), fill="#2563eb", width=2)
        for gx, gy in self._cloud_points(self.gps_path, xy, width, height, 60, min_gap_px=4.0):
            canvas.create_oval(gx - 3, gy - 3, gx + 3, gy + 3, fill="#f59e0b", outline="")
        if len(self.ekf_path) >= 2:
            display_ekf = self._display_points(self.ekf_path, 1000)
            canvas.create_line(*(coord for point in display_ekf for coord in xy(*point)), fill="#16a34a", width=2)
        for ex, ey in self._cloud_points(self.ekf_path, xy, width, height, 100):
            canvas.create_oval(ex - 2, ey - 2, ex + 2, ey + 2, fill="#16a34a", outline="")
        if self.plan_total and any(point is not None for point in self.plan_points):
            # 扫测计划点表(紫) + 当前目标点(红, 来自 0x83 的 current 下标); 稀疏抽样控元素数
            stride = 2 if self.plan_total > 150 else 1
            for point in self.plan_points[::stride]:
                if point is None:
                    continue
                px, py = xy(point[1], point[0])
                if not (-8 <= px <= width + 8 and -8 <= py <= height + 8):
                    continue  # 视口外的计划点不创建 item
                canvas.create_oval(px - 2, py - 2, px + 2, py + 2, fill="#7c3aed", outline="")
            status = self.nav_status
            if status is not None and 0 <= status.current < len(self.plan_points):
                current = self.plan_points[status.current]
                if current is not None:
                    cx, cy = xy(current[1], current[0])
                    canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                                       fill="#dc2626", outline="#7f1d1d")
        if len(route_points) >= 2:
            canvas.create_line(*(coord for point in route_points for coord in xy(*point)), fill="#475569", width=2, dash=(5, 3))
        for index, point in enumerate(route_points):
            wx, wy = xy(*point)
            active = self.route.active_index == index
            color = "#dc2626" if active else "#334155"
            radius = 7 if active else 5
            canvas.create_oval(wx - radius, wy - radius, wx + radius, wy + radius, fill=color, outline="")
            canvas.create_text(wx + 9, wy - 9, text=str(index + 1), fill=color, anchor="sw")
        if self.measure_points:
            self._draw_measure(canvas, xy)
        tx, ty = xy(*target)
        radius_px = max(3, self._value("arrival_radius") * scale)
        canvas.create_oval(tx - radius_px, ty - radius_px, tx + radius_px, ty + radius_px, outline="#16a34a", width=2)
        canvas.create_line(tx - 7, ty, tx + 7, ty, fill="#16a34a", width=2)
        canvas.create_line(tx, ty - 7, tx, ty + 7, fill="#16a34a", width=2)
        # 声呐回波点云(障碍轮廓, 深棕小点) + 当前双波束(橙实线=有回波, 浅橙虚线=无回波)
        # 点云显示上限 600: canvas 元素数是重绘成本的大头(曾致缩放时 GUI 卡顿饿死仿真)
        for px, py in self._cloud_points(self.sonar_points, xy, width, height, 300, min_gap_px=2.5):
            canvas.create_oval(px - 1.5, py - 1.5, px + 1.5, py + 1.5, fill="#b45309", outline="")
        if self.last_sonar_beams:
            for origin, end, hit in self.last_sonar_beams:
                ox, oy = xy(*origin)
                ex, ey = xy(*end)
                if hit:
                    canvas.create_line(ox, oy, ex, ey, fill="#f97316", width=2)
                    canvas.create_oval(ex - 3, ey - 3, ex + 3, ey + 3,
                                       fill="#f97316", outline="#7c2d12")
                else:
                    canvas.create_line(ox, oy, ex, ey, fill="#fdba74", dash=(4, 4))
        state = self.boat.state
        bx, by = xy(state.east_m, state.north_m)
        forward = (math.sin(state.yaw_rad), -math.cos(state.yaw_rad))
        right = (math.cos(state.yaw_rad), math.sin(state.yaw_rad))

        def hull_point(forward_m: float, lateral_m: float) -> tuple[float, float]:
            return (
                bx + forward[0] * forward_m + right[0] * lateral_m,
                by + forward[1] * forward_m + right[1] * lateral_m,
            )

        # 真实比例船体: 0.45m 长 / 0.2m 宽(与船体参数一致), 带最小像素下限保证可见。
        # 80m 视图(约 10px/m)下真实尺寸只有 ~4.5x2px, 贴 12x6px 下限; 视图放大后按比例生长。
        boat_len = max(12.0, 0.45 * scale)
        boat_wid = max(6.0, 0.20 * scale)
        hull = [hull_point(boat_len * 0.55, 0), hull_point(boat_len * 0.24, -boat_wid / 2),
                hull_point(-boat_len * 0.38, -boat_wid * 0.45), hull_point(-boat_len * 0.45, 0),
                hull_point(-boat_len * 0.38, boat_wid * 0.45), hull_point(boat_len * 0.24, boat_wid / 2)]
        canvas.create_polygon(*(coord for point in hull for coord in point), fill="#0f4c81", outline="#082f49", width=2)
        deck = [hull_point(boat_len * 0.24, 0), hull_point(boat_len * 0.10, -boat_wid * 0.32),
                hull_point(-boat_len * 0.24, -boat_wid * 0.32), hull_point(-boat_len * 0.31, 0),
                hull_point(-boat_len * 0.24, boat_wid * 0.32), hull_point(boat_len * 0.10, boat_wid * 0.32)]
        canvas.create_polygon(*(coord for point in deck for coord in point), fill="#38bdf8", outline="#0c4a6e")
        cabin = [hull_point(boat_len * 0.14, -boat_wid * 0.20), hull_point(boat_len * 0.02, -boat_wid * 0.20),
                 hull_point(-boat_len * 0.12, -boat_wid * 0.16), hull_point(-boat_len * 0.12, boat_wid * 0.16),
                 hull_point(boat_len * 0.02, boat_wid * 0.20), hull_point(boat_len * 0.14, boat_wid * 0.20)]
        canvas.create_polygon(*(coord for point in cabin for coord in point), fill="#e0f2fe", outline="#075985")
        prop_r = max(1.5, boat_wid * 0.16)
        for lateral in (-boat_wid * 0.36, boat_wid * 0.36):
            px, py = hull_point(-boat_len * 0.36, lateral)
            canvas.create_oval(px - prop_r, py - prop_r, px + prop_r, py + prop_r, fill="#f59e0b", outline="#92400e")
        canvas.create_line(bx, by, *hull_point(boat_len * 0.7, 0), fill="#f97316", width=2)
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
        if self.last_sonar_text is None:
            sonar_text = "-- / --"
        else:
            left_m, right_m, hit = self.last_sonar_text
            sonar_text = (f"{left_m:.1f}{'*' if hit & 1 else ''}/"
                          f"{right_m:.1f}{'*' if hit & 2 else ''} m")
        self.status_var.set(f"{link_state} / {run_state}{arrived}")
        # 遥测栏两行: 上行=控制与运动, 下行=传感器与链路; * = 该通道有回波
        self.telemetry_row1_var.set(
            f"输入 {source} · 原始 {raw_left}/{raw_right} · 仿真 {left}/{right} · "
            f"东/北 {state.east_m:.1f}/{state.north_m:.1f} m · 航向 {state.yaw_rad:.2f} rad · "
            f"速度 {state.speed_mps:.2f} m/s · 角速度 {state.yaw_rate_rps:.2f} rad/s · "
            f"目标距离 {distance:.1f} m"
        )
        self.telemetry_row2_var.set(
            f"GPS {self.latest_gps[0]:.7f}, {self.latest_gps[1]:.7f} · EKF {ekf_text} · "
            f"声呐 {sonar_text} · 电机帧 {self.link.stats.received_motor_frames} · "
            f"EKF 帧 {self.link.stats.received_ekf_frames} · 回波点 {len(self.sonar_points)} · "
            f"障碍物 {len(self.obstacles)} / {self.collision_status}"
        )

    def _close(self) -> None:
        self.link.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    BoatSimulatorApp(root)
    root.mainloop()
