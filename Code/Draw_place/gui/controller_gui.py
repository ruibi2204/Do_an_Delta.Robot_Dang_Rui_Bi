# ============================================================
#  gui/controller_gui.py — Giao diện điều khiển Robot Delta
# ============================================================
"""
Giao diện Tkinter full-screen cho robot Delta.

Layout:
  ┌─────────────────────────────────────────────────────┐
  │  HEADER: tiêu đề + trạng thái kết nối               │
  ├──────────────┬──────────────────────────────────────┤
  │  LEFT PANEL  │  CANVAS (vẽ quỹ đạo 2D top-view)     │
  │  (scrollable)│                                      │
  │  • Chọn hình │                                      │
  │  • Tham số   │                                      │
  │  • Serial    │                                      │
  │  • Điều khiển│                                      │
  ├──────────────┴──────────────────────────────────────┤
  │  BOTTOM: góc IK + góc stepper + serial log          │
  └─────────────────────────────────────────────────────┘
"""

import tkinter as tk
from tkinter import ttk, messagebox, font as tkfont
import threading
import math
import time
from typing import Optional

from kinematics.inverse_kinematics import inverse_kinematics
from trajectory.generator import generate_circle, generate_square, generate_triangle
from communication.uart_comm import UARTComm
from PID_control.PID_control import DeltaRobotPID


# ─── Màu sắc (dark theme) ─────────────────────────────────────────────────────
C = {
    "bg":       "#0d1117",
    "bg2":      "#161b22",
    "bg3":      "#21262d",
    "border":   "#30363d",
    "accent":   "#58a6ff",
    "green":    "#3fb950",
    "yellow":   "#d29922",
    "red":      "#f85149",
    "text":     "#e6edf3",
    "muted":    "#8b949e",
}


class DeltaRobotGUI:
    """Cửa sổ chính điều khiển Robot Delta."""

    GEAR_RATIO    = 3.2
    STEP_DELAY_MS = 50     # khoảng cách giữa các điểm (ms) — tốc độ mặc định

    def __init__(self, root: tk.Tk):
        self.root = root
        self._setup_window()
        self._build_ui()

        # Trạng thái
        self._trajectory: list = []
        self._step_idx: int = 0
        self._running: bool = False
        self._after_id: Optional[str] = None
        self._drawn_pts: list = []

        self._uart: Optional[UARTComm] = None

        # Bộ điều khiển PID (3 khớp)
        self._pid = DeltaRobotPID(
            Kp=1.0, Ki=0.05, Kd=0.02,
            output_limit=15.0,
            integral_limit=10.0,
            deadband=0.05,
            enabled=False,      # mặc định TẮT, bật qua GUI
        )
        # Lưu góc phản hồi ước tính (simulation: feedback = setpoint lần trước)
        self._fb_angles: list = [0.0, 0.0, 0.0]
        self._prev_sp:   list = [0.0, 0.0, 0.0]

        self._log("SYS", "Delta Robot Controller khởi động thành công")
        self._log("SYS", f"R=100 a=130 b=298 r=35.3 mm | Tỉ số đai u={self.GEAR_RATIO}")
        self._log("SYS", "PID Controller sẵn sàng (mặc định TẮT — bật trong tab PID)")

    # ══════════════════════════════════════════════════════════════════════════
    # THIẾT LẬP CỬA SỔ
    # ══════════════════════════════════════════════════════════════════════════
    def _setup_window(self):
        self.root.title("Delta Robot — Trajectory Controller")
        self.root.configure(bg=C["bg"])
        self.root.state("zoomed")           # full-screen (Windows/Linux)
        try:
            self.root.attributes("-zoomed", True)   # Linux fallback
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # XÂY DỰNG GIAO DIỆN
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        self._build_header()

        # Khung chính
        main = tk.Frame(self.root, bg=C["bg"])
        main.pack(fill="both", expand=True)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        self._build_left_panel(main)
        self._build_canvas(main)
        self._build_bottom_panel()

    # ── HEADER ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=C["bg2"], height=44)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)

        tk.Label(hdr, text="●", fg=C["green"], bg=C["bg2"],
                 font=("Courier", 12)).pack(side="left", padx=(12, 4), pady=8)
        tk.Label(hdr, text="DELTA ROBOT — TRAJECTORY CONTROLLER",
                 fg=C["accent"], bg=C["bg2"],
                 font=("Courier", 13, "bold")).pack(side="left", pady=8)

        self._conn_label = tk.Label(hdr, text="⬤  CHƯA KẾT NỐI",
                                    fg=C["red"], bg=C["bg2"],
                                    font=("Courier", 10, "bold"))
        self._conn_label.pack(side="right", padx=16)

        tk.Label(hdr, text=f"R=100 | a=130 | b=298 | r=35.3 mm | u={self.GEAR_RATIO}",
                 fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9)).pack(side="right", padx=24)

    # ── LEFT PANEL (scrollable) ───────────────────────────────────────────────
    def _build_left_panel(self, parent):
        # Outer frame — fixed width, clips content
        lf = tk.Frame(parent, bg=C["bg2"], width=300)
        lf.grid(row=0, column=0, sticky="nsew")
        lf.pack_propagate(False)
        lf.grid_propagate(False)

        # Canvas + Scrollbar for scrollable inner content
        scroll_canvas = tk.Canvas(lf, bg=C["bg2"], highlightthickness=0,
                                   bd=0, width=284)
        scrollbar = ttk.Scrollbar(lf, orient="vertical",
                                   command=scroll_canvas.yview)
        scroll_canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        scroll_canvas.pack(side="left", fill="both", expand=True)

        # Inner frame inside the scroll canvas
        inner = tk.Frame(scroll_canvas, bg=C["bg2"])
        inner_id = scroll_canvas.create_window((0, 0), window=inner,
                                                anchor="nw")

        def _on_inner_configure(event):
            scroll_canvas.configure(
                scrollregion=scroll_canvas.bbox("all"))

        def _on_canvas_configure(event):
            # Make inner frame fill the canvas width
            scroll_canvas.itemconfig(inner_id, width=event.width)

        inner.bind("<Configure>", _on_inner_configure)
        scroll_canvas.bind("<Configure>", _on_canvas_configure)

        # Bind mouse-wheel scrolling
        def _on_mousewheel(event):
            scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _on_mousewheel_linux(event):
            if event.num == 4:
                scroll_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                scroll_canvas.yview_scroll(1, "units")

        scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)          # Win/Mac
        scroll_canvas.bind_all("<Button-4>", _on_mousewheel_linux)      # Linux
        scroll_canvas.bind_all("<Button-5>", _on_mousewheel_linux)      # Linux

        self._fill_left_panel(inner)

    def _fill_left_panel(self, inner):
        """Điền toàn bộ nội dung vào inner frame của left panel."""
        pad = dict(padx=10)

        # ── Chọn hình ─────────────────────────────────────────────────────────
        self._section_label(inner, "CHỌN HÌNH VẼ", **pad)
        self._shape_var = tk.StringVar(value="circle")
        shapes = [("⬤  Hình tròn",   "circle"),
                  ("■  Hình vuông",   "square"),
                  ("▲  Tam giác đều", "triangle")]
        self._shape_btns = {}
        for text, val in shapes:
            b = tk.Button(inner, text=text, anchor="w",
                          bg=C["bg3"], fg=C["text"],
                          activebackground=C["accent"], activeforeground=C["bg"],
                          relief="flat", bd=0, padx=10, pady=6,
                          font=("Courier", 11),
                          command=lambda v=val: self._select_shape(v))
            b.pack(fill="x", pady=2, **pad)
            self._shape_btns[val] = b

        self._sep(inner)

        # ── Tham số hình ──────────────────────────────────────────────────────
        self._section_label(inner, "THAM SỐ HÌNH", **pad)
        self._param_frame = tk.Frame(inner, bg=C["bg2"])
        self._param_frame.pack(fill="x", **pad)
        self._params: dict[str, tk.DoubleVar] = {}

        self._select_shape("circle", init=True)

        self._sep(inner)

        # ── Tốc độ ────────────────────────────────────────────────────────────
        self._section_label(inner, "TỐC ĐỘ VẼ", **pad)
        spd_row = tk.Frame(inner, bg=C["bg2"])
        spd_row.pack(fill="x", **pad)
        self._speed_var = tk.IntVar(value=5)
        tk.Scale(spd_row, from_=1, to=20, orient="horizontal",
                 variable=self._speed_var,
                 bg=C["bg2"], fg=C["text"], troughcolor=C["bg3"],
                 highlightthickness=0, bd=0,
                 font=("Courier", 9)).pack(side="left", fill="x", expand=True)
        tk.Label(spd_row, textvariable=self._speed_var,
                 fg=C["accent"], bg=C["bg2"],
                 font=("Courier", 10, "bold"), width=3).pack(side="right")

        self._sep(inner)

        # ── Kết nối Serial ────────────────────────────────────────────────────
        self._section_label(inner, "SERIAL / UART", **pad)
        port_row = tk.Frame(inner, bg=C["bg2"])
        port_row.pack(fill="x", pady=2, **pad)
        tk.Label(port_row, text="Port:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9), width=6).pack(side="left")
        _detected = UARTComm.list_ports()
        _fixed    = ["COM4", "COM3", "COM5", "/dev/ttyUSB0", "DRY-RUN"]
        ports     = list(dict.fromkeys(_detected + _fixed))
        self._port_var = tk.StringVar(value="COM4")
        self._port_combo = ttk.Combobox(port_row, textvariable=self._port_var,
                                        values=ports, width=14,
                                        font=("Courier", 9))
        self._port_combo.pack(side="left", padx=4)

        baud_row = tk.Frame(inner, bg=C["bg2"])
        baud_row.pack(fill="x", pady=2, **pad)
        tk.Label(baud_row, text="Baud:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9), width=6).pack(side="left")
        self._baud_var = tk.StringVar(value="115200")
        ttk.Combobox(baud_row, textvariable=self._baud_var,
                     values=["9600", "57600", "115200", "250000"],
                     width=14, font=("Courier", 9)).pack(side="left", padx=4)

        self._connect_btn = tk.Button(inner, text="⚡ KẾT NỐI",
                                      bg=C["green"], fg=C["bg"],
                                      font=("Courier", 10, "bold"),
                                      relief="flat", bd=0, pady=5,
                                      command=self._toggle_connect)
        self._connect_btn.pack(fill="x", pady=4, **pad)

        self._sep(inner)

        # ── Điều khiển PID ────────────────────────────────────────────────────
        self._section_label(inner, "BỘ ĐIỀU KHIỂN PID", **pad)
        self._pid_enabled_var = tk.BooleanVar(value=False)
        pid_toggle = tk.Checkbutton(
            inner, text="⚙  BẬT PID",
            variable=self._pid_enabled_var,
            command=self._on_pid_toggle,
            bg=C["bg3"], fg=C["text"],
            selectcolor=C["bg"],
            activebackground=C["bg3"], activeforeground=C["accent"],
            font=("Courier", 10, "bold"),
            relief="flat", bd=0, padx=10, pady=4,
            anchor="w",
        )
        pid_toggle.pack(fill="x", pady=2, **pad)

        # Slider Kp
        self._kp_var = tk.DoubleVar(value=1.0)
        self._ki_var = tk.DoubleVar(value=0.05)
        self._kd_var = tk.DoubleVar(value=0.02)

        for label, var, from_, to, res in [
            ("Kp", self._kp_var, 0.0, 5.0, 0.05),
            ("Ki", self._ki_var, 0.0, 2.0, 0.01),
            ("Kd", self._kd_var, 0.0, 1.0, 0.005),
        ]:
            row = tk.Frame(inner, bg=C["bg2"])
            row.pack(fill="x", **pad)
            tk.Label(row, text=f"{label}:", fg=C["muted"], bg=C["bg2"],
                     font=("Courier", 9), width=3).pack(side="left")
            tk.Scale(
                row, from_=from_, to=to, resolution=res,
                orient="horizontal", variable=var,
                command=lambda _=None: self._on_pid_gains_change(),
                bg=C["bg2"], fg=C["text"], troughcolor=C["bg3"],
                highlightthickness=0, bd=0,
                font=("Courier", 8),
            ).pack(side="left", fill="x", expand=True)
            tk.Label(row, textvariable=var, fg=C["accent"], bg=C["bg2"],
                     font=("Courier", 9), width=5).pack(side="right")

        # Nhãn hiển thị sai lệch PID thời gian thực
        pid_err_row = tk.Frame(inner, bg=C["bg2"])
        pid_err_row.pack(fill="x", **pad)
        tk.Label(pid_err_row, text="Err:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9), width=4).pack(side="left")
        self._pid_err_var = tk.StringVar(value="0.00 | 0.00 | 0.00")
        tk.Label(pid_err_row, textvariable=self._pid_err_var,
                 fg=C["yellow"], bg=C["bg2"],
                 font=("Courier", 9)).pack(side="left")

        self._sep(inner)

        # ── Nút điều khiển ────────────────────────────────────────────────────
        self._section_label(inner, "ĐIỀU KHIỂN", **pad)
        self._run_btn = tk.Button(inner, text="▶  BẮT ĐẦU VẼ",
                                  bg=C["accent"], fg=C["bg"],
                                  font=("Courier", 12, "bold"),
                                  relief="flat", bd=0, pady=8,
                                  command=self._toggle_run)
        self._run_btn.pack(fill="x", pady=2, **pad)

        self._home_btn = tk.Button(inner, text="⌂  HOME",
                                   bg=C["bg3"], fg=C["text"],
                                   font=("Courier", 10),
                                   relief="flat", bd=0, pady=6,
                                   command=self._send_home)
        self._home_btn.pack(fill="x", pady=2, **pad)

        self._reset_btn = tk.Button(inner, text="↺  ĐẶT LẠI",
                                    bg=C["bg3"], fg=C["text"],
                                    font=("Courier", 10),
                                    relief="flat", bd=0, pady=6,
                                    command=self._reset)
        self._reset_btn.pack(fill="x", pady=2, **pad)

        # ── Thanh tiến độ ─────────────────────────────────────────────────────
        self._sep(inner)
        self._progress_var = tk.DoubleVar(value=0)
        tk.Label(inner, text="TIẾN ĐỘ", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 8)).pack(anchor="w", **pad)
        self._progress_bar = ttk.Progressbar(inner, variable=self._progress_var,
                                              maximum=100)
        self._progress_bar.pack(fill="x", pady=2, **pad)
        self._progress_lbl = tk.Label(inner, text="0 / 0",
                                      fg=C["muted"], bg=C["bg2"],
                                      font=("Courier", 9))
        self._progress_lbl.pack(anchor="e", padx=10)

        # Padding cuối để nội dung không sát đáy
        tk.Frame(inner, bg=C["bg2"], height=12).pack()

    # ── CANVAS ────────────────────────────────────────────────────────────────
    def _build_canvas(self, parent):
        cf = tk.Frame(parent, bg=C["bg"])
        cf.grid(row=0, column=1, sticky="nsew")

        self._canvas = tk.Canvas(cf, bg=C["bg"], highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._canvas.bind("<Configure>", self._on_canvas_resize)

    # ── BOTTOM PANEL ──────────────────────────────────────────────────────────
    def _build_bottom_panel(self):
        bot = tk.Frame(self.root, bg=C["bg2"], height=180)
        bot.pack(fill="x")
        bot.pack_propagate(False)

        # --- Góc IK (trái) ---
        angle_frame = tk.Frame(bot, bg=C["bg2"])
        angle_frame.pack(side="left", padx=16, pady=8)
        self._section_label(angle_frame, "GÓC KHỚP (IK → STEPPER)")

        cards_row = tk.Frame(angle_frame, bg=C["bg2"])
        cards_row.pack(side="top", anchor="w")

        self._angle_vars: dict = {}
        self._stepper_vars: dict = {}
        for i, (name, color) in enumerate([("θ₁", C["accent"]),
                                            ("θ₂", C["green"]),
                                            ("θ₃", C["yellow"])]):
            col = tk.Frame(cards_row, bg=C["bg3"],
                           padx=12, pady=8, relief="flat")
            col.pack(side="left", padx=6)
            tk.Label(col, text=name, fg=C["muted"], bg=C["bg3"],
                     font=("Courier", 10)).pack()
            av = tk.StringVar(value="0.00°")
            tk.Label(col, textvariable=av, fg=color, bg=C["bg3"],
                     font=("Courier", 18, "bold")).pack()
            sv = tk.StringVar(value="→ 0.00°")
            tk.Label(col, textvariable=sv, fg=C["yellow"], bg=C["bg3"],
                     font=("Courier", 9)).pack()
            self._angle_vars[i] = av
            self._stepper_vars[i] = sv

        # Trạng thái XYZ
        xyz_frame = tk.Frame(bot, bg=C["bg2"])
        xyz_frame.pack(side="left", padx=16, pady=8)
        self._section_label(xyz_frame, "END-EFFECTOR")
        self._xyz_vars: dict = {}
        for i, (lbl, key) in enumerate([("X", "x"), ("Y", "y"), ("Z", "z")]):
            row = tk.Frame(xyz_frame, bg=C["bg2"])
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"{lbl}:", fg=C["muted"], bg=C["bg2"],
                     font=("Courier", 10), width=3).pack(side="left")
            v = tk.StringVar(value="0.00 mm")
            tk.Label(row, textvariable=v, fg=C["accent"], bg=C["bg2"],
                     font=("Courier", 11, "bold")).pack(side="left")
            self._xyz_vars[key] = v
        self._xyz_vars["z"].set("-250.00 mm")

        ik_row = tk.Frame(xyz_frame, bg=C["bg2"])
        ik_row.pack(fill="x", pady=(6, 0))
        tk.Label(ik_row, text="IK:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 10), width=3).pack(side="left")
        self._ik_status = tk.Label(ik_row, text="OK",
                                   fg=C["green"], bg=C["bg2"],
                                   font=("Courier", 11, "bold"))
        self._ik_status.pack(side="left")

        # Serial log (phải)
        log_frame = tk.Frame(bot, bg=C["bg2"])
        log_frame.pack(side="right", fill="both", expand=True,
                       padx=8, pady=8)
        self._section_label(log_frame, "SERIAL LOG")
        self._log_text = tk.Text(log_frame, bg=C["bg"], fg=C["muted"],
                                 font=("Courier", 9), height=8,
                                 relief="flat", bd=0, state="disabled",
                                 wrap="word")
        self._log_text.pack(fill="both", expand=True)
        self._log_text.tag_config("ok",   foreground=C["green"])
        self._log_text.tag_config("err",  foreground=C["red"])
        self._log_text.tag_config("info", foreground=C["accent"])
        self._log_text.tag_config("warn", foreground=C["yellow"])

    # ══════════════════════════════════════════════════════════════════════════
    # PARAM PANELS
    # ══════════════════════════════════════════════════════════════════════════
    def _clear_params(self):
        for w in self._param_frame.winfo_children():
            w.destroy()
        self._params.clear()

    def _add_param_row(self, label: str, key: str, default: float,
                       unit: str = "mm", mn: float = 1, mx: float = 500):
        row = tk.Frame(self._param_frame, bg=C["bg3"],
                       pady=3, padx=6)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=f"{label}:", fg=C["muted"], bg=C["bg3"],
                 font=("Courier", 9), width=12, anchor="w").pack(side="left")
        var = tk.DoubleVar(value=default)
        sp = tk.Spinbox(row, from_=mn, to=mx, increment=1,
                        textvariable=var, width=7,
                        bg=C["bg"], fg=C["text"],
                        buttonbackground=C["bg3"],
                        font=("Courier", 10),
                        relief="flat", bd=1)
        sp.pack(side="left", padx=4)
        tk.Label(row, text=unit, fg=C["muted"], bg=C["bg3"],
                 font=("Courier", 9)).pack(side="left")
        self._params[key] = var

    def _build_params_circle(self):
        self._clear_params()
        self._add_param_row("Bán kính R",  "radius", 40,   "mm", 5,   120)
        self._add_param_row("Tâm X",       "cx",      0,   "mm", -80, 80)
        self._add_param_row("Tâm Y",       "cy",      0,   "mm", -80, 80)
        self._add_param_row("Độ cao Z",    "z",    -250,   "mm", -350,-150)
        self._add_param_row("Số điểm N",   "n",      72,   "pts", 8,  360)

    def _build_params_square(self):
        self._clear_params()
        self._add_param_row("Cạnh A",      "side",   60,   "mm", 10,  150)
        self._add_param_row("Tâm X",       "cx",      0,   "mm", -80, 80)
        self._add_param_row("Tâm Y",       "cy",      0,   "mm", -80, 80)
        self._add_param_row("Độ cao Z",    "z",    -250,   "mm", -350,-150)
        self._add_param_row("Pts/cạnh",    "n",      20,   "pts", 2,  100)

    def _build_params_triangle(self):
        self._clear_params()
        self._add_param_row("Cạnh A",      "side",   70,   "mm", 10,  150)
        self._add_param_row("Tâm X",       "cx",      0,   "mm", -80, 80)
        self._add_param_row("Tâm Y",       "cy",      0,   "mm", -80, 80)
        self._add_param_row("Độ cao Z",    "z",    -250,   "mm", -350,-150)
        self._add_param_row("Pts/cạnh",    "n",      20,   "pts", 2,  100)

    # ══════════════════════════════════════════════════════════════════════════
    # LOGIC CHỌN HÌNH
    # ══════════════════════════════════════════════════════════════════════════
    def _select_shape(self, shape: str, init: bool = False):
        self._shape_var.set(shape)
        for k, b in self._shape_btns.items():
            b.configure(
                bg=C["accent"] if k == shape else C["bg3"],
                fg=C["bg"]     if k == shape else C["text"],
            )
        if shape == "circle":
            self._build_params_circle()
        elif shape == "square":
            self._build_params_square()
        else:
            self._build_params_triangle()
        if not init:
            self._reset()

    # ══════════════════════════════════════════════════════════════════════════
    # SINH QUỸ ĐẠO
    # ══════════════════════════════════════════════════════════════════════════
    def _generate_trajectory(self) -> list:
        p = {k: v.get() for k, v in self._params.items()}
        shape = self._shape_var.get()
        try:
            if shape == "circle":
                return generate_circle(
                    radius=p["radius"], cx=p["cx"], cy=p["cy"],
                    z=p["z"], n_points=int(p["n"]))
            elif shape == "square":
                return generate_square(
                    side=p["side"], cx=p["cx"], cy=p["cy"],
                    z=p["z"], n_per_side=int(p["n"]))
            else:
                return generate_triangle(
                    side=p["side"], cx=p["cx"], cy=p["cy"],
                    z=p["z"], n_per_side=int(p["n"]))
        except Exception as e:
            messagebox.showerror("Lỗi tham số", str(e))
            return []

    # ══════════════════════════════════════════════════════════════════════════
    # CANVAS & VẼ
    # ══════════════════════════════════════════════════════════════════════════
    def _on_canvas_resize(self, event):
        self._draw_canvas()

    def _world_to_canvas(self, x: float, y: float):
        w = self._canvas.winfo_width()
        h = self._canvas.winfo_height()
        scale = min(w, h) * 0.004
        cx = w / 2 + x * scale
        cy = h / 2 - y * scale
        return cx, cy

    def _draw_canvas(self):
        self._canvas.delete("all")
        w = self._canvas.winfo_width()
        h = self._canvas.winfo_height()
        if w < 2 or h < 2:
            return

        # Lưới
        scale = min(w, h) * 0.004
        self._canvas.configure(bg=C["bg"])
        for g in range(-200, 201, 20):
            gx, _ = self._world_to_canvas(g, 0)
            self._canvas.create_line(gx, 0, gx, h, fill=C["bg2"], width=1)
            _, gy = self._world_to_canvas(0, g)
            self._canvas.create_line(0, gy, w, gy, fill=C["bg2"], width=1)

        # Trục chính
        ox, oy = self._world_to_canvas(0, 0)
        self._canvas.create_line(0, oy, w, oy, fill=C["border"], width=1)
        self._canvas.create_line(ox, 0, ox, h, fill=C["border"], width=1)
        self._canvas.create_text(w - 10, oy - 8, text="X",
                                  fill=C["muted"], font=("Courier", 9))
        self._canvas.create_text(ox + 8, 10, text="Y",
                                  fill=C["muted"], font=("Courier", 9))

        # Nhãn lưới
        for v in range(-100, 101, 50):
            if v == 0:
                continue
            gx, _ = self._world_to_canvas(v, 0)
            self._canvas.create_text(gx, oy + 10, text=str(v),
                                      fill=C["muted"], font=("Courier", 8))
            _, gy = self._world_to_canvas(0, v)
            self._canvas.create_text(ox + 18, gy, text=str(v),
                                      fill=C["muted"], font=("Courier", 8))

        # Quỹ đạo ghost
        if self._trajectory:
            pts_flat = []
            for p in self._trajectory:
                x, y = self._world_to_canvas(p[0], p[1])
                pts_flat.extend([x, y])
            if len(pts_flat) >= 4:
                self._canvas.create_line(*pts_flat, fill=C["bg3"],
                                          width=1, dash=(4, 4))

        # Đường đã vẽ
        if len(self._drawn_pts) >= 2:
            pts_flat = []
            for p in self._drawn_pts:
                x, y = self._world_to_canvas(p[0], p[1])
                pts_flat.extend([x, y])
            self._canvas.create_line(*pts_flat, fill=C["accent"],
                                      width=2, smooth=True)

        # Đầu bút hiện tại
        if self._drawn_pts:
            last = self._drawn_pts[-1]
            lx, ly = self._world_to_canvas(last[0], last[1])
            self._canvas.create_oval(lx - 5, ly - 5, lx + 5, ly + 5,
                                      fill=C["green"], outline="")

        # Gốc tọa độ
        self._canvas.create_oval(ox - 3, oy - 3, ox + 3, oy + 3,
                                  fill=C["accent"], outline="")

    # ══════════════════════════════════════════════════════════════════════════
    # ĐIỀU KHIỂN CHẠY
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_run(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self):
        traj = self._generate_trajectory()
        if not traj:
            return

        self._trajectory = traj
        self._drawn_pts  = []
        self._step_idx   = 0
        self._running    = True

        self._run_btn.configure(text="■  DỪNG", bg=C["red"])
        self._log("INFO", f"Bắt đầu vẽ {self._shape_var.get().upper()} — {len(traj)} điểm")
        self._draw_canvas()
        self._step()

    def _stop(self):
        self._running = False
        if self._after_id:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        if hasattr(self, "_run_btn"):
            self._run_btn.configure(text="▶  BẮT ĐẦU VẼ", bg=C["accent"])
        self._log("WARN", f"Dừng tại bước {self._step_idx}")

    def _step(self):
        if not self._running:
            return
        if self._step_idx >= len(self._trajectory):
            self._finish()
            return

        Px, Py, Pz = self._trajectory[self._step_idx]

        try:
            t1, t2, t3 = inverse_kinematics(Px, Py, Pz)

            # ── Áp dụng PID (nếu bật) ─────────────────────────────────────
            if self._pid.enabled:
                # Feedback = setpoint bước trước (mô phỏng hệ hở)
                # Khi có encoder thực → thay bằng giá trị đọc từ UART
                fb1, fb2, fb3 = self._fb_angles
                delay_s = max(0.01, (110 - self._speed_var.get() * 10) / 1000)
                u1, u2, u3 = self._pid.compute_all(
                    setpoints=(t1, t2, t3),
                    measurements=(fb1, fb2, fb3),
                    dt=delay_s,
                )
                # Cập nhật feedback cho bước tiếp theo
                self._fb_angles = [t1, t2, t3]

                # Hiển thị sai lệch
                e1, e2, e3 = t1 - fb1, t2 - fb2, t3 - fb3
                self._pid_err_var.set(
                    f"{e1:+.2f}° | {e2:+.2f}° | {e3:+.2f}°"
                )
            else:
                u1, u2, u3 = t1, t2, t3
                self._pid_err_var.set("0.00 | 0.00 | 0.00")

            self._update_angles((t1, t2, t3), (u1, u2, u3), Px, Py, Pz)
            self._drawn_pts.append((Px, Py))
            self._draw_canvas()

            if self._uart:
                self._uart.send_angles(u1, u2, u3)   # gửi góc đã bù PID

            total = len(self._trajectory)
            pct   = 100 * (self._step_idx + 1) / total
            self._progress_var.set(pct)
            self._progress_lbl.configure(
                text=f"{self._step_idx + 1} / {total}")

        except ValueError as e:
            self._log("ERR", f"IK fail @ ({Px:.1f},{Py:.1f},{Pz:.1f}): {e}")
            self._ik_status.configure(text="FAIL", fg=C["red"])

        self._step_idx += 1
        delay = max(10, 110 - self._speed_var.get() * 10)
        self._after_id = self.root.after(delay, self._step)

    def _finish(self):
        self._running = False
        self._run_btn.configure(text="▶  BẮT ĐẦU VẼ", bg=C["accent"])
        self._log("OK", f"Hoàn tất! Tổng {len(self._trajectory)} điểm")

    def _reset(self):
        self._stop()
        self._trajectory = []
        self._drawn_pts  = []
        self._step_idx   = 0
        self._fb_angles  = [0.0, 0.0, 0.0]
        self._pid.reset()   # đặt lại tích phân PID
        if hasattr(self, "_progress_var"):
            self._progress_var.set(0)
        if hasattr(self, "_progress_lbl"):
            self._progress_lbl.configure(text="0 / 0")
        if hasattr(self, "_ik_status"):
            self._ik_status.configure(text="OK", fg=C["green"])
        if hasattr(self, "_angle_vars"):
            for i in range(3):
                self._angle_vars[i].set("0.00°")
        if hasattr(self, "_stepper_vars"):
            for i in range(3):
                self._stepper_vars[i].set("→ 0.00°")
        if hasattr(self, "_xyz_vars"):
            self._xyz_vars["x"].set("0.00 mm")
            self._xyz_vars["y"].set("0.00 mm")
            self._xyz_vars["z"].set("-250.00 mm")
        if hasattr(self, "_pid_err_var"):
            self._pid_err_var.set("0.00 | 0.00 | 0.00")
        if hasattr(self, "_canvas"):
            self._draw_canvas()
        self._log("INFO", "Đặt lại hoàn tất (PID tích phân reset)")

    # ══════════════════════════════════════════════════════════════════════════
    # CẬP NHẬT GÓC + XYZ
    # ══════════════════════════════════════════════════════════════════════════
    def _update_angles(self, t_set, t_ctrl, Px, Py, Pz):
        s1, s2, s3 = t_set
        c1, c2, c3 = t_ctrl
        for i, (t, s) in enumerate(
            [(s1, c1 * self.GEAR_RATIO),
             (s2, c2 * self.GEAR_RATIO),
             (s3, c3 * self.GEAR_RATIO)]):
            self._angle_vars[i].set(f"{t:.2f}°")
            self._stepper_vars[i].set(f"→ {s:.2f}°")

        self._xyz_vars["x"].set(f"{Px:.2f} mm")
        self._xyz_vars["y"].set(f"{Py:.2f} mm")
        self._xyz_vars["z"].set(f"{Pz:.2f} mm")
        self._ik_status.configure(text="OK", fg=C["green"])

        if self._step_idx % 5 == 0:
            self._log(
                "OK",
                f"TX  T1:{c1*self.GEAR_RATIO:.2f}°  "
                f"T2:{c2*self.GEAR_RATIO:.2f}°  "
                f"T3:{c3*self.GEAR_RATIO:.2f}°"
            )

    # ══════════════════════════════════════════════════════════════════════════
    # SERIAL KẾT NỐI
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_connect(self):
        if self._uart and self._uart.is_connected:
            self._uart.disconnect()
            self._uart = None
            self._conn_label.configure(text="⬤  CHƯA KẾT NỐI", fg=C["red"])
            self._connect_btn.configure(text="⚡ KẾT NỐI", bg=C["green"])
            self._log("WARN", "Đã ngắt kết nối Serial")
        else:
            port = self._port_var.get()
            dry  = (port == "DRY-RUN" or port == "")
            self._uart = UARTComm(
                port=port if not dry else "",
                baud=int(self._baud_var.get()),
                gear_ratio=self.GEAR_RATIO,
                log_callback=lambda msg: self._log("INFO", msg),
                dry_run=dry,
            )
            ok = self._uart.connect()
            if ok:
                label = "DRY-RUN" if dry else port
                self._conn_label.configure(
                    text=f"⬤  KẾT NỐI: {label}", fg=C["green"])
                self._connect_btn.configure(
                    text="✖ NGẮT KẾT NỐI", bg=C["red"])
            else:
                self._uart = None

    def _send_home(self):
        if self._uart:
            self._uart.send_home()
        self._log("INFO", "Gửi lệnh HOME")

    # ══════════════════════════════════════════════════════════════════════════
    # LOG
    # ══════════════════════════════════════════════════════════════════════════
    def _log(self, level: str, msg: str):
        tag_map = {"OK": "ok", "ERR": "err", "INFO": "info",
                   "WARN": "warn", "SYS": "info"}
        tag = tag_map.get(level, "")
        ts  = time.strftime("%H:%M:%S")
        line = f"[{ts}][{level}] {msg}\n"
        if not hasattr(self, "_log_text"):
            print(line, end="")
            return
        self._log_text.configure(state="normal")
        self._log_text.insert("end", line, tag)
        lines = int(self._log_text.index("end-1c").split(".")[0])
        if lines > 200:
            self._log_text.delete("1.0", "10.0")
        self._log_text.see("end")
        self._log_text.configure(state="disabled")

    # ══════════════════════════════════════════════════════════════════════════
    # TIỆN ÍCH UI
    # ══════════════════════════════════════════════════════════════════════════
    def _section_label(self, parent, text: str, **pack_kwargs):
        tk.Label(parent, text=text, fg=C["muted"], bg=parent["bg"],
                 font=("Courier", 8, "bold")).pack(
                     anchor="w", pady=(6, 2), **pack_kwargs)

    def _sep(self, parent, **pack_kwargs):
        tk.Frame(parent, bg=C["border"], height=1).pack(
            fill="x", pady=6, **pack_kwargs)

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK PID
    # ══════════════════════════════════════════════════════════════════════════
    def _on_pid_toggle(self):
        """Bật/tắt PID từ checkbox trên GUI."""
        enabled = self._pid_enabled_var.get()
        self._pid.enabled = enabled
        self._pid.reset()          # reset tích phân mỗi khi toggle
        self._fb_angles = [0.0, 0.0, 0.0]
        state_str = "BẬT" if enabled else "TẮT"
        self._log("INFO", f"PID {state_str} — Kp={self._kp_var.get():.3f} "
                           f"Ki={self._ki_var.get():.3f} "
                           f"Kd={self._kd_var.get():.3f}")

    def _on_pid_gains_change(self, *_):
        """Cập nhật hệ số PID khi kéo slider."""
        self._pid.set_gains(
            Kp=self._kp_var.get(),
            Ki=self._ki_var.get(),
            Kd=self._kd_var.get(),
        )