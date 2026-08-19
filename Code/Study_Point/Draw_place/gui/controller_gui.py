import tkinter as tk
from tkinter import (ttk, messagebox, font as tkfont)
import time
from typing import Optional
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from kinematics.inverse_kinematics import inverse_kinematics

# Dòng 6 trong controller_gui.py
# Từ:
from trajectory.generator import generate_line
# Giữ nguyên, nhưng cần đảm bảo __init__.py không chặn.

# Tuy nhiên, __init__.py vẫn bị load trước, nên vẫn lỗi. Vì vậy bắt buộc phải sửa __init__.py.
from communication.uart_comm import UARTComm
import threading
import math

# ── Thử import thư viện camera ───────────────────────────────────────────────
try:
    import cv2
    from PIL import Image, ImageTk
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

C = {
    "bg": "#0d1117",
    "bg2": "#161b22",
    "bg3": "#21262d",
    "border": "#30363d",
    "accent": "#58a6ff",
    "green": "#3fb950",
    "yellow": "#d29922",
    "red": "#f85149",
    "text": "#e6edf3",
    "muted": "#8b949e",
}


class DeltaRobotGUI:
    """Cửa sổ chính điều khiển Robot Delta — vẽ đường thẳng + camera."""

    GEAR_RATIO = 3.0
    STEP_DELAY_MS = 10
    Z_LIFT = 0

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
        self._last_z: float = -250.0
        self._is_lifted: bool = False

        self._uart: Optional[UARTComm] = None

        # Camera
        self._cap = None
        self._camera_running = False
        self._camera_thread = None
        self._cam_index = 0
        self._camera_label_img = None

        self._log("SYS", "Delta Robot Controller (Line drawing mode) khởi động thành công")
        self._log("SYS", f"R=93 a=130 b=298 r=40 mm | Tỉ số đai u={self.GEAR_RATIO}")

        if not CV2_AVAILABLE:
            self._log("WARN", "opencv-python / Pillow chưa cài → pip install opencv-python pillow")

    # ── WINDOW SETUP ──────────────────────────────────────────────────────────
    def _setup_window(self):
        self.root.title("Delta Robot — Line Draw")
        self.root.configure(bg=C["bg"])
        self.root.state("zoomed")
        try:
            self.root.attributes("-zoomed", True)
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # BUILD UI
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        self._build_header()

        main = tk.Frame(self.root, bg=C["bg"])
        main.pack(fill="both", expand=True)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        self._build_left_panel(main)
        self._build_center_notebook(main)
        self._build_bottom_panel()

    # ── HEADER ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=C["bg2"], height=44)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)

        tk.Label(hdr, text="●", fg=C["green"], bg=C["bg2"],
                 font=("Courier", 12)).pack(side="left", padx=(12, 4), pady=8)
        tk.Label(hdr, text="DELTA ROBOT — LINE DRAW",
                 fg=C["accent"], bg=C["bg2"],
                 font=("Courier", 13, "bold")).pack(side="left", pady=8)

        self._conn_label = tk.Label(hdr, text="⬤  CHƯA KẾT NỐI",
                                    fg=C["red"], bg=C["bg2"],
                                    font=("Courier", 10, "bold"))
        self._conn_label.pack(side="right", padx=16)

        tk.Label(hdr, text=f"R=95 | a=130 | b=298 | r=40mm | u={self.GEAR_RATIO}",
                 fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9)).pack(side="right", padx=24)

    # ── CENTER NOTEBOOK ──────────────────────────────────────────────────────
    def _build_center_notebook(self, parent):
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Dark.TNotebook",
                        background=C["bg"],
                        borderwidth=0)
        style.configure("Dark.TNotebook.Tab",
                        background=C["bg3"],
                        foreground=C["muted"],
                        font=("Courier", 9, "bold"),
                        padding=[12, 4])
        style.map("Dark.TNotebook.Tab",
                  background=[("selected", C["bg2"])],
                  foreground=[("selected", C["accent"])])

        nb = ttk.Notebook(parent, style="Dark.TNotebook")
        nb.grid(row=0, column=1, sticky="nsew")

        # Tab quỹ đạo
        traj_tab = tk.Frame(nb, bg=C["bg"])
        nb.add(traj_tab, text="  📐  QUỸ ĐẠO  ")
        self._build_canvas_in(traj_tab)

        # Tab camera
        cam_tab = tk.Frame(nb, bg=C["bg"])
        nb.add(cam_tab, text="  📷  CAMERA  ")
        self._build_camera_tab(cam_tab)

        self._notebook = nb

    # ── CANVAS (tab quỹ đạo) ────────────────────────────────────────────────
    def _build_canvas_in(self, parent):
        self._canvas = tk.Canvas(parent, bg=C["bg"], highlightthickness=0)
        self._canvas.pack(fill="both", expand=True)
        self._canvas.bind("<Configure>", self._on_canvas_resize)

    # ── TAB CAMERA ────────────────────────────────────────────────────────────
    def _build_camera_tab(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        ctrl = tk.Frame(parent, bg=C["bg2"], height=42)
        ctrl.grid(row=0, column=0, sticky="ew")
        ctrl.pack_propagate(False)

        tk.Label(ctrl, text="Camera:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9)).pack(side="left", padx=(10, 4))

        self._cam_index_var = tk.IntVar(value=0)
        cam_spin = tk.Spinbox(ctrl, from_=0, to=9,
                              textvariable=self._cam_index_var,
                              width=4, bg=C["bg"], fg=C["text"],
                              buttonbackground=C["bg3"],
                              font=("Courier", 9), relief="flat", bd=1)
        cam_spin.pack(side="left", padx=4)

        self._cam_btn = tk.Button(ctrl, text="▶  BẬT CAMERA",
                                  bg=C["green"], fg=C["bg"],
                                  font=("Courier", 9, "bold"),
                                  relief="flat", bd=0, padx=10, pady=4,
                                  command=self._toggle_camera)
        self._cam_btn.pack(side="left", padx=8)

        self._cam_status = tk.Label(ctrl, text="● Tắt",
                                    fg=C["red"], bg=C["bg2"],
                                    font=("Courier", 9, "bold"))
        self._cam_status.pack(side="left", padx=8)

        tk.Label(ctrl,
                 text="(chỉ để quan sát — không ảnh hưởng điều khiển)",
                 fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 8)).pack(side="right", padx=12)

        cam_container = tk.Frame(parent, bg=C["bg"])
        cam_container.grid(row=1, column=0, sticky="nsew")
        cam_container.columnconfigure(0, weight=1)
        cam_container.rowconfigure(0, weight=1)

        self._cam_label = tk.Label(cam_container, bg=C["bg"],
                                   text="",
                                   fg=C["muted"],
                                   font=("Courier", 11))
        self._cam_label.grid(row=0, column=0, sticky="nsew")

        # Overlay thông tin
        self._cam_overlay = tk.Frame(cam_container, bg=C["bg2"],
                                     padx=10, pady=6)
        self._cam_overlay.place(relx=0.01, rely=0.01, anchor="nw")

        tk.Label(self._cam_overlay, text="LIVE STATUS",
                 fg=C["accent"], bg=C["bg2"],
                 font=("Courier", 8, "bold")).pack(anchor="w")

        self._ov_xyz = tk.Label(self._cam_overlay,
                                text="X: —   Y: —   Z: —",
                                fg=C["text"], bg=C["bg2"],
                                font=("Courier", 9))
        self._ov_xyz.pack(anchor="w")

        self._ov_angles = tk.Label(self._cam_overlay,
                                   text="θ₁: —   θ₂: —   θ₃: —",
                                   fg=C["green"], bg=C["bg2"],
                                   font=("Courier", 9))
        self._ov_angles.pack(anchor="w")

        self._ov_progress = tk.Label(self._cam_overlay,
                                     text="Tiến độ: —",
                                     fg=C["yellow"], bg=C["bg2"],
                                     font=("Courier", 9))
        self._ov_progress.pack(anchor="w")

        # Placeholder
        self._cam_placeholder = tk.Label(
            cam_container,
            text=(
                "📷\n\n"
                "Chưa bật camera\n\n"
                "Chọn chỉ số camera (thường là 0)\n"
                "rồi nhấn  ▶ BẬT CAMERA\n\n"
                "Camera chỉ dùng để quan sát robot vẽ thực tế.\n"
                "Mọi tham số điều khiển không bị ảnh hưởng."
            ),
            fg=C["muted"], bg=C["bg"],
            font=("Courier", 10),
            justify="center"
        )
        self._cam_placeholder.place(relx=0.5, rely=0.5, anchor="center")

    # ══════════════════════════════════════════════════════════════════════════
    # CAMERA LOGIC (không đổi)
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_camera(self):
        if self._camera_running:
            self._stop_camera()
        else:
            self._start_camera()

    def _start_camera(self):
        if not CV2_AVAILABLE:
            messagebox.showwarning(
                "Thiếu thư viện",
                "Cần cài:\n  pip install opencv-python pillow\nrồi khởi động lại."
            )
            return

        idx = self._cam_index_var.get()
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            messagebox.showerror("Camera", f"Không mở được camera {idx}.\n"
                                           "Thử chỉ số khác (0, 1, 2…)")
            return

        self._cap = cap
        self._cam_index = idx
        self._camera_running = True

        self._cam_btn.configure(text="■  TẮT CAMERA", bg=C["red"])
        self._cam_status.configure(text="● Đang chạy", fg=C["green"])
        self._cam_placeholder.place_forget()

        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True)
        self._camera_thread.start()

        self._log("INFO", f"Bật camera {idx}")

    def _stop_camera(self):
        self._camera_running = False
        if self._cap:
            if self._camera_thread and self._camera_thread.is_alive():
                self._camera_thread.join(timeout=1.0)
            self._cap.release()
            self._cap = None

        self._cam_btn.configure(text="▶  BẬT CAMERA", bg=C["green"])
        self._cam_status.configure(text="● Tắt", fg=C["red"])
        self._cam_label.configure(image="")
        self._camera_label_img = None
        self._cam_placeholder.place(relx=0.5, rely=0.5, anchor="center")
        self._log("INFO", "Tắt camera")

    def _camera_loop(self):
        while self._camera_running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if not ret:
                break
            self.root.after(0, self._update_camera_frame, frame)
            time.sleep(0.033)
        if self._camera_running:
            self.root.after(0, self._on_camera_disconnected)

    def _update_camera_frame(self, frame):
        if not self._camera_running:
            return
        lw = self._cam_label.winfo_width()
        lh = self._cam_label.winfo_height()
        if lw < 10 or lh < 10:
            return
        fh, fw = frame.shape[:2]
        ratio = min(lw / fw, lh / fh)
        nw, nh = int(fw * ratio), int(fh * ratio)
        if nw < 2 or nh < 2:
            return
        frame_resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(image=img)
        self._cam_label.configure(image=photo)
        self._camera_label_img = photo
        self._update_cam_overlay()

    def _update_cam_overlay(self):
        try:
            x = self._xyz_vars["x"].get()
            y = self._xyz_vars["y"].get()
            z = self._xyz_vars["z"].get()
            self._ov_xyz.configure(text=f"X: {x}   Y: {y}   Z: {z}")
            a1 = self._angle_vars[0].get()
            a2 = self._angle_vars[1].get()
            a3 = self._angle_vars[2].get()
            self._ov_angles.configure(text=f"θ₁: {a1}   θ₂: {a2}   θ₃: {a3}")
            total = len(self._trajectory)
            if total > 0:
                pct = 100 * self._step_idx / total
                self._ov_progress.configure(text=f"Tiến độ: {self._step_idx}/{total}  ({pct:.0f}%)")
            else:
                self._ov_progress.configure(text="Tiến độ: —")
        except Exception:
            pass

    def _on_camera_disconnected(self):
        self._log("ERR", "Camera bị ngắt kết nối")
        self._stop_camera()

    # ══════════════════════════════════════════════════════════════════════════
    # LEFT PANEL (đã sửa để chỉ có tham số đường thẳng)
    # ══════════════════════════════════════════════════════════════════════════
    def _build_left_panel(self, parent):
        lf = tk.Frame(parent, bg=C["bg2"], width=300)
        lf.grid(row=0, column=0, sticky="nsew")
        lf.pack_propagate(False)
        lf.grid_propagate(False)

        scroll_canvas = tk.Canvas(lf, bg=C["bg2"], highlightthickness=0,
                                  bd=0, width=284)
        scrollbar = ttk.Scrollbar(lf, orient="vertical",
                                  command=scroll_canvas.yview)
        scroll_canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        scroll_canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(scroll_canvas, bg=C["bg2"])
        inner_id = scroll_canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(event):
            scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all"))

        def _on_canvas_configure(event):
            scroll_canvas.itemconfig(inner_id, width=event.width)

        inner.bind("<Configure>", _on_inner_configure)
        scroll_canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _on_mousewheel_linux(event):
            if event.num == 4:
                scroll_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                scroll_canvas.yview_scroll(1, "units")

        scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        scroll_canvas.bind_all("<Button-4>", _on_mousewheel_linux)
        scroll_canvas.bind_all("<Button-5>", _on_mousewheel_linux)

        self._fill_left_panel(inner)

    def _fill_left_panel(self, inner):
        pad = dict(padx=10)

        # ── THAM SỐ ĐƯỜNG THẲNG ──────────────────────────────────────────────
        self._section_label(inner, "THAM SỐ ĐƯỜNG THẲNG", **pad)
        self._param_frame = tk.Frame(inner, bg=C["bg2"])
        self._param_frame.pack(fill="x", **pad)
        self._params: dict[str, tk.DoubleVar] = {}
        self._build_params_line()   # khởi tạo

        self._sep(inner)

        # ── TỐC ĐỘ ─────────────────────────────────────────────────────────────
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

        # ── SERIAL ─────────────────────────────────────────────────────────────
        self._section_label(inner, "SERIAL / UART", **pad)
        port_row = tk.Frame(inner, bg=C["bg2"])
        port_row.pack(fill="x", pady=2, **pad)
        tk.Label(port_row, text="Port:", fg=C["muted"], bg=C["bg2"],
                 font=("Courier", 9), width=6).pack(side="left")
        _detected = UARTComm.list_ports()
        _fixed = ["COM4", "COM6", "COM5", "/dev/ttyUSB0", "DRY-RUN"]
        ports = list(dict.fromkeys(_detected + _fixed))
        self._port_var = tk.StringVar(value="COM6")
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
                     values=["9600", "115200"],
                     width=14, font=("Courier", 9)).pack(side="left", padx=4)

        self._connect_btn = tk.Button(inner, text="⚡ KẾT NỐI",
                                      bg=C["green"], fg=C["bg"],
                                      font=("Courier", 10, "bold"),
                                      relief="flat", bd=0, pady=5,
                                      command=self._toggle_connect)
        self._connect_btn.pack(fill="x", pady=4, **pad)

        self._sep(inner)

        # ── ĐIỀU KHIỂN ─────────────────────────────────────────────────────────
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

        self._sep(inner)

        # ── PROGRESS ──────────────────────────────────────────────────────────
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

        tk.Frame(inner, bg=C["bg2"], height=12).pack()

    # ── PARAMETERS LINE ──────────────────────────────────────────────────────
    def _clear_params(self):
        for w in self._param_frame.winfo_children():
            w.destroy()
        self._params.clear()

    def _add_param_row(self, label: str, key: str, default: float,
                       unit: str = "mm", mn: float = 1, mx: float = 500):
        row = tk.Frame(self._param_frame, bg=C["bg3"], pady=3, padx=6)
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

    def _build_params_line(self):
        self._clear_params()
        self._add_param_row("Điểm đầu X", "start_x", -50, "mm", -100, 100)
        self._add_param_row("Điểm đầu Y", "start_y", -50, "mm", -100, 100)
        self._add_param_row("Điểm cuối X", "end_x", 50, "mm", -100, 100)
        self._add_param_row("Điểm cuối Y", "end_y", 50, "mm", -100, 100)
        self._add_param_row("Độ cao Z", "z", -250, "mm", -350, -150)
        self._add_param_row("Số điểm N", "n", 100, "pts", 2, 200)

    # ══════════════════════════════════════════════════════════════════════════
    # BOTTOM PANEL (không đổi)
    # ══════════════════════════════════════════════════════════════════════════
    def _build_bottom_panel(self):
        bot = tk.Frame(self.root, bg=C["bg2"], height=180)
        bot.pack(fill="x")
        bot.pack_propagate(False)

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
            col = tk.Frame(cards_row, bg=C["bg3"], padx=12, pady=8, relief="flat")
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

        log_frame = tk.Frame(bot, bg=C["bg2"])
        log_frame.pack(side="right", fill="both", expand=True, padx=8, pady=8)
        self._section_label(log_frame, "SERIAL LOG")
        self._log_text = tk.Text(log_frame, bg=C["bg"], fg=C["muted"],
                                 font=("Courier", 9), height=8,
                                 relief="flat", bd=0, state="disabled",
                                 wrap="word")
        self._log_text.pack(fill="both", expand=True)
        self._log_text.tag_config("ok", foreground=C["green"])
        self._log_text.tag_config("err", foreground=C["red"])
        self._log_text.tag_config("info", foreground=C["accent"])
        self._log_text.tag_config("warn", foreground=C["yellow"])

    # ══════════════════════════════════════════════════════════════════════════
    # LOGIC SINH QUỸ ĐẠO (ĐƯỜNG THẲNG)
    # ══════════════════════════════════════════════════════════════════════════
    def _generate_trajectory(self) -> list:
        p = {k: v.get() for k, v in self._params.items()}
        try:
            start = (p["start_x"], p["start_y"], p["z"])
            end   = (p["end_x"],   p["end_y"],   p["z"])
            return generate_line(
                start=start,
                end=end,
                n_points=int(p["n"]),
                include_endpoint=True
            )
        except Exception as e:
            messagebox.showerror("Lỗi tham số", str(e))
            return []

    # ══════════════════════════════════════════════════════════════════════════
    # CANVAS & VẼ (không đổi)
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

        scale = min(w, h) * 0.004
        self._canvas.configure(bg=C["bg"])
        for g in range(-200, 201, 20):
            gx, _ = self._world_to_canvas(g, 0)
            self._canvas.create_line(gx, 0, gx, h, fill=C["bg2"], width=1)
            _, gy = self._world_to_canvas(0, g)
            self._canvas.create_line(0, gy, w, gy, fill=C["bg2"], width=1)

        ox, oy = self._world_to_canvas(0, 0)
        self._canvas.create_line(0, oy, w, oy, fill=C["border"], width=1)
        self._canvas.create_line(ox, 0, ox, h, fill=C["border"], width=1)
        self._canvas.create_text(w - 10, oy - 8, text="X",
                                 fill=C["muted"], font=("Courier", 9))
        self._canvas.create_text(ox + 8, 10, text="Y",
                                 fill=C["muted"], font=("Courier", 9))

        for v in range(-100, 101, 50):
            if v == 0:
                continue
            gx, _ = self._world_to_canvas(v, 0)
            self._canvas.create_text(gx, oy + 10, text=str(v),
                                     fill=C["muted"], font=("Courier", 8))
            _, gy = self._world_to_canvas(0, v)
            self._canvas.create_text(ox + 18, gy, text=str(v),
                                     fill=C["muted"], font=("Courier", 8))

        if self._trajectory:
            pts_flat = []
            for p in self._trajectory:
                x, y = self._world_to_canvas(p[0], p[1])
                pts_flat.extend([x, y])
            if len(pts_flat) >= 4:
                self._canvas.create_line(*pts_flat, fill=C["bg3"],
                                         width=1, dash=(4, 4))

        if len(self._drawn_pts) >= 2:
            pts_flat = []
            for p in self._drawn_pts:
                x, y = self._world_to_canvas(p[0], p[1])
                pts_flat.extend([x, y])
            self._canvas.create_line(*pts_flat, fill=C["accent"],
                                     width=2, smooth=True)

        if self._drawn_pts:
            last = self._drawn_pts[-1]
            lx, ly = self._world_to_canvas(last[0], last[1])
            self._canvas.create_oval(lx - 5, ly - 5, lx + 5, ly + 5,
                                     fill=C["green"], outline="")

        self._canvas.create_oval(ox - 3, oy - 3, ox + 3, oy + 3,
                                 fill=C["accent"], outline="")

    # ══════════════════════════════════════════════════════════════════════════
    # ĐIỀU KHIỂN CHẠY (không đổi)
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_run(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self):
        if self._is_lifted:
            self._lower_z_before_draw()
            return

        traj = self._generate_trajectory()
        if not traj:
            return

        self._trajectory = traj
        self._drawn_pts = []
        self._step_idx = 0
        self._running = True

        self._run_btn.configure(text="■  DỪNG", bg=C["red"])
        self._log("INFO", f"Bắt đầu vẽ đường thẳng — {len(traj)} điểm")
        self._draw_canvas()
        self._step()

    def _lower_z_before_draw(self):
        if not self._trajectory:
            self._log("ERR", "Không có quỹ đạo để hạ Z")
            self._is_lifted = False
            return

        first_point = self._trajectory[0]
        if len(first_point) >= 3:
            Px, Py, Pz = first_point[0], first_point[1], first_point[2]
            try:
                t1, t2, t3 = inverse_kinematics(Px, Py, Pz)
                u1, u2, u3 = t1, t2, t3
                self._update_angles((t1, t2, t3), (u1, u2, u3), Px, Py, Pz)
                if self._uart:
                    self._uart.send_angles(u1, u2, u3)
                self._is_lifted = False
                self._log("INFO", f"Hạ Z xuống {Pz:.1f}mm, bắt đầu vẽ")
                self._running = True
                self._run_btn.configure(text="■  DỪNG", bg=C["red"])
                self._step()
            except ValueError as e:
                self._log("ERR", f"Không thể hạ Z: {e}")
                self._is_lifted = False
                self._run_btn.configure(text="▶  BẮT ĐẦU VẼ", bg=C["accent"])

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
        self._last_z = Pz

        try:
            t1, t2, t3 = inverse_kinematics(Px, Py, Pz)
            u1, u2, u3 = t1, t2, t3
            self._update_angles((t1, t2, t3), (u1, u2, u3), Px, Py, Pz)
            self._drawn_pts.append((Px, Py))
            self._draw_canvas()

            if self._uart:
                self._uart.send_angles(u1, u2, u3)

            total = len(self._trajectory)
            pct = 100 * (self._step_idx + 1) / total
            self._progress_var.set(pct)
            self._progress_lbl.configure(text=f"{self._step_idx + 1} / {total}")

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
        self._lift_z_after_finish()

    def _lift_z_after_finish(self):
        if not self._drawn_pts:
            return
        last_point = self._drawn_pts[-1]
        if len(last_point) >= 2:
            lift_z = self._last_z - self.Z_LIFT
            try:
                t1, t2, t3 = inverse_kinematics(last_point[0], last_point[1], lift_z)
                u1, u2, u3 = t1, t2, t3
                self._update_angles((t1, t2, t3), (u1, u2, u3),
                                    last_point[0], last_point[1], lift_z)
                if self._uart:
                    self._uart.send_angles(u1, u2, u3)
                self._is_lifted = True
                self._log("INFO", f"Nâng Z lên {lift_z:.1f}mm (giảm {self.Z_LIFT}mm)")
            except ValueError as e:
                self._log("ERR", f"Không thể nâng Z: {e}")

    def _reset(self):
        self._stop()
        self._trajectory = []
        self._drawn_pts = []
        self._step_idx = 0
        self._is_lifted = False
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
        if hasattr(self, "_canvas"):
            self._draw_canvas()
        self._log("INFO", "Đặt lại hoàn tất")

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
                f"TX  T1:{c1 * self.GEAR_RATIO:.2f}°  "
                f"T2:{c2 * self.GEAR_RATIO:.2f}°  "
                f"T3:{c3 * self.GEAR_RATIO:.2f}°"
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
            dry = (port == "DRY-RUN" or port == "")
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
        ts = time.strftime("%H:%M:%S")
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

    def on_closing(self):
        if self._camera_running:
            self._stop_camera()
        self.root.destroy()


# ==================== CHẠY ỨNG DỤNG ====================
if __name__ == "__main__":
    root = tk.Tk()
    app = DeltaRobotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()