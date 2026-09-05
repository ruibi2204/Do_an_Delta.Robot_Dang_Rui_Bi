# dynamic_run4dof_window.py
import contextlib

from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QGroupBox,
    QDoubleSpinBox, QTextEdit, QSizePolicy, QMessageBox,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, cv2_to_qpixmap,
    save_config, home_and_wait, TURNTABLE_MAX_RPM,
)


# ============================================================
# LUỒNG CAMERA CHO BÀI TOÁN ĐỘNG BẬC 4
# Import camera_run4dof ĐẶT LAZY bên trong run() để cửa sổ vẫn
# mở được bình thường ngay cả khi camera_run4dof.py CHƯA tồn tại
# (chỉ báo lỗi khi người dùng bấm nút "Mở Camera").
#
# YÊU CẦU CHỮ KÝ HÀM khi bạn viết camera_run4dof.py:
#
#   def run_camera_run4dof_stream(camera_index, calib_path, is_running):
#       """Generator: mỗi lần yield 1 tuple (frame_da_ve_BGR, items, fps).
#       is_running() trả về False khi cần dừng vòng lặp.
#       Mỗi item trong `items` LÝ TƯỞNG là dict có 'type' == 'object' hoặc
#       'hole', cùng x_mm/y_mm/angle_deg/slot_id... (giống Camera_4dof.py).
#
#       LƯU Ý QUAN TRỌNG (đã xác nhận bằng thực nghiệm): bản camera_run4dof.py
#       hiện tại KHÔNG gắn field "type" vào item (kế thừa đúng hành vi của
#       detect_objects()/detect_frame_holes() trong Camera_4dof.py, vốn cũng
#       không tự gắn "type"). Vì vậy phía dưới (_split_objects_holes) KHÔNG
#       được lọc chỉ dựa vào item.get('type') - phải có phương án dự phòng
#       dựa trên 'slot_id' (xem chi tiết ngay dưới _on_frame)."""
#       ...
#       yield display_frame, items, fps
# ============================================================
class CameraRun4DofThread(QThread):
    frame_ready = Signal(object, list, float)
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index, calib_path):
        super().__init__()
        self.camera_index = camera_index
        self.calib_path = calib_path
        self._running = False

    def run(self):
        try:
            from vision.camera_run4dof import run_camera_run4dof_stream
        except ImportError as e:
            self.error.emit(
                "Chưa có file camera_run4dof.py (hoặc thiếu hàm "
                f"run_camera_run4dof_stream). Chi tiết: {e}"
            )
            self.stopped.emit()
            return

        self._running = True
        try:
            for frame, items, fps in run_camera_run4dof_stream(
                camera_index=self.camera_index,
                calib_path=self.calib_path,
                is_running=lambda: self._running,
            ):
                if not self._running:
                    break
                self.frame_ready.emit(frame, items, fps)
        except Exception as e:
            self.error.emit(str(e))
        finally:
            self.stopped.emit()

    def stop(self):
        self._running = False
        self.wait()


# ============================================================
# CỬA SỔ: BÀI TOÁN ĐỘNG BẬC 4
# ============================================================
class DynamicRun4DofWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "⚙ BÀI TOÁN ĐỘNG BẬC 4")
        self._camera_thread = None

        # Trạng thái vòng lặp gắp-thả tự động (giống BÀI TOÁN BẬC 4 TĨNH,
        # nhưng để RIÊNG trên self - không dùng chung ctx.dynamic_running /
        # ctx.dynamic_busy của dynamic_window.py để 2 cửa sổ không đụng nhau).
        self.running = False
        self.cycle_busy = False
        self.latest_objects = []
        self.latest_holes = []

        # ---- Trạng thái 2 PHA: PICK (gắp vật, chưa cần khung) và PLACE
        # (đang giữ vật tại Home, chờ thấy khung mới thả) ----
        # TRƯỚC ĐÂY: 1 chu trình yêu cầu THẤY ĐỦ CẢ vật lẫn khung CÙNG LÚC
        # mới gọi pick_and_place_dof4() (bản gộp). Hậu quả thực tế (đã xác
        # nhận qua ảnh chụp màn hình): khi vật đã vào ROI và có tọa độ dự
        # đoán rõ ràng nhưng khung CHƯA xuất hiện ("Dang quet khung..."),
        # latest_holes rỗng -> toàn bộ điều kiện `if not objects or not
        # holes: return` chặn lại -> KHÔNG có lệnh pick nào được gửi đi,
        # dù vật đã sẵn sàng để gắp từ lâu.
        # GIỜ: tách 2 pha dùng đúng pick_dof4()/place_dof4() đã có sẵn
        # trong move_run4dof.py (được viết ra chính là để phục vụ ca này) -
        # thấy VẬT là gắp ngay (không chờ khung), giữ tại Home, rồi khi nào
        # thấy KHUNG mới thả.
        self.holding_object = False
        self._held_object_angle = None

        # ==================================================================
        # ĐẾM CHU TRÌNH ĐỂ TỰ ĐỘNG HIỆU CHỈNH LẠI ĐỘ CHÍNH XÁC (THÊM MỚI)
        # Sau mỗi RECAL_EVERY_N_CYCLES chu trình gắp-thả THÀNH CÔNG liên
        # tiếp, tự động tạm dừng vòng lặp, cho robot về HOME VẬT LÝ (chạm
        # công tắc hành trình - y hệt quy trình của nút STOP/home_and_wait)
        # để triệt tiêu sai số cộng dồn của động cơ bước, sau đó tự động về
        # lại HOME ĐÃ LƯU (ctx.home_pos - y hệt lúc bấm START) rồi mới cho
        # vòng lặp PICK/PLACE tiếp tục bình thường.
        # ==================================================================
        self.RECAL_EVERY_N_CYCLES = 3
        self.completed_cycles = 0
        self.is_recalibrating = False

        self._build_ui()

    def showEvent(self, event):
        super().showEvent(event)
        self._refresh_turn_label()
        self.btn_start.setEnabled(not self.running)
        self.btn_stop.setEnabled(self.running)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        # ---------- Cột video ----------
        video_col = QVBoxLayout()
        self.video_label = QLabel("Camera chưa mở")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(720, 480)
        self.video_label.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; "
            "border-radius:10px; color:#888888;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 5)

        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#333333; font-weight:700;")
        video_col.addWidget(self.lbl_fps)

        cam_btn_row = QHBoxLayout()
        self.btn_cam_open = QPushButton("▶ MỞ CAMERA (RUN 4DOF)")
        self.btn_cam_open.setObjectName("dynStartBtn")
        self.btn_cam_open.clicked.connect(self.on_open_camera)
        self.btn_cam_close = QPushButton("■ ĐÓNG CAMERA")
        self.btn_cam_close.setObjectName("dynStopBtn")
        self.btn_cam_close.clicked.connect(self.on_close_camera)
        self.btn_cam_close.setEnabled(False)
        cam_btn_row.addWidget(self.btn_cam_open)
        cam_btn_row.addWidget(self.btn_cam_close)
        video_col.addLayout(cam_btn_row)

        # ---------- Nút START / STOP vòng lặp gắp-thả tự động ----------
        start_stop_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ START")
        self.btn_start.setObjectName("dynStartBtn")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("■ STOP")
        self.btn_stop.setObjectName("dynStopBtn")
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)
        start_stop_row.addWidget(self.btn_start)
        start_stop_row.addWidget(self.btn_stop)
        video_col.addLayout(start_stop_row)

        # ---------- Nhãn trạng thái pha PICK / PLACE ----------
        self.lbl_phase = QLabel("Trạng thái: (chưa chạy)")
        self.lbl_phase.setStyleSheet("color:#555555; font-weight:700;")
        video_col.addWidget(self.lbl_phase)

        # ---------- Cột điều khiển ----------
        ctrl_col = QVBoxLayout()

        # ---------- Khung Z PICK & OFFSET TỌA ĐỘ ----------
        coord_box = QGroupBox("Z PICK && OFFSET TỌA ĐỘ")
        coord_box.setObjectName("compactBox")
        coord_layout = QVBoxLayout()
        coord_layout.setSpacing(6)

        zrow = QHBoxLayout()
        zrow.addWidget(QLabel("Z pick:"))
        self.spin_z_pick = QDoubleSpinBox()
        self.spin_z_pick.setRange(0, 500)
        self.spin_z_pick.setSingleStep(1.0)
        self.spin_z_pick.setValue(float(self.ctx.cfg.get("run4dof_z_pick", 340.0)))
        self.spin_z_pick.valueChanged.connect(self._on_zpick_changed)
        zrow.addWidget(self.spin_z_pick, 1)
        coord_layout.addLayout(zrow)

        offset_row = QHBoxLayout()
        offset_row.addWidget(QLabel("Offset X:"))
        self.spin_offset_x = QDoubleSpinBox()
        self.spin_offset_x.setRange(-100, 100)
        self.spin_offset_x.setSingleStep(0.5)
        self.spin_offset_x.setValue(float(self.ctx.cfg.get("run4dof_offset_x", 0.0)))
        self.spin_offset_x.valueChanged.connect(self._on_offset_x_changed)
        offset_row.addWidget(self.spin_offset_x, 1)

        offset_row.addWidget(QLabel("Y:"))
        self.spin_offset_y = QDoubleSpinBox()
        self.spin_offset_y.setRange(-100, 100)
        self.spin_offset_y.setSingleStep(0.5)
        self.spin_offset_y.setValue(float(self.ctx.cfg.get("run4dof_offset_y", 0.0)))
        self.spin_offset_y.valueChanged.connect(self._on_offset_y_changed)
        offset_row.addWidget(self.spin_offset_y, 1)
        coord_layout.addLayout(offset_row)

        coord_box.setLayout(coord_layout)
        ctrl_col.addWidget(coord_box)

        # ---------- (THÊM MỚI) Khung HIỆU CHỈNH ĐỊNH KỲ ----------
        # Cho phép người vận hành xem/điều chỉnh trực tiếp trên giao diện
        # sau bao nhiêu chu trình thì tự động về Home để hiệu chỉnh lại độ
        # chính xác, thay vì phải sửa code.
        recal_box = QGroupBox("HIỆU CHỈNH ĐỊNH KỲ (TỰ VỀ HOME)")
        recal_box.setObjectName("compactBox")
        recal_layout = QVBoxLayout()
        recal_layout.setSpacing(6)

        recal_row = QHBoxLayout()
        recal_row.addWidget(QLabel("Sau mỗi (chu trình):"))
        self.spin_recal_n = QDoubleSpinBox()
        self.spin_recal_n.setDecimals(0)
        self.spin_recal_n.setRange(1, 999)
        self.spin_recal_n.setSingleStep(1.0)
        self.spin_recal_n.setValue(
            float(self.ctx.cfg.get("run4dof_recal_every_n", self.RECAL_EVERY_N_CYCLES))
        )
        self.spin_recal_n.valueChanged.connect(self._on_recal_n_changed)
        recal_row.addWidget(self.spin_recal_n, 1)
        recal_layout.addLayout(recal_row)

        self.lbl_recal_progress = QLabel(self._compute_recal_progress_text())
        self.lbl_recal_progress.setStyleSheet("color:#0078d4; font-weight:700;")
        recal_layout.addWidget(self.lbl_recal_progress)

        recal_box.setLayout(recal_layout)
        ctrl_col.addWidget(recal_box)

        turn_box = QGroupBox(f"BÀN XOAY (động cơ {TURNTABLE_MAX_RPM:.0f} v/p @ PWM=255)")
        turn_box.setObjectName("compactBox")
        turn_layout = QVBoxLayout()
        turn_layout.setSpacing(6)

        rpm_row = QHBoxLayout()
        rpm_row.addWidget(QLabel("RPM mong muốn:"))
        self.spin_rpm = QDoubleSpinBox()
        self.spin_rpm.setRange(0.0, TURNTABLE_MAX_RPM)
        self.spin_rpm.setSingleStep(1.0)
        self.spin_rpm.setValue(float(self.ctx.cfg.get("run4dof_rpm", 30.0)))
        self.spin_rpm.valueChanged.connect(self._on_rpm_changed)
        rpm_row.addWidget(self.spin_rpm, 1)
        turn_layout.addLayout(rpm_row)

        self.lbl_pwm = QLabel(self._compute_pwm_text(self.spin_rpm.value()))
        self.lbl_pwm.setStyleSheet("color:#0078d4; font-weight:800; font-size:11pt;")
        turn_layout.addWidget(self.lbl_pwm)

        onoff_row = QHBoxLayout()
        self.lbl_turn_state = QLabel("● Đang TẮT")
        self.lbl_turn_state.setObjectName("deviceStateOff")
        onoff_row.addWidget(self.lbl_turn_state, 1)
        self.btn_turn_on = QPushButton("▶ BẬT")
        self.btn_turn_on.setObjectName("deviceOnBtn")
        self.btn_turn_on.clicked.connect(self.on_turn_on)
        self.btn_turn_off = QPushButton("■ TẮT")
        self.btn_turn_off.setObjectName("deviceOffBtn")
        self.btn_turn_off.clicked.connect(self.on_turn_off)
        onoff_row.addWidget(self.btn_turn_on)
        onoff_row.addWidget(self.btn_turn_off)
        turn_layout.addLayout(onoff_row)

        turn_box.setLayout(turn_layout)
        ctrl_col.addWidget(turn_box)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        ctrl_col.addWidget(self.log_box, 3)

        root.addLayout(video_col, 3)
        root.addLayout(ctrl_col, 2)

    # ---------------- Z pick & Offset tọa độ ----------------
    def _on_zpick_changed(self, val):
        self.ctx.cfg["run4dof_z_pick"] = val
        save_config(self.ctx.cfg)

    def _on_offset_x_changed(self, val):
        self.ctx.cfg["run4dof_offset_x"] = val
        save_config(self.ctx.cfg)

    def _on_offset_y_changed(self, val):
        self.ctx.cfg["run4dof_offset_y"] = val
        save_config(self.ctx.cfg)

    # ---------------- (THÊM MỚI) Hiệu chỉnh định kỳ - cấu hình số chu trình ----------------
    def _compute_recal_progress_text(self):
        return f"Đã hoàn tất: {self.completed_cycles}/{self.RECAL_EVERY_N_CYCLES} chu trình"

    def _refresh_recal_progress_label(self):
        self.lbl_recal_progress.setText(self._compute_recal_progress_text())

    def _on_recal_n_changed(self, val):
        n = max(1, int(val))
        self.RECAL_EVERY_N_CYCLES = n
        self.ctx.cfg["run4dof_recal_every_n"] = n
        save_config(self.ctx.cfg)
        self._refresh_recal_progress_label()

    # ---------------- Bàn xoay ----------------
    def _compute_pwm_text(self, rpm):
        return f"→ PWM = {self._rpm_to_pwm(rpm)} / 255"

    @staticmethod
    def _rpm_to_pwm(rpm):
        rpm = max(0.0, min(TURNTABLE_MAX_RPM, float(rpm)))
        return int(round(rpm / TURNTABLE_MAX_RPM * 255))

    def _on_rpm_changed(self, val):
        self.ctx.cfg["run4dof_rpm"] = val
        save_config(self.ctx.cfg)
        self.lbl_pwm.setText(self._compute_pwm_text(val))

    def on_turn_on(self):
        if not self.ctx.pneu_uart.is_connected:
            QMessageBox.warning(
                self, "Chưa kết nối",
                "Hãy kết nối UART 2 (thiết bị phụ) ở tab KẾT NỐI trước."
            )
            return
        pwm = self._rpm_to_pwm(self.spin_rpm.value())
        if self.ctx.pneu_uart.turn_set_speed(pwm):
            self.ctx.turn_state = True
            self._append_log(f"↻ Bàn xoay BẬT: {self.spin_rpm.value():.1f} v/p -> PWM={pwm}")
            self._refresh_turn_label()

    def on_turn_off(self):
        if self.ctx.pneu_uart.turn_off():
            self.ctx.turn_state = False
            self._append_log("■ Bàn xoay TẮT.")
            self._refresh_turn_label()

    def _refresh_turn_label(self):
        if self.ctx.turn_state:
            self.lbl_turn_state.setText("● Đang BẬT")
            self.lbl_turn_state.setObjectName("deviceStateOn")
        else:
            self.lbl_turn_state.setText("● Đang TẮT")
            self.lbl_turn_state.setObjectName("deviceStateOff")
        self.lbl_turn_state.setStyleSheet("")
        self.lbl_turn_state.style().unpolish(self.lbl_turn_state)
        self.lbl_turn_state.style().polish(self.lbl_turn_state)

    # ---------------- Camera run4dof ----------------
    def on_open_camera(self):
        if self._camera_thread is not None:
            return
        self.btn_cam_open.setEnabled(False)
        self._camera_thread = CameraRun4DofThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=self.ctx.cfg.get("calib_file"),
        )
        self._camera_thread.frame_ready.connect(self._on_frame)
        self._camera_thread.error.connect(self._on_camera_error)
        self._camera_thread.stopped.connect(self._on_camera_stopped)

        # ---- ĐĂNG KÝ camera thread của tab này lên ctx (AppContext) ----
        # Để các tab khác (circle/dof4 tĩnh, dynamic tĩnh) biết camera vật lý
        # đang bị tab "BÀI TOÁN ĐỘNG" chiếm dụng, tránh mở trùng / gửi lệnh
        # robot chồng chéo. Cần AppContext có sẵn thuộc tính
        # run4dof_camera_thread (xem ghi chú cập nhật shared_state.py).
        self.ctx.run4dof_camera_thread = self._camera_thread

        self._camera_thread.start()
        self.btn_cam_close.setEnabled(True)

    def on_close_camera(self):
        if self._camera_thread is not None:
            self._camera_thread.stop()

    # ------------------------------------------------------------------
    # TÁCH items (nhận được từ camera_run4dof.py) thành 2 danh sách vật/lỗ.
    #
    # BUG ĐÃ PHÁT HIỆN: trước đây tách bằng it.get("type") == "object"/"hole",
    # nhưng camera_run4dof.py build item như sau (xem file đó):
    #     - VẬT: item = {**o, "diameter_mm": None, "slot_id": None}
    #     - LỖ : item = {**h, "diameter_mm": None}
    # -> KHÔNG dòng nào gắn field "type" cả. Do đó lọc theo "type" SAI với
    # cả 2 loại.
    #
    # SỬA: dùng "slot_id" làm đặc trưng phân biệt - VẬT LUÔN bị ép
    # slot_id=None, còn LỖ LUÔN giữ slot_id thật (số nguyên 1..8) lấy từ
    # detect_frame_holes(). Vẫn ưu tiên field "type" nếu sau này
    # camera_run4dof.py được cập nhật để gắn đúng field này.
    # ------------------------------------------------------------------
    @staticmethod
    def _split_objects_holes(items):
        objects, holes = [], []
        for it in items:
            t = it.get("type")
            if t == "object":
                objects.append(it)
            elif t == "hole":
                holes.append(it)
            else:
                # Không có "type" đáng tin -> suy luận qua slot_id.
                if it.get("slot_id") is None:
                    objects.append(it)
                else:
                    holes.append(it)
        return objects, holes

    def _on_frame(self, frame, items, fps):
        self.latest_objects, self.latest_holes = self._split_objects_holes(items)

        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps.setText(f"FPS: {fps:.1f}")

        self._maybe_start_cycle()

    def _on_camera_error(self, msg):
        self._append_log(f"⚠ {msg}")
        QMessageBox.warning(self, "Lỗi Camera", msg)

    def _on_camera_stopped(self):
        self._camera_thread = None
        # Gỡ đăng ký khỏi ctx ngay khi camera thực sự dừng hẳn, để các tab
        # khác biết camera đã rảnh trở lại.
        if getattr(self.ctx, "run4dof_camera_thread", None) is not None:
            self.ctx.run4dof_camera_thread = None
        self.video_label.setText("Camera đã đóng")
        self.btn_cam_open.setEnabled(True)
        self.btn_cam_close.setEnabled(False)

    def _append_log(self, text):
        QTimer.singleShot(0, lambda: self.log_box.append(text))

    def _set_phase_label(self, text):
        QTimer.singleShot(0, lambda: self.lbl_phase.setText(f"Trạng thái: {text}"))

    # ---------------- START / STOP vòng lặp gắp-thả tự động ----------------
    def on_start(self):
        """START: về Home đã lưu trước, sau đó mở camera (nếu chưa mở) và
        bắt đầu vòng lặp tự động 2 PHA: PICK (gắp vật ngay khi thấy, không
        cần chờ khung) -> PLACE (giữ vật ở Home, chờ thấy khung mới thả).

        LƯU Ý: dùng ctx.planner_run4dof (kinematics/move_run4dof.py) - planner
        RIÊNG cho tab BÀI TOÁN ĐỘNG, KHÔNG dùng ctx.planner_dof4 (planner dùng
        chung với BÀI TOÁN BẬC 4 TĨNH và CSV)."""
        if self.running:
            return
        if not self.ctx.robot_uart.is_connected:
            QMessageBox.warning(self, "Chưa kết nối Robot",
                                 "Hãy kết nối UART 1 (Robot) ở tab KẾT NỐI trước.")
            return
        if (
            self.ctx.camera_thread is not None
            or self.ctx.dynamic_camera_thread is not None
        ):
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Một tab camera khác đang mở - hãy đóng camera ở tab đó trước.")
            return

        self.log_box.clear()
        self.log_box.append("=== START: đang di chuyển về Home đã lưu... ===")
        self.btn_start.setEnabled(False)
        self.holding_object = False
        self._held_object_angle = None

        # (THÊM MỚI) reset bộ đếm chu trình mỗi lần bấm START lại từ đầu
        self.completed_cycles = 0
        self.is_recalibrating = False
        self._refresh_recal_progress_label()

        self._set_phase_label("đang về Home...")
        self.ctx.busy = True

        hx, hy, hz = self.ctx.home_pos
        worker = FnWorker(self.ctx.planner_run4dof.send_position, hx, hy, hz)
        worker.done_ok.connect(self._on_start_home_done)
        worker.done_err.connect(self._on_start_error)
        self._track_worker(worker)

    def _on_start_home_done(self, ok):
        self.ctx.busy = False
        if not ok:
            self._append_log("⚠ Về Home thất bại - không thể bắt đầu.")
            self._set_phase_label("lỗi - đã dừng")
            self.btn_start.setEnabled(True)
            return
        self.ctx.current_pos = list(self.ctx.home_pos)
        self._append_log("✔ Đã về Home. Bắt đầu camera + vòng lặp tự động...")
        self.running = True
        self.btn_stop.setEnabled(True)
        self._set_phase_label("đang chờ thấy VẬT...")
        if self._camera_thread is None:
            self.on_open_camera()

    def _on_start_error(self, err):
        self.ctx.busy = False
        self._append_log(f"⚠ LỖI: {err}")
        self._set_phase_label("lỗi - đã dừng")
        self.btn_start.setEnabled(True)

    def _maybe_start_cycle(self):
        """Được gọi mỗi khi có frame mới. Chạy theo 2 PHA:

        - CHƯA giữ vật (holding_object=False): chỉ cần thấy VẬT là gắp ngay
          (pick_dof4) - KHÔNG cần thấy khung. Sau khi gắp xong, robot về Home
          và giữ nguyên vật, holding_object chuyển thành True.

        - ĐANG giữ vật (holding_object=True): không gắp thêm vật nào khác
          (tay đang bận), chỉ chờ đến khi latest_holes có dữ liệu thì gọi
          place_dof4() để thả vào khung, rồi quay lại PHA gắp cho vòng kế.
        """
        if not self.running or self.cycle_busy or self.ctx.busy:
            return

        if not self.holding_object:
            if not self.latest_objects:
                return
            self._start_pick_phase()
        else:
            if not self.latest_holes:
                return
            self._start_place_phase()

    # ---------------- PHA 1: PICK (gắp vật, chưa cần khung) ----------------
    def _start_pick_phase(self):
        obj = self.latest_objects[0]

        self.cycle_busy = True
        self.ctx.busy = True
        self._set_phase_label("đang GẮP vật...")

        offset_x = self.spin_offset_x.value()
        offset_y = self.spin_offset_y.value()
        z_pick = self.spin_z_pick.value()

        # Dùng TỌA ĐỘ DỰ ĐOÁN (x_mm_grasp/y_mm_grasp) - vị trí vật SẼ CÓ tại
        # thời điểm robot thực sự chạm/gắp (đã bù cho bàn xoay liên tục
        # trong lúc robot di chuyển tới), do camera_run4dof.py tính sẵn.
        # .get(..., fallback) để vẫn chạy được nếu camera_run4dof.py chưa
        # có 2 trường này (tương thích ngược).
        obj_x = obj.get("x_mm_grasp", obj["x_mm"])
        obj_y = obj.get("y_mm_grasp", obj["y_mm"])
        obj_angle = obj.get("angle_deg_grasp", obj.get("angle_deg"))

        self._append_log(
            f"→ [PICK] Gắp vật (dự đoán) tại ({obj_x:.1f},{obj_y:.1f}) "
            f"góc {obj_angle:.1f}° - chưa cần khung."
        )

        point_a = (obj_x + offset_x, obj_y + offset_y)
        # Lưu lại góc vật lúc gắp để dùng cho bước xoay bù ở PHA PLACE sau.
        self._held_object_angle = obj_angle

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log(s))):
                self.ctx.planner_run4dof.pick_dof4(
                    point_a,
                    z_pick=z_pick,
                    gripper_callback=self.ctx.gripper_callback,
                )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_pick_phase_done)
        worker.done_err.connect(self._on_cycle_error)
        self._track_worker(worker)

    def _on_pick_phase_done(self, ok):
        self.cycle_busy = False
        self.ctx.busy = False
        self.ctx.current_pos = list(self.ctx.planner_run4dof.HOME)
        if ok:
            self.holding_object = True
            self._append_log("✔ Đã gắp vật, đang giữ tại Home - chờ thấy khung để thả...")
            self._set_phase_label("đang GIỮ vật - chờ thấy KHUNG...")
        else:
            self.holding_object = False
            self._held_object_angle = None
            self._append_log("⏹ Gắp vật dừng giữa chừng.")
            self._set_phase_label("đang chờ thấy VẬT...")

    # ---------------- PHA 2: PLACE (chờ thấy khung rồi mới thả) ----------------
    def _start_place_phase(self):
        hole = self.latest_holes[0]

        self.cycle_busy = True
        self.ctx.busy = True
        self._set_phase_label("đang THẢ vật vào khung...")

        offset_x = self.spin_offset_x.value()
        offset_y = self.spin_offset_y.value()
        z_pick = self.spin_z_pick.value()

        hole_x = hole.get("x_mm_grasp", hole["x_mm"])
        hole_y = hole.get("y_mm_grasp", hole["y_mm"])
        hole_angle = hole.get("angle_deg_grasp", hole.get("angle_deg", 90.0))

        self._append_log(
            f"→ [PLACE] Đặt vào Lỗ #{hole.get('slot_id')} (dự đoán) "
            f"({hole_x:.1f},{hole_y:.1f}) góc {hole_angle:.1f}°"
        )

        place_point = (hole_x + offset_x, hole_y + offset_y)
        object_angle_deg = self._held_object_angle

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log(s))):
                self.ctx.planner_run4dof.place_dof4(
                    place_point,
                    z_pick=z_pick,
                    gripper_callback=self.ctx.gripper_callback,
                    rotate_callback=self.ctx.pneu_uart.step_rotate,
                    object_angle_deg=object_angle_deg,
                    target_angle_deg=hole_angle,
                )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_place_phase_done)
        worker.done_err.connect(self._on_cycle_error)
        self._track_worker(worker)

    def _on_place_phase_done(self, ok):
        self.cycle_busy = False
        self.ctx.busy = False
        self.holding_object = False
        self._held_object_angle = None
        self.ctx.current_pos = list(self.ctx.planner_run4dof.HOME)
        if ok:
            self._append_log("✔ Hoàn tất 1 chu trình gắp-thả.")
        else:
            self._append_log("⏹ Chu trình thả dừng giữa chừng.")
        self._set_phase_label("đang chờ thấy VẬT...")

        # ==================================================================
        # (THÊM MỚI) ĐẾM CHU TRÌNH + KÍCH HOẠT HIỆU CHỈNH ĐỊNH KỲ
        # Chỉ đếm khi chu trình THÀNH CÔNG (ok=True). Khi đủ số chu trình
        # cấu hình (RECAL_EVERY_N_CYCLES), tự động về HOME vật lý rồi về
        # lại HOME đã lưu trước khi cho vòng lặp PICK/PLACE tiếp tục.
        # ==================================================================
        if ok:
            self.completed_cycles += 1
            self._refresh_recal_progress_label()
            if self._maybe_trigger_periodic_recalibration():
                return  # đang hiệu chỉnh -> giữ nguyên phase hiệu chỉnh, không ghi đè

    # ==================================================================
    # (THÊM MỚI) HIỆU CHỈNH ĐỊNH KỲ: sau mỗi RECAL_EVERY_N_CYCLES chu
    # trình gắp-thả thành công, tự động về HOME vật lý (giống hệt on_stop
    # -> home_and_wait) rồi về HOME đã lưu (giống hệt on_start ->
    # send_position tới ctx.home_pos) để triệt tiêu sai số cộng dồn, sau
    # đó tiếp tục vòng lặp PICK/PLACE bình thường.
    # ==================================================================
    def _maybe_trigger_periodic_recalibration(self):
        if self.completed_cycles < self.RECAL_EVERY_N_CYCLES:
            return False

        self.completed_cycles = 0
        self._refresh_recal_progress_label()
        self.is_recalibrating = True
        self.cycle_busy = True
        self.ctx.busy = True
        self._append_log(
            f"↻ Đã hoàn tất {self.RECAL_EVERY_N_CYCLES} chu trình - "
            "tự động về HOME vật lý để hiệu chỉnh lại độ chính xác..."
        )
        self._set_phase_label("đang hiệu chỉnh: về HOME vật lý...")

        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_recal_physical_home_done)
        worker.done_err.connect(self._on_recal_error)
        self._track_worker(worker)
        return True

    def _on_recal_physical_home_done(self, ready):
        if not ready:
            self._append_log("⚠ Hiệu chỉnh: Không nhận được READY từ STM32 khi về HOME vật lý.")
        else:
            self._append_log("✔ Hiệu chỉnh: Đã về HOME vật lý xong (READY).")

        self._append_log("↻ Hiệu chỉnh: đang về HOME đã lưu...")
        self._set_phase_label("đang hiệu chỉnh: về HOME đã lưu...")

        hx, hy, hz = self.ctx.home_pos
        worker = FnWorker(self.ctx.planner_run4dof.send_position, hx, hy, hz)
        worker.done_ok.connect(self._on_recal_saved_home_done)
        worker.done_err.connect(self._on_recal_error)
        self._track_worker(worker)

    def _on_recal_saved_home_done(self, ok):
        self.cycle_busy = False
        self.ctx.busy = False
        self.is_recalibrating = False
        if ok:
            self.ctx.current_pos = list(self.ctx.home_pos)
            self._append_log("✔ Hiệu chỉnh hoàn tất - đã về lại HOME đã lưu. Tiếp tục vòng lặp...")
        else:
            self._append_log("⚠ Hiệu chỉnh: về HOME đã lưu thất bại - vẫn tiếp tục vòng lặp.")
        self._set_phase_label("đang chờ thấy VẬT..." if self.running else "(đã dừng)")

    def _on_recal_error(self, err):
        self.cycle_busy = False
        self.ctx.busy = False
        self.is_recalibrating = False
        self._append_log(f"⚠ LỖI trong lúc hiệu chỉnh định kỳ: {err}")
        self._set_phase_label("LỖI - đang chờ thấy VẬT..." if self.running else "(đã dừng)")

    def _on_cycle_error(self, err):
        self.cycle_busy = False
        self.ctx.busy = False
        # An toàn: nếu lỗi xảy ra giữa chừng ở BẤT KỲ pha nào, reset lại
        # trạng thái giữ vật để tránh vòng lặp bị "kẹt" mãi ở PHA PLACE
        # trong khi thực tế robot đã tự về Home an toàn (không rõ còn giữ
        # vật hay không) - người vận hành nên kiểm tra tay gắp bằng mắt.
        self.holding_object = False
        self._held_object_angle = None
        self._append_log(f"⚠ LỖI chu trình: {err} (đã reset trạng thái - kiểm tra tay gắp bằng mắt)")
        self._set_phase_label("LỖI - đang chờ thấy VẬT...")

    def on_stop(self):
        """STOP: dừng vòng lặp, đóng camera, chờ chu trình hiện tại (nếu có)
        hoàn tất, rồi về HOME VẬT LÝ (chạm công tắc hành trình) - GIỐNG HỆT
        nút STOP của BÀI TOÁN BẬC 4 TĨNH (dynamic_window.py).

        LƯU Ý: nếu dừng đúng lúc đang giữ vật (holding_object=True, chưa kịp
        thấy khung để thả), vật sẽ VẪN CÒN trên tay gắp khi robot về Home vật
        lý - cần người vận hành kiểm tra và gỡ vật ra thủ công nếu cần.

        LƯU Ý (THÊM MỚI): nếu STOP được bấm đúng lúc đang hiệu chỉnh định kỳ
        (is_recalibrating=True, cycle_busy=True), _wait_then_home() bên dưới
        sẽ tự động CHỜ cho tới khi hiệu chỉnh xong (cycle_busy trở về False)
        rồi mới cho robot về Home vật lý - không có lệnh nào bị chồng lên
        nhau."""
        if not self.running and self._camera_thread is None:
            return
        self.running = False
        self.btn_stop.setEnabled(False)
        if self.holding_object:
            self._append_log(
                "⚠ Đang dừng khi TAY GẮP CÒN GIỮ VẬT (chưa thấy khung để thả) "
                "- kiểm tra và gỡ vật thủ công sau khi về Home nếu cần."
            )
        self._append_log(
            "=== STOP: dừng vòng lặp, chờ chu trình hiện tại (nếu có) hoàn tất rồi về HOME vật lý... ==="
        )
        self._set_phase_label("đang dừng...")
        self.on_close_camera()
        self._wait_then_home()

    def _wait_then_home(self):
        if self.cycle_busy:
            QTimer.singleShot(300, self._wait_then_home)
            return
        self._append_log("↻ Đang về HOME vật lý (chạm công tắc hành trình) và chờ READY...")
        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_stop_home_done)
        worker.done_err.connect(self._on_start_error)
        self._track_worker(worker)

    def _on_stop_home_done(self, ready):
        if ready:
            self._append_log("✔ Đã về HOME xong (READY).")
        else:
            self._append_log("⚠ Không nhận được READY từ STM32.")
        self._set_phase_label("(đã dừng)")
        self.btn_start.setEnabled(True)

    # ------------------------------------------------------------------
    # QUAY LẠI MENU: TRƯỚC ĐÂY chỉ ẩn cửa sổ (BaseTabWindow.go_back gốc),
    # KHÔNG dừng camera lẫn vòng lặp gắp-thả tự động -> nếu người dùng bấm
    # START rồi bấm "QUAY LẠI MENU" mà không bấm STOP trước, camera + robot
    # VẪN TIẾP TỤC chạy ngầm sau khi cửa sổ đã ẩn. Override lại: nếu đang
    # chạy hoặc camera đang mở thì tự gọi on_stop() (dừng đúng quy trình)
    # trước khi thực sự rời cửa sổ.
    # ------------------------------------------------------------------
    def go_back(self):
        if self.running or self._camera_thread is not None:
            self.on_stop()
        super().go_back()

    def closeEvent(self, event):
        if self._camera_thread is not None:
            self._camera_thread.stop()
            if getattr(self.ctx, "run4dof_camera_thread", None) is not None:
                self.ctx.run4dof_camera_thread = None
        super().closeEvent(event)