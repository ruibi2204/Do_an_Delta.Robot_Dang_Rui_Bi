# dynamic_run4dof_window.py
import contextlib

from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QGroupBox,
    QDoubleSpinBox, QTextEdit, QSizePolicy, QMessageBox, QSlider,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, cv2_to_qpixmap,
    save_config, home_and_wait, TURNTABLE_MAX_RPM,
)

# Các khóa tham số tốc độ (thời gian di chuyển) mà thanh trượt SPEED sẽ
# scale trên ctx.planner_run4dof (move_run4dof.py). Bao gồm CẢ bộ tốc độ
# CHUNG lẫn 2 bộ RIÊNG cho pha PICK/PLACE, để kéo 1 thanh trượt là chỉnh
# nhanh/chậm toàn bộ chu trình cùng lúc. CONTROL_HZ và MAX_STEP_MM KHÔNG
# nằm trong danh sách này vì chúng quyết định độ mịn/thẳng quỹ đạo, cần
# giữ nguyên ở mọi mức tốc độ.
_SPEED_SLIDER_TIME_KEYS = (
    "TIME_MOVE_FAST",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_ACTION",
    "TIME_MOVE_FAST_PICK",
    "TIME_MOVE_DOWN_PICK",
    "TIME_MOVE_ACTION_PICK",
    "TIME_MOVE_FAST_PLACE",
    "TIME_MOVE_DOWN_PLACE",
    "TIME_MOVE_ACTION_PLACE",
    "TIME_DELAY_GRIPPER",
    "TIME_DELAY_DOF4",
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
        self.holding_object = False
        self._held_object_angle = None

        # Bias đã LƯU lần cuối (đọc từ ctx.cfg lúc khởi động, tức đúng giá
        # trị đang nằm trong robot_config.json). Dùng để so sánh với bias
        # ĐANG HỌC (live, có thể khác nếu người dùng chưa bấm nút LƯU BIAS)
        # và hiển thị trạng thái "đã lưu"/"chưa lưu" trên giao diện.
        self._last_saved_bias_pick = float(self.ctx.cfg.get("run4dof_bias_pick", 0.0))
        self._last_saved_bias_place = float(self.ctx.cfg.get("run4dof_bias_place", 0.0))

        # ==================================================================
        # ĐẾM CHU TRÌNH ĐỂ TỰ ĐỘNG HIỆU CHỈNH LẠI ĐỘ CHÍNH XÁC
        # ==================================================================
        self.RECAL_EVERY_N_CYCLES = 3
        self.completed_cycles = 0
        self.is_recalibrating = False

        # Ghi lại tham số tốc độ GỐC (100%) của planner_run4dof ngay khi
        # window được tạo, để thanh trượt luôn scale từ một mốc cố định
        # chứ không bị trôi dần qua nhiều lần kéo.
        self._base_speed_params = self._capture_base_speed_params()

        self._build_ui()
        self._restore_saved_speed_percent()

    def showEvent(self, event):
        super().showEvent(event)
        self._refresh_turn_label()
        self.btn_start.setEnabled(not self.running)
        self.btn_stop.setEnabled(self.running)

    # ------------------------------------------------------------------
    def _capture_base_speed_params(self):
        planner = self.ctx.planner_run4dof
        return {
            key: getattr(planner, key)
            for key in _SPEED_SLIDER_TIME_KEYS
            if hasattr(planner, key)
        }

    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        # ---------- Cột video ----------
        video_col = QVBoxLayout()
        self.video_label = QLabel("Camera chưa mở")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(720, 480)
        self.video_label.setStyleSheet(
            "background-color:#0f2540; border:2px solid #1a7ee0; "
            "border-radius:10px; color:#b9d2ee;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 5)

        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#12315c; font-weight:700;")
        video_col.addWidget(self.lbl_fps)

        cam_btn_row = QHBoxLayout()
        self.btn_cam_open = QPushButton("▶ MỞ CAMERA (CHẠY 4DOF)")
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
        self.btn_start = QPushButton("▶ BẮT ĐẦU")
        self.btn_start.setObjectName("dynStartBtn")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("■ DỪNG")
        self.btn_stop.setObjectName("dynStopBtn")
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)
        start_stop_row.addWidget(self.btn_start)
        start_stop_row.addWidget(self.btn_stop)
        video_col.addLayout(start_stop_row)

        # ---------- Nhãn trạng thái pha PICK / PLACE ----------
        self.lbl_phase = QLabel("Trạng thái: (chưa chạy)")
        self.lbl_phase.setStyleSheet("color:#4b6a93; font-weight:700;")
        video_col.addWidget(self.lbl_phase)

        # ---------- Cột điều khiển ----------
        ctrl_col = QVBoxLayout()

        # ---------- THANH TRƯỢT TỐC ĐỘ ----------
        speed_box = QGroupBox("TỐC ĐỘ DI CHUYỂN (CHUNG / PICK / PLACE)")
        speed_box.setObjectName("compactBox")
        speed_box.setMaximumHeight(76)
        speed_layout = QHBoxLayout()
        speed_layout.setContentsMargins(10, 4, 10, 6)
        speed_layout.setSpacing(8)

        speed_layout.addWidget(QLabel("Chậm"))
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(25, 200)   # 25% (chậm) -> 200% (nhanh gấp đôi)
        self.slider_speed.setSingleStep(5)
        self.slider_speed.setPageStep(25)
        self.slider_speed.setValue(100)
        self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)
        speed_layout.addWidget(self.slider_speed, 1)
        speed_layout.addWidget(QLabel("Nhanh"))

        self.lbl_speed_value = QLabel("100%")
        self.lbl_speed_value.setMinimumWidth(48)
        self.lbl_speed_value.setStyleSheet("font-weight:700;")
        speed_layout.addWidget(self.lbl_speed_value)

        speed_box.setLayout(speed_layout)
        ctrl_col.addWidget(speed_box)

        # ---------- Khung Z PICK && OFFSET TỌA ĐỘ ----------
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

        # ---------- Khung BIAS TỰ HỌC + nút LƯU thủ công ----------
        # Theo yêu cầu: KHÔNG tự động lưu bias sau mỗi chu trình nữa (xem
        # _on_place_phase_done/on_stop - đã bỏ auto-save). Người dùng chạy
        # thực nghiệm tự do, quan sát bias hội tụ qua label + overlay trên
        # camera, và chỉ bấm "LƯU BIAS" khi thấy bộ số ổn định/ưng ý.
        bias_box = QGroupBox("BIAS TỰ HỌC (mâm xoay)")
        bias_box.setObjectName("compactBox")
        bias_layout = QVBoxLayout()
        bias_layout.setSpacing(6)

        bias_row = QHBoxLayout()
        self.lbl_bias_pick = QLabel("PICK: +0.000s")
        self.lbl_bias_pick.setStyleSheet("font-weight:700;")
        self.lbl_bias_place = QLabel("PLACE: +0.000s")
        self.lbl_bias_place.setStyleSheet("font-weight:700;")
        bias_row.addWidget(self.lbl_bias_pick, 1)
        bias_row.addWidget(self.lbl_bias_place, 1)
        bias_layout.addLayout(bias_row)

        self.lbl_bias_saved_state = QLabel("(chưa có dữ liệu)")
        self.lbl_bias_saved_state.setStyleSheet("color:#4b6a93;")
        bias_layout.addWidget(self.lbl_bias_saved_state)

        self.btn_save_bias = QPushButton("💾 LƯU BIAS HIỆN TẠI")
        self.btn_save_bias.setObjectName("saveBtn")
        self.btn_save_bias.setStyleSheet(
            "QPushButton#saveBtn { font-size: 11pt; min-height: 34px; max-height: 34px; padding: 4px 6px; }"
        )
        self.btn_save_bias.clicked.connect(self.on_save_bias_clicked)
        bias_layout.addWidget(self.btn_save_bias)

        bias_box.setLayout(bias_layout)
        ctrl_col.addWidget(bias_box)

        # ---------- Khung HIỆU CHỈNH ĐỊNH KỲ ----------
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
        self.lbl_recal_progress.setStyleSheet("color:#1670d4; font-weight:700;")
        recal_layout.addWidget(self.lbl_recal_progress)

        recal_box.setLayout(recal_layout)
        ctrl_col.addWidget(recal_box)

        # ---- BÀN XOAY: step 17 (PUL/DIR). Firmware nhận thẳng TURN:<rpm>,
        # KHÔNG còn quy đổi qua PWM 0-255 (đó là của driver L298N/DC cũ). ----
        turn_box = QGroupBox(f"BÀN XOAY (động cơ tối đa {TURNTABLE_MAX_RPM:.0f} v/p)")
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

    # ---------------- THANH TRƯỢT TỐC ĐỘ ----------------
    def _restore_saved_speed_percent(self):
        """Khôi phục % tốc độ đã lưu lần trước (nếu có) khi mở lại app."""
        saved_percent = int(self.ctx.cfg.get("run4dof_speed_percent", 100))
        saved_percent = max(self.slider_speed.minimum(), min(self.slider_speed.maximum(), saved_percent))
        if saved_percent != 100:
            self.slider_speed.setValue(saved_percent)  # sẽ tự trigger _on_speed_slider_changed
        else:
            self.lbl_speed_value.setText("100%")

    def _on_speed_slider_changed(self, value):
        self.lbl_speed_value.setText(f"{value}%")
        scale = value / 100.0

        scaled_params = {}
        for key, base_time in self._base_speed_params.items():
            # % càng cao -> di chuyển càng NHANH -> thời gian càng NHỎ.
            scaled_params[key] = base_time / scale if scale > 0 else base_time

        # KHÔNG scale CONTROL_HZ / MAX_STEP_MM: hai tham số này quyết định
        # độ mịn và độ thẳng của quỹ đạo, cần giữ nguyên ở mọi tốc độ.
        #
        # LƯU Ý: không cần đồng bộ lại gì cho vision/camera_run4dof.py ở
        # đây. Module vision giữ THAM CHIẾU tới chính planner_run4dof (đã
        # gán 1 lần qua set_planner() khi mở camera - xem on_open_camera),
        # nên solve_rendezvous() ở tầng vision tự đọc TIME_MOVE_*_PICK/
        # PLACE mới nhất ngay từ lần dự đoán kế tiếp, không có độ trễ
        # đồng bộ nào cả.
        self.ctx.planner_run4dof.update_speed_params(scaled_params)

        self.ctx.cfg["run4dof_speed_percent"] = value
        save_config(self.ctx.cfg)

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

    # ---------------- Hiệu chỉnh định kỳ - cấu hình số chu trình ----------------
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
    def _on_rpm_changed(self, val):
        self.ctx.cfg["run4dof_rpm"] = val
        save_config(self.ctx.cfg)
        # Nếu bàn xoay đang chạy, đổi RPM cần cập nhật ngay cho dự đoán ảnh,
        # không đợi tắt/bật lại mới đồng bộ.
        if self.ctx.turn_state:
            self._sync_table_rpm_to_vision(val)

    def on_turn_on(self):
        if not self.ctx.pneu_uart.is_connected:
            QMessageBox.warning(
                self, "Chưa kết nối",
                "Hãy kết nối UART 2 (thiết bị phụ) ở tab KẾT NỐI trước."
            )
            return
        rpm = self.spin_rpm.value()
        if self.ctx.pneu_uart.turn_set_speed(rpm):
            self.ctx.turn_state = True
            self._append_log(f"↻ Bàn xoay BẬT: {rpm:.1f} v/p")
            self._refresh_turn_label()
            self._sync_table_rpm_to_vision(rpm)

    def on_turn_off(self):
        if self.ctx.pneu_uart.turn_off():
            self.ctx.turn_state = False
            self._append_log("■ Bàn xoay TẮT.")
            self._refresh_turn_label()
            self._sync_table_rpm_to_vision(0.0)

    def _sync_table_rpm_to_vision(self, rpm):
        """Báo RPM THỰC TẾ vừa gửi xuống Arduino cho module xử lý ảnh
        (camera_run4dof.py), để công thức dự đoán vị trí gắp
        (predict_grasp_pose) luôn dùng đúng tốc độ đang chạy thay vì một
        hằng số cố định lệch với thực tế. Import lazy + try/except giống
        cách camera_run4dof được import trong CameraRun4DofThread, để cửa
        sổ vẫn hoạt động bình thường ngay cả khi chưa mở camera lần nào."""
        try:
            from vision.camera_run4dof import set_table_rpm
            set_table_rpm(rpm)
        except Exception as e:
            self._append_log(f"⚠ Không đồng bộ được RPM cho dự đoán ảnh: {e}")

    def _sync_planner_to_vision(self):
        """Trỏ vision/camera_run4dof.py về ĐÚNG planner đang điều khiển
        robot (ctx.planner_run4dof), để solve_rendezvous() tính delay dự
        đoán theo ĐÚNG tốc độ hiện hành + ĐÚNG khoảng cách từng vật/khung -
        thay cho GRASP_DELAY_S cố định trước đây (sai module nguồn, sai bộ
        tốc độ, không cập nhật khi đổi tốc độ qua thanh trượt). Giống hệt
        mẫu hình _sync_table_rpm_to_vision ở trên. Vì planner là object
        mutable, chỉ cần gán tham chiếu 1 LẦN duy nhất ở đây - không cần
        gọi lại trong _on_speed_slider_changed hay bất kỳ đâu khác."""
        try:
            from vision.camera_run4dof import set_planner
            set_planner(self.ctx.planner_run4dof)
        except Exception as e:
            self._append_log(f"⚠ Không đồng bộ được planner cho dự đoán ảnh: {e}")

    def _restore_saved_bias(self):
        """Nạp lại bias đã học từ lần chạy trước (lưu trong
        ctx.cfg["run4dof_bias_pick"/"run4dof_bias_place"], đọc từ
        robot_config.json lúc load_config()). Gọi 1 lần khi mở camera,
        NGAY SAU _sync_planner_to_vision(), để không phải học lại từ 0
        mỗi lần mở app - đúng theo yêu cầu lưu bền vững qua các phiên."""
        try:
            from vision.camera_run4dof import set_bias
            bias_pick = float(self.ctx.cfg.get("run4dof_bias_pick", 0.0))
            bias_place = float(self.ctx.cfg.get("run4dof_bias_place", 0.0))
            set_bias(bias_pick, bias_place)
            if bias_pick != 0.0 or bias_place != 0.0:
                self._append_log(
                    f"↺ Đã nạp lại bias đã học lần trước: "
                    f"PICK={bias_pick:+.3f}s PLACE={bias_place:+.3f}s"
                )
            self._update_bias_labels()
        except Exception as e:
            self._append_log(f"⚠ Không nạp lại được bias đã lưu: {e}")

    def _save_bias_to_config(self):
        """Lưu bias hiện tại (đã được EMA cập nhật qua report_actual_delay)
        vào ctx.cfg rồi ghi xuống robot_config.json. CHỈ được gọi từ
        on_save_bias_clicked() (nút "LƯU BIAS HIỆN TẠI") - KHÔNG còn tự
        động gọi sau mỗi chu trình như trước, theo đúng yêu cầu: người
        dùng chạy thực nghiệm tự do, chỉ lưu khi thấy bộ số ổn định/ưng ý."""
        try:
            from vision.camera_run4dof import get_bias_snapshot
            bias_pick, bias_place = get_bias_snapshot()
            self.ctx.cfg["run4dof_bias_pick"] = bias_pick
            self.ctx.cfg["run4dof_bias_place"] = bias_place
            save_config(self.ctx.cfg)
        except Exception as e:
            self._append_log(f"⚠ Không lưu được bias đã học: {e}")

    def on_save_bias_clicked(self):
        """Xử lý nút '💾 LƯU BIAS HIỆN TẠI' - ghi snapshot bias ĐANG HỌC
        (live) xuống robot_config.json, cập nhật mốc _last_saved_bias_*
        để label chuyển sang trạng thái 'Đã lưu'."""
        self._save_bias_to_config()
        try:
            from vision.camera_run4dof import get_bias_snapshot
            self._last_saved_bias_pick, self._last_saved_bias_place = get_bias_snapshot()
        except Exception:
            pass
        self._append_log(
            f"💾 Đã lưu bias hiện tại vào robot_config.json: "
            f"PICK={self._last_saved_bias_pick:+.3f}s PLACE={self._last_saved_bias_place:+.3f}s"
        )
        self._update_bias_labels()

    def _update_bias_labels(self):
        """Đọc snapshot bias ĐANG HỌC (live) từ vision module và đẩy việc
        cập nhật widget về MAIN THREAD qua QTimer.singleShot - vì hàm này
        có thể được gọi từ worker thread (on_grasp_moment chạy trong luồng
        FnWorker, không phải main thread Qt), giống cách _append_log đã
        làm. Gọi định kỳ sau mỗi lần report_actual_delay và sau khi nạp
        lại bias lúc mở camera, để label luôn phản ánh giá trị mới nhất."""
        try:
            from vision.camera_run4dof import get_bias_snapshot
            bias_pick, bias_place = get_bias_snapshot()
        except Exception:
            return
        QTimer.singleShot(0, lambda: self._apply_bias_labels(bias_pick, bias_place))

    def _apply_bias_labels(self, bias_pick, bias_place):
        self.lbl_bias_pick.setText(f"PICK: {bias_pick:+.3f}s")
        self.lbl_bias_place.setText(f"PLACE: {bias_place:+.3f}s")

        is_saved = (
            abs(bias_pick - self._last_saved_bias_pick) < 1e-6
            and abs(bias_place - self._last_saved_bias_place) < 1e-6
        )
        if is_saved:
            self.lbl_bias_saved_state.setText("✔ Đã lưu (khớp với robot_config.json)")
            self.lbl_bias_saved_state.setStyleSheet("color:#0b7a3e; font-weight:700;")
        else:
            self.lbl_bias_saved_state.setText('● CHƯA LƯU - bấm "LƯU BIAS HIỆN TẠI" nếu muốn giữ lại')
            self.lbl_bias_saved_state.setStyleSheet("color:#b45309; font-weight:700;")

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

        # Đồng bộ planner cho vision TRƯỚC khi mở luồng camera, để ngay
        # frame đầu tiên solve_rendezvous() đã có planner để tính delay
        # đúng theo khoảng cách/tốc độ thật, không rơi vào nhánh dự phòng
        # GRASP_DELAY_S cố định.
        self._sync_planner_to_vision()

        # Nạp lại bias đã học từ lần chạy trước (nếu có) - PHẢI gọi SAU
        # _sync_planner_to_vision() vì cùng nằm trong module vision, thứ
        # tự không bắt buộc về mặt kỹ thuật nhưng giữ đúng logic "đồng bộ
        # nguồn rồi mới nạp trạng thái đã lưu" cho dễ đọc code.
        self._restore_saved_bias()

        self._camera_thread = CameraRun4DofThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=self.ctx.cfg.get("calib_file"),
        )
        self._camera_thread.frame_ready.connect(self._on_frame)
        self._camera_thread.error.connect(self._on_camera_error)
        self._camera_thread.stopped.connect(self._on_camera_stopped)

        self.ctx.run4dof_camera_thread = self._camera_thread

        self._camera_thread.start()
        self.btn_cam_close.setEnabled(True)

    def on_close_camera(self):
        if self._camera_thread is not None:
            self._camera_thread.stop()

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
        if getattr(self.ctx, "run4dof_camera_thread", None) is not None:
            self.ctx.run4dof_camera_thread = None
        self.video_label.setText("Camera đã đóng")
        self.btn_cam_open.setEnabled(True)
        self.btn_cam_close.setEnabled(False)

    def _append_log(self, text):
        QTimer.singleShot(0, lambda: self.log_box.append(text))

    def _set_phase_label(self, text):
        QTimer.singleShot(0, lambda: self.lbl_phase.setText(f"Trạng thái: {text}"))

    # ---------------- Bias tự học: báo cáo delay thật lên vision ----------------
    def _report_actual_delay(self, kind, predicted_delay_s, t_capture_perf, t_actual_perf):
        """Đo actual_delay_s = mốc thời gian THẬT lúc gripper kích hoạt
        (t_actual_perf, tới từ on_grasp_moment callback trong
        move_run4dof.py) trừ mốc lúc camera bắt đầu xử lý frame
        (t_capture_perf, gắn kèm item bởi camera_run4dof.py), rồi báo cho
        bộ lọc EMA (report_actual_delay) để tự động hiệu chỉnh bias -
        không cần con người đo/canh lại. Import lazy + try/except để không
        phá luồng chính nếu vision module có vấn đề."""
        if predicted_delay_s is None or t_capture_perf is None:
            return
        actual_delay_s = t_actual_perf - t_capture_perf
        try:
            from vision.camera_run4dof import report_actual_delay
            report_actual_delay(kind, predicted_delay_s, actual_delay_s)
            self._append_log(
                f"↺ [BIAS-{kind.upper()}] dự đoán={predicted_delay_s:.3f}s "
                f"thật={actual_delay_s:.3f}s (lệch={actual_delay_s - predicted_delay_s:+.3f}s)"
            )
            self._update_bias_labels()
        except Exception as e:
            self._append_log(f"⚠ Không báo cáo được delay thật cho bias tự học: {e}")

    # ---------------- START / STOP vòng lặp gắp-thả tự động ----------------
    def on_start(self):
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

        obj_x = obj.get("x_mm_grasp", obj["x_mm"])
        obj_y = obj.get("y_mm_grasp", obj["y_mm"])
        obj_angle = obj.get("angle_deg_grasp", obj.get("angle_deg"))

        # Mốc + delay dự đoán gắn kèm item bởi camera_run4dof.py (xem
        # solve_rendezvous / item["_predicted_delay_s"], item["_t_capture_perf"]).
        # Dùng để đo delay THẬT sau khi gripper kích hoạt, phục vụ bias tự học.
        predicted_delay_s = obj.get("_predicted_delay_s")
        t_capture_perf = obj.get("_t_capture_perf")

        self._append_log(
            f"→ [PICK] Gắp vật (dự đoán) tại ({obj_x:.1f},{obj_y:.1f}) "
            f"góc {obj_angle:.1f}° - chưa cần khung."
        )

        point_a = (obj_x + offset_x, obj_y + offset_y)
        self._held_object_angle = obj_angle

        def on_grasp_moment(t_grasp_perf):
            # Được gọi TỪ THREAD WORKER (không phải main thread Qt) ngay
            # sau khi gripper thực sự kích hoạt trong move_run4dof.py.
            # _report_actual_delay chỉ tính toán + gọi hàm thuần Python ở
            # vision module (không đụng widget Qt trực tiếp) nên an toàn
            # gọi thẳng từ đây; log hiển thị qua _append_log đã tự đẩy về
            # main thread bằng QTimer.singleShot.
            self._report_actual_delay("pick", predicted_delay_s, t_capture_perf, t_grasp_perf)

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log(s))):
                self.ctx.planner_run4dof.pick_dof4(
                    point_a,
                    z_pick=z_pick,
                    gripper_callback=self.ctx.gripper_callback,
                    on_grasp_moment=on_grasp_moment,
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

        predicted_delay_s = hole.get("_predicted_delay_s")
        t_capture_perf = hole.get("_t_capture_perf")

        self._append_log(
            f"→ [PLACE] Đặt vào Lỗ #{hole.get('slot_id')} (dự đoán) "
            f"({hole_x:.1f},{hole_y:.1f}) góc {hole_angle:.1f}°"
        )

        place_point = (hole_x + offset_x, hole_y + offset_y)
        object_angle_deg = self._held_object_angle

        def on_grasp_moment(t_place_perf):
            self._report_actual_delay("place", predicted_delay_s, t_capture_perf, t_place_perf)

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log(s))):
                self.ctx.planner_run4dof.place_dof4(
                    place_point,
                    z_pick=z_pick,
                    gripper_callback=self.ctx.gripper_callback,
                    rotate_callback=self.ctx.pneu_uart.step_rotate,
                    object_angle_deg=object_angle_deg,
                    target_angle_deg=hole_angle,
                    on_grasp_moment=on_grasp_moment,
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

        if ok:
            # LƯU Ý: KHÔNG tự động lưu bias xuống robot_config.json ở đây
            # nữa. Bias vẫn tiếp tục học (EMA cập nhật qua report_actual_delay
            # ở on_grasp_moment của cả 2 pha), label + overlay trên camera
            # vẫn hiển thị giá trị LIVE mới nhất - nhưng chỉ được GHI XUỐNG
            # ĐĨA khi người dùng chủ động bấm nút "💾 LƯU BIAS HIỆN TẠI"
            # (on_save_bias_clicked). Mục đích: cho phép chạy thực nghiệm
            # tự do với nhiều bộ tham số khác nhau mà không lo bias "rác"
            # của lần thử trước bị ghi đè vào file cấu hình ngoài ý muốn.
            self.completed_cycles += 1
            self._refresh_recal_progress_label()
            if self._maybe_trigger_periodic_recalibration():
                return

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
        self.holding_object = False
        self._held_object_angle = None
        self._append_log(f"⚠ LỖI chu trình: {err} (đã reset trạng thái - kiểm tra tay gắp bằng mắt)")
        self._set_phase_label("LỖI - đang chờ thấy VẬT...")

    def on_stop(self):
        if not self.running and self._camera_thread is None:
            return
        self.running = False
        self.btn_stop.setEnabled(False)

        # LƯU Ý: KHÔNG tự động lưu bias khi bấm STOP nữa. Nếu muốn giữ lại
        # bias vừa học được trong phiên chạy này, bấm nút "💾 LƯU BIAS HIỆN
        # TẠI" TRƯỚC KHI bấm STOP (hoặc bất cứ lúc nào trong lúc đang chạy).
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
        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=10.0)
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