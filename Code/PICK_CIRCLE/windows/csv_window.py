import os
import time
import contextlib

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QTextEdit, QMessageBox, QCheckBox,
    QSlider,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, save_config,
    COORD_MODE_CIRCLE, CameraThread, cv2_to_qpixmap, home_and_wait, BASE_DIR,
)

# Các khóa tham số tốc độ mà thanh trượt SPEED sẽ scale (thời gian di
# chuyển càng NHỎ thì tốc độ càng NHANH, nên khi tăng % tốc độ ta CHIA
# thời gian gốc cho hệ số scale). CONTROL_HZ và MAX_STEP_MM không bị
# scale theo thanh trượt vì chúng quyết định ĐỘ MỊN/THẲNG của quỹ đạo,
# không phải tốc độ - giữ nguyên để mọi mức tốc độ đều thẳng như nhau.
_SPEED_SLIDER_TIME_KEYS = (
    "TIME_MOVE_FAST",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_ACTION",
    "TIME_DELAY_GRIPPER",
    "TIME_DELAY_DOF4",
)


# True: khi DỪNG, robot về HOME đã lưu (an toàn) rồi chạy homing của firmware
# (về công tắc hành trình) và CHỜ tín hiệu READY trước khi cho Bắt đầu lại.
# False: chỉ về HOME đã lưu bằng planner, không homing công tắc hành trình.
USE_FIRMWARE_HOME = True


class CsvWindow(BaseTabWindow):
    # Log/tiến trình phát từ luồng nền PHẢI đi qua Signal (QTimer.singleShot
    # trong QThread không có event loop nên không bao giờ chạy).
    log_signal = Signal(str)
    progress_signal = Signal(str)

    """
    STATIC TASK — LIVE CAMERA AUTO PICK & PLACE.

    Camera luôn bật và nhận diện liên tục (CameraThread ở chế độ "circle").
    Khi thấy vật TRẮNG và lỗ ĐEN tương ứng, robot gắp vật và thả vào lỗ,
    lặp lại cho đến khi nhấn DỪNG.

    Trình tự: Z safe cố định -> hạ Z gắp -> nhấc Z safe -> sang B -> hạ Z thả
    -> nhấc Z safe -> HOME.

    Khi DỪNG: camera tắt NGAY, chu trình gắp-thả đang chạy (nếu có) bị ngắt
    ngay (nhấc lên, về HOME đã lưu, tắt bơm), sau đó robot homing về công
    tắc hành trình và chờ READY rồi mới cho phép BẮT ĐẦU lại.

    Thanh trượt TỐC ĐỘ scale tốc độ di chuyển live mà không ảnh hưởng độ
    thẳng/mịn của quỹ đạo.
    """

    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🗂️ BÀI TOÁN TĨNH — CAMERA TRỰC TIẾP")
        self._running = False
        self._stop_flag = False
        self._loop_active = False   # True khi vòng gắp-thả tự động đang chạy
        self.camera_thread = None

        # Ghi lại tham số tốc độ GỐC (100%) của planner ngay khi window
        # được tạo, để thanh trượt luôn scale từ một mốc cố định.
        self._base_speed_params = self._capture_base_speed_params()

        self._build_ui()
        self.log_signal.connect(self.log_box_csv.append)
        self.progress_signal.connect(self.lbl_csv_progress.setText)
        self._restore_saved_speed_percent()

    def showEvent(self, event):
        super().showEvent(event)
        self._set_busy_ui(self.ctx.busy)

    # ------------------------------------------------------------------
    def _capture_base_speed_params(self):
        planner = self.ctx.planner_3dof
        return {
            key: getattr(planner, key)
            for key in _SPEED_SLIDER_TIME_KEYS
            if hasattr(planner, key)
        }

    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        # ---------------- LEFT: live camera preview ----------------
        left_col = QVBoxLayout()

        title = QLabel("CAMERA TRỰC TIẾP (NHẬN DIỆN HÌNH TRÒN)")
        title.setObjectName("sectionTitle")
        left_col.addWidget(title)

        self.video_label = QLabel("CAMERA ĐANG TẮT.\nNHẤN BẮT ĐẦU ĐỂ KHỞI ĐỘNG.")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setFixedSize(560, 420)
        self.video_label.setStyleSheet(
            "QLabel { background-color:#101010; color:#cccccc; border:2px solid #c0c0c0; border-radius:8px; }"
        )
        left_col.addWidget(self.video_label, 0, Qt.AlignTop | Qt.AlignHCenter)

        self.lbl_cam_status = QLabel("FPS: -- | ĐÃ PHÁT HIỆN: --")
        self.lbl_cam_status.setStyleSheet("color:#333333; font-size:11pt;")
        left_col.addWidget(self.lbl_cam_status)

        left_col.addStretch(1)

        # ---------------- RIGHT: controls + log ----------------
        right_col = QVBoxLayout()

        # ---- THANH TRƯỢT TỐC ĐỘ ----
        speed_box = QGroupBox("TỐC ĐỘ DI CHUYỂN")
        speed_box.setMaximumHeight(76)
        speed_box.setStyleSheet(
            "QGroupBox { margin-top:8px; padding-top:6px; }"
        )
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
        right_col.addWidget(speed_box)

        offset_box = QGroupBox("OFFSET CAMERA (mm)")
        offset_box.setMaximumHeight(86)
        offset_box.setStyleSheet(
            "QGroupBox { margin-top:8px; padding-top:6px; }"
            "QDoubleSpinBox { padding:4px 6px; }"
        )
        offset_layout = QHBoxLayout()
        offset_layout.setContentsMargins(10, 4, 10, 6)
        offset_layout.setSpacing(8)
        offset_layout.addWidget(QLabel("X:"))
        self.spin_csv_offset_x = QDoubleSpinBox()
        self.spin_csv_offset_x.setRange(-100, 100)
        self.spin_csv_offset_x.setValue(self.ctx.csv_offset_x)
        self.spin_csv_offset_x.setSingleStep(0.5)
        self.spin_csv_offset_x.valueChanged.connect(self._on_csv_offset_x_changed)
        offset_layout.addWidget(self.spin_csv_offset_x)
        offset_layout.addWidget(QLabel("Y:"))
        self.spin_csv_offset_y = QDoubleSpinBox()
        self.spin_csv_offset_y.setRange(-100, 100)
        self.spin_csv_offset_y.setValue(self.ctx.csv_offset_y)
        self.spin_csv_offset_y.setSingleStep(0.5)
        self.spin_csv_offset_y.valueChanged.connect(self._on_csv_offset_y_changed)
        offset_layout.addWidget(self.spin_csv_offset_y)
        offset_box.setLayout(offset_layout)
        right_col.addWidget(offset_box)

        self.place_box_circle = QGroupBox("GHÉP VẬT (TRẮNG) → LỖ (ĐEN)")
        place_layout = QGridLayout()
        place_layout.addWidget(QLabel("DUNG SAI ĐƯỜNG KÍNH (mm):"), 0, 0)
        self.spin_csv_tolerance = QDoubleSpinBox()
        self.spin_csv_tolerance.setRange(0.0, 50.0)
        self.spin_csv_tolerance.setSingleStep(0.5)
        self.spin_csv_tolerance.setValue(self.ctx.cfg.get("csv_match_tolerance", 6.0))
        place_layout.addWidget(self.spin_csv_tolerance, 0, 1)

        self.chk_csv_same_color = QCheckBox("CHỈ GHÉP CÙNG MÀU (ĐỎ↔ĐỎ, XANH↔XANH)")
        self.chk_csv_same_color.setChecked(bool(self.ctx.cfg.get("csv_match_same_color", True)))
        place_layout.addWidget(self.chk_csv_same_color, 1, 0, 1, 2)

        place_layout.addWidget(QLabel("Z GẮP:"), 2, 0)
        self.spin_csv_z_pick = QDoubleSpinBox()
        self.spin_csv_z_pick.setRange(0, 500)
        self.spin_csv_z_pick.setValue(self.ctx.cfg.get("csv_z_pick", 300.0))
        place_layout.addWidget(self.spin_csv_z_pick, 2, 1)

        place_layout.addWidget(QLabel("Z THẢ:"), 3, 0)
        self.spin_csv_z_place = QDoubleSpinBox()
        self.spin_csv_z_place.setRange(0, 500)
        self.spin_csv_z_place.setValue(
            self.ctx.cfg.get("csv_z_place", self.ctx.cfg.get("csv_z_pick", 300.0))
        )
        place_layout.addWidget(self.spin_csv_z_place, 3, 1)
        self.place_box_circle.setLayout(place_layout)
        right_col.addWidget(self.place_box_circle)

        run_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ BẮT ĐẦU")
        self.btn_start.setObjectName("actionBtn")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("⏹ DỪNG")
        self.btn_stop.setObjectName("disconnectBtn")
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)
        run_row.addWidget(self.btn_start, 3)
        run_row.addWidget(self.btn_stop, 1)
        right_col.addLayout(run_row)

        self.lbl_csv_progress = QLabel("SẴN SÀNG.")
        self.lbl_csv_progress.setWordWrap(True)
        self.lbl_csv_progress.setStyleSheet("color:#333333; font-weight:700; font-size:12pt;")
        right_col.addWidget(self.lbl_csv_progress)

        self.log_box_csv = QTextEdit()
        self.log_box_csv.setReadOnly(True)
        right_col.addWidget(self.log_box_csv, 4)

        root.addLayout(left_col, 3)
        root.addLayout(right_col, 2)

    def _set_busy_ui(self, busy):
        self.btn_start.setEnabled(not busy)
        # Thanh trượt tốc độ vẫn cho phép chỉnh ngay cả khi robot đang chạy,
        # để người dùng có thể tăng/giảm tốc "live" mà không cần dừng.

    # ------------------------------------------------------------------
    # SPEED SLIDER
    # ------------------------------------------------------------------
    def _restore_saved_speed_percent(self):
        """Khôi phục % tốc độ đã lưu lần trước (nếu có) khi mở lại app."""
        saved_percent = int(self.ctx.cfg.get("csv_speed_percent", 100))
        saved_percent = max(self.slider_speed.minimum(), min(self.slider_speed.maximum(), saved_percent))
        if saved_percent != 100:
            self.slider_speed.setValue(saved_percent)  # tự trigger _on_speed_slider_changed
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
        self.ctx.planner_3dof.update_speed_params(scaled_params)

        self.ctx.cfg["csv_speed_percent"] = value
        save_config(self.ctx.cfg)

    # ------------------------------------------------------------------
    def _match_white_black_pairs(self, points, same_color_only=True, tolerance_mm=6.0):
        whites = [p for p in points if "white" in str(p.get("type", ""))]
        blacks = [p for p in points if "black" in str(p.get("type", ""))]
        used_black_idx = set()
        pairs = []
        unmatched = []

        for w in whites:
            w_color = str(w.get("type", "")).split("_")[0]
            w_dia = float(w.get("diameter_mm", 0) or 0)
            best_idx, best_diff = None, None
            for i, b in enumerate(blacks):
                if i in used_black_idx:
                    continue
                b_color = str(b.get("type", "")).split("_")[0]
                if same_color_only and b_color != w_color:
                    continue
                diff = abs(float(b.get("diameter_mm", 0) or 0) - w_dia)
                if diff <= tolerance_mm and (best_diff is None or diff < best_diff):
                    best_diff, best_idx = diff, i
            if best_idx is not None:
                used_black_idx.add(best_idx)
                pairs.append((w, blacks[best_idx]))
            else:
                unmatched.append(w)

        return pairs, unmatched

    def _log_pick(self, w, b, point_a, point_b):
        self._append_log_threadsafe(
            f"→ ĐANG GẮP {w.get('type')} Ø{float(w.get('diameter_mm', 0) or 0):.1f}mm "
            f"tại ({point_a[0]:.1f},{point_a[1]:.1f})  →  đặt vào "
            f"{b.get('type')} Ø{float(b.get('diameter_mm', 0) or 0):.1f}mm "
            f"tại ({point_b[0]:.1f},{point_b[1]:.1f})"
        )

    def _on_csv_offset_x_changed(self, val):
        self.ctx.csv_offset_x = val
        self.ctx.cfg["csv_offset_x"] = val
        save_config(self.ctx.cfg)

    def _on_csv_offset_y_changed(self, val):
        self.ctx.csv_offset_y = val
        self.ctx.cfg["csv_offset_y"] = val
        save_config(self.ctx.cfg)

    # ------------------------------------------------------------------
    # CAMERA
    # ------------------------------------------------------------------
    def _start_camera(self):
        # Xóa dữ liệu cũ của lần chạy trước để vòng lặp không gắp theo
        # vị trí đã lỗi thời.
        self.ctx.latest_circles = []

        calib_file = self.ctx.cfg.get("calib_file")
        calib_path = os.path.join(BASE_DIR, calib_file) if calib_file else None

        self.camera_thread = CameraThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=calib_path,
            detect_mode=COORD_MODE_CIRCLE,
            brightness_step_down=2,
        )
        self.camera_thread.frame_ready.connect(self._on_camera_frame)
        self.camera_thread.error.connect(self._on_camera_error)
        self.camera_thread.start()

    def _stop_camera(self):
        """Tắt camera ngay lập tức. An toàn khi gọi nhiều lần."""
        cam = self.camera_thread
        self.camera_thread = None
        if cam:
            # Ngắt signal trước để frame cũ còn tồn đọng không vẽ đè lên label.
            for sig, slot in ((cam.frame_ready, self._on_camera_frame),
                              (cam.error, self._on_camera_error)):
                with contextlib.suppress(Exception):
                    sig.disconnect(slot)
            cam.stop()
            # KHÔNG dùng terminate(): kill thread đang giữ VideoCapture sẽ
            # làm cap.release() không chạy, camera bị kẹt sáng đèn.
            if not cam.wait(5000):
                self._append_log_threadsafe("⚠ Camera chưa dừng kịp sau 5s")
        self.ctx.latest_circles = []
        self.video_label.clear()
        self.video_label.setText("CAMERA ĐANG TẮT.\nNHẤN BẮT ĐẦU ĐỂ KHỞI ĐỘNG.")
        self.lbl_cam_status.setText("FPS: -- | ĐÃ PHÁT HIỆN: --")

    def _on_camera_frame(self, frame, items, fps):
        self.ctx.latest_circles = items
        pixmap = cv2_to_qpixmap(frame)
        scaled = pixmap.scaled(
            self.video_label.width(), self.video_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)
        self.lbl_cam_status.setText(f"FPS: {fps:.1f} | ĐÃ PHÁT HIỆN: {len(items)}")

    def _on_camera_error(self, msg):
        self._append_log_threadsafe(f"⚠ LỖI CAMERA: {msg}")

    # ------------------------------------------------------------------
    # START / STOP
    # ------------------------------------------------------------------
    def on_start(self):
        if self.ctx.busy:
            QMessageBox.warning(self, "Đang bận", "Robot đang thực hiện tác vụ khác.")
            return
        if self._running:
            return

        same_color_only = self.chk_csv_same_color.isChecked()
        tolerance = self.spin_csv_tolerance.value()
        z_pick = self.spin_csv_z_pick.value()
        z_place = self.spin_csv_z_place.value()
        offset_x, offset_y = self.ctx.csv_offset_x, self.ctx.csv_offset_y

        self.ctx.cfg["csv_match_tolerance"] = tolerance
        self.ctx.cfg["csv_match_same_color"] = same_color_only
        self.ctx.cfg["csv_z_pick"] = z_pick
        self.ctx.cfg["csv_z_place"] = z_place
        save_config(self.ctx.cfg)

        self._running = True
        self._stop_flag = False
        self.ctx.busy = True
        self._set_busy_ui(True)
        self.btn_stop.setEnabled(True)

        self.log_box_csv.clear()
        self.log_box_csv.append("=== ĐÃ NHẤN BẮT ĐẦU: đang di chuyển về vị trí HOME đã lưu ===")
        self.lbl_csv_progress.setText("Đang di chuyển về vị trí HOME đã lưu...")

        def home_first_task():
            planner = self.ctx.planner_3dof
            planner.clear_abort()
            # Đồng bộ HOME với vị trí đã lưu ở tab MANUAL.
            with contextlib.suppress(Exception):
                planner.set_home(*self.ctx.home_pos)
            # Dùng vị trí thực tế mà GUI đang theo dõi (jog tay...) làm điểm
            # xuất phát để nội suy mượt tới HOME đã lưu.
            # Nếu planner đang "không rõ vị trí" (vừa homing công tắc hành
            # trình) thì GIỮ nguyên trạng thái đó, không ghi đè bằng ctx.
            cur = getattr(self.ctx, "current_pos", None)
            if planner.current_pos is not None:
                if cur is not None and len(cur) == 3:
                    planner.current_pos = tuple(float(v) for v in cur)
                else:
                    planner.invalidate_position()
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                return planner.move_home()

        worker = FnWorker(home_first_task)
        worker.done_ok.connect(
            lambda ok: self._on_start_home_done(
                ok, same_color_only, tolerance, z_pick, z_place, offset_x, offset_y)
        )
        worker.done_err.connect(self._on_start_home_error)
        self._track_worker(worker)

    def _on_start_home_done(self, ok, same_color_only, tolerance, z_pick, z_place, offset_x, offset_y):
        self.ctx.current_pos = list(self.ctx.planner_3dof.HOME)

        if not ok:
            self.log_box_csv.append("⚠ Không thể về vị trí HOME đã lưu. Hủy khởi động.")
            self.lbl_csv_progress.setText("⚠ Không thể về HOME.")
            self.ctx.busy = False
            self._set_busy_ui(False)
            self._running = False
            self.btn_stop.setEnabled(False)
            return

        if self._stop_flag:
            # STOP được nhấn khi đang về HOME -> đi thẳng quy trình dừng.
            self._finish_stop()
            return

        self.log_box_csv.append("=== Đã về HOME. Bắt đầu camera + tự động gắp-thả ===")
        self.lbl_csv_progress.setText("Đang khởi động camera...")

        self._start_camera()

        self._loop_active = True
        worker = FnWorker(self._auto_loop, same_color_only, tolerance,
                          z_pick, z_place, offset_x, offset_y)
        worker.done_ok.connect(self._on_auto_loop_done)
        worker.done_err.connect(self._on_auto_loop_error)
        self._track_worker(worker)

    def _on_start_home_error(self, err):
        self.log_box_csv.append(f"⚠ LỖI khi di chuyển về HOME: {err}")
        self.lbl_csv_progress.setText("⚠ Lỗi khi di chuyển về HOME.")
        self.ctx.busy = False
        self._set_busy_ui(False)
        self._running = False
        self.btn_stop.setEnabled(False)

    def on_stop(self):
        if not self._running:
            return
        self._stop_flag = True
        self.btn_stop.setEnabled(False)
        # NGẮT NGAY chu trình gắp-thả đang chạy (nếu có): robot nhấc lên,
        # về HOME và tắt bơm, không chờ chạy hết chu trình.
        if self._loop_active:
            self.ctx.planner_3dof.request_abort()
        # Tắt camera NGAY. Chu trình gắp-thả đang chạy không cần camera nữa,
        # và _auto_loop sẽ thoát vì cờ dừng đã bật.
        try:
            self._stop_camera()
        except Exception as e:
            self.log_box_csv.append(f"⚠ Lỗi khi tắt camera: {e}")
        self.lbl_csv_progress.setText("⏹ Đã tắt camera. Đang dừng và về HOME...")

    def _auto_loop(self, same_color_only, tolerance, z_pick, z_place, offset_x, offset_y):
        # Cho camera khởi động trước khi tin dữ liệu nhận diện đầu tiên.
        time.sleep(1.0)

        while not self._stop_flag:
            items = list(self.ctx.latest_circles)
            if not items:
                time.sleep(0.2)
                continue

            pairs, _unmatched = self._match_white_black_pairs(
                items, same_color_only=same_color_only, tolerance_mm=tolerance
            )
            if not pairs:
                time.sleep(0.2)
                continue

            w, b = pairs[0]
            point_a = (w["x_mm"] + offset_x, w["y_mm"] + offset_y)
            point_b = (b["x_mm"] + offset_x, b["y_mm"] + offset_y)
            self._log_pick(w, b, point_a, point_b)

            self.progress_signal.emit(
                f"Đang gắp ({point_a[0]:.1f},{point_a[1]:.1f}) → "
                f"Đang đặt ({point_b[0]:.1f},{point_b[1]:.1f})"
            )

            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                self.ctx.planner_3dof.pick_and_place(
                    point_a, point_b, z_pick=z_pick, z_place=z_place,
                    gripper_callback=self.ctx.gripper_callback
                )

        return True

    def _append_log_threadsafe(self, text):
        self.log_signal.emit(str(text))

    def _on_auto_loop_done(self, ok):
        self._loop_active = False
        self.log_box_csv.append("=== VÒNG LẶP TỰ ĐỘNG ĐÃ DỪNG ===")
        self._finish_stop()

    def _on_auto_loop_error(self, err):
        self._loop_active = False
        self.log_box_csv.append(f"⚠ LỖI: {err}")
        self._finish_stop()

    def _finish_stop(self):
        # Bọc try/except: nếu tắt camera lỗi thì vẫn phải chạy tiếp bước về
        # HOME, tránh kẹt ctx.busy / _running = True.
        try:
            self._stop_camera()
        except Exception as e:
            self.log_box_csv.append(f"⚠ Lỗi khi tắt camera: {e}")

        if USE_FIRMWARE_HOME:
            self.lbl_csv_progress.setText("Đang về HOME (công tắc hành trình) và chờ READY...")
            self.log_box_csv.append("=== ĐANG VỀ HOME — ĐANG CHỜ READY ===")
        else:
            self.lbl_csv_progress.setText("Đang về HOME...")
            self.log_box_csv.append("=== ĐANG VỀ HOME ===")

        def home_task():
            planner = self.ctx.planner_3dof
            planner.clear_abort()  # xóa cờ ngắt còn sót để lệnh về HOME chạy được
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                ok = planner.move_home()
            if not USE_FIRMWARE_HOME:
                time.sleep(0.3)  # chờ robot ổn định tại HOME đã lưu
                return ok
            self._append_log_threadsafe("→ Gửi lệnh HOME (công tắc hành trình), đang chờ READY...")
            ready = home_and_wait(self.ctx.robot_uart)
            planner.invalidate_position()  # firmware đã tự đưa robot đi chỗ khác
            self._append_log_threadsafe(
                "→ Đã nhận READY." if ready else "→ KHÔNG nhận được READY (hết thời gian chờ)."
            )
            return ready

        worker = FnWorker(home_task)
        worker.done_ok.connect(self._on_home_done)
        worker.done_err.connect(self._on_home_error)
        self._track_worker(worker)

    def _on_home_done(self, ready_ok):
        self.ctx.busy = False
        self._set_busy_ui(False)
        self._running = False
        self.btn_stop.setEnabled(False)
        self.ctx.current_pos = list(self.ctx.planner_3dof.HOME)
        if ready_ok:
            if USE_FIRMWARE_HOME:
                self.lbl_csv_progress.setText("✔ Đã dừng. Camera đã tắt, robot đã HOME và READY.")
                self.log_box_csv.append("=== HOME + READY ===")
            else:
                self.lbl_csv_progress.setText("✔ Đã dừng. Camera đã tắt, robot đang ở HOME.")
                self.log_box_csv.append("=== ĐÃ VỀ HOME ===")
        else:
            if USE_FIRMWARE_HOME:
                self.lbl_csv_progress.setText("⚠ Đã dừng, nhưng chưa nhận được READY từ robot.")
                self.log_box_csv.append("⚠ Chưa nhận được READY (xem log phía trên).")
            else:
                self.lbl_csv_progress.setText("⚠ Đã dừng, nhưng về HOME không thành công.")
                self.log_box_csv.append("⚠ Về HOME không thành công (xem log phía trên).")

    def _on_home_error(self, err):
        self.ctx.busy = False
        self._set_busy_ui(False)
        self._running = False
        self.btn_stop.setEnabled(False)
        # Lỗi giữa chừng -> không biết robot đang ở đâu.
        with contextlib.suppress(Exception):
            self.ctx.planner_3dof.invalidate_position()
        self.log_box_csv.append(f"⚠ LỖI khi về HOME: {err}")
        self.lbl_csv_progress.setText("⚠ Lỗi khi về HOME.")