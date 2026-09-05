import contextlib
import os
import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QGroupBox,
    QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem, QHeaderView,
    QMessageBox, QSizePolicy,
)

# MỚI
from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, cv2_to_qpixmap,
    save_config, home_and_wait, DYN_COLUMNS, _fmt_angle_col,
)
from vision.Camera_4dof import (
    detect_frame_holes, detect_objects, undistort_image,
    DET_SCALE, _build_red_mask, required_dof4_rotation,
)

# ==================== LUỒNG CAMERA CHO BÀI TOÁN ĐỘNG (có ROI + vẽ) ====================
class DynamicCameraThread(QThread):
    """
    Luồng đọc camera, xử lý ảnh với ROI cho vật, vẽ các annotation lên frame
    (giống như trong run_live_display của Camera_4dof) và phát ra frame đã vẽ.
    """
    frame_ready = Signal(object, list, float)   # frame (đã vẽ), items, fps
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index, calib_path):
        super().__init__()
        self.camera_index = camera_index
        self.calib_path = calib_path
        self._running = False
        self._cap = None
        self._calib_data = None

    def load_calib(self):
        try:
            data = np.load(self.calib_path)
            self._calib_data = (data['camera_matrix'], data['dist_coeffs'])
            return True
        except Exception as e:
            self.error.emit(f"Không thể load calib: {e}")
            return False

    def run(self):
        if not self.load_calib():
            return
        self._cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(self.camera_index)
        if not self._cap.isOpened():
            self.error.emit(f"Không thể mở camera {self.camera_index}")
            return
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._running = True
        camera_matrix, dist_coeffs = self._calib_data
        fps = 0.0
        frame_count = 0
        start_time = cv2.getTickCount()

        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                break

            # Undistort
            undistorted, new_cm = undistort_image(frame, camera_matrix, dist_coeffs)
            fx, fy = new_cm[0, 0], new_cm[1, 1]
            cx, cy = new_cm[0, 2], new_cm[1, 2]

            # Tính ROI: ô vuông ở giữa, kích thước = 1/3 chiều nhỏ nhất
            h, w = undistorted.shape[:2]
            roi_size = min(w, h) // 3
            roi = (w//2 - roi_size//2, h//2 - roi_size//2, roi_size, roi_size)

            # Chuẩn bị mask (dùng chung cho cả frame và vật)
            small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
            small_min_area = max(20, int(round(150 * DET_SCALE * DET_SCALE)))
            red_mask = _build_red_mask(small, min_component_area=small_min_area)
            upscale = 1.0 / DET_SCALE

            # Phát hiện khung (không dùng ROI)
            frame_result = detect_frame_holes(undistorted, cx, cy, fx, fy, mask=red_mask, upscale=upscale)

            # Phát hiện vật (có ROI)
            objects = detect_objects(
                undistorted, cx, cy, fx, fy,
                mask=red_mask,
                upscale=upscale,
                refine_frame=undistorted,
                roi=roi
            )

            # --- Bắt đầu vẽ lên ảnh (giống hệt run_live_display) ---
            display = undistorted.copy()

            # Vẽ tâm (điểm gốc)
            origin_px = (int(round(cx)), int(round(cy)))
            cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            # Vẽ ROI
            cv2.rectangle(display, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (255, 0, 0), 2)
            cv2.putText(display, "Vung tim vat", (roi[0], roi[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # Vẽ các vật thể
            for o in objects:
                center_px = (int(o['cx_px']), int(o['cy_px']))
                rot_needed = required_dof4_rotation(o['angle_deg'], target_angle_deg=90.0)
                cv2.circle(display, center_px, 3, (0, 255, 0), -1)
                txt1 = f"X:{o['x_mm']:.1f} Y:{o['y_mm']:.1f} mm  Goc:{o['angle_deg']:.1f}deg"
                txt2 = f"{o['width_mm']:.1f}x{o['height_mm']:.1f}mm  Xoay_can:{rot_needed:.1f}deg"
                cv2.putText(display, txt1, (center_px[0] - 90, center_px[1] - 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(display, txt2, (center_px[0] - 90, center_px[1] - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            # Vẽ thông tin khung và các lỗ
            if not frame_result['frame_found']:
                cv2.putText(display, "Khong thay khung", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                for h in frame_result['holes']:
                    center_px = (int(round(h['cx_px'])), int(round(h['cy_px'])))
                    cv2.circle(display, center_px, 4, (0, 255, 255), -1)
                    cv2.putText(display, f"#{h['slot_id']}", (center_px[0] - 10, center_px[1] + 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    txt = f"({h['x_mm']:.1f},{h['y_mm']:.1f}) {h['angle_deg']:.0f}deg"
                    cv2.putText(display, txt, (center_px[0] - 70, center_px[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cv2.putText(
                    display,
                    f"Khung: {frame_result['empty_count']}/{frame_result['expected_count']} lo trong",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
                )

            # Thêm FPS và độ sáng (tuỳ chọn, để giống hệt Camera_4dof)
            cv2.putText(display, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # Không có điều chỉnh độ sáng ở đây, nhưng vẫn có thể thêm
            # cv2.putText(display, f"Do sang: ...", ...)

            # Tạo danh sách items (kết hợp khung + vật) để hiển thị trên bảng
            items = []
            if frame_result['frame_found']:
                for h in frame_result['holes']:
                    items.append(h)
            for o in objects:
                items.append(o)

            # Tính FPS
            frame_count += 1
            if frame_count >= 10:
                end_time = cv2.getTickCount()
                seconds = (end_time - start_time) / cv2.getTickFrequency()
                fps = frame_count / seconds if seconds > 0 else 0.0
                frame_count = 0
                start_time = cv2.getTickCount()

            self.frame_ready.emit(display, items, fps)

        self._cap.release()
        self.stopped.emit()

    def stop(self):
        self._running = False
        self.wait()


# ==================== CỬA SỔ BÀI TOÁN ĐỘNG ====================
class DynamicWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🧩 BÀI TOÁN BẬC 4 TĨNH")
        self._build_ui()

    def showEvent(self, event):
        super().showEvent(event)
        self.btn_dyn_start.setEnabled(not self.ctx.dynamic_running)
        self.btn_dyn_stop.setEnabled(self.ctx.dynamic_running)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        video_col = QVBoxLayout()
        self.video_label_dyn = QLabel("Camera chưa kết nối")
        self.video_label_dyn.setAlignment(Qt.AlignCenter)
        self.video_label_dyn.setMinimumSize(720, 480)
        self.video_label_dyn.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; border-radius:10px; color:#888888;"
        )
        self.video_label_dyn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label_dyn, 5)

        self.lbl_fps_dyn = QLabel("FPS: --")
        self.lbl_fps_dyn.setStyleSheet("color:#333333; font-weight:700;")
        video_col.addWidget(self.lbl_fps_dyn)

        self.dyn_table = QTableWidget(0, len(DYN_COLUMNS))
        self.dyn_table.setHorizontalHeaderLabels(DYN_COLUMNS)
        self.dyn_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.dyn_table.setMaximumHeight(180)
        video_col.addWidget(self.dyn_table, 2)

        ctrl_col = QVBoxLayout()

        info_box = QGroupBox("LOGIC TỰ ĐỘNG")
        info_box.setObjectName("compactBox")
        info_layout = QVBoxLayout()
        info_lbl = QLabel(
            "Camera nhận diện đồng thời: vật rời (khối đặc đỏ 10x20mm) và khuôn "
            "8 lỗ (viền đỏ, rỗng) - cả hai đều được nhận diện lại mỗi khung hình. "
            "Robot tự động gắp từng vật rồi thả đúng vào lỗ trống kế tiếp của "
            "khuôn, xoay bậc tự do 4 theo ĐÚNG góc của lỗ đó (không dùng góc cố "
            "định). Không cần CSV - máy tự chạy liên tục khi còn vật và còn lỗ trống."
        )
        info_lbl.setWordWrap(True)
        info_lbl.setStyleSheet("color:#333333; font-size:9pt;")
        info_layout.addWidget(info_lbl)

        zrow = QHBoxLayout()
        zrow.addWidget(QLabel("Z pick:"))
        self.spin_dyn_z_pick = QDoubleSpinBox()
        self.spin_dyn_z_pick.setRange(0, 500)
        self.spin_dyn_z_pick.setValue(self.ctx.cfg.get("dyn_z_pick", 340.0))
        self.spin_dyn_z_pick.valueChanged.connect(self._on_dyn_zpick_changed)
        zrow.addWidget(self.spin_dyn_z_pick)
        info_layout.addLayout(zrow)

        offset_layout = QHBoxLayout()
        offset_layout.addWidget(QLabel("Offset X:"))
        self.spin_dyn_offset_x = QDoubleSpinBox()
        self.spin_dyn_offset_x.setRange(-100, 100)
        self.spin_dyn_offset_x.setSingleStep(0.5)
        self.spin_dyn_offset_x.setValue(self.ctx.dyn_offset_x)
        self.spin_dyn_offset_x.valueChanged.connect(self._on_dyn_offset_x_changed)
        offset_layout.addWidget(self.spin_dyn_offset_x)

        offset_layout.addWidget(QLabel("Y:"))
        self.spin_dyn_offset_y = QDoubleSpinBox()
        self.spin_dyn_offset_y.setRange(-100, 100)
        self.spin_dyn_offset_y.setSingleStep(0.5)
        self.spin_dyn_offset_y.setValue(self.ctx.dyn_offset_y)
        self.spin_dyn_offset_y.valueChanged.connect(self._on_dyn_offset_y_changed)
        offset_layout.addWidget(self.spin_dyn_offset_y)

        info_layout.addLayout(offset_layout)

        info_box.setLayout(info_layout)
        ctrl_col.addWidget(info_box)

        start_stop_row = QHBoxLayout()
        self.btn_dyn_start = QPushButton("▶ START")
        self.btn_dyn_start.setObjectName("dynStartBtn")
        self.btn_dyn_start.clicked.connect(self.on_dynamic_start)
        self.btn_dyn_stop = QPushButton("■ STOP")
        self.btn_dyn_stop.setObjectName("dynStopBtn")
        self.btn_dyn_stop.clicked.connect(self.on_dynamic_stop)
        self.btn_dyn_stop.setEnabled(False)
        start_stop_row.addWidget(self.btn_dyn_start)
        start_stop_row.addWidget(self.btn_dyn_stop)
        ctrl_col.addLayout(start_stop_row)

        self.log_box_dyn = QTextEdit()
        self.log_box_dyn.setReadOnly(True)
        ctrl_col.addWidget(self.log_box_dyn, 3)

        root.addLayout(video_col, 3)
        root.addLayout(ctrl_col, 2)

    def _on_dyn_zpick_changed(self, val):
        self.ctx.cfg["dyn_z_pick"] = val
        save_config(self.ctx.cfg)

    def _on_dyn_offset_x_changed(self, val):
        self.ctx.dyn_offset_x = val
        self.ctx.cfg["dyn_offset_x"] = val
        save_config(self.ctx.cfg)

    def _on_dyn_offset_y_changed(self, val):
        self.ctx.dyn_offset_y = val
        self.ctx.cfg["dyn_offset_y"] = val
        save_config(self.ctx.cfg)

    def _append_log_threadsafe_dyn(self, text):
        QTimer.singleShot(0, lambda: self.log_box_dyn.append(text))

    def on_dynamic_start(self):
        if self.ctx.dynamic_running:
            return
        if not self.ctx.robot_uart.is_connected:
            QMessageBox.warning(self, "Chưa kết nối Robot",
                                 "Hãy kết nối UART 1 (Robot) ở tab KẾT NỐI trước.")
            return
        if self.ctx.camera_thread is not None:
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Tab CAMERA đang mở camera - hãy bấm DỪNG CAMERA ở tab đó trước.")
            return

        self.log_box_dyn.clear()
        self.log_box_dyn.append("=== START: đang di chuyển về Home đã lưu... ===")
        self.btn_dyn_start.setEnabled(False)
        self.ctx.busy = True

        hx, hy, hz = self.ctx.home_pos
        worker = FnWorker(self.ctx.planner_dof4.send_position, hx, hy, hz)
        worker.done_ok.connect(self._on_dynamic_home_done)
        worker.done_err.connect(self._on_dynamic_start_error)
        self._track_worker(worker)

    def _on_dynamic_home_done(self, ok):
        self.ctx.busy = False
        if not ok:
            self._append_log_threadsafe_dyn("⚠ Về Home thất bại - không thể bắt đầu.")
            self.btn_dyn_start.setEnabled(True)
            return
        self.ctx.current_pos = list(self.ctx.home_pos)
        self._append_log_threadsafe_dyn("✔ Đã về Home. Bắt đầu camera + vòng lặp tự động...")
        self.ctx.dynamic_running = True
        self.btn_dyn_stop.setEnabled(True)
        self._start_dynamic_camera()

    def _on_dynamic_start_error(self, err):
        self.ctx.busy = False
        self._append_log_threadsafe_dyn(f"⚠ LỖI: {err}")
        self.btn_dyn_start.setEnabled(True)

    def _start_dynamic_camera(self):
        if self.ctx.dynamic_camera_thread is not None:
            return
        self.ctx.dynamic_camera_thread = DynamicCameraThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=self.ctx.cfg.get("calib_file"),
        )
        self.ctx.dynamic_camera_thread.frame_ready.connect(self._on_dynamic_frame)
        self.ctx.dynamic_camera_thread.error.connect(self._on_dynamic_camera_error)
        self.ctx.dynamic_camera_thread.stopped.connect(self._on_dynamic_camera_stopped)
        self.ctx.dynamic_camera_thread.start()

    def _stop_dynamic_camera(self):
        if self.ctx.dynamic_camera_thread:
            self.ctx.dynamic_camera_thread.stop()

    def _on_dynamic_camera_stopped(self):
        self.ctx.dynamic_camera_thread = None
        self.video_label_dyn.setText("Camera đã dừng")

    def _on_dynamic_camera_error(self, msg):
        self._append_log_threadsafe_dyn(f"⚠ Lỗi Camera: {msg}")

    def _dyn_frame_counter(self):
        if not hasattr(self, "_fc_dyn"):
            self._fc_dyn = 0
        self._fc_dyn += 1
        return self._fc_dyn

    def _on_dynamic_frame(self, frame, items, fps):
        self.ctx.latest_dyn_objects = [it for it in items if it.get("type") == "object"]
        self.ctx.latest_dyn_holes = [it for it in items if it.get("type") == "hole"]

        pix = cv2_to_qpixmap(frame)
        self.video_label_dyn.setPixmap(
            pix.scaled(self.video_label_dyn.width(), self.video_label_dyn.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps_dyn.setText(f"FPS: {fps:.1f}")

        if self._dyn_frame_counter() % 3 == 0:
            rows = self.ctx.latest_dyn_objects + self.ctx.latest_dyn_holes
            self.dyn_table.setRowCount(len(rows))
            for i, c in enumerate(rows):
                self.dyn_table.setItem(i, 0, QTableWidgetItem("Vật" if c["type"] == "object" else "Lỗ"))
                slot = c.get("slot_id")
                self.dyn_table.setItem(i, 1, QTableWidgetItem(str(slot) if slot is not None else "-"))
                self.dyn_table.setItem(i, 2, QTableWidgetItem(f'{c["x_mm"]:.2f}'))
                self.dyn_table.setItem(i, 3, QTableWidgetItem(f'{c["y_mm"]:.2f}'))
                self.dyn_table.setItem(i, 4, QTableWidgetItem(_fmt_angle_col(c)))

        self._dynamic_maybe_start_cycle()

    def _dynamic_maybe_start_cycle(self):
        if not self.ctx.dynamic_running or self.ctx.dynamic_busy or self.ctx.busy:
            return
        if not self.ctx.latest_dyn_objects or not self.ctx.latest_dyn_holes:
            return

        obj = self.ctx.latest_dyn_objects[0]
        hole = self.ctx.latest_dyn_holes[0]

        self.ctx.dynamic_busy = True
        self.ctx.busy = True

        self._append_log_threadsafe_dyn(
            f"→ Gắp vật tại ({obj['x_mm']:.1f},{obj['y_mm']:.1f}) góc {obj['angle_deg']:.1f}° "
            f"→ đặt vào Lỗ #{hole['slot_id']} ({hole['x_mm']:.1f},{hole['y_mm']:.1f}) góc {hole['angle_deg']:.1f}°"
        )

        point_a = (obj["x_mm"] + self.ctx.dyn_offset_x, obj["y_mm"] + self.ctx.dyn_offset_y)
        place_point = (hole["x_mm"] + self.ctx.dyn_offset_x, hole["y_mm"] + self.ctx.dyn_offset_y)

        obj_angle = obj.get("angle_deg")
        hole_angle = hole.get("angle_deg", 90.0)
        z_pick = self.spin_dyn_z_pick.value()

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe_dyn(s))):
                self.ctx.planner_dof4.pick_and_place_dof4(
                    point_a,
                    z_pick=z_pick,
                    gripper_callback=self.ctx.gripper_callback,
                    rotate_callback=self.ctx.pneu_uart.step_rotate,
                    object_angle_deg=obj_angle,
                    place_point=place_point,
                    target_angle_deg=hole_angle,
                )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_dynamic_cycle_done)
        worker.done_err.connect(self._on_dynamic_cycle_error)
        self._track_worker(worker)

    def _on_dynamic_cycle_done(self, ok):
        self.ctx.dynamic_busy = False
        self.ctx.busy = False
        self.ctx.current_pos = list(self.ctx.planner_dof4.HOME)
        self._append_log_threadsafe_dyn("✔ Hoàn tất 1 chu trình gắp-thả." if ok else "⏹ Chu trình dừng giữa chừng.")

    def _on_dynamic_cycle_error(self, err):
        self.ctx.dynamic_busy = False
        self.ctx.busy = False
        self._append_log_threadsafe_dyn(f"⚠ LỖI chu trình: {err}")

    def on_dynamic_stop(self):
        if not self.ctx.dynamic_running and self.ctx.dynamic_camera_thread is None:
            return
        self.ctx.dynamic_running = False
        self.btn_dyn_stop.setEnabled(False)
        self._append_log_threadsafe_dyn(
            "=== STOP: dừng vòng lặp, chờ chu trình hiện tại (nếu có) hoàn tất rồi về HOME vật lý... ==="
        )
        self._stop_dynamic_camera()
        self._dynamic_wait_then_home()

    def _dynamic_wait_then_home(self):
        if self.ctx.dynamic_busy:
            QTimer.singleShot(300, self._dynamic_wait_then_home)
            return
        self._append_log_threadsafe_dyn("↻ Đang về HOME vật lý (chạm công tắc hành trình) và chờ READY...")
        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_dynamic_stop_home_done)
        worker.done_err.connect(self._on_dynamic_start_error)
        self._track_worker(worker)

    def _on_dynamic_stop_home_done(self, ready):
        if ready:
            self._append_log_threadsafe_dyn("✔ Đã về HOME xong (READY).")
        else:
            self._append_log_threadsafe_dyn("⚠ Không nhận được READY từ STM32.")
        self.btn_dyn_start.setEnabled(True)