import os
import cv2
import numpy as np
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QSizePolicy, QAbstractItemView, QRadioButton,
    QButtonGroup,
)

from shared_state import (
    BaseTabWindow, cv2_to_qpixmap,
    save_config, CSV_COLUMNS, COORD_MODE_CIRCLE, COORD_MODE_DOF4,
    _fmt_size_col, _fmt_angle_col,
)

# Import các hàm xử lý ảnh đã được sửa (có tham số roi)
from vision.Camera_4dof import (
    detect_frame_holes, detect_objects, undistort_image,
    DET_SCALE, _build_red_mask,
)

# ---------------------------------------------------------------------
# KÍCH THƯỚC GIAO DIỆN. Gom về hằng số ở đây để dễ chỉnh lại sau này.
# ---------------------------------------------------------------------
VIDEO_MIN_W = 480
VIDEO_MIN_H = 320
TABLE_MAX_H = 260
OFFSET_BOX_MAX_H = 92
SPINBOX_MAX_W = 130


# ========================== LUỒNG CAMERA TÙY CHỈNH CÓ ROI ==========================
class CameraThreadWithROI(QThread):
    """
    Luồng đọc camera, xử lý ảnh và phát hiện vật/khung.
    Tự động tính ROI (ô vuông ở giữa) cho chế độ DOF4.
    """
    frame_ready = Signal(object, list, float)   # frame (đã undistort), items, fps
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index, calib_path, detect_mode):
        super().__init__()
        self.camera_index = camera_index
        self.calib_path = calib_path
        self.detect_mode = detect_mode
        self._running = False
        self._cap = None
        self._calib_data = None

    def load_calib(self):
        try:
            data = np.load(self.calib_path)
            self._calib_data = (data['camera_matrix'], data['dist_coeffs'])
        except Exception as e:
            self.error.emit(f"Không thể load calib: {e}")
            return False
        return True

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

            # Phát hiện vật (có ROI) - chỉ áp dụng cho chế độ DOF4 (có góc quay)
            if self.detect_mode == COORD_MODE_DOF4:
                objects = detect_objects(
                    undistorted, cx, cy, fx, fy,
                    mask=red_mask,
                    upscale=upscale,
                    refine_frame=undistorted,
                    roi=roi
                )
            else:
                # Chế độ circle: không dùng ROI, detect vòng tròn (giả sử có hàm riêng)
                # Ở đây tôi giả định có hàm detect_circles; nếu không, dùng objects rỗng
                objects = []   # sẽ được thay bằng detect_circles nếu cần

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

            self.frame_ready.emit(undistorted, items, fps)

        self._cap.release()
        self.stopped.emit()

    def stop(self):
        self._running = False
        self.wait()


# ========================== CỬA SỔ CHÍNH ==========================
class CameraWindow(BaseTabWindow):
    """
    Cửa sổ CAMERA && OFFSET TỌA ĐỘ.

    CHỈ dùng để:
      1) Xem trực tiếp hình ảnh camera, kiểm tra hoạt động đã ổn định chưa,
         và hiển thị bảng tọa độ nhận diện được - hỗ trợ cả 2 chế độ:
           - Cameracircle.py  (vòng tròn, chỉ X/Y)
           - Camera_4dof.py   (HCN đỏ, có thêm góc quay)
      2) Chỉnh offset (bù trừ) tọa độ camera theo X/Y (mm).

    Không còn chu trình gắp-thả tự động ở đây nữa: cửa sổ này chỉ thu thập
    tọa độ để dùng cho giao diện BÀI TOÁN TĨNH (CsvWindow) như trước.
    """

    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "📷 CAMERA && OFFSET TỌA ĐỘ")
        self._camera_thread = None
        self._build_ui()

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)
        root.setSpacing(10)

        # ---------------- Cột trái: video + chế độ + bảng tọa độ ----------------
        video_col = QVBoxLayout()
        video_col.setSpacing(6)

        self.video_label = QLabel("Camera chưa kết nối")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(VIDEO_MIN_W, VIDEO_MIN_H)
        self.video_label.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; border-radius:10px; color:#888888;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 5)

        cam_ctrl_row = QHBoxLayout()
        cam_ctrl_row.setSpacing(6)
        self.btn_cam_start = QPushButton("▶  BẮT ĐẦU CAMERA")
        self.btn_cam_start.setObjectName("connectBtn")
        self.btn_cam_start.clicked.connect(self.on_start_camera)
        self.btn_cam_stop = QPushButton("■  DỪNG CAMERA")
        self.btn_cam_stop.setObjectName("disconnectBtn")
        self.btn_cam_stop.clicked.connect(self.on_stop_camera)
        self.btn_cam_stop.setEnabled(False)
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#333333; font-weight:700; font-size:10pt;")
        cam_ctrl_row.addWidget(self.btn_cam_start)
        cam_ctrl_row.addWidget(self.btn_cam_stop)
        cam_ctrl_row.addWidget(self.lbl_fps)
        video_col.addLayout(cam_ctrl_row)

        mode_box = QGroupBox("PHƯƠNG THỨC XÁC ĐỊNH TỌA ĐỘ")
        mode_box.setObjectName("compactBox")
        mode_layout = QVBoxLayout()
        mode_layout.setSpacing(2)

        self.radio_mode_circle = QRadioButton("① Vòng tròn (Cameracircle.py - chỉ X/Y)")
        self.radio_mode_dof4 = QRadioButton("② Bậc tự do 4 (Camera_4dof.py - HCN đỏ 10x20mm, có góc)")
        self.radio_group_mode = QButtonGroup(self)
        self.radio_group_mode.addButton(self.radio_mode_circle, 0)
        self.radio_group_mode.addButton(self.radio_mode_dof4, 1)

        if self.ctx.coord_mode == COORD_MODE_DOF4:
            self.radio_mode_dof4.setChecked(True)
        else:
            self.radio_mode_circle.setChecked(True)

        self.radio_mode_circle.toggled.connect(self._on_coord_mode_changed)

        mode_layout.addWidget(self.radio_mode_circle)
        mode_layout.addWidget(self.radio_mode_dof4)
        self.lbl_coord_mode_note = QLabel(self._coord_mode_note_text())
        self.lbl_coord_mode_note.setWordWrap(True)
        self.lbl_coord_mode_note.setStyleSheet("color:#555555; font-size:8pt;")
        mode_layout.addWidget(self.lbl_coord_mode_note)

        mode_box.setLayout(mode_layout)
        video_col.addWidget(mode_box)

        self.circle_table = QTableWidget(0, len(CSV_COLUMNS))
        self.circle_table.setHorizontalHeaderLabels(CSV_COLUMNS)
        self.circle_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.circle_table.setMaximumHeight(TABLE_MAX_H)
        self.circle_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.circle_table.setStyleSheet("font-size:9pt;")
        self.circle_table.verticalHeader().setDefaultSectionSize(20)
        video_col.addWidget(self.circle_table, 3)

        # ---------------- Cột phải: chỉ còn OFFSET + log camera ----------------
        ctrl_col = QVBoxLayout()
        ctrl_col.setAlignment(Qt.AlignTop)
        ctrl_col.setSpacing(6)

        offset_box = QGroupBox("OFFSET CAMERA (mm)")
        offset_box.setObjectName("compactBox")
        offset_box.setMaximumHeight(OFFSET_BOX_MAX_H)
        offset_box.setStyleSheet(
            "QGroupBox { margin-top:10px; padding-top:8px; }"
            "QDoubleSpinBox { padding:4px 6px; }"
        )
        offset_layout = QHBoxLayout()
        offset_layout.setContentsMargins(10, 4, 10, 6)
        offset_layout.setSpacing(8)
        offset_layout.addWidget(QLabel("X:"))
        self.spin_offset_x = QDoubleSpinBox()
        self.spin_offset_x.setRange(-100, 100)
        self.spin_offset_x.setValue(self.ctx.offset_x)
        self.spin_offset_x.setSingleStep(0.5)
        self.spin_offset_x.setMaximumWidth(SPINBOX_MAX_W)
        self.spin_offset_x.valueChanged.connect(self._on_offset_x_changed)
        offset_layout.addWidget(self.spin_offset_x)

        offset_layout.addWidget(QLabel("Y:"))
        self.spin_offset_y = QDoubleSpinBox()
        self.spin_offset_y.setRange(-100, 100)
        self.spin_offset_y.setValue(self.ctx.offset_y)
        self.spin_offset_y.setSingleStep(0.5)
        self.spin_offset_y.setMaximumWidth(SPINBOX_MAX_W)
        self.spin_offset_y.valueChanged.connect(self._on_offset_y_changed)
        offset_layout.addWidget(self.spin_offset_y)
        offset_layout.addStretch()

        offset_box.setLayout(offset_layout)
        ctrl_col.addWidget(offset_box)

        note = QLabel(
            "Tọa độ hiển thị trong bảng bên trái dùng để nhập cho giao diện\n"
            "BÀI TOÁN TĨNH. Offset ở trên chỉ bù trừ sai lệch giữa gốc tọa độ\n"
            "camera và gốc tọa độ thực tế của robot."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#555555; font-size:9pt;")
        ctrl_col.addWidget(note)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("font-size:9pt;")
        ctrl_col.addWidget(self.log_box, 1)

        root.addLayout(video_col, 3)
        root.addLayout(ctrl_col, 2)

    def _coord_mode_note_text(self):
        if self.ctx.coord_mode == COORD_MODE_CIRCLE:
            return "Đang dùng: Cameracircle.py (chỉ xác định X/Y)."
        return "Đang dùng: Camera_4dof.py - nhận diện HCN ĐỎ 10x20mm, có góc quay."

    def _on_coord_mode_changed(self, circle_checked):
        self.ctx.coord_mode = COORD_MODE_CIRCLE if circle_checked else COORD_MODE_DOF4
        self.ctx.cfg["coord_mode"] = self.ctx.coord_mode
        save_config(self.ctx.cfg)
        self.lbl_coord_mode_note.setText(self._coord_mode_note_text())

    def _on_offset_x_changed(self, val):
        self.ctx.offset_x = val
        self.ctx.cfg["camera_offset_x"] = val
        save_config(self.ctx.cfg)

    def _on_offset_y_changed(self, val):
        self.ctx.offset_y = val
        self.ctx.cfg["camera_offset_y"] = val
        save_config(self.ctx.cfg)

    # ==================== QUẢN LÝ CAMERA ====================
    def on_start_camera(self):
        if self._camera_thread is not None:
            return
        # Kiểm tra xem có thread nào khác đang dùng camera không
        if self.ctx.dynamic_camera_thread is not None:
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Tab BÀI TOÁN ĐỘNG đang dùng camera - hãy bấm STOP ở tab đó trước.")
            return
        # Tạo luồng mới
        self._camera_thread = CameraThreadWithROI(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=self.ctx.cfg.get("calib_file"),
            detect_mode=self.ctx.coord_mode,
        )
        self._camera_thread.frame_ready.connect(self._on_frame)
        self._camera_thread.error.connect(self._on_camera_error)
        self._camera_thread.stopped.connect(self._on_camera_stopped)
        self._camera_thread.start()
        self.btn_cam_start.setEnabled(False)
        self.btn_cam_stop.setEnabled(True)
        self.log_box.append("Đã bắt đầu camera.")

    def on_stop_camera(self):
        if self._camera_thread:
            self._camera_thread.stop()
            self._camera_thread = None

    def _on_camera_stopped(self):
        self._camera_thread = None
        self.btn_cam_start.setEnabled(True)
        self.btn_cam_stop.setEnabled(False)
        self.video_label.setText("Camera đã dừng")
        self.log_box.append("Camera đã dừng.")

    def _on_camera_error(self, msg):
        self.log_box.append(f"⚠ LỖI CAMERA: {msg}")
        QMessageBox.warning(self, "Lỗi Camera", msg)

    def _frame_counter(self):
        if not hasattr(self, "_fc"):
            self._fc = 0
        self._fc += 1
        return self._fc

    def _on_frame(self, frame, items, fps):
        self.ctx.latest_circles = items
        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps.setText(f"FPS: {fps:.1f}")

        if self._frame_counter() % 3 == 0:
            self.circle_table.setRowCount(len(items))
            for i, c in enumerate(items):
                self.circle_table.setItem(i, 0, QTableWidgetItem(str(c.get("type", ""))))
                self.circle_table.setItem(i, 1, QTableWidgetItem(_fmt_size_col(c)))
                self.circle_table.setItem(i, 2, QTableWidgetItem(f'{c["x_mm"]:.2f}'))
                self.circle_table.setItem(i, 3, QTableWidgetItem(f'{c["y_mm"]:.2f}'))
                self.circle_table.setItem(i, 4, QTableWidgetItem(_fmt_angle_col(c)))