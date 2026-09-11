from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QTextEdit, QMessageBox, QSizePolicy,
)

from shared_state import (
    BaseTabWindow, CameraThread, cv2_to_qpixmap,
    save_config, COORD_MODE_DOF4, launch_manual_camera_center,
)

# ---------------------------------------------------------------------
# KÍCH THƯỚC GIAO DIỆN. Gom về hằng số ở đây để dễ chỉnh lại sau này.
# ---------------------------------------------------------------------
VIDEO_MIN_W = 900
VIDEO_MIN_H = 640
OFFSET_BOX_MAX_H = 92
SPINBOX_MAX_W = 130


# ========================== CỬA SỔ CHÍNH ==========================
class CameraWindow(BaseTabWindow):
    """
    Cửa sổ CAMERA && OFFSET TỌA ĐỘ.

    CHỈ dùng để:
      1) Xem trực tiếp hình ảnh camera (chế độ Camera_4dof.py - HCN đỏ
         10x20mm, có góc quay) để kiểm tra hoạt động đã ổn định chưa. Tâm
         khung hình (0,0) được đánh dấu CÙNG STYLE với chế độ MANUAL (chấm
         cam + nhãn) để dễ đối chiếu.
      2) Chỉnh offset (bù trừ) tọa độ camera theo X/Y (mm).
      3) Mở chế độ MANUAL (vision/camera_center_dot.py) - chấm cam giữa
         khung hình đánh dấu tọa độ (0,0), dùng để canh tâm bàn xoay bằng
         mắt trước khi chạy tự động. Nút này nằm ở cột phải, bấm là chuyển
         thẳng sang cửa sổ MANUAL ngay (không hỏi lại).

    Dùng CHUNG lớp CameraThread trong shared_state.py (không tự định nghĩa
    luồng camera riêng) để tránh trùng lặp code với các cửa sổ khác. Chỉ
    hỗ trợ chế độ DOF4 ở đây.

    Không còn bảng tọa độ / chu trình gắp-thả tự động ở đây nữa - cửa sổ
    này chỉ để xem hình + canh offset; tọa độ nhận diện vẫn được lưu vào
    ctx.latest_circles để giao diện BÀI TOÁN TĨNH (CsvWindow) dùng như cũ.
    """

    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "📷 CAMERA && OFFSET TỌA ĐỘ")
        self._camera_thread = None
        self._build_ui()

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)
        root.setSpacing(10)

        # ---------------- Cột trái: video (phóng to) ----------------
        video_col = QVBoxLayout()
        video_col.setSpacing(6)

        self.video_label = QLabel("Camera chưa kết nối")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(VIDEO_MIN_W, VIDEO_MIN_H)
        self.video_label.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; border-radius:10px; color:#888888;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 1)

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

        self.lbl_mode_note = QLabel(
            "Chế độ nhận diện: Camera_4dof.py - HCN đỏ 10x20mm, có góc quay."
        )
        self.lbl_mode_note.setWordWrap(True)
        self.lbl_mode_note.setStyleSheet("color:#555555; font-size:9pt;")
        video_col.addWidget(self.lbl_mode_note)

        # ---------------- Cột phải: MANUAL + OFFSET + log camera ----------------
        ctrl_col = QVBoxLayout()
        ctrl_col.setAlignment(Qt.AlignTop)
        ctrl_col.setSpacing(6)

        self.btn_manual = QPushButton("🎯  CHẾ ĐỘ MANUAL\n(canh tâm bàn xoay)")
        self.btn_manual.setObjectName("actionBtn")
        self.btn_manual.setToolTip(
            "Dừng camera ở cửa sổ này (nếu đang chạy) rồi mở ngay vision/camera_center_dot.py\n"
            "ở cửa sổ riêng - hiển thị chấm cam đánh dấu tọa độ (0,0) để canh tâm bàn xoay."
        )
        self.btn_manual.clicked.connect(self.on_open_manual)
        ctrl_col.addWidget(self.btn_manual)

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
            "Offset ở trên chỉ bù trừ sai lệch giữa gốc tọa độ camera và\n"
            "gốc tọa độ thực tế của robot."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#555555; font-size:9pt;")
        ctrl_col.addWidget(note)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("font-size:9pt;")
        ctrl_col.addWidget(self.log_box, 1)

        root.addLayout(video_col, 4)
        root.addLayout(ctrl_col, 1)

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
        # Luôn dùng chế độ DOF4 ở cửa sổ này. KHÔNG đổi ctx.coord_mode ở đây
        # để không ảnh hưởng lựa chọn chế độ của các tab khác.
        self._camera_thread = CameraThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            calib_path=self.ctx.cfg.get("calib_file"),
            detect_mode=COORD_MODE_DOF4,
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

    def _stop_camera_and_wait(self):
        """Dừng camera thread ở cửa sổ này VÀ CHỜ đến khi thật sự giải
        phóng camera - dùng trước khi mở chế độ MANUAL để tránh 2 tiến
        trình cùng giữ 1 camera."""
        if self._camera_thread is None:
            return
        self._camera_thread.stop()
        self._camera_thread.wait(3000)
        self._camera_thread = None
        self.btn_cam_start.setEnabled(True)
        self.btn_cam_stop.setEnabled(False)
        self.video_label.setText("Camera đã dừng")

    def _on_camera_stopped(self):
        self._camera_thread = None
        self.btn_cam_start.setEnabled(True)
        self.btn_cam_stop.setEnabled(False)
        self.video_label.setText("Camera đã dừng")
        self.log_box.append("Camera đã dừng.")

    def _on_camera_error(self, msg):
        self.log_box.append(f"⚠ LỖI CAMERA: {msg}")
        QMessageBox.warning(self, "Lỗi Camera", msg)

    # ==================== CHẾ ĐỘ MANUAL ====================
    def on_open_manual(self):
        if self.ctx.dynamic_camera_thread is not None:
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Tab BÀI TOÁN ĐỘNG đang dùng camera - hãy bấm STOP ở tab đó trước.")
            return

        # Bấm là chuyển thẳng sang MANUAL ngay: tự dừng camera ở đây (nếu
        # đang chạy) rồi mở cửa sổ MANUAL luôn, không hỏi lại.
        self._stop_camera_and_wait()

        proc = launch_manual_camera_center(self.ctx.cfg.get("camera_index", 0))
        if proc is None:
            QMessageBox.warning(
                self, "Lỗi mở MANUAL",
                "Không mở được vision/camera_center_dot.py.\n"
                "Kiểm tra file có tồn tại và đã cài opencv-python chưa."
            )
            return
        self.log_box.append("Đã chuyển sang chế độ MANUAL (canh tâm bàn xoay).")

    def _on_frame(self, frame, items, fps):
        self.ctx.latest_circles = items
        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps.setText(f"FPS: {fps:.1f}")