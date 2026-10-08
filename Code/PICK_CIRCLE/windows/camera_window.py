from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QWidget,
    QGroupBox, QDoubleSpinBox, QTextEdit, QMessageBox, QSizePolicy,
)

from shared_state import (
    BaseTabWindow, CameraThread, cv2_to_qpixmap,
    save_config, COORD_MODE_DOF4, launch_manual_camera_center,
)

# THÊM MỚI: toàn bộ phần "phát hiện vòng tròn đỏ + tính tọa độ mm" đã
# được TÁCH RIÊNG ra file vision/red_circle_vision.py (không dùng Qt,
# không phụ thuộc shared_state.py hay vision/Cameracircle.py). File
# camera_tab.py này CHỈ còn code giao diện (Qt) - không còn xử lý ảnh
# (OpenCV) trực tiếp ở đây nữa, chỉ gọi sang RedCircleVision để lấy kết
# quả (tọa độ) và khung hình đã được vẽ chú thích sẵn.
from vision.red_circle_vision import RedCircleVision

# ---------------------------------------------------------------------
# KÍCH THƯỚC GIAO DIỆN. Gom về hằng số ở đây để dễ chỉnh lại sau này.
# ---------------------------------------------------------------------
VIDEO_MIN_W = 320
VIDEO_MIN_H = 240
OFFSET_BOX_MAX_H = 92
SPINBOX_MAX_W = 200
CTRL_COL_W = 500   # độ rộng cột phải (px), muốn hẹp/rộng hơn thì chỉnh số này


# ========================== CỬA SỔ CHÍNH ==========================
class CameraWindow(BaseTabWindow):
    """
    Cửa sổ CAMERA && OFFSET TỌA ĐỘ.

    CHỈ dùng để:
      1) Xem trực tiếp hình ảnh camera (chế độ Camera_4dof.py - HCN đỏ
         10x20mm, có góc quay) để kiểm tra hoạt động đã ổn định chưa. Tâm
         khung hình (0,0) được đánh dấu CÙNG STYLE với chế độ THỦ CÔNG (chấm
         cam + nhãn) để dễ đối chiếu.
      2) Chỉnh offset (bù trừ) tọa độ camera theo X/Y (mm).
      3) Mở chế độ THỦ CÔNG (vision/red_circle_vision.py) - chấm cam giữa
         khung hình đánh dấu tọa độ (0,0), dùng để canh tâm bàn xoay bằng
         mắt trước khi chạy tự động. Nút này nằm ở cột phải, bấm là chuyển
         THẲNG sang cửa sổ THỦ CÔNG ngay (không hỏi lại, không cần tự tay
         chuyển cửa sổ - CameraWindow tự ẩn đi ngay khi THỦ CÔNG khởi chạy
         thành công).
      4) [MỚI] Phát hiện vòng tròn ĐỎ đường kính thật 30mm ngay trên cùng
         khung hình đang xem, và hiển thị tọa độ (X, Y) tính bằng mm -
         vừa vẽ trực tiếp lên khung hình video, vừa hiển thị ở 1 nhãn
         riêng trong cột điều khiển bên phải. Toàn bộ phần TÍNH TOÁN nằm
         trong vision/red_circle_vision.py (file riêng, xem module đó để
         biết chi tiết thuật toán + công thức hiệu chuẩn) - CameraWindow
         chỉ gọi sang và hiển thị kết quả. Tọa độ trả về đã CỘNG SẴN Offset
         Camera (X/Y) đang chỉnh ở cột phải (ctx.offset_x/ctx.offset_y).
         Không liên quan gì tới điều khiển robot.

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
        # [MỚI] đối tượng xử lý vision cho vòng tròn đỏ - xem
        # vision/red_circle_vision.py. reset() được gọi lại mỗi lần bật
        # camera (xem on_start_camera).
        self._circle_vision = RedCircleVision()
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
            "background-color:#0f2540; border:2px solid #2a4a75; border-radius:10px; color:#9fb6d6;"
        )
        # Ignored: kích thước label chỉ do layout quyết định, KHÔNG bị pixmap
        # đẩy to lên -> các nút bên dưới luôn còn chỗ hiển thị.
        self.video_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        video_col.addWidget(self.video_label, 1)

        cam_ctrl_row = QHBoxLayout()
        cam_ctrl_row.setSpacing(6)
        self.btn_cam_start = QPushButton("▶  BẬT CAMERA")
        self.btn_cam_start.setObjectName("connectBtn")
        self.btn_cam_start.clicked.connect(self.on_start_camera)
        self.btn_cam_stop = QPushButton("■  TẮT CAMERA")
        self.btn_cam_stop.setObjectName("disconnectBtn")
        self.btn_cam_stop.clicked.connect(self.on_stop_camera)
        self.btn_cam_stop.setEnabled(False)
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#cfe3ff; font-weight:700; font-size:10pt;")
        cam_ctrl_row.addWidget(self.btn_cam_start)
        cam_ctrl_row.addWidget(self.btn_cam_stop)
        cam_ctrl_row.addWidget(self.lbl_fps)
        video_col.addLayout(cam_ctrl_row)

        self.lbl_mode_note = QLabel(
            "Chế độ nhận diện: Camera_4dof.py - HCN đỏ 10x20mm, có góc quay."
        )
        self.lbl_mode_note.setWordWrap(True)
        self.lbl_mode_note.setStyleSheet("color:#9fb6d6; font-size:9pt;")
        video_col.addWidget(self.lbl_mode_note)

        # ---------------- Cột phải: THỦ CÔNG + OFFSET + log camera ----------------
        ctrl_col = QVBoxLayout()
        ctrl_col.setAlignment(Qt.AlignTop)
        ctrl_col.setSpacing(6)

        self.btn_manual = QPushButton("THỦ CÔNG")
        self.btn_manual.setObjectName("actionBtn")
        self.btn_manual.setToolTip(
            "Dừng camera ở cửa sổ này (nếu đang chạy) rồi mở ngay vision/red_circle_vision.py\n"
            "ở cửa sổ riêng - hiển thị chấm cam đánh dấu tọa độ (0,0) để canh tâm bàn xoay.\n"
            "Cửa sổ CAMERA này sẽ tự ẩn đi để chuyển thẳng sang chế độ THỦ CÔNG."
        )
        self.btn_manual.clicked.connect(self.on_open_manual)
        ctrl_col.addWidget(self.btn_manual)

        offset_box = QGroupBox("BÙ TRỪ CAMERA (mm)")
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
        note.setStyleSheet("color:#9fb6d6; font-size:9pt;")
        ctrl_col.addWidget(note)

        # ---------------- [MỚI] Tọa độ vòng tròn đỏ Ø30mm ----------------
        circle_box = QGroupBox("Vòng tròn đỏ Ø30mm")
        circle_box.setObjectName("compactBox")
        circle_layout = QVBoxLayout()
        circle_layout.setContentsMargins(10, 4, 10, 6)
        self.lbl_circle_coord = QLabel("Chưa phát hiện")
        self.lbl_circle_coord.setStyleSheet(
            "color:#9aa7b8; font-weight:800; font-size:13pt;"
        )
        self.lbl_circle_coord.setWordWrap(True)
        circle_layout.addWidget(self.lbl_circle_coord)
        circle_box.setLayout(circle_layout)
        ctrl_col.addWidget(circle_box)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("font-size:9pt;")
        ctrl_col.addWidget(self.log_box, 1)

        # Cột phải bọc trong widget có độ rộng cố định -> không bị phình ra,
        # toàn bộ chỗ trống còn lại dành cho vùng camera.
        ctrl_widget = QWidget()
        ctrl_widget.setLayout(ctrl_col)
        ctrl_col.setContentsMargins(0, 0, 0, 0)
        ctrl_widget.setFixedWidth(CTRL_COL_W)

        root.addLayout(video_col, 1)
        root.addWidget(ctrl_widget)

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

        # [MỚI] reset trạng thái vision (tracker + đọc lại calib) mỗi lần
        # bật camera - xem vision/red_circle_vision.py.
        err = self._circle_vision.reset(self.ctx.cfg.get("calib_file"))
        if err:
            self.log_box.append(f"⚠ {err}")

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
        phóng camera - dùng trước khi mở chế độ THỦ CÔNG để tránh 2 tiến
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

    # ==================== CHẾ ĐỘ THỦ CÔNG ====================
    def on_open_manual(self):
        if self.ctx.dynamic_camera_thread is not None:
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Tab BÀI TOÁN ĐỘNG đang dùng camera - hãy bấm STOP ở tab đó trước.")
            return

        # Bấm là chuyển thẳng sang chế độ THỦ CÔNG ngay: tự dừng camera ở
        # đây (nếu đang chạy) rồi mở cửa sổ THỦ CÔNG luôn, không hỏi lại.
        self._stop_camera_and_wait()

        proc = launch_manual_camera_center(self.ctx.cfg.get("camera_index", 0))
        if proc is None:
            QMessageBox.warning(
                self, "Lỗi mở chế độ THỦ CÔNG",
                "Không mở được vision/red_circle_vision.py.\n"
                "Kiểm tra file có tồn tại và đã cài opencv-python chưa."
            )
            return

        self.log_box.append("Đã chuyển sang chế độ THỦ CÔNG (canh tâm bàn xoay).")

        # ==================================================================
        # THÊM MỚI: tự động ẩn cửa sổ CAMERA này đi ngay sau khi chế độ
        # THỦ CÔNG khởi chạy thành công, để người dùng KHÔNG cần tự bấm
        # chuyển cửa sổ - THỦ CÔNG (cv2.imshow, tiến trình riêng) sẽ là
        # thứ hiển thị ngay sau khi bấm nút, thay vì vẫn đứng ở màn CAMERA
        # && OFFSET.
        #
        # LƯU Ý: vision/red_circle_vision.py chạy như TIẾN TRÌNH RIÊNG
        # (subprocess, xem launch_manual_camera_center() trong
        # shared_state.py) - đây KHÔNG phải cửa sổ Qt của app này nên
        # không thể setFocus()/raise_() trực tiếp từ đây. go_back() chỉ
        # đảm bảo CameraWindow (Qt) không còn che phía trước; cửa sổ
        # cv2.imshow mới tạo thường tự nổi lên foreground (hành vi mặc
        # định của hệ điều hành khi 1 tiến trình mới tạo cửa sổ).
        # ==================================================================
        self.go_back()

    def _on_frame(self, frame, items, fps):
        self.ctx.latest_circles = items

        # [MỚI] Gọi sang vision/red_circle_vision.py để phát hiện vòng
        # tròn đỏ Ø30mm + tính tọa độ (đã cộng sẵn offset giao diện) - hàm
        # này VẼ LUÔN chú thích lên `frame`. CameraWindow chỉ hiển thị kết
        # quả, không tự xử lý ảnh.
        coords = self._circle_vision.process_frame(
            frame, offset_x=self.ctx.offset_x, offset_y=self.ctx.offset_y,
        )
        if coords is not None:
            x_mm, y_mm = coords
            self.lbl_circle_coord.setText(f"X = {x_mm:.1f} mm\nY = {y_mm:.1f} mm")
            self.lbl_circle_coord.setStyleSheet(
                "color:#4cd98a; font-weight:800; font-size:13pt;"
            )
        else:
            self.lbl_circle_coord.setText("Chưa phát hiện")
            self.lbl_circle_coord.setStyleSheet(
                "color:#9aa7b8; font-weight:800; font-size:13pt;"
            )

        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps.setText(f"FPS: {fps:.1f}")