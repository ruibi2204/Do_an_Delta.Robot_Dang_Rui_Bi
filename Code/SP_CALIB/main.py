import sys
import json
import time
import threading
import cv2
import numpy as np
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtMultimedia import QCamera, QCameraImageCapture
from PyQt5.QtMultimediaWidgets import QCameraViewfinder

from robot_controller import DeltaRobotController


class CameraThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.cap = None
        self.is_running = False
        self.camera_id = 0

    def start_camera(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            return False
        self.is_running = True
        self.start()
        return True

    def stop_camera(self):
        self.is_running = False
        if self.cap:
            self.cap.release()
        self.wait()

    def run(self):
        while self.is_running:
            ret, frame = self.cap.read()
            if ret:
                # Chuyển đổi BGR sang RGB cho PyQt
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.frame_ready.emit(frame_rgb)
            time.sleep(0.03)


class DeltaRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.controller = DeltaRobotController(gear_ratio=3.0)
        self.camera_thread = CameraThread()
        self.is_camera_on = False
        self.scan_thread = None

        # Biến trạng thái
        self.target_z = 380.0
        self.lift_height = 20.0
        self.dwell_time = 1.0
        self.port_selected = ""

        self.init_ui()
        self.load_styles()
        self.refresh_ports()

    def init_ui(self):
        """Khởi tạo giao diện"""
        self.setWindowTitle("🤖 DELTA ROBOT CONTROLLER PRO")
        self.setGeometry(100, 100, 1400, 800)
        self.setMinimumSize(1200, 700)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # ===== COLUMN LEFT: CAMERA =====
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)

        # Camera view
        camera_group = QGroupBox("📷 CAMERA VIEW")
        camera_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        camera_layout = QVBoxLayout(camera_group)

        # Camera display
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("""
            QLabel {
                background-color: #1a1a2e;
                border: 2px solid #2a2a4e;
                border-radius: 8px;
            }
        """)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setText("📷 CAMERA OFF\n\nNhấn 'Bật Camera' để bắt đầu")
        self.camera_label.setFont(QFont("Arial", 14))
        self.camera_label.setStyleSheet(
            "color: #666; background-color: #1a1a2e; border: 2px solid #2a2a4e; border-radius: 8px;")

        camera_layout.addWidget(self.camera_label)
        left_layout.addWidget(camera_group)

        # Camera controls
        cam_control_layout = QHBoxLayout()

        self.cam_btn = QPushButton("📷 BẬT CAMERA")
        self.cam_btn.setFixedHeight(40)
        self.cam_btn.clicked.connect(self.toggle_camera)
        cam_control_layout.addWidget(self.cam_btn)

        self.cam_status_label = QLabel("🔴 TẮT")
        self.cam_status_label.setStyleSheet("color: #ff4444; font-weight: bold;")
        cam_control_layout.addWidget(self.cam_status_label)

        left_layout.addLayout(cam_control_layout)

        # Points information
        info_group = QGroupBox("📋 POINTS INFORMATION")
        info_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        info_layout = QVBoxLayout(info_group)

        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(120)
        self.info_text.setStyleSheet("""
            QTextEdit {
                background-color: #16213e;
                color: #a0d4ff;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                font-family: Consolas;
                font-size: 11px;
            }
        """)
        self.info_text.setText("Chưa có dữ liệu điểm\n\n📂 Nhấn 'LOAD FILE' để đọc dữ liệu")
        info_layout.addWidget(self.info_text)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                text-align: center;
                background-color: #16213e;
                height: 20px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00d4ff, stop:1 #0066ff);
                border-radius: 5px;
            }
        """)
        self.progress_bar.setVisible(False)
        info_layout.addWidget(self.progress_bar)

        left_layout.addWidget(info_group)

        # ===== COLUMN RIGHT: CONTROL PANEL =====
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(10)

        # Header
        header_label = QLabel("🎮 CONTROL PANEL")
        header_label.setStyleSheet("""
            QLabel {
                font-size: 20px;
                font-weight: bold;
                color: #00d4ff;
                padding: 10px;
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #16213e, stop:1 #0a1628);
                border-radius: 10px;
            }
        """)
        header_label.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(header_label)

        # UART Connection
        uart_group = QGroupBox("🔌 UART CONNECTION")
        uart_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        uart_layout = QVBoxLayout(uart_group)

        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Cổng COM:"))
        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet("""
            QComboBox {
                background-color: #16213e;
                color: white;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
                min-width: 100px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
            }
        """)
        self.port_combo.currentTextChanged.connect(self.on_port_changed)
        port_layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("🔄")
        refresh_btn.setFixedSize(30, 30)
        refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(refresh_btn)

        uart_layout.addLayout(port_layout)

        # Connect/Disconnect buttons
        conn_layout = QHBoxLayout()

        self.connect_btn = QPushButton("🔗 KẾT NỐI")
        self.connect_btn.setFixedHeight(35)
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)

        self.conn_status_label = QLabel("🔴 CHƯA KẾT NỐI")
        self.conn_status_label.setStyleSheet("color: #ff4444; font-weight: bold;")
        conn_layout.addWidget(self.conn_status_label)

        uart_layout.addLayout(conn_layout)
        right_layout.addWidget(uart_group)

        # Motion Parameters
        motion_group = QGroupBox("⚙️ MOTION PARAMETERS")
        motion_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        motion_layout = QGridLayout(motion_group)

        # Target Z
        motion_layout.addWidget(QLabel("🎯 Target Z (mm):"), 0, 0)
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(0, 500)
        self.z_spin.setValue(380.0)
        self.z_spin.setSingleStep(10)
        self.z_spin.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #16213e;
                color: #00d4ff;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }
        """)
        self.z_spin.valueChanged.connect(lambda v: setattr(self, 'target_z', v))
        motion_layout.addWidget(self.z_spin, 0, 1)

        # Lift Height
        motion_layout.addWidget(QLabel("⬆️ Lift Height (mm):"), 1, 0)
        self.lift_spin = QDoubleSpinBox()
        self.lift_spin.setRange(0, 100)
        self.lift_spin.setValue(20.0)
        self.lift_spin.setSingleStep(5)
        self.lift_spin.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #16213e;
                color: #ffaa00;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }
        """)
        self.lift_spin.valueChanged.connect(lambda v: setattr(self, 'lift_height', v))
        motion_layout.addWidget(self.lift_spin, 1, 1)

        # Dwell Time
        motion_layout.addWidget(QLabel("⏱️ Dwell Time (s):"), 2, 0)
        self.dwell_spin = QDoubleSpinBox()
        self.dwell_spin.setRange(0.1, 10)
        self.dwell_spin.setValue(2.0)
        self.dwell_spin.setSingleStep(0.5)
        self.dwell_spin.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #16213e;
                color: #ff6b6b;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
                font-weight: bold;
            }
        """)
        self.dwell_spin.valueChanged.connect(lambda v: setattr(self, 'dwell_time', v))
        motion_layout.addWidget(self.dwell_spin, 2, 1)

        right_layout.addWidget(motion_group)

        # Control Buttons
        control_group = QGroupBox("🎮 CONTROLS")
        control_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        control_layout = QGridLayout(control_group)

        # Buttons
        self.home_btn = QPushButton("🏠 HOME")
        self.home_btn.setFixedHeight(40)
        self.home_btn.clicked.connect(self.go_home)
        control_layout.addWidget(self.home_btn, 0, 0)

        self.load_btn = QPushButton("📂 LOAD FILE")
        self.load_btn.setFixedHeight(40)
        self.load_btn.clicked.connect(self.load_file)
        control_layout.addWidget(self.load_btn, 0, 1)

        self.run_btn = QPushButton("▶️ RUN")
        self.run_btn.setFixedHeight(40)
        self.run_btn.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00d4ff, stop:1 #0066ff);
                color: white;
                font-weight: bold;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #33ddff, stop:1 #3377ff);
            }
            QPushButton:pressed {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #0099cc, stop:1 #0044cc);
            }
        """)
        self.run_btn.clicked.connect(self.run_scan)
        control_layout.addWidget(self.run_btn, 0, 2)

        self.stop_btn = QPushButton("⏹️ STOP")
        self.stop_btn.setFixedHeight(40)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff4444, stop:1 #cc0000);
                color: white;
                font-weight: bold;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff6666, stop:1 #ee2222);
            }
            QPushButton:pressed {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #cc2222, stop:1 #aa0000);
            }
        """)
        self.stop_btn.clicked.connect(self.stop_scan)
        control_layout.addWidget(self.stop_btn, 0, 3)

        right_layout.addWidget(control_group)

        # Theta Display
        theta_group = QGroupBox("📊 THETA VALUES")
        theta_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                color: #00d4ff;
                border: 2px solid #00d4ff;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        theta_layout = QVBoxLayout(theta_group)

        self.theta_text = QTextEdit()
        self.theta_text.setReadOnly(True)
        self.theta_text.setMaximumHeight(60)
        self.theta_text.setStyleSheet("""
            QTextEdit {
                background-color: #16213e;
                color: #ffaa00;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                font-family: Consolas;
                font-size: 12px;
                font-weight: bold;
            }
        """)
        self.theta_text.setText("Chưa có dữ liệu")
        theta_layout.addWidget(self.theta_text)

        right_layout.addWidget(theta_group)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.setStyleSheet("""
            QStatusBar {
                background-color: #0a0a1a;
                color: #88bbdd;
                padding: 5px;
            }
        """)
        self.status_bar.showMessage("🟢 Sẵn sàng")

        # Add panels to main layout
        main_layout.addWidget(left_panel, 6)
        main_layout.addWidget(right_panel, 4)

        # Set window icon
        self.setWindowIcon(QIcon())

    def load_styles(self):
        """Load stylesheet cho ứng dụng"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #0a0a1a, stop:1 #1a1a3e);
            }
            QWidget {
                background-color: transparent;
            }
            QLabel {
                color: #ccddff;
            }
            QPushButton {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2a2a4e, stop:1 #1a1a3e);
                color: white;
                border: 1px solid #3a3a6e;
                border-radius: 8px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #3a3a6e, stop:1 #2a2a4e);
                border: 1px solid #4a4a8e;
            }
            QPushButton:pressed {
                background-color: #1a1a3e;
            }
            QPushButton:disabled {
                background-color: #2a2a3e;
                color: #666;
                border: 1px solid #3a3a4e;
            }
            QComboBox {
                background-color: #16213e;
                color: white;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
            }
            QComboBox QAbstractItemView {
                background-color: #16213e;
                color: white;
                border: 1px solid #2a2a4e;
            }
            QComboBox::drop-down {
                border: none;
            }
            QSpinBox, QDoubleSpinBox {
                background-color: #16213e;
                color: white;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                padding: 5px;
            }
            QProgressBar {
                border: 1px solid #2a2a4e;
                border-radius: 5px;
                text-align: center;
                background-color: #16213e;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00d4ff, stop:1 #0066ff);
                border-radius: 5px;
            }
            QTabWidget::pane {
                border: 1px solid #2a2a4e;
                background-color: #0a0a1a;
            }
            QTabBar::tab {
                background-color: #16213e;
                color: #88bbdd;
                padding: 8px 20px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QTabBar::tab:selected {
                background-color: #1a1a3e;
                color: #00d4ff;
                border-bottom: 2px solid #00d4ff;
            }
            QTabBar::tab:hover {
                background-color: #2a2a4e;
            }
            QTextEdit {
                background-color: #0a0a1a;
                color: #88bbdd;
                border: 1px solid #2a2a4e;
                border-radius: 5px;
            }
            QGroupBox {
                color: #88bbdd;
                border: 1px solid #2a2a4e;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #00d4ff;
            }
            QScrollBar:vertical {
                background-color: #0a0a1a;
                width: 12px;
                border-radius: 6px;
            }
            QScrollBar::handle:vertical {
                background-color: #2a2a4e;
                border-radius: 6px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #3a3a6e;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0;
            }
        """)

    def refresh_ports(self):
        """Refresh danh sách cổng COM"""
        self.port_combo.clear()
        ports = self.controller.get_ports()
        if ports:
            self.port_combo.addItems(ports)
            self.port_combo.setCurrentIndex(0)
            self.port_selected = ports[0]
        else:
            self.port_combo.addItem("Không tìm thấy cổng")

    def on_port_changed(self, port):
        """Xử lý khi đổi cổng"""
        self.port_selected = port

    def toggle_connection(self):
        """Bật/tắt kết nối UART"""
        if not self.controller.is_connected():
            if not self.port_selected or self.port_selected == "Không tìm thấy cổng":
                QMessageBox.warning(self, "Lỗi", "Không có cổng COM khả dụng!")
                return

            success, msg = self.controller.connect_uart(self.port_selected)
            if success:
                self.connect_btn.setText("🔌 NGẮT KẾT NỐI")
                self.conn_status_label.setText("🟢 ĐÃ KẾT NỐI")
                self.conn_status_label.setStyleSheet("color: #44ff44; font-weight: bold;")
                self.status_bar.showMessage(f"✅ {msg}")
                QMessageBox.information(self, "Thành công", msg)
            else:
                QMessageBox.critical(self, "Lỗi", msg)
                self.status_bar.showMessage(f"❌ {msg}")
        else:
            success, msg = self.controller.disconnect_uart()
            self.connect_btn.setText("🔗 KẾT NỐI")
            self.conn_status_label.setText("🔴 CHƯA KẾT NỐI")
            self.conn_status_label.setStyleSheet("color: #ff4444; font-weight: bold;")
            self.status_bar.showMessage(f"✅ {msg}")

    def toggle_camera(self):
        """Bật/tắt camera"""
        if not self.is_camera_on:
            if self.camera_thread.start_camera(0):
                self.is_camera_on = True
                self.cam_btn.setText("📷 TẮT CAMERA")
                self.cam_status_label.setText("🟢 ĐANG CHẠY")
                self.cam_status_label.setStyleSheet("color: #44ff44; font-weight: bold;")
                self.camera_label.setText("📷 Đang kết nối...")
                self.status_bar.showMessage("📷 Camera đã bật")

                # Kết nối signal
                self.camera_thread.frame_ready.connect(self.update_camera_frame)
            else:
                QMessageBox.critical(self, "Lỗi", "Không thể mở camera!")
        else:
            self.camera_thread.stop_camera()
            self.is_camera_on = False
            self.cam_btn.setText("📷 BẬT CAMERA")
            self.cam_status_label.setText("🔴 TẮT")
            self.cam_status_label.setStyleSheet("color: #ff4444; font-weight: bold;")
            self.camera_label.setText("📷 CAMERA OFF\n\nNhấn 'Bật Camera' để bắt đầu")
            self.camera_label.setStyleSheet(
                "color: #666; background-color: #1a1a2e; border: 2px solid #2a2a4e; border-radius: 8px;")
            self.status_bar.showMessage("📷 Camera đã tắt")

    def update_camera_frame(self, frame_rgb):
        """Cập nhật frame camera lên giao diện"""
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        qt_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_img)

        # Scale để vừa với label
        scaled_pixmap = pixmap.scaled(
            self.camera_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.camera_label.setPixmap(scaled_pixmap)
        self.camera_label.setStyleSheet("border: 2px solid #00d4ff; border-radius: 8px;")

    def go_home(self):
        """Di chuyển về home"""
        if not self.controller.is_connected():
            QMessageBox.warning(self, "Lỗi", "Chưa kết nối UART!")
            return

        self.status_bar.showMessage("🏠 Đang về home...")
        success, msg, thetas = self.controller.go_home()
        if success:
            self.update_theta_display(thetas)
            self.status_bar.showMessage(f"✅ {msg}")
            QMessageBox.information(self, "Thành công", "Đã về vị trí HOME")
        else:
            self.status_bar.showMessage(f"❌ {msg}")
            QMessageBox.critical(self, "Lỗi", msg)

    def load_file(self):
        """Đọc file JSON"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Chọn file tọa độ", "", "JSON Files (*.json)"
        )
        if file_path:
            success, msg = self.controller.load_corners_from_json(file_path)
            if success:
                info = self.controller.get_points_info()
                self.info_text.setText(info)
                self.status_bar.showMessage(f"✅ {msg}")
                QMessageBox.information(self, "Thành công", msg)
            else:
                self.status_bar.showMessage(f"❌ {msg}")
                QMessageBox.critical(self, "Lỗi", msg)

    def run_scan(self):
        """Bắt đầu quét"""
        if not self.controller.is_connected():
            QMessageBox.warning(self, "Lỗi", "Chưa kết nối UART!")
            return

        if not self.controller.corners_data:
            QMessageBox.warning(self, "Lỗi", "Chưa đọc file dữ liệu!")
            return

        # Disable buttons trong khi chạy
        self.run_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.load_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)

        # Chạy trong thread riêng
        self.scan_thread = threading.Thread(target=self._run_scan_thread, daemon=True)
        self.scan_thread.start()

    def _run_scan_thread(self):
        """Thread chạy quét"""

        def progress_callback(current, total, result):
            progress = int((current / total) * 100)
            self.progress_bar.setValue(progress)
            if result['success']:
                self.status_bar.showMessage(f"✅ Đã đến điểm {current}/{total}")
            else:
                self.status_bar.showMessage(f"❌ Lỗi tại điểm {current}/{total}")

        success, results = self.controller.run_auto_scan(
            self.target_z,
            delay=0.5,
            callback=progress_callback,
            lift_height=self.lift_height,
            dwell_time=self.dwell_time
        )

        # Cập nhật UI sau khi hoàn thành
        self.progress_bar.setVisible(False)
        self.run_btn.setEnabled(True)
        self.home_btn.setEnabled(True)
        self.load_btn.setEnabled(True)

        if success:
            success_count = sum(1 for r in results if r['success'])
            total = len(results)
            self.status_bar.showMessage(f"✅ Hoàn thành! {success_count}/{total} điểm")
            QMessageBox.information(self, "Thành công", f"Hoàn thành {success_count}/{total} điểm")
        else:
            self.status_bar.showMessage(f"❌ {results}")
            QMessageBox.critical(self, "Lỗi", str(results))

    def stop_scan(self):
        """Dừng quét"""
        if self.controller.is_running:
            self.controller.stop_scan()
            self.status_bar.showMessage("⏹️ Đã dừng quét")
            self.progress_bar.setVisible(False)
            self.run_btn.setEnabled(True)
            self.home_btn.setEnabled(True)
            self.load_btn.setEnabled(True)

    def update_theta_display(self, thetas):
        """Cập nhật hiển thị theta"""
        if thetas:
            t1, t2, t3 = thetas
            gear = self.controller.get_gear_ratio()
            text = f"Motor: T1={t1:.2f}°  T2={t2:.2f}°  T3={t3:.2f}°\n"
            text += f"Sau đai: T1'={t1 * gear:.2f}°  T2'={t2 * gear:.2f}°  T3'={t3 * gear:.2f}°"
            self.theta_text.setText(text)

    def closeEvent(self, event):
        """Xử lý khi đóng ứng dụng"""
        if self.is_camera_on:
            self.camera_thread.stop_camera()
        if self.controller.is_connected():
            self.controller.disconnect_uart()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Set application icon
    app.setWindowIcon(QIcon())

    window = DeltaRobotGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()