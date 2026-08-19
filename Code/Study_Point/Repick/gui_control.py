#!/usr/bin/env python3
# gui_control_pyqt5.py
# Giao diện điều khiển robot Delta – phiên bản xịn sò


import sys
import threading
import time
import csv
import cv2
from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget,
                             QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
                             QLineEdit, QListWidget, QFileDialog, QMessageBox,
                             QGroupBox, QFormLayout, QComboBox, QTextEdit,
                             QProgressBar, QSplitter)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage

# Import các module đã viết
from kinematics import inverse_kinematics
from uart_comm import UARTComm
from movement_sequence import HOME_POS, PICK_Z, WAIT_HOME, WAIT_PICK

# ==================== Camera Thread ====================
class CameraThread(QThread):
    image_updated = pyqtSignal(QImage)

    def __init__(self, cam_id):
        super().__init__()
        self.cam_id = cam_id
        self.running = False
        self.cap = None

    def run(self):
        self.running = True
        self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_DSHOW if sys.platform == 'win32' else None)
        if not self.cap.isOpened():
            self.image_updated.emit(QImage())
            return
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                h, w, ch = frame.shape
                max_w, max_h = 640, 480
                scale = min(max_w / w, max_h / h, 1.0)
                if scale < 1:
                    new_w = int(w * scale)
                    new_h = int(h * scale)
                    frame = cv2.resize(frame, (new_w, new_h))
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                qt_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.image_updated.emit(qt_img)
            else:
                time.sleep(0.05)
        self.cap.release()

    def stop(self):
        self.running = False
        self.wait()

# ==================== Main Window với QSS ====================
class DeltaRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🤖 Điều khiển Robot Delta")
        self.setGeometry(100, 100, 900, 700)
        self.setStyleSheet(self.load_stylesheet())

        # Khởi tạo UART (chưa kết nối)
        self.comm = None
        self.uart_connected = False

        # Biến camera
        self.cam_thread = None
        self.cam_running = False

        # Biến CSV
        self.csv_data = []       # du lieu DA CONG OFFSET, dung de di chuyen/hien thi
        self.csv_data_raw = []   # du lieu GOC doc tu file, chua cong offset
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.auto_running = False
        self.auto_stop = False

        # Widget trung tâm
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # === Thanh trạng thái trên cùng ===
        top_bar = QHBoxLayout()
        self.status_indicator = QLabel("⚪ Chưa kết nối")
        self.status_indicator.setStyleSheet("color: gray; font-weight: bold;")
        top_bar.addWidget(self.status_indicator)
        top_bar.addStretch()

        # Chọn cổng COM
        top_bar.addWidget(QLabel("Cổng COM:"))
        self.com_port_combo = QComboBox()
        self.com_port_combo.addItems(UARTComm.list_ports() or ["COM6"])
        self.com_port_combo.setEditable(True)
        top_bar.addWidget(self.com_port_combo)

        self.btn_connect = QPushButton("🔗 Kết nối")
        self.btn_connect.clicked.connect(self.toggle_uart)
        top_bar.addWidget(self.btn_connect)

        main_layout.addLayout(top_bar)

        # === Tab widget ===
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #ccc;
                border-radius: 6px;
                background: #fafafa;
            }
            QTabBar::tab {
                background: #e0e0e0;
                padding: 8px 16px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                font-weight: bold;
            }
        """)
        main_layout.addWidget(self.tabs)

        # Tạo các tab
        self.tab_camera = QWidget()
        self.tab_move = QWidget()
        self.tab_csv = QWidget()

        self.tabs.addTab(self.tab_camera, "📷  Camera")
        self.tabs.addTab(self.tab_move, "🕹️  Di chuyển")
        self.tabs.addTab(self.tab_csv, "📂  CSV & Tự động")

        self.build_tab_camera()
        self.build_tab_move()
        self.build_tab_csv()

        # === Log console ===
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        self.log_text.setStyleSheet("background: #1e1e1e; color: #d4d4d4; font-family: monospace;")
        main_layout.addWidget(self.log_text)

        # Timer để cập nhật trạng thái
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)

        # Log khởi tạo
        self.log("🚀 Ứng dụng khởi động")

    def load_stylesheet(self):
        return """
            QMainWindow {
                background: #f0f2f5;
            }
            QPushButton {
                background-color: #4a90e2;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #5a9cf2;
            }
            QPushButton:pressed {
                background-color: #3a7fc8;
            }
            QPushButton:disabled {
                background-color: #b0b0b0;
                color: #666;
            }
            QLineEdit, QComboBox, QListWidget {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 4px 6px;
                background: white;
            }
            QLineEdit:focus, QComboBox:focus {
                border: 1px solid #4a90e2;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #ccc;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }
            QLabel {
                color: #333;
            }
            QTabWidget::pane {
                background: #ffffff;
                border-radius: 6px;
            }
        """

    def log(self, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {msg}")

    # ==================== UART ====================
    def toggle_uart(self):
        if self.uart_connected:
            self.disconnect_uart()
        else:
            self.connect_uart()

    def connect_uart(self):
        port = self.com_port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "Cảnh báo", "Vui lòng nhập cổng COM")
            return
        self.comm = UARTComm(port=port, dry_run=False, log_callback=self.log)
        if self.comm.connect():
            self.uart_connected = True
            self.btn_connect.setText("🔌 Ngắt kết nối")
            self.status_indicator.setText("🟢 Đã kết nối")
            self.status_indicator.setStyleSheet("color: green; font-weight: bold;")
            self.log(f"✅ Kết nối thành công {port}")
        else:
            self.comm = None
            self.uart_connected = False

    def disconnect_uart(self):
        if self.comm:
            self.comm.disconnect()
            self.comm = None
        self.uart_connected = False
        self.btn_connect.setText("🔗 Kết nối")
        self.status_indicator.setText("🔴 Đã ngắt")
        self.status_indicator.setStyleSheet("color: red; font-weight: bold;")
        self.log("⛔ Ngắt kết nối")

    def update_status(self):
        pass

    # ==================== Tab Camera ====================
    def build_tab_camera(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(15, 15, 15, 15)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addWidget(QLabel("Camera ID:"))
        self.cam_id_edit = QLineEdit("0")
        self.cam_id_edit.setFixedWidth(50)
        ctrl_layout.addWidget(self.cam_id_edit)
        self.btn_cam = QPushButton("📷 Mở camera")
        self.btn_cam.clicked.connect(self.toggle_camera)
        ctrl_layout.addWidget(self.btn_cam)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

        self.cam_label = QLabel("Chưa mở camera")
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setStyleSheet("background-color: #222; color: white; border-radius: 8px;")
        self.cam_label.setMinimumHeight(400)
        layout.addWidget(self.cam_label)

        self.tab_camera.setLayout(layout)

    def toggle_camera(self):
        if self.cam_running:
            self.stop_camera()
        else:
            self.start_camera()

    def start_camera(self):
        try:
            cam_id = int(self.cam_id_edit.text())
            self.cam_thread = CameraThread(cam_id)
            self.cam_thread.image_updated.connect(self.update_camera_image)
            self.cam_thread.start()
            self.cam_running = True
            self.btn_cam.setText("📷 Đóng camera")
            self.log(f"📷 Camera {cam_id} đã mở")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Không mở được camera: {e}")

    def stop_camera(self):
        if self.cam_thread:
            self.cam_thread.stop()
            self.cam_thread = None
        self.cam_running = False
        self.cam_label.setText("Camera đã đóng")
        self.cam_label.setPixmap(QPixmap())
        self.btn_cam.setText("📷 Mở camera")
        self.log("📷 Camera đã đóng")

    def update_camera_image(self, qt_img):
        if qt_img.isNull():
            self.cam_label.setText("⚠️ Không nhận được hình ảnh")
            return
        pixmap = QPixmap.fromImage(qt_img)
        self.cam_label.setPixmap(pixmap.scaled(self.cam_label.width(), self.cam_label.height(),
                                               Qt.KeepAspectRatio, Qt.SmoothTransformation))

    # ==================== Tab Di chuyển ====================
    def build_tab_move(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(15, 15, 15, 15)

        group = QGroupBox("📍 Nhập tọa độ (mm)")
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        self.entry_x = QLineEdit("0")
        self.entry_y = QLineEdit("0")
        self.entry_z = QLineEdit("300")
        self.entry_x.setStyleSheet("font-size: 14px;")
        self.entry_y.setStyleSheet("font-size: 14px;")
        self.entry_z.setStyleSheet("font-size: 14px;")
        form.addRow("X:", self.entry_x)
        form.addRow("Y:", self.entry_y)
        form.addRow("Z:", self.entry_z)
        group.setLayout(form)
        layout.addWidget(group)

        btn_layout = QHBoxLayout()
        btn_move = QPushButton("▶ Di chuyển đến")
        btn_move.clicked.connect(self.move_to_coord)
        btn_layout.addWidget(btn_move)

        btn_home = QPushButton("🏠 Về HOME")
        btn_home.clicked.connect(self.move_home)
        btn_layout.addWidget(btn_home)

        btn_pick = QPushButton("⬇ Hạ xuống Z=Z offset")
        btn_pick.clicked.connect(self.move_pick)
        btn_layout.addWidget(btn_pick)

        layout.addLayout(btn_layout)

        self.status_label = QLabel("✅ Sẵn sàng")
        self.status_label.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 14px; padding: 5px;")
        layout.addWidget(self.status_label)

        layout.addStretch()
        self.tab_move.setLayout(layout)

    def move_to_coord(self):
        try:
            x = float(self.entry_x.text())
            y = float(self.entry_y.text())
            z = float(self.entry_z.text())
            self.status_label.setText(f"⏳ Đang di chuyển đến ({x},{y},{z})...")
            self.status_label.setStyleSheet("color: #ed6c02; font-weight: bold;")
            threading.Thread(target=self._move, args=(x, y, z), daemon=True).start()
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Tọa độ không hợp lệ: {e}")

    def _move(self, x, y, z):
        try:
            if not self.uart_connected:
                self.log("⚠️ Chưa kết nối UART, giả lập di chuyển")
                time.sleep(0.5)
            else:
                t1, t2, t3 = inverse_kinematics(x, y, z)
                self.comm.send_angles(t1, t2, t3)
            self.status_label.setText(f"✅ Đã đến ({x:.1f}, {y:.1f}, {z:.1f})")
            self.status_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
            self.log(f"🔄 Di chuyển đến ({x:.1f}, {y:.1f}, {z:.1f})")
        except Exception as e:
            self.status_label.setText(f"❌ Lỗi: {e}")
            self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
            self.log(f"❌ Lỗi di chuyển: {e}")

    def _move_home(self) -> bool:
        """Ve HOME THAT SU: gui lenh 'HOME' cho STM32, roi CHO cho den khi
        STM32 xac nhan da cham du 3 cong tac hanh trinh bang cach tra ve
        dong 'READY' (xem ham doHoming()/xuly_Uart() ben firmware STM32).

        KHONG con gui goc (0,0,0) nhu truoc nua, vi gia tri goc khong noi
        len viec robot da thuc su cham cong tac hanh trinh hay chua.

        Tra ve True neu STM32 xac nhan HOME DONE trong thoi gian cho phep,
        False neu timeout / mat ket noi — luc do KHONG duoc coi la da home.
        """
        try:
            if not self.uart_connected or self.comm is None:
                self.log("⚠️ Chưa kết nối UART, giả lập về HOME")
                time.sleep(0.5)
                self.status_label.setText("✅ Đã về HOME (giả lập, chưa kết nối thật)")
                self.status_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
                return True

            self.status_label.setText("⏳ Đang HOME — chờ chạm đủ 3 công tắc hành trình...")
            self.status_label.setStyleSheet("color: #ed6c02; font-weight: bold;")
            self.log("🏠 Gửi lệnh HOME cho STM32, chờ xác nhận READY...")

            ok = self.comm.send_home_and_wait()

            if ok:
                self.status_label.setText("✅ HOME DONE (đã chạm đủ 3 công tắc hành trình)")
                self.status_label.setStyleSheet("color: #2e7d32; font-weight: bold;")
                self.log("🏠 STM32 xác nhận HOME DONE (nhận được READY)")
                return True
            else:
                self.status_label.setText("❌ HOME thất bại / timeout — chưa chạm đủ 3 công tắc")
                self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
                self.log("❌ Không nhận được xác nhận HOME (READY) từ STM32")
                return False
        except Exception as e:
            self.status_label.setText(f"❌ Lỗi: {e}")
            self.status_label.setStyleSheet("color: #d32f2f; font-weight: bold;")
            self.log(f"❌ Lỗi gửi HOME: {e}")
            return False

    def move_home(self):
        self.entry_x.setText("0")
        self.entry_y.setText("0")
        self.entry_z.setText("0")
        threading.Thread(target=self._move_home, daemon=True).start()

    def move_pick(self):
        try:
            x = float(self.entry_x.text())
            y = float(self.entry_y.text())
            self.entry_z.setText(str(PICK_Z))
            self.move_to_coord()
        except:
            pass

    # ==================== Tab CSV ====================
    def build_tab_csv(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(15, 15, 15, 15)

        hbox = QHBoxLayout()
        self.csv_path_edit = QLineEdit()
        self.csv_path_edit.setPlaceholderText("Chưa chọn file CSV")
        hbox.addWidget(self.csv_path_edit)
        btn_load = QPushButton("📂 Chọn CSV")
        btn_load.clicked.connect(self.load_csv)
        hbox.addWidget(btn_load)
        layout.addLayout(hbox)

        # === Khung nhap offset X/Y: bu do lech giua tam ban co va tam base robot ===
        offset_group = QGroupBox("🎯 Offset bù tọa độ (mm) — tâm bàn cờ lệch so với tâm base robot")
        offset_form = QHBoxLayout()
        offset_form.addWidget(QLabel("Offset X:"))
        self.offset_x_edit = QLineEdit("0")
        self.offset_x_edit.setFixedWidth(80)
        offset_form.addWidget(self.offset_x_edit)
        offset_form.addWidget(QLabel("Offset Y:"))
        self.offset_y_edit = QLineEdit("0")
        self.offset_y_edit.setFixedWidth(80)
        offset_form.addWidget(self.offset_y_edit)
        btn_apply_offset = QPushButton("✅ Áp dụng offset")
        btn_apply_offset.clicked.connect(self.apply_offset)
        offset_form.addWidget(btn_apply_offset)
        offset_form.addStretch()
        offset_group.setLayout(offset_form)
        layout.addWidget(offset_group)

        splitter = QSplitter(Qt.Vertical)
        self.csv_list = QListWidget()
        self.csv_list.setStyleSheet("font-family: monospace;")
        splitter.addWidget(self.csv_list)

        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        splitter.addWidget(self.progress_bar)

        layout.addWidget(splitter)

        ctrl_layout = QHBoxLayout()
        ctrl_layout.addWidget(QLabel("corner_index:"))
        self.corner_idx_edit = QLineEdit()
        self.corner_idx_edit.setFixedWidth(80)
        ctrl_layout.addWidget(self.corner_idx_edit)
        btn_move_idx = QPushButton("▶ Di chuyển đến điểm này")
        btn_move_idx.clicked.connect(self.move_to_selected)
        ctrl_layout.addWidget(btn_move_idx)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

        auto_layout = QHBoxLayout()
        btn_auto = QPushButton("▶ Chạy tự động tất cả điểm")
        btn_auto.clicked.connect(self.run_auto_sequence)
        auto_layout.addWidget(btn_auto)
        btn_stop = QPushButton("⏹ Dừng")
        btn_stop.clicked.connect(self.stop_auto)
        auto_layout.addWidget(btn_stop)
        auto_layout.addStretch()
        layout.addLayout(auto_layout)

        self.tab_csv.setLayout(layout)

    def load_csv(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Chọn file CSV", "", "CSV files (*.csv)")
        if not filepath:
            return
        self.csv_path_edit.setText(filepath)
        self.csv_data_raw = []
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    idx = int(row.get('corner_index', 0))
                    x_mm = float(row.get('x_mm', 0))
                    y_mm = float(row.get('y_mm', 0))
                    z_mm = float(row.get('z_mm', 345.0)) if 'z_mm' in row else 345.0
                    self.csv_data_raw.append((idx, x_mm, y_mm, z_mm))
            self.log(f"📂 Đã load {len(self.csv_data_raw)} điểm từ CSV (tọa độ gốc, chưa cộng offset)")
            self.apply_offset()
        except Exception as e:
            QMessageBox.critical(self, "Lỗi", f"Không đọc được CSV: {e}")

    def apply_offset(self):
        """Cong offset X/Y (bu do lech tam ban co - tam base robot) vao toan bo
        toa do da load tu CSV, roi cap nhat lai danh sach hien thi.
        Co the goi lai nhieu lan (khi doi offset) ma KHONG can load lai file."""
        if not self.csv_data_raw:
            QMessageBox.warning(self, "Cảnh báo", "Chưa load file CSV để áp dụng offset")
            return
        try:
            self.offset_x = float(self.offset_x_edit.text())
            self.offset_y = float(self.offset_y_edit.text())
        except ValueError:
            QMessageBox.critical(self, "Lỗi", "Offset X/Y phải là số hợp lệ")
            return

        self.csv_data = []
        self.csv_list.clear()
        for idx, x_raw, y_raw, z_mm in self.csv_data_raw:
            x_mm = x_raw + self.offset_x
            y_mm = y_raw + self.offset_y
            self.csv_data.append((idx, x_mm, y_mm, z_mm))
            self.csv_list.addItem(f"{idx:3d} → ({x_mm:8.3f}, {y_mm:8.3f}, {z_mm:8.3f})")

        self.log(f"🎯 Đã áp dụng offset X={self.offset_x:.3f}mm, Y={self.offset_y:.3f}mm "
                 f"cho {len(self.csv_data)} điểm")

    def move_to_selected(self):
        if not self.csv_data:
            QMessageBox.warning(self, "Cảnh báo", "Chưa load file CSV")
            return
        try:
            idx = int(self.corner_idx_edit.text())
            for item in self.csv_data:
                if item[0] == idx:
                    x, y, z = item[1], item[2], item[3]
                    self.log(f"📌 Di chuyển đến điểm {idx} ({x:.1f}, {y:.1f}, {z:.1f})")
                    threading.Thread(target=self._move, args=(x, y, z), daemon=True).start()
                    return
            QMessageBox.critical(self, "Lỗi", f"Không tìm thấy corner_index {idx}")
        except:
            QMessageBox.critical(self, "Lỗi", "corner_index không hợp lệ")

    def run_auto_sequence(self):
        if not self.csv_data:
            QMessageBox.warning(self, "Cảnh báo", "Chưa load file CSV")
            return
        if self.auto_running:
            QMessageBox.information(self, "Thông báo", "Đã đang chạy")
            return
        self.auto_running = True
        self.auto_stop = False
        self.progress_bar.setVisible(True)
        self.progress_bar.setMaximum(len(self.csv_data))
        self.progress_bar.setValue(0)
        threading.Thread(target=self._auto_loop, daemon=True).start()

    def _auto_loop(self):
        HOME_INTERVAL = 15  # cu sau 15 diem thi ve HOME 1 lan
        try:
            self.log("🏁 Bắt đầu chuỗi tự động")
            if not self._move_home():
                self.log("⛔ HOME đầu chuỗi thất bại → dừng chuỗi tự động")
                return

            for i, (idx, x, y, z) in enumerate(self.csv_data):
                if self.auto_stop:
                    break
                self.log(f"🔄 Điểm {i+1}/{len(self.csv_data)} (idx={idx})")
                self._move(x, y, z)
                time.sleep(2)
                self._move(x, y, PICK_Z)
                time.sleep(1)
                self._move(x, y, PICK_Z-15)
                time.sleep(WAIT_PICK)
                self.progress_bar.setValue(i+1)

                # Cu sau moi HOME_INTERVAL diem thi cho robot ve HOME 1 lan
                if (i + 1) % HOME_INTERVAL == 0 and not self.auto_stop:
                    self.log(f"🏠 Đã xong {i+1} điểm → về HOME")
                    if not self._move_home():
                        self.log("⛔ HOME thất bại giữa chuỗi → dừng chuỗi tự động")
                        return

            # Neu diem cuoi cung khong roi dung vao moc HOME_INTERVAL thi ve HOME lan cuoi
            if not self.auto_stop and len(self.csv_data) % HOME_INTERVAL != 0:
                if not self._move_home():
                    self.log("⛔ HOME cuối chuỗi thất bại")
                    return

            self.status_label.setText("✅ Hoàn thành tự động")
            self.log("✅ Hoàn thành chuỗi tự động")
        except Exception as e:
            self.status_label.setText(f"❌ Lỗi: {e}")
            self.log(f"❌ Lỗi tự động: {e}")
        finally:
            self.auto_running = False
            self.progress_bar.setVisible(False)

    def stop_auto(self):
        self.auto_stop = True
        self.status_label.setText("⏹ Đang dừng...")
        self.log("⏹ Yêu cầu dừng tự động")

    # ==================== Đóng ứng dụng ====================
    def closeEvent(self, event):
        self.stop_camera()
        if self.uart_connected:
            self.disconnect_uart()
        event.accept()

# ==================== Chạy ứng dụng ====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = DeltaRobotGUI()
    window.show()
    sys.exit(app.exec_())