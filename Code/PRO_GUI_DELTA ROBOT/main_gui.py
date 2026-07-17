# ============================================================================
# FILE: main_gui.py - Giao diện điều khiển robot Delta
# ============================================================================
"""
main_gui.py - Giao diện điều khiển robot Delta
Tập trung thiết kế UI và kết nối với RobotController
"""

import os
import threading
import traceback
from datetime import datetime

import numpy as np

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QSpinBox,
    QComboBox, QLineEdit, QPlainTextEdit, QFileDialog, QTableWidget,
    QTableWidgetItem, QMessageBox, QSplitter, QAbstractItemView, QCheckBox
)

import matplotlib

matplotlib.use("Qt5Agg")

# Import các module
from robot_controller import RobotController
from camera_module import (
    CameraThread,
    ObjectDetector,
    ColorRange,
    DetectedObject
)
from Math_Control.gear_ratio import GEAR_RATIO

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


# =========================================================================
#  MAIN WINDOW - CHỈ TẬP TRUNG VÀO GIAO DIỆN
# =========================================================================
class MainWindow(QMainWindow):
    """Cửa sổ chính - Chỉ quản lý giao diện và kết nối với controller"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"Dieu khien Robot Delta (gear ratio u={GEAR_RATIO})")
        self.resize(1200, 820)

        # ---- Khởi tạo các biến trước ----
        self.log_console = None
        self.camera_thread = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.undistorted_frame = None
        self.calib_npz_loaded = False
        self.detector = ObjectDetector()
        self.detector.add_red_color()
        self._last_pixel_xy = None
        self._last_robot_xy = None
        self.last_open_dir = os.getcwd()

        # ---- Xây dựng giao diện TRƯỚC ----
        self._build_ui()

        # ---- Khởi tạo controller SAU khi có log_console ----
        self.controller = RobotController(log_callback=self.append_log)
        self.controller.set_connection_callback(self.on_connection_changed)

    # ------------------------------------------------------------------
    #  PHƯƠNG THỨC LOG
    # ------------------------------------------------------------------
    def append_log(self, text: str):
        """Ghi log vào console"""
        if self.log_console is not None:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_console.appendPlainText(f"[{timestamp}] {text}")

    # ------------------------------------------------------------------
    #  XÂY DỰNG GIAO DIỆN
    # ------------------------------------------------------------------
    def _build_ui(self):
        """Xây dựng toàn bộ giao diện"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Thanh kết nối
        main_layout.addWidget(self._build_connection_bar())

        # Tabs
        splitter = QSplitter(Qt.Vertical)
        self.tabs = QTabWidget()
        splitter.addWidget(self.tabs)

        self.tabs.addTab(self._build_tab_free_jog(), "Dieu khien tu do")
        self.tabs.addTab(self._build_tab_load_file(), "Load file toa do")
        self.tabs.addTab(self._build_tab_camera(), "Camera && Nhan toa do")
        self.tabs.addTab(self._build_tab_system(), "He thong / Log")

        # Log panel
        splitter.addWidget(self._build_log_panel())
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        main_layout.addWidget(splitter)

    # ------------------------------------------------------------------
    #  THANH KẾT NỐI
    # ------------------------------------------------------------------
    def _build_connection_bar(self):
        """Thanh kết nối UART"""
        box = QGroupBox("Ket noi UART & Dieu khien nhanh")
        layout = QHBoxLayout(box)

        layout.addWidget(QLabel("Cong:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(120)
        self.refresh_ports()
        layout.addWidget(self.combo_port)

        btn_refresh = QPushButton("Lam moi")
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)

        layout.addWidget(QLabel("Baud:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["9600", "19200", "38400", "57600", "115200", "250000"])
        self.combo_baud.setCurrentText("115200")
        layout.addWidget(self.combo_baud)

        self.btn_connect = QPushButton("Ket noi")
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)

        self.lbl_status = QLabel("● Chua ket noi")
        self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
        layout.addWidget(self.lbl_status)

        layout.addStretch()

        btn_home = QPushButton("Home")
        btn_home.clicked.connect(self.on_home)
        layout.addWidget(btn_home)

        btn_estop = QPushButton("E-STOP")
        btn_estop.setStyleSheet(
            "background-color: #d9534f; color: white; font-weight: bold; padding: 4px 14px;"
        )
        btn_estop.clicked.connect(self.on_estop)
        layout.addWidget(btn_estop)

        return box

    def refresh_ports(self):
        """Làm mới danh sách cổng COM"""
        self.combo_port.clear()
        if hasattr(self, 'controller'):
            ports = self.controller.list_ports()
            if ports:
                self.combo_port.addItems(ports)
            else:
                self.combo_port.addItem("Khong tim thay cong")
        else:
            self.combo_port.addItem("Khong tim thay cong")

    def toggle_connection(self):
        """Bật/tắt kết nối"""
        if hasattr(self, 'controller'):
            if self.controller.is_connected():
                self.controller.disconnect()
            else:
                port = self.combo_port.currentText()
                baud = int(self.combo_baud.currentText())
                self.controller.connect(port, baud)

    def on_connection_changed(self, connected):
        """Cập nhật trạng thái kết nối"""
        if connected:
            self.lbl_status.setText("● Da ket noi")
            self.lbl_status.setStyleSheet("color: green; font-weight: bold;")
            self.btn_connect.setText("Ngat ket noi")
        else:
            self.lbl_status.setText("● Chua ket noi")
            self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
            self.btn_connect.setText("Ket noi")

    def on_home(self):
        """Đưa robot về home"""
        if hasattr(self, 'controller'):
            self.controller.home()

    def on_estop(self):
        """Dừng khẩn cấp"""
        if hasattr(self, 'controller'):
            self.controller.emergency_stop()
        self.append_log("[CANH BAO] EMERGENCY STOP da duoc kich hoat!")

    # ------------------------------------------------------------------
    #  TAB 1: ĐIỀU KHIỂN TỰ DO
    # ------------------------------------------------------------------
    def _build_tab_free_jog(self):
        """Tab điều khiển tự do"""
        w = QWidget()
        layout = QHBoxLayout(w)

        # ---- Nhập tọa độ ----
        box_manual = QGroupBox("Nhap toa do truc tiep (X, Y, Z)")
        form = QGridLayout(box_manual)

        form.addWidget(QLabel("X (mm):"), 0, 0)
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-500, 500)
        self.spin_x.setDecimals(2)
        form.addWidget(self.spin_x, 0, 1)

        form.addWidget(QLabel("Y (mm):"), 1, 0)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-500, 500)
        self.spin_y.setDecimals(2)
        form.addWidget(self.spin_y, 1, 1)

        form.addWidget(QLabel("Z (mm):"), 2, 0)
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(-500, 500)
        self.spin_z.setDecimals(2)
        form.addWidget(self.spin_z, 2, 1)

        form.addWidget(QLabel("Feedrate:"), 3, 0)
        self.spin_feed_manual = QDoubleSpinBox()
        self.spin_feed_manual.setRange(1, 20000)
        self.spin_feed_manual.setValue(1500)
        form.addWidget(self.spin_feed_manual, 3, 1)

        btn_move = QPushButton("DI CHUYEN DEN TOA DO")
        btn_move.setStyleSheet("padding: 8px; font-weight: bold;")
        btn_move.clicked.connect(self.on_manual_move)
        form.addWidget(btn_move, 4, 0, 1, 2)

        self.lbl_manual_angles = QLabel("Goc khop / dong co: --")
        self.lbl_manual_angles.setWordWrap(True)
        form.addWidget(self.lbl_manual_angles, 5, 0, 1, 2)

        layout.addWidget(box_manual)

        # ---- Jog ----
        box_jog = QGroupBox("Jog theo buoc")
        jog_layout = QVBoxLayout(box_jog)

        step_layout = QHBoxLayout()
        step_layout.addWidget(QLabel("Buoc nhay (mm):"))
        self.spin_step = QDoubleSpinBox()
        self.spin_step.setRange(0.1, 100)
        self.spin_step.setValue(5)
        step_layout.addWidget(self.spin_step)
        jog_layout.addLayout(step_layout)

        grid = QGridLayout()
        btn_yplus = QPushButton("Y+")
        btn_yminus = QPushButton("Y-")
        btn_xplus = QPushButton("X+")
        btn_xminus = QPushButton("X-")
        btn_zplus = QPushButton("Z+")
        btn_zminus = QPushButton("Z-")
        for b in (btn_yplus, btn_yminus, btn_xplus, btn_xminus, btn_zplus, btn_zminus):
            b.setMinimumSize(60, 45)

        grid.addWidget(btn_yplus, 0, 1)
        grid.addWidget(btn_xminus, 1, 0)
        grid.addWidget(btn_xplus, 1, 2)
        grid.addWidget(btn_yminus, 2, 1)
        grid.addWidget(btn_zplus, 0, 3)
        grid.addWidget(btn_zminus, 2, 3)
        jog_layout.addLayout(grid)

        btn_xplus.clicked.connect(lambda: self.on_jog("X", 1))
        btn_xminus.clicked.connect(lambda: self.on_jog("X", -1))
        btn_yplus.clicked.connect(lambda: self.on_jog("Y", 1))
        btn_yminus.clicked.connect(lambda: self.on_jog("Y", -1))
        btn_zplus.clicked.connect(lambda: self.on_jog("Z", 1))
        btn_zminus.clicked.connect(lambda: self.on_jog("Z", -1))

        jog_layout.addStretch()
        layout.addWidget(box_jog)

        return w

    def on_manual_move(self):
        """Di chuyển đến tọa độ nhập"""
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return

        x, y, z = self.spin_x.value(), self.spin_y.value(), self.spin_z.value()
        f = self.spin_feed_manual.value()
        threading.Thread(target=self.controller.move_to, args=(x, y, z, f), daemon=True).start()

    def on_jog(self, axis, direction):
        """Jog theo trục"""
        step = self.spin_step.value() * direction
        if axis == "X":
            self.spin_x.setValue(self.spin_x.value() + step)
        elif axis == "Y":
            self.spin_y.setValue(self.spin_y.value() + step)
        elif axis == "Z":
            self.spin_z.setValue(self.spin_z.value() + step)
        self.on_manual_move()

    # ------------------------------------------------------------------
    #  TAB 2: LOAD FILE (TIẾP TỤC)
    # ------------------------------------------------------------------
    def _build_tab_load_file(self):
        """Tab load file tọa độ"""
        w = QWidget()
        layout = QVBoxLayout(w)

        top_bar = QHBoxLayout()
        btn_browse = QPushButton("Chon file...")
        btn_browse.clicked.connect(self.on_browse_file)
        top_bar.addWidget(btn_browse)

        self.lbl_file_path = QLabel("Chua chon file")
        top_bar.addWidget(self.lbl_file_path)
        top_bar.addStretch()

        top_bar.addWidget(QLabel("Feedrate mac dinh:"))
        self.file_feed = QDoubleSpinBox()
        self.file_feed.setRange(1, 20000)
        self.file_feed.setValue(1200)
        top_bar.addWidget(self.file_feed)

        self.file_has_header = QCheckBox("File co dong tieu de (header)")
        self.file_has_header.setChecked(True)
        top_bar.addWidget(self.file_has_header)

        layout.addLayout(top_bar)

        note = QLabel(
            "Ho tro CSV va JSON.\n"
            "CSV: cot X, Y, Z (bat buoc) va F (tuy chon).\n"
            "JSON: mang cac object chua robot_x, robot_y, robot_z (va f tuy chon)."
        )
        note.setStyleSheet("color: gray;")
        layout.addWidget(note)

        self.table_points = QTableWidget(0, 4)
        self.table_points.setHorizontalHeaderLabels(["X", "Y", "Z", "F"])
        self.table_points.setEditTriggers(QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.table_points)

        btn_bar = QHBoxLayout()
        self.btn_run_file = QPushButton("CHAY CHUOI TOA DO")
        self.btn_run_file.setStyleSheet("padding: 8px; font-weight: bold; background-color:#5cb85c; color:white;")
        self.btn_run_file.clicked.connect(self.on_run_file)
        btn_bar.addWidget(self.btn_run_file)

        self.btn_stop_file = QPushButton("DUNG")
        self.btn_stop_file.setStyleSheet("padding: 8px; font-weight: bold; background-color:#d9534f; color:white;")
        self.btn_stop_file.clicked.connect(self.on_stop_motion)
        btn_bar.addWidget(self.btn_stop_file)

        self.lbl_file_progress = QLabel("San sang")
        btn_bar.addWidget(self.lbl_file_progress)
        btn_bar.addStretch()
        layout.addLayout(btn_bar)

        return w

    def on_browse_file(self):
        """Chọn file tọa độ"""
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Chon file toa do",
            self.last_open_dir,
            "CSV Files (*.csv);;JSON Files (*.json);;All Files (*)"
        )
        if not path:
            return

        self.last_open_dir = os.path.dirname(path)
        self.lbl_file_path.setText(os.path.basename(path))

        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return

        ext = os.path.splitext(path)[1].lower()
        try:
            if ext == '.json':
                points = self.controller.load_json_file(path)
            else:
                has_header = self.file_has_header.isChecked()
                points = self.controller.load_csv_file(path, has_header)

            self._update_table(points)

        except Exception as e:
            QMessageBox.critical(self, "Loi doc file", str(e))
            self.append_log(f"[LOI] Doc file: {e}")

    def _update_table(self, points):
        """Cập nhật bảng hiển thị"""
        self.table_points.setRowCount(len(points))
        for i, pt in enumerate(points):
            for c in range(3):
                self.table_points.setItem(i, c, QTableWidgetItem(f"{pt[c]:.3f}"))
            f_display = f"{pt[3]:.1f}" if len(pt) == 4 else ""
            self.table_points.setItem(i, 3, QTableWidgetItem(f_display))

    def on_run_file(self):
        """Chạy các điểm đã load"""
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return

        if not self.controller.is_connected():
            QMessageBox.warning(self, "Chua ket noi", "Vui long ket noi robot truoc khi chay.")
            return

        feed = self.file_feed.value()
        self.controller.run_loaded_points(
            feed,
            progress_callback=lambda done, total: self.lbl_file_progress.setText(f"Dang chay: {done}/{total}")
        )

    def on_stop_motion(self):
        """Dừng chuyển động"""
        if hasattr(self, 'controller'):
            self.controller.stop_motion()
        self.lbl_file_progress.setText("Da dung")

    # ------------------------------------------------------------------
    #  TAB 3: CAMERA (TIẾP TỤC)
    # ------------------------------------------------------------------
    def _build_tab_camera(self):
        """Tab camera và nhận dạng"""
        w = QWidget()
        outer = QHBoxLayout(w)

        # ---- Cột trái: Hiển thị camera ----
        left = QVBoxLayout()
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("Camera index:"))
        self.spin_cam_index = QSpinBox()
        self.spin_cam_index.setRange(0, 10)
        top_bar.addWidget(self.spin_cam_index)

        self.btn_cam_toggle = QPushButton("Bat Camera")
        self.btn_cam_toggle.clicked.connect(self.on_toggle_camera)
        top_bar.addWidget(self.btn_cam_toggle)

        btn_snapshot = QPushButton("Chup anh")
        btn_snapshot.clicked.connect(self.on_snapshot)
        top_bar.addWidget(btn_snapshot)
        top_bar.addStretch()
        left.addLayout(top_bar)

        self.lbl_camera_view = QLabel("Camera dang tat")
        self.lbl_camera_view.setAlignment(Qt.AlignCenter)
        self.lbl_camera_view.setMinimumSize(560, 420)
        self.lbl_camera_view.setStyleSheet("background-color: #222; color: white;")
        left.addWidget(self.lbl_camera_view)

        if not HAS_CV2:
            warn = QLabel("Chua cai opencv-python. Chay: pip install opencv-python")
            warn.setStyleSheet("color: orange;")
            left.addWidget(warn)

        outer.addLayout(left, 2)

        # ---- Cột phải: Điều khiển ----
        right = QVBoxLayout()

        # HSV Filter
        box_hsv = QGroupBox("1. Loc mau vat the (HSV)")
        hsv_form = QGridLayout(box_hsv)
        self.spin_h_low = QSpinBox()
        self.spin_h_low.setRange(0, 179)
        self.spin_h_low.setValue(0)
        self.spin_s_low = QSpinBox()
        self.spin_s_low.setRange(0, 255)
        self.spin_s_low.setValue(120)
        self.spin_v_low = QSpinBox()
        self.spin_v_low.setRange(0, 255)
        self.spin_v_low.setValue(70)
        self.spin_h_high = QSpinBox()
        self.spin_h_high.setRange(0, 179)
        self.spin_h_high.setValue(10)
        self.spin_s_high = QSpinBox()
        self.spin_s_high.setRange(0, 255)
        self.spin_s_high.setValue(255)
        self.spin_v_high = QSpinBox()
        self.spin_v_high.setRange(0, 255)
        self.spin_v_high.setValue(255)

        hsv_form.addWidget(QLabel("H thap:"), 0, 0)
        hsv_form.addWidget(self.spin_h_low, 0, 1)
        hsv_form.addWidget(QLabel("S thap:"), 1, 0)
        hsv_form.addWidget(self.spin_s_low, 1, 1)
        hsv_form.addWidget(QLabel("V thap:"), 2, 0)
        hsv_form.addWidget(self.spin_v_low, 2, 1)
        hsv_form.addWidget(QLabel("H cao:"), 0, 2)
        hsv_form.addWidget(self.spin_h_high, 0, 3)
        hsv_form.addWidget(QLabel("S cao:"), 1, 2)
        hsv_form.addWidget(self.spin_s_high, 1, 3)
        hsv_form.addWidget(QLabel("V cao:"), 2, 2)
        hsv_form.addWidget(self.spin_v_high, 2, 3)
        note_hsv = QLabel("Mac dinh: loc mau DO. Chinh lai theo mau vat that.")
        note_hsv.setStyleSheet("color: gray;")
        hsv_form.addWidget(note_hsv, 3, 0, 1, 4)
        right.addWidget(box_hsv)

        # Detection
        box_detect = QGroupBox("2. Phat hien vat")
        detect_layout = QVBoxLayout(box_detect)
        btn_detect = QPushButton("Phat hien vat (dung frame moi nhat)")
        btn_detect.clicked.connect(self.on_detect_object)
        detect_layout.addWidget(btn_detect)
        self.lbl_detect_result = QLabel("Chua phat hien")
        detect_layout.addWidget(self.lbl_detect_result)
        right.addWidget(box_detect)

        # Calibration
        box_calib = QGroupBox("3. Hieu chinh camera")
        calib_layout = QVBoxLayout(box_calib)
        calib_form = QGridLayout()
        calib_form.addWidget(QLabel("X thuc (mm):"), 0, 0)
        self.calib_x = QDoubleSpinBox()
        self.calib_x.setRange(-500, 500)
        calib_form.addWidget(self.calib_x, 0, 1)
        calib_form.addWidget(QLabel("Y thuc (mm):"), 0, 2)
        self.calib_y = QDoubleSpinBox()
        self.calib_y.setRange(-500, 500)
        calib_form.addWidget(self.calib_y, 0, 3)
        calib_layout.addLayout(calib_form)

        note_calib = QLabel(
            "Cach lam: dat vat mau tai vi tri robot biet truoc (X,Y) o tren, bam "
            "'Phat hien vat' de lay pixel, roi bam 'Them cap diem'. Lap lai >= 3 lan "
            "o cac vi tri khac nhau, sau do bam 'Tinh hieu chinh'."
        )
        note_calib.setWordWrap(True)
        note_calib.setStyleSheet("color: gray;")
        calib_layout.addWidget(note_calib)

        calib_btns = QHBoxLayout()
        btn_add_pair = QPushButton("Them cap diem")
        btn_add_pair.clicked.connect(self.on_add_calibration_pair)
        calib_btns.addWidget(btn_add_pair)
        btn_compute = QPushButton("Tinh hieu chinh")
        btn_compute.clicked.connect(self.on_compute_calibration)
        calib_btns.addWidget(btn_compute)
        btn_clear_calib = QPushButton("Xoa het")
        btn_clear_calib.clicked.connect(self.on_clear_calibration)
        calib_btns.addWidget(btn_clear_calib)
        calib_layout.addLayout(calib_btns)

        # Load NPZ
        calib_load_layout = QHBoxLayout()
        btn_load_npz = QPushButton("Load NPZ (camera calib)")
        btn_load_npz.clicked.connect(self.on_load_calib_npz)
        calib_load_layout.addWidget(btn_load_npz)

        btn_clear_npz = QPushButton("Xóa NPZ")
        btn_clear_npz.clicked.connect(self.on_clear_calib_npz)
        calib_load_layout.addWidget(btn_clear_npz)

        self.lbl_calib_npz_status = QLabel("Chưa load")
        calib_load_layout.addWidget(self.lbl_calib_npz_status)
        calib_layout.addLayout(calib_load_layout)

        self.lbl_calib_status = QLabel("Chua co diem hieu chinh nao")
        calib_layout.addWidget(self.lbl_calib_status)
        right.addWidget(box_calib)

        # Move to detected
        box_move = QGroupBox("4. Di chuyen robot toi vat vua phat hien")
        move_layout = QVBoxLayout(box_move)
        self.lbl_robot_coord = QLabel("Toa do robot: --")
        move_layout.addWidget(self.lbl_robot_coord)

        move_form = QHBoxLayout()
        move_form.addWidget(QLabel("Z (mm):"))
        self.vision_z = QDoubleSpinBox()
        self.vision_z.setRange(-500, 500)
        move_form.addWidget(self.vision_z)
        move_form.addWidget(QLabel("Feed:"))
        self.vision_feed = QDoubleSpinBox()
        self.vision_feed.setRange(1, 20000)
        self.vision_feed.setValue(1200)
        move_form.addWidget(self.vision_feed)
        move_layout.addLayout(move_form)

        btn_move_vision = QPushButton("DI CHUYEN ROBOT TOI VAT")
        btn_move_vision.setStyleSheet("padding: 8px; font-weight: bold; background-color:#f0ad4e; color:white;")
        btn_move_vision.clicked.connect(self.on_move_to_detected)
        move_layout.addWidget(btn_move_vision)
        right.addWidget(box_move)

        right.addStretch()
        outer.addLayout(right, 2)

        return w

    # ===== CAMERA METHODS =====
    def on_toggle_camera(self):
        """Bật/tắt camera"""
        if self.camera_thread and self.camera_thread.isRunning():
            self.camera_thread.stop()
            self.camera_thread = None
            self.btn_cam_toggle.setText("Bat Camera")
            self.lbl_camera_view.setText("Camera dang tat")
            self.lbl_camera_view.setPixmap(QPixmap())
        else:
            idx = self.spin_cam_index.value()
            self.camera_thread = CameraThread(idx, enable_detection=True)
            self.camera_thread.frame_ready.connect(self.on_frame_ready)
            self.camera_thread.frame_with_detection.connect(self.on_frame_with_detection)
            self.camera_thread.detection_ready.connect(self.on_detection_ready)
            self.camera_thread.log_signal.connect(self.append_log)
            self.camera_thread.start()
            self.btn_cam_toggle.setText("Tat Camera")

    def on_frame_ready(self, qimg: QImage):
        """Xử lý frame từ camera"""
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            try:
                width = qimg.width()
                height = qimg.height()
                ptr = qimg.bits()
                ptr.setsize(qimg.byteCount())
                frame = np.array(ptr).reshape(height, width, 3)
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                undistorted = cv2.undistort(frame_bgr, self.camera_matrix, self.dist_coeffs)
                self.undistorted_frame = undistorted
                undistorted_rgb = cv2.cvtColor(undistorted, cv2.COLOR_BGR2RGB)
                h, w, ch = undistorted_rgb.shape
                bytes_per_line = ch * w
                qimg_display = QImage(undistorted_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimg_display).scaled(
                    self.lbl_camera_view.width(), self.lbl_camera_view.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                self.lbl_camera_view.setPixmap(pixmap)
            except Exception as e:
                self.append_log(f"[CAMERA] Loi undistort: {e}")
                self.lbl_camera_view.setPixmap(QPixmap.fromImage(qimg).scaled(
                    self.lbl_camera_view.width(), self.lbl_camera_view.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation
                ))
        else:
            self.lbl_camera_view.setPixmap(QPixmap.fromImage(qimg).scaled(
                self.lbl_camera_view.width(), self.lbl_camera_view.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation
            ))

    def on_frame_with_detection(self, qimg: QImage):
        """Xử lý frame có vẽ detection"""
        # Có thể hiển thị frame có detection hoặc lưu lại
        pass

    def on_detection_ready(self, detected_obj: DetectedObject):
        """Xử lý khi phát hiện vật thể"""
        self._last_pixel_xy = (detected_obj.center_x, detected_obj.center_y)
        self.lbl_detect_result.setText(
            f"Phát hiện: {detected_obj.color_name} tại "
            f"px={detected_obj.center_x:.1f}, py={detected_obj.center_y:.1f}"
        )
        self.append_log(f"[VISION] Phat hien vat tai pixel ({detected_obj.center_x:.1f}, {detected_obj.center_y:.1f})")

    def on_snapshot(self):
        """Chụp ảnh"""
        if self.undistorted_frame is None:
            QMessageBox.warning(self, "Chua co anh", "Khong co khung hinh de chup.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Luu anh", f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
            "PNG Files (*.png)"
        )
        if not path:
            return
        cv2.imwrite(path, self.undistorted_frame)
        self.append_log(f"[OK] Da luu anh (da khua meo): {path}")

    def _current_hsv_range(self):
        """Lấy khoảng HSV hiện tại"""
        lower = (self.spin_h_low.value(), self.spin_s_low.value(), self.spin_v_low.value())
        upper = (self.spin_h_high.value(), self.spin_s_high.value(), self.spin_v_high.value())
        return lower, upper

    def on_detect_object(self):
        """Phát hiện vật thể từ frame hiện tại"""
        if not (self.camera_thread and self.camera_thread.isRunning()):
            QMessageBox.warning(self, "Camera chua bat", "Vui long bat camera truoc.")
            return
        frame = self.undistorted_frame if self.undistorted_frame is not None else self.camera_thread.latest_frame
        if frame is None:
            QMessageBox.warning(self, "Chua co frame", "Khong co khung hinh tu camera.")
            return

        lower, upper = self._current_hsv_range()
        # Sử dụng detector từ camera_module
        self.detector.clear_color_ranges()
        self.detector.add_color_range(ColorRange(lower, upper))
        result = self.detector.detect(frame)

        if result is None:
            self.lbl_detect_result.setText("Khong phat hien vat nao (chinh lai HSV)")
            self._last_pixel_xy = None
            return
        self._last_pixel_xy = (result.center_x, result.center_y)
        self.lbl_detect_result.setText(f"Diem anh phat hien: px={result.center_x:.1f}, py={result.center_y:.1f}")
        self.append_log(f"[VISION] Phat hien vat tai pixel ({result.center_x:.1f}, {result.center_y:.1f})")

    def on_add_calibration_pair(self):
        """Thêm cặp điểm hiệu chỉnh"""
        if self._last_pixel_xy is None:
            QMessageBox.warning(self, "Chua co diem anh", "Vui long 'Phat hien vat' truoc.")
            return
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return

        robot_xy = (self.calib_x.value(), self.calib_y.value())
        n = self.controller.calibrate_add_point(self._last_pixel_xy, robot_xy)
        self.lbl_calib_status.setText(f"Da co {n} cap diem hieu chinh")
        self.append_log(f"[VISION] Them cap hieu chinh: pixel={self._last_pixel_xy} <-> robot={robot_xy}")

    def on_compute_calibration(self):
        """Tính toán hiệu chỉnh"""
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return
        try:
            n = self.controller.calibrate_compute()
            self.lbl_calib_status.setText(f"Da tinh hieu chinh xong voi {n} cap diem")
            self.append_log("[VISION] Da tinh xong ma tran hieu chinh camera")
        except ValueError as e:
            QMessageBox.warning(self, "Chua du diem", str(e))

    def on_clear_calibration(self):
        """Xóa hiệu chỉnh"""
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return
        self.controller.calibrate_clear()
        self.lbl_calib_status.setText("Chua co diem hieu chinh nao")
        self.append_log("[VISION] Da xoa toan bo diem hieu chinh")

    def on_load_calib_npz(self):
        """Load file NPZ calibration"""
        path, _ = QFileDialog.getOpenFileName(
            self, "Chon file NPZ camera calibration", "", "NPZ Files (*.npz);;All Files (*)"
        )
        if not path:
            return
        try:
            data = np.load(path)
            possible_camera_keys = ['camera_matrix', 'mtx', 'cameraMatrix', 'K', 'cameraMat']
            possible_dist_keys = ['dist_coeffs', 'dist', 'distCoeffs', 'D', 'distortion']

            camera_key = None
            dist_key = None
            for key in possible_camera_keys:
                if key in data:
                    camera_key = key
                    break
            for key in possible_dist_keys:
                if key in data:
                    dist_key = key
                    break

            if camera_key is None or dist_key is None:
                available_keys = list(data.keys())
                QMessageBox.critical(
                    self,
                    "Loi dinh dang",
                    f"Khong tim thay key cho ma tran camera hoac he so meo.\n"
                    f"Cac key co trong file: {available_keys}"
                )
                return

            self.camera_matrix = data[camera_key]
            self.dist_coeffs = data[dist_key]
            self.calib_npz_loaded = True
            self.lbl_calib_npz_status.setText(f"Da load: {os.path.basename(path)}")
            self.append_log(f"[CAMERA] Load calib NPZ thanh cong: {os.path.basename(path)}")

        except Exception as e:
            QMessageBox.critical(self, "Loi load NPZ", str(e))
            self.append_log(f"[CAMERA] Loi load NPZ: {e}")

    def on_clear_calib_npz(self):
        """Xóa NPZ calibration"""
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calib_npz_loaded = False
        self.lbl_calib_npz_status.setText("Chua load")
        self.append_log("[CAMERA] Da xoa thong so khua meo")

    def on_move_to_detected(self):
        """Di chuyển robot đến vật thể phát hiện"""
        if self._last_pixel_xy is None:
            QMessageBox.warning(self, "Chua phat hien vat", "Vui long 'Phat hien vat' truoc.")
            return
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return

        try:
            X, Y = self.controller.pixel_to_robot(*self._last_pixel_xy)
        except ValueError as e:
            QMessageBox.warning(self, "Chua hieu chinh", str(e))
            return

        self._last_robot_xy = (X, Y)
        self.lbl_robot_coord.setText(f"Toa do robot: X={X:.2f}, Y={Y:.2f}")
        z = self.vision_z.value()
        feed = self.vision_feed.value()

        # Sử dụng trajectory approach
        self.controller.approach_target(
            target_x=X,
            target_y=Y,
            target_z=z,
            callback=self.on_approach_complete
        )
        self.append_log(f"[VISION] Bat dau tiep can muc tieu: X={X:.2f}, Y={Y:.2f}, Z={z:.2f}")

    def on_approach_complete(self, success):
        """Callback khi tiếp cận hoàn tất"""
        if success:
            self.append_log("[VISION] Tiep can muc tieu thanh cong!")
            self.lbl_robot_coord.setText("Da den vi tri muc tieu")
        else:
            self.append_log("[VISION] Tiep can muc tieu that bai!")

    # ------------------------------------------------------------------
    #  TAB 4: HỆ THỐNG
    # ------------------------------------------------------------------
    def _build_tab_system(self):
        """Tab hệ thống"""
        w = QWidget()
        layout = QVBoxLayout(w)

        info = QLabel(
            "Tab nay danh cho cac thao tac he thong: kiem tra thu vien, gui lenh tuy y "
            "de test truc tiep voi robot qua UART, va xem log day du o khung ben duoi."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        box_libs = QGroupBox("Trang thai thu vien")
        libs_layout = QVBoxLayout(box_libs)
        libs_layout.addWidget(QLabel(f"pyserial (UART): {'DA CAI' if HAS_CV2 else 'CHUA CAI'}"))
        libs_layout.addWidget(QLabel(f"opencv-python (camera): {'DA CAI' if HAS_CV2 else 'CHUA CAI'}"))
        libs_layout.addWidget(QLabel(f"Gear ratio hien tai: u = {GEAR_RATIO}"))
        layout.addWidget(box_libs)

        box_manual_cmd = QGroupBox("Gui lenh UART tuy y (nang cao)")
        cmd_layout = QHBoxLayout(box_manual_cmd)
        self.txt_manual_cmd = QLineEdit()
        self.txt_manual_cmd.setPlaceholderText("Vi du: T1:0 T2:0 T3:0 F:1000")
        cmd_layout.addWidget(self.txt_manual_cmd)
        btn_send_manual = QPushButton("Gui")
        btn_send_manual.clicked.connect(self.on_send_manual_cmd)
        cmd_layout.addWidget(btn_send_manual)
        layout.addWidget(box_manual_cmd)

        layout.addStretch()
        return w

    def on_send_manual_cmd(self):
        """Gửi lệnh UART thủ công"""
        cmd = self.txt_manual_cmd.text().strip()
        if not cmd:
            return
        if not hasattr(self, 'controller'):
            self.append_log("[LOI] Controller chưa được khởi tạo")
            return
        if not cmd.endswith("\n"):
            cmd += "\n"
        threading.Thread(target=self.controller.uart.send_raw, args=(cmd,), daemon=True).start()

    # ------------------------------------------------------------------
    #  LOG PANEL
    # ------------------------------------------------------------------
    def _build_log_panel(self):
        """Xây dựng panel log"""
        box = QGroupBox("Log he thong")
        layout = QVBoxLayout(box)

        self.log_console = QPlainTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setMaximumBlockCount(2000)
        layout.addWidget(self.log_console)

        btn_bar = QHBoxLayout()
        btn_clear = QPushButton("Xoa log")
        btn_clear.clicked.connect(self.log_console.clear)
        btn_bar.addWidget(btn_clear)

        btn_save = QPushButton("Luu log ra file")
        btn_save.clicked.connect(self.on_save_log)
        btn_bar.addWidget(btn_save)
        btn_bar.addStretch()
        layout.addLayout(btn_bar)

        return box

    def on_save_log(self):
        """Lưu log ra file"""
        if self.log_console is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Luu log", f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt", "Text Files (*.txt)"
        )
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.log_console.toPlainText())
        self.append_log(f"[OK] Da luu log: {path}")

    # ------------------------------------------------------------------
    #  CLOSE EVENT
    # ------------------------------------------------------------------
    def closeEvent(self, event):
        """Xử lý khi đóng cửa sổ"""
        try:
            if hasattr(self, 'controller'):
                if self.controller.motion_thread and self.controller.motion_thread.isRunning():
                    self.controller.motion_thread.stop()
                    self.controller.motion_thread.wait(1000)
                if self.camera_thread and self.camera_thread.isRunning():
                    self.camera_thread.stop()
                if self.controller.is_connected():
                    self.controller.disconnect()
        except Exception:
            traceback.print_exc()
        event.accept()