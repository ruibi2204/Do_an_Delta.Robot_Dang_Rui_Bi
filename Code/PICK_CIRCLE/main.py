import os
import sys
import csv
import json
import time
import threading
import contextlib
from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:
    print("LỖI: chưa cài opencv-python. Chạy: pip install opencv-python")
    sys.exit(1)

try:
    from PySide6.QtCore import Qt, QThread, Signal, QSize, QTimer
    from PySide6.QtGui import QImage, QPixmap, QFont, QColor, QIcon, QKeySequence, QShortcut, QPalette
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QTabWidget, QGridLayout, QVBoxLayout,
        QHBoxLayout, QPushButton, QLabel, QGroupBox, QComboBox, QDoubleSpinBox,
        QSpinBox, QLineEdit, QTextEdit, QTableWidget, QTableWidgetItem, QHeaderView,
        QFileDialog, QMessageBox, QFrame, QSizePolicy, QAbstractItemView, QCheckBox,
        QSplitter, QScrollArea, QRadioButton, QButtonGroup,
    )
except ImportError:
    print("LỖI: chưa cài PySide6. Chạy: pip install PySide6")
    sys.exit(1)

# ---- Các module có sẵn trong project (Vietnamese identifiers giữ nguyên) ----
from Uart_1 import UARTComm, DEFAULT_BAUD as ROBOT_DEFAULT_BAUD, SERIAL_AVAILABLE
from ĐHNghich import inverse_kinematics
from move_delta import DeltaMotionPlanner, TIME_MOVE_FAST
from Uart_2 import PneumaticComm, DEFAULT_BAUD as PNEU_DEFAULT_BAUD
from Cameracircle import (
    CircleTracker,
    calculate_real_properties,
    px_to_mm_scale,
    undistort_image,
    detect_circles_hsv_optimized,
)

if SERIAL_AVAILABLE:
    import serial
    import serial.tools.list_ports

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")

DEFAULT_HOME = (60.0, 0.0, 280.0)
DEFAULT_Z_SAFE = 300.0
DEFAULT_JOG_XY = 5.0
DEFAULT_JOG_Z = 5.0
PNEU_DEFAULT_BAUD = 115200

# NEW: các chế độ xác định tọa độ (tab CAMERA)
#   "circle" -> dùng module Cameracircle.py hiện tại (chỉ xác định tâm vòng tròn X/Y)
#   "dof4"   -> dùng module khác (sẽ làm sau) có xác định thêm góc quay cho bậc tự do 4
COORD_MODE_CIRCLE = "circle"
COORD_MODE_DOF4 = "dof4"


# =====================================================================
# CẤU HÌNH LƯU/ĐỌC FILE JSON
# =====================================================================
def load_config():
    default_cfg = {
        "home_position": list(DEFAULT_HOME),
        "z_safe": DEFAULT_Z_SAFE,
        "jog_step_xy": DEFAULT_JOG_XY,
        "jog_step_z": DEFAULT_JOG_Z,
        "robot_port": "COM9",
        "robot_baud": ROBOT_DEFAULT_BAUD,
        "pneumatic_port": "COM3",
        "pneumatic_baud": PNEU_DEFAULT_BAUD,
        "camera_index": 0,
        "calib_file": "calibration_result.npz",
        "camera_offset_x": 0.0,
        "camera_offset_y": 0.0,
        "csv_offset_x": 0.0,
        "csv_offset_y": 0.0,
        "csv_match_tolerance": 6.0,
        "csv_match_same_color": True,
        "csv_z_pick": 320.0,
        "turn_pwm": 150,
        "step_test_angle": 90.0,
        "coord_mode": COORD_MODE_CIRCLE,       # NEW: chế độ xác định tọa độ đang chọn ở tab CAMERA
        "csv_apply_dof4": False,               # NEW: có áp dụng bậc tự do 4 khi chạy CSV không
        "csv_dof4_angle": 90.0,                # NEW: góc quay áp dụng cho bậc tự do 4 khi chạy CSV
    }
    if os.path.isfile(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                saved = json.load(f)
            default_cfg.update(saved)
        except Exception as e:
            print(f"[CONFIG] Không đọc được config cũ, dùng mặc định: {e}")
    return default_cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        print(f"[CONFIG] Lỗi lưu config: {e}")
        return False


# =====================================================================
# LỚP GIAO TIẾP UART 2 - THIẾT BỊ PHỤ (bơm / bàn xoay / step bậc tự do 4)
# Giao thức lệnh dạng text, kết thúc bằng '\n':
#   PUMP:1 / PUMP:0      -> bật / tắt máy bơm
#   TURN:<0-255>         -> đặt tốc độ PWM bàn xoay (0 = dừng)
#   STEP:<độ>             -> quay step (bậc tự do 4) thêm N độ (âm = ngược chiều)
# =====================================================================
class PneumaticComm:
    """Điều khiển bơm / bàn xoay / step (bậc tự do 4) qua UART 2."""

    def __init__(self, port="COM3", baud=PNEU_DEFAULT_BAUD, log_callback=None, dry_run=False):
        self.port = port
        self.baud = baud
        self.log = log_callback or print
        self.dry_run = dry_run or (not SERIAL_AVAILABLE)
        self._serial = None
        self._lock = threading.Lock()
        self._connected = False

    def connect(self) -> bool:
        if self.dry_run:
            self._connected = True
            self.log(f"[DRY-RUN] Giả lập kết nối UART2 tới {self.port or 'VIRTUAL_PORT'}")
            return True
        if not self.port:
            self.log("[ERR] Chưa chọn cổng COM UART2!")
            return False
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            self._connected = True
            self.log(f"[OK] Đã kết nối UART2 {self.port} @ {self.baud} baud")
            return True
        except Exception as e:
            self.log(f"[ERR] Không thể mở {self.port}: {e}")
            self._connected = False
            return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._connected = False
        self.log("[INFO] Đã ngắt kết nối UART2")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def send_cmd(self, cmd: str) -> bool:
        if self.dry_run:
            self.log(f"[TX-DRY-UART2] {cmd}")
            return True
        if not self._connected or self._serial is None:
            self.log("[ERR] UART2 chưa kết nối!")
            return False
        try:
            with self._lock:
                self._serial.write((cmd + "\n").encode("utf-8"))
            self.log(f"[TX-UART2] {cmd}")
            return True
        except Exception as e:
            self.log(f"[ERR] Gửi UART2 thất bại: {e}")
            return False

    # ---- Lệnh tiện ích: MÁY BƠM ----
    def pump_on(self) -> bool:
        return self.send_cmd("PUMP:1")

    def pump_off(self) -> bool:
        return self.send_cmd("PUMP:0")

    # ---- Lệnh tiện ích: BÀN XOAY (PWM 0-255) ----
    def turn_set_speed(self, speed: int) -> bool:
        speed = max(0, min(255, int(speed)))
        return self.send_cmd(f"TURN:{speed}")

    def turn_off(self) -> bool:
        return self.turn_set_speed(0)

    # ---- Lệnh tiện ích: BẬC TỰ DO 4 (STEP - GÓC QUAY) ----
    def step_rotate(self, degree: float) -> bool:
        return self.send_cmd(f"STEP:{degree}")

    @staticmethod
    def list_ports():
        if not SERIAL_AVAILABLE:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]


# =====================================================================
# WORKER CHUNG - chạy tác vụ (có sleep/blocking) trên thread riêng
# =====================================================================
class FnWorker(QThread):
    done_ok = Signal(object)
    done_err = Signal(str)

    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):
        try:
            result = self.fn(*self.args, **self.kwargs)
            self.done_ok.emit(result)
        except Exception as e:
            self.done_err.emit(str(e))


class StreamToSignal:
    """File-like object: chuyển print() thành tín hiệu Qt để log lên UI."""

    def __init__(self, emit_fn):
        self.emit_fn = emit_fn

    def write(self, text):
        text = text.rstrip("\n")
        if text:
            self.emit_fn(text)

    def flush(self):
        pass


# =====================================================================
# THREAD CAMERA - chạy vòng lặp đọc/undistort/nhận diện liên tục
# =====================================================================
class CameraThread(QThread):
    frame_ready = Signal(np.ndarray, list, float)  # frame BGR đã vẽ, list circles(mm), fps
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index=0, calib_path=None):
        super().__init__()
        self.camera_index = camera_index
        self.calib_path = calib_path
        self._running = False
        self.camera_matrix = None
        self.dist_coeffs = None

    def load_calibration(self):
        if self.calib_path and os.path.isfile(self.calib_path):
            try:
                data = np.load(self.calib_path)
                self.camera_matrix = data["camera_matrix"]
                self.dist_coeffs = data["dist_coeffs"]
                return True
            except Exception as e:
                self.error.emit(f"Lỗi đọc file calib: {e}")
        return False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        has_calib = self.load_calibration()

        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW if os.name == "nt" else 0)
        if not cap.isOpened():
            self.error.emit(f"Không thể mở camera index {self.camera_index}")
            self.stopped.emit()
            return

        cap.set(cv2.CAP_PROP_FPS, 30)
        tracker = CircleTracker(alpha=0.35, max_disappeared=15, dist_threshold=30)

        fps = 0.0
        frame_count = 0
        start_time = cv2.getTickCount()

        while self._running:
            ret, frame = cap.read()
            if not ret:
                self.error.emit("Mất tín hiệu camera")
                break

            if has_calib:
                undistorted, new_cm = undistort_image(frame, self.camera_matrix, self.dist_coeffs)
                cx, cy = new_cm[0, 2], new_cm[1, 2]
            else:
                undistorted = frame
                h, w = frame.shape[:2]
                cx, cy = w / 2.0, h / 2.0

            detected = detect_circles_hsv_optimized(undistorted)
            stable_circles = tracker.update(detected)

            display = undistorted.copy()
            origin_px = (int(round(cx)), int(round(cy)))
            cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            circles_mm = []
            for item in stable_circles:
                center_f = item["center"]  # float sub-pixel — dùng để TÍNH tọa độ
                radius_f = item["radius"]
                c_type = item["type"]
                diameter_mm = calculate_real_properties(radius_f, c_type)

                # Hệ số mm/px đúng (không phụ thuộc kích thước từng vật), khác với
                # cách suy ra cũ diameter_mm/(2*radius) vốn gây lệch tọa độ theo size.
                scale = px_to_mm_scale(c_type)
                dx_px = -center_f[0] + cx
                dy_px = cy - center_f[1]
                x_mm = dx_px * scale
                y_mm = dy_px * scale

                # Chỉ làm tròn về int khi VẼ overlay, không ảnh hưởng giá trị đã tính ở trên
                center = (int(round(center_f[0])), int(round(center_f[1])))
                radius = int(round(radius_f))

                main_color = (0, 0, 255) if "red" in c_type else (255, 0, 0)
                cv2.circle(display, center, radius, main_color, 2)
                cv2.circle(display, center, 2, (0, 0, 255), 3)
                label = f'{c_type} {diameter_mm:.1f}mm'
                cv2.putText(display, label, (center[0] - 55, center[1] - radius - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, main_color, 2)

                circles_mm.append({
                    "type": c_type,
                    "diameter_mm": round(diameter_mm, 2),
                    "x_mm": round(x_mm, 2),
                    "y_mm": round(y_mm, 2),
                })

            frame_count += 1
            if frame_count >= 10:
                end_time = cv2.getTickCount()
                seconds = (end_time - start_time) / cv2.getTickFrequency()
                fps = frame_count / seconds if seconds > 0 else 0.0
                frame_count = 0
                start_time = cv2.getTickCount()

            self.frame_ready.emit(display, circles_mm, fps)

        cap.release()
        self.stopped.emit()


def home_and_wait(uart: UARTComm, timeout: float = 20.0) -> bool:
    """Gửi HOME và tự chờ STM32 phản hồi READY/OK/HOME_DONE.

    Bản UARTComm hiện tại (Uart_1.py) chỉ có send_home() gửi lệnh mà KHÔNG chờ
    phản hồi, nên phần chờ được cài đặt ở đây, đọc trực tiếp cổng serial bên
    trong đối tượng UARTComm.
    """
    if not uart.is_connected:
        uart.log("[ERR] Chưa kết nối Serial!")
        return False

    if not uart.send_home():
        return False

    if uart.dry_run or uart._serial is None:
        uart.log("[DRY-RUN] Giả lập chờ HOME xong -> OK")
        return True

    start_time = time.time()
    old_timeout = uart._serial.timeout
    try:
        uart._serial.timeout = 0.5
        while time.time() - start_time < timeout:
            with uart._lock:
                raw = uart._serial.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            upper = line.upper()
            if any(k in upper for k in ("READY", "OK", "HOME_DONE", "HOME OK")):
                uart.log(f"[RX] Nhận phản hồi: {line}")
                return True
            uart.log(f"[RX] Dòng nhận được: {line}")
    finally:
        uart._serial.timeout = old_timeout

    uart.log("[ERR] Timeout chờ phản hồi HOME")
    return False


def cv2_to_qpixmap(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
    return QPixmap.fromImage(qimg)


def apply_light_palette(app: QApplication):
    """Áp dụng bảng màu sáng cho toàn bộ ứng dụng."""
    palette = QPalette()
    bg = QColor("#f5f5f5")          # nền chính
    base = QColor("#ffffff")        # nền cho các ô nhập, bảng
    text = QColor("#1a1a1a")        # chữ chính
    disabled_text = QColor("#888888")
    highlight = QColor("#0078d4")   # màu xanh dương nhấn (giữ nguyên tông)

    palette.setColor(QPalette.Window, bg)
    palette.setColor(QPalette.WindowText, text)
    palette.setColor(QPalette.Base, base)
    palette.setColor(QPalette.AlternateBase, bg)
    palette.setColor(QPalette.ToolTipBase, base)
    palette.setColor(QPalette.ToolTipText, text)
    palette.setColor(QPalette.Text, text)
    palette.setColor(QPalette.Button, bg)
    palette.setColor(QPalette.ButtonText, text)
    palette.setColor(QPalette.BrightText, Qt.blue)
    palette.setColor(QPalette.Highlight, highlight)
    palette.setColor(QPalette.HighlightedText, Qt.white)
    palette.setColor(QPalette.Disabled, QPalette.Text, disabled_text)
    palette.setColor(QPalette.Disabled, QPalette.ButtonText, disabled_text)
    app.setPalette(palette)


# =====================================================================
# GIAO DIỆN CHÍNH
# =====================================================================
STYLE_SHEET = """
QWidget {
    background-color: #f5f5f5;
    color: #1a1a1a;
    font-family: 'Segoe UI', Arial;
    font-size: 14pt;
}
QMainWindow { background-color: #f5f5f5; }
QDialog { background-color: #f5f5f5; color: #1a1a1a; }
QMessageBox { background-color: #f5f5f5; color: #1a1a1a; }

QTabWidget::pane {
    border: 2px solid #c0c0c0;
    border-radius: 10px;
    top: -1px;
    background: #ffffff;
}
QTabBar::tab {
    background: #e0e0e0;
    color: #333333;
    padding: 16px 32px;
    font-size: 16pt;
    font-weight: 600;
    border-top-left-radius: 10px;
    border-top-right-radius: 10px;
    margin-right: 4px;
}
QTabBar::tab:selected {
    background: #0078d4;
    color: #ffffff;
}

QGroupBox {
    border: 2px solid #c0c0c0;
    border-radius: 12px;
    margin-top: 14px;
    font-weight: 700;
    font-size: 13pt;
    padding-top: 10px;
    color: #1a1a1a;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
}

QPushButton {
    background-color: #e0e0e0;
    color: #1a1a1a;
    border-radius: 14px;
    border: 2px solid #b0b0b0;
    padding: 10px;
    font-weight: 700;
    font-size: 15pt;
}
QPushButton:hover { background-color: #d0d0d0; border-color: #0078d4; }
QPushButton:pressed { background-color: #0078d4; color: #ffffff; }
QPushButton:disabled {
    background-color: #d0d0d0;
    color: #888888;
    border-color: #b0b0b0;
}

QPushButton#jogBtn {
    background-color: #e8e8e8;
    color: #1a1a1a;
    font-size: 20pt;
    min-height: 90px;
}
QPushButton#jogBtn:hover { background-color: #d0d0d0; }

QPushButton#homeBtn {
    background-color: #d0d0d0;
    border-color: #b0b0b0;
    font-size: 16pt;
    min-height: 70px;
}
QPushButton#homeBtn:hover { background-color: #c0c0c0; }

QPushButton#saveBtn {
    background-color: #0078d4;
    color: #ffffff;
    font-size: 15pt;
    min-height: 60px;
}
QPushButton#saveBtn:hover { background-color: #106ebe; }

QPushButton#connectBtn {
    background-color: #0078d4;
    color: #ffffff;
    min-height: 38px;
    font-size: 11pt;
    padding: 6px;
}
QPushButton#connectBtn:hover { background-color: #106ebe; }

QPushButton#disconnectBtn {
    background-color: #c00000;
    color: #ffffff;
    min-height: 38px;
    font-size: 11pt;
    padding: 6px;
}
QPushButton#disconnectBtn:hover { background-color: #a00000; }

QPushButton#actionBtn {
    background-color: #0078d4;
    color: #ffffff;
    min-height: 65px;
    font-size: 16pt;
}
QPushButton#actionBtn:hover { background-color: #106ebe; }

QPushButton#deviceOnBtn {
    background-color: #0078d4;
    color: #ffffff;
    min-height: 36px;
    font-size: 11pt;
    padding: 4px;
}
QPushButton#deviceOnBtn:hover { background-color: #106ebe; }

QPushButton#deviceOffBtn {
    background-color: #c00000;
    color: #ffffff;
    min-height: 36px;
    font-size: 11pt;
    padding: 4px;
}
QPushButton#deviceOffBtn:hover { background-color: #a00000; }

QPushButton#deviceApplyBtn {
    background-color: #b0b0b0;
    color: #1a1a1a;
    min-height: 32px;
    font-size: 10pt;
    padding: 4px;
}
QPushButton#deviceApplyBtn:hover { background-color: #a0a0a0; }

QPushButton#deviceQuickBtn {
    background-color: #e0e0e0;
    color: #1a1a1a;
    min-height: 30px;
    font-size: 10pt;
    padding: 2px;
    border-color: #b0b0b0;
}
QPushButton#deviceQuickBtn:hover { background-color: #d0d0d0; border-color: #0078d4; }

QLabel#posDisplay {
    background-color: #ffffff;
    border: 2px solid #0078d4;
    border-radius: 10px;
    font-size: 26pt;
    font-weight: 800;
    color: #0a6b3a;
    padding: 14px;
}

QLabel#statusOk {
    color: #0a7a3a;
    font-weight: 800;
    font-size: 14pt;
}
QLabel#statusBad {
    color: #c00000;
    font-weight: 800;
    font-size: 14pt;
}

QLabel#sectionTitle {
    font-size: 15pt;
    font-weight: 800;
    color: #1a1a1a;
}

QLabel#deviceStateOn {
    color: #0a7a3a;
    font-weight: 800;
    font-size: 11pt;
}
QLabel#deviceStateOff {
    color: #c00000;
    font-weight: 800;
    font-size: 11pt;
}

QGroupBox#compactBox {
    margin-top: 10px;
    padding-top: 8px;
    font-size: 11pt;
}
QGroupBox#compactBox QSpinBox,
QGroupBox#compactBox QDoubleSpinBox {
    padding: 4px 6px;
    font-size: 11pt;
    min-height: 26px;
}

QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit {
    background-color: #ffffff;
    border: 2px solid #c0c0c0;
    border-radius: 8px;
    padding: 8px;
    font-size: 13pt;
    color: #1a1a1a;
}
QComboBox QAbstractItemView {
    background-color: #ffffff;
    color: #1a1a1a;
    border: 1px solid #c0c0c0;
    selection-background-color: #0078d4;
    selection-color: #ffffff;
    outline: none;
}
QListView {
    background-color: #ffffff;
    color: #1a1a1a;
}

QCheckBox, QRadioButton {
    spacing: 8px;
}
QCheckBox::indicator, QRadioButton::indicator {
    width: 18px;
    height: 18px;
    border: 2px solid #b0b0b0;
    background-color: #ffffff;
}
QRadioButton::indicator { border-radius: 9px; }
QCheckBox::indicator:checked, QRadioButton::indicator:checked {
    background-color: #0078d4;
    border-color: #0078d4;
}

QTextEdit {
    background-color: #ffffff;
    border: 2px solid #c0c0c0;
    border-radius: 8px;
    color: #1a1a1a;
    font-family: Consolas, monospace;
    font-size: 11pt;
}

QTableWidget {
    background-color: #ffffff;
    border: 2px solid #c0c0c0;
    border-radius: 8px;
    gridline-color: #d0d0d0;
    font-size: 11pt;
}
QHeaderView::section {
    background-color: #e0e0e0;
    color: #1a1a1a;
    padding: 6px;
    border: none;
    font-weight: 700;
}

QScrollBar:vertical {
    background: #e0e0e0;
    width: 12px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #b0b0b0;
    border-radius: 6px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover { background: #0078d4; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QScrollBar:horizontal {
    background: #e0e0e0;
    height: 12px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: #b0b0b0;
    border-radius: 6px;
    min-width: 24px;
}
QScrollBar::handle:horizontal:hover { background: #0078d4; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
"""


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Điều khiển Robot Delta - Pick & Place")

        self.cfg = load_config()

        # ---- Giao tiếp phần cứng ----
        self.robot_uart = UARTComm(
            port=self.cfg["robot_port"], baud=self.cfg["robot_baud"],
            log_callback=self.log_conn, dry_run=not SERIAL_AVAILABLE,
        )
        self.pneu_uart = PneumaticComm(
            port=self.cfg["pneumatic_port"], baud=self.cfg["pneumatic_baud"],
            log_callback=self.log_conn, dry_run=not SERIAL_AVAILABLE,
        )
        self.planner = DeltaMotionPlanner(uart_comm=self.robot_uart)
        self.planner.HOME = tuple(self.cfg["home_position"])
        self.planner.Z_SAFE = self.cfg["z_safe"]

        # ---- Trạng thái vị trí ảo (jog) ----
        self.current_pos = list(self.cfg["home_position"])
        self.home_pos = list(self.cfg["home_position"])
        self.jog_step_xy = self.cfg["jog_step_xy"]
        self.jog_step_z = self.cfg["jog_step_z"]
        self._busy = False

        # ---- Offset camera ----
        self.offset_x = self.cfg.get("camera_offset_x", 0.0)
        self.offset_y = self.cfg.get("camera_offset_y", 0.0)

        self.camera_thread = None
        self.selected_pick = None
        self.selected_place = None
        self.latest_circles = []

        # ---- CSV / Tự động ----
        self.csv_offset_x = self.cfg.get("csv_offset_x", 0.0)
        self.csv_offset_y = self.cfg.get("csv_offset_y", 0.0)
        self.csv_points = []
        self._csv_stop_flag = False

        # ---- Trạng thái thiết bị phụ (UART2): bơm / bàn xoay / bậc tự do 4 ----
        self.pump_state = False
        self.turn_state = False
        self.turn_pwm_value = int(self.cfg.get("turn_pwm", 150))

        # ---- NEW: chế độ xác định tọa độ (tab CAMERA) ----
        self.coord_mode = self.cfg.get("coord_mode", COORD_MODE_CIRCLE)

        self._workers = []

        self._build_ui()
        self._update_pos_display()
        self._refresh_conn_status()

    # ------------------------------------------------------------------
    # HẠ TẦNG UI
    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)

        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)

        self.tabs.addTab(self._build_manual_tab(), "🕹️ THỦ CÔNG")
        self.tabs.addTab(self._build_camera_tab(), "📷 CAMERA && OFFSET TỌA ĐỘ")
        self.tabs.addTab(self._build_csv_tab(), "🗂️ BÀI TOÁN TĨNH")
        self.tabs.addTab(self._build_connection_tab(), "🔌 KẾT NỐI")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showMaximized()
            else:
                self.showFullScreen()
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.showMaximized()
        super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # TAB 1 - ĐIỀU KHIỂN THỦ CÔNG
    # ------------------------------------------------------------------
    def _build_manual_tab(self):
        tab = QWidget()
        root = QHBoxLayout(tab)

        xy_box = QGroupBox("DI CHUYỂN NGANG (X / Y)")
        xy_layout = QGridLayout()
        xy_layout.setSpacing(10)

        def mk_jog_btn(text):
            b = QPushButton(text)
            b.setObjectName("jogBtn")
            b.setMinimumSize(140, 110)
            return b

        self.btn_y_plus = mk_jog_btn("▲\nX+")
        self.btn_y_minus = mk_jog_btn("▼\nX−")
        self.btn_x_minus = mk_jog_btn("◀\nY−")
        self.btn_x_plus = mk_jog_btn("▶\nY+")
        self.btn_center_info = QLabel("XY")
        self.btn_center_info.setAlignment(Qt.AlignCenter)
        self.btn_center_info.setStyleSheet("font-size:22pt; font-weight:800; color:#333333;")

        xy_layout.addWidget(self.btn_y_plus, 0, 1)
        xy_layout.addWidget(self.btn_x_minus, 1, 0)
        xy_layout.addWidget(self.btn_center_info, 1, 1)
        xy_layout.addWidget(self.btn_x_plus, 1, 2)
        xy_layout.addWidget(self.btn_y_minus, 2, 1)
        xy_box.setLayout(xy_layout)

        self.btn_y_plus.clicked.connect(lambda: self.jog(dx=self.jog_step_xy))
        self.btn_y_minus.clicked.connect(lambda: self.jog(dx=-self.jog_step_xy))
        self.btn_x_plus.clicked.connect(lambda: self.jog(dy=self.jog_step_xy))
        self.btn_x_minus.clicked.connect(lambda: self.jog(dy=-self.jog_step_xy))

        z_box = QGroupBox("ĐỘ CAO (Z)")
        z_layout = QVBoxLayout()
        self.btn_z_plus = mk_jog_btn("⤒\nZ LÊN")
        self.btn_z_minus = mk_jog_btn("⤓\nZ XUỐNG")
        self.btn_z_plus.setMinimumSize(140, 150)
        self.btn_z_minus.setMinimumSize(140, 150)
        z_layout.addWidget(self.btn_z_plus)
        z_layout.addWidget(self.btn_z_minus)
        z_box.setLayout(z_layout)

        self.btn_z_plus.clicked.connect(lambda: self.jog(dz=-self.jog_step_z))
        self.btn_z_minus.clicked.connect(lambda: self.jog(dz=self.jog_step_z))

        left_col = QVBoxLayout()
        left_col.addWidget(xy_box)

        mid_col = QVBoxLayout()
        mid_col.addWidget(z_box)

        step_box = QGroupBox("BƯỚC NHẢY (mm/lần)")
        step_layout = QGridLayout()
        step_layout.addWidget(QLabel("XY:"), 0, 0)
        self.spin_step_xy = QDoubleSpinBox()
        self.spin_step_xy.setRange(0.1, 50.0)
        self.spin_step_xy.setValue(self.jog_step_xy)
        self.spin_step_xy.setSingleStep(0.5)
        self.spin_step_xy.valueChanged.connect(self._on_step_xy_changed)
        step_layout.addWidget(self.spin_step_xy, 0, 1)

        step_layout.addWidget(QLabel("Z:"), 1, 0)
        self.spin_step_z = QDoubleSpinBox()
        self.spin_step_z.setRange(0.1, 50.0)
        self.spin_step_z.setValue(self.jog_step_z)
        self.spin_step_z.setSingleStep(0.5)
        self.spin_step_z.valueChanged.connect(self._on_step_z_changed)
        step_layout.addWidget(self.spin_step_z, 1, 1)
        step_box.setLayout(step_layout)
        mid_col.addWidget(step_box)

        right_col = QVBoxLayout()

        pos_box = QGroupBox("TỌA ĐỘ HIỆN TẠI (mm)")
        pos_layout = QVBoxLayout()
        self.lbl_pos = QLabel("X:0.00  Y:0.00  Z:0.00")
        self.lbl_pos.setObjectName("posDisplay")
        self.lbl_pos.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(self.lbl_pos)
        self.lbl_angles = QLabel("θ1:--°  θ2:--°  θ3:--°")
        self.lbl_angles.setAlignment(Qt.AlignCenter)
        self.lbl_angles.setStyleSheet("color:#333333; font-size:12pt;")
        pos_layout.addWidget(self.lbl_angles)
        pos_box.setLayout(pos_layout)
        right_col.addWidget(pos_box)

        home_box = QGroupBox("HOME")
        home_layout = QVBoxLayout()

        self.btn_home_wait = QPushButton("⌂  VỀ HOME (chờ READY từ STM32)")
        self.btn_home_wait.setObjectName("homeBtn")
        self.btn_home_wait.clicked.connect(self.on_home_wait)
        home_layout.addWidget(self.btn_home_wait)

        self.btn_goto_saved_home = QPushButton("➤  DI CHUYỂN ĐẾN HOME ĐÃ LƯU")
        self.btn_goto_saved_home.setObjectName("homeBtn")
        self.btn_goto_saved_home.clicked.connect(self.on_goto_saved_home)
        home_layout.addWidget(self.btn_goto_saved_home)

        self.btn_save_home = QPushButton("💾  LƯU VỊ TRÍ HIỆN TẠI LÀM HOME AN TOÀN (SAVE)")
        self.btn_save_home.setObjectName("saveBtn")
        self.btn_save_home.clicked.connect(self.on_save_home)
        home_layout.addWidget(self.btn_save_home)

        self.lbl_saved_home = QLabel(self._home_str())
        self.lbl_saved_home.setStyleSheet("color:#0a6b3a; font-size:12pt;")
        home_layout.addWidget(self.lbl_saved_home)

        home_box.setLayout(home_layout)
        right_col.addWidget(home_box)

        self.lbl_manual_status = QLabel("Sẵn sàng.")
        self.lbl_manual_status.setWordWrap(True)
        self.lbl_manual_status.setStyleSheet("color:#333333; font-size:11pt;")
        right_col.addWidget(self.lbl_manual_status)
        right_col.addStretch()

        root.addLayout(left_col, 3)
        root.addLayout(mid_col, 2)
        root.addLayout(right_col, 3)
        return tab

    def _home_str(self):
        x, y, z = self.home_pos
        return f"Home đã lưu: X={x:.2f}  Y={y:.2f}  Z={z:.2f}"

    def _on_step_xy_changed(self, v):
        self.jog_step_xy = v

    def _on_step_z_changed(self, v):
        self.jog_step_z = v

    def _update_pos_display(self):
        x, y, z = self.current_pos
        self.lbl_pos.setText(f"X:{x:.2f}  Y:{y:.2f}  Z:{z:.2f}")
        try:
            t1, t2, t3 = inverse_kinematics(x, y, z)
            self.lbl_angles.setText(f"θ1:{t1:.2f}°  θ2:{t2:.2f}°  θ3:{t3:.2f}°")
        except Exception:
            self.lbl_angles.setText("θ1:--°  θ2:--°  θ3:--°  (ngoài vùng làm việc)")

    def _set_manual_busy(self, busy, msg=""):
        self._busy = busy
        for b in (self.btn_x_plus, self.btn_x_minus, self.btn_y_plus, self.btn_y_minus,
                  self.btn_z_plus, self.btn_z_minus, self.btn_home_wait,
                  self.btn_goto_saved_home, self.btn_save_home):
            b.setEnabled(not busy)
        if msg:
            self.lbl_manual_status.setText(msg)

    def jog(self, dx=0.0, dy=0.0, dz=0.0):
        if self._busy:
            return
        new_pos = [self.current_pos[0] + dx, self.current_pos[1] + dy, self.current_pos[2] + dz]

        try:
            inverse_kinematics(*new_pos)
        except Exception as e:
            self.lbl_manual_status.setText(f"⚠ Ngoài vùng làm việc: {e}")
            return

        self._set_manual_busy(True, "Đang di chuyển...")
        worker = FnWorker(self.planner.send_position, *new_pos)
        worker.done_ok.connect(lambda ok, p=new_pos: self._on_jog_done(ok, p))
        worker.done_err.connect(lambda err: self._on_jog_error(err))
        self._workers.append(worker)
        worker.start()

    def _on_jog_done(self, ok, new_pos):
        self._set_manual_busy(False)
        if ok:
            self.current_pos = new_pos
            self._update_pos_display()
            self.lbl_manual_status.setText("Sẵn sàng.")
        else:
            self.lbl_manual_status.setText("⚠ Gửi lệnh thất bại (kiểm tra kết nối UART Robot).")

    def _on_jog_error(self, err):
        self._set_manual_busy(False)
        self.lbl_manual_status.setText(f"⚠ Lỗi: {err}")

    def on_home_wait(self):
        if self._busy:
            return
        if not self.robot_uart.is_connected:
            self.lbl_manual_status.setText(
                "⚠ Chưa kết nối Robot (UART 1). Hãy kết nối ở tab KẾT NỐI trước."
            )
            return

        self._set_manual_busy(
            True, "Đang HOME (chạm 3 công tắc hành trình)... chờ STM32 phản hồi READY"
        )

        worker = FnWorker(home_and_wait, self.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_home_wait_done)
        worker.done_err.connect(self._on_jog_error)
        self._workers.append(worker)
        worker.start()

    def _on_home_wait_done(self, ready):
        self._set_manual_busy(False)
        if ready:
            self.lbl_manual_status.setText(
                "✔ Đã HOME xong (chạm công tắc hành trình, encoder = 0). "
                "Bấm 'DI CHUYỂN ĐẾN HOME ĐÃ LƯU' để về vị trí an toàn."
            )
        else:
            self.lbl_manual_status.setText(
                "⚠ Không nhận được READY từ STM32 (kiểm tra dây UART1 / công tắc hành trình)."
            )

    def on_goto_saved_home(self):
        if self._busy:
            return
        self._set_manual_busy(True, "Đang di chuyển tới Home đã lưu...")
        hx, hy, hz = self.home_pos
        worker = FnWorker(self.planner.send_position, hx, hy, hz)
        worker.done_ok.connect(lambda ok: self._on_jog_done(ok, list(self.home_pos)))
        worker.done_err.connect(self._on_jog_error)
        self._workers.append(worker)
        worker.start()

    def on_save_home(self):
        self.home_pos = list(self.current_pos)
        self.planner.HOME = tuple(self.home_pos)
        self.cfg["home_position"] = self.home_pos
        save_config(self.cfg)
        self.lbl_saved_home.setText(self._home_str())
        self.lbl_manual_status.setText("💾 Đã lưu vị trí hiện tại làm Home an toàn.")

    # ------------------------------------------------------------------
    # TAB 2 - CAMERA & GẮP THẢ TỰ ĐỘNG
    # ------------------------------------------------------------------
    def _build_camera_tab(self):
        tab = QWidget()
        root = QHBoxLayout(tab)

        video_col = QVBoxLayout()
        self.video_label = QLabel("Camera chưa kết nối")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(720, 480)
        self.video_label.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; border-radius:10px; color:#888888;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 5)

        cam_ctrl_row = QHBoxLayout()
        self.btn_cam_start = QPushButton("▶  BẮT ĐẦU CAMERA")
        self.btn_cam_start.setObjectName("connectBtn")
        self.btn_cam_start.clicked.connect(self.on_start_camera)
        self.btn_cam_stop = QPushButton("■  DỪNG CAMERA")
        self.btn_cam_stop.setObjectName("disconnectBtn")
        self.btn_cam_stop.clicked.connect(self.on_stop_camera)
        self.btn_cam_stop.setEnabled(False)
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#333333; font-weight:700;")
        cam_ctrl_row.addWidget(self.btn_cam_start)
        cam_ctrl_row.addWidget(self.btn_cam_stop)
        cam_ctrl_row.addWidget(self.lbl_fps)
        video_col.addLayout(cam_ctrl_row)

        # NEW: chọn phương thức xác định tọa độ
        mode_box = QGroupBox("PHƯƠNG THỨC XÁC ĐỊNH TỌA ĐỘ")
        mode_box.setObjectName("compactBox")
        mode_layout = QVBoxLayout()
        mode_layout.setSpacing(4)

        self.radio_mode_circle = QRadioButton("① File tọa độ vòng tròn (Cameracircle.py - hiện tại)")
        self.radio_mode_dof4 = QRadioButton("② File bậc tự do 4 (tọa độ + góc quay - sẽ bổ sung sau)")
        self.radio_group_mode = QButtonGroup(self)
        self.radio_group_mode.addButton(self.radio_mode_circle, 0)
        self.radio_group_mode.addButton(self.radio_mode_dof4, 1)

        if self.coord_mode == COORD_MODE_DOF4:
            self.radio_mode_dof4.setChecked(True)
        else:
            self.radio_mode_circle.setChecked(True)

        self.radio_mode_circle.toggled.connect(self._on_coord_mode_changed)

        mode_layout.addWidget(self.radio_mode_circle)
        mode_layout.addWidget(self.radio_mode_dof4)
        self.lbl_coord_mode_note = QLabel(
            "Đang dùng: Cameracircle.py (chỉ xác định X/Y)."
            if self.coord_mode == COORD_MODE_CIRCLE else
            "Đang dùng: file bậc tự do 4 (chưa gán module - sẽ cập nhật sau)."
        )
        self.lbl_coord_mode_note.setWordWrap(True)
        self.lbl_coord_mode_note.setStyleSheet("color:#555555; font-size:9pt;")
        mode_layout.addWidget(self.lbl_coord_mode_note)

        mode_box.setLayout(mode_layout)
        video_col.addWidget(mode_box)

        self.circle_table = QTableWidget(0, 4)
        self.circle_table.setHorizontalHeaderLabels(["Loại", "Đường kính (mm)", "X (mm)", "Y (mm)"])
        self.circle_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.circle_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.circle_table.setMaximumHeight(180)
        video_col.addWidget(self.circle_table, 2)

        ctrl_col = QVBoxLayout()

        pick_box = QGroupBox("ĐIỂM GẮP (A)")
        pick_layout = QGridLayout()
        self.spin_a_x = QDoubleSpinBox(); self.spin_a_x.setRange(-500, 500); self.spin_a_x.setSuffix(" mm")
        self.spin_a_y = QDoubleSpinBox(); self.spin_a_y.setRange(-500, 500); self.spin_a_y.setSuffix(" mm")
        pick_layout.addWidget(QLabel("X:"), 0, 0); pick_layout.addWidget(self.spin_a_x, 0, 1)
        pick_layout.addWidget(QLabel("Y:"), 1, 0); pick_layout.addWidget(self.spin_a_y, 1, 1)
        self.btn_use_selected_a = QPushButton("↙ Lấy từ vòng tròn đã chọn")
        self.btn_use_selected_a.clicked.connect(lambda: self._copy_selected_to(self.spin_a_x, self.spin_a_y))
        pick_layout.addWidget(self.btn_use_selected_a, 2, 0, 1, 2)
        pick_box.setLayout(pick_layout)
        ctrl_col.addWidget(pick_box)

        place_box = QGroupBox("ĐIỂM ĐẶT (B)")
        place_layout = QGridLayout()
        self.spin_b_x = QDoubleSpinBox(); self.spin_b_x.setRange(-500, 500); self.spin_b_x.setSuffix(" mm")
        self.spin_b_y = QDoubleSpinBox(); self.spin_b_y.setRange(-500, 500); self.spin_b_y.setSuffix(" mm")
        place_layout.addWidget(QLabel("X:"), 0, 0); place_layout.addWidget(self.spin_b_x, 0, 1)
        place_layout.addWidget(QLabel("Y:"), 1, 0); place_layout.addWidget(self.spin_b_y, 1, 1)
        self.btn_use_selected_b = QPushButton("↙ Lấy từ vòng tròn đã chọn")
        self.btn_use_selected_b.clicked.connect(lambda: self._copy_selected_to(self.spin_b_x, self.spin_b_y))
        place_layout.addWidget(self.btn_use_selected_b, 2, 0, 1, 2)
        place_box.setLayout(place_layout)
        ctrl_col.addWidget(place_box)

        z_box = QGroupBox("ĐỘ CAO GẮP")
        z_layout = QHBoxLayout()
        z_layout.addWidget(QLabel("Z pick:"))
        self.spin_z_pick = QDoubleSpinBox()
        self.spin_z_pick.setRange(0, 500)
        self.spin_z_pick.setValue(320)
        z_layout.addWidget(self.spin_z_pick)
        z_box.setLayout(z_layout)
        ctrl_col.addWidget(z_box)

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
        self.spin_offset_x = QDoubleSpinBox()
        self.spin_offset_x.setRange(-100, 100)
        self.spin_offset_x.setValue(self.offset_x)
        self.spin_offset_x.setSingleStep(0.5)
        self.spin_offset_x.valueChanged.connect(self._on_offset_x_changed)
        offset_layout.addWidget(self.spin_offset_x)

        offset_layout.addWidget(QLabel("Y:"))
        self.spin_offset_y = QDoubleSpinBox()
        self.spin_offset_y.setRange(-100, 100)
        self.spin_offset_y.setValue(self.offset_y)
        self.spin_offset_y.setSingleStep(0.5)
        self.spin_offset_y.valueChanged.connect(self._on_offset_y_changed)
        offset_layout.addWidget(self.spin_offset_y)

        offset_box.setLayout(offset_layout)
        ctrl_col.addWidget(offset_box)

        self.btn_run_pick_place = QPushButton("🤖  BẮT ĐẦU GẮP THẢ TỰ ĐỘNG")
        self.btn_run_pick_place.setObjectName("actionBtn")
        self.btn_run_pick_place.clicked.connect(self.on_run_pick_place)
        ctrl_col.addWidget(self.btn_run_pick_place)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        ctrl_col.addWidget(self.log_box, 3)

        root.addLayout(video_col, 3)
        root.addLayout(ctrl_col, 2)
        return tab

    # NEW: slot đổi chế độ xác định tọa độ
    def _on_coord_mode_changed(self, circle_checked):
        self.coord_mode = COORD_MODE_CIRCLE if circle_checked else COORD_MODE_DOF4
        self.cfg["coord_mode"] = self.coord_mode
        save_config(self.cfg)
        if self.coord_mode == COORD_MODE_CIRCLE:
            self.lbl_coord_mode_note.setText("Đang dùng: Cameracircle.py (chỉ xác định X/Y).")
        else:
            self.lbl_coord_mode_note.setText(
                "Đang dùng: file bậc tự do 4 (chưa gán module - sẽ cập nhật sau khi bạn hoàn thiện file)."
            )

    def _copy_selected_to(self, spin_x, spin_y):
        row = self.circle_table.currentRow()
        if row < 0:
            QMessageBox.warning(self, "Chưa chọn", "Hãy chọn 1 dòng vòng tròn trong bảng trước.")
            return
        x = float(self.circle_table.item(row, 2).text())
        y = float(self.circle_table.item(row, 3).text())
        x += self.offset_x
        y += self.offset_y
        spin_x.setValue(x)
        spin_y.setValue(y)

    def _on_offset_x_changed(self, val):
        self.offset_x = val
        self.cfg["camera_offset_x"] = val
        save_config(self.cfg)

    def _on_offset_y_changed(self, val):
        self.offset_y = val
        self.cfg["camera_offset_y"] = val
        save_config(self.cfg)

    def on_start_camera(self):
        if self.camera_thread is not None:
            return
        # TODO: khi mode == COORD_MODE_DOF4 và bạn đã có module xác định góc quay riêng,
        # đổi CameraThread để dùng module đó thay cho Cameracircle.py ở đây.
        self.camera_thread = CameraThread(
            camera_index=self.cfg.get("camera_index", 0),
            calib_path=self.cfg.get("calib_file"),
        )
        self.camera_thread.frame_ready.connect(self._on_frame)
        self.camera_thread.error.connect(self._on_camera_error)
        self.camera_thread.stopped.connect(self._on_camera_stopped)
        self.camera_thread.start()
        self.btn_cam_start.setEnabled(False)
        self.btn_cam_stop.setEnabled(True)

    def on_stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()

    def _on_camera_stopped(self):
        self.camera_thread = None
        self.btn_cam_start.setEnabled(True)
        self.btn_cam_stop.setEnabled(False)
        self.video_label.setText("Camera đã dừng")

    def _on_camera_error(self, msg):
        QMessageBox.warning(self, "Lỗi Camera", msg)

    def _frame_counter(self):
        if not hasattr(self, "_fc"):
            self._fc = 0
        self._fc += 1
        return self._fc

    def _on_frame(self, frame, circles, fps):
        self.latest_circles = circles
        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.lbl_fps.setText(f"FPS: {fps:.1f}")

        if self._frame_counter() % 3 == 0:
            self.circle_table.setRowCount(len(circles))
            for i, c in enumerate(circles):
                self.circle_table.setItem(i, 0, QTableWidgetItem(c["type"]))
                self.circle_table.setItem(i, 1, QTableWidgetItem(f'{c["diameter_mm"]:.2f}'))
                self.circle_table.setItem(i, 2, QTableWidgetItem(f'{c["x_mm"]:.2f}'))
                self.circle_table.setItem(i, 3, QTableWidgetItem(f'{c["y_mm"]:.2f}'))

    def _gripper_callback(self, state):
        # Khớp với PneumaticComm (UART2) hiện có trong file này: dùng đúng
        # giao thức PUMP:1 / PUMP:0 thay vì "on1"/"off1" cũ.
        if state == "on":
            self.pneu_uart.pump_on()
        else:
            self.pneu_uart.pump_off()

    def on_run_pick_place(self):
        if self._busy:
            QMessageBox.warning(self, "Đang bận", "Robot đang thực hiện lệnh khác.")
            return
        point_a = (self.spin_a_x.value(), self.spin_a_y.value())
        point_b = (self.spin_b_x.value(), self.spin_b_y.value())
        z_pick = self.spin_z_pick.value()

        self._set_manual_busy(True, "")
        self.btn_run_pick_place.setEnabled(False)
        self.log_box.append(f"=== Bắt đầu gắp {point_a} -> đặt {point_b}, Z_pick={z_pick} ===")

        def task():
            with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                self.planner.pick_and_place(point_a, point_b, z_pick=z_pick,
                                             gripper_callback=self._gripper_callback)
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_pick_place_done)
        worker.done_err.connect(self._on_pick_place_error)
        self._workers.append(worker)
        worker.start()

    def _append_log_threadsafe(self, text):
        QTimer.singleShot(0, lambda: self.log_box.append(text))

    def _on_pick_place_done(self, ok):
        self._set_manual_busy(False)
        self.btn_run_pick_place.setEnabled(True)
        self.log_box.append("=== HOÀN TẤT ===")
        self.current_pos = list(self.planner.HOME)
        self._update_pos_display()

    def _on_pick_place_error(self, err):
        self._set_manual_busy(False)
        self.btn_run_pick_place.setEnabled(True)
        self.log_box.append(f"⚠ LỖI: {err}")

    # ------------------------------------------------------------------
    # TAB 3 - CSV & TỰ ĐỘNG
    # ------------------------------------------------------------------
    def _build_csv_tab(self):
        tab = QWidget()
        root = QHBoxLayout(tab)

        left_col = QVBoxLayout()

        title = QLabel("DANH SÁCH ĐIỂM GẮP (CSV)")
        title.setObjectName("sectionTitle")
        left_col.addWidget(title)

        self.csv_table = QTableWidget(0, 4)
        self.csv_table.setHorizontalHeaderLabels(["Loại", "Đường kính (mm)", "X (mm)", "Y (mm)"])
        self.csv_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.csv_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        left_col.addWidget(self.csv_table, 5)

        csv_btn_row = QHBoxLayout()
        self.btn_csv_capture = QPushButton("📥 LẤY DANH SÁCH TỪ CAMERA")
        self.btn_csv_capture.clicked.connect(self.on_csv_capture)
        self.btn_csv_save = QPushButton("💾 LƯU RA FILE CSV")
        self.btn_csv_save.setObjectName("saveBtn")
        self.btn_csv_save.clicked.connect(self.on_csv_save)
        self.btn_csv_load = QPushButton("📂 NẠP FILE CSV")
        self.btn_csv_load.clicked.connect(self.on_csv_load)
        csv_btn_row.addWidget(self.btn_csv_capture)
        csv_btn_row.addWidget(self.btn_csv_save)
        csv_btn_row.addWidget(self.btn_csv_load)
        left_col.addLayout(csv_btn_row)

        self.lbl_csv_count = QLabel("Chưa có điểm nào.")
        self.lbl_csv_count.setStyleSheet("color:#333333; font-size:11pt;")
        left_col.addWidget(self.lbl_csv_count)

        right_col = QVBoxLayout()

        offset_box = QGroupBox("OFFSET CAMERA (mm) — riêng cho CSV")
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
        self.spin_csv_offset_x.setValue(self.csv_offset_x)
        self.spin_csv_offset_x.setSingleStep(0.5)
        self.spin_csv_offset_x.valueChanged.connect(self._on_csv_offset_x_changed)
        offset_layout.addWidget(self.spin_csv_offset_x)
        offset_layout.addWidget(QLabel("Y:"))
        self.spin_csv_offset_y = QDoubleSpinBox()
        self.spin_csv_offset_y.setRange(-100, 100)
        self.spin_csv_offset_y.setValue(self.csv_offset_y)
        self.spin_csv_offset_y.setSingleStep(0.5)
        self.spin_csv_offset_y.valueChanged.connect(self._on_csv_offset_y_changed)
        offset_layout.addWidget(self.spin_csv_offset_y)
        offset_box.setLayout(offset_layout)
        right_col.addWidget(offset_box)

        place_box = QGroupBox("GHÉP CẶP VẬT (WHITE) → LỖ (BLACK)")
        place_layout = QGridLayout()
        place_layout.addWidget(QLabel("Dung sai đường kính (mm):"), 0, 0)
        self.spin_csv_tolerance = QDoubleSpinBox()
        self.spin_csv_tolerance.setRange(0.0, 50.0)
        self.spin_csv_tolerance.setSingleStep(0.5)
        self.spin_csv_tolerance.setValue(self.cfg.get("csv_match_tolerance", 6.0))
        place_layout.addWidget(self.spin_csv_tolerance, 0, 1)

        self.chk_csv_same_color = QCheckBox("Chỉ ghép cùng màu (đỏ↔đỏ, xanh↔xanh)")
        self.chk_csv_same_color.setChecked(bool(self.cfg.get("csv_match_same_color", True)))
        place_layout.addWidget(self.chk_csv_same_color, 1, 0, 1, 2)

        place_layout.addWidget(QLabel("Z pick:"), 2, 0)
        self.spin_csv_z_pick = QDoubleSpinBox()
        self.spin_csv_z_pick.setRange(0, 500)
        self.spin_csv_z_pick.setValue(self.cfg.get("csv_z_pick", 320.0))
        place_layout.addWidget(self.spin_csv_z_pick, 2, 1)
        place_box.setLayout(place_layout)
        right_col.addWidget(place_box)

        # NEW: khối "Áp dụng bậc tự do thứ 4" cho tab BÀI TOÁN TĨNH
        dof4_box = QGroupBox("BẬC TỰ DO 4 (BÀN XOAY / STEP)")
        dof4_box.setObjectName("compactBox")
        dof4_layout = QHBoxLayout()
        dof4_layout.setSpacing(8)

        self.chk_csv_apply_dof4 = QCheckBox("Áp dụng bậc tự do thứ 4")
        self.chk_csv_apply_dof4.setChecked(bool(self.cfg.get("csv_apply_dof4", False)))
        self.chk_csv_apply_dof4.toggled.connect(self._on_csv_apply_dof4_changed)
        dof4_layout.addWidget(self.chk_csv_apply_dof4)

        dof4_layout.addWidget(QLabel("Góc (độ):"))
        self.spin_csv_dof4_angle = QDoubleSpinBox()
        self.spin_csv_dof4_angle.setRange(-3600.0, 3600.0)
        self.spin_csv_dof4_angle.setSingleStep(5.0)
        self.spin_csv_dof4_angle.setValue(float(self.cfg.get("csv_dof4_angle", 90.0)))
        self.spin_csv_dof4_angle.valueChanged.connect(self._on_csv_dof4_angle_changed)
        dof4_layout.addWidget(self.spin_csv_dof4_angle, 1)

        dof4_box.setLayout(dof4_layout)
        right_col.addWidget(dof4_box)

        self.btn_csv_preview = QPushButton("🔍 XEM TRƯỚC GHÉP CẶP")
        self.btn_csv_preview.clicked.connect(self.on_csv_preview)
        right_col.addWidget(self.btn_csv_preview)

        run_row = QHBoxLayout()
        self.btn_csv_run = QPushButton("🤖 CHẠY TỰ ĐỘNG TỪ DANH SÁCH")
        self.btn_csv_run.setObjectName("actionBtn")
        self.btn_csv_run.clicked.connect(self.on_csv_run)
        self.btn_csv_stop = QPushButton("⏹ DỪNG")
        self.btn_csv_stop.setObjectName("disconnectBtn")
        self.btn_csv_stop.clicked.connect(self.on_csv_stop)
        self.btn_csv_stop.setEnabled(False)
        run_row.addWidget(self.btn_csv_run, 3)
        run_row.addWidget(self.btn_csv_stop, 1)
        right_col.addLayout(run_row)

        self.lbl_csv_progress = QLabel("Sẵn sàng.")
        self.lbl_csv_progress.setWordWrap(True)
        self.lbl_csv_progress.setStyleSheet("color:#333333; font-weight:700; font-size:12pt;")
        right_col.addWidget(self.lbl_csv_progress)

        self.log_box_csv = QTextEdit()
        self.log_box_csv.setReadOnly(True)
        right_col.addWidget(self.log_box_csv, 4)

        root.addLayout(left_col, 3)
        root.addLayout(right_col, 2)
        return tab

    # NEW: slot cho khối bậc tự do 4 ở tab CSV
    def _on_csv_apply_dof4_changed(self, checked):
        self.cfg["csv_apply_dof4"] = checked
        save_config(self.cfg)

    def _on_csv_dof4_angle_changed(self, val):
        self.cfg["csv_dof4_angle"] = val
        save_config(self.cfg)

    def _refresh_csv_table(self):
        self.csv_table.setRowCount(len(self.csv_points))
        for i, c in enumerate(self.csv_points):
            self.csv_table.setItem(i, 0, QTableWidgetItem(str(c.get("type", ""))))
            self.csv_table.setItem(i, 1, QTableWidgetItem(f'{float(c.get("diameter_mm", 0)):.2f}'))
            self.csv_table.setItem(i, 2, QTableWidgetItem(f'{float(c.get("x_mm", 0)):.2f}'))
            self.csv_table.setItem(i, 3, QTableWidgetItem(f'{float(c.get("y_mm", 0)):.2f}'))
        self.lbl_csv_count.setText(f"Đang có {len(self.csv_points)} điểm.")

    def on_csv_capture(self):
        if not self.latest_circles:
            QMessageBox.warning(
                self, "Không có dữ liệu",
                "Camera chưa phát hiện vòng tròn nào (hãy bật camera ở tab CAMERA trước)."
            )
            return
        self.csv_points = list(self.latest_circles)
        self._refresh_csv_table()
        self.lbl_csv_progress.setText(f"📥 Đã lấy {len(self.csv_points)} điểm từ camera.")

    def on_csv_save(self):
        if not self.csv_points:
            QMessageBox.warning(self, "Không có dữ liệu", "Danh sách điểm đang trống, không có gì để lưu.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Lưu danh sách điểm", "points.csv", "CSV files (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["type", "diameter_mm", "x_mm", "y_mm"])
                writer.writeheader()
                for c in self.csv_points:
                    writer.writerow({
                        "type": c.get("type", ""),
                        "diameter_mm": c.get("diameter_mm", 0),
                        "x_mm": c.get("x_mm", 0),
                        "y_mm": c.get("y_mm", 0),
                    })
            self.lbl_csv_progress.setText(f"💾 Đã lưu {len(self.csv_points)} điểm vào {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi lưu CSV", str(e))

    def on_csv_load(self):
        path, _ = QFileDialog.getOpenFileName(self, "Nạp danh sách điểm", "", "CSV files (*.csv)")
        if not path:
            return
        try:
            points = []
            with open(path, "r", newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    points.append({
                        "type": row.get("type", ""),
                        "diameter_mm": float(row.get("diameter_mm") or 0),
                        "x_mm": float(row.get("x_mm") or 0),
                        "y_mm": float(row.get("y_mm") or 0),
                    })
            self.csv_points = points
            self._refresh_csv_table()
            self.lbl_csv_progress.setText(f"📂 Đã nạp {len(points)} điểm từ {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi nạp CSV", str(e))

    def _match_white_black_pairs(self, points, same_color_only=True, tolerance_mm=6.0):
        whites = [p for p in points if "white" in str(p.get("type", ""))]
        blacks = [p for p in points if "black" in str(p.get("type", ""))]
        used_black_idx = set()
        pairs = []
        unmatched = []

        for w in whites:
            w_color = str(w.get("type", "")).split("_")[0]
            w_dia = float(w.get("diameter_mm", 0))
            best_idx, best_diff = None, None
            for i, b in enumerate(blacks):
                if i in used_black_idx:
                    continue
                b_color = str(b.get("type", "")).split("_")[0]
                if same_color_only and b_color != w_color:
                    continue
                diff = abs(float(b.get("diameter_mm", 0)) - w_dia)
                if diff <= tolerance_mm and (best_diff is None or diff < best_diff):
                    best_diff, best_idx = diff, i
            if best_idx is not None:
                used_black_idx.add(best_idx)
                pairs.append((w, blacks[best_idx]))
            else:
                unmatched.append(w)

        return pairs, unmatched

    def _log_pairs(self, pairs, unmatched, log_widget):
        log_widget.append(
            f"=== Ghép được {len(pairs)} cặp Vật→Lỗ (bỏ qua {len(unmatched)} vật không có lỗ phù hợp) ==="
        )
        for idx, (w, b) in enumerate(pairs, start=1):
            log_widget.append(
                f"  {idx}. {w.get('type')} Ø{float(w.get('diameter_mm', 0)):.1f}mm "
                f"({float(w.get('x_mm', 0)):.1f},{float(w.get('y_mm', 0)):.1f}) → "
                f"{b.get('type')} Ø{float(b.get('diameter_mm', 0)):.1f}mm "
                f"({float(b.get('x_mm', 0)):.1f},{float(b.get('y_mm', 0)):.1f})"
            )
        for w in unmatched:
            log_widget.append(
                f"  ⚠ Bỏ qua: {w.get('type')} Ø{float(w.get('diameter_mm', 0)):.1f}mm "
                f"không tìm thấy lỗ phù hợp."
            )

    def on_csv_preview(self):
        if not self.csv_points:
            QMessageBox.warning(self, "Không có dữ liệu", "Danh sách điểm đang trống.")
            return
        pairs, unmatched = self._match_white_black_pairs(
            self.csv_points,
            same_color_only=self.chk_csv_same_color.isChecked(),
            tolerance_mm=self.spin_csv_tolerance.value(),
        )
        self.log_box_csv.clear()
        self._log_pairs(pairs, unmatched, self.log_box_csv)
        self.lbl_csv_progress.setText(f"🔍 Xem trước: {len(pairs)} cặp sẽ chạy, {len(unmatched)} vật bị bỏ qua.")

    def _on_csv_offset_x_changed(self, val):
        self.csv_offset_x = val
        self.cfg["csv_offset_x"] = val
        save_config(self.cfg)

    def _on_csv_offset_y_changed(self, val):
        self.csv_offset_y = val
        self.cfg["csv_offset_y"] = val
        save_config(self.cfg)

    def on_csv_stop(self):
        self._csv_stop_flag = True
        self.lbl_csv_progress.setText("⏹ Đang dừng sau khi hoàn tất cặp hiện tại...")

    def on_csv_run(self):
        if self._busy:
            QMessageBox.warning(self, "Đang bận", "Robot đang thực hiện lệnh khác.")
            return
        if not self.csv_points:
            QMessageBox.warning(
                self, "Không có dữ liệu",
                "Danh sách điểm đang trống. Hãy lấy từ camera hoặc nạp file CSV."
            )
            return

        same_color_only = self.chk_csv_same_color.isChecked()
        tolerance = self.spin_csv_tolerance.value()
        pairs, unmatched = self._match_white_black_pairs(
            self.csv_points, same_color_only=same_color_only, tolerance_mm=tolerance
        )
        if not pairs:
            QMessageBox.warning(
                self, "Không ghép được cặp nào",
                "Không tìm thấy cặp Vật (white) - Lỗ (black) nào phù hợp trong danh sách.\n"
                "Hãy thử tăng dung sai đường kính hoặc bỏ chọn 'chỉ ghép cùng màu'."
            )
            return

        self.cfg["csv_match_tolerance"] = tolerance
        self.cfg["csv_match_same_color"] = same_color_only
        self.cfg["csv_z_pick"] = self.spin_csv_z_pick.value()
        save_config(self.cfg)

        self._csv_stop_flag = False
        self._set_manual_busy(True, "")
        self.btn_csv_run.setEnabled(False)
        self.btn_csv_stop.setEnabled(True)
        self.log_box_csv.clear()
        self._log_pairs(pairs, unmatched, self.log_box_csv)

        z_pick = self.spin_csv_z_pick.value()
        offset_x, offset_y = self.csv_offset_x, self.csv_offset_y

        # NEW: lấy cấu hình bậc tự do 4 tại thời điểm bấm chạy
        apply_dof4 = self.chk_csv_apply_dof4.isChecked()
        dof4_angle = self.spin_csv_dof4_angle.value()

        def task():
            total = len(pairs)
            for i, (w, b) in enumerate(pairs, start=1):
                if self._csv_stop_flag:
                    self._append_log_threadsafe_csv("⏹ Đã dừng theo yêu cầu.")
                    return False
                point_a = (w["x_mm"] + offset_x, w["y_mm"] + offset_y)
                point_b = (b["x_mm"] + offset_x, b["y_mm"] + offset_y)
                QTimer.singleShot(
                    0,
                    lambda i=i, total=total, pa=point_a, pb=point_b: self.lbl_csv_progress.setText(
                        f"Đang xử lý cặp {i}/{total}: Vật({pa[0]:.1f},{pa[1]:.1f}) "
                        f"→ Lỗ({pb[0]:.1f},{pb[1]:.1f})"
                    ),
                )

                # NEW: nếu bật "Áp dụng bậc tự do thứ 4" -> quay bàn xoay/step trước khi gắp-thả cặp này
                if apply_dof4:
                    self._append_log_threadsafe_csv(f"↻ [DOF4] Quay {dof4_angle:.1f}° trước khi xử lý cặp {i}")
                    self.pneu_uart.step_rotate(dof4_angle)

                with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe_csv(s))):
                    self.planner.pick_and_place(
                        point_a, point_b, z_pick=z_pick, gripper_callback=self._gripper_callback
                    )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_csv_run_done)
        worker.done_err.connect(self._on_csv_run_error)
        self._workers.append(worker)
        worker.start()

    def _append_log_threadsafe_csv(self, text):
        QTimer.singleShot(0, lambda: self.log_box_csv.append(text))

    def _on_csv_run_done(self, ok):
        self._set_manual_busy(False)
        self.btn_csv_run.setEnabled(True)
        self.btn_csv_stop.setEnabled(False)
        if ok:
            self.log_box_csv.append("=== HOÀN TẤT TOÀN BỘ DANH SÁCH ===")
            self.lbl_csv_progress.setText("✔ Đã chạy xong toàn bộ danh sách.")
        else:
            self.lbl_csv_progress.setText("⏹ Đã dừng giữa chừng.")
        self.current_pos = list(self.planner.HOME)
        self._update_pos_display()

    def _on_csv_run_error(self, err):
        self._set_manual_busy(False)
        self.btn_csv_run.setEnabled(True)
        self.btn_csv_stop.setEnabled(False)
        self.log_box_csv.append(f"⚠ LỖI: {err}")
        self.lbl_csv_progress.setText("⚠ Có lỗi xảy ra.")

    # ------------------------------------------------------------------
    # TAB 4 - KẾT NỐI
    # ------------------------------------------------------------------
    def _build_connection_tab(self):
        tab = QWidget()
        root = QHBoxLayout(tab)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        left_content = QWidget()
        left = QVBoxLayout(left_content)
        left.setContentsMargins(2, 2, 8, 2)
        left.setSpacing(10)

        robot_box = QGroupBox("KẾT NỐI ROBOT (UART 1)")
        robot_box.setObjectName("compactBox")
        rl = QGridLayout()
        rl.setSpacing(6)
        rl.addWidget(QLabel("Cổng COM:"), 0, 0)
        self.combo_robot_port = QComboBox()
        self.combo_robot_port.setEditable(True)
        rl.addWidget(self.combo_robot_port, 0, 1)

        rl.addWidget(QLabel("Baudrate:"), 0, 2)
        self.combo_robot_baud = QComboBox()
        self.combo_robot_baud.addItems(["115200"])
        self.combo_robot_baud.setCurrentText(str(self.cfg["robot_baud"]))
        rl.addWidget(self.combo_robot_baud, 0, 3)

        self.btn_robot_connect = QPushButton("🔌 KẾT NỐI")
        self.btn_robot_connect.setObjectName("connectBtn")
        self.btn_robot_connect.clicked.connect(self.on_connect_robot)
        rl.addWidget(self.btn_robot_connect, 1, 0, 1, 2)

        self.btn_robot_disconnect = QPushButton("⏻ NGẮT")
        self.btn_robot_disconnect.setObjectName("disconnectBtn")
        self.btn_robot_disconnect.clicked.connect(self.on_disconnect_robot)
        rl.addWidget(self.btn_robot_disconnect, 1, 2, 1, 2)

        self.lbl_robot_status = QLabel("● Chưa kết nối")
        self.lbl_robot_status.setObjectName("statusBad")
        rl.addWidget(self.lbl_robot_status, 2, 0, 1, 4)
        robot_box.setLayout(rl)
        left.addWidget(robot_box)

        pneu_box = QGroupBox("KẾT NỐI THIẾT BỊ PHỤ (UART 2)")
        pneu_box.setObjectName("compactBox")
        pl = QGridLayout()
        pl.setSpacing(6)
        pl.addWidget(QLabel("Cổng COM:"), 0, 0)
        self.combo_pneu_port = QComboBox()
        self.combo_pneu_port.setEditable(True)
        pl.addWidget(self.combo_pneu_port, 0, 1)

        pl.addWidget(QLabel("Baudrate:"), 0, 2)
        self.combo_pneu_baud = QComboBox()
        self.combo_pneu_baud.addItems(["115200"])
        self.combo_pneu_baud.setCurrentText(str(self.cfg["pneumatic_baud"]))
        pl.addWidget(self.combo_pneu_baud, 0, 3)

        self.btn_pneu_connect = QPushButton("🔌 KẾT NỐI")
        self.btn_pneu_connect.setObjectName("connectBtn")
        self.btn_pneu_connect.clicked.connect(self.on_connect_pneu)
        pl.addWidget(self.btn_pneu_connect, 1, 0, 1, 2)

        self.btn_pneu_disconnect = QPushButton("⏻ NGẮT")
        self.btn_pneu_disconnect.setObjectName("disconnectBtn")
        self.btn_pneu_disconnect.clicked.connect(self.on_disconnect_pneu)
        pl.addWidget(self.btn_pneu_disconnect, 1, 2, 1, 2)

        self.lbl_pneu_status = QLabel("● Chưa kết nối")
        self.lbl_pneu_status.setObjectName("statusBad")
        pl.addWidget(self.lbl_pneu_status, 2, 0, 1, 4)
        pneu_box.setLayout(pl)
        left.addWidget(pneu_box)

        left.addWidget(self._build_device_control_box())
        left.addStretch()

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setWidget(left_content)

        right = QVBoxLayout()
        right.setSpacing(10)

        cam_box = QGroupBox("CẤU HÌNH CAMERA")
        cam_box.setObjectName("compactBox")
        cl = QGridLayout()
        cl.setSpacing(6)
        cl.addWidget(QLabel("Chỉ số Camera:"), 0, 0)
        self.spin_cam_index = QSpinBox()
        self.spin_cam_index.setRange(0, 10)
        self.spin_cam_index.setValue(self.cfg.get("camera_index", 0))
        cl.addWidget(self.spin_cam_index, 0, 1)

        cl.addWidget(QLabel("File hiệu chuẩn (.npz):"), 1, 0)
        self.edit_calib_path = QLineEdit(self.cfg.get("calib_file", ""))
        cl.addWidget(self.edit_calib_path, 1, 1)
        self.btn_browse_calib = QPushButton("...")
        self.btn_browse_calib.setMaximumWidth(44)
        self.btn_browse_calib.clicked.connect(self.on_browse_calib)
        cl.addWidget(self.btn_browse_calib, 1, 2)
        cam_box.setLayout(cl)
        right.addWidget(cam_box)

        util_box = QGroupBox("TIỆN ÍCH")
        util_box.setObjectName("compactBox")
        ul = QHBoxLayout()
        ul.setSpacing(8)
        self.btn_refresh_ports = QPushButton("🔄 LÀM MỚI CỔNG COM")
        self.btn_refresh_ports.clicked.connect(self.on_refresh_ports)
        ul.addWidget(self.btn_refresh_ports)
        self.btn_save_conn_cfg = QPushButton("💾 LƯU CẤU HÌNH")
        self.btn_save_conn_cfg.setObjectName("saveBtn")
        self.btn_save_conn_cfg.clicked.connect(self.on_save_conn_cfg)
        ul.addWidget(self.btn_save_conn_cfg)
        util_box.setLayout(ul)
        right.addWidget(util_box)

        conn_log_box = QGroupBox("NHẬT KÝ KẾT NỐI")
        conn_log_box.setObjectName("compactBox")
        conn_log_layout = QVBoxLayout()
        self.conn_log = QTextEdit()
        self.conn_log.setReadOnly(True)
        conn_log_layout.addWidget(self.conn_log)
        conn_log_box.setLayout(conn_log_layout)
        right.addWidget(conn_log_box, 3)

        root.addWidget(left_scroll, 1)
        root.addLayout(right, 1)

        self.on_refresh_ports()
        return tab

    # ------------------------------------------------------------------
    # KHỐI ĐIỀU KHIỂN THIẾT BỊ PHỤ (UART 2): BƠM / BÀN XOAY / BẬC TỰ DO 4
    # ------------------------------------------------------------------
    def _build_device_control_box(self):
        outer_box = QGroupBox("ĐIỀU KHIỂN THIẾT BỊ PHỤ (UART 2)")
        outer_box.setObjectName("compactBox")
        outer_layout = QVBoxLayout()
        outer_layout.setSpacing(8)

        pump_box = QGroupBox("MÁY BƠM")
        pump_box.setObjectName("compactBox")
        pump_layout = QHBoxLayout()
        pump_layout.setSpacing(8)

        self.lbl_pump_state = QLabel("● TẮT")
        self.lbl_pump_state.setObjectName("deviceStateOff")
        self.lbl_pump_state.setMinimumWidth(60)
        pump_layout.addWidget(self.lbl_pump_state)

        self.btn_pump_on = QPushButton("▶ BẬT")
        self.btn_pump_on.setObjectName("deviceOnBtn")
        self.btn_pump_on.clicked.connect(self.on_pump_on)
        self.btn_pump_off = QPushButton("■ TẮT")
        self.btn_pump_off.setObjectName("deviceOffBtn")
        self.btn_pump_off.clicked.connect(self.on_pump_off)
        pump_layout.addWidget(self.btn_pump_on, 1)
        pump_layout.addWidget(self.btn_pump_off, 1)

        pump_box.setLayout(pump_layout)
        outer_layout.addWidget(pump_box)

        turn_box = QGroupBox("BÀN XOAY (PWM)")
        turn_box.setObjectName("compactBox")
        turn_layout = QVBoxLayout()
        turn_layout.setSpacing(6)

        turn_row1 = QHBoxLayout()
        turn_row1.setSpacing(6)
        turn_row1.addWidget(QLabel("PWM:"))
        self.spin_turn_pwm = QSpinBox()
        self.spin_turn_pwm.setRange(0, 255)
        self.spin_turn_pwm.setValue(self.turn_pwm_value)
        self.spin_turn_pwm.valueChanged.connect(self._on_turn_pwm_changed)
        turn_row1.addWidget(self.spin_turn_pwm, 1)
        self.btn_turn_on = QPushButton("▶ BẬT")
        self.btn_turn_on.setObjectName("deviceOnBtn")
        self.btn_turn_on.clicked.connect(self.on_turn_on)
        self.btn_turn_off = QPushButton("■ TẮT")
        self.btn_turn_off.setObjectName("deviceOffBtn")
        self.btn_turn_off.clicked.connect(self.on_turn_off)
        turn_row1.addWidget(self.btn_turn_on, 1)
        turn_row1.addWidget(self.btn_turn_off, 1)
        turn_layout.addLayout(turn_row1)

        turn_row2 = QHBoxLayout()
        turn_row2.setSpacing(6)
        self.lbl_turn_state = QLabel("● Đang TẮT")
        self.lbl_turn_state.setObjectName("deviceStateOff")
        turn_row2.addWidget(self.lbl_turn_state, 1)
        self.btn_turn_apply_pwm = QPushButton("⟳ ÁP DỤNG PWM")
        self.btn_turn_apply_pwm.setObjectName("deviceApplyBtn")
        self.btn_turn_apply_pwm.clicked.connect(self.on_turn_apply_pwm)
        turn_row2.addWidget(self.btn_turn_apply_pwm)
        turn_layout.addLayout(turn_row2)

        turn_box.setLayout(turn_layout)
        outer_layout.addWidget(turn_box)

        step_box = QGroupBox("BẬC TỰ DO 4 — GÓC QUAY (STEP)")
        step_box.setObjectName("compactBox")
        step_layout = QVBoxLayout()
        step_layout.setSpacing(6)

        step_row1 = QHBoxLayout()
        step_row1.setSpacing(6)
        step_row1.addWidget(QLabel("Góc (độ):"))
        self.spin_step_test_angle = QDoubleSpinBox()
        self.spin_step_test_angle.setRange(-3600.0, 3600.0)
        self.spin_step_test_angle.setSingleStep(5.0)
        self.spin_step_test_angle.setValue(float(self.cfg.get("step_test_angle", 90.0)))
        self.spin_step_test_angle.valueChanged.connect(self._on_step_test_angle_changed)
        step_row1.addWidget(self.spin_step_test_angle, 1)
        self.btn_step_rotate = QPushButton("↻ QUAY")
        self.btn_step_rotate.setObjectName("deviceApplyBtn")
        self.btn_step_rotate.clicked.connect(self.on_step_rotate)
        step_row1.addWidget(self.btn_step_rotate)
        step_layout.addLayout(step_row1)

        quick_row = QHBoxLayout()
        quick_row.setSpacing(6)
        quick_row.addWidget(QLabel("Nhanh:"))
        for deg, label in ((90, "+90°"), (-90, "-90°"), (180, "+180°")):
            b = QPushButton(label)
            b.setObjectName("deviceQuickBtn")
            b.clicked.connect(lambda checked=False, d=deg: self.pneu_uart.step_rotate(d))
            quick_row.addWidget(b)
        step_layout.addLayout(quick_row)

        self.lbl_step_state = QLabel("Chưa gửi lệnh quay nào.")
        self.lbl_step_state.setStyleSheet("color:#555555; font-size:9pt;")
        step_layout.addWidget(self.lbl_step_state)

        step_box.setLayout(step_layout)
        outer_layout.addWidget(step_box)

        outer_box.setLayout(outer_layout)

        self._device_ctrl_widgets = [
            self.btn_pump_on, self.btn_pump_off,
            self.btn_turn_on, self.btn_turn_off, self.btn_turn_apply_pwm, self.spin_turn_pwm,
            self.btn_step_rotate, self.spin_step_test_angle,
        ]
        return outer_box

    def on_pump_on(self):
        if self.pneu_uart.pump_on():
            self.pump_state = True
            self.lbl_pump_state.setText("● BẬT")
            self.lbl_pump_state.setObjectName("deviceStateOn")
            self._repolish(self.lbl_pump_state)

    def on_pump_off(self):
        if self.pneu_uart.pump_off():
            self.pump_state = False
            self.lbl_pump_state.setText("● TẮT")
            self.lbl_pump_state.setObjectName("deviceStateOff")
            self._repolish(self.lbl_pump_state)

    def _on_turn_pwm_changed(self, val):
        self.turn_pwm_value = val
        self.cfg["turn_pwm"] = val
        save_config(self.cfg)

    def on_turn_on(self):
        if self.pneu_uart.turn_set_speed(self.spin_turn_pwm.value()):
            self.turn_state = True
            self.lbl_turn_state.setText(f"● Đang BẬT (PWM={self.spin_turn_pwm.value()})")
            self.lbl_turn_state.setObjectName("deviceStateOn")
            self._repolish(self.lbl_turn_state)

    def on_turn_off(self):
        if self.pneu_uart.turn_off():
            self.turn_state = False
            self.lbl_turn_state.setText("● Đang TẮT")
            self.lbl_turn_state.setObjectName("deviceStateOff")
            self._repolish(self.lbl_turn_state)

    def on_turn_apply_pwm(self):
        if not self.turn_state:
            QMessageBox.information(self, "Bàn xoay đang tắt", "Hãy bấm 'BẬT' trước khi áp dụng PWM mới.")
            return
        if self.pneu_uart.turn_set_speed(self.spin_turn_pwm.value()):
            self.lbl_turn_state.setText(f"● Đang BẬT (PWM={self.spin_turn_pwm.value()})")

    def _on_step_test_angle_changed(self, val):
        self.cfg["step_test_angle"] = val
        save_config(self.cfg)

    def on_step_rotate(self):
        deg = self.spin_step_test_angle.value()
        if self.pneu_uart.step_rotate(deg):
            self.lbl_step_state.setText(f"↻ Đã gửi lệnh quay {deg:.1f}° lúc {time.strftime('%H:%M:%S')}")

    def _repolish(self, widget):
        widget.setStyleSheet("")
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def log_conn(self, text):
        if hasattr(self, "conn_log"):
            QTimer.singleShot(0, lambda: self.conn_log.append(text))
        else:
            print(text)

    def on_refresh_ports(self):
        ports = UARTComm.list_ports()
        self.combo_robot_port.clear()
        self.combo_robot_port.addItems(ports)
        if self.cfg["robot_port"] in ports:
            self.combo_robot_port.setCurrentText(self.cfg["robot_port"])
        else:
            self.combo_robot_port.setCurrentText(self.cfg["robot_port"])

        self.combo_pneu_port.clear()
        self.combo_pneu_port.addItems(ports)
        if self.cfg["pneumatic_port"] in ports:
            self.combo_pneu_port.setCurrentText(self.cfg["pneumatic_port"])
        else:
            self.combo_pneu_port.setCurrentText(self.cfg["pneumatic_port"])

    def on_connect_robot(self):
        self.robot_uart.port = self.combo_robot_port.currentText()
        self.robot_uart.baud = int(self.combo_robot_baud.currentText())
        self.robot_uart.dry_run = not SERIAL_AVAILABLE
        worker = FnWorker(self.robot_uart.connect)
        worker.done_ok.connect(lambda ok: self._refresh_conn_status())
        worker.done_err.connect(lambda err: self.log_conn(f"[ERR] {err}"))
        self._workers.append(worker)
        worker.start()

    def on_disconnect_robot(self):
        self.robot_uart.disconnect()
        self._refresh_conn_status()

    def on_connect_pneu(self):
        self.pneu_uart.port = self.combo_pneu_port.currentText()
        self.pneu_uart.baud = int(self.combo_pneu_baud.currentText())
        self.pneu_uart.dry_run = not SERIAL_AVAILABLE
        worker = FnWorker(self.pneu_uart.connect)
        worker.done_ok.connect(lambda ok: self._refresh_conn_status())
        worker.done_err.connect(lambda err: self.log_conn(f"[ERR] {err}"))
        self._workers.append(worker)
        worker.start()

    def on_disconnect_pneu(self):
        self.pneu_uart.disconnect()
        self._refresh_conn_status()

    def _refresh_conn_status(self):
        if self.robot_uart.is_connected:
            self.lbl_robot_status.setText("● Đã kết nối" + (" (DRY-RUN)" if self.robot_uart.dry_run else ""))
            self.lbl_robot_status.setObjectName("statusOk")
        else:
            self.lbl_robot_status.setText("● Chưa kết nối")
            self.lbl_robot_status.setObjectName("statusBad")
        self.lbl_robot_status.setStyleSheet("")
        self.lbl_robot_status.style().unpolish(self.lbl_robot_status)
        self.lbl_robot_status.style().polish(self.lbl_robot_status)

        if self.pneu_uart.is_connected:
            self.lbl_pneu_status.setText("● Đã kết nối" + (" (DRY-RUN)" if self.pneu_uart.dry_run else ""))
            self.lbl_pneu_status.setObjectName("statusOk")
        else:
            self.lbl_pneu_status.setText("● Chưa kết nối")
            self.lbl_pneu_status.setObjectName("statusBad")
        self.lbl_pneu_status.style().unpolish(self.lbl_pneu_status)
        self.lbl_pneu_status.style().polish(self.lbl_pneu_status)

        if hasattr(self, "_device_ctrl_widgets"):
            for w in self._device_ctrl_widgets:
                w.setEnabled(self.pneu_uart.is_connected)

    def on_browse_calib(self):
        path, _ = QFileDialog.getOpenFileName(self, "Chọn file hiệu chuẩn", "", "NPZ files (*.npz)")
        if path:
            self.edit_calib_path.setText(path)

    def on_save_conn_cfg(self):
        self.cfg["robot_port"] = self.combo_robot_port.currentText()
        self.cfg["robot_baud"] = int(self.combo_robot_baud.currentText())
        self.cfg["pneumatic_port"] = self.combo_pneu_port.currentText()
        self.cfg["pneumatic_baud"] = int(self.combo_pneu_baud.currentText())
        self.cfg["camera_index"] = self.spin_cam_index.value()
        self.cfg["calib_file"] = self.edit_calib_path.text()
        self.cfg["jog_step_xy"] = self.jog_step_xy
        self.cfg["jog_step_z"] = self.jog_step_z
        self.cfg["camera_offset_x"] = self.spin_offset_x.value()
        self.cfg["camera_offset_y"] = self.spin_offset_y.value()
        self.cfg["turn_pwm"] = self.spin_turn_pwm.value()
        self.cfg["step_test_angle"] = self.spin_step_test_angle.value()
        if save_config(self.cfg):
            self.log_conn("💾 Đã lưu cấu hình kết nối.")
        else:
            self.log_conn("⚠ Lưu cấu hình thất bại.")

    def closeEvent(self, event):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait(2000)
        self.robot_uart.disconnect()
        self.pneu_uart.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    apply_light_palette(app)   # Đã đổi tên và nội dung thành light
    app.setStyleSheet(STYLE_SHEET)
    win = MainWindow()
    win.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()