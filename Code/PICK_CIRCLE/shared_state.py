import os
import sys
import json
import time

import numpy as np

try:
    import cv2
except ImportError:
    print("LỖI: chưa cài opencv-python. Chạy: pip install opencv-python")
    sys.exit(1)

try:
    from PySide6.QtCore import Qt, QThread, Signal, QTimer
    from PySide6.QtGui import QImage, QPixmap, QColor, QPalette
    from PySide6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    )
except ImportError:
    print("LỖI: chưa cài PySide6. Chạy: pip install PySide6")
    sys.exit(1)

from hardware.Uart_1 import UARTComm, DEFAULT_BAUD as ROBOT_DEFAULT_BAUD, SERIAL_AVAILABLE
from kinematics.dhnghich import inverse_kinematics
from kinematics.move_delta_4dof import DeltaMotionPlanner, TIME_MOVE_FAST
# Dof4Planner (ctx.planner_dof4): planner CHUNG cho các bài toán dof4 KHÁC
# ngoài "BÀI TOÁN ĐỘNG" - ví dụ "BÀI TOÁN BẬC 4 TĨNH" (dynamic_window.py) và
# cờ csv_apply_dof4 ở CSV. VẪN dùng move_delta_4dof.py như cũ - KHÔNG đổi.
from kinematics.move_delta_4dof import DeltaMotionPlanner as Dof4Planner
# Run4DofPlanner (ctx.planner_run4dof): planner RIÊNG, CHỈ dùng bởi tab
# "BÀI TOÁN ĐỘNG" (DynamicRun4DofWindow) - có pick_dof4()/place_dof4() tách
# rời để gắp trước - chờ khung - thả sau. Không tab nào khác được dùng cái này.
from kinematics.move_run4dof import DeltaMotionPlanner as Run4DofPlanner
from hardware.Uart_2 import PneumaticComm, DEFAULT_BAUD as PNEU_DEFAULT_BAUD
from vision.Cameracircle import (
    CircleTracker,
    calculate_real_properties,
    px_to_mm_scale,
    undistort_image,
    detect_circles_hsv_optimized,
)
from vision.Camera_4dof import (
    detect_objects,
    detect_frame_holes,
    mold_bounding_region,
    is_inside_region,
    _build_red_mask,
)
if SERIAL_AVAILABLE:
    pass

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")

DEFAULT_HOME = (60.0, 0.0, 330.0)
DEFAULT_Z_SAFE = 340.0
DEFAULT_JOG_XY = 5.0
DEFAULT_JOG_Z = 5.0

COORD_MODE_CIRCLE = "circle"
COORD_MODE_DOF4 = "dof4"
COORD_MODE_DYNAMIC = "dynamic"

DOF4_PLACE_POINT = (0.0, 0.0)
DOF4_TARGET_ANGLE_DEG = 90.0

TURNTABLE_MAX_RPM = 65.0
DET_SCALE = 0.5

CSV_COLUMNS = ["Loại", "KT/Đường kính", "X (mm)", "Y (mm)", "Góc (°)"]
CSV_FIELDNAMES = ["type", "diameter_mm", "width_mm", "height_mm", "angle_deg", "x_mm", "y_mm"]
DYN_COLUMNS = ["Loại", "Slot", "X (mm)", "Y (mm)", "Góc (°)"]


def _fmt_size_col(item):
    if item.get("width_mm") is not None and item.get("height_mm") is not None:
        return f'{float(item["width_mm"]):.1f}x{float(item["height_mm"]):.1f}'
    d = item.get("diameter_mm")
    return f'{float(d):.2f}' if d is not None else "-"


def _fmt_angle_col(item):
    a = item.get("angle_deg")
    return f'{float(a):.1f}' if a is not None else "-"


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
        "csv_z_pick": 340.0,
        "turn_pwm": 150,
        "step_test_angle": 90.0,
        "coord_mode": COORD_MODE_CIRCLE,
        "csv_apply_dof4": False,
        "csv_dof4_angle": 90.0,
        "dyn_z_pick": 340.0,
        "dyn_rpm": 30.0,
        "dyn_offset_x": 0.0,
        "dyn_offset_y": 0.0,
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
# WORKER CHUNG
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
    def __init__(self, emit_fn):
        self.emit_fn = emit_fn

    def write(self, text):
        text = text.rstrip("\n")
        if text:
            self.emit_fn(text)

    def flush(self):
        pass


# =====================================================================
# THREAD CAMERA - hỗ trợ 3 chế độ:
#   "circle"  -> Cameracircle.py (vòng tròn, chỉ X/Y)
#   "dof4"    -> Camera_4dof.py, đơn vật đỏ 10x20mm + góc
#   "dynamic" -> Camera_4dof.py, khuôn 8 lỗ + vật rời (stateless)
# =====================================================================
class CameraThread(QThread):
    frame_ready = Signal(np.ndarray, list, float)
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index=0, calib_path=None, detect_mode=COORD_MODE_CIRCLE):
        super().__init__()
        self.camera_index = camera_index
        self.calib_path = calib_path
        self.detect_mode = detect_mode
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

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[CAMERA] Camera báo lại: {actual_fps:.1f}fps @ {int(actual_w)}x{int(actual_h)}")

        circle_tracker = CircleTracker(alpha=0.35, max_disappeared=15, dist_threshold=30)

        fps = 0.0
        frame_count = 0
        start_time = cv2.getTickCount()

        while self._running:
            ret, frame = cap.read()
            if not ret:
                self.error.emit("Mất tín hiệu camera")
                break

            try:
                if has_calib:
                    undistorted, new_cm = undistort_image(frame, self.camera_matrix, self.dist_coeffs)
                    fx, fy = new_cm[0, 0], new_cm[1, 1]
                    cx, cy = new_cm[0, 2], new_cm[1, 2]
                else:
                    undistorted = frame
                    h, w = frame.shape[:2]
                    fx = fy = 1.0
                    cx, cy = w / 2.0, h / 2.0

                display = undistorted.copy()
                origin_px = (int(round(cx)), int(round(cy)))
                cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

                items_out = []

                if self.detect_mode == COORD_MODE_DYNAMIC:
                    small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
                    small_min_area = max(20, int(round(80 * DET_SCALE * DET_SCALE)))
                    dyn_mask = _build_red_mask(small, min_component_area=small_min_area)
                    dyn_upscale = 1.0 / DET_SCALE

                    frame_result = detect_frame_holes(undistorted, cx, cy, fx, fy, mask=dyn_mask, upscale=dyn_upscale)
                    holes = frame_result['holes'] if frame_result['frame_found'] else []
                    objects = detect_objects(
                        undistorted, cx, cy, fx, fy,
                        mask=dyn_mask, upscale=dyn_upscale, refine_frame=undistorted,
                    )

                    if holes:
                        region = mold_bounding_region(holes, margin_mm=15.0)
                        objects = [o for o in objects if not is_inside_region(o["x_mm"], o["y_mm"], region)]

                    for o_item in objects:
                        px, py = int(round(o_item['cx_px'])), int(round(o_item['cy_px']))
                        cv2.circle(display, (px, py), 5, (0, 0, 255), -1)
                        label = f"Vat ({o_item['x_mm']:.1f},{o_item['y_mm']:.1f}) {o_item['angle_deg']:.0f}deg"
                        cv2.putText(display, label, (px - 65, py - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

                    if not holes:
                        cv2.putText(display,
                                    "Dang quet khung... (khong tim thay hoac chua co khung)",
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
                    else:
                        for h_item in holes:
                            px, py = int(round(h_item['cx_px'])), int(round(h_item['cy_px']))
                            cv2.circle(display, (px, py), 5, (0, 255, 0), -1)
                            cv2.putText(display, f"Lo#{h_item['slot_id']}", (px - 28, py - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                        anchor_x = sum(h['x_mm'] for h in holes) / len(holes)
                        anchor_y = sum(h['y_mm'] for h in holes) / len(holes)
                        anchor_angle = sum(h['angle_deg'] for h in holes) / len(holes)
                        cv2.putText(display,
                                    f"Khung: {len(holes)} lo trong, X={anchor_x:.1f} Y={anchor_y:.1f} Goc={anchor_angle:.1f}deg",
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)

                    for o_item in objects:
                        items_out.append({**o_item, "diameter_mm": None, "slot_id": None})
                    for h_item in holes:
                        items_out.append({**h_item, "diameter_mm": None})

                elif self.detect_mode == COORD_MODE_DOF4:
                    small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
                    small_min_area = max(20, int(round(150 * DET_SCALE * DET_SCALE)))
                    dof4_mask = _build_red_mask(small, min_component_area=small_min_area)
                    dof4_upscale = 1.0 / DET_SCALE

                    stable = detect_objects(
                        undistorted, cx, cy, fx, fy,
                        mask=dof4_mask, upscale=dof4_upscale, refine_frame=undistorted,
                    )
                    for t in stable:
                        px, py = int(round(t['cx_px'])), int(round(t['cy_px']))
                        cv2.circle(display, (px, py), 3, (0, 255, 0), -1)
                        label = f"{t['width_mm']:.1f}x{t['height_mm']:.1f}mm {t['angle_deg']:.0f}deg"
                        cv2.putText(display, label, (px - 70, py - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                        items_out.append({
                            "type": "red_rect", "diameter_mm": None,
                            "width_mm": t['width_mm'], "height_mm": t['height_mm'],
                            "angle_deg": t['angle_deg'],
                            "x_mm": t['x_mm'], "y_mm": t['y_mm'],
                        })
                else:
                    detected = detect_circles_hsv_optimized(undistorted)
                    stable_circles = circle_tracker.update(detected)
                    for item in stable_circles:
                        center_f = item["center"]
                        radius_f = item["radius"]
                        c_type = item["type"]
                        diameter_mm = calculate_real_properties(radius_f, c_type)
                        scale = px_to_mm_scale(c_type)
                        dx_px = -center_f[0] + cx
                        dy_px = cy - center_f[1]
                        x_mm = dx_px * scale
                        y_mm = dy_px * scale

                        center = (int(round(center_f[0])), int(round(center_f[1])))
                        radius = int(round(radius_f))
                        main_color = (0, 0, 255) if "red" in c_type else (255, 0, 0)
                        cv2.circle(display, center, radius, main_color, 2)
                        cv2.circle(display, center, 2, (0, 0, 255), 3)
                        label = f'{c_type} {diameter_mm:.1f}mm'
                        cv2.putText(display, label, (center[0] - 55, center[1] - radius - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, main_color, 2)

                        items_out.append({
                            "type": c_type, "diameter_mm": round(diameter_mm, 2),
                            "width_mm": None, "height_mm": None, "angle_deg": None,
                            "x_mm": round(x_mm, 2), "y_mm": round(y_mm, 2),
                        })

                frame_count += 1
                if frame_count >= 10:
                    end_time = cv2.getTickCount()
                    seconds = (end_time - start_time) / cv2.getTickFrequency()
                    fps = frame_count / seconds if seconds > 0 else 0.0
                    frame_count = 0
                    start_time = cv2.getTickCount()

                self.frame_ready.emit(display, items_out, fps)

            except Exception as e:
                self.error.emit(f"Lỗi xử lý frame: {e}")

        cap.release()
        self.stopped.emit()


def home_and_wait(uart, timeout: float = 20.0) -> bool:
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


def apply_light_palette(app):
    palette = QPalette()
    bg = QColor("#f5f5f5")
    base = QColor("#ffffff")
    text = QColor("#1a1a1a")
    disabled_text = QColor("#888888")
    highlight = QColor("#0078d4")

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


STYLE_SHEET = """
QWidget { background-color: #f5f5f5; color: #1a1a1a; font-family: 'Segoe UI', Arial; font-size: 14pt; }
QMainWindow { background-color: #f5f5f5; }
QDialog { background-color: #f5f5f5; color: #1a1a1a; }
QMessageBox { background-color: #f5f5f5; color: #1a1a1a; }
QGroupBox { border: 2px solid #c0c0c0; border-radius: 12px; margin-top: 14px; font-weight: 700; font-size: 13pt; padding-top: 10px; color: #1a1a1a; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }
QPushButton { background-color: #e0e0e0; color: #1a1a1a; border-radius: 14px; border: 2px solid #b0b0b0; padding: 10px; font-weight: 700; font-size: 15pt; }
QPushButton:hover { background-color: #d0d0d0; border-color: #0078d4; }
QPushButton:pressed { background-color: #0078d4; color: #ffffff; }
QPushButton:disabled { background-color: #d0d0d0; color: #888888; border-color: #b0b0b0; }
QPushButton#jogBtn { background-color: #e8e8e8; color: #1a1a1a; font-size: 20pt; min-height: 90px; }
QPushButton#jogBtn:hover { background-color: #d0d0d0; }
QPushButton#homeBtn { background-color: #d0d0d0; border-color: #b0b0b0; font-size: 16pt; min-height: 70px; }
QPushButton#homeBtn:hover { background-color: #c0c0c0; }
QPushButton#saveBtn { background-color: #0078d4; color: #ffffff; font-size: 15pt; min-height: 60px; }
QPushButton#saveBtn:hover { background-color: #106ebe; }
QPushButton#connectBtn { background-color: #0078d4; color: #ffffff; min-height: 38px; font-size: 11pt; padding: 6px; }
QPushButton#connectBtn:hover { background-color: #106ebe; }
QPushButton#disconnectBtn { background-color: #c00000; color: #ffffff; min-height: 38px; font-size: 11pt; padding: 6px; }
QPushButton#disconnectBtn:hover { background-color: #a00000; }
QPushButton#actionBtn { background-color: #0078d4; color: #ffffff; min-height: 65px; font-size: 16pt; }
QPushButton#actionBtn:hover { background-color: #106ebe; }
QPushButton#dynStartBtn { background-color: #0a7a3a; color: #ffffff; min-height: 65px; font-size: 16pt; }
QPushButton#dynStartBtn:hover { background-color: #0d9a4a; }
QPushButton#dynStopBtn { background-color: #c00000; color: #ffffff; min-height: 65px; font-size: 16pt; }
QPushButton#dynStopBtn:hover { background-color: #a00000; }
QPushButton#deviceOnBtn { background-color: #0078d4; color: #ffffff; min-height: 36px; font-size: 11pt; padding: 4px; }
QPushButton#deviceOnBtn:hover { background-color: #106ebe; }
QPushButton#deviceOffBtn { background-color: #c00000; color: #ffffff; min-height: 36px; font-size: 11pt; padding: 4px; }
QPushButton#deviceOffBtn:hover { background-color: #a00000; }
QPushButton#deviceApplyBtn { background-color: #b0b0b0; color: #1a1a1a; min-height: 32px; font-size: 10pt; padding: 4px; }
QPushButton#deviceApplyBtn:hover { background-color: #a0a0a0; }
QPushButton#deviceQuickBtn { background-color: #e0e0e0; color: #1a1a1a; min-height: 30px; font-size: 10pt; padding: 2px; border-color: #b0b0b0; }
QPushButton#deviceQuickBtn:hover { background-color: #d0d0d0; border-color: #0078d4; }
QPushButton#backBtn { background-color: #6c757d; color: #ffffff; min-height: 44px; font-size: 12pt; border-color: #5a6268; }
QPushButton#backBtn:hover { background-color: #5a6268; }
QPushButton#menuBtn { min-height: 80px; font-size: 17pt; background-color: #0078d4; color: #ffffff; border-color: #0a63ad; }
QPushButton#menuBtn:hover { background-color: #106ebe; }
QLabel#menuTitle1 { font-size: 22pt; font-weight: 900; color: #0078d4; }
QLabel#menuTitle2 { font-size: 18pt; font-weight: 800; color: #1a1a1a; }
QLabel#menuTitle3 { font-size: 14pt; font-weight: 700; color: #555555; }
QLabel#posDisplay { background-color: #ffffff; border: 2px solid #0078d4; border-radius: 10px; font-size: 26pt; font-weight: 800; color: #0a6b3a; padding: 14px; }
QLabel#statusOk { color: #0a7a3a; font-weight: 800; font-size: 14pt; }
QLabel#statusBad { color: #c00000; font-weight: 800; font-size: 14pt; }
QLabel#sectionTitle { font-size: 15pt; font-weight: 800; color: #1a1a1a; }
QLabel#deviceStateOn { color: #0a7a3a; font-weight: 800; font-size: 11pt; }
QLabel#deviceStateOff { color: #c00000; font-weight: 800; font-size: 11pt; }
QGroupBox#compactBox { margin-top: 10px; padding-top: 8px; font-size: 11pt; }
QGroupBox#compactBox QSpinBox, QGroupBox#compactBox QDoubleSpinBox { padding: 4px 6px; font-size: 11pt; min-height: 26px; }
QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit { background-color: #ffffff; border: 2px solid #c0c0c0; border-radius: 8px; padding: 8px; font-size: 13pt; color: #1a1a1a; }
QComboBox QAbstractItemView { background-color: #ffffff; color: #1a1a1a; border: 1px solid #c0c0c0; selection-background-color: #0078d4; selection-color: #ffffff; outline: none; }
QListView { background-color: #ffffff; color: #1a1a1a; }
QCheckBox, QRadioButton { spacing: 8px; }
QCheckBox::indicator, QRadioButton::indicator { width: 18px; height: 18px; border: 2px solid #b0b0b0; background-color: #ffffff; }
QRadioButton::indicator { border-radius: 9px; }
QCheckBox::indicator:checked, QRadioButton::indicator:checked { background-color: #0078d4; border-color: #0078d4; }
QTextEdit { background-color: #ffffff; border: 2px solid #c0c0c0; border-radius: 8px; color: #1a1a1a; font-family: Consolas, monospace; font-size: 11pt; }
QTableWidget { background-color: #ffffff; border: 2px solid #c0c0c0; border-radius: 8px; gridline-color: #d0d0d0; font-size: 11pt; }
QHeaderView::section { background-color: #e0e0e0; color: #1a1a1a; padding: 6px; border: none; font-weight: 700; }
QScrollBar:vertical { background: #e0e0e0; width: 12px; margin: 0; }
QScrollBar::handle:vertical { background: #b0b0b0; border-radius: 6px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: #0078d4; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: #e0e0e0; height: 12px; margin: 0; }
QScrollBar::handle:horizontal { background: #b0b0b0; border-radius: 6px; min-width: 24px; }
QScrollBar::handle:horizontal:hover { background: #0078d4; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
"""


# =====================================================================
# TRẠNG THÁI DÙNG CHUNG GIỮA CÁC CỬA SỔ TAB
# =====================================================================
class AppContext:
    def __init__(self):
        self.cfg = load_config()

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

        # planner_dof4: planner CHUNG cho các bài toán dof4 KHÁC ngoài
        # "BÀI TOÁN ĐỘNG" (ví dụ BÀI TOÁN BẬC 4 TĨNH, csv_apply_dof4...).
        # GIỮ NGUYÊN như cũ, dùng move_delta_4dof.py.
        self.planner_dof4 = Dof4Planner(uart_comm=self.robot_uart)
        self.planner_dof4.HOME = tuple(self.cfg["home_position"])
        self.planner_dof4.Z_SAFE = self.cfg["z_safe"]

        # planner_run4dof: planner RIÊNG, CHỈ dùng bởi tab "BÀI TOÁN ĐỘNG"
        # (DynamicRun4DofWindow) - từ kinematics/move_run4dof.py, có sẵn
        # pick_dof4()/place_dof4() để gắp trước - chờ khung - thả sau.
        self.planner_run4dof = Run4DofPlanner(uart_comm=self.robot_uart)
        self.planner_run4dof.HOME = tuple(self.cfg["home_position"])
        self.planner_run4dof.Z_SAFE = self.cfg["z_safe"]

        self.current_pos = list(self.cfg["home_position"])
        self.home_pos = list(self.cfg["home_position"])
        self.jog_step_xy = self.cfg["jog_step_xy"]
        self.jog_step_z = self.cfg["jog_step_z"]
        self.busy = False

        self.offset_x = self.cfg.get("camera_offset_x", 0.0)
        self.offset_y = self.cfg.get("camera_offset_y", 0.0)

        self.camera_thread = None
        self.latest_circles = []

        self.csv_offset_x = self.cfg.get("csv_offset_x", 0.0)
        self.csv_offset_y = self.cfg.get("csv_offset_y", 0.0)
        self.csv_points = []

        self.pump_state = False
        self.turn_state = False
        self.turn_pwm_value = int(self.cfg.get("turn_pwm", 150))

        self.coord_mode = self.cfg.get("coord_mode", COORD_MODE_CIRCLE)

        self.dynamic_camera_thread = None
        self.dynamic_running = False
        self.dynamic_busy = False
        self.dyn_offset_x = self.cfg.get("dyn_offset_x", 0.0)
        self.dyn_offset_y = self.cfg.get("dyn_offset_y", 0.0)
        self.latest_dyn_objects = []
        self.latest_dyn_holes = []

        self.conn_log_callback = None

    def log_conn(self, text):
        if self.conn_log_callback:
            self.conn_log_callback(text)
        else:
            print(text)

    def gripper_callback(self, state):
        if state == "on":
            self.pneu_uart.pump_on()
        else:
            self.pneu_uart.pump_off()

    def shutdown(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait(2000)
        if self.dynamic_camera_thread:
            self.dynamic_camera_thread.stop()
            self.dynamic_camera_thread.wait(2000)
        self.robot_uart.disconnect()
        self.pneu_uart.disconnect()


# =====================================================================
# LỚP NỀN CHO MỖI CỬA SỔ TAB - có nút QUAY LẠI MENU chung
# =====================================================================
class BaseTabWindow(QMainWindow):
    def __init__(self, ctx: AppContext, launcher, title: str):
        super().__init__()
        self.ctx = ctx
        self.launcher = launcher
        self._force_close = False
        self._workers = []
        self.setWindowTitle(title)

        central = QWidget()
        self.setCentralWidget(central)
        self.root_layout = QVBoxLayout(central)
        self.root_layout.setContentsMargins(10, 10, 10, 10)
        self.root_layout.setSpacing(8)

        back_row = QHBoxLayout()
        self.btn_back = QPushButton("⬅ QUAY LẠI MENU")
        self.btn_back.setObjectName("backBtn")
        self.btn_back.setFixedSize(180, 70)  # rộng 130px, cao 32px
        self.btn_back.setStyleSheet(
            "QPushButton#backBtn { font-size: 10pt; min-height: 0; padding: 2px 6px; }"
        )
        self.btn_back.clicked.connect(self.go_back)
        back_row.addWidget(self.btn_back)
        back_row.addStretch(1)
        self.root_layout.addLayout(back_row)

        self.content_widget = QWidget()
        self.root_layout.addWidget(self.content_widget, 1)

    def go_back(self):
        self.hide()
        if self.launcher:
            self.launcher.show()
            self.launcher.showMaximized()

    def closeEvent(self, event):
        if self._force_close:
            event.accept()
        else:
            event.ignore()
            self.go_back()

    def _track_worker(self, worker):
        self._workers.append(worker)

        def _cleanup():
            try:
                self._workers.remove(worker)
            except ValueError:
                pass

        worker.finished.connect(_cleanup)
        worker.start()