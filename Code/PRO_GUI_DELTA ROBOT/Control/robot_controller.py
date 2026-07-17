# ============================================================================
# FILE: robot_controller.py
# ============================================================================
"""
robot_controller.py - Module điều khiển robot
Chuyên xử lý: đọc file, điều khiển chuyển động, quản lý trạng thái robot
"""

import os
import csv
import json
import time
import threading
from typing import List, Tuple, Optional, Callable

from PyQt5.QtCore import QThread, pyqtSignal

# Import các module của dự án
from Math_Control.kinematics import inverse_kinematics
from Math_Control.gear_ratio import joints_to_motors
from uart_comm import UartComm
from vision_coords import CameraCalibration
from trajectory_planner import TrajectoryManager


# =========================================================================
#  THREAD CHAY CHUOI DIEM
# =========================================================================
class MotionThread(QThread):
    """Thread chạy chuỗi điểm di chuyển"""
    progress = pyqtSignal(int, int)
    finished_signal = pyqtSignal(bool)
    log_signal = pyqtSignal(str)

    def __init__(self, move_func, points, feed, delay=0.0, get_current_pos_func=None):
        super().__init__()
        self.move_func = move_func
        self.points = points
        self.default_feed = feed
        self.delay = delay
        self._stop_flag = False
        self.get_current_pos_func = get_current_pos_func
        self.safe_z = 300.0  # Safe Z position (nhấc lên cao)

    def stop(self):
        self._stop_flag = True

    def run(self):
        total = len(self.points)
        for i, pt in enumerate(self.points):
            if self._stop_flag:
                self.log_signal.emit("[INFO] Da dung chuoi lenh theo yeu cau")
                self.finished_signal.emit(False)
                return
            if len(pt) == 4:
                x, y, z, f = pt
            else:
                x, y, z = pt
                f = self.default_feed

            # Lấy vị trí hiện tại của robot để thực hiện nhấc lên
            if self.get_current_pos_func:
                curr_x, curr_y, curr_z = self.get_current_pos_func()
            else:
                curr_x, curr_y, curr_z = x, y, self.safe_z

            # 1. Nhấc lên vị trí an toàn tại tọa độ hiện tại (chỉ nhấc nếu Z hiện tại thấp hơn safe_z, tức Z > safe_z)
            if curr_z > self.safe_z:
                self.move_func(curr_x, curr_y, self.safe_z, f)

            # 2. Di chuyển ngang đến X, Y đích tại vị trí safe_z
            self.move_func(x, y, self.safe_z, f)

            # 3. Hạ xuống Z đích
            self.move_func(x, y, z, f)

            self.progress.emit(i + 1, total)
            if self.delay > 0:
                time.sleep(self.delay)
        self.finished_signal.emit(True)


# =========================================================================
#  LỚP ĐIỀU KHIỂN ROBOT
# =========================================================================
class RobotController:
    """
    Lớp điều khiển robot - Xử lý tất cả logic liên quan đến robot
    """

    def __init__(self, log_callback: Optional[Callable] = None):
        """
        Khởi tạo bộ điều khiển robot

        Args:
            log_callback: Hàm callback để ghi log
        """
        self.log_callback = log_callback

        # UART Communication
        self.uart = UartComm()
        self.uart.log_signal.connect(self._on_uart_log)

        # Motion thread
        self.motion_thread = None

        # Trajectory manager
        self.traj_manager = None

        # Camera calibration
        self.calibration = CameraCalibration()

        # Dữ liệu điểm đã load
        self.loaded_points = []

        # Vị trí hiện tại (giả định)
        self.current_position = (0, 0, 300)

        # Callback khi kết nối thay đổi
        self.connection_changed_callback = None

    def _on_uart_log(self, text: str):
        """Xử lý log từ UART"""
        self._log(text)

    def _log(self, text: str):
        """Ghi log"""
        if self.log_callback:
            self.log_callback(text)

    def set_connection_callback(self, callback: Callable[[bool], None]):
        """Thiết lập callback khi kết nối thay đổi"""
        self.connection_changed_callback = callback
        self.uart.connection_changed.connect(callback)

    # ===== UART CONNECTION =====

    def list_ports(self) -> List[str]:
        """Danh sách cổng COM"""
        return self.uart.list_ports()

    def connect(self, port: str, baud: int) -> bool:
        """Kết nối đến robot"""
        return self.uart.connect(port, baud)

    def disconnect(self):
        """Ngắt kết nối"""
        self.uart.disconnect()

    def is_connected(self) -> bool:
        """Kiểm tra kết nối"""
        return self.uart.is_connected

    def home(self):
        """Đưa robot về vị trí home"""
        threading.Thread(target=self.uart.home, daemon=True).start()

    def emergency_stop(self):
        """Dừng khẩn cấp"""
        if self.motion_thread and self.motion_thread.isRunning():
            self.motion_thread.stop()
        self.uart.emergency_stop()

    # ===== MOVEMENT =====

    def move_to(self, x: float, y: float, z: float, feed: float) -> bool:
        """
        Di chuyển robot đến vị trí (x, y, z) với feedrate

        Returns:
            bool: Thành công hay không
        """
        try:
            t1, t2, t3 = inverse_kinematics(x, y, z)
        except ValueError as e:
            self._log(f"[LOI IK] {e}")
            return False

        m1, m2, m3 = joints_to_motors(t1, t2, t3)
        self.current_position = (x, y, z)
        return self.uart.send_motor_angles(m1, m2, m3, feed)

    def move_with_trajectory(self, x: float, y: float, z: float,
                             safe: bool = True, callback: Optional[Callable] = None):
        """
        Di chuyển với quỹ đạo an toàn

        Args:
            x, y, z: Tọa độ đích
            safe: Di chuyển an toàn
            callback: Hàm gọi khi hoàn thành
        """
        if self.traj_manager is None:
            self.traj_manager = TrajectoryManager(self.move_to)
            self.traj_manager.set_current_position(*self.current_position)

        self.traj_manager.set_current_position(*self.current_position)
        self.traj_manager.move_to_point(x, y, z, safe=safe, callback=callback)

    def approach_target(self, x: float, y: float, z: Optional[float] = None,
                        callback: Optional[Callable] = None):
        """
        Tiếp cận mục tiêu (nâng lên, di chuyển ngang, hạ xuống)

        Args:
            x, y: Tọa độ mục tiêu
            z: Chiều cao làm việc
            callback: Hàm gọi khi hoàn thành
        """
        if self.traj_manager is None:
            self.traj_manager = TrajectoryManager(self.move_to)
            self.traj_manager.set_current_position(*self.current_position)

        self.traj_manager.set_current_position(*self.current_position)
        self.traj_manager.approach_target(x, y, z, callback)

    def stop_motion(self):
        """Dừng chuyển động hiện tại"""
        if self.motion_thread and self.motion_thread.isRunning():
            self.motion_thread.stop()
        if self.traj_manager:
            self.traj_manager.stop()

    # ===== FILE LOADING =====

    def load_csv_file(self, path: str, has_header: bool = True) -> List[Tuple]:
        """
        Load file CSV

        Returns:
            List[Tuple]: Danh sách các điểm (x, y, z, f) hoặc (x, y, z)
        """
        points = []
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = list(reader)

        if has_header and rows:
            rows = rows[1:]

        for row in rows:
            if len(row) < 3:
                continue
            try:
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                f_val = float(row[3]) if len(row) > 3 and row[3].strip() != "" else None
                if f_val is not None:
                    points.append((x, y, z, f_val))
                else:
                    points.append((x, y, z))
            except ValueError:
                continue

        self.loaded_points = points
        self._log(f"[INFO] Da load {len(points)} diem tu CSV: {os.path.basename(path)}")
        return points

    def load_json_file(self, path: str) -> List[Tuple]:
        """
        Load file JSON

        Returns:
            List[Tuple]: Danh sách các điểm (x, y, z, f) hoặc (x, y, z)
        """
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, list):
            raise ValueError("File JSON phai la mot mang cac diem.")

        points = []
        key_x_candidates = ['robot_x', 'x', 'X', 'Robot_X']
        key_y_candidates = ['robot_y', 'y', 'Y', 'Robot_Y']
        key_z_candidates = ['robot_z', 'z', 'Z', 'Robot_Z']
        key_f_candidates = ['f', 'F', 'feed', 'feedrate', 'speed']

        for item in data:
            if isinstance(item, dict):
                x = None
                y = None
                z = None
                f = None
                for k in key_x_candidates:
                    if k in item:
                        x = item[k]
                        break
                for k in key_y_candidates:
                    if k in item:
                        y = item[k]
                        break
                for k in key_z_candidates:
                    if k in item:
                        z = item[k]
                        break
                for k in key_f_candidates:
                    if k in item and item[k] is not None:
                        f = float(item[k])
                        break

                if x is not None and y is not None and z is not None:
                    try:
                        x = float(x);
                        y = float(y);
                        z = float(z)
                        if f is not None:
                            points.append((x, y, z, f))
                        else:
                            points.append((x, y, z))
                    except (ValueError, TypeError):
                        continue

            elif isinstance(item, list) and len(item) >= 3:
                try:
                    x, y, z = float(item[0]), float(item[1]), float(item[2])
                    f_val = float(item[3]) if len(item) > 3 and item[3] is not None else None
                    if f_val is not None:
                        points.append((x, y, z, f_val))
                    else:
                        points.append((x, y, z))
                except (ValueError, TypeError):
                    continue

        self.loaded_points = points
        self._log(f"[INFO] Da load {len(points)} diem tu JSON: {os.path.basename(path)}")
        return points

    def run_loaded_points(self, feed: float, progress_callback=None) -> bool:
        """
        Chạy các điểm đã load

        Args:
            feed: Feedrate mặc định
            progress_callback: Hàm callback (done, total)

        Returns:
            bool: Thành công hay không
        """
        if not self.loaded_points:
            self._log("[ERROR] Chua co diem nao duoc load")
            return False

        if not self.uart.is_connected:
            self._log("[ERROR] Chua ket noi robot")
            return False

        self.motion_thread = MotionThread(
            self.move_to, 
            self.loaded_points, 
            feed, 
            get_current_pos_func=lambda: self.current_position
        )

        if progress_callback:
            self.motion_thread.progress.connect(progress_callback)

        self.motion_thread.log_signal.connect(self._log)
        self.motion_thread.start()
        self._log(f"[INFO] Bat dau chay {len(self.loaded_points)} diem tu file")
        return True

    # ===== CAMERA CALIBRATION =====

    def calibrate_add_point(self, pixel: Tuple[float, float], robot: Tuple[float, float]):
        """Thêm cặp điểm hiệu chỉnh"""
        self.calibration.add_point_pair(pixel, robot)
        return len(self.calibration.pixel_points)

    def calibrate_compute(self):
        """Tính toán ma trận hiệu chỉnh"""
        self.calibration.compute()
        return len(self.calibration.pixel_points)

    def calibrate_clear(self):
        """Xóa tất cả điểm hiệu chỉnh"""
        self.calibration.clear()

    def pixel_to_robot(self, px: float, py: float) -> Tuple[float, float]:
        """Chuyển đổi pixel sang tọa độ robot"""
        return self.calibration.pixel_to_robot(px, py)

    def get_calibration_points(self) -> int:
        """Số điểm hiệu chỉnh"""
        return len(self.calibration.pixel_points)