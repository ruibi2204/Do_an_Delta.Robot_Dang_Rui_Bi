"""
trajectory_planner.py - Quản lý quỹ đạo chuyển động cho robot Delta
Hỗ trợ các loại quỹ đạo: điểm-điểm, đường thẳng, vòng tròn, hình chữ nhật
"""

import time
import math
import threading
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# Import các module cần thiết
try:
    from Math_Control.kinematics import inverse_kinematics
    from Math_Control.gear_ratio import joints_to_motors, GEAR_RATIO
except ImportError:
    print("[WARNING] Không thể import kinematics hoặc gear_ratio")


    # Tạo hàm giả cho testing
    def inverse_kinematics(x, y, z):
        return x * 0.1, y * 0.1, z * 0.1


    GEAR_RATIO = 1.0


    def joints_to_motors(t1, t2, t3):
        return t1 * GEAR_RATIO, t2 * GEAR_RATIO, t3 * GEAR_RATIO


class MotionType(Enum):
    """Các loại quỹ đạo chuyển động"""
    POINT_TO_POINT = "point_to_point"  # Di chuyển thẳng từ điểm hiện tại đến đích
    LINEAR = "linear"  # Di chuyển theo đường thẳng với các điểm trung gian
    CIRCLE = "circle"  # Quỹ đạo hình tròn
    RECTANGLE = "rectangle"  # Quỹ đạo hình chữ nhật
    SPIRAL = "spiral"  # Quỹ đạo xoắn ốc
    ZIGZAG = "zigzag"  # Quỹ đạo zigzag
    APPROACH = "approach"  # Tiếp cận mục tiêu (di chuyển từ trên xuống)


@dataclass
class TrajectoryPoint:
    """Điểm trên quỹ đạo"""
    x: float
    y: float
    z: float
    feedrate: Optional[float] = None
    delay: float = 0.0  # Thời gian dừng tại điểm (giây)

    def to_tuple(self):
        """Chuyển đổi thành tuple để tương thích với code cũ"""
        if self.feedrate is not None:
            return (self.x, self.y, self.z, self.feedrate)
        return (self.x, self.y, self.z)


class TrajectoryPlanner:
    """
    Lớp lập kế hoạch và tạo quỹ đạo chuyển động cho robot Delta

    Các chức năng chính:
    - Tạo quỹ đạo điểm-điểm an toàn (bay lên cao, di chuyển, hạ xuống)
    - Tạo quỹ đạo đường thẳng, vòng tròn, hình chữ nhật, xoắn ốc
    - Tạo quỹ đạo tiếp cận mục tiêu (approach)
    - Chuyển đổi quỹ đạo thành danh sách điểm để gửi cho robot
    """

    def __init__(self,
                 safe_z: float = 300.0,  # Chiều cao an toàn khi di chuyển ngang
                 work_z: float = 380.0,  # Chiều cao làm việc (thấp hơn)
                 z_speed: float = 500.0,  # Tốc độ di chuyển theo trục Z
                 xy_speed: float = 1000.0,  # Tốc độ di chuyển XY
                 approach_height: float = 350.0,  # Chiều cao khi bắt đầu tiếp cận
                 ):
        """
        Khởi tạo bộ lập quỹ đạo

        Args:
            safe_z: Chiều cao an toàn để di chuyển ngang (mm)
            work_z: Chiều cao làm việc (mm)
            z_speed: Tốc độ di chuyển trục Z (mm/phút)
            xy_speed: Tốc độ di chuyển XY (mm/phút)
            approach_height: Chiều cao bắt đầu tiếp cận mục tiêu (mm)
        """
        self.safe_z = safe_z
        self.work_z = work_z
        self.z_speed = z_speed
        self.xy_speed = xy_speed
        self.approach_height = approach_height

        # Lưu vị trí hiện tại của robot (giả định)
        self.current_position = (0, 0, self.safe_z)

        # Callback để di chuyển robot
        self.move_callback: Optional[Callable] = None

    def set_move_callback(self, callback: Callable[[float, float, float, float], bool]):
        """
        Thiết lập hàm callback để di chuyển robot

        Args:
            callback: Hàm nhận (x, y, z, feedrate) và trả về bool thành công
        """
        self.move_callback = callback

    def set_current_position(self, x: float, y: float, z: float):
        """Cập nhật vị trí hiện tại của robot"""
        self.current_position = (x, y, z)

    # =========================================================================
    #  CÁC PHƯƠNG THỨC TẠO QUỸ ĐẠO CƠ BẢN
    # =========================================================================

    def generate_approach_trajectory(self,
                                     target_x: float,
                                     target_y: float,
                                     target_z: Optional[float] = None,
                                     approach_height: Optional[float] = None,
                                     hold_time: float = 0.5) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo tiếp cận mục tiêu (approach trajectory)

        Quy trình:
        1. Từ vị trí hiện tại, nâng lên safe_z
        2. Di chuyển ngang đến vị trí target_x, target_y ở safe_z
        3. Hạ xuống approach_height
        4. Hạ từ từ đến work_z (hoặc target_z)

        Args:
            target_x: Tọa độ X đích
            target_y: Tọa độ Y đích
            target_z: Tọa độ Z đích (mặc định = work_z)
            approach_height: Chiều cao bắt đầu tiếp cận (mặc định = self.approach_height)
            hold_time: Thời gian dừng tại đích (giây)

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên quỹ đạo
        """
        if target_z is None:
            target_z = self.work_z
        if approach_height is None:
            approach_height = self.approach_height

        points = []
        current_x, current_y, current_z = self.current_position

        # Bước 1: Nâng lên safe_z nếu đang thấp hơn
        if current_z < self.safe_z:
            points.append(TrajectoryPoint(current_x, current_y, self.safe_z, self.z_speed))

        # Bước 2: Di chuyển ngang đến vị trí mục tiêu ở safe_z
        points.append(TrajectoryPoint(target_x, target_y, self.safe_z, self.xy_speed))

        # Bước 3: Hạ xuống approach_height
        if approach_height < self.safe_z:
            points.append(TrajectoryPoint(target_x, target_y, approach_height, self.z_speed))

        # Bước 4: Hạ từ từ đến target_z (work_z)
        # Chia thành nhiều bước nhỏ để chuyển động mượt hơn
        num_steps = 10
        z_start = approach_height
        z_end = target_z
        for i in range(num_steps):
            t = (i + 1) / num_steps
            # Sử dụng easing function để giảm tốc khi đến gần
            # ease_out_quad: chậm dần khi đến cuối
            ease_t = 1 - (1 - t) * (1 - t)
            z = z_start + (z_end - z_start) * ease_t
            feed = self.z_speed * (0.5 + 0.5 * ease_t)  # Giảm tốc dần
            points.append(TrajectoryPoint(target_x, target_y, z, feed))

        # Dừng tại điểm đích
        if hold_time > 0:
            points.append(TrajectoryPoint(target_x, target_y, target_z, feed, hold_time))

        return points

    def generate_point_to_point(self,
                                target_x: float,
                                target_y: float,
                                target_z: Optional[float] = None,
                                safe_move: bool = True,
                                hold_time: float = 0.0) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo điểm-điểm với tùy chọn an toàn

        Args:
            target_x: Tọa độ X đích
            target_y: Tọa độ Y đích
            target_z: Tọa độ Z đích (mặc định = current_z)
            safe_move: Di chuyển an toàn (nâng lên safe_z trước khi di chuyển ngang)
            hold_time: Thời gian dừng tại đích

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên quỹ đạo
        """
        if target_z is None:
            target_z = self.current_position[2]

        points = []
        current_x, current_y, current_z = self.current_position

        if safe_move:
            # Nâng lên safe_z
            if current_z < self.safe_z:
                points.append(TrajectoryPoint(current_x, current_y, self.safe_z, self.z_speed))

            # Di chuyển ngang đến đích
            points.append(TrajectoryPoint(target_x, target_y, self.safe_z, self.xy_speed))

            # Hạ xuống target_z
            if target_z < self.safe_z:
                points.append(TrajectoryPoint(target_x, target_y, target_z, self.z_speed))
        else:
            # Di chuyển thẳng đến đích
            points.append(TrajectoryPoint(target_x, target_y, target_z, self.xy_speed))

        if hold_time > 0:
            points[-1].delay = hold_time

        return points

    def generate_linear_trajectory(self,
                                   start_x: float, start_y: float, start_z: float,
                                   end_x: float, end_y: float, end_z: float,
                                   num_points: int = 50) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo đường thẳng với các điểm trung gian

        Args:
            start_x, start_y, start_z: Điểm bắt đầu
            end_x, end_y, end_z: Điểm kết thúc
            num_points: Số điểm trung gian

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên đường thẳng
        """
        points = []
        for i in range(num_points + 1):
            t = i / num_points
            x = start_x + (end_x - start_x) * t
            y = start_y + (end_y - start_y) * t
            z = start_z + (end_z - start_z) * t
            points.append(TrajectoryPoint(x, y, z, self.xy_speed))
        return points

    def generate_circle_trajectory(self,
                                   center_x: float,
                                   center_y: float,
                                   center_z: float,
                                   radius: float,
                                   num_points: int = 360,
                                   start_angle: float = 0,
                                   end_angle: float = 360,
                                   clockwise: bool = True) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo hình tròn

        Args:
            center_x, center_y, center_z: Tâm đường tròn
            radius: Bán kính
            num_points: Số điểm trên đường tròn
            start_angle: Góc bắt đầu (độ)
            end_angle: Góc kết thúc (độ)
            clockwise: Quay theo chiều kim đồng hồ

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên đường tròn
        """
        points = []
        angle_range = end_angle - start_angle
        direction = -1 if clockwise else 1

        for i in range(num_points + 1):
            angle_deg = start_angle + direction * angle_range * (i / num_points)
            angle_rad = math.radians(angle_deg)
            x = center_x + radius * math.cos(angle_rad)
            y = center_y + radius * math.sin(angle_rad)
            points.append(TrajectoryPoint(x, y, center_z, self.xy_speed))

        return points

    def generate_rectangle_trajectory(self,
                                      center_x: float,
                                      center_y: float,
                                      center_z: float,
                                      width: float,
                                      height: float,
                                      num_points_per_side: int = 20) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo hình chữ nhật

        Args:
            center_x, center_y, center_z: Tâm hình chữ nhật
            width: Chiều rộng
            height: Chiều cao
            num_points_per_side: Số điểm trên mỗi cạnh

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên hình chữ nhật
        """
        points = []
        half_w = width / 2
        half_h = height / 2

        # 4 góc của hình chữ nhật
        corners = [
            (center_x - half_w, center_y - half_h),
            (center_x + half_w, center_y - half_h),
            (center_x + half_w, center_y + half_h),
            (center_x - half_w, center_y + half_h),
        ]

        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]

            for j in range(num_points_per_side):
                t = j / num_points_per_side
                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t
                points.append(TrajectoryPoint(x, y, center_z, self.xy_speed))

        # Điểm cuối về góc đầu tiên
        points.append(TrajectoryPoint(corners[0][0], corners[0][1], center_z, self.xy_speed))

        return points

    def generate_spiral_trajectory(self,
                                   center_x: float,
                                   center_y: float,
                                   center_z: float,
                                   max_radius: float,
                                   num_turns: int = 3,
                                   num_points_per_turn: int = 60) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo xoắn ốc

        Args:
            center_x, center_y, center_z: Tâm xoắn ốc
            max_radius: Bán kính tối đa
            num_turns: Số vòng xoắn
            num_points_per_turn: Số điểm mỗi vòng

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên xoắn ốc
        """
        points = []
        total_points = num_turns * num_points_per_turn

        for i in range(total_points + 1):
            t = i / total_points
            angle = 2 * math.pi * num_turns * t
            radius = max_radius * t

            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            points.append(TrajectoryPoint(x, y, center_z, self.xy_speed))

        return points

    def generate_zigzag_trajectory(self,
                                   start_x: float,
                                   start_y: float,
                                   start_z: float,
                                   end_x: float,
                                   end_y: float,
                                   amplitude: float,
                                   frequency: int = 5,
                                   num_points: int = 100) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo zigzag

        Args:
            start_x, start_y, start_z: Điểm bắt đầu
            end_x, end_y: Điểm kết thúc
            amplitude: Biên độ zigzag
            frequency: Số lần zigzag
            num_points: Số điểm trên quỹ đạo

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên zigzag
        """
        points = []

        for i in range(num_points + 1):
            t = i / num_points

            # Đường thẳng từ start đến end
            x = start_x + (end_x - start_x) * t
            y_base = start_y + (end_y - start_y) * t

            # Zigzag theo trục Y
            y_offset = amplitude * math.sin(2 * math.pi * frequency * t)
            y = y_base + y_offset

            points.append(TrajectoryPoint(x, y, start_z, self.xy_speed))

        return points

    def generate_with_waypoints(self,
                                waypoints: List[Tuple[float, float, float]],
                                safe_move: bool = True,
                                smooth: bool = True,
                                num_intermediate: int = 10) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo đi qua các waypoint

        Args:
            waypoints: Danh sách các điểm (x, y, z) cần đi qua
            safe_move: Di chuyển an toàn (nâng lên safe_z)
            smooth: Làm mượt quỹ đạo (sử dụng spline)
            num_intermediate: Số điểm trung gian giữa các waypoint

        Returns:
            List[TrajectoryPoint]: Danh sách các điểm trên quỹ đạo
        """
        if len(waypoints) < 2:
            return []

        all_points = []

        # Bắt đầu từ vị trí hiện tại
        current_pos = self.current_position
        all_waypoints = [current_pos] + waypoints

        for i in range(len(all_waypoints) - 1):
            x1, y1, z1 = all_waypoints[i]
            x2, y2, z2 = all_waypoints[i + 1]

            if safe_move:
                # Nâng lên safe_z từ vị trí hiện tại
                if z1 < self.safe_z:
                    # Chỉ nâng nếu đang thấp hơn safe_z
                    pass
                # Di chuyển đến điểm tiếp theo ở safe_z
                # Đi xuống target_z
                pass

            # Tạo các điểm trung gian
            for j in range(num_intermediate):
                t = (j + 1) / num_intermediate
                # Sử dụng easing function
                # ease_in_out_cubic
                if t < 0.5:
                    t2 = 2 * t * t
                else:
                    t2 = 1 - pow(-2 * t + 2, 2) / 2

                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t
                z = z1 + (z2 - z1) * t
                all_points.append(TrajectoryPoint(x, y, z, self.xy_speed))

        return all_points

    # =========================================================================
    #  PHƯƠNG THỨC TIỆN ÍCH
    # =========================================================================

    def to_flat_points(self, trajectory: List[TrajectoryPoint]) -> List[Tuple]:
        """
        Chuyển đổi danh sách TrajectoryPoint thành tuple để tương thích với code cũ

        Args:
            trajectory: Danh sách TrajectoryPoint

        Returns:
            List[Tuple]: Danh sách các tuple (x, y, z) hoặc (x, y, z, feedrate)
        """
        return [pt.to_tuple() for pt in trajectory]

    def execute_trajectory(self,
                           trajectory: List[TrajectoryPoint],
                           move_callback: Optional[Callable] = None,
                           progress_callback: Optional[Callable[[int, int], None]] = None) -> bool:
        """
        Thực thi quỹ đạo thông qua callback

        Args:
            trajectory: Danh sách các điểm trên quỹ đạo
            move_callback: Hàm di chuyển (mặc định dùng self.move_callback)
            progress_callback: Hàm báo tiến trình (done, total)

        Returns:
            bool: Thành công hay không
        """
        if move_callback is None:
            move_callback = self.move_callback

        if move_callback is None:
            print("[ERROR] Không có callback di chuyển")
            return False

        total = len(trajectory)
        for i, pt in enumerate(trajectory):
            # Cập nhật vị trí hiện tại
            self.current_position = (pt.x, pt.y, pt.z)

            # Di chuyển đến điểm
            if pt.feedrate is not None:
                success = move_callback(pt.x, pt.y, pt.z, pt.feedrate)
            else:
                success = move_callback(pt.x, pt.y, pt.z, self.xy_speed)

            if not success:
                print(f"[ERROR] Di chuyển thất bại tại điểm {i}")
                return False

            # Dừng tại điểm nếu có delay
            if pt.delay > 0:
                time.sleep(pt.delay)

            # Báo tiến trình
            if progress_callback:
                progress_callback(i + 1, total)

        return True

    def get_approach_to_target(self,
                               target_x: float,
                               target_y: float,
                               z_approach: float = None) -> List[TrajectoryPoint]:
        """
        Tạo quỹ đạo tiếp cận mục tiêu đơn giản cho camera

        Quy trình:
        1. Di chuyển từ vị trí hiện tại đến (target_x, target_y) ở safe_z
        2. Hạ xuống work_z

        Args:
            target_x: Tọa độ X đích
            target_y: Tọa độ Y đích
            z_approach: Chiều cao làm việc (mặc định = self.work_z)

        Returns:
            List[TrajectoryPoint]: Quỹ đạo tiếp cận
        """
        if z_approach is None:
            z_approach = self.work_z

        return self.generate_approach_trajectory(
            target_x=target_x,
            target_y=target_y,
            target_z=z_approach,
            hold_time=0.3
        )

    def move_to_safe_position(self, x: float, y: float) -> List[TrajectoryPoint]:
        """
        Di chuyển đến vị trí (x, y) ở safe_z

        Args:
            x, y: Tọa độ đích

        Returns:
            List[TrajectoryPoint]: Quỹ đạo di chuyển an toàn
        """
        return self.generate_point_to_point(
            target_x=x,
            target_y=y,
            target_z=self.safe_z,
            safe_move=True,
            hold_time=0
        )


# =========================================================================
#  LỚP QUẢN LÝ QUỸ ĐẠO CHO MAIN GUI
# =========================================================================

class TrajectoryManager:
    """
    Lớp quản lý quỹ đạo tích hợp với main GUI

    Cung cấp các phương thức đơn giản để gọi từ GUI
    """

    def __init__(self, move_callback: Callable):
        """
        Khởi tạo với callback di chuyển robot

        Args:
            move_callback: Hàm move_robot_to(x, y, z, feedrate)
        """
        self.planner = TrajectoryPlanner()
        self.planner.set_move_callback(move_callback)
        self.trajectory_thread = None
        self.is_running = False
        self.stop_flag = False

    def set_current_position(self, x: float, y: float, z: float):
        """Cập nhật vị trí hiện tại"""
        self.planner.set_current_position(x, y, z)

    def approach_target(self,
                        target_x: float,
                        target_y: float,
                        target_z: Optional[float] = None,
                        callback: Optional[Callable] = None):
        """
        Tiếp cận mục tiêu (phương thức chính cho camera)

        Robot sẽ:
        1. Nâng lên chiều cao an toàn
        2. Di chuyển ngang đến target_x, target_y
        3. Hạ xuống target_z

        Args:
            target_x, target_y: Tọa độ mục tiêu
            target_z: Chiều cao làm việc (mặc định: work_z)
            callback: Hàm gọi khi hoàn thành
        """
        if target_z is None:
            target_z = self.planner.work_z

        trajectory = self.planner.get_approach_to_target(target_x, target_y, target_z)
        self._execute_in_thread(trajectory, callback)

    def move_to_point(self,
                      x: float,
                      y: float,
                      z: Optional[float] = None,
                      safe: bool = True,
                      callback: Optional[Callable] = None):
        """
        Di chuyển đến một điểm

        Args:
            x, y, z: Tọa độ đích
            safe: Di chuyển an toàn (nâng lên trước khi di chuyển ngang)
            callback: Hàm gọi khi hoàn thành
        """
        if z is None:
            z = self.planner.current_position[2]

        trajectory = self.planner.generate_point_to_point(x, y, z, safe_move=safe)
        self._execute_in_thread(trajectory, callback)

    def move_circle(self,
                    center_x: float,
                    center_y: float,
                    center_z: float,
                    radius: float,
                    num_points: int = 360,
                    callback: Optional[Callable] = None):
        """
        Di chuyển theo quỹ đạo hình tròn
        """
        trajectory = self.planner.generate_circle_trajectory(
            center_x, center_y, center_z, radius, num_points
        )
        self._execute_in_thread(trajectory, callback)

    def move_rectangle(self,
                       center_x: float,
                       center_y: float,
                       center_z: float,
                       width: float,
                       height: float,
                       callback: Optional[Callable] = None):
        """
        Di chuyển theo quỹ đạo hình chữ nhật
        """
        trajectory = self.planner.generate_rectangle_trajectory(
            center_x, center_y, center_z, width, height
        )
        self._execute_in_thread(trajectory, callback)

    def stop(self):
        """Dừng quỹ đạo đang chạy"""
        self.stop_flag = True
        if self.trajectory_thread and self.trajectory_thread.is_alive():
            self.trajectory_thread.join(timeout=1.0)
        self.is_running = False

    def _execute_in_thread(self,
                           trajectory: List[TrajectoryPoint],
                           callback: Optional[Callable] = None):
        """Thực thi quỹ đạo trong thread riêng"""
        if self.is_running:
            print("[WARNING] Quỹ đạo khác đang chạy, dừng lại...")
            self.stop()

        self.stop_flag = False
        self.is_running = True

        def run():
            try:
                def move_cb(x, y, z, feed):
                    if self.stop_flag:
                        return False
                    return self.planner.move_callback(x, y, z, feed)

                success = self.planner.execute_trajectory(
                    trajectory,
                    move_callback=move_cb
                )

                if callback:
                    callback(success)

            except Exception as e:
                print(f"[ERROR] Lỗi khi thực thi quỹ đạo: {e}")
            finally:
                self.is_running = False

        self.trajectory_thread = threading.Thread(target=run, daemon=True)
        self.trajectory_thread.start()


# =========================================================================
#  HÀM TIỆN ÍCH DEMO / TEST
# =========================================================================

def demo_trajectory():
    """Hàm demo các loại quỹ đạo"""

    # Tạo callback giả
    def fake_move(x, y, z, feed):
        print(f"Move to: ({x:.2f}, {y:.2f}, {z:.2f}) @ {feed:.0f}")
        return True

    planner = TrajectoryPlanner()
    planner.set_move_callback(fake_move)
    planner.set_current_position(0, 0, 300)

    print("=" * 60)
    print("DEMO QUỸ ĐẠO TIẾP CẬN")
    print("=" * 60)

    # Demo approach trajectory
    traj = planner.get_approach_to_target(100, 80)
    print(f"Generated {len(traj)} points for approach")

    print("\nFirst 5 points:")
    for i, pt in enumerate(traj[:5]):
        print(f"  {i}: ({pt.x:.2f}, {pt.y:.2f}, {pt.z:.2f}) f={pt.feedrate}")

    print(f"\nLast point: ({traj[-1].x:.2f}, {traj[-1].y:.2f}, {traj[-1].z:.2f}) delay={traj[-1].delay}s")

    print("\n" + "=" * 60)
    print("DEMO CÁC QUỸ ĐẠO KHÁC")
    print("=" * 60)

    # Demo circle
    traj = planner.generate_circle_trajectory(0, 0, 380, 50, 72)
    print(f"Circle: {len(traj)} points")

    # Demo rectangle
    traj = planner.generate_rectangle_trajectory(0, 0, 380, 100, 60)
    print(f"Rectangle: {len(traj)} points")

    # Demo spiral
    traj = planner.generate_spiral_trajectory(0, 0, 380, 50, 2)
    print(f"Spiral: {len(traj)} points")

    # Demo zigzag
    traj = planner.generate_zigzag_trajectory(-50, -50, 380, 50, 50, 10, 5)
    print(f"Zigzag: {len(traj)} points")

    print("\n" + "=" * 60)
    print("DEMO TRAJECTORY MANAGER")
    print("=" * 60)

    # Demo TrajectoryManager
    manager = TrajectoryManager(fake_move)
    manager.set_current_position(0, 0, 300)

    def on_complete(success):
        print(f"Trajectory completed: {'SUCCESS' if success else 'FAILED'}")

    # Sử dụng manager
    print("\nApproaching target...")
    manager.approach_target(100, 80, callback=on_complete)

    # Chờ hoàn thành
    time.sleep(1)

    print("\nMoving to point...")
    manager.move_to_point(-50, -30, 400, callback=on_complete)

    time.sleep(1)
    print("\nDemo complete!")


if __name__ == "__main__":
    demo_trajectory()