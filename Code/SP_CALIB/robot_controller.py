import json
import time
from inverse_kinematics import inverse_kinematics
from uart_handler import UARTController


class DeltaRobotController:
    # ĐỒNG BỘ VỚI camera_calib_XY.py:
    #   robot_x = trục X, duong TU DUOI LEN (doc)
    #   robot_y = trục Y, duong TU PHAI SANG TRAI (ngang)
    # Neu robot chay nguoc huong thuc te (do khac biet co khi giua khung camera
    # va khung robot), doi SWAP_XY = True de hoan doi lai x/y khi nap diem.
    SWAP_XY = False

    def __init__(self, gear_ratio=3.0):
        self.uart = UARTController()
        self.current_thetas = [0, 0, 0]
        self.corners_data = []
        self.is_running = False
        self.current_index = 0
        self.target_z = 300.0
        self.gear_ratio = gear_ratio
        self.safe_z = 380.0

    def connect_uart(self, port, baudrate=115200):
        return self.uart.connect(port, baudrate)

    def disconnect_uart(self):
        return self.uart.disconnect()

    def get_ports(self):
        return self.uart.find_ports()

    def is_connected(self):
        return self.uart.is_connected

    def send_theta(self, theta1, theta2, theta3):
        success, msg = self.uart.send_theta(
            theta1 * self.gear_ratio,
            theta2 * self.gear_ratio,
            theta3 * self.gear_ratio
        )
        if success:
            self.current_thetas = [theta1, theta2, theta3]
        return success, msg

    def go_to_position(self, x, y, z):
        try:
            theta1, theta2, theta3 = inverse_kinematics(x, y, z)
            success, msg = self.send_theta(theta1, theta2, theta3)
            return success, msg, (theta1, theta2, theta3)
        except ValueError as e:
            return False, str(e), None

    def go_home(self):
        return self.go_to_position(0, 0, self.target_z)

    def move_to_point_from_home(self, x, y, z, lift_height=20.0, dwell_time=2.0, speed_delay=1.0):
        """
        Di chuyển từ home (0,0, target_z) đến điểm (x,y,z) và quay về home
        Quy trình: nâng lên -> di chuyển ngang -> hạ xuống -> dừng -> nâng lên -> về home -> hạ xuống
        speed_delay: thời gian chờ giữa các bước để di chuyển chậm hơn (mặc định 1.0s)
        dwell_time: thời gian dừng tại điểm (mặc định 5.0s)
        """
        try:
            lift_z = z - lift_height

            # 1. Nâng từ home lên
            print(f"  ⬆️ Nâng từ home lên {lift_z:.1f}mm...")
            success, msg, _ = self.go_to_position(0, 0, lift_z)
            if not success:
                return False, f"Lỗi nâng từ home: {msg}", None
            time.sleep(speed_delay)

            # 2. Di chuyển ngang đến (x,y) ở độ cao lift_z
            print(f"  ➡️ Di chuyển đến ({x:.1f}, {y:.1f}) ở độ cao {lift_z:.1f}mm...")
            success, msg, _ = self.go_to_position(x, y, lift_z)
            if not success:
                return False, f"Lỗi di chuyển ngang: {msg}", None
            time.sleep(speed_delay)

            # 3. Hạ xuống điểm đích
            print(f"  ⬇️ Hạ xuống ({x:.1f}, {y:.1f}, {z:.1f})...")
            success, msg, thetas = self.go_to_position(x, y, z)
            if not success:
                return False, f"Lỗi hạ xuống: {msg}", None

            # 4. Dừng tại điểm (dwell_time = 5s)
            print(f"  ⏱️ Dừng {dwell_time}s...")
            time.sleep(dwell_time)

            # 5. Nâng lên
            print(f"  ⬆️ Nâng lên {lift_z:.1f}mm...")
            success, msg, _ = self.go_to_position(x, y, lift_z)
            if not success:
                return False, f"Lỗi nâng lên sau điểm: {msg}", None
            time.sleep(speed_delay)

            # 6. Về home (0,0) ở độ cao lift_z
            print(f"  ⬅️ Về home (0,0) ở độ cao {lift_z:.1f}mm...")
            success, msg, _ = self.go_to_position(0, 0, lift_z)
            if not success:
                return False, f"Lỗi về home: {msg}", None
            time.sleep(speed_delay)

            # 7. Hạ xuống home (về target_z = 300)
            print(f"  ⬇️ Hạ xuống home (0,0,{self.target_z:.1f})...")
            success, msg, _ = self.go_to_position(0, 0, self.target_z)
            if not success:
                return False, f"Lỗi hạ xuống home: {msg}", None
            time.sleep(speed_delay)

            return True, "Hoàn thành di chuyển từ home", thetas
        except Exception as e:
            return False, f"Lỗi: {str(e)}", None

    def load_corners_from_json(self, filepath="board_corners.json"):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                self.corners_data = json.load(f)
            return True, f"Đã đọc {len(self.corners_data)} điểm từ file"
        except FileNotFoundError:
            return False, f"Không tìm thấy file {filepath}"
        except Exception as e:
            return False, f"Lỗi đọc file: {str(e)}"

    def get_points_by_z(self, target_z):
        points = []
        for point in self.corners_data:
            if abs(point.get('robot_z', 0) - target_z) < 0.01:
                robot_x = point.get('robot_x', 0)   # trục X: duoi len (theo camera_calib_XY.py)
                robot_y = point.get('robot_y', 0)   # trục Y: phai sang trai (theo camera_calib_XY.py)

                if self.SWAP_XY:
                    x_val, y_val = robot_y, robot_x
                else:
                    x_val, y_val = robot_x, robot_y

                points.append({
                    'x': x_val,
                    'y': y_val,
                    'z': point.get('robot_z', 0),
                    'row': point.get('row', 0),
                    'col': point.get('col', 0)
                })
        return points

    def get_all_z_levels(self):
        if not self.corners_data:
            return []
        return sorted(set([p.get('robot_z', 0) for p in self.corners_data]))

    def get_points_info(self):
        if not self.corners_data:
            return "Chưa có dữ liệu điểm"

        info = [f"Tổng số điểm: {len(self.corners_data)}"]
        z_values = self.get_all_z_levels()
        info.append(f"Các mức Z: {z_values}")

        for z in z_values:
            count = sum(1 for p in self.corners_data if abs(p.get('robot_z', 0) - z) < 0.01)
            info.append(f"  Z={z:.1f}mm: {count} điểm")

        return '\n'.join(info)

    def run_auto_scan(self, target_z, delay=1.0, callback=None, lift_height=20.0, dwell_time=2.0, speed_delay=1.0):
        """
        Tự động quét tất cả các điểm có độ cao target_z.
        Mỗi điểm: từ home -> điểm -> về home.
        - delay: thời gian chờ giữa các điểm (mặc định 1.0s)
        - speed_delay: thời gian chờ giữa các bước di chuyển (mặc định 1.0s)
        - dwell_time: thời gian dừng tại điểm (mặc định 5.0s)
        """
        self.target_z = target_z

        if not self.corners_data:
            return False, "Chưa có dữ liệu điểm!"

        points = self.get_points_by_z(target_z)
        if not points:
            return False, f"Không tìm thấy điểm nào có Z = {target_z}mm"

        self.is_running = True
        results = []

        print(f"\n🚀 Bắt đầu quét {len(points)} điểm")
        print(f"   LIFT: {lift_height}mm, DWELL: {dwell_time}s, SPEED DELAY: {speed_delay}s\n")

        for i, point in enumerate(points):
            if not self.is_running:
                print("⏹️ Đã dừng quét")
                break

            x, y, z = point['x'], point['y'], point['z']
            print(f"\n📍 Điểm {i + 1}/{len(points)}: ({x:.1f}, {y:.1f}, {z:.1f})")

            success, msg, thetas = self.move_to_point_from_home(x, y, z, lift_height, dwell_time, speed_delay)

            result = {
                'index': i,
                'x': x, 'y': y, 'z': z,
                'row': point['row'],
                'col': point['col'],
                'success': success,
                'message': msg,
                'thetas': thetas,
                'thetas_with_gear': [t * self.gear_ratio for t in thetas] if thetas else None
            }
            results.append(result)

            if callback:
                callback(i + 1, len(points), result)

            if success:
                print(f"  ✅ Thành công")
                if thetas:
                    print(f"     Góc motor: T1={thetas[0]:.2f}°, T2={thetas[1]:.2f}°, T3={thetas[2]:.2f}°")
            else:
                print(f"  ❌ Lỗi: {msg}")

            if i < len(points) - 1 and self.is_running:
                print(f"  ⏳ Đợi {delay}s trước điểm tiếp theo...")
                time.sleep(delay)

        self.is_running = False
        return True, results

    def stop_scan(self):
        self.is_running = False

    def get_gear_ratio(self):
        return self.gear_ratio

    def set_gear_ratio(self, ratio):
        self.gear_ratio = ratio
        return f"Đã cập nhật tỉ số truyền: u = {ratio}"

    def set_safe_z(self, z):
        self.safe_z = z
        return f"Đã cập nhật độ cao an toàn: {z}mm"