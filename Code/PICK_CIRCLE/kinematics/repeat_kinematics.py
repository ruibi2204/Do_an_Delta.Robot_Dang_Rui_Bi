import time
import numpy as np

from kinematics.dhnghich import inverse_kinematics


# ---------------------------------------------------------------------
# THAM SỐ TỐC ĐỘ / TẦN SỐ ĐIỀU KHIỂN - RIÊNG cho bài test độ lặp lại.
# Đây chỉ là giá trị mặc định (fallback); có thể ghi đè qua speed_params
# khi khởi tạo RepeatKinematicsPlanner hoặc qua update_speed_params().
# ---------------------------------------------------------------------
CONTROL_HZ = 100.0          # Tần số gửi lệnh điều khiển (Hz) - CỐ ĐỊNH

TIME_MOVE_XY = 2          # Thời gian di chuyển ngang tới điểm (ở Z an toàn)
TIME_MOVE_DOWN = 1      # Thời gian hạ từ Z an toàn xuống z_pick
TIME_MOVE_DIP = 1        # Thời gian hạ thêm +2mm (chạm sâu)
TIME_MOVE_LIFT = 1       # Thời gian nâng lại -2mm (về z_pick)
TIME_MOVE_UP = 1         # Thời gian nâng từ z_pick về lại Z an toàn

DIP_MM = 10.0                # Độ chạm sâu thêm (mm) theo yêu cầu bài test

# Danh sách khóa tham số tốc độ mà nơi gọi (GUI) có thể ghi đè.
SPEED_PARAM_KEYS = (
    "TIME_MOVE_XY",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_DIP",
    "TIME_MOVE_LIFT",
    "TIME_MOVE_UP",
    "CONTROL_HZ",
    "DIP_MM",
)


class MotionError(Exception):
    pass


class RepeatKinematicsPlanner:
    """
    Planner CHUYỂN ĐỘNG RIÊNG, ĐỘC LẬP cho tab "ĐÁNH GIÁ ĐỘ LẶP LẠI".
    KHÔNG dùng chung instance/class với DeltaMotionPlanner (planner),
    Dof4Planner (planner_dof4) hay Run4DofPlanner (planner_run4dof).
    KHÔNG điều khiển gripper - bài test chỉ đánh giá độ lặp lại vị trí cơ khí.
    """

    def __init__(self, uart_comm=None, speed_params=None):
        """
        uart_comm: đối tượng UART robot do GUI quản lý (hoặc None -> dry-run).
        speed_params: dict tùy chọn các khóa trong SPEED_PARAM_KEYS để GUI
            ghi đè tốc độ/tần số ngay lúc khởi tạo.
        """
        self.uart = uart_comm
        self.HOME = (60.0, 0.0, 300.0)
        self.Z_SAFE = 290.0
        self.current_pos = self.HOME  # theo dõi vị trí hiện tại để nội suy đường thẳng

        self._init_speed_params(speed_params)

    def _init_speed_params(self, speed_params):
        defaults = {
            "TIME_MOVE_XY": TIME_MOVE_XY,
            "TIME_MOVE_DOWN": TIME_MOVE_DOWN,
            "TIME_MOVE_DIP": TIME_MOVE_DIP,
            "TIME_MOVE_LIFT": TIME_MOVE_LIFT,
            "TIME_MOVE_UP": TIME_MOVE_UP,
            "CONTROL_HZ": CONTROL_HZ,
            "DIP_MM": DIP_MM,
        }
        if speed_params:
            for key in SPEED_PARAM_KEYS:
                if key in speed_params:
                    try:
                        value = float(speed_params[key])
                        if value > 0:
                            defaults[key] = value
                    except (TypeError, ValueError):
                        pass  # giữ giá trị mặc định nếu dữ liệu không hợp lệ

        self.TIME_MOVE_XY = defaults["TIME_MOVE_XY"]
        self.TIME_MOVE_DOWN = defaults["TIME_MOVE_DOWN"]
        self.TIME_MOVE_DIP = defaults["TIME_MOVE_DIP"]
        self.TIME_MOVE_LIFT = defaults["TIME_MOVE_LIFT"]
        self.TIME_MOVE_UP = defaults["TIME_MOVE_UP"]
        self.CONTROL_HZ = defaults["CONTROL_HZ"]
        self.CONTROL_DT = 1.0 / self.CONTROL_HZ
        self.DIP_MM = defaults["DIP_MM"]

    def update_speed_params(self, speed_params):
        """Cho phép GUI cập nhật lại tham số tốc độ ngay cả khi planner
        đã được tạo. Chỉ khóa hợp lệ (số thực > 0) mới được áp dụng."""
        self._init_speed_params({
            **{
                "TIME_MOVE_XY": self.TIME_MOVE_XY,
                "TIME_MOVE_DOWN": self.TIME_MOVE_DOWN,
                "TIME_MOVE_DIP": self.TIME_MOVE_DIP,
                "TIME_MOVE_LIFT": self.TIME_MOVE_LIFT,
                "TIME_MOVE_UP": self.TIME_MOVE_UP,
                "CONTROL_HZ": self.CONTROL_HZ,
                "DIP_MM": self.DIP_MM,
            },
            **(speed_params or {}),
        })

    def send_position(self, x, y, z):
        try:
            theta1, theta2, theta3 = inverse_kinematics(x, y, z)
        except Exception as e:
            print(f"[LỖI ĐỘNG HỌC] Không thể tính tọa độ ({x}, {y}, {z}): {e}")
            return False

        if self.uart and self.uart.is_connected:
            ok = self.uart.send_angles(theta1, theta2, theta3)
            if not ok:
                print(f"[LỖI UART] Gửi góc thất bại tại ({x:.1f}, {y:.1f}, {z:.1f}).")
            return ok
        else:
            print(
                f"[MOVE-DRY] X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f} | Góc:"
                f" {theta1:.1f}°, {theta2:.1f}°, {theta3:.1f}°"
            )
            return True

    @staticmethod
    def _s_curve(t_frac):
        """Smoothstep bậc 5 (6t^5 - 15t^4 + 10t^3): vận tốc = 0 ở đầu/cuối
        đoạn di chuyển -> chuyển động êm, không giật cục."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển từ self.current_pos tới (x, y, z) trong wait_s giây,
        gửi lệnh với tần số CỐ ĐỊNH self.CONTROL_HZ, dùng S-curve để mượt."""
        p1 = np.array(self.current_pos, dtype=float)
        p2 = np.array((x, y, z), dtype=float)
        target = (x, y, z)

        if np.linalg.norm(p2 - p1) < 1e-6:
            time.sleep(wait_s)
            self.current_pos = target
            return

        n_steps = max(int(wait_s * self.CONTROL_HZ), 1)

        for i in range(1, n_steps + 1):
            t_start = time.perf_counter()

            frac = self._s_curve(i / n_steps)
            point = tuple(p1 + (p2 - p1) * frac)

            if not self.send_position(*point):
                raise MotionError(f"Di chuyển tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")

            elapsed = time.perf_counter() - t_start
            remaining = self.CONTROL_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

        self.current_pos = target

    def move_home(self):
        print("--> Đang về Home...")
        x, y, z = self.HOME
        try:
            self._move_or_raise(x, y, z, self.TIME_MOVE_XY)
        except MotionError as e:
            print(f"[LỖI] Về Home thất bại! Robot có thể vẫn đang ở vị trí cũ. ({e})")
            return False
        return True

    def _try_safe_retreat(self, x, y):
        print("[AN TOÀN] Đang cố nhấc lên độ cao an toàn và về Home sau lỗi...")
        try:
            self._move_or_raise(x, y, self.Z_SAFE, self.TIME_MOVE_UP)
        except Exception as e:
            print(f"[AN TOÀN] Không thể nhấc lên an toàn: {e}")
        self.move_home()

    def touch_point(self, point, z_pick):
        """
        1 lần "chạm" tại 1 điểm, dùng để đánh giá độ lặp lại:
            1. XY tới điểm, giữ Z an toàn
            2. Hạ xuống z_pick
            3. Hạ thêm +DIP_MM (mặc định +2mm)
            4. Nâng lên -DIP_MM (về lại z_pick)
            5. Nâng lên lại Z an toàn
        KHÔNG dùng gripper.
        """
        x, y = point
        z_dip = z_pick + self.DIP_MM

        try:
            # 1. Tới điểm, giữ Z an toàn
            self._move_or_raise(x, y, self.Z_SAFE, self.TIME_MOVE_XY)

            # 2. Hạ xuống z_pick
            self._move_or_raise(x, y, z_pick, self.TIME_MOVE_DOWN)

            # 3. Hạ thêm +2mm (chạm sâu để test độ lặp lại)
            self._move_or_raise(x, y, z_dip, self.TIME_MOVE_DIP)

            # 4. Nâng lên -2mm (quay lại đúng z_pick)
            self._move_or_raise(x, y, z_pick, self.TIME_MOVE_LIFT)

            # 5. Nâng lên lại Z an toàn
            self._move_or_raise(x, y, self.Z_SAFE, self.TIME_MOVE_UP)

        except MotionError as e:
            print(f"[LỖI QUY TRÌNH] {e}")
            self._try_safe_retreat(x, y)
            raise

    def repeat_cycle(self, point_a, point_b, z_pick):
        """
        1 CHU KỲ LẶP hoàn chỉnh cho bài test độ lặp lại:

            Điểm A -> z_pick -> +2 -> -2 -> về Home
            -> Điểm B -> z_pick -> +2 -> -2 -> về Home

        Được RepeatTestWindow (windows/repeat_test_window.py) gọi lặp lại
        N lần (số lần lặp do người dùng nhập).
        """
        print("=== BẮT ĐẦU CHU KỲ: ĐIỂM A ===")
        self.touch_point(point_a, z_pick)
        self.move_home()

        print("=== TIẾP TỤC CHU KỲ: ĐIỂM B ===")
        self.touch_point(point_b, z_pick)
        self.move_home()

        print("=== HOÀN TẤT 1 CHU KỲ LẶP (A -> B) ===\n")
        return True