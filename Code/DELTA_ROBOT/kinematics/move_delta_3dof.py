import time
import math
import threading
import numpy as np
from kinematics.dhnghich import inverse_kinematics

# =====================================================================
# move_delta3dof.py — PLANNER CHUYỂN ĐỘNG RIÊNG CHO ROBOT DELTA 3 BẬC
#
# CHỈ dùng bởi tab "BÀI TOÁN TĨNH — CAMERA TRỰC TIẾP" (CsvWindow,
# ctx.planner_3dof). Nội suy đường thẳng + S-curve, MAX_STEP_MM làm thẳng
# quỹ đạo. KHÔNG có bậc tự do 4 (không xoay bàn xoay).
#
# Trình tự gắp-thả TỐI GIẢN (chỉ các điểm cần thiết):
#   Z_SAFE (cố định) -> hạ Z_PICK -> hút -> nhấc Z_SAFE -> sang B ->
#   hạ Z_PLACE -> nhả -> nhấc Z_SAFE -> HOME
#
# File này KHÔNG import Uart_2 trực tiếp. Bơm hút đi qua gripper_callback
# do GUI truyền vào, để chỉ có DUY NHẤT một kết nối UART2 trong toàn ứng
# dụng.
#
# self.current_pos = None nghĩa là "không biết robot đang ở đâu" (sau khi
# firmware tự HOME, jog tay...). Khi đó lần di chuyển kế tiếp luôn gửi
# lệnh thật tới đích, không bị bỏ qua vì tưởng khoảng cách = 0.
# =====================================================================

# Giá trị mặc định
TIME_MOVE_FAST = 1.0
TIME_MOVE_DOWN = 0.5
TIME_MOVE_ACTION = 0.5
TIME_DELAY_GRIPPER = 0.5

# Tần số gửi lệnh điều khiển (Hz) — cố định.
CONTROL_HZ = 100.0

# Khoảng cách Cartesian tối đa (mm) giữa 2 điểm gửi liên tiếp.
MAX_STEP_MM = 3.0

# Thời gian (s) giữ/gửi lặp lệnh HOME sau khi đã tới nơi.
HOME_HOLD_S = 0.3

SPEED_PARAM_KEYS = (
    "TIME_MOVE_FAST",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_ACTION",
    "TIME_DELAY_GRIPPER",
    "CONTROL_HZ",
    "MAX_STEP_MM",
)


class MotionError(Exception):
    pass


class MotionAborted(Exception):
    """Người dùng nhấn DỪNG giữa chừng (request_abort()). Không kế thừa
    MotionError để không bị nhầm với lỗi gửi lệnh."""
    pass


class DeltaMotionPlanner:

    def __init__(self, uart_comm=None, speed_params=None):
        self.uart = uart_comm
        self.HOME = (60.0, 0.0, 280.0)
        self.Z_SAFE = 280.0           # độ cao di chuyển an toàn, cố định
        self.current_pos = self.HOME  # None = không rõ vị trí thật
        self._abort = threading.Event()

        self._init_speed_params(speed_params)

    # ------------------------------------------------------------------
    # THAM SỐ TỐC ĐỘ
    # ------------------------------------------------------------------
    def _init_speed_params(self, speed_params):
        defaults = {
            "TIME_MOVE_FAST": TIME_MOVE_FAST,
            "TIME_MOVE_DOWN": TIME_MOVE_DOWN,
            "TIME_MOVE_ACTION": TIME_MOVE_ACTION,
            "TIME_DELAY_GRIPPER": TIME_DELAY_GRIPPER,
            "CONTROL_HZ": CONTROL_HZ,
            "MAX_STEP_MM": MAX_STEP_MM,
        }
        if speed_params:
            for key in SPEED_PARAM_KEYS:
                if key in speed_params:
                    try:
                        value = float(speed_params[key])
                        if value > 0:
                            defaults[key] = value
                    except (TypeError, ValueError):
                        pass

        self.TIME_MOVE_FAST = defaults["TIME_MOVE_FAST"]
        self.TIME_MOVE_DOWN = defaults["TIME_MOVE_DOWN"]
        self.TIME_MOVE_ACTION = defaults["TIME_MOVE_ACTION"]
        self.TIME_DELAY_GRIPPER = defaults["TIME_DELAY_GRIPPER"]
        self.CONTROL_HZ = defaults["CONTROL_HZ"]
        self.CONTROL_DT = 1.0 / self.CONTROL_HZ
        self.MAX_STEP_MM = defaults["MAX_STEP_MM"]

    def update_speed_params(self, speed_params):
        """Cập nhật tham số tốc độ lúc chạy (thanh trượt tốc độ). Khóa lạ
        hoặc giá trị không hợp lệ (<= 0) sẽ bị bỏ qua."""
        self._init_speed_params({
            "TIME_MOVE_FAST": self.TIME_MOVE_FAST,
            "TIME_MOVE_DOWN": self.TIME_MOVE_DOWN,
            "TIME_MOVE_ACTION": self.TIME_MOVE_ACTION,
            "TIME_DELAY_GRIPPER": self.TIME_DELAY_GRIPPER,
            "CONTROL_HZ": self.CONTROL_HZ,
            "MAX_STEP_MM": self.MAX_STEP_MM,
            **(speed_params or {}),
        })

    # ------------------------------------------------------------------
    # NGẮT KHẨN (nút DỪNG)
    # ------------------------------------------------------------------
    def request_abort(self):
        """Gọi từ luồng GUI: yêu cầu ngắt chuyển động đang chạy NGAY. Chu
        trình gắp-thả sẽ nhấc lên độ cao an toàn, về HOME và tắt bơm."""
        self._abort.set()

    def clear_abort(self):
        self._abort.clear()

    def _check_abort(self):
        if self._abort.is_set():
            raise MotionAborted()

    def _sleep(self, seconds):
        """time.sleep có thể bị ngắt bởi request_abort()."""
        end = time.perf_counter() + max(seconds, 0.0)
        while True:
            self._check_abort()
            remaining = end - time.perf_counter()
            if remaining <= 0:
                return
            time.sleep(min(remaining, 0.02))

    # ------------------------------------------------------------------
    # HOME / VỊ TRÍ
    # ------------------------------------------------------------------
    def set_home(self, x, y, z):
        """Đồng bộ HOME với vị trí đã lưu (ctx.home_pos)."""
        self.HOME = (float(x), float(y), float(z))

    def invalidate_position(self):
        """Robot đã bị di chuyển NGOÀI planner này -> không còn biết vị trí
        thật. Lần di chuyển kế tiếp sẽ gửi thẳng tới đích."""
        self.current_pos = None

    def _connection_ok(self):
        """Báo lỗi RÕ RÀNG khi Serial chưa kết nối, thay vì giả vờ thành công
        ở chế độ [MOVE-DRY] (robot không hề nhúc nhích)."""
        uart = self.uart
        if uart is None:
            return True  # chạy mô phỏng có chủ đích
        if getattr(uart, "dry_run", False):
            return True
        if not getattr(uart, "is_connected", False):
            print("[LỖI] Robot CHƯA kết nối Serial -> không gửi được lệnh. "
                  "Hãy kết nối ở tab Kết nối trước.")
            return False
        return True

    def _hold_target(self, x, y, z, duration_s):
        """Gửi lặp điểm (x, y, z) ở CONTROL_HZ trong duration_s giây."""
        n = max(int(duration_s * self.CONTROL_HZ), 1)
        for _ in range(n):
            self._check_abort()
            t0 = time.perf_counter()
            if not self.send_position(x, y, z):
                raise MotionError(f"Gửi lệnh tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
            remaining = self.CONTROL_DT - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)
        self.current_pos = (x, y, z)

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
        """Smoothstep bậc 5: vận tốc = 0 ở đầu và cuối đoạn."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển từ current_pos tới (x, y, z) trong wait_s giây, gửi lệnh
        ở CONTROL_HZ với S-curve. n_steps = max(theo thời gian, theo khoảng
        cách / MAX_STEP_MM) để quỹ đạo bám sát đường thẳng.

        current_pos is None -> gửi thẳng tới đích một lần rồi chờ wait_s.
        """
        target = (x, y, z)

        if self.current_pos is None:
            print("[MOVE] Không rõ vị trí hiện tại -> gửi thẳng tới đích.")
            if not self.send_position(x, y, z):
                raise MotionError(f"Di chuyển tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
            self._sleep(wait_s)
            self.current_pos = target
            return

        p1 = np.array(self.current_pos, dtype=float)
        p2 = np.array(target, dtype=float)

        distance = float(np.linalg.norm(p2 - p1))
        if distance < 1e-6:
            self._sleep(wait_s)
            self.current_pos = target
            return

        n_steps_time = max(int(wait_s * self.CONTROL_HZ), 1)
        n_steps_dist = max(int(math.ceil(distance / self.MAX_STEP_MM)), 1)
        n_steps = max(n_steps_time, n_steps_dist)

        for i in range(1, n_steps + 1):
            self._check_abort()
            t_start = time.perf_counter()

            frac = self._s_curve(i / n_steps)
            point = tuple(p1 + (p2 - p1) * frac)

            if not self.send_position(*point):
                raise MotionError(f"Di chuyển tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
            self.current_pos = point  # theo dõi vị trí thật từng bước

            elapsed = time.perf_counter() - t_start
            remaining = self.CONTROL_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

        self.current_pos = target

    def move_home(self):
        """Về HOME đã lưu. LUÔN gửi lệnh thật xuống robot:
          - biết vị trí hiện tại và khác HOME -> nội suy S-curve tới HOME;
          - không biết vị trí (None) hoặc đã ở HOME theo planner -> gửi lặp
            điểm HOME."""
        x, y, z = self.HOME
        print(f"--> Đang về Home {self.HOME} (planner nhớ vị trí: {self.current_pos})")
        if not self._connection_ok():
            return False
        try:
            if self.current_pos is not None:
                dist = float(np.linalg.norm(
                    np.array(self.current_pos, dtype=float) - np.array((x, y, z), dtype=float)))
                if dist > 1e-6:
                    self._move_or_raise(x, y, z, self.TIME_MOVE_FAST)
                self._hold_target(x, y, z, HOME_HOLD_S)
            else:
                self._hold_target(x, y, z, max(self.TIME_MOVE_FAST, 1.0))
        except MotionAborted:
            print("[DỪNG] Lệnh về Home bị ngắt.")
            return False
        except MotionError as e:
            print(f"[LỖI] Gửi lệnh về Home thất bại! Robot có thể vẫn đang ở vị trí cũ. ({e})")
            self.current_pos = None
            return False
        print("--> Đã gửi xong lệnh về Home.")
        return True

    def _try_safe_retreat(self, x, y):
        print("[AN TOÀN] Đang cố gắng nhấc lên độ cao an toàn và về Home sau lỗi...")
        try:
            self._move_or_raise(x, y, self.Z_SAFE, self.TIME_MOVE_DOWN)
        except Exception as e:
            print(f"[AN TOÀN] Không thể nhấc lên an toàn: {e}")
        self.move_home()

    def _abort_retreat(self, set_gripper):
        """Sau khi bị ngắt giữa chừng: nhấc lên độ cao an toàn tại chỗ, về
        HOME rồi TẮT bơm (nếu đang giữ vật, vật sẽ được nhả tại HOME)."""
        self._abort.clear()  # xóa cờ để các lệnh dưới đây chạy được
        try:
            cur = self.current_pos
            if cur is not None:
                cx, cy, cz = cur
                # Z lớn = thấp hơn: chỉ nhấc lên nếu đang thấp hơn Z_SAFE
                self._move_or_raise(cx, cy, min(cz, self.Z_SAFE), self.TIME_MOVE_DOWN)
        except Exception as e:
            print(f"[DỪNG] Không thể nhấc lên an toàn: {e}")
        self.move_home()
        try:
            set_gripper("off")
        except Exception as e:
            print(f"[CẢNH BÁO] Tắt bơm sau khi dừng thất bại: {e}")

    # ------------------------------------------------------------------
    # GẮP - THẢ (3 bậc, không xoay bậc 4)
    # ------------------------------------------------------------------
    def pick_and_place(self, point_a, point_b, z_pick=290, z_place=None,
                       gripper_callback=None):
        ax, ay = point_a
        bx, by = point_b
        z_safe = self.Z_SAFE
        if z_place is None:
            z_place = z_pick

        if gripper_callback is None:
            print("[CẢNH BÁO] Không có gripper_callback -> chỉ mô phỏng bơm (dry-run).")

        def set_gripper(state):
            if gripper_callback is None:
                print(f"[GRIPPER-DRY] {state.upper()}")
                return
            try:
                gripper_callback(state)
            except Exception as e:
                raise MotionError(f"Điều khiển gripper ('{state}') thất bại: {e}")

        if not self._connection_ok():
            raise MotionError("Robot chưa kết nối Serial.")

        print("=== BẮT ĐẦU QUY TRÌNH GẮP THẢ (3DOF) ===")

        try:
            # 1. Tới điểm A ở Z safe -> hạ xuống Z pick -> hút
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_FAST)
            self._move_or_raise(ax, ay, z_pick, self.TIME_MOVE_DOWN)

            print("[GRIPPER] Bật hút/kẹp vật")
            set_gripper("on")
            self._sleep(self.TIME_DELAY_GRIPPER)

            # 2. Nhấc lên Z safe -> sang B -> hạ xuống Z place -> nhả
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_DOWN)
            self._move_or_raise(bx, by, 290, self.TIME_MOVE_FAST)
            self._move_or_raise(bx, by, z_place, self.TIME_MOVE_DOWN)

            print("[GRIPPER] Tắt hút/nhả vật")
            set_gripper("off")
            self._sleep(self.TIME_DELAY_GRIPPER)

            # 3. Nhấc lên Z safe -> về Home
            self._move_or_raise(bx, by, 290, self.TIME_MOVE_DOWN)
            self.move_home()
            self._check_abort()
            print("=== HOÀN TẤT QUY TRÌNH GẮP THẢ (3DOF) ===\n")
            return True

        except MotionAborted:
            print("[DỪNG] Người dùng nhấn DỪNG -> nhấc lên, về HOME, tắt bơm.")
            self._abort_retreat(set_gripper)
            return False

        except MotionError as e:
            print(f"[LỖI QUY TRÌNH] {e}")
            self._try_safe_retreat(bx, by)
            raise