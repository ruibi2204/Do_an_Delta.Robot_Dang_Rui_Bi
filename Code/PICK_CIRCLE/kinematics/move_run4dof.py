import time
import math
import numpy as np
from kinematics.dhnghich import inverse_kinematics

# ==== Tốc độ CHUNG (dùng cho move_home() và _try_safe_retreat() - những
# đoạn di chuyển không thuộc riêng pha gắp hay pha thả) ====
TIME_MOVE_FAST = 0.3
TIME_MOVE_DOWN = 0.3
TIME_MOVE_ACTION = 0.3

# ==== Tốc độ RIÊNG cho pha GẮP (pick_dof4): điểm A -> hạ Z -> tiến sâu gắp.
# Mặc định lấy bằng giá trị chung ở trên để không phá vỡ hành vi cũ, nhưng
# giờ có thể chỉnh riêng qua speed_params/GUI mà không ảnh hưởng pha thả. ====
TIME_MOVE_FAST_PICK = 0.3
TIME_MOVE_DOWN_PICK = 0.55
TIME_MOVE_ACTION_PICK = 0.25

# ==== Tốc độ RIÊNG cho pha THẢ (place_dof4): điểm B -> hạ Z -> tiến sâu thả. ====
TIME_MOVE_FAST_PLACE = 0.3
TIME_MOVE_DOWN_PLACE = 0.3
TIME_MOVE_ACTION_PLACE = 0.3

TIME_DELAY_GRIPPER = 0.2
TIME_DELAY_DOF4 = 0.2   # Thời gian chờ bàn xoay/step quay xong

CONTROL_HZ = 100.0

# ==== (MỚI - làm thẳng quỹ đạo) ====
# Khoảng cách Cartesian tối đa (mm) cho phép giữa 2 điểm gửi liên tiếp.
# TRƯỚC ĐÂY: số bước nội suy (n_steps) chỉ phụ thuộc THỜI GIAN di chuyển
# (wait_s * CONTROL_HZ), không phụ thuộc khoảng cách. Với đoạn dài nhưng
# thời gian ngắn (vd TIME_MOVE_ACTION_PICK = 0.25s), số điểm gửi có thể
# rất ít -> khoảng cách giữa 2 điểm liên tiếp lớn. Vì driver/servo nội suy
# góc khớp giữa 2 lệnh liên tiếp theo KHÔNG GIAN KHỚP (không phải X-Y-Z),
# quỹ đạo thực tế bị cong ra khỏi đường thẳng lý tưởng dù nội suy trong
# phần mềm là tuyến tính hoàn toàn đúng trong Cartesian.
# SỬA: số bước thực tế = max(số bước theo thời gian, số bước theo khoảng
# cách = distance / MAX_STEP_MM). Đoạn nào dài sẽ tự chia nhỏ hơn (có thể
# mất nhiều hơn wait_s một chút), đảm bảo quỹ đạo luôn bám sát đường thẳng
# bất kể tốc độ đặt ra.
MAX_STEP_MM = 3.0

# Danh sách tên các tham số tốc độ mà GUI (khung "THAM SỐ TỐC ĐỘ") có thể
# ghi đè. Dùng chung một danh sách để tránh gõ nhầm tên khóa ở nhiều nơi.
# Đã thêm 6 khóa _PICK / _PLACE để GUI có thể chỉnh riêng từng pha; 3 khóa
# gốc (không hậu tố) vẫn giữ để chỉnh tốc độ move_home()/an toàn. Đã thêm
# MAX_STEP_MM để GUI/skill có thể tinh chỉnh độ mịn quỹ đạo nếu cần.
SPEED_PARAM_KEYS = (
    "TIME_MOVE_FAST",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_ACTION",
    "TIME_MOVE_FAST_PICK",
    "TIME_MOVE_DOWN_PICK",
    "TIME_MOVE_ACTION_PICK",
    "TIME_MOVE_FAST_PLACE",
    "TIME_MOVE_DOWN_PLACE",
    "TIME_MOVE_ACTION_PLACE",
    "TIME_DELAY_GRIPPER",
    "TIME_DELAY_DOF4",
    "CONTROL_HZ",
    "MAX_STEP_MM",
)


class MotionError(Exception):
    pass


class DeltaMotionPlanner:

    def __init__(self, uart_comm=None, speed_params=None, home_pos=None):
        """
        home_pos: tuple (x, y, z) tùy chọn - vị trí Home ĐÃ LƯU ở tab MANUAL
            (ctx.home_pos). Nếu không truyền, dùng giá trị mặc định cứng
            (60, 0, 300) để vẫn chạy được dry-run/test độc lập. Có thể cập
            nhật lại bất cứ lúc nào qua set_home(), ví dụ khi GUI vừa nạp
            xong ctx.home_pos hoặc người dùng vừa lưu lại Home mới.
        """
        self.uart = uart_comm
        self.HOME = tuple(home_pos) if home_pos is not None else (60.0, 0.0, 300.0)
        self.Z_SAFE = 295.0
        self.current_pos = self.HOME  # theo dõi vị trí hiện tại để nội suy đường thẳng

        # Khởi tạo các tham số tốc độ dạng thuộc tính riêng của instance,
        # để có thể đọc lại/ghi đè lúc chạy mà không đụng tới hằng số module.
        self._init_speed_params(speed_params)

    def _init_speed_params(self, speed_params):
        defaults = {
            "TIME_MOVE_FAST": TIME_MOVE_FAST,
            "TIME_MOVE_DOWN": TIME_MOVE_DOWN,
            "TIME_MOVE_ACTION": TIME_MOVE_ACTION,
            "TIME_MOVE_FAST_PICK": TIME_MOVE_FAST_PICK,
            "TIME_MOVE_DOWN_PICK": TIME_MOVE_DOWN_PICK,
            "TIME_MOVE_ACTION_PICK": TIME_MOVE_ACTION_PICK,
            "TIME_MOVE_FAST_PLACE": TIME_MOVE_FAST_PLACE,
            "TIME_MOVE_DOWN_PLACE": TIME_MOVE_DOWN_PLACE,
            "TIME_MOVE_ACTION_PLACE": TIME_MOVE_ACTION_PLACE,
            "TIME_DELAY_GRIPPER": TIME_DELAY_GRIPPER,
            "TIME_DELAY_DOF4": TIME_DELAY_DOF4,
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
                        pass  # giữ giá trị mặc định nếu dữ liệu không hợp lệ

        self.TIME_MOVE_FAST = defaults["TIME_MOVE_FAST"]
        self.TIME_MOVE_DOWN = defaults["TIME_MOVE_DOWN"]
        self.TIME_MOVE_ACTION = defaults["TIME_MOVE_ACTION"]
        self.TIME_MOVE_FAST_PICK = defaults["TIME_MOVE_FAST_PICK"]
        self.TIME_MOVE_DOWN_PICK = defaults["TIME_MOVE_DOWN_PICK"]
        self.TIME_MOVE_ACTION_PICK = defaults["TIME_MOVE_ACTION_PICK"]
        self.TIME_MOVE_FAST_PLACE = defaults["TIME_MOVE_FAST_PLACE"]
        self.TIME_MOVE_DOWN_PLACE = defaults["TIME_MOVE_DOWN_PLACE"]
        self.TIME_MOVE_ACTION_PLACE = defaults["TIME_MOVE_ACTION_PLACE"]
        self.TIME_DELAY_GRIPPER = defaults["TIME_DELAY_GRIPPER"]
        self.TIME_DELAY_DOF4 = defaults["TIME_DELAY_DOF4"]
        self.CONTROL_HZ = defaults["CONTROL_HZ"]
        self.CONTROL_DT = 1.0 / self.CONTROL_HZ
        self.MAX_STEP_MM = defaults["MAX_STEP_MM"]

    def update_speed_params(self, speed_params):

        self._init_speed_params({
            **{
                "TIME_MOVE_FAST": self.TIME_MOVE_FAST,
                "TIME_MOVE_DOWN": self.TIME_MOVE_DOWN,
                "TIME_MOVE_ACTION": self.TIME_MOVE_ACTION,
                "TIME_MOVE_FAST_PICK": self.TIME_MOVE_FAST_PICK,
                "TIME_MOVE_DOWN_PICK": self.TIME_MOVE_DOWN_PICK,
                "TIME_MOVE_ACTION_PICK": self.TIME_MOVE_ACTION_PICK,
                "TIME_MOVE_FAST_PLACE": self.TIME_MOVE_FAST_PLACE,
                "TIME_MOVE_DOWN_PLACE": self.TIME_MOVE_DOWN_PLACE,
                "TIME_MOVE_ACTION_PLACE": self.TIME_MOVE_ACTION_PLACE,
                "TIME_DELAY_GRIPPER": self.TIME_DELAY_GRIPPER,
                "TIME_DELAY_DOF4": self.TIME_DELAY_DOF4,
                "CONTROL_HZ": self.CONTROL_HZ,
                "MAX_STEP_MM": self.MAX_STEP_MM,
            },
            **(speed_params or {}),
        })

    def set_home(self, x, y, z):
        """Cập nhật lại HOME (ví dụ đồng bộ với ctx.home_pos đã lưu ở tab
        MANUAL). Gọi hàm này ở đầu on_start() của window - giống cách các
        window khác đang gán ctx.current_pos = list(ctx.home_pos) - để mọi
        lần move_home()/_try_safe_retreat() sau đó đưa robot về ĐÚNG vị trí
        Home thực tế thay vì tọa độ mặc định cứng (60, 0, 300)."""
        self.HOME = (float(x), float(y), float(z))

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

        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)  # 6t^5 - 15t^4 + 10t^3

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển từ self.current_pos tới (x, y, z) trong khoảng thời gian
        wait_s. Số bước nội suy (n_steps) lấy giá trị LỚN HƠN giữa:
          - số bước theo THỜI GIAN  (wait_s * CONTROL_HZ)
          - số bước theo KHOẢNG CÁCH (distance / MAX_STEP_MM)
        để đảm bảo khoảng cách Cartesian giữa 2 điểm gửi liên tiếp không
        bao giờ vượt quá MAX_STEP_MM -> quỹ đạo thực tế bám sát đường thẳng
        thay vì bị cong do nội suy góc khớp ở phía driver/servo, kể cả khi
        đoạn đường dài mà thời gian di chuyển ngắn."""
        p1 = np.array(self.current_pos, dtype=float)
        p2 = np.array((x, y, z), dtype=float)
        target = (x, y, z)

        distance = float(np.linalg.norm(p2 - p1))
        if distance < 1e-6:
            # Không di chuyển thực sự (ví dụ giữ Z), vẫn tôn trọng wait_s để
            # các bước tiếp theo (gripper, dof4...) không bị lệch timing.
            time.sleep(wait_s)
            self.current_pos = target
            return

        n_steps_time = max(int(wait_s * self.CONTROL_HZ), 1)
        n_steps_dist = max(int(math.ceil(distance / self.MAX_STEP_MM)), 1)
        n_steps = max(n_steps_time, n_steps_dist)

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
            self._move_or_raise(x, y, z, self.TIME_MOVE_FAST)
        except MotionError as e:
            print(f"[LỖI] Gửi lệnh về Home thất bại! Robot có thể vẫn đang ở vị trí cũ. ({e})")
            return False
        return True

    def _try_safe_retreat(self, x, y):
        print("[AN TOÀN] Đang cố gắng nhấc lên độ cao an toàn và về Home sau lỗi...")
        try:
            self._move_or_raise(x, y, self.Z_SAFE, self.TIME_MOVE_DOWN)
        except Exception as e:
            print(f"[AN TOÀN] Không thể nhấc lên an toàn: {e}")
        self.move_home()

    @staticmethod
    def _wrap_angle_180(angle_deg):
        """Đưa góc về [-90, 90) - hình chữ nhật đối xứng 180 độ."""
        a = angle_deg % 180.0
        if a >= 90.0:
            a -= 180.0
        return a

    def _rotate_dof4(self, rotate_callback, degrees):
        if rotate_callback is None or degrees is None:
            return
        if abs(degrees) < 0.01:
            return  # không cần xoay
        print(f"[DOF4] Xoay {degrees:.2f}°")
        try:
            rotate_callback(degrees)
        except Exception as e:
            print(f"[CẢNH BÁO] Xoay bậc tự do 4 thất bại: {e}")
        time.sleep(self.TIME_DELAY_DOF4)

    @staticmethod
    def _make_set_gripper(gripper_callback):
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

        return set_gripper

    def pick_dof4(self, point_a, z_pick=290, gripper_callback=None):
        ax, ay = point_a
        z_action_pick = z_pick + 12
        set_gripper = self._make_set_gripper(gripper_callback)

        print("=== BẮT ĐẦU GẮP VẬT (chưa thả - chờ khung) ===")
        try:
            # 1. Đến điểm A trên cao -> hạ Z_pick -> tiến sâu gắp
            #    (dùng bộ tốc độ RIÊNG cho pha gắp: *_PICK)
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_FAST_PICK)
            self._move_or_raise(ax, ay, z_pick, self.TIME_MOVE_DOWN_PICK)
            self._move_or_raise(ax, ay, z_action_pick, self.TIME_MOVE_ACTION_PICK)

            print("[GRIPPER] Bật hút/kẹp vật")
            set_gripper("on")
            time.sleep(self.TIME_DELAY_GRIPPER)

            # 2. Nhấc lên an toàn -> về Home, VẪN GIỮ vật (không xoay,
            #    không thả) - chờ lệnh place_dof4() sau khi thấy khung.
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_DOWN_PICK)
            self.move_home()
            print("=== ĐÃ GẮP VẬT, ĐANG GIỮ TẠI HOME - CHỜ THẤY KHUNG ===\n")
            return True

        except MotionError as e:
            print(f"[LỖI GẮP VẬT] {e}")
            self._try_safe_retreat(ax, ay)
            raise

    def place_dof4(self, place_point, z_pick=290, gripper_callback=None,
                    rotate_callback=None, object_angle_deg=None, target_angle_deg=90.0):
        bx, by = place_point
        z_action_place = z_pick + 10
        set_gripper = self._make_set_gripper(gripper_callback)

        rotation_needed = None
        if object_angle_deg is not None:
            rotation_needed = self._wrap_angle_180(target_angle_deg - object_angle_deg)

        print("=== BẮT ĐẦU THẢ VẬT VÀO KHUNG ===")
        if rotation_needed is not None:
            print(f"[DOF4] Góc vật lúc gắp: {object_angle_deg:.2f}° "
                  f"-> cần xoay {rotation_needed:.2f}° để đạt {target_angle_deg:.1f}°")

        try:
            # 1. XOAY bậc tự do 4 (vật đang được giữ tại Home) cho khớp góc khung
            self._rotate_dof4(rotate_callback, rotation_needed)

            # 2. Sang điểm thả -> hạ xuống -> nhả vật
            #    (dùng bộ tốc độ RIÊNG cho pha thả: *_PLACE)
            self._move_or_raise(bx, by, 290, self.TIME_MOVE_FAST_PLACE)
            self._move_or_raise(bx, by, z_pick, self.TIME_MOVE_DOWN_PLACE)
            self._move_or_raise(bx, by, z_action_place, self.TIME_MOVE_ACTION_PLACE)

            print("[GRIPPER] Tắt hút/nhả vật")
            set_gripper("off")
            time.sleep(self.TIME_DELAY_GRIPPER)

            # 3. Nhấc lên an toàn -> XOAY NGƯỢC LẠI để reset bậc tự do 4 -> về Home
            self._move_or_raise(bx, by, 290, self.TIME_MOVE_DOWN_PLACE)
            if rotation_needed is not None:
                self._rotate_dof4(rotate_callback, -rotation_needed)

            self.move_home()
            print("=== HOÀN TẤT THẢ VẬT VÀO KHUNG ===\n")
            return True

        except MotionError as e:
            print(f"[LỖI THẢ VẬT] {e}")
            self._try_safe_retreat(bx, by)
            raise

    def pick_and_place_dof4(self, point_a, z_pick=290, gripper_callback=None,
                             rotate_callback=None, object_angle_deg=None,
                             place_point=(0.0, 0.0), target_angle_deg=90.0):
        self.pick_dof4(point_a, z_pick=z_pick, gripper_callback=gripper_callback)
        return self.place_dof4(
            place_point, z_pick=z_pick, gripper_callback=gripper_callback,
            rotate_callback=rotate_callback, object_angle_deg=object_angle_deg,
            target_angle_deg=target_angle_deg,
        )

    def pick_and_place(self, point_a, point_b, z_pick=290, gripper_callback=None):
        """Bản KHÔNG có bậc tự do 4 - giữ nguyên y hệt move_delta_4dof.py gốc,
        dùng khi không cần xoay (ví dụ chế độ Cameracircle.py cũ)."""
        return self.pick_and_place_dof4(
            point_a, z_pick=z_pick, gripper_callback=gripper_callback,
            rotate_callback=None, object_angle_deg=None, place_point=point_b,
        )