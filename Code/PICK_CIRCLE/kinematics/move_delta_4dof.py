import time
import math
import numpy as np
from kinematics.dhnghich import inverse_kinematics

# =====================================================================
# LƯU Ý (kế thừa từ move_delta_4dof.py bản an toàn):
# File này KHÔNG import Uart_2 trực tiếp. Mọi điều khiển bơm và bậc tự do
# 4 (bàn xoay) đều đi qua callback do nơi gọi (GUI) truyền vào, để chỉ có
# DUY NHẤT một kết nối UART2 (do GUI quản lý) trong toàn ứng dụng - tránh
# lỗi "Access is denied" do mở trùng cổng COM.
# =====================================================================
#
# CẬP NHẬT (làm thẳng quỹ đạo):
# Trước đây số điểm nội suy gửi đi (n_steps) chỉ phụ thuộc THỜI GIAN di
# chuyển (wait_s * CONTROL_HZ). Với đoạn đường DÀI nhưng thời gian NGẮN
# (ví dụ TIME_MOVE_ACTION = 0.25s), số điểm gửi có thể rất ít -> khoảng
# cách Cartesian giữa 2 điểm liên tiếp lớn. Vì driver/servo nội suy góc
# khớp giữa 2 lệnh liên tiếp theo KHÔNG GIAN KHỚP (không phải không gian
# X-Y-Z), quỹ đạo thực tế bị cong ra khỏi đường thẳng lý tưởng dù thuật
# toán nội suy trong phần mềm là tuyến tính hoàn toàn đúng trong Cartesian.
#
# Cách khắc phục: thêm giới hạn MAX_STEP_MM - khoảng cách Cartesian tối đa
# cho phép giữa 2 điểm gửi liên tiếp. Số bước thực tế = max(số bước theo
# thời gian, số bước theo khoảng cách). Nhờ vậy các đoạn dài sẽ tự động
# được chia nhỏ hơn (và có thể mất nhiều hơn wait_s một chút), đảm bảo
# quỹ đạo luôn bám sát đường thẳng bất kể tốc độ đặt ra.
# =====================================================================

# ---------------------------------------------------------------------
# GIÁ TRỊ MẶC ĐỊNH của các tham số thời gian/tốc độ. Đây chỉ còn là giá
# trị dự phòng (fallback) khi không có tham số nào được truyền vào từ
# GUI (khung "THAM SỐ TỐC ĐỘ" trong main.py). DeltaMotionPlanner giờ đọc
# các giá trị này thành THUỘC TÍNH RIÊNG của từng instance
# (self.TIME_MOVE_FAST, ...), cho phép GUI ghi đè lúc khởi tạo hoặc lúc
# chạy (qua update_speed_params) mà không cần sửa code.
# ---------------------------------------------------------------------
TIME_MOVE_FAST = 1
TIME_MOVE_DOWN = 0.5
TIME_MOVE_ACTION = 0.5
TIME_DELAY_GRIPPER = 0.5
TIME_DELAY_DOF4 = 0.5   # Thời gian chờ bàn xoay/step quay xong

# Tần số gửi lệnh điều khiển (Hz) — CỐ ĐỊNH bất kể đoạn dài hay ngắn.
# Đây là điểm khác biệt chính so với bản step_mm: số điểm gửi phụ thuộc
# THỜI GIAN di chuyển (wait_s), không phụ thuộc khoảng cách -> không còn
# tình trạng đoạn dài bị dồn quá nhiều lệnh trong thời gian ngắn.
CONTROL_HZ = 100.0

# Khoảng cách Cartesian tối đa (mm) cho phép giữa 2 điểm gửi liên tiếp.
# Đây là tham số MỚI để làm thẳng quỹ đạo: nếu đoạn di chuyển dài mà thời
# gian ngắn khiến khoảng cách giữa 2 điểm > giá trị này, số bước sẽ tự
# động tăng lên (bất kể wait_s) để đảm bảo độ mịn/thẳng của đường đi.
MAX_STEP_MM = 3.0

# Danh sách tên các tham số tốc độ mà GUI (khung "THAM SỐ TỐC ĐỘ") có thể
# ghi đè. Dùng chung một danh sách để tránh gõ nhầm tên khóa ở nhiều nơi.
SPEED_PARAM_KEYS = (
    "TIME_MOVE_FAST",
    "TIME_MOVE_DOWN",
    "TIME_MOVE_ACTION",
    "TIME_DELAY_GRIPPER",
    "TIME_DELAY_DOF4",
    "CONTROL_HZ",
    "MAX_STEP_MM",
)


class MotionError(Exception):
    pass


class DeltaMotionPlanner:

    def __init__(self, uart_comm=None, speed_params=None):
        """
        uart_comm: đối tượng UART2 do GUI quản lý (hoặc None để chạy dry-run).
        speed_params: dict tùy chọn chứa các tham số tốc độ lấy từ khung
            "THAM SỐ TỐC ĐỘ" trên giao diện (ctx.speed_params), gồm các khóa
            trong SPEED_PARAM_KEYS. Khóa nào không có trong dict sẽ dùng giá
            trị mặc định của module (các hằng số ở đầu file).
        """
        self.uart = uart_comm
        self.HOME = (60.0, 0.0, 300.0)
        self.Z_SAFE = 290.0
        # LƯU Ý: self.HOME chỉ là giá trị MẶC ĐỊNH cứng. Nơi gọi (GUI) nên
        # gọi set_home(*ctx.home_pos) khi bắt đầu 1 phiên chạy để move_home()
        # đưa robot về đúng vị trí Home ĐÃ LƯU ở tab MANUAL thay vì tọa độ
        # cố định này - xem set_home() bên dưới.
        self.current_pos = self.HOME  # theo dõi vị trí hiện tại để nội suy đường thẳng

        # Khởi tạo các tham số tốc độ dạng thuộc tính riêng của instance,
        # để có thể đọc lại/ghi đè lúc chạy mà không đụng tới hằng số module.
        self._init_speed_params(speed_params)

    def _init_speed_params(self, speed_params):
        defaults = {
            "TIME_MOVE_FAST": TIME_MOVE_FAST,
            "TIME_MOVE_DOWN": TIME_MOVE_DOWN,
            "TIME_MOVE_ACTION": TIME_MOVE_ACTION,
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
        self.TIME_DELAY_GRIPPER = defaults["TIME_DELAY_GRIPPER"]
        self.TIME_DELAY_DOF4 = defaults["TIME_DELAY_DOF4"]
        self.CONTROL_HZ = defaults["CONTROL_HZ"]
        self.CONTROL_DT = 1.0 / self.CONTROL_HZ
        self.MAX_STEP_MM = defaults["MAX_STEP_MM"]

    def update_speed_params(self, speed_params):
        """Cho phép GUI cập nhật lại tham số tốc độ ngay cả khi
        DeltaMotionPlanner đã được tạo (ví dụ sau khi người dùng bấm
        "Lưu tham số" ở khung THAM SỐ TỐC ĐỘ, hoặc kéo thanh trượt tốc độ).
        Chỉ những khóa hợp lệ (số thực > 0) mới được áp dụng, các khóa còn
        lại giữ nguyên."""
        self._init_speed_params({
            **{
                "TIME_MOVE_FAST": self.TIME_MOVE_FAST,
                "TIME_MOVE_DOWN": self.TIME_MOVE_DOWN,
                "TIME_MOVE_ACTION": self.TIME_MOVE_ACTION,
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
        """Đường cong S (smoothstep bậc 5) cho tỉ lệ tiến trình t_frac trong [0,1].
        Cho vận tốc = 0 tại đầu và cuối đoạn, tăng/giảm tốc êm ở giữa -> khử
        cảm giác giật/khựng khi bắt đầu và kết thúc mỗi đoạn di chuyển."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)  # 6t^5 - 15t^4 + 10t^3

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển từ self.current_pos tới (x, y, z) trong khoảng thời gian
        wait_s, gửi lệnh với tần số cố định self.CONTROL_HZ và áp dụng S-curve
        để chuyển động mượt, không giật cục dù đoạn ngắn hay dài.

        Số bước nội suy (n_steps) được lấy theo giá trị LỚN HƠN giữa:
          - số bước theo THỜI GIAN  (wait_s * CONTROL_HZ)
          - số bước theo KHOẢNG CÁCH (distance / MAX_STEP_MM)
        Điều này đảm bảo khoảng cách Cartesian giữa 2 điểm gửi liên tiếp
        không bao giờ vượt quá MAX_STEP_MM, kể cả khi đoạn đường dài mà
        thời gian di chuyển ngắn -> quỹ đạo thực tế bám sát đường thẳng
        thay vì bị cong do nội suy góc khớp ở phía driver/servo.
        """
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

            # Bù thời gian xử lý (tính toán + gửi UART) vào chu kỳ điều khiển,
            # để tần số gửi lệnh thực tế bám sát self.CONTROL_HZ, tránh chuyển
            # động bị chậm/lệch khi thời gian gửi UART không phải 0.
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

    def pick_and_place_dof4(self, point_a, z_pick=290, gripper_callback=None,
                             rotate_callback=None, object_angle_deg=None,
                             place_point=(0.0, 0.0), target_angle_deg=90.0):

        ax, ay = point_a
        bx, by = place_point
        z_action_pick = z_pick + 10
        z_action_place = z_pick + 5

        rotation_needed = None
        if object_angle_deg is not None:
            rotation_needed = self._wrap_angle_180(target_angle_deg - object_angle_deg)

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

        print("=== BẮT ĐẦU QUY TRÌNH GẮP THẢ (4DOF) ===")
        if rotation_needed is not None:
            print(f"[DOF4] Góc vật hiện tại: {object_angle_deg:.2f}° "
                  f"-> cần xoay {rotation_needed:.2f}° để đạt {target_angle_deg:.1f}°")

        try:
            # 1. Đến điểm A trên cao -> hạ Z_pick -> tiến sâu gắp
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_FAST)
            self._move_or_raise(ax, ay, z_pick, self.TIME_MOVE_DOWN)
            self._move_or_raise(ax, ay, z_action_pick, self.TIME_MOVE_ACTION)

            print("[GRIPPER] Bật hút/kẹp vật")
            set_gripper("on")
            time.sleep(self.TIME_DELAY_GRIPPER)

            # 2. Nhấc lên cao -> XOAY bậc tự do 4 (vật đã được giữ) -> sang điểm thả cố định
            self._move_or_raise(ax, ay, 290, self.TIME_MOVE_DOWN)
            self._rotate_dof4(rotate_callback, rotation_needed)

            self._move_or_raise(bx, by, 290, self.TIME_MOVE_FAST)
            self._move_or_raise(bx, by, z_pick, self.TIME_MOVE_DOWN)
            self._move_or_raise(bx, by, z_action_place, self.TIME_MOVE_ACTION)

            print("[GRIPPER] Tắt hút/nhả vật")
            set_gripper("off")
            time.sleep(self.TIME_DELAY_GRIPPER)

            # 3. Nhấc lên an toàn -> XOAY NGƯỢC LẠI để reset bậc tự do 4 -> về Home
            self._move_or_raise(bx, by, 290, self.TIME_MOVE_DOWN)
            if rotation_needed is not None:
                self._rotate_dof4(rotate_callback, -rotation_needed)

            self.move_home()
            print("=== HOÀN TẤT QUY TRÌNH GẮP THẢ (4DOF) ===\n")
            return True

        except MotionError as e:
            print(f"[LỖI QUY TRÌNH] {e}")
            self._try_safe_retreat(bx, by)
            raise

    def pick_and_place(self, point_a, point_b, z_pick=290, gripper_callback=None):
        """Bản KHÔNG có bậc tự do 4 - giữ nguyên y hệt move_delta_4dof.py gốc,
        dùng khi không cần xoay (ví dụ chế độ Cameracircle.py cũ)."""
        return self.pick_and_place_dof4(
            point_a, z_pick=z_pick, gripper_callback=gripper_callback,
            rotate_callback=None, object_angle_deg=None, place_point=point_b,
        )