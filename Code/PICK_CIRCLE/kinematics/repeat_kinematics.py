import time
import numpy as np

from kinematics.dhnghich import inverse_kinematics


# ---------------------------------------------------------------------
# THAM SỐ TỐC ĐỘ / TẦN SỐ ĐIỀU KHIỂN - RIÊNG cho bài test độ lặp lại.
# Đây chỉ là giá trị mặc định (fallback); có thể ghi đè qua speed_params
# khi khởi tạo RepeatKinematicsPlanner hoặc qua update_speed_params().
# ---------------------------------------------------------------------
CONTROL_HZ = 200.0          # Tần số gửi lệnh điều khiển (Hz) - CỐ ĐỊNH

TIME_MOVE_XY = 2          # Thời gian di chuyển ngang tới điểm (ở Z an toàn)
TIME_MOVE_DOWN = 1      # Thời gian hạ từ Z an toàn xuống z_pick
TIME_MOVE_DIP = 1        # Thời gian hạ thêm +2mm (chạm sâu)
TIME_MOVE_LIFT = 1       # Thời gian nâng lại -2mm (về z_pick)
TIME_MOVE_UP = 1         # Thời gian nâng từ z_pick về lại Z an toàn

DIP_MM = 10.0                # Độ chạm sâu thêm (mm) theo yêu cầu bài test

# Giới hạn hệ số tốc độ (speed_scale) áp dụng cho thanh trượt tốc độ trên GUI.
# speed_scale = 1.0  -> dùng đúng các TIME_MOVE_* mặc định/đã cấu hình.
# speed_scale > 1.0  -> di chuyển NHANH hơn (thời gian mỗi đoạn rút ngắn lại).
# speed_scale < 1.0  -> di chuyển CHẬM hơn (thời gian mỗi đoạn kéo dài ra).
SPEED_SCALE_MIN = 0.2
SPEED_SCALE_MAX = 3.0

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

    Ghi chú làm mượt chuyển động (chống rung):
        - Mỗi đoạn di chuyển dùng S-curve bậc 5 (vận tốc VÀ gia tốc đều = 0 ở
          đầu/cuối đoạn) để không có thay đổi vận tốc đột ngột.
        - Điểm nội suy được tính theo THỜI GIAN THỰC ĐÃ TRÔI QUA (elapsed
          time) so với tổng thời lượng đoạn di chuyển, KHÔNG tính theo số thứ
          tự bước cố định. Nhờ vậy, nếu UART gửi lệnh bị trễ (jitter) ở 1 vài
          bước, quỹ đạo vẫn bám đúng theo thời gian thực và không bị "dồn
          tốc" (tăng tốc đột ngột để đuổi kịp) - đây là nguyên nhân chính gây
          rung/giật trước đây khi độ trễ UART không đều.
        - Lịch gửi lệnh dùng mốc thời gian tuyệt đối (t0 + step * dt) thay vì
          cộng dồn sleep từng bước, nên không bị trôi (drift) tần số theo
          thời gian.

    Ghi chú vị trí HOME (MỚI - sửa theo yêu cầu):
        - TRƯỚC ĐÂY: self.HOME bị hard-code cứng = (60.0, 0.0, 300.0), hoàn
          toàn không liên quan tới vị trí Home mà người dùng đã hiệu chỉnh/
          lưu ở tab MANUAL (ctx.home_pos). Hậu quả: move_home() (gọi trong
          touch_point()/repeat_cycle()/_try_safe_retreat()) luôn đưa robot
          về đúng tọa độ cố định đó, dù Home vật lý thực tế đã được lưu khác
          đi -> bài test độ lặp lại chạy sai điểm neo giữa 2 lần chạm.
        - GIỜ: constructor nhận thêm home_pos tùy chọn, và có thêm
          set_home(x, y, z) để nơi gọi (RepeatTestWindow) đồng bộ lại HOME
          với ctx.home_pos - y hệt cách dynamic_window.py/dynamic_run4dof_
          window.py đang làm (ctx.current_pos = list(ctx.home_pos) khi bấm
          START). Nếu không truyền/không gọi, vẫn dùng mặc định cứng cũ để
          không phá vỡ hành vi dry-run/test độc lập.
    """

    def __init__(self, uart_comm=None, speed_params=None, home_pos=None):
        """
        uart_comm: đối tượng UART robot do GUI quản lý (hoặc None -> dry-run).
        speed_params: dict tùy chọn các khóa trong SPEED_PARAM_KEYS để GUI
            ghi đè tốc độ/tần số ngay lúc khởi tạo.
        home_pos: tuple (x, y, z) tùy chọn - vị trí Home ĐÃ LƯU ở tab MANUAL
            (ctx.home_pos). Nếu không truyền, dùng giá trị mặc định cứng
            (60, 0, 300). Có thể cập nhật lại bất cứ lúc nào qua set_home().
        """
        self.uart = uart_comm
        self.HOME = tuple(home_pos) if home_pos is not None else (0.0, 0.0, 280.0)
        self.Z_SAFE = 280.0
        self.current_pos = self.HOME  # theo dõi vị trí hiện tại để nội suy đường thẳng

        # Hệ số tốc độ điều chỉnh được TRỰC TIẾP từ thanh trượt trên GUI, có
        # thể thay đổi ngay cả khi bài test đang chạy (áp dụng cho đoạn di
        # chuyển tiếp theo).
        self.speed_scale = 1.0

        self._init_speed_params(speed_params)

    def set_home(self, x, y, z):
        """Cập nhật lại HOME (ví dụ đồng bộ với ctx.home_pos đã lưu ở tab
        MANUAL). Gọi hàm này ở đầu on_start() của RepeatTestWindow - giống
        cách các window khác đang gán ctx.current_pos = list(ctx.home_pos) -
        để move_home()/_try_safe_retreat() sau đó đưa robot về ĐÚNG vị trí
        Home thực tế thay vì tọa độ mặc định cứng (60, 0, 300)."""
        self.HOME = (float(x), float(y), float(z))

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

    def set_speed_scale(self, scale):
        """Đặt hệ số tốc độ (dùng cho thanh trượt TỐC ĐỘ trên GUI).

        scale = 1.0  : tốc độ mặc định (dùng đúng TIME_MOVE_* hiện tại)
        scale > 1.0  : nhanh hơn (ví dụ 2.0 = nhanh gấp đôi)
        scale < 1.0  : chậm hơn (ví dụ 0.5 = chậm một nửa)

        An toàn khi gọi trong lúc bài test đang chạy ở thread khác - chỉ là
        1 phép gán số thực đơn giản, áp dụng cho đoạn di chuyển kế tiếp.
        """
        try:
            scale = float(scale)
        except (TypeError, ValueError):
            return
        self.speed_scale = min(max(scale, SPEED_SCALE_MIN), SPEED_SCALE_MAX)

    def _dt(self, base_seconds):
        """Thời gian thực tế (giây) cho 1 đoạn di chuyển, đã áp dụng
        speed_scale. Luôn giữ tối thiểu vài chu kỳ điều khiển để S-curve có
        đủ điểm nội suy (tránh giật do quá ít bước)."""
        scaled = base_seconds / max(self.speed_scale, 1e-3)
        return max(scaled, self.CONTROL_DT * 4)

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
        """Smoothstep bậc 5 (6t^5 - 15t^4 + 10t^3): vận tốc VÀ gia tốc đều
        = 0 ở đầu/cuối đoạn di chuyển -> chuyển động êm, không giật cục."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)

    def _move_or_raise(self, x, y, z, wait_s):
        """Di chuyển từ self.current_pos tới (x, y, z), áp dụng speed_scale
        lên wait_s, gửi lệnh với tần số CỐ ĐỊNH self.CONTROL_HZ, dùng
        S-curve theo THỜI GIAN THỰC để mượt và không trôi theo độ trễ UART."""
        p1 = np.array(self.current_pos, dtype=float)
        p2 = np.array((x, y, z), dtype=float)
        target = (x, y, z)

        duration = self._dt(wait_s)

        if np.linalg.norm(p2 - p1) < 1e-6:
            time.sleep(duration)
            self.current_pos = target
            return

        t0 = time.perf_counter()
        step = 0

        while True:
            step += 1
            # Mốc thời gian tuyệt đối tính từ t0 (KHÔNG cộng dồn sleep từng
            # bước) -> không bị trôi tần số dù có trễ giữa chừng.
            target_time = t0 + step * self.CONTROL_DT
            elapsed_at_send = min(target_time - t0, duration)
            frac_time = elapsed_at_send / duration
            finished = frac_time >= 1.0

            frac = self._s_curve(frac_time)
            point = tuple(p1 + (p2 - p1) * frac)

            if not self.send_position(*point):
                raise MotionError(f"Di chuyển tới ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")

            if finished:
                break

            # Ngủ tới đúng mốc thời gian kế tiếp trên lưới cố định. Nếu đã bị
            # trễ so với mốc (UART chậm), KHÔNG cố "đuổi kịp" bằng cách tăng
            # tốc đột ngột ở bước sau - cứ gửi ngay lệnh kế tiếp và để frac
            # (tính theo thời gian thực) tự phản ánh đúng vị trí cần có tại
            # thời điểm đó. Đây chính là điểm sửa để hết rung do trễ UART.
            now = time.perf_counter()
            sleep_time = target_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

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
            self._move_or_raise(x, y, 280, self.TIME_MOVE_XY)

            # 2. Hạ xuống z_pick
            self._move_or_raise(x, y, z_pick, self.TIME_MOVE_DOWN)

            # 3. Hạ thêm +2mm (chạm sâu để test độ lặp lại)
            self._move_or_raise(x, y, z_dip, self.TIME_MOVE_DIP)

            # 4. Nâng lên -2mm (quay lại đúng z_pick)
            self._move_or_raise(x, y, z_pick, self.TIME_MOVE_LIFT)

            # 5. Nâng lên lại Z an toàn
            self._move_or_raise(x, y, 280, self.TIME_MOVE_UP)

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