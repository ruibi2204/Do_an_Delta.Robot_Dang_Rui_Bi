"""
kinematics/board_points_motion.py
=================================

Quỹ đạo chuyển động RIÊNG, TÁCH BIỆT HOÀN TOÀN cho phần "4) Run Robot (CSV)"
của giao diện HIỆU CHỈNH CAMERA BẰNG BÀN CỜ (windows/board_calib_window.py).

Không dùng chung logic nội suy với DeltaMotionPlanner._move_or_raise (planner
gắp-thả chính) hay với các module quỹ đạo khác (kinematics/draw_motion.py,
kinematics/repeat_kinematics.py, kinematics/manual_kinematic.py) - chỉ tái sử
dụng planner.send_position() (để gửi lệnh UART), planner.current_pos (để
biết vị trí hiện tại) và planner.HOME (vị trí Home đã lưu trong config).

QUY TRÌNH cho MỖI điểm (x_mm, y_mm) trong CSV:
    1) Di chuyển THẲNG (X, Y cùng lúc, Z giữ nguyên = z_travel) từ vị trí
       hiện tại tới (x_mm, y_mm, z_travel).
    2) Hạ xuống nhẹ một đoạn (z_down, "+" là xuống) - chỉ trục Z thay đổi.
    3) Dừng lại (dwell_down_s giây) ở đáy.
    4) Nâng trở lại lên z_travel.
    5) Về VỊ TRÍ HOME ĐÃ LƯU (planner.HOME) bằng một đường DI CHUYỂN THẲNG
       THÔNG THƯỜNG (y hệt bước 1/2/4, cùng S-curve mượt) - KHÔNG gửi lệnh
       "HOME" vật lý, KHÔNG chờ chạm công tắc hành trình, KHÔNG chờ STM32
       phản hồi READY. Đây chỉ là 1 điểm đích toạ độ như mọi điểm khác.

LÀM MƯỢT / KHÔNG BỊ CONG:
    - Mỗi đoạn di chuyển (bước 1, 2, 4, và 5 - về Home) đều là TỔ HỢP LỒI
      tuyến tính giữa điểm đầu và điểm cuối: point = p1 + (p2 - p1) * s.
      Vì X, Y, Z đều nhân cùng 1 hệ số s tại mọi thời điểm, quỹ đạo LUÔN
      nằm đúng trên đoạn thẳng nối 2 điểm - không thể bị cong lệch.
    - s được lấy từ S-curve bậc 5 (smoothstep: 6s^5-15s^4+10s^3), có vận
      tốc VÀ gia tốc = 0 ở đầu/cuối mỗi đoạn -> vào/ra chuyển động êm.
    - Lịch gửi lệnh theo tần số cố định CONTROL_HZ, dùng mốc thời gian
      TUYỆT ĐỐI (t0 + step*dt) và s tính theo THỜI GIAN THỰC ĐÃ TRÔI QUA.

TỐC ĐỘ:
    - speed_mm_s điều khiển tốc độ di chuyển ngang tại bước 1 (và cả tốc độ
      hạ/nâng ở bước 2, 4, và tốc độ về Home ở bước 5), có thể chỉnh trực
      tiếp qua thanh trượt trên GUI bằng set_speed() - an toàn để gọi ngay
      cả khi đang chạy.
"""

import math
import time


class MotionError(Exception):
    pass


class BoardPointsMotionPlanner:
    MIN_SPEED_MM_S = 0.5
    DEFAULT_SPEED_MM_S = 30.0

    def __init__(self, planner, robot_uart, control_hz=100.0):
        """
        planner    : DeltaMotionPlanner dùng chung (ctx.planner). CHỈ dùng
                     planner.send_position(x, y, z), planner.current_pos và
                     planner.HOME (vị trí Home đã lưu) - KHÔNG gọi
                     planner._move_or_raise (tách biệt hoàn toàn nội suy
                     chuyển động khỏi planner gắp-thả chính).
        robot_uart : đối tượng UART robot (ctx.robot_uart) - GIỮ LẠI tham
                     số này để không phải sửa nơi khởi tạo ở GUI, nhưng
                     KHÔNG còn dùng để chờ READY/Home vật lý nữa (chỉ
                     planner.send_position() mới thực sự gửi UART).
        control_hz : tần số gửi lệnh (Hz) RIÊNG của module này.
        """
        self.planner = planner
        self.robot_uart = robot_uart
        self.CONTROL_HZ = control_hz
        self.CONTROL_DT = 1.0 / control_hz
        self.speed_mm_s = self.DEFAULT_SPEED_MM_S
        self._stop_flag = False

    # ------------------------------------------------------------------
    def set_speed(self, speed_mm_s):
        """Đặt tốc độ di chuyển (mm/s) - gọi từ thanh trượt tốc độ trên GUI.
        An toàn khi gọi trong lúc đang chạy (thread khác)."""
        try:
            speed_mm_s = float(speed_mm_s)
        except (TypeError, ValueError):
            return
        self.speed_mm_s = max(speed_mm_s, self.MIN_SPEED_MM_S)

    def stop(self):
        """Yêu cầu dừng ngay khi có thể (sau bước di chuyển hiện tại)."""
        self._stop_flag = True

    def reset_stop(self):
        self._stop_flag = False

    # ------------------------------------------------------------------
    @staticmethod
    def _s_curve(t_frac):
        """Smoothstep bậc 5: vận tốc và gia tốc = 0 ở đầu/cuối đoạn."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)

    def _send_or_raise(self, x, y, z):
        if not self.planner.send_position(x, y, z):
            raise MotionError(f"Gửi tọa độ ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
        self.planner.current_pos = (x, y, z)

    def _walk_linear(self, target, duration):
        """Di chuyển THẲNG (nội suy tuyến tính) từ planner.current_pos tới
        `target` = (x, y, z) trong `duration` giây, dùng S-curve theo thời
        gian thực. Trả về True nếu đi hết đoạn, False nếu bị dừng giữa
        chừng (STOP)."""
        p1 = tuple(self.planner.current_pos)
        p2 = tuple(target)
        dist = math.dist(p1, p2)

        if dist < 1e-6:
            time.sleep(max(duration, 0.0))
            self.planner.current_pos = p2
            return True

        duration = max(duration, self.CONTROL_DT * 4)
        t0 = time.perf_counter()
        step = 0

        while True:
            if self._stop_flag:
                return False

            step += 1
            target_time = t0 + step * self.CONTROL_DT
            elapsed = min(target_time - t0, duration)
            frac_time = elapsed / duration
            finished = frac_time >= 1.0
            s = self._s_curve(frac_time)

            x = p1[0] + (p2[0] - p1[0]) * s
            y = p1[1] + (p2[1] - p1[1]) * s
            z = p1[2] + (p2[2] - p1[2]) * s
            self._send_or_raise(x, y, z)

            if finished:
                return True

            now = time.perf_counter()
            sleep_time = target_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _return_to_saved_home(self, speed):
        """Về VỊ TRÍ HOME ĐÃ LƯU (planner.HOME) bằng di chuyển thẳng bình
        thường (không chờ READY, không chạm công tắc hành trình). Trả về
        True nếu đi hết đoạn, False nếu bị dừng giữa chừng (STOP)."""
        home_pos = tuple(self.planner.HOME)
        dist_home = math.dist(self.planner.current_pos, home_pos)
        return self._walk_linear(home_pos, dist_home / speed)

    # ------------------------------------------------------------------
    def visit_point(self, x_mm, y_mm, z_travel, z_down=0.0, dwell_down_s=1.0):
        """
        Thực hiện trọn vẹn quy trình 5 bước cho 1 điểm (x_mm, y_mm):
            1) Đi thẳng, mượt tới (x_mm, y_mm, z_travel).
            2) Hạ xuống nhẹ z_down (nếu > 0).
            3) Dừng dwell_down_s giây.
            4) Nâng lại lên z_travel.
            5) Về VỊ TRÍ HOME ĐÃ LƯU (planner.HOME) - di chuyển thẳng bình
               thường, KHÔNG chờ READY từ STM32.

        Trả về True nếu hoàn tất trọn vẹn cả 5 bước, False nếu bị dừng
        giữa chừng (STOP) ở bất kỳ bước nào (kể cả bước 5).
        """
        self.reset_stop()
        speed = max(self.speed_mm_s, self.MIN_SPEED_MM_S)

        # 1) Đi thẳng tới điểm, giữ Z travel
        target_xy = (x_mm, y_mm, z_travel)
        dist_travel = math.dist(self.planner.current_pos, target_xy)
        if not self._walk_linear(target_xy, dist_travel / speed):
            return False

        # 2) + 3) + 4) Hạ nhẹ - dừng - nâng lại (chỉ khi có khai báo z_down > 0)
        if z_down and z_down > 0:
            z_bottom = z_travel + z_down  # quy ước "+" là đi XUỐNG
            target_down = (x_mm, y_mm, z_bottom)
            if not self._walk_linear(target_down, z_down / speed):
                return False

            if self._stop_flag:
                return False
            time.sleep(max(dwell_down_s, 0.0))

            target_up = (x_mm, y_mm, z_travel)
            if not self._walk_linear(target_up, z_down / speed):
                return False

        if self._stop_flag:
            return False

        # 5) Về vị trí Home ĐÃ LƯU - chỉ là 1 điểm đích như mọi điểm khác,
        # KHÔNG gửi lệnh HOME vật lý, KHÔNG chờ READY.
        if not self._return_to_saved_home(speed):
            return False

        return True