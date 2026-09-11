import math
import time

from kinematics.move_delta_4dof import DeltaMotionPlanner, MotionError


class DrawMotionError(Exception):
    pass


class DrawMotionPlanner:
    """
    Bộ điều khiển chuyển động dành riêng cho giao diện VẼ (đường thẳng /
    hình tròn). KHÔNG mở kết nối UART riêng - dùng chung DeltaMotionPlanner
    (và do đó dùng chung UART2 do GUI quản lý) truyền vào từ ctx.planner,
    giống cách các cửa sổ khác tái sử dụng planner.

    Khác với các bước gắp-thả (thời gian di chuyển cố định), quỹ đạo vẽ
    được sinh theo TỐC ĐỘ VẼ (mm/s) người dùng nhập: quãng đường càng dài
    thì thời gian di chuyển càng lâu, số điểm gửi theo tần số điều khiển
    cố định của planner (planner.CONTROL_HZ) để chuyển động mượt.

    Ngoài ra hỗ trợ "hạ thêm" (dip): sau khi hạ tới Z pick (z truyền vào),
    có thể hạ thêm một đoạn nhẹ (dip, mm) trước khi bắt đầu vẽ, để đảm bảo
    bút/đầu vẽ chạm mặt phẳng vẽ chắc chắn hơn Z pick ban đầu.
    """

    MIN_SPEED_MM_S = 0.1  # chặn dưới, tránh chia cho 0 / tốc độ quá nhỏ

    def __init__(self, planner: DeltaMotionPlanner):
        self.planner = planner
        self._stop_flag = False

    # ------------------------------------------------------------------
    def stop(self):
        """Gọi từ nút STOP (có thể gọi từ luồng GUI, chỉ set cờ)."""
        self._stop_flag = True

    def _reset_stop(self):
        self._stop_flag = False

    def _send_or_raise(self, x, y, z):
        if not self.planner.send_position(x, y, z):
            raise DrawMotionError(f"Gửi tọa độ ({x:.1f}, {y:.1f}, {z:.1f}) thất bại.")
        # Đồng bộ vị trí hiện tại của planner để các thao tác sau (ví dụ
        # về Home) nội suy đúng từ điểm cuối cùng vừa vẽ.
        self.planner.current_pos = (x, y, z)

    def _lift_and_travel(self, x, y, z_travel):
        """Nhấc bút lên độ cao an toàn Z_SAFE rồi di chuyển ngang tới (x, y)."""
        cur_x, cur_y, _cur_z = self.planner.current_pos
        self.planner._move_or_raise(cur_x, cur_y, z_travel, self.planner.TIME_MOVE_DOWN)
        self.planner._move_or_raise(x, y, z_travel, self.planner.TIME_MOVE_FAST)

    def _descend_to_draw_z(self, x, y, z_pick, dip):
        """Hạ xuống Z pick, sau đó nếu dip > 0 thì hạ thêm một đoạn nhẹ
        trước khi vẽ. Trả về độ cao Z thực tế sẽ dùng để vẽ."""
        self.planner._move_or_raise(x, y, z_pick, self.planner.TIME_MOVE_DOWN)
        z_draw = z_pick
        if dip > 0:
            z_draw = z_pick + dip
            self.planner._move_or_raise(x, y, z_draw, self.planner.TIME_MOVE_DOWN)
        return z_draw

    # ------------------------------------------------------------------
    def draw_line(self, x1, y1, x2, y2, z, speed_mm_s, dip=0.0, return_home=True):
        """Vẽ đoạn thẳng từ (x1,y1) tới (x2,y2) tại độ cao z (Z pick), tốc độ
        speed_mm_s (mm/s). Nếu dip > 0, sau khi hạ tới z sẽ hạ thêm dip (mm)
        rồi mới vẽ. Trả về True nếu vẽ xong trọn vẹn, False nếu bị dừng giữa
        chừng (STOP)."""
        self._reset_stop()
        speed = max(speed_mm_s, self.MIN_SPEED_MM_S)
        dip = max(dip, 0.0)

        try:
            # 1. Nhấc bút lên an toàn -> di chuyển tới phía trên điểm bắt đầu
            #    -> hạ xuống Z pick -> hạ thêm đoạn nhẹ (nếu có) -> vẽ.
            self._lift_and_travel(x1, y1, self.planner.Z_SAFE)
            z_draw = self._descend_to_draw_z(x1, y1, z, dip)

            # 2. Vẽ đường thẳng: nội suy tuyến tính theo tốc độ vẽ.
            length = math.hypot(x2 - x1, y2 - y1)
            if length > 1e-6:
                duration = length / speed
                n_steps = max(int(duration * self.planner.CONTROL_HZ), 2)

                for i in range(1, n_steps + 1):
                    if self._stop_flag:
                        return False
                    t_start = time.perf_counter()

                    t = i / n_steps
                    x = x1 + (x2 - x1) * t
                    y = y1 + (y2 - y1) * t
                    self._send_or_raise(x, y, z_draw)

                    elapsed = time.perf_counter() - t_start
                    remaining = self.planner.CONTROL_DT - elapsed
                    if remaining > 0:
                        time.sleep(remaining)

            # 3. Nhấc bút lên an toàn sau khi vẽ xong.
            self.planner._move_or_raise(x2, y2, self.planner.Z_SAFE, self.planner.TIME_MOVE_DOWN)

            if return_home:
                self.planner.move_home()
            return True

        except (MotionError, DrawMotionError) as e:
            print(f"[LỖI VẼ] {e}")
            raise

    # ------------------------------------------------------------------
    def draw_circle(self, cx, cy, radius, z, speed_mm_s, dip=0.0, clockwise=True, return_home=True):
        """Vẽ hình tròn tâm (cx,cy) bán kính radius tại độ cao z (Z pick),
        tốc độ speed_mm_s (mm/s). Nếu dip > 0, sau khi hạ tới z sẽ hạ thêm
        dip (mm) rồi mới vẽ. Trả về True nếu vẽ xong trọn vẹn, False nếu bị
        dừng giữa chừng (STOP)."""
        self._reset_stop()
        speed = max(speed_mm_s, self.MIN_SPEED_MM_S)
        dip = max(dip, 0.0)

        if radius <= 0:
            raise DrawMotionError("Bán kính hình tròn phải > 0.")

        start_x = cx + radius
        start_y = cy

        try:
            # 1. Nhấc bút lên an toàn -> di chuyển tới phía trên điểm xuất
            #    phát trên đường tròn (góc 0°) -> hạ xuống Z pick -> hạ thêm
            #    đoạn nhẹ (nếu có) -> vẽ.
            self._lift_and_travel(start_x, start_y, self.planner.Z_SAFE)
            z_draw = self._descend_to_draw_z(start_x, start_y, z, dip)

            # 2. Vẽ hình tròn: nội suy theo góc, tốc độ dài không đổi.
            circumference = 2.0 * math.pi * radius
            duration = circumference / speed
            n_steps = max(int(duration * self.planner.CONTROL_HZ), 16)
            direction = -1.0 if clockwise else 1.0

            for i in range(1, n_steps + 1):
                if self._stop_flag:
                    return False
                t_start = time.perf_counter()

                theta = direction * 2.0 * math.pi * (i / n_steps)
                x = cx + radius * math.cos(theta)
                y = cy + radius * math.sin(theta)
                self._send_or_raise(x, y, z_draw)

                elapsed = time.perf_counter() - t_start
                remaining = self.planner.CONTROL_DT - elapsed
                if remaining > 0:
                    time.sleep(remaining)

            # 3. Nhấc bút lên an toàn sau khi vẽ xong (đang ở lại điểm xuất phát).
            self.planner._move_or_raise(start_x, start_y, self.planner.Z_SAFE, self.planner.TIME_MOVE_DOWN)

            if return_home:
                self.planner.move_home()
            return True

        except (MotionError, DrawMotionError) as e:
            print(f"[LỖI VẼ] {e}")
            raise