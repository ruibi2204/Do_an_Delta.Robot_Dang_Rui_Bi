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

    Ghi chú làm mượt / chống rung (QUAN TRỌNG: đường vẽ vẫn phải THẲNG):
        - QUỸ ĐẠO KHÔNG GIAN không đổi: đường thẳng vẫn là tổ hợp tuyến
          tính chính xác giữa 2 điểm đầu/cuối (x = x1 + (x2-x1)*s), hình
          tròn vẫn là (cx + R*cos, cy + R*sin) với R không đổi. Việc làm
          mượt chỉ tác động lên THAM SỐ TIẾN TRÌNH s (0..1) theo thời gian,
          không làm lệch hình dạng đường đi.
        - THAM SỐ s được lấy từ S-curve bậc 5 (vận tốc và gia tốc = 0 ở
          đầu/cuối đoạn), thay vì tăng vọt vận tốc từ 0 lên tốc độ vẽ ngay
          lập tức như trước - đây là nguồn rung chính khi bắt đầu/kết thúc
          nét vẽ.
        - Lịch gửi lệnh dùng mốc THỜI GIAN TUYỆT ĐỐI (t0 + step*dt) và s
          được tính theo THỜI GIAN THỰC ĐÃ TRÔI QUA, không theo số thứ tự
          bước cố định -> nếu UART gửi bị trễ (jitter) ở vài điểm, quỹ đạo
          không bị "dồn tốc" bù trừ đột ngột (nguồn rung thứ hai).

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
    # Làm mượt: S-curve bậc 5 áp dụng cho THAM SỐ TIẾN TRÌNH theo thời gian,
    # KHÔNG tác động tới hình dạng quỹ đạo không gian (đường vẫn thẳng /
    # hình tròn vẫn tròn tuyệt đối).
    # ------------------------------------------------------------------
    @staticmethod
    def _s_curve(t_frac):
        """Smoothstep bậc 5 (6t^5 - 15t^4 + 10t^3): vận tốc VÀ gia tốc đều
        = 0 ở đầu/cuối đoạn -> vào/ra nét vẽ êm, không giật cục."""
        t = min(max(t_frac, 0.0), 1.0)
        return t * t * t * (t * (t * 6 - 15) + 10)

    def _walk_time_based(self, duration, xy_fn, z):
        """Chạy 1 đoạn quỹ đạo trong `duration` giây. `xy_fn(s)` trả về
        (x, y) tương ứng với tham số tiến trình s trong [0, 1] - s đã được
        làm mượt bằng S-curve, còn xy_fn chỉ áp dụng nội suy KHÔNG GIAN
        thuần túy (tuyến tính cho đường thẳng, lượng giác cho hình tròn)
        nên hình dạng đường đi luôn chính xác.

        Gửi lệnh theo tần số cố định planner.CONTROL_HZ, dùng mốc thời gian
        tuyệt đối (không cộng dồn sleep) và s tính theo thời gian thực đã
        trôi qua để không bị trôi / dồn tốc khi UART có độ trễ không đều.

        Trả về True nếu đi hết đoạn, False nếu bị dừng giữa chừng (STOP).
        """
        dt = self.planner.CONTROL_DT
        duration = max(duration, dt * 4)  # đủ điểm để S-curve mượt
        t0 = time.perf_counter()
        step = 0

        while True:
            if self._stop_flag:
                return False

            step += 1
            target_time = t0 + step * dt
            elapsed = min(target_time - t0, duration)
            frac_time = elapsed / duration
            finished = frac_time >= 1.0

            s = self._s_curve(frac_time)
            x, y = xy_fn(s)
            self._send_or_raise(x, y, z)

            if finished:
                return True

            # Ngủ tới đúng mốc thời gian kế tiếp trên lưới cố định. Nếu bị
            # trễ so với mốc (UART chậm), KHÔNG cố đuổi kịp bằng cách tăng
            # tốc đột ngột ở bước sau - s (tính theo thời gian thực) đã tự
            # phản ánh đúng vị trí cần có tại thời điểm đó.
            now = time.perf_counter()
            sleep_time = target_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

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
            self._lift_and_travel(x1, y1, 280)
            z_draw = self._descend_to_draw_z(x1, y1, z, dip)

            # 2. Vẽ đường thẳng: nội suy TUYẾN TÍNH tuyệt đối theo tốc độ vẽ
            #    (s là tổ hợp lồi của 2 điểm đầu/cuối -> luôn nằm đúng trên
            #    đường thẳng nối x1,y1 - x2,y2, bất kể s tiến triển theo
            #    thời gian tuyến tính hay theo S-curve).
            length = math.hypot(x2 - x1, y2 - y1)
            if length > 1e-6:
                duration = length / speed

                def xy_fn(s, _x1=x1, _y1=y1, _x2=x2, _y2=y2):
                    return (_x1 + (_x2 - _x1) * s, _y1 + (_y2 - _y1) * s)

                completed = self._walk_time_based(duration, xy_fn, z_draw)
                if not completed:
                    return False

            # 3. Nhấc bút lên an toàn sau khi vẽ xong.
            self.planner._move_or_raise(x2, y2, 280, self.planner.TIME_MOVE_DOWN)

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
        direction = -1.0 if clockwise else 1.0

        try:
            # 1. Nhấc bút lên an toàn -> di chuyển tới phía trên điểm xuất
            #    phát trên đường tròn (góc 0°) -> hạ xuống Z pick -> hạ thêm
            #    đoạn nhẹ (nếu có) -> vẽ.
            self._lift_and_travel(start_x, start_y, 280)
            z_draw = self._descend_to_draw_z(start_x, start_y, z, dip)

            # 2. Vẽ hình tròn: bán kính R KHÔNG ĐỔI trong suốt nét vẽ (đúng
            #    hình tròn tuyệt đối) - chỉ có góc theta tiến triển theo s
            #    đã làm mượt để vào/ra nét vẽ êm hơn.
            circumference = 2.0 * math.pi * radius
            duration = circumference / speed

            def xy_fn(s, _cx=cx, _cy=cy, _r=radius, _dir=direction):
                theta = _dir * 2.0 * math.pi * s
                return (_cx + _r * math.cos(theta), _cy + _r * math.sin(theta))

            completed = self._walk_time_based(duration, xy_fn, z_draw)
            if not completed:
                return False

            # 3. Nhấc bút lên an toàn sau khi vẽ xong (đang ở lại điểm xuất phát).
            self.planner._move_or_raise(start_x, start_y, 280, self.planner.TIME_MOVE_DOWN)

            if return_home:
                self.planner.move_home()
            return True

        except (MotionError, DrawMotionError) as e:
            print(f"[LỖI VẼ] {e}")
            raise