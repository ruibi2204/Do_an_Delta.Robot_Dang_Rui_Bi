"""
kinematics/manual_kinematic.py
================================

Module chuyển động riêng cho giao diện MANUAL (manual.py).

Mục tiêu:
    - Quỹ đạo giữa 2 điểm luôn là ĐƯỜNG THẲNG (nội suy tuyến tính trong không
      gian Cartesian X/Y/Z), không đi vòng, không lệch đường.
    - Vận tốc theo profile hình thang (trapezoidal velocity profile): có pha
      TĂNG TỐC -> ĐỀU TỐC -> GIẢM TỐC, tránh đổi vận tốc đột ngột ở đầu/cuối
      quỹ đạo -> giảm rung, giật cơ cấu.
    - Tốc độ di chuyển (mm/s) có thể chỉnh qua thanh trượt (slider) trên giao
      diện Manual, áp dụng ngay cho lần di chuyển tiếp theo.

Cách dùng trong manual.py:

    from kinematics.manual_kinematic import LinearMotionWorker

    worker = LinearMotionWorker(
        send_fn=self.ctx.planner.send_position,
        start_pos=self.ctx.current_pos,
        end_pos=new_pos,
        speed_mm_s=self.ctx.jog_speed,
    )
    worker.point_reached.connect(self._on_traj_point)
    worker.finished_ok.connect(self._on_traj_finished)
    worker.error.connect(self._on_traj_error)
    worker.start()
"""

import math
import time

from PySide6.QtCore import QThread, Signal


# ---------------------------------------------------------------------------
# Sinh quỹ đạo thẳng (linear trajectory generation)
# ---------------------------------------------------------------------------

def _distance(p0, p1):
    return math.sqrt(sum((b - a) ** 2 for a, b in zip(p0, p1)))


def generate_linear_trajectory(start, end, speed_mm_s, update_hz=50.0,
                                accel_mm_s2=None, min_points=2):
    """Sinh danh sách các điểm [x, y, z] nội suy TUYẾN TÍNH từ `start` đến
    `end`, theo profile vận tốc hình thang (accel / cruise / decel).

    Tham số:
        start, end      : (x, y, z) mm - điểm đầu và điểm cuối.
        speed_mm_s      : tốc độ mong muốn (mm/s), lấy từ thanh trượt tốc độ.
        update_hz       : tần số phát điểm trên quỹ đạo (Hz). 50 Hz cho
                           chuyển động rất mượt trên tay máy loại nhỏ.
        accel_mm_s2     : gia tốc dùng cho pha tăng/giảm tốc. Mặc định lấy
                           bằng speed_mm_s * 4 (đạt tốc độ max trong ~0.25s),
                           đủ êm để không gây rung cơ khí khi khởi động/dừng.
        min_points      : số điểm tối thiểu trả về (kể cả điểm cuối).

    Trả về:
        list[[x, y, z], ...] theo thứ tự thời gian, luôn kết thúc đúng bằng
        `end`.
    """
    start = list(start)
    end = list(end)
    dist = _distance(start, end)

    if dist < 1e-6:
        return [list(end)]

    speed_mm_s = max(0.1, float(speed_mm_s))
    if accel_mm_s2 is None:
        accel_mm_s2 = speed_mm_s * 4.0
    accel_mm_s2 = max(1.0, float(accel_mm_s2))

    # Thời gian / quãng đường cần để tăng tốc lên tới speed_mm_s
    t_acc = speed_mm_s / accel_mm_s2
    d_acc = 0.5 * accel_mm_s2 * t_acc ** 2

    if 2 * d_acc >= dist:
        # Quãng đường quá ngắn để đạt tới tốc độ đặt -> quỹ đạo hình tam giác
        # (tăng tốc rồi giảm tốc ngay, không có đoạn đều tốc).
        t_acc = math.sqrt(dist / accel_mm_s2)
        d_acc = dist / 2.0
        v_peak = accel_mm_s2 * t_acc
        t_flat = 0.0
        d_flat = 0.0
    else:
        v_peak = speed_mm_s
        d_flat = dist - 2 * d_acc
        t_flat = d_flat / speed_mm_s

    t_total = 2 * t_acc + t_flat

    dt = 1.0 / update_hz
    n_steps = max(min_points - 1, int(math.ceil(t_total / dt)))

    ux = (end[0] - start[0]) / dist
    uy = (end[1] - start[1]) / dist
    uz = (end[2] - start[2]) / dist

    points = []
    for i in range(1, n_steps + 1):
        t = min(i * dt, t_total)
        if t <= t_acc:
            s = 0.5 * accel_mm_s2 * t ** 2
        elif t <= t_acc + t_flat:
            s = d_acc + v_peak * (t - t_acc)
        else:
            td = t - t_acc - t_flat
            s = d_acc + d_flat + v_peak * td - 0.5 * accel_mm_s2 * td ** 2
        s = min(max(s, 0.0), dist)

        points.append([
            start[0] + ux * s,
            start[1] + uy * s,
            start[2] + uz * s,
        ])

    # Đảm bảo điểm cuối cùng khớp chính xác điểm đích (tránh sai số cộng dồn
    # do nội suy số thực).
    points[-1] = list(end)
    return points


# ---------------------------------------------------------------------------
# Worker chạy quỹ đạo trong luồng riêng (không chặn giao diện)
# ---------------------------------------------------------------------------

class LinearMotionWorker(QThread):
    """Gửi tuần tự từng điểm trên quỹ đạo thẳng xuống robot, trong một luồng
    (QThread) riêng để không làm treo giao diện, đồng thời giữ đúng nhịp thời
    gian giữa các điểm (giữ đúng profile vận tốc -> không rung).

    Signals:
        point_reached(list) : phát ra sau khi gửi thành công 1 điểm - dùng để
                               cập nhật hiển thị vị trí/góc theo thời gian thực.
        finished_ok(bool)   : phát ra khi kết thúc. True = đi hết quỹ đạo,
                               False = bị dừng giữa chừng hoặc robot từ chối
                               lệnh (send_fn trả về False).
        error(str)          : phát ra khi send_fn ném exception.
    """

    point_reached = Signal(list)
    finished_ok = Signal(bool)
    error = Signal(str)

    def __init__(self, send_fn, start_pos, end_pos, speed_mm_s,
                 update_hz=50.0, accel_mm_s2=None, parent=None):
        super().__init__(parent)
        self._send_fn = send_fn
        self._dt = 1.0 / update_hz
        self._points = generate_linear_trajectory(
            start_pos, end_pos, speed_mm_s,
            update_hz=update_hz, accel_mm_s2=accel_mm_s2,
        )
        self._stop_flag = False

    def stop(self):
        """Yêu cầu dừng quỹ đạo hiện tại càng sớm càng tốt (an toàn)."""
        self._stop_flag = True

    def run(self):
        for pt in self._points:
            if self._stop_flag:
                self.finished_ok.emit(False)
                return

            t0 = time.monotonic()
            try:
                ok = self._send_fn(*pt)
            except Exception as e:  # noqa: BLE001 - báo lỗi ra ngoài qua signal
                self.error.emit(str(e))
                return

            if ok is False:
                self.finished_ok.emit(False)
                return

            self.point_reached.emit(pt)

            # Giữ đúng nhịp thời gian giữa các điểm để tốc độ thực tế bám sát
            # profile đã tính (đây là yếu tố then chốt để tránh rung/giật).
            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0:
                time.sleep(remaining)

        self.finished_ok.emit(True)


# ---------------------------------------------------------------------------
# Worker gửi lệnh xoay bậc tự do 4 (θ4) qua UART 2, chạy trong luồng riêng
# ---------------------------------------------------------------------------
#
# Trước đây nút mũi tên xoay θ4 gọi thẳng uart2.step_rotate(...) NGAY trong
# hàm xử lý sự kiện click (tức là chạy trên main/GUI thread). Nếu lệnh này bị
# chặn (chờ ACK từ STM32, cổng serial bận, timeout...) dù chỉ vài trăm ms,
# toàn bộ giao diện sẽ đơ (không vẽ lại được, không phản hồi chuột/bàn phím)
# vì Qt event loop bị nghẽn. LinearMotionWorker ở trên đã né được vấn đề này
# cho jog X/Y/Z bằng cách chạy trong QThread riêng - Dof4RotateWorker áp dụng
# đúng kiểu mẫu đó cho lệnh xoay θ4.
# ---------------------------------------------------------------------------

class Dof4RotateWorker(QThread):
    """Gọi `step_rotate_fn(delta_deg)` (thường là `uart2.step_rotate`) trong
    một luồng riêng, để thao tác gửi lệnh xoay bậc tự do 4 không làm treo
    giao diện chính, dù cổng UART có bị chặn/chờ phản hồi.

    Signals:
        finished_ok(bool) : phát ra sau khi step_rotate_fn trả về. True nếu
                             kết quả trả về là "truthy" (gửi lệnh thành công),
                             False nếu robot/driver từ chối lệnh.
        error(str)        : phát ra khi step_rotate_fn ném exception (ví dụ
                             cổng serial bị rút giữa chừng).
    """

    finished_ok = Signal(bool)
    error = Signal(str)

    def __init__(self, step_rotate_fn, delta_deg, parent=None):
        super().__init__(parent)
        self._step_rotate_fn = step_rotate_fn
        self._delta_deg = delta_deg

    def run(self):
        try:
            ok = self._step_rotate_fn(self._delta_deg)
        except Exception as e:  # noqa: BLE001 - báo lỗi ra ngoài qua signal
            self.error.emit(str(e))
            return
        self.finished_ok.emit(bool(ok))