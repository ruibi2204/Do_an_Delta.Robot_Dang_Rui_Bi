# ============================================================
#  control/pid.py — Bộ điều khiển PID
# ============================================================
import time
from config.settings import PID_KP, PID_KI, PID_KD, PID_DT


class PIDController:
    """
    Bộ điều khiển PID rời rạc.

    Có thể dùng riêng cho từng trục hoặc từng khớp.
    """

    def __init__(
        self,
        kp: float = PID_KP,
        ki: float = PID_KI,
        kd: float = PID_KD,
        dt: float = PID_DT,
        output_limits: tuple[float, float] = (-180.0, 180.0),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_min, self.output_max = output_limits

        self._integral   = 0.0
        self._prev_error = 0.0
        self._setpoint   = 0.0
        self._last_time  = time.time()

    # ----------------------------------------------------------
    def set_setpoint(self, setpoint: float) -> None:
        """Cập nhật giá trị đặt."""
        self._setpoint = setpoint

    def reset(self) -> None:
        """Reset tích phân và sai số trước."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._last_time  = time.time()

    # ----------------------------------------------------------
    def compute(self, measured_value: float) -> float:
        """
        Tính đầu ra PID.

        Tham số:
            measured_value : Giá trị đo được hiện tại

        Trả về:
            output : Giá trị điều khiển (đã clamp trong output_limits)
        """
        now = time.time()
        dt  = now - self._last_time
        if dt <= 0.0:
            dt = self.dt
        self._last_time = now

        error = self._setpoint - measured_value

        # Tỉ lệ
        p_term = self.kp * error

        # Tích phân (có anti-windup)
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Vi phân
        derivative = (error - self._prev_error) / dt
        d_term     = self.kd * derivative

        self._prev_error = error

        output = p_term + i_term + d_term

        # Clamp đầu ra
        output = max(self.output_min, min(self.output_max, output))
        return output

    # ----------------------------------------------------------
    def tune(self, kp: float, ki: float, kd: float) -> None:
        """Cập nhật hệ số PID trong lúc chạy."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()
