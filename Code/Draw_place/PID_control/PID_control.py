# ============================================================
#  PID_control/PID_control.py — Bộ điều khiển PID Robot Delta
# ============================================================
"""
Module thực hiện điều khiển PID cho 3 khớp của Robot Delta.

Cách hoạt động trong vòng lặp:
    1. Nhận setpoint (góc IK đặt) từ quỹ đạo (trajectory)
    2. Nhận feedback (góc thực tế đo được) từ encoder/cảm biến hoặc
       ước lượng từ phản hồi UART
    3. Tính sai lệch error = setpoint - feedback
    4. Tính tín hiệu điều khiển u = Kp*e + Ki*∑e*dt + Kd*(de/dt)
    5. Giới hạn (clamp) tín hiệu đầu ra trong [-output_limit, +output_limit]
    6. Gửi tín hiệu u (góc đã bù PID) xuống UART

Cài đặt / Sử dụng nhanh:
    from PID_control.PID_control import PIDController, DeltaRobotPID

    # Tạo bộ PID cho một khớp
    pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
    output = pid.compute(setpoint=30.0, measurement=28.5, dt=0.05)

    # Hoặc dùng wrapper 3 khớp
    delta_pid = DeltaRobotPID(Kp=1.2, Ki=0.08, Kd=0.04)
    u1, u2, u3 = delta_pid.compute_all(
        setpoints=(30.0, -15.0, 20.0),
        measurements=(28.5, -14.0, 19.5),
        dt=0.05,
    )
"""

import time
from dataclasses import dataclass, field
from typing import Tuple, Optional


# ─── Kiểu dữ liệu tiện ích ───────────────────────────────────────────────────

@dataclass
class PIDState:
    """Lưu trữ trạng thái nội bộ của một kênh PID."""
    integral:    float = 0.0   # tổng tích phân
    prev_error:  float = 0.0   # sai lệch bước trước (cho vi phân)
    prev_time:   float = field(default_factory=time.monotonic)
    initialized: bool  = False


@dataclass
class PIDGains:
    """Tập hệ số PID."""
    Kp: float = 1.0
    Ki: float = 0.0
    Kd: float = 0.0


# ─── Lớp PID đơn kênh ────────────────────────────────────────────────────────

class PIDController:
    """
    Bộ điều khiển PID cho **một** kênh (một khớp).

    Tham số:
        Kp           : Hệ số tỉ lệ
        Ki           : Hệ số tích phân
        Kd           : Hệ số vi phân
        output_limit : Giá trị tuyệt đối tối đa của đầu ra (°)
        integral_limit: Giới hạn tích phân chống wind-up (°)
        deadband     : Vùng chết — sai lệch nhỏ hơn ngưỡng này bị bỏ qua
        name         : Tên kênh (để log)
    """

    def __init__(
        self,
        Kp:             float = 1.0,
        Ki:             float = 0.0,
        Kd:             float = 0.0,
        output_limit:   float = 30.0,
        integral_limit: float = 20.0,
        deadband:       float = 0.1,
        name:           str   = "PID",
    ):
        self.gains          = PIDGains(Kp=Kp, Ki=Ki, Kd=Kd)
        self.output_limit   = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.deadband       = abs(deadband)
        self.name           = name
        self._state         = PIDState()

    # ── Thuộc tính getter/setter hệ số ────────────────────────────────────────
    @property
    def Kp(self) -> float: return self.gains.Kp
    @Kp.setter
    def Kp(self, v: float): self.gains.Kp = float(v)

    @property
    def Ki(self) -> float: return self.gains.Ki
    @Ki.setter
    def Ki(self, v: float): self.gains.Ki = float(v)

    @property
    def Kd(self) -> float: return self.gains.Kd
    @Kd.setter
    def Kd(self, v: float): self.gains.Kd = float(v)

    # ── Tính toán chính ────────────────────────────────────────────────────────
    def compute(
        self,
        setpoint:    float,
        measurement: float,
        dt:          Optional[float] = None,
    ) -> float:
        """
        Tính tín hiệu điều khiển PID.

        Tham số:
            setpoint    : Giá trị mong muốn (góc đặt, °)
            measurement : Giá trị đo được (góc thực, °)
            dt          : Thời gian lấy mẫu (giây). Nếu None → tự đo.

        Trả về:
            output : Tín hiệu điều khiển đã giới hạn (°)
        """
        state = self._state

        # ── Tự đo dt nếu không truyền vào ──────────────────────────────────
        now = time.monotonic()
        if dt is None:
            if not state.initialized:
                dt = 0.0
            else:
                dt = now - state.prev_time
        state.prev_time   = now
        state.initialized = True

        # ── Sai lệch ────────────────────────────────────────────────────────
        error = setpoint - measurement

        # Vùng chết — tránh rung micro khi gần đích
        if abs(error) < self.deadband:
            error = 0.0

        # ── Tỉ lệ (P) ───────────────────────────────────────────────────────
        P = self.gains.Kp * error

        # ── Tích phân (I) với anti-windup clamping ───────────────────────────
        if dt > 0:
            state.integral += error * dt
            state.integral  = max(-self.integral_limit,
                                  min(self.integral_limit, state.integral))
        I = self.gains.Ki * state.integral

        # ── Vi phân (D) — trên measurement để tránh derivative kick ─────────
        D = 0.0
        if dt > 0:
            d_error = (error - state.prev_error) / dt
            D = self.gains.Kd * d_error

        state.prev_error = error

        # ── Đầu ra tổng hợp + giới hạn ──────────────────────────────────────
        output = P + I + D
        output = max(-self.output_limit, min(self.output_limit, output))

        return output

    def reset(self):
        """Đặt lại toàn bộ trạng thái bộ PID (dùng khi bắt đầu lại quỹ đạo)."""
        self._state = PIDState()

    def set_gains(self, Kp: float, Ki: float, Kd: float):
        """Cập nhật hệ số PID tức thời (có thể gọi từ GUI)."""
        self.gains.Kp = float(Kp)
        self.gains.Ki = float(Ki)
        self.gains.Kd = float(Kd)

    def get_state(self) -> dict:
        """Trả về dict trạng thái hiện tại để hiển thị trên GUI."""
        return {
            "name":     self.name,
            "Kp":       self.gains.Kp,
            "Ki":       self.gains.Ki,
            "Kd":       self.gains.Kd,
            "integral": self._state.integral,
            "prev_err": self._state.prev_error,
        }

    def __repr__(self) -> str:
        g = self.gains
        return (f"PIDController({self.name}: "
                f"Kp={g.Kp}, Ki={g.Ki}, Kd={g.Kd})")


# ─── Wrapper 3 khớp cho Robot Delta ──────────────────────────────────────────

class DeltaRobotPID:
    """
    Bộ điều khiển PID cho **3 khớp** Robot Delta.

    Mỗi khớp có một đối tượng PIDController riêng với cùng hệ số khởi đầu,
    có thể chỉnh riêng lẻ sau qua thuộc tính .pid1, .pid2, .pid3.

    Tham số:
        Kp, Ki, Kd      : Hệ số PID khởi tạo cho cả 3 khớp
        output_limit    : Giới hạn đầu ra (°) mỗi khớp
        integral_limit  : Giới hạn tích phân chống wind-up (°)
        deadband        : Vùng chết (°)
        enabled         : Bật/tắt PID — nếu False, trả về setpoint nguyên vẹn
    """

    def __init__(
        self,
        Kp:             float = 1.0,
        Ki:             float = 0.0,
        Kd:             float = 0.0,
        output_limit:   float = 30.0,
        integral_limit: float = 20.0,
        deadband:       float = 0.1,
        enabled:        bool  = True,
    ):
        _kw = dict(
            output_limit=output_limit,
            integral_limit=integral_limit,
            deadband=deadband,
        )
        self.pid1 = PIDController(Kp, Ki, Kd, name="θ₁", **_kw)
        self.pid2 = PIDController(Kp, Ki, Kd, name="θ₂", **_kw)
        self.pid3 = PIDController(Kp, Ki, Kd, name="θ₃", **_kw)
        self.enabled = enabled

    # ── Cập nhật hệ số đồng loạt ──────────────────────────────────────────────
    def set_gains(self, Kp: float, Ki: float, Kd: float):
        """Gán cùng hệ số PID cho cả 3 khớp cùng lúc."""
        for pid in (self.pid1, self.pid2, self.pid3):
            pid.set_gains(Kp, Ki, Kd)

    # ── Reset ─────────────────────────────────────────────────────────────────
    def reset(self):
        """Đặt lại tất cả 3 bộ PID."""
        for pid in (self.pid1, self.pid2, self.pid3):
            pid.reset()

    # ── Tính toán chính ────────────────────────────────────────────────────────
    def compute_all(
        self,
        setpoints:    Tuple[float, float, float],
        measurements: Tuple[float, float, float],
        dt:           Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        Tính tín hiệu điều khiển cho cả 3 khớp.

        Tham số:
            setpoints    : (theta1_sp, theta2_sp, theta3_sp) — góc IK đặt (°)
            measurements : (theta1_fb, theta2_fb, theta3_fb) — góc phản hồi (°)
            dt           : Thời gian lấy mẫu (giây). None → tự đo.

        Trả về:
            (u1, u2, u3) : Tín hiệu điều khiển đã bù PID (°)
                           Nếu enabled=False → trả về setpoints nguyên vẹn.
        """
        if not self.enabled:
            return setpoints  # bypass PID

        s1, s2, s3 = setpoints
        m1, m2, m3 = measurements

        u1 = s1 + self.pid1.compute(s1, m1, dt)
        u2 = s2 + self.pid2.compute(s2, m2, dt)
        u3 = s3 + self.pid3.compute(s3, m3, dt)

        return u1, u2, u3

    # ── Trạng thái ────────────────────────────────────────────────────────────
    def get_state(self) -> dict:
        """Trả về dict trạng thái 3 bộ PID (để hiển thị trên GUI)."""
        return {
            "enabled": self.enabled,
            "pid1":    self.pid1.get_state(),
            "pid2":    self.pid2.get_state(),
            "pid3":    self.pid3.get_state(),
        }

    def __repr__(self) -> str:
        g = self.pid1.gains
        return (f"DeltaRobotPID(Kp={g.Kp}, Ki={g.Ki}, Kd={g.Kd}, "
                f"enabled={self.enabled})")


# ─── Hàm tiện ích ────────────────────────────────────────────────────────────

def simulate_step(
    delta_pid: DeltaRobotPID,
    setpoints: Tuple[float, float, float],
    measurements: Tuple[float, float, float],
    dt: float = 0.05,
) -> Tuple[float, float, float]:
    """
    Hàm wrapper tiện lợi cho một bước điều khiển PID.

    Tham số:
        delta_pid    : Đối tượng DeltaRobotPID
        setpoints    : Góc IK mong muốn (°)
        measurements : Góc phản hồi thực tế (°)
        dt           : Chu kỳ điều khiển (giây)

    Trả về:
        (u1, u2, u3) : Góc sau bù PID gửi xuống stepper
    """
    return delta_pid.compute_all(setpoints, measurements, dt)
