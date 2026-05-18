import time


class PIDController:
    def __init__(self, kp, ki, kd, out_min=None, out_max=None):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Giới hạn đầu ra (Anti-windup & bảo vệ cơ khí)
        self.out_min = out_min
        self.out_max = out_max

        # Các biến trạng thái
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, target, current):

        now = time.time()

        # Xử lý thời gian lấy mẫu thực tế (dt)
        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time

        self.last_time = now

        # 1. Tính toán sai số (Error)
        error = target - current

        # 2. Thành phần Tỉ lệ (Proportional)
        P_out = self.kp * error

        # 3. Thành phần Tích phân (Integral) - Tích lũy sai số theo thời gian
        # Chỉ tính toán nếu dt > 0 để tránh chia cho 0 hoặc lỗi vòng lặp đầu tiên
        if dt > 0:
            self.integral += error * dt

        # Chống bão hòa tích phân (Anti-windup) bằng cách giới hạn khâu I
        if self.out_max is not None and self.out_min is not None:
            # Ước lượng giới hạn khâu I dựa trên giới hạn đầu ra tổng thể
            self.integral = max(min(self.integral, self.out_max / self.ki if self.ki != 0 else 0),
                                self.out_min / self.ki if self.ki != 0 else 0)

        I_out = self.ki * self.integral

        # 4. Thành phần Vi phân (Derivative) - Dự đoán xu hướng sai số
        if dt > 0:
            D_out = self.kd * (error - self.prev_error) / dt
        else:
            D_out = 0.0

        # Cập nhật lại sai số cho chu kỳ sau
        self.prev_error = error

        # Tổng hợp tín hiệu điều khiển đầu ra
        output = P_out + I_out + D_out

        # 5. Giới hạn saturation đầu ra cuối cùng (Bảo vệ cơ khí)
        if self.out_max is not None:
            output = min(output, self.out_max)
        if self.out_min is not None:
            output = max(output, self.out_min)

        return output

    def reset(self):
        """
        Hàm xóa lịch sử điều khiển, dùng khi robot chuyển trạng thái 
        hoặc reset quỹ đạo để tránh bị giật (shock) khâu D.
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None