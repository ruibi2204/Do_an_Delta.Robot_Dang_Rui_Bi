# ============================================================
#  config/settings.py — Thông số cấu hình hệ thống Robot Delta
# ============================================================

# --- UART ---
UART_PORT     = "COM4"       # Thay đổi theo cổng thực tế (Linux: /dev/ttyUSB0)
UART_BAUDRATE = 115200
UART_TIMEOUT  = 1            # giây

# --- Thông số cơ học Robot Delta (đơn vị: mm) ---
BASE_RADIUS   = 100.0        # Bán kính mâm tĩnh (base)
EE_RADIUS     = 35.3         # Bán kính mâm động (end-effector)
UPPER_ARM     = 130.0        # Chiều dài cánh tay trên
LOWER_ARM     = 298.0        # Chiều dài cánh tay dưới (thanh nối)

# --- Bộ truyền đai ---
PULLEY_RATIO  = 3.8         # Tỉ số truyền u=4 (motor quay 4 vòng → khớp quay 1 vòng)

# --- Giới hạn vùng làm việc (mm) ---
X_MIN, X_MAX  = -150.0, 150.0
Y_MIN, Y_MAX  = -150.0, 150.0
Z_MIN, Z_MAX  = -450.0, 450.0

# --- PID ---
PID_KP        = 1.2
PID_KI        = 0.05
PID_KD        = 0.01
PID_SETPOINT  = 0.0
PID_DT        = 0.05         # chu kỳ lấy mẫu (giây)

# --- Giao thức truyền dữ liệu ---
# Frame gửi xuống STM32 : "<theta1,theta2,theta3>"
# Ví dụ               : "<30.25,45.00,-12.50>"
