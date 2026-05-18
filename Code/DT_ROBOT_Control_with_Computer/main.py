import tkinter as tk
import time
from GD import RobotDeltaGUI
from DHNghich import tinh_dhn
from Uart import UartController
from PID import PIDController

# Khởi tạo các module cứng toàn cục để hàm điều khiển nút bấm gọi được
uart = UartController(port='COM4', baud=115200)
pid1 = PIDController(kp=1.0, ki=0.0, kd=0.01, out_min=-45.0, out_max=90.0)
pid2 = PIDController(kp=1.0, ki=0.0, kd=0.01, out_min=-45.0, out_max=90.0)
pid3 = PIDController(kp=1.0, ki=0.0, kd=0.01, out_min=-45.0, out_max=90.0)


def xử_lý_khi_bấm_gửi(x, y, z):
    """Hàm này sẽ tự động chạy khi bạn ấn nút GỬI TỌA ĐỘ trên giao diện"""
    # 1. Tính toán động học nghịch
    ket_qua = tinh_dhn(x, y, z)

    if ket_qua is None:
        from tkinter import messagebox
        messagebox.showwarning("Cảnh báo vùng làm việc", "Tọa độ này vượt tầm cơ khí của cánh tay!")
        return

    t1_set, t2_set, t3_set = ket_qua

    # 2. Lọc qua bộ PID
    t1_ctrl = pid1.compute(t1_set, 0)
    t2_ctrl = pid2.compute(t2_set, 0)
    t3_ctrl = pid3.compute(t3_set, 0)

    # 3. Bắn lệnh xuống STM32 qua UART
    uart.send_angles(t1_ctrl, t2_ctrl, t3_ctrl)
    print(f"Giao diện đã ra lệnh: X={x}, Y={y}, Z={z} -> Gửi góc UART.")


def main():
    # Đợi robot STM32 bật nguồn và tự động Set Home xong rồi mới mở giao diện lên
    if uart.ser and uart.wait_for_homing():
        root = tk.Tk()

        # Truyền hàm xử lý vào giao diện để liên kết nút bấm
        app = RobotDeltaGUI(root, on_send_callback=xử_lý_khi_bấm_gửi)

        root.mainloop()  # Giữ giao diện luôn mở


if __name__ == "__main__":
    main()