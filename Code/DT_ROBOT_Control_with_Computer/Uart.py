import serial
import time


class UartController:
    def __init__(self, port='COM3', baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            print(f"Đang kết nối cổng {port}...")
        except Exception as e:
            print(f"Không thể mở cổng {port}: {e}")
            self.ser = None

    def wait_for_homing(self):
        """Hàm đợi STM32 gửi chữ READY báo hiệu đã Set Home xong"""
        if self.ser is None:
            return False

        print("Đang chờ Robot thực hiện quá trình Set Home từ STM32...")
        while True:
            if self.ser.in_waiting > 0:
                try:
                    # Đọc dữ liệu phản hồi từ STM32
                    line = self.ser.readline().decode('utf-8').strip()
                    if line == "READY":
                        print("--> Robot đã Set Home xong! Kích hoạt giao diện.")
                        return True
                except Exception as e:
                    print(f"Lỗi đọc dữ liệu UART: {e}")
            time.sleep(0.1)

    def send_angles(self, t1, t2, t3):
        """Hàm đóng gói dữ liệu dạng <t1,t2,t3> gửi xuống STM32"""
        if self.ser and self.ser.is_open:
            data = f"<{t1:.3f},{t2:.3f},{t3:.3f}>\n"
            self.ser.write(data.encode('utf-8'))

    def close(self):
        if self.ser:
            self.ser.close()