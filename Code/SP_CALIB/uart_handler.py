import serial
import serial.tools.list_ports


class UARTController:
    def __init__(self):
        self.serial_port = None
        self.is_connected = False
        self.port = None
        self.baudrate = 115200

    def find_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port, baudrate=115200):
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True
            self.port = port
            self.baudrate = baudrate
            return True, f"Kết nối thành công với {port}!"
        except Exception as e:
            return False, f"Lỗi kết nối: {str(e)}"

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
        self.port = None
        return True, "Đã ngắt kết nối"

    def send_theta(self, theta1, theta2, theta3):
        if not self.is_connected:
            return False, "Chưa kết nối UART!"

        try:
            command = f"T1:{theta1:.2f},T2:{theta2:.2f},T3:{theta3:.2f}\n"
            self.serial_port.write(command.encode())
            return True, "Đã gửi lệnh theta"
        except Exception as e:
            return False, f"Lỗi gửi dữ liệu: {str(e)}"

    def read_data(self):
        if not self.is_connected:
            return None, "Chưa kết nối UART!"

        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode().strip()
                return data, "Đọc thành công"
            return None, "Không có dữ liệu"
        except Exception as e:
            return None, f"Lỗi đọc dữ liệu: {str(e)}"