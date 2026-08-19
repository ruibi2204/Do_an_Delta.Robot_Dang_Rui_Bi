"""
Uart_2.py
=========
Giao tiếp UART 2 - THIẾT BỊ PHỤ (bơm / bàn xoay / step bậc tự do 4).

QUAN TRỌNG - LÝ DO FILE NÀY ĐƯỢC VIẾT LẠI:
--------------------------------------------
Bản cũ của file này mở cổng Serial NGAY KHI IMPORT MODULE:

    ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)

Điều này gây ra 2 lỗi nghiêm trọng:
  1. Nó luôn mở cứng COM3, bất kể người dùng chọn cổng nào trong GUI.
  2. Vì GUI (Robot_GUI.py) CŨNG tự mở một kết nối UART2 riêng (qua class
     PneumaticComm), nên khi move_delta.py gọi pump_on()/pump_off() từ file
     này, pyserial sẽ cố mở lại đúng cổng đó lần thứ 2 -> Windows trả về
     lỗi "Access is denied" (cổng đang bị chiếm bởi tiến trình/khối lệnh
     kia). Lỗi này ném ra NGAY GIỮA robot đang ở điểm A, đã hạ xuống, và
     đang gọi pump_on() -> robot dừng cứng, không bơm, không có gì xảy ra
     tiếp theo.

Cách sửa: CHỈ MỘT class PneumaticComm duy nhất được định nghĩa ở đây. Cả
Robot_GUI.py (tab KẾT NỐI) và move_delta.py đều dùng chung 1 THỰC THỂ
(instance) của class này - không có module nào tự mở serial khi import.

Giao thức lệnh dạng text, kết thúc bằng '\\n':
    PUMP:1 / PUMP:0      -> bật / tắt máy bơm
    TURN:<0-255>         -> đặt tốc độ PWM bàn xoay (0 = dừng)
    STEP:<độ>             -> quay step (bậc tự do 4) thêm N độ (âm = ngược chiều)
"""

import time
import threading

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200


class PneumaticComm:
    """Điều khiển bơm / bàn xoay / step (bậc tự do 4) qua UART 2.

    Không mở cổng khi khởi tạo (__init__) hay khi import module - chỉ mở
    khi gọi connect() một cách tường minh. Điều này tránh việc 2 nơi
    trong code (GUI và move_delta) vô tình mở trùng cổng.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, log_callback=None, dry_run=False):
        self.port = port
        self.baud = baud
        self.log = log_callback or print
        self.dry_run = dry_run or (not SERIAL_AVAILABLE)
        self._serial = None
        self._lock = threading.Lock()
        self._connected = False

    def connect(self) -> bool:
        if self._connected:
            # Đã kết nối rồi -> không cố mở lại cổng lần 2 (tránh Access Denied)
            self.log(f"[INFO] UART2 đã kết nối sẵn ({self.port}), bỏ qua connect() trùng lặp.")
            return True

        if self.dry_run:
            self._connected = True
            self.log(f"[DRY-RUN] Giả lập kết nối UART2 tới {self.port or 'VIRTUAL_PORT'}")
            return True

        if not self.port:
            self.log("[ERR] Chưa chọn cổng COM UART2!")
            return False

        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            self._connected = True
            self.log(f"[OK] Đã kết nối UART2 {self.port} @ {self.baud} baud")
            return True
        except serial.SerialException as e:
            # Nguyên nhân phổ biến nhất: cổng đang bị 1 tiến trình/đối tượng
            # khác giữ (vd: 2 PneumaticComm cùng mở 1 cổng). Log rõ ràng để
            # dễ chẩn đoán thay vì để robot "đứng hình" không rõ lý do.
            self.log(f"[ERR] Không thể mở {self.port}: {e} "
                     f"(kiểm tra xem cổng có đang bị chương trình khác/kết nối khác chiếm không)")
            self._connected = False
            return False
        except Exception as e:
            self.log(f"[ERR] Không thể mở {self.port}: {e}")
            self._connected = False
            return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._connected = False
        self.log("[INFO] Đã ngắt kết nối UART2")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def send_cmd(self, cmd: str) -> bool:
        if self.dry_run:
            self.log(f"[TX-DRY-UART2] {cmd}")
            return True
        if not self._connected or self._serial is None:
            self.log(f"[ERR] UART2 chưa kết nối! (lệnh bị bỏ qua: {cmd})")
            return False
        try:
            with self._lock:
                self._serial.write((cmd + "\n").encode("utf-8"))
            self.log(f"[TX-UART2] {cmd}")
            return True
        except Exception as e:
            self.log(f"[ERR] Gửi UART2 thất bại: {e}")
            return False

    # ---- Lệnh tiện ích: MÁY BƠM ----
    def pump_on(self) -> bool:
        return self.send_cmd("PUMP:1")

    def pump_off(self) -> bool:
        return self.send_cmd("PUMP:0")

    # ---- Lệnh tiện ích: BÀN XOAY (PWM 0-255) ----
    def turn_set_speed(self, speed: int) -> bool:
        speed = max(0, min(255, int(speed)))
        return self.send_cmd(f"TURN:{speed}")

    def turn_off(self) -> bool:
        return self.turn_set_speed(0)

    # ---- Lệnh tiện ích: BẬC TỰ DO 4 (STEP - GÓC QUAY) ----
    def step_rotate(self, degree: float) -> bool:
        return self.send_cmd(f"STEP:{degree}")

    @staticmethod
    def list_ports():
        if not SERIAL_AVAILABLE:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]