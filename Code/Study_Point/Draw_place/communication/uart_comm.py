import time
import threading
from typing import Optional, Callable
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

GEAR_RATIO   = 3.0
DEFAULT_BAUD = 115200
SEND_TIMEOUT = 1
class UARTComm:
    def __init__(
        self,
        port: str = "COM6",
        baud: int = DEFAULT_BAUD,
        gear_ratio: float = GEAR_RATIO,
        log_callback: Optional[Callable[[str], None]] = None,
        dry_run: bool = False,
    ):
        self.port       = port
        self.baud       = baud
        self.gear_ratio = gear_ratio
        self.log        = log_callback or print
        self.dry_run    = dry_run or (not SERIAL_AVAILABLE)
        self._serial: Optional["serial.Serial"] = None
        self._lock   = threading.Lock()
        self._connected = False
        if not SERIAL_AVAILABLE and not dry_run:
            self.log("[WARN] pyserial chưa cài — tự động chuyển DRY-RUN mode")
    def connect(self) -> bool:
        if self.dry_run:
            self._connected = True
            self.log(f"[DRY-RUN] Giả lập kết nối tới {self.port or 'VIRTUAL_PORT'}")
            return True
        if not self.port:
            self.log("[ERR] Chưa chọn cổng COM!")
            return False
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SEND_TIMEOUT,
            )
            time.sleep(2.0)
            self._connected = True
            self.log(f"[OK] Đã kết nối {self.port} @ {self.baud} baud")
            return True
        except Exception as e:
            self.log(f"[ERR] Không thể mở {self.port}: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Đóng cổng Serial."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._connected = False
        self.log("[INFO] Đã ngắt kết nối Serial")

    @property
    def is_connected(self) -> bool:
        return self._connected
    def send_angles(self, theta1: float, theta2: float, theta3: float) -> bool:
        cmd = f"T1:{theta1 * self.gear_ratio:.3f} T2:{theta2 * self.gear_ratio:.3f} T3:{theta3 * self.gear_ratio:.3f}\n"
        if self.dry_run:
            self.log(f"[TX-DRY] {cmd.strip()}")
            return True
        if not self._connected or self._serial is None:
            self.log("[ERR] Chưa kết nối Serial!")
            return False
        try:
            with self._lock:
                self._serial.write(cmd.encode("ascii"))
                self._serial.flush()
            self.log(f"[TX] {cmd.strip()}") # Thêm log ở đây để dễ debug trên GUI
            return True
        except Exception as e:
            self.log(f"[ERR] Gửi thất bại: {e}")
            return False
    @staticmethod
    def list_ports() -> list[str]:
        if not SERIAL_AVAILABLE:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]
    def send_home(self) -> bool:
        """Gửi lệnh HOME để robot về vị trí zero."""
        cmd = "HOME\n"
        if self.dry_run:
            self.log("[TX-DRY] HOME")
            return True
        if not self._connected or self._serial is None:
            self.log("[ERR] Chưa kết nối Serial!")
            return False
        try:
            with self._lock:
                self._serial.write(cmd.encode("ascii"))
                self._serial.flush()
            self.log("[TX] HOME")
            return True
        except Exception as e:
            self.log(f"[ERR] HOME thất bại: {e}")
            return False