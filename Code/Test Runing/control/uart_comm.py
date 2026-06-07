# ============================================================
#  control/uart_comm.py — Giao tiếp UART với STM32
# ============================================================
#  Frame gửi xuống STM32 : "<theta1,theta2,theta3>"
#  Frame nhận từ STM32   : "READY" (sau khi homing xong)
# ============================================================
import serial
import serial.tools.list_ports
import threading
import logging
from config.settings import UART_PORT, UART_BAUDRATE, UART_TIMEOUT, PULLEY_RATIO

logger = logging.getLogger(__name__)


class UARTComm:

    def __init__(
        self,
        port: str      = UART_PORT,
        baudrate: int  = UART_BAUDRATE,
        timeout: float = UART_TIMEOUT,
        on_ready=None,       # callback khi STM32 gửi "READY"
    ):
        self.port      = port
        self.baudrate  = baudrate
        self.timeout   = timeout
        self.on_ready  = on_ready   # hàm gọi khi nhận được READY
        self.stm_ready = False      # True sau khi homing xong

        self._serial: serial.Serial | None = None
        self._lock    = threading.Lock()
        self._rx_thread: threading.Thread | None = None
        self._running = False

    # ----------------------------------------------------------
    #  Kết nối / ngắt kết nối
    # ----------------------------------------------------------
    def connect(self) -> bool:
        """Mở cổng UART và khởi động thread đọc phản hồi."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            self._running = True
            self._rx_thread = threading.Thread(
                target=self._read_loop, daemon=True
            )
            self._rx_thread.start()
            logger.info(f"[UART] Đã kết nối {self.port} @ {self.baudrate} baud")
            logger.info("[UART] Đang chờ STM32 homing xong (READY)...")
            return True
        except serial.SerialException as e:
            logger.error(f"[UART] Lỗi kết nối: {e}")
            self._serial = None
            return False

    def disconnect(self) -> None:
        """Đóng cổng UART an toàn."""
        self._running = False
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                logger.info("[UART] Đã ngắt kết nối")
        self.stm_ready = False

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ----------------------------------------------------------
    #  Thread đọc phản hồi từ STM32
    # ----------------------------------------------------------
    def _read_loop(self):
        """Chạy nền, lắng nghe mọi phản hồi từ STM32."""
        while self._running and self.is_connected:
            try:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    logger.debug(f"[UART] ← STM32: {line}")

                    # STM32 báo homing xong
                    if line == "READY":
                        self.stm_ready = True
                        logger.info("[UART] ✅ STM32 đã READY, có thể điều khiển!")
                        if self.on_ready:
                            self.on_ready()
            except serial.SerialException as e:
                logger.error(f"[UART] Lỗi đọc: {e}")
                break

    # ----------------------------------------------------------
    #  Gửi góc xuống STM32
    # ----------------------------------------------------------
    def send_angles(self, theta1: float, theta2: float, theta3: float) -> bool:
        """
        Đóng gói và gửi 3 góc khớp xuống STM32.
        Góc đã được nhân với PULLEY_RATIO (u=4) trước khi gửi.

        Frame: "<theta1_motor,theta2_motor,theta3_motor>"
        Ví dụ: "<121.00,180.00,-50.00>"
        """
        if not self.is_connected:
            logger.warning("[UART] Chưa kết nối!")
            return False

        if not self.stm_ready:
            logger.warning("[UART] STM32 chưa READY (đang homing)!")
            return False

        # Quy đổi góc khớp → góc motor (nhân tỉ số truyền đai)
        m1 = theta1 * PULLEY_RATIO
        m2 = theta2 * PULLEY_RATIO
        m3 = theta3 * PULLEY_RATIO

        frame = f"<{m1:.2f},{m2:.2f},{m3:.2f}>"
        with self._lock:
            try:
                self._serial.write(frame.encode("utf-8"))
                logger.debug(f"[UART] → STM32 (motor): {frame}  |  khớp: θ=({theta1:.2f}°,{theta2:.2f}°,{theta3:.2f}°)")
                return True
            except serial.SerialException as e:
                logger.error(f"[UART] Lỗi gửi: {e}")
                return False

    # ----------------------------------------------------------
    #  Tiện ích
    # ----------------------------------------------------------
    @staticmethod
    def list_ports() -> list[str]:
        """Liệt kê các cổng COM khả dụng trên máy."""
        return [p.device for p in serial.tools.list_ports.comports()]
