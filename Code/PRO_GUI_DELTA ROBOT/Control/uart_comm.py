import time
import threading

from PyQt5.QtCore import QObject, pyqtSignal

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class UartComm(QObject):
    """Quan ly ket noi Serial va gui lenh xuong robot delta qua UART."""

    log_signal = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.lock = threading.Lock()
        self.is_connected = False

    # ---------------- Ket noi ----------------
    def list_ports(self):
        if not HAS_SERIAL:
            return []
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=115200, timeout=2):
        if not HAS_SERIAL:
            self.log_signal.emit("[LOI] Chua cai pyserial. Chay: pip install pyserial")
            return False
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # cho firmware khoi dong (Arduino tu reset khi mo cong)
            self.is_connected = True
            self.connection_changed.emit(True)
            self.log_signal.emit(f"[OK] Da ket noi {port} @ {baudrate} baud")
            return True
        except Exception as e:
            self.log_signal.emit(f"[LOI] Khong the ket noi {port}: {e}")
            self.is_connected = False
            self.connection_changed.emit(False)
            return False

    def disconnect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.log_signal.emit("[INFO] Da ngat ket noi")
        except Exception as e:
            self.log_signal.emit(f"[LOI] Loi khi ngat ket noi: {e}")
        finally:
            self.is_connected = False
            self.connection_changed.emit(False)

    # ---------------- Xay dung lenh (SUA O DAY NEU DOI GIAO THUC) ----------------
    def build_move_command(self, t1_motor, t2_motor, t3_motor, feed):
        return f"T1:{t1_motor:.3f} T2:{t2_motor:.3f} T3:{t3_motor:.3f} F:{feed:.1f}\n"

    def build_home_command(self):
        return "HOME\n"

    def build_estop_command(self):
        return "ESTOP\n"

    # ---------------- Gui lenh ----------------
    def send_raw(self, cmd: str, wait_ack=True, ack_timeout=5.0):
        """Gui 1 chuoi lenh xuong robot. Neu wait_ack=True se cho phan hoi 'ok'."""
        if not self.is_connected or self.ser is None:
            self.log_signal.emit("[CANH BAO] Chua ket noi robot, bo qua lenh: " + cmd.strip())
            return False
        with self.lock:
            try:
                self.ser.write(cmd.encode("utf-8"))
                self.log_signal.emit(f">> {cmd.strip()}")
                if wait_ack:
                    start = time.time()
                    while time.time() - start < ack_timeout:
                        line = self.ser.readline().decode(errors="ignore").strip()
                        if line:
                            self.log_signal.emit(f"<< {line}")
                            if "ok" in line.lower() or "error" in line.lower():
                                return "error" not in line.lower()
                    self.log_signal.emit("[CANH BAO] Khong nhan duoc phan hoi (timeout)")
                return True
            except Exception as e:
                self.log_signal.emit(f"[LOI] Gui lenh that bai: {e}")
                return False

    def send_motor_angles(self, t1_motor, t2_motor, t3_motor, feed, wait_ack=True):
        """Gui truc tiep 3 goc TRUC DONG CO (da qua gear ratio) + feedrate."""
        cmd = self.build_move_command(t1_motor, t2_motor, t3_motor, feed)
        return self.send_raw(cmd, wait_ack=wait_ack)

    def home(self):
        return self.send_raw(self.build_home_command(), wait_ack=True, ack_timeout=15.0)

    def emergency_stop(self):
        return self.send_raw(self.build_estop_command(), wait_ack=False)
