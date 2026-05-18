# ============================================================
#  gui/control_panel.py — Giao diện điều khiển Robot Delta (PyQt5)
# ============================================================
import sys
import logging
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QDoubleSpinBox, QPushButton, QComboBox,
    QGroupBox, QStatusBar, QTextEdit, QSplitter
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, pyqtSlot
from PyQt5.QtGui import QFont, QColor, QPalette

from config.settings import (
    X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX,
    PID_KP, PID_KI, PID_KD
)
from control.uart_comm import UARTComm
from kinematics.inverse_kinematics import inverse_kinematics
from control.pid import PIDController

logger = logging.getLogger(__name__)


# ==============================================================
#  Widget: Spinbox có nhãn
# ==============================================================
class LabeledSpinBox(QWidget):
    def __init__(self, label: str, min_val: float, max_val: float,
                 default: float = 0.0, suffix: str = " mm", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        lbl = QLabel(label)
        lbl.setFixedWidth(30)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setFont(QFont("Consolas", 11, QFont.Bold))

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(min_val, max_val)
        self.spinbox.setValue(default)
        self.spinbox.setSuffix(suffix)
        self.spinbox.setDecimals(2)
        self.spinbox.setSingleStep(1.0)
        self.spinbox.setFixedHeight(36)

        layout.addWidget(lbl)
        layout.addWidget(self.spinbox)

    @property
    def value(self) -> float:
        return self.spinbox.value()


# ==============================================================
#  Cửa sổ chính
# ==============================================================
class ControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.uart   = UARTComm(on_ready=self._on_stm32_ready)
        self.pid_x  = PIDController()
        self.pid_y  = PIDController()
        self.pid_z  = PIDController()

        self._setup_ui()
        self._setup_style()
        self._refresh_ports()

    # ----------------------------------------------------------
    #  Xây dựng UI
    # ----------------------------------------------------------
    def _setup_ui(self):
        self.setWindowTitle("🤖  Delta Robot Controller")
        self.setMinimumSize(800, 600)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)

        # ── Tiêu đề ──────────────────────────────────────────
        title = QLabel("DELTA ROBOT CONTROLLER")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Consolas", 18, QFont.Bold))
        root.addWidget(title)

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter)

        # ── Cột trái ─────────────────────────────────────────
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(10)

        left_layout.addWidget(self._build_uart_group())
        left_layout.addWidget(self._build_position_group())
        left_layout.addWidget(self._build_pid_group())
        left_layout.addWidget(self._build_angle_group())
        left_layout.addStretch()

        splitter.addWidget(left)

        # ── Cột phải: Log ────────────────────────────────────
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)

        log_label = QLabel("📋  Log")
        log_label.setFont(QFont("Consolas", 11, QFont.Bold))
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFont(QFont("Consolas", 9))

        btn_clear = QPushButton("Xóa Log")
        btn_clear.clicked.connect(self.log_box.clear)

        right_layout.addWidget(log_label)
        right_layout.addWidget(self.log_box)
        right_layout.addWidget(btn_clear)

        splitter.addWidget(right)
        splitter.setSizes([480, 300])

        # ── Status bar ────────────────────────────────────────
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._set_status("Chưa kết nối", error=True)

    # ----------------------------------------------------------
    def _build_uart_group(self) -> QGroupBox:
        grp = QGroupBox("🔌  Kết nối UART")
        layout = QHBoxLayout(grp)

        self.combo_port = QComboBox()
        self.combo_port.setFixedHeight(34)

        btn_refresh = QPushButton("🔄")
        btn_refresh.setFixedSize(34, 34)
        btn_refresh.setToolTip("Quét lại cổng COM")
        btn_refresh.clicked.connect(self._refresh_ports)

        self.btn_connect = QPushButton("Kết nối")
        self.btn_connect.setFixedHeight(34)
        self.btn_connect.clicked.connect(self._toggle_connection)

        layout.addWidget(QLabel("Cổng:"))
        layout.addWidget(self.combo_port, 1)
        layout.addWidget(btn_refresh)
        layout.addWidget(self.btn_connect)
        return grp

    def _build_position_group(self) -> QGroupBox:
        grp = QGroupBox("📍  Vị trí End-Effector")
        layout = QVBoxLayout(grp)

        self.spin_x = LabeledSpinBox("X", X_MIN, X_MAX, 0.0)
        self.spin_y = LabeledSpinBox("Y", Y_MIN, Y_MAX, 0.0)
        self.spin_z = LabeledSpinBox("Z", Z_MIN, Z_MAX, 210.0)

        self.btn_send = QPushButton("▶  Gửi lệnh")
        self.btn_send.setFixedHeight(40)
        self.btn_send.setEnabled(False)
        self.btn_send.clicked.connect(self._send_command)

        self.btn_home = QPushButton("🏠  Home (0, 0, 210)")
        self.btn_home.setFixedHeight(36)
        self.btn_home.clicked.connect(self._go_home)

        layout.addWidget(self.spin_x)
        layout.addWidget(self.spin_y)
        layout.addWidget(self.spin_z)
        layout.addWidget(self.btn_send)
        layout.addWidget(self.btn_home)
        return grp

    def _build_pid_group(self) -> QGroupBox:
        grp = QGroupBox("⚙️  Thông số PID")
        grid = QGridLayout(grp)

        def make_pid_spin(default):
            sb = QDoubleSpinBox()
            sb.setRange(0.0, 100.0)
            sb.setDecimals(4)
            sb.setSingleStep(0.01)
            sb.setValue(default)
            sb.setFixedHeight(30)
            return sb

        for col, label in enumerate(["Kp", "Ki", "Kd"]):
            grid.addWidget(QLabel(label), 0, col, Qt.AlignCenter)

        self.pid_kp = make_pid_spin(PID_KP)
        self.pid_ki = make_pid_spin(PID_KI)
        self.pid_kd = make_pid_spin(PID_KD)

        grid.addWidget(self.pid_kp, 1, 0)
        grid.addWidget(self.pid_ki, 1, 1)
        grid.addWidget(self.pid_kd, 1, 2)

        btn_apply = QPushButton("Áp dụng PID")
        btn_apply.clicked.connect(self._apply_pid)
        grid.addWidget(btn_apply, 2, 0, 1, 3)
        return grp

    def _build_angle_group(self) -> QGroupBox:
        grp = QGroupBox("📐  Góc khớp tính được (độ)")
        grid = QGridLayout(grp)

        for col, label in enumerate(["θ₁", "θ₂", "θ₃"]):
            lbl = QLabel(label)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Consolas", 11, QFont.Bold))
            grid.addWidget(lbl, 0, col)

        self.lbl_t1 = QLabel("—")
        self.lbl_t2 = QLabel("—")
        self.lbl_t3 = QLabel("—")
        for col, lbl in enumerate([self.lbl_t1, self.lbl_t2, self.lbl_t3]):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont("Consolas", 13))
            grid.addWidget(lbl, 1, col)

        return grp

    # ----------------------------------------------------------
    #  Style
    # ----------------------------------------------------------
    def _setup_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
            }
            QGroupBox {
                border: 1px solid #45475a;
                border-radius: 8px;
                margin-top: 8px;
                padding: 8px;
                font-weight: bold;
                color: #89b4fa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
            }
            QPushButton {
                background-color: #313244;
                color: #cdd6f4;
                border: 1px solid #45475a;
                border-radius: 6px;
                padding: 4px 12px;
                font-family: Consolas;
            }
            QPushButton:hover  { background-color: #45475a; }
            QPushButton:pressed{ background-color: #89b4fa; color: #1e1e2e; }
            QPushButton#send   { background-color: #a6e3a1; color: #1e1e2e; font-weight: bold; }
            QDoubleSpinBox, QComboBox {
                background-color: #313244;
                color: #cdd6f4;
                border: 1px solid #45475a;
                border-radius: 4px;
                padding: 2px 6px;
                font-family: Consolas;
            }
            QTextEdit {
                background-color: #11111b;
                color: #a6e3a1;
                border: 1px solid #45475a;
                border-radius: 6px;
                font-family: Consolas;
            }
            QLabel { color: #cdd6f4; }
            QStatusBar { background-color: #181825; color: #cdd6f4; }
        """)
        self.btn_send.setObjectName("send")

    # ----------------------------------------------------------
    #  Logic
    # ----------------------------------------------------------
    def _refresh_ports(self):
        self.combo_port.clear()
        ports = UARTComm.list_ports()
        if ports:
            self.combo_port.addItems(ports)
        else:
            self.combo_port.addItem("Không tìm thấy cổng")
        self._log("🔄 Đã quét cổng COM")

    def _toggle_connection(self):
        if self.uart.is_connected:
            self.uart.disconnect()
            self.btn_connect.setText("Kết nối")
            self.btn_send.setEnabled(False)
            self._set_status("Đã ngắt kết nối", error=True)
            self._log("🔌 Đã ngắt kết nối UART")
        else:
            self.uart.port = self.combo_port.currentText()
            if self.uart.connect():
                self.btn_connect.setText("Ngắt kết nối")
                self.btn_send.setEnabled(False)   # chờ STM32 READY mới bật
                self._set_status(f"⏳ Đã kết nối {self.uart.port} — Chờ STM32 homing...", error=False)
                self._log(f"✅ Kết nối thành công: {self.uart.port}")
                self._log("⏳ Đang chờ STM32 homing xong...")
            else:
                self._set_status("Kết nối thất bại!", error=True)
                self._log(f"❌ Không thể kết nối cổng {self.uart.port}")

    def _on_stm32_ready(self):
        """Callback được gọi khi STM32 gửi 'READY' sau khi homing xong."""
        # Gọi từ thread phụ → dùng QTimer để cập nhật UI an toàn
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(0, self._handle_ready)

    def _handle_ready(self):
        self.btn_send.setEnabled(True)
        self._set_status("✅ STM32 READY — Sẵn sàng điều khiển!", error=False)
        self._log("🟢 STM32 đã homing xong, có thể gửi lệnh!")

    def _send_command(self):
        x = self.spin_x.value
        y = self.spin_y.value
        z = self.spin_z.value

        if not self.uart.stm_ready:
            self._log("⚠️  STM32 chưa READY, vui lòng chờ homing xong!")
            return

        try:
            t1, t2, t3 = inverse_kinematics(x, y, z)
            self.lbl_t1.setText(f"{t1:.2f}°")
            self.lbl_t2.setText(f"{t2:.2f}°")
            self.lbl_t3.setText(f"{t3:.2f}°")
            ok = self.uart.send_angles(t1, t2, t3)
            if ok:
                self._log(f"📤 Gửi → X={x:.1f} Y={y:.1f} Z={z:.1f} | θ=({t1:.2f}°, {t2:.2f}°, {t3:.2f}°)")
                self._set_status(f"Đã gửi: ({x:.1f}, {y:.1f}, {z:.1f})")
            else:
                self._log("❌ Gửi thất bại!")
        except ValueError as e:
            self._log(f"⚠️  {e}")
            self._set_status(str(e), error=True)

    def _go_home(self):
        self.spin_x.spinbox.setValue(0.0)
        self.spin_y.spinbox.setValue(0.0)
        self.spin_z.spinbox.setValue(220.0)
        self._send_command()

    def _apply_pid(self):
        kp = self.pid_kp.value()
        ki = self.pid_ki.value()
        kd = self.pid_kd.value()
        for pid in (self.pid_x, self.pid_y, self.pid_z):
            pid.tune(kp, ki, kd)
        self._log(f"⚙️  PID cập nhật: Kp={kp} Ki={ki} Kd={kd}")

    # ----------------------------------------------------------
    #  Tiện ích
    # ----------------------------------------------------------
    def _log(self, msg: str):
        self.log_box.append(msg)

    def _set_status(self, msg: str, error: bool = False):
        color = "#f38ba8" if error else "#a6e3a1"
        self.status_bar.showMessage(msg)
        self.status_bar.setStyleSheet(f"color: {color};")

    def closeEvent(self, event):
        self.uart.disconnect()
        event.accept()
