import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QSpinBox, QLineEdit, QTextEdit, QComboBox,
    QFileDialog, QFrame, QScrollArea, QMessageBox,
)
from PySide6.QtCore import Qt

from shared_state import BaseTabWindow, FnWorker, save_config, UARTComm, SERIAL_AVAILABLE


class ConnectionWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🔌 KẾT NỐI")
        self._build_ui()
        self.ctx.conn_log_callback = self._append_conn_log
        self._refresh_conn_status()

    def showEvent(self, event):
        super().showEvent(event)
        self.ctx.conn_log_callback = self._append_conn_log
        self._refresh_conn_status()

    def _append_conn_log(self, text):
        QTimer.singleShot(0, lambda: self.conn_log.append(text))

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        left_content = QWidget()
        left = QVBoxLayout(left_content)
        left.setContentsMargins(2, 2, 8, 2)
        left.setSpacing(10)

        robot_box = QGroupBox("KẾT NỐI ROBOT (UART 1)")
        robot_box.setObjectName("compactBox")
        rl = QGridLayout()
        rl.setSpacing(6)
        rl.addWidget(QLabel("Cổng COM:"), 0, 0)
        self.combo_robot_port = QComboBox()
        self.combo_robot_port.setEditable(True)
        rl.addWidget(self.combo_robot_port, 0, 1)

        rl.addWidget(QLabel("Baudrate:"), 0, 2)
        self.combo_robot_baud = QComboBox()
        self.combo_robot_baud.addItems(["115200"])
        self.combo_robot_baud.setCurrentText(str(self.ctx.cfg["robot_baud"]))
        rl.addWidget(self.combo_robot_baud, 0, 3)

        self.btn_robot_connect = QPushButton("🔌 KẾT NỐI")
        self.btn_robot_connect.setObjectName("connectBtn")
        self.btn_robot_connect.clicked.connect(self.on_connect_robot)
        rl.addWidget(self.btn_robot_connect, 1, 0, 1, 2)

        self.btn_robot_disconnect = QPushButton("⏻ NGẮT")
        self.btn_robot_disconnect.setObjectName("disconnectBtn")
        self.btn_robot_disconnect.clicked.connect(self.on_disconnect_robot)
        rl.addWidget(self.btn_robot_disconnect, 1, 2, 1, 2)

        self.lbl_robot_status = QLabel("● Chưa kết nối")
        self.lbl_robot_status.setObjectName("statusBad")
        rl.addWidget(self.lbl_robot_status, 2, 0, 1, 4)
        robot_box.setLayout(rl)
        left.addWidget(robot_box)

        pneu_box = QGroupBox("KẾT NỐI THIẾT BỊ PHỤ (UART 2)")
        pneu_box.setObjectName("compactBox")
        pl = QGridLayout()
        pl.setSpacing(6)
        pl.addWidget(QLabel("Cổng COM:"), 0, 0)
        self.combo_pneu_port = QComboBox()
        self.combo_pneu_port.setEditable(True)
        pl.addWidget(self.combo_pneu_port, 0, 1)

        pl.addWidget(QLabel("Baudrate:"), 0, 2)
        self.combo_pneu_baud = QComboBox()
        self.combo_pneu_baud.addItems(["115200"])
        self.combo_pneu_baud.setCurrentText(str(self.ctx.cfg["pneumatic_baud"]))
        pl.addWidget(self.combo_pneu_baud, 0, 3)

        self.btn_pneu_connect = QPushButton("🔌 KẾT NỐI")
        self.btn_pneu_connect.setObjectName("connectBtn")
        self.btn_pneu_connect.clicked.connect(self.on_connect_pneu)
        pl.addWidget(self.btn_pneu_connect, 1, 0, 1, 2)

        self.btn_pneu_disconnect = QPushButton("⏻ NGẮT")
        self.btn_pneu_disconnect.setObjectName("disconnectBtn")
        self.btn_pneu_disconnect.clicked.connect(self.on_disconnect_pneu)
        pl.addWidget(self.btn_pneu_disconnect, 1, 2, 1, 2)

        self.lbl_pneu_status = QLabel("● Chưa kết nối")
        self.lbl_pneu_status.setObjectName("statusBad")
        pl.addWidget(self.lbl_pneu_status, 2, 0, 1, 4)
        pneu_box.setLayout(pl)
        left.addWidget(pneu_box)

        left.addWidget(self._build_device_control_box())
        left.addStretch()

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QFrame.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setWidget(left_content)

        right = QVBoxLayout()
        right.setSpacing(10)

        cam_box = QGroupBox("CẤU HÌNH CAMERA")
        cam_box.setObjectName("compactBox")
        cl = QGridLayout()
        cl.setSpacing(6)
        cl.addWidget(QLabel("Chỉ số Camera:"), 0, 0)
        self.spin_cam_index = QSpinBox()
        self.spin_cam_index.setRange(0, 10)
        self.spin_cam_index.setValue(self.ctx.cfg.get("camera_index", 0))
        cl.addWidget(self.spin_cam_index, 0, 1)

        cl.addWidget(QLabel("File hiệu chuẩn (.npz):"), 1, 0)
        self.edit_calib_path = QLineEdit(self.ctx.cfg.get("calib_file", ""))
        cl.addWidget(self.edit_calib_path, 1, 1)
        self.btn_browse_calib = QPushButton("...")
        self.btn_browse_calib.setMaximumWidth(44)
        self.btn_browse_calib.clicked.connect(self.on_browse_calib)
        cl.addWidget(self.btn_browse_calib, 1, 2)
        cam_box.setLayout(cl)
        right.addWidget(cam_box)

        util_box = QGroupBox("TIỆN ÍCH")
        util_box.setObjectName("compactBox")
        ul = QHBoxLayout()
        ul.setSpacing(8)
        self.btn_refresh_ports = QPushButton("🔄 LÀM MỚI CỔNG COM")
        self.btn_refresh_ports.clicked.connect(self.on_refresh_ports)
        ul.addWidget(self.btn_refresh_ports)
        self.btn_save_conn_cfg = QPushButton("💾 LƯU CẤU HÌNH")
        self.btn_save_conn_cfg.setObjectName("saveBtn")
        self.btn_save_conn_cfg.clicked.connect(self.on_save_conn_cfg)
        ul.addWidget(self.btn_save_conn_cfg)
        util_box.setLayout(ul)
        right.addWidget(util_box)

        conn_log_box = QGroupBox("NHẬT KÝ KẾT NỐI")
        conn_log_box.setObjectName("compactBox")
        conn_log_layout = QVBoxLayout()
        self.conn_log = QTextEdit()
        self.conn_log.setReadOnly(True)
        conn_log_layout.addWidget(self.conn_log)
        conn_log_box.setLayout(conn_log_layout)
        right.addWidget(conn_log_box, 3)

        root.addWidget(left_scroll, 1)
        root.addLayout(right, 1)

        self.on_refresh_ports()

    # ------------------------------------------------------------------
    def _build_device_control_box(self):
        outer_box = QGroupBox("ĐIỀU KHIỂN THIẾT BỊ PHỤ (UART 2)")
        outer_box.setObjectName("compactBox")
        outer_layout = QVBoxLayout()
        outer_layout.setSpacing(8)

        pump_box = QGroupBox("MÁY BƠM")
        pump_box.setObjectName("compactBox")
        pump_layout = QHBoxLayout()
        pump_layout.setSpacing(8)

        self.lbl_pump_state = QLabel("● TẮT")
        self.lbl_pump_state.setObjectName("deviceStateOff")
        self.lbl_pump_state.setMinimumWidth(60)
        pump_layout.addWidget(self.lbl_pump_state)

        self.btn_pump_on = QPushButton("▶ BẬT")
        self.btn_pump_on.setObjectName("deviceOnBtn")
        self.btn_pump_on.clicked.connect(self.on_pump_on)
        self.btn_pump_off = QPushButton("■ TẮT")
        self.btn_pump_off.setObjectName("deviceOffBtn")
        self.btn_pump_off.clicked.connect(self.on_pump_off)
        pump_layout.addWidget(self.btn_pump_on, 1)
        pump_layout.addWidget(self.btn_pump_off, 1)

        pump_box.setLayout(pump_layout)
        outer_layout.addWidget(pump_box)

        turn_box = QGroupBox("BÀN XOAY (PWM)")
        turn_box.setObjectName("compactBox")
        turn_layout = QVBoxLayout()
        turn_layout.setSpacing(6)

        turn_row1 = QHBoxLayout()
        turn_row1.setSpacing(6)
        turn_row1.addWidget(QLabel("PWM:"))
        self.spin_turn_pwm = QSpinBox()
        self.spin_turn_pwm.setRange(0, 255)
        self.spin_turn_pwm.setValue(self.ctx.turn_pwm_value)
        self.spin_turn_pwm.valueChanged.connect(self._on_turn_pwm_changed)
        turn_row1.addWidget(self.spin_turn_pwm, 1)
        self.btn_turn_on = QPushButton("▶ BẬT")
        self.btn_turn_on.setObjectName("deviceOnBtn")
        self.btn_turn_on.clicked.connect(self.on_turn_on)
        self.btn_turn_off = QPushButton("■ TẮT")
        self.btn_turn_off.setObjectName("deviceOffBtn")
        self.btn_turn_off.clicked.connect(self.on_turn_off)
        turn_row1.addWidget(self.btn_turn_on, 1)
        turn_row1.addWidget(self.btn_turn_off, 1)
        turn_layout.addLayout(turn_row1)

        turn_row2 = QHBoxLayout()
        turn_row2.setSpacing(6)
        self.lbl_turn_state = QLabel("● Đang TẮT")
        self.lbl_turn_state.setObjectName("deviceStateOff")
        turn_row2.addWidget(self.lbl_turn_state, 1)
        self.btn_turn_apply_pwm = QPushButton("⟳ ÁP DỤNG PWM")
        self.btn_turn_apply_pwm.setObjectName("deviceApplyBtn")
        self.btn_turn_apply_pwm.clicked.connect(self.on_turn_apply_pwm)
        turn_row2.addWidget(self.btn_turn_apply_pwm)
        turn_layout.addLayout(turn_row2)

        turn_box.setLayout(turn_layout)
        outer_layout.addWidget(turn_box)

        step_box = QGroupBox("BẬC TỰ DO 4 — GÓC QUAY (STEP)")
        step_box.setObjectName("compactBox")
        step_layout = QVBoxLayout()
        step_layout.setSpacing(6)

        step_row1 = QHBoxLayout()
        step_row1.setSpacing(6)
        step_row1.addWidget(QLabel("Góc (độ):"))
        self.spin_step_test_angle = QDoubleSpinBox()
        self.spin_step_test_angle.setRange(-3600.0, 3600.0)
        self.spin_step_test_angle.setSingleStep(5.0)
        self.spin_step_test_angle.setValue(float(self.ctx.cfg.get("step_test_angle", 90.0)))
        self.spin_step_test_angle.valueChanged.connect(self._on_step_test_angle_changed)
        step_row1.addWidget(self.spin_step_test_angle, 1)
        self.btn_step_rotate = QPushButton("↻ QUAY")
        self.btn_step_rotate.setObjectName("deviceApplyBtn")
        self.btn_step_rotate.clicked.connect(self.on_step_rotate)
        step_row1.addWidget(self.btn_step_rotate)
        step_layout.addLayout(step_row1)

        quick_row = QHBoxLayout()
        quick_row.setSpacing(6)
        quick_row.addWidget(QLabel("Nhanh:"))
        for deg, label in ((90, "+90°"), (-90, "-90°"), (180, "+180°")):
            b = QPushButton(label)
            b.setObjectName("deviceQuickBtn")
            b.clicked.connect(lambda checked=False, d=deg: self.ctx.pneu_uart.step_rotate(d))
            quick_row.addWidget(b)
        step_layout.addLayout(quick_row)

        self.lbl_step_state = QLabel("Chưa gửi lệnh quay nào.")
        self.lbl_step_state.setStyleSheet("color:#555555; font-size:9pt;")
        step_layout.addWidget(self.lbl_step_state)

        step_box.setLayout(step_layout)
        outer_layout.addWidget(step_box)

        outer_box.setLayout(outer_layout)

        self._device_ctrl_widgets = [
            self.btn_pump_on, self.btn_pump_off,
            self.btn_turn_on, self.btn_turn_off, self.btn_turn_apply_pwm, self.spin_turn_pwm,
            self.btn_step_rotate, self.spin_step_test_angle,
        ]
        return outer_box

    def on_pump_on(self):
        if self.ctx.pneu_uart.pump_on():
            self.ctx.pump_state = True
            self.lbl_pump_state.setText("● BẬT")
            self.lbl_pump_state.setObjectName("deviceStateOn")
            self._repolish(self.lbl_pump_state)

    def on_pump_off(self):
        if self.ctx.pneu_uart.pump_off():
            self.ctx.pump_state = False
            self.lbl_pump_state.setText("● TẮT")
            self.lbl_pump_state.setObjectName("deviceStateOff")
            self._repolish(self.lbl_pump_state)

    def _on_turn_pwm_changed(self, val):
        self.ctx.turn_pwm_value = val
        self.ctx.cfg["turn_pwm"] = val
        save_config(self.ctx.cfg)

    def on_turn_on(self):
        if self.ctx.pneu_uart.turn_set_speed(self.spin_turn_pwm.value()):
            self.ctx.turn_state = True
            self.lbl_turn_state.setText(f"● Đang BẬT (PWM={self.spin_turn_pwm.value()})")
            self.lbl_turn_state.setObjectName("deviceStateOn")
            self._repolish(self.lbl_turn_state)

    def on_turn_off(self):
        if self.ctx.pneu_uart.turn_off():
            self.ctx.turn_state = False
            self.lbl_turn_state.setText("● Đang TẮT")
            self.lbl_turn_state.setObjectName("deviceStateOff")
            self._repolish(self.lbl_turn_state)

    def on_turn_apply_pwm(self):
        if not self.ctx.turn_state:
            QMessageBox.information(self, "Bàn xoay đang tắt", "Hãy bấm 'BẬT' trước khi áp dụng PWM mới.")
            return
        if self.ctx.pneu_uart.turn_set_speed(self.spin_turn_pwm.value()):
            self.lbl_turn_state.setText(f"● Đang BẬT (PWM={self.spin_turn_pwm.value()})")

    def _on_step_test_angle_changed(self, val):
        self.ctx.cfg["step_test_angle"] = val
        save_config(self.ctx.cfg)

    def on_step_rotate(self):
        deg = self.spin_step_test_angle.value()
        if self.ctx.pneu_uart.step_rotate(deg):
            self.lbl_step_state.setText(f"↻ Đã gửi lệnh quay {deg:.1f}° lúc {time.strftime('%H:%M:%S')}")

    def _repolish(self, widget):
        widget.setStyleSheet("")
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def on_refresh_ports(self):
        ports = UARTComm.list_ports()
        self.combo_robot_port.clear()
        self.combo_robot_port.addItems(ports)
        self.combo_robot_port.setCurrentText(self.ctx.cfg["robot_port"])

        self.combo_pneu_port.clear()
        self.combo_pneu_port.addItems(ports)
        self.combo_pneu_port.setCurrentText(self.ctx.cfg["pneumatic_port"])

    def on_connect_robot(self):
        self.ctx.robot_uart.port = self.combo_robot_port.currentText()
        self.ctx.robot_uart.baud = int(self.combo_robot_baud.currentText())
        self.ctx.robot_uart.dry_run = not SERIAL_AVAILABLE
        worker = FnWorker(self.ctx.robot_uart.connect)
        worker.done_ok.connect(lambda ok: self._refresh_conn_status())
        worker.done_err.connect(lambda err: self._append_conn_log(f"[ERR] {err}"))
        self._track_worker(worker)

    def on_disconnect_robot(self):
        self.ctx.robot_uart.disconnect()
        self._refresh_conn_status()

    def on_connect_pneu(self):
        self.ctx.pneu_uart.port = self.combo_pneu_port.currentText()
        self.ctx.pneu_uart.baud = int(self.combo_pneu_baud.currentText())
        self.ctx.pneu_uart.dry_run = not SERIAL_AVAILABLE
        worker = FnWorker(self.ctx.pneu_uart.connect)
        worker.done_ok.connect(lambda ok: self._refresh_conn_status())
        worker.done_err.connect(lambda err: self._append_conn_log(f"[ERR] {err}"))
        self._track_worker(worker)

    def on_disconnect_pneu(self):
        self.ctx.pneu_uart.disconnect()
        self._refresh_conn_status()

    def _refresh_conn_status(self):
        if self.ctx.robot_uart.is_connected:
            self.lbl_robot_status.setText("● Đã kết nối" + (" (DRY-RUN)" if self.ctx.robot_uart.dry_run else ""))
            self.lbl_robot_status.setObjectName("statusOk")
        else:
            self.lbl_robot_status.setText("● Chưa kết nối")
            self.lbl_robot_status.setObjectName("statusBad")
        self.lbl_robot_status.setStyleSheet("")
        self.lbl_robot_status.style().unpolish(self.lbl_robot_status)
        self.lbl_robot_status.style().polish(self.lbl_robot_status)

        if self.ctx.pneu_uart.is_connected:
            self.lbl_pneu_status.setText("● Đã kết nối" + (" (DRY-RUN)" if self.ctx.pneu_uart.dry_run else ""))
            self.lbl_pneu_status.setObjectName("statusOk")
        else:
            self.lbl_pneu_status.setText("● Chưa kết nối")
            self.lbl_pneu_status.setObjectName("statusBad")
        self.lbl_pneu_status.style().unpolish(self.lbl_pneu_status)
        self.lbl_pneu_status.style().polish(self.lbl_pneu_status)

        if hasattr(self, "_device_ctrl_widgets"):
            for w in self._device_ctrl_widgets:
                w.setEnabled(self.ctx.pneu_uart.is_connected)

    def on_browse_calib(self):
        path, _ = QFileDialog.getOpenFileName(self, "Chọn file hiệu chuẩn", "", "NPZ files (*.npz)")
        if path:
            self.edit_calib_path.setText(path)

    def on_save_conn_cfg(self):
        self.ctx.cfg["robot_port"] = self.combo_robot_port.currentText()
        self.ctx.cfg["robot_baud"] = int(self.combo_robot_baud.currentText())
        self.ctx.cfg["pneumatic_port"] = self.combo_pneu_port.currentText()
        self.ctx.cfg["pneumatic_baud"] = int(self.combo_pneu_baud.currentText())
        self.ctx.cfg["camera_index"] = self.spin_cam_index.value()
        self.ctx.cfg["calib_file"] = self.edit_calib_path.text()
        self.ctx.cfg["jog_step_xy"] = self.ctx.jog_step_xy
        self.ctx.cfg["jog_step_z"] = self.ctx.jog_step_z
        self.ctx.cfg["camera_offset_x"] = self.ctx.offset_x
        self.ctx.cfg["camera_offset_y"] = self.ctx.offset_y
        self.ctx.cfg["turn_pwm"] = self.spin_turn_pwm.value()
        self.ctx.cfg["step_test_angle"] = self.spin_step_test_angle.value()
        self.ctx.cfg["dyn_offset_x"] = self.ctx.dyn_offset_x
        self.ctx.cfg["dyn_offset_y"] = self.ctx.dyn_offset_y
        if save_config(self.ctx.cfg):
            self._append_conn_log("💾 Đã lưu cấu hình kết nối.")
        else:
            self._append_conn_log("⚠ Lưu cấu hình thất bại.")
