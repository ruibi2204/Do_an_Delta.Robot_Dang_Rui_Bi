from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QSlider,
)

from shared_state import BaseTabWindow, FnWorker, inverse_kinematics, home_and_wait, save_config
from kinematics.manual_kinematic import LinearMotionWorker

# Giới hạn tốc độ điều chỉnh được qua thanh trượt (mm/s)
SPEED_MIN = 5
SPEED_MAX = 150
SPEED_DEFAULT = 40


class ManualWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "THỦ CÔNG")
        if not hasattr(self.ctx, "jog_speed"):
            self.ctx.jog_speed = SPEED_DEFAULT
        self._motion_worker = None
        self._build_ui()
        self._update_pos_display()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_pos_display()
        self._set_manual_busy(self.ctx.busy)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        xy_box = QGroupBox("(X / Y)")
        xy_layout = QGridLayout()
        xy_layout.setSpacing(10)

        def mk_jog_btn(text):
            b = QPushButton(text)
            b.setObjectName("jogBtn")
            b.setMinimumSize(140, 110)
            return b

        self.btn_y_plus = mk_jog_btn("▲\nX")
        self.btn_y_minus = mk_jog_btn("▼\nX")
        self.btn_x_minus = mk_jog_btn("◀\nY")
        self.btn_x_plus = mk_jog_btn("▶\nY")
        self.btn_center_info = QLabel("XY")
        self.btn_center_info.setAlignment(Qt.AlignCenter)
        self.btn_center_info.setStyleSheet("font-size:22pt; font-weight:800; color:#333333;")

        xy_layout.addWidget(self.btn_y_plus, 0, 1)
        xy_layout.addWidget(self.btn_x_minus, 1, 0)
        xy_layout.addWidget(self.btn_center_info, 1, 1)
        xy_layout.addWidget(self.btn_x_plus, 1, 2)
        xy_layout.addWidget(self.btn_y_minus, 2, 1)
        xy_box.setLayout(xy_layout)

        self.btn_y_plus.clicked.connect(lambda: self.jog(dx=self.ctx.jog_step_xy))
        self.btn_y_minus.clicked.connect(lambda: self.jog(dx=-self.ctx.jog_step_xy))
        self.btn_x_plus.clicked.connect(lambda: self.jog(dy=self.ctx.jog_step_xy))
        self.btn_x_minus.clicked.connect(lambda: self.jog(dy=-self.ctx.jog_step_xy))

        z_box = QGroupBox("(Z)")
        z_layout = QVBoxLayout()
        self.btn_z_plus = mk_jog_btn("⤒\nZ")
        self.btn_z_minus = mk_jog_btn("⤓\nZ")
        self.btn_z_plus.setMinimumSize(140, 150)
        self.btn_z_minus.setMinimumSize(140, 150)
        z_layout.addWidget(self.btn_z_plus)
        z_layout.addWidget(self.btn_z_minus)
        z_box.setLayout(z_layout)

        self.btn_z_plus.clicked.connect(lambda: self.jog(dz=-self.ctx.jog_step_z))
        self.btn_z_minus.clicked.connect(lambda: self.jog(dz=self.ctx.jog_step_z))

        left_col = QVBoxLayout()
        left_col.addWidget(xy_box)

        mid_col = QVBoxLayout()
        mid_col.addWidget(z_box)

        step_box = QGroupBox("BƯỚC DI CHUYỂN (mm/step)")
        step_layout = QGridLayout()
        step_layout.addWidget(QLabel("XY:"), 0, 0)
        self.spin_step_xy = QDoubleSpinBox()
        self.spin_step_xy.setRange(0.1, 50.0)
        self.spin_step_xy.setValue(self.ctx.jog_step_xy)
        self.spin_step_xy.setSingleStep(0.5)
        self.spin_step_xy.valueChanged.connect(self._on_step_xy_changed)
        step_layout.addWidget(self.spin_step_xy, 0, 1)

        step_layout.addWidget(QLabel("Z:"), 1, 0)
        self.spin_step_z = QDoubleSpinBox()
        self.spin_step_z.setRange(0.1, 50.0)
        self.spin_step_z.setValue(self.ctx.jog_step_z)
        self.spin_step_z.setSingleStep(0.5)
        self.spin_step_z.valueChanged.connect(self._on_step_z_changed)
        step_layout.addWidget(self.spin_step_z, 1, 1)
        step_box.setLayout(step_layout)
        mid_col.addWidget(step_box)

        # ---- Thanh trượt điều chỉnh tốc độ (kéo để chỉnh, không rung) ----
        speed_box = QGroupBox("TỐC ĐỘ (mm/s)")
        speed_layout = QVBoxLayout()

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setObjectName("speedSlider")
        self.slider_speed.setMinimum(SPEED_MIN)
        self.slider_speed.setMaximum(SPEED_MAX)
        self.slider_speed.setValue(int(self.ctx.jog_speed))
        self.slider_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_speed.setTickInterval(10)
        self.slider_speed.valueChanged.connect(self._on_speed_changed)

        self.lbl_speed_value = QLabel(f"{int(self.ctx.jog_speed)} mm/s")
        self.lbl_speed_value.setAlignment(Qt.AlignCenter)
        self.lbl_speed_value.setStyleSheet("font-size:13pt; font-weight:600; color:#0a5b8a;")

        speed_layout.addWidget(self.slider_speed)
        speed_layout.addWidget(self.lbl_speed_value)
        speed_box.setLayout(speed_layout)
        mid_col.addWidget(speed_box)

        right_col = QVBoxLayout()

        pos_box = QGroupBox("TỌA ĐỘ HIỆN TẠI (mm)")
        pos_layout = QVBoxLayout()
        self.lbl_pos = QLabel("X:0.00  Y:0.00  Z:0.00")
        self.lbl_pos.setObjectName("posDisplay")
        self.lbl_pos.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(self.lbl_pos)
        self.lbl_angles = QLabel("θ1:--°  θ2:--°  θ3:--°")
        self.lbl_angles.setAlignment(Qt.AlignCenter)
        self.lbl_angles.setStyleSheet("color:#333333; font-size:12pt;")
        pos_layout.addWidget(self.lbl_angles)
        pos_box.setLayout(pos_layout)
        right_col.addWidget(pos_box)

        home_box = QGroupBox("HOME")
        home_layout = QVBoxLayout()

        self.btn_home_wait = QPushButton("HOME")
        self.btn_home_wait.setObjectName("homeBtn")
        self.btn_home_wait.clicked.connect(self.on_home_wait)
        home_layout.addWidget(self.btn_home_wait)

        self.btn_goto_saved_home = QPushButton("DI CHUYỂN ĐẾN HOME ĐÃ LƯU")
        self.btn_goto_saved_home.setObjectName("homeBtn")
        self.btn_goto_saved_home.clicked.connect(self.on_goto_saved_home)
        home_layout.addWidget(self.btn_goto_saved_home)

        self.btn_save_home = QPushButton("LƯU HOME AN TOÀN")
        self.btn_save_home.setObjectName("saveBtn")
        self.btn_save_home.clicked.connect(self.on_save_home)
        home_layout.addWidget(self.btn_save_home)

        self.lbl_saved_home = QLabel(self._home_str())
        self.lbl_saved_home.setStyleSheet("color:#0a6b3a; font-size:12pt;")
        home_layout.addWidget(self.lbl_saved_home)

        home_box.setLayout(home_layout)
        right_col.addWidget(home_box)

        self.lbl_manual_status = QLabel("SẴN SÀNG.")
        self.lbl_manual_status.setWordWrap(True)
        self.lbl_manual_status.setStyleSheet("color:#333333; font-size:11pt;")
        right_col.addWidget(self.lbl_manual_status)
        right_col.addStretch()

        root.addLayout(left_col, 3)
        root.addLayout(mid_col, 2)
        root.addLayout(right_col, 3)

    def _home_str(self):
        x, y, z = self.ctx.home_pos
        return f"Home đã lưu: X={x:.2f}  Y={y:.2f}  Z={z:.2f}"

    def _on_step_xy_changed(self, v):
        self.ctx.jog_step_xy = v

    def _on_step_z_changed(self, v):
        self.ctx.jog_step_z = v

    def _on_speed_changed(self, v):
        self.ctx.jog_speed = float(v)
        self.lbl_speed_value.setText(f"{v} mm/s")

    def _update_pos_display(self):
        x, y, z = self.ctx.current_pos
        self.lbl_pos.setText(f"X:{x:.2f}  Y:{y:.2f}  Z:{z:.2f}")
        try:
            t1, t2, t3 = inverse_kinematics(x, y, z)
            self.lbl_angles.setText(f"θ1:{t1:.2f}°  θ2:{t2:.2f}°  θ3:{t3:.2f}°")
        except Exception:
            self.lbl_angles.setText("θ1:--°  θ2:--°  θ3:--°  (ngoài vùng làm việc)")

    def _set_manual_busy(self, busy, msg=""):
        self.ctx.busy = busy
        for b in (self.btn_x_plus, self.btn_x_minus, self.btn_y_plus, self.btn_y_minus,
                  self.btn_z_plus, self.btn_z_minus, self.btn_home_wait,
                  self.btn_goto_saved_home, self.btn_save_home):
            b.setEnabled(not busy)
        if msg:
            self.lbl_manual_status.setText(msg)

    # ------------------------------------------------------------------
    # Chuyển động thẳng (straight-line, không rung) dùng LinearMotionWorker
    # ------------------------------------------------------------------
    def _run_linear_motion(self, target_pos, busy_msg, on_finished):
        """Khởi chạy 1 quỹ đạo thẳng từ vị trí hiện tại tới target_pos, dùng
        tốc độ đang đặt trên thanh trượt. `on_finished(ok)` được gọi khi xong."""
        if self.ctx.busy:
            return
        if self._motion_worker is not None and self._motion_worker.isRunning():
            return

        self._set_manual_busy(True, busy_msg)

        worker = LinearMotionWorker(
            send_fn=self.ctx.planner.send_position,
            start_pos=list(self.ctx.current_pos),
            end_pos=list(target_pos),
            speed_mm_s=self.ctx.jog_speed,
        )
        worker.point_reached.connect(self._on_traj_point)
        worker.finished_ok.connect(lambda ok: self._on_traj_finished(ok, target_pos, on_finished))
        worker.error.connect(self._on_traj_error)
        self._motion_worker = worker
        self._track_worker(worker)
        worker.start()

    def _on_traj_point(self, pt):
        # Cập nhật hiển thị vị trí/góc theo thời gian thực khi robot đang
        # di chuyển trên quỹ đạo thẳng.
        self.ctx.current_pos = list(pt)
        self._update_pos_display()

    def _on_traj_finished(self, ok, target_pos, on_finished):
        self._motion_worker = None
        self._set_manual_busy(False)
        if ok:
            self.ctx.current_pos = list(target_pos)
            self._update_pos_display()
        on_finished(ok)

    def _on_traj_error(self, err):
        self._motion_worker = None
        self._set_manual_busy(False)
        self.lbl_manual_status.setText(f"⚠ Lỗi: {err}")

    def jog(self, dx=0.0, dy=0.0, dz=0.0):
        if self.ctx.busy:
            return
        new_pos = [self.ctx.current_pos[0] + dx, self.ctx.current_pos[1] + dy, self.ctx.current_pos[2] + dz]

        try:
            inverse_kinematics(*new_pos)
        except Exception as e:
            self.lbl_manual_status.setText(f"⚠ Ngoài vùng làm việc: {e}")
            return

        def _finished(ok):
            if ok:
                self.lbl_manual_status.setText("Sẵn sàng.")
            else:
                self.lbl_manual_status.setText("⚠ Gửi lệnh thất bại (kiểm tra kết nối UART Robot).")

        self._run_linear_motion(new_pos, "Đang di chuyển (đường thẳng)...", _finished)

    def on_home_wait(self):
        if self.ctx.busy:
            return
        if not self.ctx.robot_uart.is_connected:
            self.lbl_manual_status.setText(
                "⚠ Chưa kết nối Robot (UART 1). Hãy kết nối ở tab KẾT NỐI trước."
            )
            return

        self._set_manual_busy(
            True, "Đang HOME (chạm 3 công tắc hành trình)... chờ STM32 phản hồi READY"
        )

        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_home_wait_done)
        worker.done_err.connect(self._on_jog_error)
        self._track_worker(worker)

    def _on_jog_error(self, err):
        self._set_manual_busy(False)
        self.lbl_manual_status.setText(f"⚠ Lỗi: {err}")

    def _on_home_wait_done(self, ready):
        self._set_manual_busy(False)
        if ready:
            self.lbl_manual_status.setText(
                "✔ Đã HOME xong (chạm công tắc hành trình, encoder = 0). "
                "Bấm 'DI CHUYỂN ĐẾN HOME ĐÃ LƯU' để về vị trí an toàn."
            )
        else:
            self.lbl_manual_status.setText(
                "⚠ Không nhận được READY từ STM32 (kiểm tra dây UART1 / công tắc hành trình)."
            )

    def on_goto_saved_home(self):
        if self.ctx.busy:
            return

        def _finished(ok):
            if ok:
                self.lbl_manual_status.setText("Sẵn sàng.")
            else:
                self.lbl_manual_status.setText("⚠ Gửi lệnh thất bại (kiểm tra kết nối UART Robot).")

        self._run_linear_motion(
            list(self.ctx.home_pos),
            "Đang di chuyển tới Home đã lưu (đường thẳng)...",
            _finished,
        )

    def on_save_home(self):
        self.ctx.home_pos = list(self.ctx.current_pos)
        self.ctx.planner.HOME = tuple(self.ctx.home_pos)
        self.ctx.planner_dof4.HOME = tuple(self.ctx.home_pos)
        self.ctx.cfg["home_position"] = self.ctx.home_pos
        save_config(self.ctx.cfg)
        self.lbl_saved_home.setText(self._home_str())
        self.lbl_manual_status.setText("💾 Đã lưu vị trí hiện tại làm Home an toàn.")