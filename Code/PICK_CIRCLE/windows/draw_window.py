import contextlib

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel, QDoubleSpinBox,
    QPushButton, QRadioButton, QButtonGroup, QTextEdit, QMessageBox, QSlider,
)

from shared_state import BaseTabWindow, FnWorker, StreamToSignal
from kinematics.draw_motion import DrawMotionPlanner, DrawMotionError

SHAPE_LINE = "line"
SHAPE_CIRCLE = "circle"

SPINBOX_MAX_W = 130

SPEED_SLIDER_MIN = 1
SPEED_SLIDER_MAX = 300

# Nút hành động thường (Start/Run/Chọn file/...) - màu XANH DƯƠNG gradient, đồng bộ
# với windows/board_calib_window.py.
GRAY_BTN_STYLE = """
QPushButton#grayActionBtn {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #3a9bf0, stop:1 #1670d4);
    color: #ffffff;
    border: 2px solid #1670d4;
    border-radius: 12px;
    padding: 12px 18px;
    font-weight: 700;
    font-size: 14pt;
    min-height: 40px;
}
QPushButton#grayActionBtn:hover {
    background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #55aef6, stop:1 #2285e6);
    border-color: #1262b4;
}
QPushButton#grayActionBtn:pressed { background-color: #1262b4; }
QPushButton#grayActionBtn:disabled { background-color: #d9dde3; border-color: #c3c9d1; color: #8e98a6; }
"""

# Nút DỪNG / HUỶ (Stop / Cancel) - màu XÁM, đồng bộ với
# windows/board_calib_window.py.
RED_BTN_STYLE = """
QPushButton#redActionBtn {
    background-color: #e3e7ec;
    color: #3d4b5e;
    border: 2px solid #c3c9d1;
    border-radius: 12px;
    padding: 12px 18px;
    font-weight: 700;
    font-size: 14pt;
    min-height: 40px;
}
QPushButton#redActionBtn:hover { background-color: #d3d9e0; border-color: #9aa7b8; }
QPushButton#redActionBtn:pressed { background-color: #bcc2ca; color: #1f2d40; }
QPushButton#redActionBtn:disabled { background-color: #d9dde3; border-color: #c3c9d1; color: #8e98a6; }
"""


class DrawWindow(BaseTabWindow):
    """
    Cửa sổ VẼ HÌNH: cho phép robot vẽ một ĐƯỜNG THẲNG hoặc một HÌNH TRÒN
    với các tham số nhập tay và tốc độ vẽ (mm/s, chỉnh bằng ô số hoặc kéo
    thanh trượt), có nút START/STOP.
    """

    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "✏  VẼ ĐƯỜNG THẲNG VÀ HÌNH TRÒN")
        self.draw_motion = DrawMotionPlanner(self.ctx.planner)
        self._build_ui()

    def showEvent(self, event):
        super().showEvent(event)
        self._set_busy_ui(self.ctx.busy)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)
        root.setSpacing(16)

        left_col = QVBoxLayout()
        left_col.setSpacing(10)

        # ---------------- Chọn hình cần vẽ ----------------
        shape_box = QGroupBox("CHỌN HÌNH VẼ")
        shape_box.setObjectName("compactBox")
        shape_layout = QVBoxLayout()
        self.radio_line = QRadioButton("ĐƯỜNG THẲNG")
        self.radio_circle = QRadioButton("HÌNH TRÒN")
        self.radio_line.setChecked(True)
        self.radio_group_shape = QButtonGroup(self)
        self.radio_group_shape.addButton(self.radio_line, 0)
        self.radio_group_shape.addButton(self.radio_circle, 1)
        self.radio_line.toggled.connect(self._on_shape_changed)
        shape_layout.addWidget(self.radio_line)
        shape_layout.addWidget(self.radio_circle)
        shape_box.setLayout(shape_layout)
        left_col.addWidget(shape_box)

        # ---------------- Tham số ĐƯỜNG THẲNG ----------------
        self.line_box = QGroupBox("THÔNG TIN")
        self.line_box.setObjectName("compactBox")
        line_grid = QGridLayout()
        line_grid.setSpacing(6)
        line_grid.setColumnMinimumWidth(0, 60)

        self.spin_line_x1 = self._make_spin(-500, 500, 0.0)
        self.spin_line_y1 = self._make_spin(-500, 500, 0.0)
        self.spin_line_x2 = self._make_spin(-500, 500, 100.0)
        self.spin_line_y2 = self._make_spin(-500, 500, 0.0)
        self.spin_line_z = self._make_spin(0, 500, 300.0)

        line_grid.addWidget(QLabel("X1 (mm):"), 0, 0); line_grid.addWidget(self.spin_line_x1, 0, 1)
        line_grid.addWidget(QLabel("Y1 (mm):"), 1, 0); line_grid.addWidget(self.spin_line_y1, 1, 1)
        line_grid.addWidget(QLabel("X2 (mm):"), 2, 0); line_grid.addWidget(self.spin_line_x2, 2, 1)
        line_grid.addWidget(QLabel("Y2 (mm):"), 3, 0); line_grid.addWidget(self.spin_line_y2, 3, 1)
        line_grid.addWidget(QLabel("Z VẼ (mm):"), 4, 0); line_grid.addWidget(self.spin_line_z, 4, 1)
        self.line_box.setLayout(line_grid)
        left_col.addWidget(self.line_box)

        # ---------------- Tham số HÌNH TRÒN ----------------
        self.circle_box = QGroupBox("THÔNG TIN")
        self.circle_box.setObjectName("compactBox")
        circle_grid = QGridLayout()
        circle_grid.setSpacing(6)
        circle_grid.setColumnMinimumWidth(0, 60)

        self.spin_circle_cx = self._make_spin(-500, 500, 0.0)
        self.spin_circle_cy = self._make_spin(-500, 500, 0.0)
        self.spin_circle_r = self._make_spin(1, 300, 50.0)
        self.spin_circle_z = self._make_spin(0, 500, 300.0)

        circle_grid.addWidget(QLabel("Xo (mm):"), 0, 0); circle_grid.addWidget(self.spin_circle_cx, 0, 1)
        circle_grid.addWidget(QLabel("Yo (mm):"), 1, 0); circle_grid.addWidget(self.spin_circle_cy, 1, 1)
        circle_grid.addWidget(QLabel("R (mm):"), 2, 0); circle_grid.addWidget(self.spin_circle_r, 2, 1)
        circle_grid.addWidget(QLabel("Z VẼ (mm):"), 3, 0); circle_grid.addWidget(self.spin_circle_z, 3, 1)
        self.circle_box.setLayout(circle_grid)
        left_col.addWidget(self.circle_box)

        # ---------------- Tốc độ vẽ (ô số + thanh kéo) + Hạ thêm ----------------
        speed_box = QGroupBox("TỐC ĐỘ VẼ")
        speed_box.setObjectName("compactBox")
        speed_grid = QGridLayout()
        speed_grid.setSpacing(6)
        speed_grid.setColumnMinimumWidth(0, 60)

        self.spin_speed = self._make_spin(1, 300, 30.0)
        speed_grid.addWidget(QLabel("TỐC ĐỘ:"), 0, 0)
        speed_grid.addWidget(self.spin_speed, 0, 1)
        speed_grid.addWidget(QLabel("mm/s"), 0, 2)

        # Thanh trượt tốc độ - kéo để chỉnh nhanh, đồng bộ 2 chiều với ô số.
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setObjectName("drawSpeedSlider")
        self.slider_speed.setMinimum(SPEED_SLIDER_MIN)
        self.slider_speed.setMaximum(SPEED_SLIDER_MAX)
        self.slider_speed.setValue(int(round(self.spin_speed.value())))
        self.slider_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_speed.setTickInterval(30)
        speed_grid.addWidget(self.slider_speed, 1, 0, 1, 3)

        self.spin_speed.valueChanged.connect(self._on_speed_spin_changed)
        self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)

        self.spin_dip = self._make_spin(0, 50, 2.0)
        self.spin_dip.setToolTip(
            "Sau khi hạ tới Z vẽ (Z pick), robot sẽ hạ thêm đoạn này (mm) "
            "trước khi bắt đầu vẽ, giúp đầu vẽ/bút chạm mặt chắc chắn hơn."
        )
        speed_grid.addWidget(QLabel("Z HẠ:"), 2, 0)
        speed_grid.addWidget(self.spin_dip, 2, 1)
        speed_grid.addWidget(QLabel("mm"), 2, 2)

        speed_box.setLayout(speed_grid)
        left_col.addWidget(speed_box)

        left_col.addStretch()

        # ---------------- Cột phải: Start/Stop + log ----------------
        right_col = QVBoxLayout()
        right_col.setSpacing(10)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        self.btn_start = QPushButton("▶  BẮT ĐẦU")
        self.btn_start.setObjectName("grayActionBtn")
        self.btn_start.setStyleSheet(GRAY_BTN_STYLE)
        self.btn_start.setMinimumHeight(44)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("■  DỪNG")
        self.btn_stop.setObjectName("redActionBtn")
        self.btn_stop.setStyleSheet(RED_BTN_STYLE)
        self.btn_stop.setMinimumHeight(44)
        self.btn_stop.clicked.connect(self.on_stop)
        self.btn_stop.setEnabled(False)
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        right_col.addLayout(btn_row)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setStyleSheet("font-size:10pt;")
        right_col.addWidget(self.log_box, 1)

        root.addLayout(left_col, 2)
        root.addLayout(right_col, 3)

        self._on_shape_changed(self.radio_line.isChecked())

    def _make_spin(self, min_v, max_v, default):
        spin = QDoubleSpinBox()
        spin.setRange(min_v, max_v)
        spin.setValue(default)
        spin.setDecimals(1)
        spin.setMaximumWidth(SPINBOX_MAX_W)
        return spin

    def _on_shape_changed(self, line_checked):
        self.line_box.setVisible(line_checked)
        self.circle_box.setVisible(not line_checked)

    def _set_busy_ui(self, busy):
        self.btn_start.setEnabled(not busy)
        self.btn_stop.setEnabled(busy)

    def _append_log_threadsafe(self, text):
        QTimer.singleShot(0, lambda: self.log_box.append(text))

    # ---------------- Đồng bộ ô số tốc độ <-> thanh trượt tốc độ ----------------
    def _on_speed_spin_changed(self, value):
        v_int = int(round(value))
        if self.slider_speed.value() != v_int:
            self.slider_speed.blockSignals(True)
            self.slider_speed.setValue(v_int)
            self.slider_speed.blockSignals(False)

    def _on_speed_slider_changed(self, value):
        if abs(self.spin_speed.value() - value) > 0.05:
            self.spin_speed.blockSignals(True)
            self.spin_speed.setValue(float(value))
            self.spin_speed.blockSignals(False)

    # ==================== START / STOP ====================
    def on_start(self):
        if self.ctx.busy:
            QMessageBox.warning(self, "Đang bận", "Robot đang thực hiện lệnh khác.")
            return

        is_line = self.radio_line.isChecked()
        dip = self.spin_dip.value()

        if is_line:
            x1 = self.spin_line_x1.value(); y1 = self.spin_line_y1.value()
            x2 = self.spin_line_x2.value(); y2 = self.spin_line_y2.value()
            z = self.spin_line_z.value()
            speed = self.spin_speed.value()
            self.log_box.append(
                f"=== Bắt đầu vẽ ĐƯỜNG THẲNG ({x1:.1f},{y1:.1f}) -> "
                f"({x2:.1f},{y2:.1f}), Z pick={z:.1f}, hạ thêm={dip:.1f} mm, "
                f"tốc độ={speed:.1f} mm/s ==="
            )

            def task():
                with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                    return self.draw_motion.draw_line(x1, y1, x2, y2, z, speed, dip=dip)
        else:
            cx = self.spin_circle_cx.value(); cy = self.spin_circle_cy.value()
            r = self.spin_circle_r.value()
            z = self.spin_circle_z.value()
            speed = self.spin_speed.value()
            self.log_box.append(
                f"=== Bắt đầu vẽ HÌNH TRÒN tâm ({cx:.1f},{cy:.1f}), R={r:.1f}, "
                f"Z pick={z:.1f}, hạ thêm={dip:.1f} mm, tốc độ={speed:.1f} mm/s ==="
            )

            def task():
                with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe(s))):
                    return self.draw_motion.draw_circle(cx, cy, r, z, speed, dip=dip)

        self.ctx.busy = True
        self._set_busy_ui(True)

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_draw_done)
        worker.done_err.connect(self._on_draw_error)
        self._track_worker(worker)

    def on_stop(self):
        self.draw_motion.stop()
        self.log_box.append(">>> Đã yêu cầu DỪNG - robot sẽ dừng sau điểm hiện tại.")

    def _on_draw_done(self, completed):
        self.ctx.busy = False
        self._set_busy_ui(False)
        if completed:
            self.log_box.append("=== HOÀN TẤT VẼ ===")
        else:
            self.log_box.append("=== ĐÃ DỪNG THEO YÊU CẦU ===")
        self.ctx.current_pos = list(self.ctx.planner.current_pos)

    def _on_draw_error(self, err):
        self.ctx.busy = False
        self._set_busy_ui(False)
        self.log_box.append(f"⚠ LỖI: {err}")