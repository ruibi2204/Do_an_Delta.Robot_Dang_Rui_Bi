# windows/repeat_test_window.py
import sys

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel,
    QDoubleSpinBox, QSpinBox, QSlider, QPushButton, QTextEdit, QProgressBar,
    QMessageBox,
)

from shared_state import BaseTabWindow, StreamToSignal, save_config

# Giới hạn / mặc định cho thanh trượt tốc độ (%). 100% = tốc độ mặc định
# (dùng đúng các TIME_MOVE_* cấu hình trong kinematics/repeat_kinematics.py).
SPEED_PERCENT_MIN = 20
SPEED_PERCENT_MAX = 300
SPEED_PERCENT_DEFAULT = 100

# Giá trị mặc định cho Z DOWN (độ chạm sâu thêm, tương ứng DIP_MM trong
# kinematics/repeat_kinematics.py) khi chưa có gì lưu trong config.
Z_DOWN_DEFAULT = 10.0

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


# =====================================================================
# WORKER - chạy vòng lặp A -> B trong 1 thread riêng, không làm treo
# giao diện. Dùng planner_repeat (kinematics/repeat_kinematics.py) -
# planner RIÊNG, ĐỘC LẬP với planner gắp-thả chính, KHÔNG điều khiển
# gripper (bài test chỉ đánh giá độ lặp lại vị trí cơ khí).
#
# Mỗi chu kỳ (planner_repeat.repeat_cycle):
#   Điểm A -> hạ z_pick -> +Z DOWN -> -Z DOWN -> về Home
#   -> Điểm B -> hạ z_pick -> +Z DOWN -> -Z DOWN -> về Home
#
# Có hỗ trợ dừng "mềm" (dừng sau khi hoàn tất chu trình hiện tại, không
# ngắt robot giữa chừng để tránh lệch vị trí).
# =====================================================================
class RepeatWorker(QThread):
    log_line = Signal(str)
    progress = Signal(int, int)     # (lần hiện tại, tổng số lần)
    finished_ok = Signal(int)       # số lần đã hoàn tất
    finished_err = Signal(str)

    def __init__(self, ctx, point_a, point_b, z_pick, repeat_count):
        super().__init__()
        self.ctx = ctx
        self.point_a = point_a
        self.point_b = point_b
        self.z_pick = z_pick
        self.repeat_count = repeat_count
        self._stop_requested = False

    def request_stop(self):
        self._stop_requested = True

    def run(self):
        old_stdout = sys.stdout
        sys.stdout = StreamToSignal(self.log_line.emit)
        completed = 0
        try:
            # planner_repeat: planner RIÊNG cho bài test độ lặp lại
            # (kinematics/repeat_kinematics.py), KHÔNG dùng ctx.planner
            # (planner gắp-thả chính) và KHÔNG cần gripper_callback.
            planner = self.ctx.planner_repeat
            for i in range(1, self.repeat_count + 1):
                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Người dùng yêu cầu dừng trước lần {i}.")
                    break

                self.log_line.emit(f"--- LẦN LẶP {i}/{self.repeat_count} ---")

                planner.repeat_cycle(
                    self.point_a,
                    self.point_b,
                    z_pick=self.z_pick,
                )

                completed = i
                self.progress.emit(i, self.repeat_count)

                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Đã dừng sau khi hoàn tất lần {i}.")
                    break

            self.finished_ok.emit(completed)
        except Exception as e:
            self.finished_err.emit(str(e))
        finally:
            sys.stdout = old_stdout


# =====================================================================
# CỬA SỔ ĐÁNH GIÁ ĐỘ LẶP LẠI
# =====================================================================
class RepeatTestWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "ĐÁNH GIÁ ĐỘ LẶP LẠI")
        self.worker = None
        self._build_ui()
        # Áp dụng ngay tốc độ và Z DOWN đã lưu (hoặc mặc định) cho
        # planner_repeat lúc mở tab, để planner và giao diện luôn đồng bộ.
        self._apply_speed_to_planner(self.slider_speed.value())
        self._apply_zdown_to_planner(self.spin_zdown.value())

    # ---------------- UI ----------------
    def _build_ui(self):
        layout = QVBoxLayout(self.content_widget)
        layout.setSpacing(14)

        title = QLabel("KIỂM TRA ĐỘ LẶP LẠI")
        title.setObjectName("menuTitle2")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(20)

        cfg = self.ctx.cfg

        # ---- Điểm A ----
        group_a = QGroupBox("ĐIỂM A")
        grid_a = QGridLayout(group_a)
        self.spin_ax = self._make_spin(cfg.get("repeat_ax", 10.0))
        self.spin_ay = self._make_spin(cfg.get("repeat_ay", 10.0))
        grid_a.addWidget(QLabel("X (mm):"), 0, 0)
        grid_a.addWidget(self.spin_ax, 0, 1)
        grid_a.addWidget(QLabel("Y (mm):"), 1, 0)
        grid_a.addWidget(self.spin_ay, 1, 1)
        row.addWidget(group_a)

        # ---- Điểm B ----
        group_b = QGroupBox("ĐIỂM B")
        grid_b = QGridLayout(group_b)
        self.spin_bx = self._make_spin(cfg.get("repeat_bx", -10.0))
        self.spin_by = self._make_spin(cfg.get("repeat_by", 10.0))
        grid_b.addWidget(QLabel("X (mm):"), 0, 0)
        grid_b.addWidget(self.spin_bx, 0, 1)
        grid_b.addWidget(QLabel("Y (mm):"), 1, 0)
        grid_b.addWidget(self.spin_by, 1, 1)
        row.addWidget(group_b)

        # ---- Tham số ----
        group_p = QGroupBox("THAM SỐ")
        grid_p = QGridLayout(group_p)
        self.spin_zpick = self._make_spin(cfg.get("repeat_z_pick", 300.0), mn=0.0, mx=500.0)
        self.spin_repeat = QSpinBox()
        self.spin_repeat.setRange(1, 10000)
        self.spin_repeat.setValue(int(cfg.get("repeat_count", 10)))

        # (MỚI) Z DOWN - độ chạm sâu thêm sau khi hạ tới z_pick, tương ứng
        # tham số DIP_MM trong kinematics/repeat_kinematics.py. Trước đây
        # giá trị này bị cố định trong code (DIP_MM = 10.0), không có ô
        # nhập trên giao diện. Giờ cho phép người vận hành chỉnh trực tiếp.
        self.spin_zdown = self._make_spin(
            cfg.get("repeat_z_down", Z_DOWN_DEFAULT), mn=0.0, mx=100.0
        )
        self.spin_zdown.valueChanged.connect(self._on_zdown_changed)

        grid_p.addWidget(QLabel("Z pick (mm):"), 0, 0)
        grid_p.addWidget(self.spin_zpick, 0, 1)
        grid_p.addWidget(QLabel("SỐ LẦN LẶP:"), 1, 0)
        grid_p.addWidget(self.spin_repeat, 1, 1)
        grid_p.addWidget(QLabel("Z HẠ (mm):"), 2, 0)
        grid_p.addWidget(self.spin_zdown, 2, 1)
        row.addWidget(group_p)

        # ---- Tốc độ (thanh trượt kéo) ----
        group_speed = QGroupBox("TỐC ĐỘ")
        speed_layout = QVBoxLayout(group_speed)

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setObjectName("repeatSpeedSlider")
        self.slider_speed.setMinimum(SPEED_PERCENT_MIN)
        self.slider_speed.setMaximum(SPEED_PERCENT_MAX)
        self.slider_speed.setValue(int(cfg.get("repeat_speed_percent", SPEED_PERCENT_DEFAULT)))
        self.slider_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_speed.setTickInterval(20)
        self.slider_speed.valueChanged.connect(self._on_speed_changed)

        self.lbl_speed_value = QLabel(f"{self.slider_speed.value()}%")
        self.lbl_speed_value.setAlignment(Qt.AlignCenter)
        self.lbl_speed_value.setStyleSheet("font-size:13pt; font-weight:600; color:#1670d4;")

        speed_hint = QLabel("100% = mặc định, kéo để nhanh/chậm hơn")
        speed_hint.setAlignment(Qt.AlignCenter)
        speed_hint.setWordWrap(True)
        speed_hint.setStyleSheet("color:#4b6a93; font-size:9pt;")

        speed_layout.addWidget(self.slider_speed)
        speed_layout.addWidget(self.lbl_speed_value)
        speed_layout.addWidget(speed_hint)
        row.addWidget(group_speed)

        layout.addLayout(row)

        # ---- Nút điều khiển ----
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("▶  BẮT ĐẦU")
        self.btn_start.setObjectName("grayActionBtn")
        self.btn_start.setStyleSheet(GRAY_BTN_STYLE)
        self.btn_start.setMinimumHeight(44)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("■  DỪNG")
        self.btn_stop.setObjectName("redActionBtn")
        self.btn_stop.setStyleSheet(RED_BTN_STYLE)
        self.btn_stop.setMinimumHeight(44)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.on_stop)
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        layout.addLayout(btn_row)

        # ---- Tiến trình ----
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 1)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("SẴN SÀNG")
        layout.addWidget(self.progress_bar)

        self.lbl_status = QLabel("Trạng thái: Sẵn sàng")
        self.lbl_status.setObjectName("statusOk")
        layout.addWidget(self.lbl_status)

        # ---- Nhật ký ----
        log_title = QLabel("Nhật ký thực thi:")
        log_title.setObjectName("sectionTitle")
        layout.addWidget(log_title)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view, 1)

    def _make_spin(self, val, mn=-500.0, mx=500.0):
        sb = QDoubleSpinBox()
        sb.setRange(mn, mx)
        sb.setDecimals(1)
        sb.setSingleStep(1.0)
        sb.setValue(float(val))
        return sb

    def _refresh_style(self, widget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    # ---------------- Tốc độ ----------------
    def _apply_speed_to_planner(self, percent):
        """Áp dụng % tốc độ hiện tại xuống planner_repeat (nếu đã tồn tại).
        An toàn để gọi bất cứ lúc nào, kể cả khi bài test đang chạy - chỉ áp
        dụng cho đoạn di chuyển tiếp theo, không ngắt chuyển động hiện tại."""
        planner = getattr(self.ctx, "planner_repeat", None)
        if planner is not None and hasattr(planner, "set_speed_scale"):
            planner.set_speed_scale(percent / 100.0)

    def _on_speed_changed(self, value):
        self.lbl_speed_value.setText(f"{value}%")
        self._apply_speed_to_planner(value)
        self.ctx.cfg["repeat_speed_percent"] = value
        save_config(self.ctx.cfg)

    # ---------------- (MỚI) Z DOWN ----------------
    def _apply_zdown_to_planner(self, value):
        """Áp dụng giá trị Z DOWN (độ chạm sâu thêm) hiện tại xuống
        planner_repeat (nếu đã tồn tại), qua update_speed_params với khóa
        DIP_MM - đúng tên tham số trong kinematics/repeat_kinematics.py.
        An toàn để gọi bất cứ lúc nào, kể cả khi bài test đang chạy - chỉ
        áp dụng cho lần touch_point() tiếp theo."""
        planner = getattr(self.ctx, "planner_repeat", None)
        if planner is not None and hasattr(planner, "update_speed_params"):
            planner.update_speed_params({"DIP_MM": value})

    def _on_zdown_changed(self, value):
        self._apply_zdown_to_planner(value)
        self.ctx.cfg["repeat_z_down"] = value
        save_config(self.ctx.cfg)

    # ---------------- Logic ----------------
    def on_start(self):
        if self.ctx.busy:
            QMessageBox.warning(self, "Bận", "Robot đang bận thực hiện thao tác khác.")
            return

        if not self.ctx.robot_uart.is_connected:
            reply = QMessageBox.question(
                self, "Chưa kết nối",
                "Chưa kết nối UART robot. Chạy chế độ mô phỏng (dry-run)?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        point_a = (self.spin_ax.value(), self.spin_ay.value())
        point_b = (self.spin_bx.value(), self.spin_by.value())
        z_pick = self.spin_zpick.value()
        repeat_count = self.spin_repeat.value()
        z_down = self.spin_zdown.value()

        # Đảm bảo planner đang dùng đúng tốc độ và Z DOWN hiện tại trên
        # giao diện trước khi bắt đầu chạy.
        self._apply_speed_to_planner(self.slider_speed.value())
        self._apply_zdown_to_planner(z_down)

        # lưu lại tham số vào config để lần sau mở lên có sẵn
        self.ctx.cfg["repeat_ax"] = point_a[0]
        self.ctx.cfg["repeat_ay"] = point_a[1]
        self.ctx.cfg["repeat_bx"] = point_b[0]
        self.ctx.cfg["repeat_by"] = point_b[1]
        self.ctx.cfg["repeat_z_pick"] = z_pick
        self.ctx.cfg["repeat_count"] = repeat_count
        self.ctx.cfg["repeat_speed_percent"] = self.slider_speed.value()
        self.ctx.cfg["repeat_z_down"] = z_down
        save_config(self.ctx.cfg)

        self.log_view.clear()
        self.progress_bar.setRange(0, repeat_count)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat(f"0/{repeat_count}")

        self.lbl_status.setText("Trạng thái: Đang chạy...")
        self.lbl_status.setObjectName("statusBad")
        self._refresh_style(self.lbl_status)

        self.ctx.busy = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        self.worker = RepeatWorker(self.ctx, point_a, point_b, z_pick, repeat_count)
        self.worker.log_line.connect(self.append_log)
        self.worker.progress.connect(self.on_progress)
        self.worker.finished_ok.connect(self.on_finished_ok)
        self.worker.finished_err.connect(self.on_finished_err)
        self.worker.finished.connect(self._on_thread_finished)
        self.worker.start()

    def on_stop(self):
        if self.worker:
            self.append_log("[YÊU CẦU DỪNG] Sẽ dừng sau khi hoàn tất chu trình hiện tại...")
            self.worker.request_stop()
            self.btn_stop.setEnabled(False)

    def append_log(self, text):
        self.log_view.append(text)

    def on_progress(self, current, total):
        self.progress_bar.setValue(current)
        self.progress_bar.setFormat(f"{current}/{total}")

    def on_finished_ok(self, completed):
        self.ctx.busy = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText(f"Trạng thái: Hoàn tất {completed} lần.")
        self.lbl_status.setObjectName("statusOk")
        self._refresh_style(self.lbl_status)
        self.append_log(f"=== HOÀN TẤT: {completed}/{self.spin_repeat.value()} lần lặp ===")

    def on_finished_err(self, msg):
        self.ctx.busy = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("Trạng thái: LỖI")
        self.lbl_status.setObjectName("statusBad")
        self._refresh_style(self.lbl_status)
        self.append_log(f"[LỖI] {msg}")
        QMessageBox.critical(self, "Lỗi", msg)

    def _on_thread_finished(self):
        self.worker = None

    # ---------------- Đóng cửa sổ / quay lại menu khi đang chạy ----------------
    def go_back(self):
        if self.worker and self.worker.isRunning():
            reply = QMessageBox.question(
                self, "Đang chạy",
                "Đang thực hiện đánh giá độ lặp lại. Bạn có muốn dừng và quay lại menu?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            self.worker.request_stop()
            self.worker.wait(5000)
            self.ctx.busy = False
        super().go_back()

    def closeEvent(self, event):
        if self._force_close and self.worker and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(3000)
        super().closeEvent(event)