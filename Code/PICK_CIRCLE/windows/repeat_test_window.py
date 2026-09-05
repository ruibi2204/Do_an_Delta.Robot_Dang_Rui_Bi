# windows/repeat_test_window.py
import sys

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel,
    QDoubleSpinBox, QSpinBox, QPushButton, QTextEdit, QProgressBar, QMessageBox,
)

from shared_state import BaseTabWindow, StreamToSignal, save_config


# =====================================================================
# WORKER - chạy vòng lặp gắp-thả A -> B trong 1 thread riêng, không làm
# treo giao diện. Có hỗ trợ dừng "mềm" (dừng sau khi hoàn tất chu trình
# hiện tại, không ngắt robot giữa chừng để tránh làm rơi vật / lệch vị trí).
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
            planner = self.ctx.planner
            for i in range(1, self.repeat_count + 1):
                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Người dùng yêu cầu dừng trước lần {i}.")
                    break

                self.log_line.emit(f"--- LẦN LẶP {i}/{self.repeat_count} ---")

                planner.pick_and_place(
                    self.point_a,
                    self.point_b,
                    z_pick=self.z_pick,
                    gripper_callback=self.ctx.gripper_callback,
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

    # ---------------- UI ----------------
    def _build_ui(self):
        layout = QVBoxLayout(self.content_widget)
        layout.setSpacing(14)

        title = QLabel("ĐÁNH GIÁ ĐỘ LẶP LẠI (REPEATABILITY TEST)")
        title.setObjectName("menuTitle2")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(20)

        cfg = self.ctx.cfg

        # ---- Điểm A ----
        group_a = QGroupBox("ĐIỂM A (tọa ộ 1)")
        grid_a = QGridLayout(group_a)
        self.spin_ax = self._make_spin(cfg.get("repeat_ax", 10.0))
        self.spin_ay = self._make_spin(cfg.get("repeat_ay", 10.0))
        grid_a.addWidget(QLabel("X (mm):"), 0, 0)
        grid_a.addWidget(self.spin_ax, 0, 1)
        grid_a.addWidget(QLabel("Y (mm):"), 1, 0)
        grid_a.addWidget(self.spin_ay, 1, 1)
        row.addWidget(group_a)

        # ---- Điểm B ----
        group_b = QGroupBox("ĐIỂM B (Tọa độ 2)")
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
        self.spin_zpick = self._make_spin(cfg.get("csv_z_pick", 340.0), mn=0.0, mx=500.0)
        self.spin_repeat = QSpinBox()
        self.spin_repeat.setRange(1, 10000)
        self.spin_repeat.setValue(int(cfg.get("repeat_count", 10)))
        grid_p.addWidget(QLabel("Z gắp (mm):"), 0, 0)
        grid_p.addWidget(self.spin_zpick, 0, 1)
        grid_p.addWidget(QLabel("Số lần lặp:"), 1, 0)
        grid_p.addWidget(self.spin_repeat, 1, 1)
        row.addWidget(group_p)

        layout.addLayout(row)

        # ---- Nút điều khiển ----
        btn_row = QHBoxLayout()
        self.btn_start = QPushButton("▶  START")
        self.btn_start.setObjectName("dynStartBtn")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("■  STOP")
        self.btn_stop.setObjectName("dynStopBtn")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.on_stop)
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_stop)
        layout.addLayout(btn_row)

        # ---- Tiến trình ----
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 1)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Sẵn sàng")
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

        # lưu lại tham số vào config để lần sau mở lên có sẵn
        self.ctx.cfg["repeat_ax"] = point_a[0]
        self.ctx.cfg["repeat_ay"] = point_a[1]
        self.ctx.cfg["repeat_bx"] = point_b[0]
        self.ctx.cfg["repeat_by"] = point_b[1]
        self.ctx.cfg["repeat_count"] = repeat_count
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