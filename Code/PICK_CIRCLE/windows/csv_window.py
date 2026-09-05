import os
import csv
import contextlib

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QGridLayout, QPushButton, QLabel,
    QGroupBox, QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QAbstractItemView, QCheckBox, QFileDialog,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, save_config, CSV_COLUMNS,
    CSV_FIELDNAMES, COORD_MODE_CIRCLE, DOF4_TARGET_ANGLE_DEG,
    _fmt_size_col, _fmt_angle_col,
)


class CsvWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🗂️ BÀI TOÁN TĨNH")
        self._csv_stop_flag = False
        self._build_ui()
        self._update_csv_mode_ui()
        self._refresh_csv_table()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_csv_mode_ui()
        self._set_busy_ui(self.ctx.busy)

    # ------------------------------------------------------------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        left_col = QVBoxLayout()

        title = QLabel("DANH SÁCH ĐIỂM (CSV)")
        title.setObjectName("sectionTitle")
        left_col.addWidget(title)

        self.csv_table = QTableWidget(0, len(CSV_COLUMNS))
        self.csv_table.setHorizontalHeaderLabels(CSV_COLUMNS)
        self.csv_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.csv_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        left_col.addWidget(self.csv_table, 5)

        csv_btn_row = QHBoxLayout()
        self.btn_csv_capture = QPushButton("📥 LẤY DANH SÁCH TỪ CAMERA")
        self.btn_csv_capture.clicked.connect(self.on_csv_capture)
        self.btn_csv_save = QPushButton("💾 LƯU RA FILE CSV")
        self.btn_csv_save.setObjectName("saveBtn")
        self.btn_csv_save.clicked.connect(self.on_csv_save)
        self.btn_csv_load = QPushButton("📂 NẠP FILE CSV")
        self.btn_csv_load.clicked.connect(self.on_csv_load)
        csv_btn_row.addWidget(self.btn_csv_capture)
        csv_btn_row.addWidget(self.btn_csv_save)
        csv_btn_row.addWidget(self.btn_csv_load)
        left_col.addLayout(csv_btn_row)

        self.lbl_csv_count = QLabel("Chưa có điểm nào.")
        self.lbl_csv_count.setStyleSheet("color:#333333; font-size:11pt;")
        left_col.addWidget(self.lbl_csv_count)

        right_col = QVBoxLayout()

        offset_box = QGroupBox("OFFSET CAMERA (mm) — riêng cho CSV")
        offset_box.setMaximumHeight(86)
        offset_box.setStyleSheet(
            "QGroupBox { margin-top:8px; padding-top:6px; }"
            "QDoubleSpinBox { padding:4px 6px; }"
        )
        offset_layout = QHBoxLayout()
        offset_layout.setContentsMargins(10, 4, 10, 6)
        offset_layout.setSpacing(8)
        offset_layout.addWidget(QLabel("X:"))
        self.spin_csv_offset_x = QDoubleSpinBox()
        self.spin_csv_offset_x.setRange(-100, 100)
        self.spin_csv_offset_x.setValue(self.ctx.csv_offset_x)
        self.spin_csv_offset_x.setSingleStep(0.5)
        self.spin_csv_offset_x.valueChanged.connect(self._on_csv_offset_x_changed)
        offset_layout.addWidget(self.spin_csv_offset_x)
        offset_layout.addWidget(QLabel("Y:"))
        self.spin_csv_offset_y = QDoubleSpinBox()
        self.spin_csv_offset_y.setRange(-100, 100)
        self.spin_csv_offset_y.setValue(self.ctx.csv_offset_y)
        self.spin_csv_offset_y.setSingleStep(0.5)
        self.spin_csv_offset_y.valueChanged.connect(self._on_csv_offset_y_changed)
        offset_layout.addWidget(self.spin_csv_offset_y)
        offset_box.setLayout(offset_layout)
        right_col.addWidget(offset_box)

        self.place_box_circle = QGroupBox("GHÉP CẶP VẬT (WHITE) → LỖ (BLACK)  [chế độ vòng tròn]")
        place_layout = QGridLayout()
        place_layout.addWidget(QLabel("Dung sai đường kính (mm):"), 0, 0)
        self.spin_csv_tolerance = QDoubleSpinBox()
        self.spin_csv_tolerance.setRange(0.0, 50.0)
        self.spin_csv_tolerance.setSingleStep(0.5)
        self.spin_csv_tolerance.setValue(self.ctx.cfg.get("csv_match_tolerance", 6.0))
        place_layout.addWidget(self.spin_csv_tolerance, 0, 1)

        self.chk_csv_same_color = QCheckBox("Chỉ ghép cùng màu (đỏ↔đỏ, xanh↔xanh)")
        self.chk_csv_same_color.setChecked(bool(self.ctx.cfg.get("csv_match_same_color", True)))
        place_layout.addWidget(self.chk_csv_same_color, 1, 0, 1, 2)

        place_layout.addWidget(QLabel("Z pick:"), 2, 0)
        self.spin_csv_z_pick = QDoubleSpinBox()
        self.spin_csv_z_pick.setRange(0, 500)
        self.spin_csv_z_pick.setValue(self.ctx.cfg.get("csv_z_pick", 340.0))
        place_layout.addWidget(self.spin_csv_z_pick, 2, 1)
        self.place_box_circle.setLayout(place_layout)
        right_col.addWidget(self.place_box_circle)

        self.dof4_box_manual = QGroupBox("BẬC TỰ DO 4 (BÀN XOAY / STEP) - góc cố định  [chế độ vòng tròn]")
        self.dof4_box_manual.setObjectName("compactBox")
        dof4_layout = QHBoxLayout()
        dof4_layout.setSpacing(8)

        self.chk_csv_apply_dof4 = QCheckBox("Áp dụng bậc tự do thứ 4")
        self.chk_csv_apply_dof4.setChecked(bool(self.ctx.cfg.get("csv_apply_dof4", False)))
        self.chk_csv_apply_dof4.toggled.connect(self._on_csv_apply_dof4_changed)
        dof4_layout.addWidget(self.chk_csv_apply_dof4)

        dof4_layout.addWidget(QLabel("Góc (độ):"))
        self.spin_csv_dof4_angle = QDoubleSpinBox()
        self.spin_csv_dof4_angle.setRange(-3600.0, 3600.0)
        self.spin_csv_dof4_angle.setSingleStep(5.0)
        self.spin_csv_dof4_angle.setValue(float(self.ctx.cfg.get("csv_dof4_angle", 90.0)))
        self.spin_csv_dof4_angle.valueChanged.connect(self._on_csv_dof4_angle_changed)
        dof4_layout.addWidget(self.spin_csv_dof4_angle, 1)

        self.dof4_box_manual.setLayout(dof4_layout)
        right_col.addWidget(self.dof4_box_manual)

        self.dof4_box_auto = QGroupBox("GẮP-THẢ TỰ ĐỘNG THEO BẬC TỰ DO 4  [chế độ Camera_4dof]")
        self.dof4_box_auto.setObjectName("compactBox")
        dof4_auto_layout = QVBoxLayout()
        dof4_auto_layout.setSpacing(6)

        info_row = QLabel(
            f"Mỗi vật đỏ 10x20mm phát hiện được sẽ tự động: Gắp tại vị trí camera đo được "
            f"→ Xoay bậc tự do 4 để bù góc → Thả cố định tại "
            f"(0, 0) với hướng {DOF4_TARGET_ANGLE_DEG:.0f}° so với trục X → Xoay trả về 0 cho lần gắp kế tiếp."
        )
        info_row.setWordWrap(True)
        info_row.setStyleSheet("color:#333333; font-size:9pt;")
        dof4_auto_layout.addWidget(info_row)

        zrow = QHBoxLayout()
        zrow.addWidget(QLabel("Z pick:"))
        self.spin_csv_z_pick_dof4 = QDoubleSpinBox()
        self.spin_csv_z_pick_dof4.setRange(0, 500)
        self.spin_csv_z_pick_dof4.setValue(self.ctx.cfg.get("csv_z_pick", 340.0))
        zrow.addWidget(self.spin_csv_z_pick_dof4)
        dof4_auto_layout.addLayout(zrow)

        self.dof4_box_auto.setLayout(dof4_auto_layout)
        right_col.addWidget(self.dof4_box_auto)

        self.btn_csv_preview = QPushButton("🔍 XEM TRƯỚC GHÉP CẶP")
        self.btn_csv_preview.clicked.connect(self.on_csv_preview)
        right_col.addWidget(self.btn_csv_preview)

        run_row = QHBoxLayout()
        self.btn_csv_run = QPushButton("🤖 CHẠY TỰ ĐỘNG TỪ DANH SÁCH")
        self.btn_csv_run.setObjectName("actionBtn")
        self.btn_csv_run.clicked.connect(self.on_csv_run)
        self.btn_csv_stop = QPushButton("⏹ DỪNG")
        self.btn_csv_stop.setObjectName("disconnectBtn")
        self.btn_csv_stop.clicked.connect(self.on_csv_stop)
        self.btn_csv_stop.setEnabled(False)
        run_row.addWidget(self.btn_csv_run, 3)
        run_row.addWidget(self.btn_csv_stop, 1)
        right_col.addLayout(run_row)

        self.lbl_csv_progress = QLabel("Sẵn sàng.")
        self.lbl_csv_progress.setWordWrap(True)
        self.lbl_csv_progress.setStyleSheet("color:#333333; font-weight:700; font-size:12pt;")
        right_col.addWidget(self.lbl_csv_progress)

        self.log_box_csv = QTextEdit()
        self.log_box_csv.setReadOnly(True)
        right_col.addWidget(self.log_box_csv, 4)

        root.addLayout(left_col, 3)
        root.addLayout(right_col, 2)

    def _update_csv_mode_ui(self):
        is_circle = (self.ctx.coord_mode == COORD_MODE_CIRCLE)
        self.place_box_circle.setVisible(is_circle)
        self.dof4_box_manual.setVisible(is_circle)
        self.dof4_box_auto.setVisible(not is_circle)
        self.btn_csv_preview.setVisible(is_circle)

    def _set_busy_ui(self, busy):
        self.btn_csv_run.setEnabled(not busy)

    def _on_csv_apply_dof4_changed(self, checked):
        self.ctx.cfg["csv_apply_dof4"] = checked
        save_config(self.ctx.cfg)

    def _on_csv_dof4_angle_changed(self, val):
        self.ctx.cfg["csv_dof4_angle"] = val
        save_config(self.ctx.cfg)

    def _refresh_csv_table(self):
        self.csv_table.setRowCount(len(self.ctx.csv_points))
        for i, c in enumerate(self.ctx.csv_points):
            self.csv_table.setItem(i, 0, QTableWidgetItem(str(c.get("type", ""))))
            self.csv_table.setItem(i, 1, QTableWidgetItem(_fmt_size_col(c)))
            self.csv_table.setItem(i, 2, QTableWidgetItem(f'{float(c.get("x_mm", 0)):.2f}'))
            self.csv_table.setItem(i, 3, QTableWidgetItem(f'{float(c.get("y_mm", 0)):.2f}'))
            self.csv_table.setItem(i, 4, QTableWidgetItem(_fmt_angle_col(c)))
        self.lbl_csv_count.setText(f"Đang có {len(self.ctx.csv_points)} điểm.")

    def on_csv_capture(self):
        if not self.ctx.latest_circles:
            QMessageBox.warning(
                self, "Không có dữ liệu",
                "Camera chưa phát hiện vật nào (hãy bật camera ở tab CAMERA trước)."
            )
            return
        self.ctx.csv_points = list(self.ctx.latest_circles)
        self._refresh_csv_table()
        self.lbl_csv_progress.setText(f"📥 Đã lấy {len(self.ctx.csv_points)} điểm từ camera.")

    def on_csv_save(self):
        if not self.ctx.csv_points:
            QMessageBox.warning(self, "Không có dữ liệu", "Danh sách điểm đang trống, không có gì để lưu.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Lưu danh sách điểm", "../point.csv", "CSV files (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
                writer.writeheader()
                for c in self.ctx.csv_points:
                    writer.writerow({k: c.get(k, "") if c.get(k) is not None else "" for k in CSV_FIELDNAMES})
            self.lbl_csv_progress.setText(f"💾 Đã lưu {len(self.ctx.csv_points)} điểm vào {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi lưu CSV", str(e))

    def on_csv_load(self):
        path, _ = QFileDialog.getOpenFileName(self, "Nạp danh sách điểm", "", "CSV files (*.csv)")
        if not path:
            return
        try:
            points = []

            def _f(row, key):
                v = row.get(key)
                if v is None or v == "":
                    return None
                return float(v)

            with open(path, "r", newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    points.append({
                        "type": row.get("type", ""),
                        "diameter_mm": _f(row, "diameter_mm"),
                        "width_mm": _f(row, "width_mm"),
                        "height_mm": _f(row, "height_mm"),
                        "angle_deg": _f(row, "angle_deg"),
                        "x_mm": _f(row, "x_mm") or 0.0,
                        "y_mm": _f(row, "y_mm") or 0.0,
                    })
            self.ctx.csv_points = points
            self._refresh_csv_table()
            self.lbl_csv_progress.setText(f"📂 Đã nạp {len(points)} điểm từ {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Lỗi nạp CSV", str(e))

    def _match_white_black_pairs(self, points, same_color_only=True, tolerance_mm=6.0):
        whites = [p for p in points if "white" in str(p.get("type", ""))]
        blacks = [p for p in points if "black" in str(p.get("type", ""))]
        used_black_idx = set()
        pairs = []
        unmatched = []

        for w in whites:
            w_color = str(w.get("type", "")).split("_")[0]
            w_dia = float(w.get("diameter_mm", 0) or 0)
            best_idx, best_diff = None, None
            for i, b in enumerate(blacks):
                if i in used_black_idx:
                    continue
                b_color = str(b.get("type", "")).split("_")[0]
                if same_color_only and b_color != w_color:
                    continue
                diff = abs(float(b.get("diameter_mm", 0) or 0) - w_dia)
                if diff <= tolerance_mm and (best_diff is None or diff < best_diff):
                    best_diff, best_idx = diff, i
            if best_idx is not None:
                used_black_idx.add(best_idx)
                pairs.append((w, blacks[best_idx]))
            else:
                unmatched.append(w)

        return pairs, unmatched

    def _log_pairs(self, pairs, unmatched, log_widget):
        log_widget.append(
            f"=== Ghép được {len(pairs)} cặp Vật→Lỗ (bỏ qua {len(unmatched)} vật không có lỗ phù hợp) ==="
        )
        for idx, (w, b) in enumerate(pairs, start=1):
            log_widget.append(
                f"  {idx}. {w.get('type')} Ø{float(w.get('diameter_mm', 0) or 0):.1f}mm "
                f"({float(w.get('x_mm', 0)):.1f},{float(w.get('y_mm', 0)):.1f}) → "
                f"{b.get('type')} Ø{float(b.get('diameter_mm', 0) or 0):.1f}mm "
                f"({float(b.get('x_mm', 0)):.1f},{float(b.get('y_mm', 0)):.1f})"
            )
        for w in unmatched:
            log_widget.append(
                f"  ⚠ Bỏ qua: {w.get('type')} Ø{float(w.get('diameter_mm', 0) or 0):.1f}mm "
                f"không tìm thấy lỗ phù hợp."
            )

    def on_csv_preview(self):
        if self.ctx.coord_mode != COORD_MODE_CIRCLE:
            return
        if not self.ctx.csv_points:
            QMessageBox.warning(self, "Không có dữ liệu", "Danh sách điểm đang trống.")
            return
        pairs, unmatched = self._match_white_black_pairs(
            self.ctx.csv_points,
            same_color_only=self.chk_csv_same_color.isChecked(),
            tolerance_mm=self.spin_csv_tolerance.value(),
        )
        self.log_box_csv.clear()
        self._log_pairs(pairs, unmatched, self.log_box_csv)
        self.lbl_csv_progress.setText(f"🔍 Xem trước: {len(pairs)} cặp sẽ chạy, {len(unmatched)} vật bị bỏ qua.")

    def _on_csv_offset_x_changed(self, val):
        self.ctx.csv_offset_x = val
        self.ctx.cfg["csv_offset_x"] = val
        save_config(self.ctx.cfg)

    def _on_csv_offset_y_changed(self, val):
        self.ctx.csv_offset_y = val
        self.ctx.cfg["csv_offset_y"] = val
        save_config(self.ctx.cfg)

    def on_csv_stop(self):
        self._csv_stop_flag = True
        self.lbl_csv_progress.setText("⏹ Đang dừng sau khi hoàn tất mục hiện tại...")

    def on_csv_run(self):
        if self.ctx.busy:
            QMessageBox.warning(self, "Đang bận", "Robot đang thực hiện lệnh khác.")
            return
        if not self.ctx.csv_points:
            QMessageBox.warning(
                self, "Không có dữ liệu",
                "Danh sách điểm đang trống. Hãy lấy từ camera hoặc nạp file CSV."
            )
            return

        if self.ctx.coord_mode == COORD_MODE_CIRCLE:
            self._run_csv_circle_mode()
        else:
            self._run_csv_dof4_mode()

    def _run_csv_circle_mode(self):
        same_color_only = self.chk_csv_same_color.isChecked()
        tolerance = self.spin_csv_tolerance.value()
        pairs, unmatched = self._match_white_black_pairs(
            self.ctx.csv_points, same_color_only=same_color_only, tolerance_mm=tolerance
        )
        if not pairs:
            QMessageBox.warning(
                self, "Không ghép được cặp nào",
                "Không tìm thấy cặp Vật (white) - Lỗ (black) nào phù hợp trong danh sách.\n"
                "Hãy thử tăng dung sai đường kính hoặc bỏ chọn 'chỉ ghép cùng màu'."
            )
            return

        self.ctx.cfg["csv_match_tolerance"] = tolerance
        self.ctx.cfg["csv_match_same_color"] = same_color_only
        self.ctx.cfg["csv_z_pick"] = self.spin_csv_z_pick.value()
        save_config(self.ctx.cfg)

        self._csv_stop_flag = False
        self.ctx.busy = True
        self._set_busy_ui(True)
        self.btn_csv_stop.setEnabled(True)
        self.log_box_csv.clear()
        self._log_pairs(pairs, unmatched, self.log_box_csv)

        z_pick = self.spin_csv_z_pick.value()
        offset_x, offset_y = self.ctx.csv_offset_x, self.ctx.csv_offset_y
        apply_dof4 = self.chk_csv_apply_dof4.isChecked()
        dof4_angle = self.spin_csv_dof4_angle.value()

        def task():
            total = len(pairs)
            for i, (w, b) in enumerate(pairs, start=1):
                if self._csv_stop_flag:
                    self._append_log_threadsafe_csv("⏹ Đã dừng theo yêu cầu.")
                    return False
                point_a = (w["x_mm"] + offset_x, w["y_mm"] + offset_y)
                point_b = (b["x_mm"] + offset_x, b["y_mm"] + offset_y)
                QTimer.singleShot(
                    0,
                    lambda i=i, total=total, pa=point_a, pb=point_b: self.lbl_csv_progress.setText(
                        f"Đang xử lý cặp {i}/{total}: Vật({pa[0]:.1f},{pa[1]:.1f}) "
                        f"→ Lỗ({pb[0]:.1f},{pb[1]:.1f})"
                    ),
                )
                if apply_dof4:
                    self._append_log_threadsafe_csv(f"↻ [DOF4] Quay {dof4_angle:.1f}° trước khi xử lý cặp {i}")
                    self.ctx.pneu_uart.step_rotate(dof4_angle)

                with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe_csv(s))):
                    self.ctx.planner.pick_and_place(
                        point_a, point_b, z_pick=z_pick, gripper_callback=self.ctx.gripper_callback
                    )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_csv_run_done)
        worker.done_err.connect(self._on_csv_run_error)
        self._track_worker(worker)

    def _run_csv_dof4_mode(self):
        points = [p for p in self.ctx.csv_points if p.get("x_mm") is not None]
        if not points:
            QMessageBox.warning(self, "Không có dữ liệu", "Danh sách điểm đang trống.")
            return

        self.ctx.cfg["csv_z_pick"] = self.spin_csv_z_pick_dof4.value()
        save_config(self.ctx.cfg)

        self._csv_stop_flag = False
        self.ctx.busy = True
        self._set_busy_ui(True)
        self.btn_csv_stop.setEnabled(True)
        self.log_box_csv.clear()

        self.log_box_csv.append(
            f"=== [DOF4] Sẽ gắp {len(points)} vật, mỗi vật thả lần lượt tại các vị trí "
            f"(0,0), (-10,0), (-20,0), ... (cách nhau 10mm theo X), với hướng {DOF4_TARGET_ANGLE_DEG:.0f}° ==="
        )

        z_pick = self.spin_csv_z_pick_dof4.value()
        offset_x, offset_y = self.ctx.csv_offset_x, self.ctx.csv_offset_y

        def task():
            total = len(points)
            for i, p in enumerate(points, start=1):
                if self._csv_stop_flag:
                    self._append_log_threadsafe_csv("⏹ Đã dừng theo yêu cầu.")
                    return False

                point_a = (p["x_mm"] + offset_x, p["y_mm"] + offset_y)
                angle = p.get("angle_deg")
                place_point = (-(i - 1) * 10.0, 0.0)

                QTimer.singleShot(
                    0,
                    lambda i=i, total=total, pa=point_a, ang=angle, pp=place_point: self.lbl_csv_progress.setText(
                        f"Đang xử lý vật {i}/{total}: A({pa[0]:.1f},{pa[1]:.1f}) "
                        f"góc={('%.1f' % ang) if ang is not None else '?'}° "
                        f"→ B({pp[0]:.0f},{pp[1]:.0f})"
                    ),
                )

                with contextlib.redirect_stdout(StreamToSignal(lambda s: self._append_log_threadsafe_csv(s))):
                    self.ctx.planner_dof4.pick_and_place_dof4(
                        point_a,
                        z_pick=z_pick,
                        gripper_callback=self.ctx.gripper_callback,
                        rotate_callback=self.ctx.pneu_uart.step_rotate,
                        object_angle_deg=angle,
                        place_point=place_point,
                        target_angle_deg=DOF4_TARGET_ANGLE_DEG,
                    )
            return True

        worker = FnWorker(task)
        worker.done_ok.connect(self._on_csv_run_done)
        worker.done_err.connect(self._on_csv_run_error)
        self._track_worker(worker)

    def _append_log_threadsafe_csv(self, text):
        QTimer.singleShot(0, lambda: self.log_box_csv.append(text))

    def _on_csv_run_done(self, ok):
        self.ctx.busy = False
        self._set_busy_ui(False)
        self.btn_csv_stop.setEnabled(False)
        if ok:
            self.log_box_csv.append("=== HOÀN TẤT TOÀN BỘ DANH SÁCH ===")
            self.lbl_csv_progress.setText("✔ Đã chạy xong toàn bộ danh sách.")
        else:
            self.lbl_csv_progress.setText("⏹ Đã dừng giữa chừng.")
        self.ctx.current_pos = list(self.ctx.planner.HOME)

    def _on_csv_run_error(self, err):
        self.ctx.busy = False
        self._set_busy_ui(False)
        self.btn_csv_stop.setEnabled(False)
        self.log_box_csv.append(f"⚠ LỖI: {err}")
        self.lbl_csv_progress.setText("⚠ Có lỗi xảy ra.")
