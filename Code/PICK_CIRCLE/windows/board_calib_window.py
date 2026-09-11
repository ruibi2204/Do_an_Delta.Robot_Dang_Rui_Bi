# windows/board_calib_window.py
"""
Giao diện HIỆU CHỈNH CAMERA BẰNG BÀN CỜ - gộp 4 tác vụ từ 2 file gốc
(capture_images.py: chụp ảnh bàn cờ | calib_camera.py: calib -> .npz/.yaml)
thành 1 cửa sổ, có thêm bước xuất lưới toạ độ ra .csv và cho robot tự
chạy qua từng điểm đó (dùng move_delta_4dof.py) để kiểm tra offset.

Mỗi tác vụ có 1 nút riêng:
  1) CHỤP ẢNH BÀN CỜ  - tự động chụp cho tới khi đủ N ảnh (mặc định 30).
  2) CALIB -> .npz     - chạy calib từ thư mục ảnh, xuất .npz + .yaml.
  3) XUẤT TOẠ ĐỘ .csv  - xuất lưới toạ độ các giao điểm bàn cờ (theo
     board_w/board_h/square_size + gốc toạ độ offset_x/offset_y) ra CSV.
  4) START / STOP      - cho robot tự chạy lần lượt qua các điểm trong
     CSV để kiểm tra bằng mắt. STOP: dừng vòng lặp rồi về HOME VẬT LÝ
     (chạm công tắc hành trình, chờ READY từ STM32) - giống các cửa sổ
     BÀI TOÁN BẬC 4 TĨNH / BÀI TOÁN ĐỘNG BẬC 4.
"""
import csv
import os
import time

import cv2
import numpy as np

from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QPushButton, QLineEdit,
    QSpinBox, QDoubleSpinBox, QGroupBox, QTextEdit, QProgressBar,
    QFileDialog, QMessageBox, QSizePolicy, QScrollArea, QWidget,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, cv2_to_qpixmap,
    save_config, home_and_wait,
)

# Tất cả nút trong cửa sổ này PHẢI màu xanh dương (theo yêu cầu).
BLUE_BTN_STYLE = """
QPushButton#blueActionBtn {
    background-color: #0078d4;
    color: #ffffff;
    border: 2px solid #0a63ad;
    border-radius: 12px;
    padding: 12px 18px;
    font-weight: 700;
    font-size: 14pt;
    min-height: 40px;
}
QPushButton#blueActionBtn:hover { background-color: #106ebe; }
QPushButton#blueActionBtn:pressed { background-color: #005a9e; }
QPushButton#blueActionBtn:disabled { background-color: #8fbfe0; border-color: #6ea5cc; color: #eef6fc; }
"""

# Style chung cho các ô nhập liệu / label trong panel bên phải, để chữ to
# rõ ràng hơn và không bị tràn / bị cắt như trước.
INPUT_STYLE = """
QLineEdit, QSpinBox, QDoubleSpinBox {
    font-size: 12pt;
    padding: 6px 8px;
    min-height: 30px;
    border: 1px solid #b0b0b0;
    border-radius: 6px;
    background-color: #ffffff;
}
QLabel {
    font-size: 12pt;
}
QGroupBox {
    font-size: 12.5pt;
    font-weight: 700;
    margin-top: 14px;
    padding-top: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QProgressBar {
    font-size: 11pt;
    min-height: 24px;
}
"""


def _make_blue_button(text):
    btn = QPushButton(text)
    btn.setObjectName("blueActionBtn")
    btn.setStyleSheet(BLUE_BTN_STYLE)
    btn.setMinimumHeight(44)
    return btn


def find_corners_auto(gray, board_w, board_h):
    """Y HỆT logic trong calib_camera.py: thử cả (board_w,board_h) và
    (board_w-1,board_h-1) để tự dò đúng số góc trong, phòng người dùng nhập
    số ô vuông thay vì số góc trong."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    for size in ((board_w, board_h), (board_w - 1, board_h - 1)):
        if size[0] < 2 or size[1] < 2:
            continue
        ok, corners = cv2.findChessboardCorners(gray, size, flags)
        if ok:
            return ok, corners, size
    return False, None, None


# =====================================================================
# 1) THREAD CHỤP ẢNH BÀN CỜ TỰ ĐỘNG
# =====================================================================
class ChessboardCaptureThread(QThread):
    frame_ready = Signal(object, bool, int)   # display, found, saved_count
    finished_ok = Signal(int, str)            # saved_count, out_dir
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index, out_dir, board_w, board_h,
                 target_count=30, min_interval_s=1.5):
        super().__init__()
        self.camera_index = camera_index
        self.out_dir = out_dir
        self.board_w = board_w
        self.board_h = board_h
        self.target_count = target_count
        self.min_interval_s = min_interval_s
        self._running = False

    def request_stop(self):
        self._running = False

    def run(self):
        os.makedirs(self.out_dir, exist_ok=True)
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self.error.emit(f"Không thể mở camera index {self.camera_index}")
            self.stopped.emit()
            return

        self._running = True
        saved_count = 0
        last_save_time = 0.0

        try:
            while self._running and saved_count < self.target_count:
                ret, frame = cap.read()
                if not ret:
                    self.error.emit("Mất tín hiệu camera.")
                    break

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners, used_size = find_corners_auto(gray, self.board_w, self.board_h)

                display = frame.copy()
                if found:
                    cv2.drawChessboardCorners(display, used_size, corners, found)
                    cv2.putText(display, f"Da tim thay ban co: {used_size[0]}x{used_size[1]}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display, "Chua tim thay ban co...",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(display, f"Da luu: {saved_count}/{self.target_count} anh",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                now = time.time()
                if found and (now - last_save_time) >= self.min_interval_s:
                    filename = os.path.join(self.out_dir, f"calib_{saved_count:03d}.png")
                    cv2.imwrite(filename, frame)
                    saved_count += 1
                    last_save_time = now

                self.frame_ready.emit(display, found, saved_count)
        except Exception as e:
            self.error.emit(str(e))
        finally:
            cap.release()
            self.finished_ok.emit(saved_count, self.out_dir)
            self.stopped.emit()


# =====================================================================
# 2) HÀM CALIB (Y HỆT LOGIC calib_camera.py) - chạy trong FnWorker (nền)
# =====================================================================
def run_calibration_task(images_dir, board_w, board_h, square_size, output_path, log_fn):
    import glob

    log_fn(f"Đang tìm ảnh trong thư mục: {os.path.abspath(images_dir)}")
    if not os.path.isdir(images_dir):
        raise RuntimeError(f"Thư mục '{images_dir}' không tồn tại.")

    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    image_paths = []
    for ext in exts:
        image_paths.extend(glob.glob(os.path.join(images_dir, ext)))
    image_paths = sorted(image_paths)

    if len(image_paths) == 0:
        raise RuntimeError(f"Không tìm thấy ảnh nào trong '{images_dir}'.")
    if len(image_paths) < 5:
        log_fn(f"Cảnh báo: chỉ có {len(image_paths)} ảnh (nên có >=10-20 ảnh).")

    size_votes = {}
    detections = []
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            log_fn(f"Bỏ qua (không đọc được): {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners, size = find_corners_auto(gray, board_w, board_h)
        if ok:
            size_votes[size] = size_votes.get(size, 0) + 1
            detections.append((path, gray.shape[::-1], corners, size, img))
        else:
            log_fn(f"Không phát hiện được bàn cờ trong: {os.path.basename(path)}")

    if not detections:
        raise RuntimeError("Không phát hiện được bàn cờ trong bất kỳ ảnh nào.")

    best_size = max(size_votes, key=size_votes.get)
    log_fn(f"Kích thước góc trong dùng: {best_size[0]}x{best_size[1]} "
           f"({size_votes[best_size]}/{len(image_paths)} ảnh khớp)")
    bw, bh = best_size

    objp = np.zeros((bh * bw, 3), np.float32)
    objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2)
    objp *= square_size

    objpoints, imgpoints = [], []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    used_count = 0
    for path, shape, corners, size, img in detections:
        if size != best_size:
            continue
        image_size = shape
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        used_count += 1

    log_fn(f"Sử dụng {used_count} ảnh để calib...")
    if used_count < 5:
        log_fn("Cảnh báo: số ảnh hợp lệ quá ít (<5), kết quả có thể không chính xác.")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    total_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        total_error += cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error = total_error / len(objpoints)

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    log_fn(f"RMS reprojection error: {ret:.4f} | Mean error: {mean_error:.4f} px")
    log_fn(f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    if mean_error < 0.5:
        log_fn("Đánh giá: Calib TỐT (sai số < 0.5 px)")
    elif mean_error < 1.0:
        log_fn("Đánh giá: Calib CHẤP NHẬN ĐƯỢC (< 1.0 px)")
    else:
        log_fn("Đánh giá: Sai số cao (>1.0 px) - nên chụp thêm ảnh nhiều góc độ hơn.")

    npz_path = f"{output_path}.npz"
    np.savez(
        npz_path, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
        image_size=image_size, mean_error=mean_error,
        board_w=bw, board_h=bh, square_size=square_size,
    )
    yaml_path = f"{output_path}.yaml"
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("mean_reprojection_error", mean_error)
    fs.release()

    log_fn(f"Đã lưu: {npz_path}")
    log_fn(f"Đã lưu: {yaml_path}")
    return {"npz_path": npz_path, "yaml_path": yaml_path, "mean_error": mean_error}


# =====================================================================
# 3) XUẤT LƯỚI TOẠ ĐỘ BÀN CỜ RA CSV (tức thời, không cần thread)
# =====================================================================
def export_chessboard_csv(csv_path, board_w, board_h, square_size, origin_x, origin_y):
    """Xuất lưới toạ độ các giao điểm bàn cờ (board_w x board_h góc trong,
    cách nhau square_size mm), dịch theo gốc (origin_x, origin_y) cho khớp
    với vị trí đặt bàn cờ thật trên bàn robot."""
    rows = []
    idx = 0
    for j in range(board_h):
        for i in range(board_w):
            x_mm = origin_x + i * square_size
            y_mm = origin_y + j * square_size
            rows.append((idx, x_mm, y_mm))
            idx += 1

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x_mm", "y_mm"])
        writer.writerows(rows)

    return rows


# =====================================================================
# 4) THREAD CHO ROBOT CHẠY QUA TỪNG ĐIỂM TRONG CSV
# =====================================================================
class RunPointsThread(QThread):
    log_line = Signal(str)
    progress = Signal(int, int)     # (điểm hiện tại, tổng số điểm)
    finished_ok = Signal(int)
    finished_err = Signal(str)

    def __init__(self, ctx, points, z_travel, dwell_s=1.0):
        super().__init__()
        self.ctx = ctx
        self.points = points
        self.z_travel = z_travel
        self.dwell_s = dwell_s
        self._stop_requested = False

    def request_stop(self):
        self._stop_requested = True

    def run(self):
        import sys
        old_stdout = sys.stdout
        sys.stdout = StreamToSignal(self.log_line.emit)
        completed = 0
        try:
            planner = self.ctx.planner
            total = len(self.points)
            for i, (x_mm, y_mm) in enumerate(self.points, 1):
                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Người dùng yêu cầu dừng trước điểm {i}/{total}.")
                    break

                self.log_line.emit(f"--- Điểm {i}/{total}: ({x_mm:.1f}, {y_mm:.1f}) ---")
                planner._move_or_raise(x_mm, y_mm, self.z_travel, planner.TIME_MOVE_FAST)
                time.sleep(self.dwell_s)

                completed = i
                self.progress.emit(i, total)

                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Đã dừng sau khi tới điểm {i}/{total}.")
                    break

            self.finished_ok.emit(completed)
        except Exception as e:
            self.finished_err.emit(str(e))
        finally:
            sys.stdout = old_stdout


# =====================================================================
# CỬA SỔ CHÍNH
# =====================================================================
class BoardCalibWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🏁 HIỆU CHỈNH CAMERA BẰNG BÀN CỜ")
        self._capture_thread = None
        self._run_thread = None
        self._last_csv_points = []
        self._build_ui()

    # ---------------- UI ----------------
    def _build_ui(self):
        root = QHBoxLayout(self.content_widget)

        # ---------- Cột trái: video preview ----------
        video_col = QVBoxLayout()
        self.video_label = QLabel("Camera chưa chạy")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 420)
        self.video_label.setStyleSheet(
            "background-color:#ffffff; border:2px solid #c0c0c0; border-radius:10px; color:#888888;"
        )
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_col.addWidget(self.video_label, 1)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(200)
        video_col.addWidget(self.log_view)

        root.addLayout(video_col, 3)

        # ---------- Cột phải: 4 nhóm tác vụ (đặt trong QScrollArea để khi
        # nội dung bị tràn thì có thể cuộn xuống xem thay vì bị cắt/lỏ) ----------
        ctrl_container = QWidget()
        ctrl_container.setStyleSheet(INPUT_STYLE)
        ctrl_col = QVBoxLayout(ctrl_container)
        ctrl_col.setSpacing(14)
        ctrl_col.setContentsMargins(4, 4, 12, 4)

        cfg = self.ctx.cfg

        # ---- 1) CHỤP ẢNH BÀN CỜ ----
        g1 = QGroupBox("1) CHỤP ẢNH BÀN CỜ")
        g1.setObjectName("compactBox")
        l1 = QGridLayout(g1)
        l1.setVerticalSpacing(10)
        l1.setHorizontalSpacing(10)
        self.spin_board_w = QSpinBox(); self.spin_board_w.setRange(2, 30)
        self.spin_board_w.setValue(int(cfg.get("board_w", 9)))
        self.spin_board_h = QSpinBox(); self.spin_board_h.setRange(2, 30)
        self.spin_board_h.setValue(int(cfg.get("board_h", 7)))
        self.spin_num_images = QSpinBox(); self.spin_num_images.setRange(5, 200)
        self.spin_num_images.setValue(int(cfg.get("board_capture_count", 30)))
        self.edit_images_dir = QLineEdit(cfg.get("board_images_dir", "images"))

        l1.addWidget(QLabel("Số góc trong W:"), 0, 0); l1.addWidget(self.spin_board_w, 0, 1)
        l1.addWidget(QLabel("Số góc trong H:"), 1, 0); l1.addWidget(self.spin_board_h, 1, 1)
        l1.addWidget(QLabel("Số ảnh cần chụp:"), 2, 0); l1.addWidget(self.spin_num_images, 2, 1)
        l1.addWidget(QLabel("Thư mục lưu:"), 3, 0); l1.addWidget(self.edit_images_dir, 3, 1)

        self.progress_capture = QProgressBar()
        self.progress_capture.setFormat("0/%d ảnh" % self.spin_num_images.value())
        l1.addWidget(self.progress_capture, 4, 0, 1, 2)

        cap_btn_row = QVBoxLayout()
        cap_btn_row.setSpacing(8)
        self.btn_capture_start = _make_blue_button("▶ BẮT ĐẦU CHỤP")
        self.btn_capture_start.clicked.connect(self.on_start_capture)
        self.btn_capture_stop = _make_blue_button("■ DỪNG CHỤP")
        self.btn_capture_stop.clicked.connect(self.on_stop_capture)
        self.btn_capture_stop.setEnabled(False)
        cap_btn_row.addWidget(self.btn_capture_start)
        cap_btn_row.addWidget(self.btn_capture_stop)
        l1.addLayout(cap_btn_row, 5, 0, 1, 2)

        ctrl_col.addWidget(g1)

        # ---- 2) CALIB -> .npz ----
        g2 = QGroupBox("2) CALIB CAMERA -> .npz")
        g2.setObjectName("compactBox")
        l2 = QGridLayout(g2)
        l2.setVerticalSpacing(10)
        l2.setHorizontalSpacing(10)
        self.spin_square_size = QDoubleSpinBox()
        self.spin_square_size.setRange(1.0, 200.0)
        self.spin_square_size.setValue(float(cfg.get("board_square_size", 9.96)))
        self.edit_output_name = QLineEdit(cfg.get("board_calib_output", "calibration_result"))

        l2.addWidget(QLabel("Kích thước ô vuông (mm):"), 0, 0); l2.addWidget(self.spin_square_size, 0, 1)
        l2.addWidget(QLabel("Tên file output:"), 1, 0); l2.addWidget(self.edit_output_name, 1, 1)

        self.btn_calib_run = _make_blue_button("▶ CHẠY CALIB")
        self.btn_calib_run.clicked.connect(self.on_run_calibration)
        l2.addWidget(self.btn_calib_run, 2, 0, 1, 2)

        ctrl_col.addWidget(g2)

        # ---- 3) XUẤT TOẠ ĐỘ .csv ----
        g3 = QGroupBox("3) XUẤT TOẠ ĐỘ LƯỚI BÀN CỜ -> .csv")
        g3.setObjectName("compactBox")
        l3 = QGridLayout(g3)
        l3.setVerticalSpacing(10)
        l3.setHorizontalSpacing(10)
        self.spin_origin_x = QDoubleSpinBox(); self.spin_origin_x.setRange(-1000, 1000)
        self.spin_origin_x.setValue(float(cfg.get("board_origin_x", 0.0)))
        self.spin_origin_y = QDoubleSpinBox(); self.spin_origin_y.setRange(-1000, 1000)
        self.spin_origin_y.setValue(float(cfg.get("board_origin_y", 0.0)))
        self.edit_csv_path = QLineEdit(cfg.get("board_csv_path", "board_points.csv"))
        btn_browse_csv_save = _make_blue_button("Chọn nơi lưu...")
        btn_browse_csv_save.clicked.connect(self.on_browse_csv_save)

        l3.addWidget(QLabel("Gốc X (mm):"), 0, 0); l3.addWidget(self.spin_origin_x, 0, 1)
        l3.addWidget(QLabel("Gốc Y (mm):"), 1, 0); l3.addWidget(self.spin_origin_y, 1, 1)
        l3.addWidget(QLabel("File CSV:"), 2, 0); l3.addWidget(self.edit_csv_path, 2, 1)
        l3.addWidget(btn_browse_csv_save, 3, 0, 1, 2)

        self.btn_export_csv = _make_blue_button("▶ XUẤT CSV")
        self.btn_export_csv.clicked.connect(self.on_export_csv)
        l3.addWidget(self.btn_export_csv, 4, 0, 1, 2)

        ctrl_col.addWidget(g3)

        # ---- 4) CHẠY ROBOT QUA CÁC ĐIỂM CSV ----
        g4 = QGroupBox("4) CHẠY ROBOT QUA CÁC ĐIỂM (CSV)")
        g4.setObjectName("compactBox")
        l4 = QGridLayout(g4)
        l4.setVerticalSpacing(10)
        l4.setHorizontalSpacing(10)
        self.edit_run_csv_path = QLineEdit(cfg.get("board_csv_path", "board_points.csv"))
        btn_browse_csv_load = _make_blue_button("Chọn file CSV...")
        btn_browse_csv_load.clicked.connect(self.on_browse_csv_load)
        self.spin_z_travel = QDoubleSpinBox(); self.spin_z_travel.setRange(0, 500)
        self.spin_z_travel.setValue(float(cfg.get("board_z_travel", 300.0)))
        self.spin_dwell = QDoubleSpinBox(); self.spin_dwell.setRange(0.0, 10.0)
        self.spin_dwell.setSingleStep(0.5)
        self.spin_dwell.setValue(float(cfg.get("board_dwell_s", 1.0)))

        l4.addWidget(QLabel("File CSV:"), 0, 0); l4.addWidget(self.edit_run_csv_path, 0, 1)
        l4.addWidget(btn_browse_csv_load, 1, 0, 1, 2)
        l4.addWidget(QLabel("Z di chuyển (mm):"), 2, 0); l4.addWidget(self.spin_z_travel, 2, 1)
        l4.addWidget(QLabel("Dừng tại mỗi điểm (s):"), 3, 0); l4.addWidget(self.spin_dwell, 3, 1)

        self.progress_run = QProgressBar()
        self.progress_run.setFormat("Sẵn sàng")
        l4.addWidget(self.progress_run, 4, 0, 1, 2)

        run_btn_row = QVBoxLayout()
        run_btn_row.setSpacing(8)
        self.btn_run_start = _make_blue_button("▶ START")
        self.btn_run_start.clicked.connect(self.on_run_start)
        self.btn_run_stop = _make_blue_button("■ STOP (VỀ HOME)")
        self.btn_run_stop.clicked.connect(self.on_run_stop)
        self.btn_run_stop.setEnabled(False)
        run_btn_row.addWidget(self.btn_run_start)
        run_btn_row.addWidget(self.btn_run_stop)
        l4.addLayout(run_btn_row, 5, 0, 1, 2)

        ctrl_col.addWidget(g4)
        ctrl_col.addStretch()

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setWidget(ctrl_container)
        scroll_area.setMinimumWidth(420)

        root.addWidget(scroll_area, 2)

    # ---------------- Tiện ích ----------------
    def _append_log(self, text):
        QTimer.singleShot(0, lambda: self.log_view.append(text))

    def _save_cfg(self):
        cfg = self.ctx.cfg
        cfg["board_w"] = self.spin_board_w.value()
        cfg["board_h"] = self.spin_board_h.value()
        cfg["board_capture_count"] = self.spin_num_images.value()
        cfg["board_images_dir"] = self.edit_images_dir.text()
        cfg["board_square_size"] = self.spin_square_size.value()
        cfg["board_calib_output"] = self.edit_output_name.text()
        cfg["board_origin_x"] = self.spin_origin_x.value()
        cfg["board_origin_y"] = self.spin_origin_y.value()
        cfg["board_csv_path"] = self.edit_csv_path.text()
        cfg["board_z_travel"] = self.spin_z_travel.value()
        cfg["board_dwell_s"] = self.spin_dwell.value()
        save_config(cfg)

    # ---------------- 1) Chụp ảnh ----------------
    def on_start_capture(self):
        if self._capture_thread is not None:
            return
        if self.ctx.camera_thread is not None or self.ctx.dynamic_camera_thread is not None:
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Một tab camera khác đang mở - hãy đóng trước.")
            return

        self._save_cfg()
        target = self.spin_num_images.value()
        self.progress_capture.setRange(0, target)
        self.progress_capture.setValue(0)
        self.progress_capture.setFormat(f"0/{target} ảnh")
        self.log_view.clear()
        self.btn_capture_start.setEnabled(False)
        self.btn_capture_stop.setEnabled(True)

        self._capture_thread = ChessboardCaptureThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            out_dir=self.edit_images_dir.text(),
            board_w=self.spin_board_w.value(),
            board_h=self.spin_board_h.value(),
            target_count=target,
        )
        self._capture_thread.frame_ready.connect(self._on_capture_frame)
        self._capture_thread.error.connect(self._on_capture_error)
        self._capture_thread.finished_ok.connect(self._on_capture_finished)
        self._capture_thread.stopped.connect(self._on_capture_stopped)
        self._capture_thread.start()

    def on_stop_capture(self):
        if self._capture_thread is not None:
            self._capture_thread.request_stop()

    def _on_capture_frame(self, frame, found, saved_count):
        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        self.progress_capture.setValue(saved_count)
        self.progress_capture.setFormat(f"{saved_count}/{self.spin_num_images.value()} ảnh")

    def _on_capture_error(self, msg):
        self._append_log(f"⚠ {msg}")

    def _on_capture_finished(self, saved_count, out_dir):
        self._append_log(f"✔ HOÀN TẤT (Done): đã chụp {saved_count} ảnh vào '{out_dir}'.")

    def _on_capture_stopped(self):
        self._capture_thread = None
        self.video_label.setText("Camera đã dừng")
        self.btn_capture_start.setEnabled(True)
        self.btn_capture_stop.setEnabled(False)

    # ---------------- 2) Calib -> npz ----------------
    def on_run_calibration(self):
        self._save_cfg()
        self.btn_calib_run.setEnabled(False)
        self._append_log("=== Bắt đầu chạy CALIB... ===")

        worker = FnWorker(
            run_calibration_task,
            self.edit_images_dir.text(),
            self.spin_board_w.value(),
            self.spin_board_h.value(),
            self.spin_square_size.value(),
            self.edit_output_name.text(),
            self._append_log,
        )
        worker.done_ok.connect(self._on_calib_done)
        worker.done_err.connect(self._on_calib_error)
        self._track_worker(worker)

    def _on_calib_done(self, result):
        self.btn_calib_run.setEnabled(True)
        self._append_log(f"✔ HOÀN TẤT (Done): {result['npz_path']} "
                          f"(sai số {result['mean_error']:.4f} px)")

    def _on_calib_error(self, err):
        self.btn_calib_run.setEnabled(True)
        self._append_log(f"⚠ LỖI CALIB: {err}")
        QMessageBox.critical(self, "Lỗi Calib", err)

    # ---------------- 3) Xuất CSV ----------------
    def on_browse_csv_save(self):
        path, _ = QFileDialog.getSaveFileName(self, "Chọn nơi lưu CSV", self.edit_csv_path.text(), "CSV (*.csv)")
        if path:
            self.edit_csv_path.setText(path)

    def on_export_csv(self):
        self._save_cfg()
        try:
            rows = export_chessboard_csv(
                self.edit_csv_path.text(),
                self.spin_board_w.value(),
                self.spin_board_h.value(),
                self.spin_square_size.value(),
                self.spin_origin_x.value(),
                self.spin_origin_y.value(),
            )
            self._last_csv_points = [(r[1], r[2]) for r in rows]
            self.edit_run_csv_path.setText(self.edit_csv_path.text())
            self._append_log(f"✔ HOÀN TẤT (Done): đã xuất {len(rows)} điểm vào '{self.edit_csv_path.text()}'.")
        except Exception as e:
            self._append_log(f"⚠ LỖI xuất CSV: {e}")
            QMessageBox.critical(self, "Lỗi xuất CSV", str(e))

    # ---------------- 4) Chạy robot qua các điểm ----------------
    def on_browse_csv_load(self):
        path, _ = QFileDialog.getOpenFileName(self, "Chọn file CSV", self.edit_run_csv_path.text(), "CSV (*.csv)")
        if path:
            self.edit_run_csv_path.setText(path)

    def _load_points_from_csv(self, path):
        points = []
        with open(path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append((float(row["x_mm"]), float(row["y_mm"])))
        return points

    def on_run_start(self):
        if self._run_thread is not None:
            return
        if self.ctx.busy:
            QMessageBox.warning(self, "Bận", "Robot đang bận thực hiện thao tác khác.")
            return
        if not self.ctx.robot_uart.is_connected:
            reply = QMessageBox.question(
                self, "Chưa kết nối",
                "Chưa kết nối UART robot, chạy chế độ mô phỏng (dry-run)?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        csv_path = self.edit_run_csv_path.text()
        if not os.path.isfile(csv_path):
            QMessageBox.warning(self, "Không tìm thấy file", f"Không tìm thấy file CSV: {csv_path}")
            return

        try:
            points = self._load_points_from_csv(csv_path)
        except Exception as e:
            QMessageBox.critical(self, "Lỗi đọc CSV", str(e))
            return
        if not points:
            QMessageBox.warning(self, "File rỗng", "File CSV không có điểm nào.")
            return

        self._save_cfg()
        self.log_view.clear()
        self.progress_run.setRange(0, len(points))
        self.progress_run.setValue(0)
        self.progress_run.setFormat(f"0/{len(points)}")

        self.ctx.busy = True
        self.btn_run_start.setEnabled(False)
        self.btn_run_stop.setEnabled(True)

        self._run_thread = RunPointsThread(
            self.ctx, points,
            z_travel=self.spin_z_travel.value(),
            dwell_s=self.spin_dwell.value(),
        )
        self._run_thread.log_line.connect(self._append_log)
        self._run_thread.progress.connect(self._on_run_progress)
        self._run_thread.finished_ok.connect(self._on_run_finished_ok)
        self._run_thread.finished_err.connect(self._on_run_finished_err)
        self._run_thread.finished.connect(self._on_run_thread_finished)
        self._run_thread.start()

    def _on_run_progress(self, current, total):
        self.progress_run.setValue(current)
        self.progress_run.setFormat(f"{current}/{total}")

    def _on_run_finished_ok(self, completed):
        self.ctx.busy = False
        self._append_log(f"=== HOÀN TẤT (Done): đã tới {completed} điểm. ===")
        self.btn_run_start.setEnabled(True)
        self.btn_run_stop.setEnabled(False)

    def _on_run_finished_err(self, err):
        self.ctx.busy = False
        self._append_log(f"⚠ LỖI: {err}")
        self.btn_run_start.setEnabled(True)
        self.btn_run_stop.setEnabled(False)

    def _on_run_thread_finished(self):
        self._run_thread = None

    def on_run_stop(self):
        """STOP: dừng vòng chạy điểm, rồi về HOME VẬT LÝ (chạm công tắc hành
        trình, chờ READY từ STM32) - giống các cửa sổ BÀI TOÁN BẬC 4 TĨNH /
        BÀI TOÁN ĐỘNG BẬC 4."""
        if self._run_thread is not None:
            self._append_log("[YÊU CẦU DỪNG] Sẽ dừng sau khi tới điểm hiện tại...")
            self._run_thread.request_stop()
        self.btn_run_stop.setEnabled(False)
        self._wait_then_home()

    def _wait_then_home(self):
        if self._run_thread is not None and self._run_thread.isRunning():
            QTimer.singleShot(300, self._wait_then_home)
            return
        self._append_log("↻ Đang về HOME vật lý (chạm công tắc hành trình) và chờ READY...")
        worker = FnWorker(home_and_wait, self.ctx.robot_uart, timeout=20.0)
        worker.done_ok.connect(self._on_stop_home_done)
        worker.done_err.connect(lambda err: self._append_log(f"⚠ LỖI về Home: {err}"))
        self._track_worker(worker)

    def _on_stop_home_done(self, ready):
        self.ctx.busy = False
        if ready:
            self._append_log("✔ Đã về HOME xong (READY).")
        else:
            self._append_log("⚠ Không nhận được READY từ STM32.")
        self.btn_run_start.setEnabled(True)

    # ---------------- Đóng cửa sổ ----------------
    def closeEvent(self, event):
        if self._capture_thread is not None:
            self._capture_thread.request_stop()
            self._capture_thread.wait(2000)
        if self._run_thread is not None:
            self._run_thread.request_stop()
            self._run_thread.wait(2000)
        super().closeEvent(event)