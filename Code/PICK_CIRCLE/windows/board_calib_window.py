# windows/board_calib_window.py
"""
Giao diện HIỆU CHỈNH CAMERA BẰNG BÀN CỜ - gộp 4 tác vụ từ 2 file gốc
(capture_images.py: chụp ảnh bàn cờ | calib_camera.py: calib -> .npz/.yaml)
thành 1 cửa sổ, có thêm bước xuất lưới toạ độ ra .csv và cho robot tự
chạy qua từng điểm đó (dùng move_delta_4dof.py) để kiểm tra offset.

Mỗi tác vụ có 1 nút riêng:
  1) CHỤP ẢNH BÀN CỜ  - tự động chụp cho tới khi đủ N ảnh (mặc định 30).
  2) CALIB -> .npz     - chạy calib từ thư mục ảnh, xuất .npz + .yaml.
  3) XUẤT TOẠ ĐỘ .csv  - 3 BƯỚC BẮT BUỘC THEO THỨ TỰ:
       3a) Chụp ảnh bàn cờ Ở VỊ TRÍ CHUẨN - ĐẾM NGƯỢC CỐ ĐỊNH 3 GIÂY để
           người dùng có thời gian đặt/giữ yên bàn cờ, rồi TỰ ĐỘNG chụp
           đúng 1 khung hình tại thời điểm hết giờ. Nếu không dò được góc
           bàn cờ trong khung chụp đó -> báo lỗi, yêu cầu bấm chụp lại.
       3b) Thu thập toạ độ THỰC từ ảnh vừa chụp: BẮT BUỘC dùng đúng file
           .npz vừa xuất ra ở Bước 2 (CALIB -> .npz) TRONG CHÍNH CỬA SỔ
           NÀY để khử méo, rồi suy ra khoảng cách mm/px THẬT từ chính
           khoảng cách pixel đo được giữa các góc (so với square_size khai
           báo) để tính toạ độ X/Y mm cho từng điểm. Nếu chưa chạy Bước 2,
           hệ thống sẽ hỏi người dùng có muốn tiếp tục KHÔNG khử méo hay
           không (kết quả sẽ kém chính xác hơn).
       3c) Xuất các toạ độ đã thu thập ra file .csv.
     Nút xuất CSV chỉ bấm được sau khi đã thu thập toạ độ; nút thu thập
     chỉ bấm được sau khi đã chụp ảnh chuẩn thành công.
  4) START / STOP      - cho robot tự chạy lần lượt qua các điểm trong
     CSV để kiểm tra bằng mắt. Quỹ đạo chuyển động cho bước này được TÁCH
     RIÊNG hoàn toàn ra module kinematics/board_points_motion.py (mượt,
     đường thẳng tuyệt đối, không cong). Tại MỖI điểm:
       1) Đi thẳng, mượt tới (X, Y) ở độ cao Z travel.
       2) Hạ xuống nhẹ Z down (quy ước trục Z: "+" là đi XUỐNG, "-" là đi
          LÊN, nên hạ xuống = z_travel + z_down).
       3) Dừng lại một khoảng "Dwell khi xuống".
       4) Nâng trở lại lên Z travel.
       5) Về VỊ TRÍ HOME ĐÃ LƯU (planner.HOME) bằng di chuyển thẳng bình
          thường - KHÔNG chạm công tắc hành trình, KHÔNG chờ phản hồi
          READY từ STM32 (để chạy nhanh hơn giữa các điểm).
     Tốc độ di chuyển ngang (mm/s) chỉnh được bằng thanh trượt kéo trên
     giao diện, áp dụng ngay cả khi đang chạy.

     STOP (nút "■ STOP AND HOME"): dừng vòng lặp NGAY SAU điểm hiện tại,
     rồi về HẲN HOME VẬT LÝ (chạm công tắc hành trình, chờ READY từ
     STM32) - đây là bước an toàn khi người dùng chủ động dừng, KHÁC với
     bước "về Home đã lưu" (nhanh, không chờ READY) diễn ra tự động sau
     mỗi điểm trong lúc chạy bình thường.

     TIẾP TỤC SAU KHI STOP (MỚI): các điểm CHƯA kịp chạy tới được GIỮ LẠI
     trong bộ nhớ của cửa sổ (gắn với đúng file CSV đang dùng). Nếu người
     dùng bấm lại "▶ START" trong khi vẫn dùng CÙNG file CSV đó, hệ thống
     sẽ CHẠY TIẾP từ điểm còn dang dở (nối đuôi), KHÔNG đọc lại và chạy
     lại từ điểm đầu tiên. Chỉ khi nào đã chạy hết TOÀN BỘ điểm, hoặc
     người dùng đổi sang một file CSV khác, thì lần START kế tiếp mới
     load lại từ đầu.
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
    QFileDialog, QMessageBox, QSizePolicy, QScrollArea, QWidget, QSlider,
)

from shared_state import (
    BaseTabWindow, FnWorker, StreamToSignal, cv2_to_qpixmap,
    save_config, home_and_wait,
)
from kinematics.board_points_motion import BoardPointsMotionPlanner, MotionError

# Giới hạn / mặc định cho thanh trượt tốc độ (mm/s) của bước 4 (Run Robot).
RUN_SPEED_MIN = 5
RUN_SPEED_MAX = 100
RUN_SPEED_DEFAULT = 30

# Thời gian đếm ngược (giây) trước khi tự động chụp ảnh bàn cờ "chuẩn" ở
# bước 3a - cố định, không đổi qua GUI.
REFERENCE_CAPTURE_WAIT_S = 3.0

# Nút hành động thường (Start/Run/Chọn file/...) - màu XANH DƯƠNG gradient,
# đồng bộ với nút chính của giao diện ngoài.
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

# Nút DỪNG / HUỶ (Stop / Cancel) - màu XÁM, đồng bộ với nút Dừng ngoài.
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

# Style chung cho các ô nhập liệu / label trong panel bên phải, để chữ to
# rõ ràng hơn và không bị tràn / bị cắt như trước. Đã chỉnh cho nền SÁNG.
INPUT_STYLE = """
QLineEdit, QSpinBox, QDoubleSpinBox {
    font-size: 12pt;
    padding: 6px 8px;
    min-height: 30px;
    border: 1px solid #c5d8ee;
    border-radius: 8px;
    background-color: #ffffff;
    color: #12315c;
}
QLabel {
    font-size: 12pt;
    color: #12315c;
}
QGroupBox {
    font-size: 12.5pt;
    font-weight: 700;
    margin-top: 14px;
    padding-top: 14px;
    color: #0d3b82;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QProgressBar {
    font-size: 11pt;
    min-height: 24px;
    border: 1px solid #c5d8ee;
    border-radius: 6px;
    background-color: #ffffff;
    color: #12315c;
    text-align: center;
}
QProgressBar::chunk {
    background-color: #1a7ee0;
    border-radius: 4px;
}
"""


def _make_blue_button(text):
    """Nút hành động thường - màu XANH DƯƠNG (giữ tên hàm cũ để không phải
    sửa lại toàn bộ nơi gọi bên dưới)."""
    btn = QPushButton(text)
    btn.setObjectName("grayActionBtn")
    btn.setStyleSheet(GRAY_BTN_STYLE)
    btn.setMinimumHeight(44)
    return btn


def _make_stop_button(text):
    """Nút DỪNG / HUỶ - màu XÁM."""
    btn = QPushButton(text)
    btn.setObjectName("redActionBtn")
    btn.setStyleSheet(RED_BTN_STYLE)
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
# 1b) THREAD CHỤP 1 ẢNH BÀN CỜ "Ở VỊ TRÍ CHUẨN" (dùng cho bước 3a)
#
# Đếm ngược cố định `wait_seconds` giây (mặc định 3s) để người dùng có
# thời gian đặt/giữ yên bàn cờ, rồi CHỤP ĐÚNG 1 khung hình tại đúng thời
# điểm hết giờ và thử dò góc bàn cờ trên khung đó. Nếu không dò được góc
# trong khung chụp đó -> báo lỗi, KHÔNG tự chụp lại.
# =====================================================================
class ReferenceBoardCaptureThread(QThread):
    frame_ready = Signal(object, float)        # display, seconds_left
    captured = Signal(object, object, tuple)   # raw_frame(BGR), corners, used_size
    error = Signal(str)
    stopped = Signal()

    def __init__(self, camera_index, board_w, board_h, wait_seconds=REFERENCE_CAPTURE_WAIT_S):
        super().__init__()
        self.camera_index = camera_index
        self.board_w = board_w
        self.board_h = board_h
        self.wait_seconds = wait_seconds
        self._running = False

    def request_stop(self):
        self._running = False

    def run(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            self.error.emit(f"Không thể mở camera index {self.camera_index}")
            self.stopped.emit()
            return

        self._running = True
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        t_start = time.time()

        try:
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    self.error.emit("Mất tín hiệu camera.")
                    break

                elapsed = time.time() - t_start
                remaining = self.wait_seconds - elapsed

                display = frame.copy()

                if remaining > 0:
                    cv2.putText(
                        display, f"Giu yen ban co... chup sau {remaining:.1f}s",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2,
                    )
                    self.frame_ready.emit(display, remaining)
                    continue

                # Đã hết thời gian đếm ngược -> CHỤP NGAY khung hình hiện
                # tại (chỉ 1 lần duy nhất), rồi dò góc bàn cờ trên đó.
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, corners, used_size = find_corners_auto(gray, self.board_w, self.board_h)

                if not found:
                    cv2.putText(
                        display, "Khong tim thay ban co trong anh vua chup!",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                    )
                    self.frame_ready.emit(display, 0.0)
                    self.error.emit(
                        "Đã chụp sau {:.0f}s nhưng không phát hiện được bàn cờ trong ảnh. "
                        "Hãy đặt lại bàn cờ cho rõ nét và bấm chụp lại.".format(self.wait_seconds)
                    )
                    break

                cv2.drawChessboardCorners(display, used_size, corners, found)
                self.frame_ready.emit(display, 0.0)

                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                self.captured.emit(frame, corners_refined, used_size)
                break
        except Exception as e:
            self.error.emit(str(e))
        finally:
            cap.release()
            self.stopped.emit()


# =====================================================================
# 2) HÀM CALIB (Y HỆT LOGIC calib_camera.py) - chạy trong FnWorker (nền)
# =====================================================================
def _format_matrix(mat, fmt="{:10.4f}"):
    """In một ma trận numpy 2D ra dạng chuỗi nhiều dòng, căn cột cho dễ đọc."""
    lines = []
    for row in np.atleast_2d(mat):
        lines.append("[ " + "  ".join(fmt.format(float(v)) for v in row) + " ]")
    return "\n".join(lines)


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

    # ---------------------------------------------------------------
    # HIỂN THỊ ĐẦY ĐỦ MA TRẬN NỘI (INTRINSIC) VÀ MA TRẬN NGOẠI (EXTRINSIC)
    # ---------------------------------------------------------------
    log_fn("")
    log_fn("========== MA TRẬN NỘI (INTRINSIC MATRIX - camera_matrix) ==========")
    log_fn(_format_matrix(camera_matrix))
    log_fn(
        f"(fx={fx:.4f}, fy={fy:.4f}, cx={cx:.4f}, cy={cy:.4f} - đơn vị: pixel)"
    )

    log_fn("")
    log_fn("---------- Hệ số méo (Distortion coefficients, dist_coeffs) ----------")
    dist_flat = dist_coeffs.reshape(-1)
    dist_names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    dist_str_parts = []
    for i, v in enumerate(dist_flat):
        name = dist_names[i] if i < len(dist_names) else f"d{i}"
        dist_str_parts.append(f"{name}={v:.6f}")
    log_fn("  " + ", ".join(dist_str_parts))

    log_fn("")
    log_fn("========== MA TRẬN NGOẠI (EXTRINSIC MATRIX) THEO TỪNG ẢNH ==========")
    used_paths = [path for path, shape, corners, size, img in detections if size == best_size]
    for idx_img, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        img_name = os.path.basename(used_paths[idx_img]) if idx_img < len(used_paths) else f"#{idx_img}"
        R, _ = cv2.Rodrigues(rvec)
        # Ma trận ngoại đầy đủ [R | t] (3x4): biến đổi từ hệ toạ độ bàn cờ
        # (world) sang hệ toạ độ camera.
        extrinsic_3x4 = np.hstack([R, tvec.reshape(3, 1)])
        log_fn(f"--- Ảnh: {img_name} ---")
        log_fn("  Rotation vector (rvec): "
               + "[ " + "  ".join(f"{float(v):.6f}" for v in rvec.reshape(-1)) + " ]")
        log_fn("  Rotation matrix (R):")
        log_fn(_format_matrix(R, fmt="{:9.5f}"))
        log_fn("  Translation vector (tvec, mm): "
               + "[ " + "  ".join(f"{float(v):.4f}" for v in tvec.reshape(-1)) + " ]")
        log_fn("  Ma trận ngoại [R | t] (3x4):")
        log_fn(_format_matrix(extrinsic_3x4, fmt="{:9.5f}"))
        log_fn("")

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
# 3) THU THẬP TOẠ ĐỘ THỰC TỪ ẢNH BÀN CỜ ĐÃ CHỤP (bước 3b)
# Khác với bản cũ (lưới toạ độ hoàn toàn lý thuyết dựa trên square_size
# nhập tay), hàm này ĐO trực tiếp khoảng cách pixel giữa các góc bàn cờ
# đã phát hiện được trong ảnh chụp thật, suy ra tỉ lệ mm/px THẬT, rồi mới
# quy đổi sang toạ độ X/Y (mm) - vì vậy phải có ảnh chụp ở "vị trí chuẩn"
# trước khi gọi hàm này.
# =====================================================================
def collect_real_board_coordinates(raw_frame_bgr, corners, used_size, square_size,
                                    origin_x, origin_y, calib_npz_path=None):
    bw, bh = used_size
    corners_px = corners.reshape(-1, 2).astype(np.float64)

    h, w = raw_frame_bgr.shape[:2]
    fx = fy = 1.0
    cx, cy = w / 2.0, h / 2.0

    if calib_npz_path and os.path.isfile(calib_npz_path):
        data = np.load(calib_npz_path)
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        new_camera_matrix, _roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 0, (w, h)
        )
        pts = corners_px.reshape(-1, 1, 2).astype(np.float32)
        undistorted_pts = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=new_camera_matrix)
        corners_px = undistorted_pts.reshape(-1, 2)
        fx, fy = new_camera_matrix[0, 0], new_camera_matrix[1, 1]
        cx, cy = new_camera_matrix[0, 2], new_camera_matrix[1, 2]

    pts_grid = corners_px.reshape(bh, bw, 2)

    # Đo khoảng cách pixel THẬT giữa các góc liền kề (ngang + dọc), rồi suy
    # ra tỉ lệ mm/px từ chính square_size khai báo -> phản ánh đúng board
    # thật đang đặt trước camera, không phải lưới lý thuyết.
    horiz = np.linalg.norm(pts_grid[:, 1:, :] - pts_grid[:, :-1, :], axis=2) if bw > 1 else np.array([])
    vert = np.linalg.norm(pts_grid[1:, :, :] - pts_grid[:-1, :, :], axis=2) if bh > 1 else np.array([])
    all_spacing = np.concatenate([horiz.ravel(), vert.ravel()])
    if all_spacing.size == 0 or float(np.mean(all_spacing)) <= 0:
        raise RuntimeError("Không đo được khoảng cách pixel giữa các góc bàn cờ.")

    avg_px_spacing = float(np.mean(all_spacing))
    scale_mm_per_px = square_size / avg_px_spacing

    # ĐIỂM NEO (anchor): dùng chính GÓC BÀN CỜ SỐ 0 (góc đầu tiên phát hiện
    # được, tức pts_grid[0, 0] - cũng là điểm có index=0 trong CSV xuất ra)
    # làm gốc quy đổi, KHÔNG dùng tâm ảnh (cx, cy) như trước nữa.
    anchor_px, anchor_py = pts_grid[0, 0]

    rows = []
    idx = 0
    for j in range(bh):
        for i in range(bw):
            px, py = pts_grid[j, i]
            # Cùng quy ước trục với Cameracircle.py: X hướng ngược pixel,
            # Y hướng lên trên so với điểm neo (đã khử méo nếu có calib).
            dx_px = -(px - anchor_px)
            dy_px = (anchor_py - py)
            x_mm = origin_x + dx_px * scale_mm_per_px
            y_mm = origin_y + dy_px * scale_mm_per_px
            rows.append((idx, x_mm, y_mm))
            idx += 1

    return rows, scale_mm_per_px, avg_px_spacing


def write_points_csv(csv_path, rows):
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x_mm", "y_mm"])
        writer.writerows(rows)


# =====================================================================
# 4) THREAD CHO ROBOT CHẠY QUA TỪNG ĐIỂM TRONG CSV
#
# Toàn bộ nội suy chuyển động (đi thẳng - hạ nhẹ - nâng lại - về vị trí
# Home ĐÃ LƯU) được TÁCH RIÊNG ra kinematics/board_points_motion.py
# (BoardPointsMotionPlanner). Thread này chỉ điều phối vòng lặp qua danh
# sách điểm ĐƯỢC GIAO (có thể là toàn bộ CSV, hoặc chỉ phần CÒN LẠI khi
# tiếp tục sau STOP) và phát tín hiệu log/tiến trình cho GUI.
#
# `finished_err` giờ mang thêm số điểm ĐÃ hoàn tất trước khi lỗi xảy ra
# (completed), để cửa sổ chính vẫn tính đúng được phần còn lại cần chạy
# tiếp, thay vì mất dấu tiến trình khi có lỗi giữa chừng.
# =====================================================================
class RunPointsThread(QThread):
    log_line = Signal(str)
    progress = Signal(int, int)     # (điểm hiện tại - trong danh sách được giao, tổng số điểm được giao)
    finished_ok = Signal(int)       # số điểm đã hoàn tất trong danh sách được giao
    finished_err = Signal(str, int)  # (thông báo lỗi, số điểm đã hoàn tất trước khi lỗi)

    def __init__(self, motion, points, z_travel, z_down=0.0, dwell_s=1.0, dwell_down_s=1.0):
        super().__init__()
        self.motion = motion              # BoardPointsMotionPlanner (đã tạo sẵn ở GUI thread)
        self.points = points              # danh sách điểm SẼ CHẠY trong lần này (có thể là phần còn lại)
        self.z_travel = z_travel
        self.z_down = z_down              # khoảng cách hạ thêm xuống (mm), "+" là xuống
        self.dwell_s = dwell_s
        self.dwell_down_s = dwell_down_s  # thời gian dừng lại khi đã ở dưới đáy
        self._stop_requested = False

    def request_stop(self):
        self._stop_requested = True
        self.motion.stop()

    def run(self):
        import sys
        old_stdout = sys.stdout
        sys.stdout = StreamToSignal(self.log_line.emit)
        completed = 0
        try:
            total = len(self.points)

            for i, (x_mm, y_mm) in enumerate(self.points, 1):
                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Người dùng yêu cầu dừng trước điểm {i}/{total}.")
                    break

                self.log_line.emit(f"--- Điểm {i}/{total}: ({x_mm:.1f}, {y_mm:.1f}) ---")

                ok = self.motion.visit_point(
                    x_mm, y_mm,
                    z_travel=self.z_travel,
                    z_down=self.z_down,
                    dwell_down_s=self.dwell_down_s,
                )
                if not ok:
                    self.log_line.emit(f"[DỪNG] Đã dừng giữa chừng tại điểm {i}/{total}.")
                    break

                # Không còn "Home vật lý" cho từng điểm - chỉ là di chuyển
                # thẳng về toạ độ Home đã lưu, không chờ READY từ STM32.
                self.log_line.emit(f"    ✔ Đã về vị trí Home đã lưu sau điểm {i}/{total}.")

                completed = i
                self.progress.emit(i, total)

                time.sleep(self.dwell_s)

                if self._stop_requested:
                    self.log_line.emit(f"[DỪNG] Đã dừng sau khi hoàn tất điểm {i}/{total}.")
                    break

            self.finished_ok.emit(completed)
        except (MotionError, Exception) as e:
            self.finished_err.emit(str(e), completed)
        finally:
            sys.stdout = old_stdout


# =====================================================================
# CỬA SỔ CHÍNH
# =====================================================================
class BoardCalibWindow(BaseTabWindow):
    def __init__(self, ctx, launcher):
        super().__init__(ctx, launcher, "🏁 HIỆU CHỈNH CAMERA BẰNG BÀN CỜ")
        self._capture_thread = None
        self._ref_capture_thread = None
        self._run_thread = None
        self._active_motion = None   # BoardPointsMotionPlanner đang chạy (nếu có)

        # Trạng thái cho quy trình 3 bước (3a chụp -> 3b thu thập -> 3c xuất)
        self._captured_frame = None
        self._captured_corners = None
        self._captured_board_size = None
        self._collected_rows = None

        # ---- Trạng thái TIẾP TỤC (resume) cho bước 4 (Run Robot) ----
        # Toàn bộ danh sách điểm đã load từ CSV cho "phiên chạy" hiện tại,
        # cùng đường dẫn CSV tương ứng và số điểm ĐÃ hoàn tất (đã về Home
        # đã lưu). Khi bấm START lần nữa với CÙNG file CSV này, nếu vẫn
        # còn điểm chưa chạy -> chạy tiếp từ điểm kế tiếp (không load lại
        # từ đầu). Nếu đổi CSV khác hoặc đã chạy hết -> load lại từ đầu.
        self._csv_points_cache = None
        self._csv_points_path = None
        self._points_done_count = 0

        # Thông tin của LẦN CHẠY hiện tại (dùng để quy đổi tiến trình
        # tương đối trong RunPointsThread về tiến trình tuyệt đối trên
        # progress bar, và để cập nhật lại _points_done_count khi xong).
        self._run_start_index = 0
        self._run_total_points = 0

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
            "background-color:#0f2540; border:2px solid #1a7ee0; border-radius:10px; color:#b9d2ee;"
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
        g1 = QGroupBox("1) CHỤP ẢNH")
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

        l1.addWidget(QLabel("W:"), 0, 0); l1.addWidget(self.spin_board_w, 0, 1)
        l1.addWidget(QLabel("H:"), 1, 0); l1.addWidget(self.spin_board_h, 1, 1)
        l1.addWidget(QLabel("SỐ ẢNH CẦN CHỤP:"), 2, 0); l1.addWidget(self.spin_num_images, 2, 1)
        l1.addWidget(QLabel("THƯ MỤC:"), 3, 0); l1.addWidget(self.edit_images_dir, 3, 1)

        self.progress_capture = QProgressBar()
        self.progress_capture.setFormat("0/%d ẢNH" % self.spin_num_images.value())
        l1.addWidget(self.progress_capture, 4, 0, 1, 2)

        cap_btn_row = QVBoxLayout()
        cap_btn_row.setSpacing(8)
        self.btn_capture_start = _make_blue_button("▶ BẮT ĐẦU")
        self.btn_capture_start.clicked.connect(self.on_start_capture)
        self.btn_capture_stop = _make_stop_button("■ DỪNG")
        self.btn_capture_stop.clicked.connect(self.on_stop_capture)
        self.btn_capture_stop.setEnabled(False)
        cap_btn_row.addWidget(self.btn_capture_start)
        cap_btn_row.addWidget(self.btn_capture_stop)
        l1.addLayout(cap_btn_row, 5, 0, 1, 2)

        ctrl_col.addWidget(g1)

        # ---- 2) CALIB -> .npz ----
        g2 = QGroupBox("2) HIỆU CHỈNH CAMERA -> .NPZ")
        g2.setObjectName("compactBox")
        l2 = QGridLayout(g2)
        l2.setVerticalSpacing(10)
        l2.setHorizontalSpacing(10)
        self.spin_square_size = QDoubleSpinBox()
        self.spin_square_size.setRange(1.0, 200.0)
        self.spin_square_size.setValue(float(cfg.get("board_square_size", 9.96)))
        self.edit_output_name = QLineEdit(cfg.get("board_calib_output", "calibration_result"))

        l2.addWidget(QLabel("KÍCH THƯỚC Ô VUÔNG (mm):"), 0, 0); l2.addWidget(self.spin_square_size, 0, 1)
        l2.addWidget(QLabel("TỆP XUẤT:"), 1, 0); l2.addWidget(self.edit_output_name, 1, 1)

        self.btn_calib_run = _make_blue_button("▶ CHẠY HIỆU CHỈNH")
        self.btn_calib_run.clicked.connect(self.on_run_calibration)
        l2.addWidget(self.btn_calib_run, 2, 0, 1, 2)

        ctrl_col.addWidget(g2)

        # ---- 3) CHỤP CHUẨN -> THU THẬP TOẠ ĐỘ -> XUẤT .csv ----
        g3 = QGroupBox("3) ẢNH CHUẨN → THU THẬP TỌA ĐỘ → XUẤT CSV")
        g3.setObjectName("compactBox")
        l3 = QGridLayout(g3)
        l3.setVerticalSpacing(10)
        l3.setHorizontalSpacing(10)
        self.spin_origin_x = QDoubleSpinBox(); self.spin_origin_x.setRange(-1000, 1000)
        self.spin_origin_x.setValue(float(cfg.get("board_origin_x", 0.0)))
        self.spin_origin_y = QDoubleSpinBox(); self.spin_origin_y.setRange(-1000, 1000)
        self.spin_origin_y.setValue(float(cfg.get("board_origin_y", 0.0)))
        self.edit_csv_path = QLineEdit(cfg.get("board_csv_path", "board_points.csv"))
        btn_browse_csv_save = _make_blue_button(" CHỌN NƠI LƯU")
        btn_browse_csv_save.clicked.connect(self.on_browse_csv_save)

        l3.addWidget(QLabel("Xo (mm):"), 0, 0); l3.addWidget(self.spin_origin_x, 0, 1)
        l3.addWidget(QLabel("Yo (mm):"), 1, 0); l3.addWidget(self.spin_origin_y, 1, 1)
        l3.addWidget(QLabel("TỆP CSV:"), 2, 0); l3.addWidget(self.edit_csv_path, 2, 1)
        l3.addWidget(btn_browse_csv_save, 3, 0, 1, 2)

        self.lbl_step_status = QLabel("BƯỚC 1/3: CHƯA CHỤP ẢNH.")
        self.lbl_step_status.setWordWrap(True)
        self.lbl_step_status.setStyleSheet("color:#12315c; font-weight:600;")
        l3.addWidget(self.lbl_step_status, 4, 0, 1, 2)

        self.btn_capture_ref = _make_blue_button("📷 1. CHỤP ẢNH CHUẨN")
        self.btn_capture_ref.clicked.connect(self.on_capture_reference)
        l3.addWidget(self.btn_capture_ref, 5, 0, 1, 2)

        self.btn_cancel_ref = _make_stop_button("■ HỦY CHỤP")
        self.btn_cancel_ref.clicked.connect(self.on_cancel_reference)
        self.btn_cancel_ref.setEnabled(False)
        l3.addWidget(self.btn_cancel_ref, 6, 0, 1, 2)

        self.btn_collect_coords = _make_blue_button("📐 2. THU THẬP TỌA ĐỘ")
        self.btn_collect_coords.clicked.connect(self.on_collect_coordinates)
        self.btn_collect_coords.setEnabled(False)
        l3.addWidget(self.btn_collect_coords, 7, 0, 1, 2)

        self.btn_export_csv = _make_blue_button("💾 3. XUẤT CSV")
        self.btn_export_csv.clicked.connect(self.on_export_csv)
        self.btn_export_csv.setEnabled(False)
        l3.addWidget(self.btn_export_csv, 8, 0, 1, 2)

        ctrl_col.addWidget(g3)

        # ---- 4) CHẠY ROBOT QUA CÁC ĐIỂM CSV ----
        g4 = QGroupBox("4) CHẠY ROBOT (CSV)")
        g4.setObjectName("compactBox")
        l4 = QGridLayout(g4)
        l4.setVerticalSpacing(10)
        l4.setHorizontalSpacing(10)
        self.edit_run_csv_path = QLineEdit(cfg.get("board_csv_path", "board_points.csv"))
        btn_browse_csv_load = _make_blue_button("CHỌN TỆP CSV...")
        btn_browse_csv_load.clicked.connect(self.on_browse_csv_load)

        self.spin_z_travel = QDoubleSpinBox(); self.spin_z_travel.setRange(-500, 500)
        self.spin_z_travel.setValue(float(cfg.get("board_z_travel", 300.0)))

        # Z down: khoảng cách hạ THÊM xuống tại mỗi điểm, quy ước "+" là
        # đi XUỐNG. Đặt = 0 để tắt tính năng này (giữ hành vi cũ).
        self.spin_z_down = QDoubleSpinBox(); self.spin_z_down.setRange(0, 500)
        self.spin_z_down.setValue(float(cfg.get("board_z_down", 0.0)))

        # Thời gian dừng lại khi đã ở dưới đáy (z_travel + z_down), trước
        # khi nâng lên lại. Chỉ có tác dụng khi Z down > 0.
        self.spin_dwell_down = QDoubleSpinBox(); self.spin_dwell_down.setRange(0.0, 10.0)
        self.spin_dwell_down.setSingleStep(0.5)
        self.spin_dwell_down.setValue(float(cfg.get("board_dwell_down_s", 1.0)))

        self.spin_dwell = QDoubleSpinBox(); self.spin_dwell.setRange(0.0, 10.0)
        self.spin_dwell.setSingleStep(0.5)
        self.spin_dwell.setValue(float(cfg.get("board_dwell_s", 1.0)))

        l4.addWidget(QLabel("TỆP CSV:"), 0, 0); l4.addWidget(self.edit_run_csv_path, 0, 1)
        l4.addWidget(btn_browse_csv_load, 1, 0, 1, 2)
        l4.addWidget(QLabel("Z DI CHUYỂN (mm):"), 2, 0); l4.addWidget(self.spin_z_travel, 2, 1)
        l4.addWidget(QLabel("Z HẠ (mm, + LÀ XUỐNG):"), 3, 0); l4.addWidget(self.spin_z_down, 3, 1)
        l4.addWidget(QLabel("THỜI GIAN DỪNG KHI XUỐNG (s):"), 4, 0); l4.addWidget(self.spin_dwell_down, 4, 1)
        l4.addWidget(QLabel("THỜI GIAN TRỄ MỖI ĐIỂM (s):"), 5, 0); l4.addWidget(self.spin_dwell, 5, 1)

        # ---- Thanh trượt tốc độ di chuyển (mm/s), kéo được ----
        l4.addWidget(QLabel("TỐC ĐỘ DI CHUYỂN (mm/s):"), 6, 0, 1, 2)
        self.slider_run_speed = QSlider(Qt.Horizontal)
        self.slider_run_speed.setObjectName("boardRunSpeedSlider")
        self.slider_run_speed.setMinimum(RUN_SPEED_MIN)
        self.slider_run_speed.setMaximum(RUN_SPEED_MAX)
        self.slider_run_speed.setValue(int(cfg.get("board_run_speed", RUN_SPEED_DEFAULT)))
        self.slider_run_speed.setTickPosition(QSlider.TicksBelow)
        self.slider_run_speed.setTickInterval(10)
        self.slider_run_speed.valueChanged.connect(self._on_run_speed_changed)
        l4.addWidget(self.slider_run_speed, 7, 0, 1, 2)

        self.lbl_run_speed_value = QLabel(f"{self.slider_run_speed.value()} mm/s")
        self.lbl_run_speed_value.setAlignment(Qt.AlignCenter)
        self.lbl_run_speed_value.setStyleSheet("font-size:12pt; font-weight:600; color:#1a7ee0;")
        l4.addWidget(self.lbl_run_speed_value, 8, 0, 1, 2)

        self.progress_run = QProgressBar()
        self.progress_run.setFormat("SẴN SÀNG")
        l4.addWidget(self.progress_run, 9, 0, 1, 2)

        run_btn_row = QVBoxLayout()
        run_btn_row.setSpacing(8)
        self.btn_run_start = _make_blue_button("▶ BẮT ĐẦU")
        self.btn_run_start.clicked.connect(self.on_run_start)
        self.btn_run_stop = _make_stop_button("■ DỪNG VÀ VỀ HOME")
        self.btn_run_stop.clicked.connect(self.on_run_stop)
        self.btn_run_stop.setEnabled(False)
        run_btn_row.addWidget(self.btn_run_start)
        run_btn_row.addWidget(self.btn_run_stop)
        l4.addLayout(run_btn_row, 10, 0, 1, 2)

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
        cfg["board_z_down"] = self.spin_z_down.value()
        cfg["board_dwell_down_s"] = self.spin_dwell_down.value()
        cfg["board_dwell_s"] = self.spin_dwell.value()
        cfg["board_run_speed"] = self.slider_run_speed.value()
        save_config(cfg)

    def _any_camera_busy(self):
        return (
            self.ctx.camera_thread is not None
            or self.ctx.dynamic_camera_thread is not None
            or self._capture_thread is not None
            or self._ref_capture_thread is not None
        )

    # ---------------- Tốc độ di chuyển (bước 4) ----------------
    def _on_run_speed_changed(self, value):
        self.lbl_run_speed_value.setText(f"{value} mm/s")
        if self._active_motion is not None:
            # An toàn để chỉnh ngay cả khi đang chạy - chỉ áp dụng cho
            # đoạn di chuyển tiếp theo, không ngắt chuyển động hiện tại.
            self._active_motion.set_speed(value)
        self.ctx.cfg["board_run_speed"] = value
        save_config(self.ctx.cfg)

    # ---------------- 1) Chụp ảnh ----------------
    def on_start_capture(self):
        if self._capture_thread is not None:
            return
        if self._any_camera_busy():
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Một tab camera khác đang mở - hãy đóng trước.")
            return

        self._save_cfg()
        target = self.spin_num_images.value()
        self.progress_capture.setRange(0, target)
        self.progress_capture.setValue(0)
        self.progress_capture.setFormat(f"0/{target} ẢNH")
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
        self.progress_capture.setFormat(f"{saved_count}/{self.spin_num_images.value()} ẢNH")

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

    # ---------------- 3a) Chụp ảnh bàn cờ ở vị trí chuẩn ----------------
    def on_browse_csv_save(self):
        path, _ = QFileDialog.getSaveFileName(self, "Chọn nơi lưu CSV", self.edit_csv_path.text(), "CSV (*.csv)")
        if path:
            self.edit_csv_path.setText(path)

    def on_capture_reference(self):
        if self._ref_capture_thread is not None:
            return
        if self._any_camera_busy():
            QMessageBox.warning(self, "Đang chạy tab khác",
                                 "Một tác vụ camera khác đang mở - hãy dừng trước.")
            return

        self._save_cfg()

        # Chụp ảnh mới -> mọi kết quả thu thập/xuất cũ trước đó không còn
        # hợp lệ, phải làm lại từ đầu theo đúng thứ tự 1 -> 2 -> 3.
        self._captured_frame = None
        self._captured_corners = None
        self._captured_board_size = None
        self._collected_rows = None
        self.btn_collect_coords.setEnabled(False)
        self.btn_export_csv.setEnabled(False)

        self.lbl_step_status.setText(
            f"BƯỚC 1/3: GIỮ YÊN BÀN CỜ - SẼ TỰ ĐỘNG CHỤP SAU {REFERENCE_CAPTURE_WAIT_S:.1f}S..."
        )
        self._append_log(
            f"=== BƯỚC 1: Đặt bàn cờ ở vị trí chuẩn và giữ yên trong {REFERENCE_CAPTURE_WAIT_S:.0f}s tới lúc tự chụp ==="
        )
        self.btn_capture_ref.setEnabled(False)
        self.btn_cancel_ref.setEnabled(True)

        self._ref_capture_thread = ReferenceBoardCaptureThread(
            camera_index=self.ctx.cfg.get("camera_index", 0),
            board_w=self.spin_board_w.value(),
            board_h=self.spin_board_h.value(),
            wait_seconds=REFERENCE_CAPTURE_WAIT_S,
        )
        self._ref_capture_thread.frame_ready.connect(self._on_ref_frame)
        self._ref_capture_thread.captured.connect(self._on_ref_captured)
        self._ref_capture_thread.error.connect(self._on_ref_error)
        self._ref_capture_thread.stopped.connect(self._on_ref_stopped)
        self._ref_capture_thread.start()

    def on_cancel_reference(self):
        if self._ref_capture_thread is not None:
            self._ref_capture_thread.request_stop()

    def _on_ref_frame(self, frame, seconds_left):
        pix = cv2_to_qpixmap(frame)
        self.video_label.setPixmap(
            pix.scaled(self.video_label.width(), self.video_label.height(),
                       Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )
        if seconds_left > 0:
            self.lbl_step_status.setText(
                f"BƯỚC 1/3: GIỮ YÊN BÀN CỜ - SẼ TỰ ĐỘNG CHỤP SAU {seconds_left:.1f}S..."
            )

    def _on_ref_captured(self, raw_frame, corners, used_size):
        self._captured_frame = raw_frame
        self._captured_corners = corners
        self._captured_board_size = used_size
        self.lbl_step_status.setText(
            f"BƯỚC 1/3 HOÀN TẤT: ĐÃ CHỤP BÀN CỜ {used_size[0]}x{used_size[1]}. "
            f"TIẾP TỤC BƯỚC 2."
        )
        self._append_log(
            f"✔ BƯỚC 1 xong: đã chụp bàn cờ {used_size[0]}x{used_size[1]} ở vị trí chuẩn (sau {REFERENCE_CAPTURE_WAIT_S:.0f}s)."
        )
        self.btn_collect_coords.setEnabled(True)

    def _on_ref_error(self, msg):
        self._append_log(f"⚠ {msg}")
        QMessageBox.warning(self, "Chưa chụp được ảnh chuẩn", msg)

    def _on_ref_stopped(self):
        self._ref_capture_thread = None
        self.btn_capture_ref.setEnabled(True)
        self.btn_cancel_ref.setEnabled(False)
        if self._captured_corners is None:
            self.video_label.setText("Camera đã dừng")
            self.lbl_step_status.setText("BƯỚC 1/3: CHƯA CHỤP ẢNH.")

    # ---------------- 3b) Thu thập toạ độ thực từ ảnh đã chụp ----------------
    def on_collect_coordinates(self):
        if self._captured_corners is None or self._captured_frame is None:
            QMessageBox.warning(
                self, "Chưa có ảnh chuẩn",
                "Hãy thực hiện Bước 1 (chụp ảnh bàn cờ ở vị trí chuẩn) trước.",
            )
            return

        self._save_cfg()

        # BẮT BUỘC dùng đúng file calib.npz vừa xuất ra từ Bước 2
        # (CALIB -> .npz) TRONG CHÍNH CỬA SỔ NÀY - KHÔNG âm thầm rơi về
        # ctx.cfg["calib_file"] (file calib mặc định chung của app).
        calib_npz_path = f"{self.edit_output_name.text()}.npz"
        if not os.path.isfile(calib_npz_path):
            reply = QMessageBox.question(
                self, "Chưa có file CALIB",
                f"Không tìm thấy file '{calib_npz_path}'.\n"
                f"Hãy chạy Bước 2 (CALIB -> .npz) trước để có kết quả chính xác nhất.\n\n"
                f"Bạn có muốn tiếp tục thu thập toạ độ KHÔNG khử méo ống kính "
                f"(kém chính xác hơn) không?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            calib_npz_path = None
            self._append_log(
                "⚠ Chưa có file CALIB của Bước 2 - thu thập toạ độ KHÔNG khử méo ống kính."
            )
        else:
            self._append_log(f"Dùng file CALIB (Bước 2): {calib_npz_path}")

        try:
            rows, scale_mm_per_px, avg_px_spacing = collect_real_board_coordinates(
                self._captured_frame,
                self._captured_corners,
                self._captured_board_size,
                self.spin_square_size.value(),
                self.spin_origin_x.value(),
                self.spin_origin_y.value(),
                calib_npz_path=calib_npz_path,
            )
        except Exception as e:
            self._append_log(f"⚠ LỖI khi thu thập tọa độ: {e}")
            QMessageBox.critical(self, "Lỗi thu thập tọa độ", str(e))
            return

        self._collected_rows = rows
        self.lbl_step_status.setText(
            f"BƯỚC 2/3 HOÀN TẤT: ĐÃ THU THẬP {len(rows)} ĐIỂM "
            f"(TỶ LỆ ≈ {scale_mm_per_px:.4f} mm/px). TIẾP TỤC BƯỚC 3."
        )
        self._append_log(
            f"✔ BƯỚC 2 xong: thu thập {len(rows)} toạ độ từ ảnh chụp thật "
            f"(tỉ lệ đo được ≈ {scale_mm_per_px:.4f} mm/px, khoảng cách ô "
            f"trung bình ≈ {avg_px_spacing:.1f}px)."
        )
        self.btn_export_csv.setEnabled(True)

    # ---------------- 3c) Xuất CSV ----------------
    def on_export_csv(self):
        if not self._collected_rows:
            QMessageBox.warning(
                self, "Chưa có tọa độ",
                "Hãy thực hiện Bước 1 (chụp ảnh chuẩn) và Bước 2 (thu thập tọa độ) trước khi xuất CSV.",
            )
            return

        self._save_cfg()
        path = self.edit_csv_path.text()
        try:
            write_points_csv(path, self._collected_rows)
            self.edit_run_csv_path.setText(path)
            self.lbl_step_status.setText(f"BƯỚC 3/3 HOÀN TẤT: ĐÃ XUẤT {len(self._collected_rows)} ĐIỂM.")
            self._append_log(
                f"✔ BƯỚC 3 xong: đã xuất {len(self._collected_rows)} điểm vào '{path}'."
            )
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

        # ---- Quyết định: TIẾP TỤC (resume) hay LOAD LẠI TỪ ĐẦU ----
        # Chỉ resume khi: (a) đã có cache của đúng file CSV này, và
        # (b) cache đó CHƯA chạy hết (còn điểm dở dang). Ngược lại (đổi
        # CSV khác, hoặc lần trước đã hoàn tất toàn bộ) -> đọc lại từ đầu.
        can_resume = (
            self._csv_points_cache is not None
            and self._csv_points_path == csv_path
            and 0 < self._points_done_count < len(self._csv_points_cache)
        )

        if can_resume:
            points_all = self._csv_points_cache
            start_index = self._points_done_count
            self._append_log(
                f"↪ TIẾP TỤC: đã hoàn tất {start_index}/{len(points_all)} điểm trước đó - "
                f"chạy tiếp từ điểm {start_index + 1}."
            )
        else:
            try:
                points_all = self._load_points_from_csv(csv_path)
            except Exception as e:
                QMessageBox.critical(self, "Lỗi đọc CSV", str(e))
                return
            if not points_all:
                QMessageBox.warning(self, "File rỗng", "File CSV không có điểm nào.")
                return

            self._csv_points_cache = points_all
            self._csv_points_path = csv_path
            self._points_done_count = 0
            start_index = 0
            self.log_view.clear()

        points_to_run = points_all[start_index:]
        if not points_to_run:
            QMessageBox.information(
                self, "Đã chạy xong",
                "File CSV này đã được chạy hết toàn bộ điểm rồi.\n"
                "Bấm lại START để chạy lại từ đầu, hoặc chọn file CSV khác.",
            )
            # Cho phép lần bấm START kế tiếp load lại từ đầu.
            self._csv_points_cache = None
            self._csv_points_path = None
            self._points_done_count = 0
            return

        self._save_cfg()
        self.progress_run.setRange(0, len(points_all))
        self.progress_run.setValue(start_index)
        self.progress_run.setFormat(f"{start_index}/{len(points_all)}")

        self.ctx.busy = True
        self.btn_run_start.setEnabled(False)
        self.btn_run_stop.setEnabled(True)

        # Lưu lại để quy đổi tiến trình tương đối (từ RunPointsThread) về
        # tiến trình tuyệt đối trên progress bar, và để cộng dồn đúng khi
        # lần chạy này kết thúc (dù hoàn tất hay bị dừng/lỗi giữa chừng).
        self._run_start_index = start_index
        self._run_total_points = len(points_all)

        # Quỹ đạo chuyển động RIÊNG (kinematics/board_points_motion.py) -
        # tạo ở GUI thread để thanh trượt tốc độ có thể chỉnh trực tiếp lên
        # đối tượng này ngay cả khi đang chạy.
        motion = BoardPointsMotionPlanner(self.ctx.planner, self.ctx.robot_uart)
        motion.set_speed(self.slider_run_speed.value())
        self._active_motion = motion

        self._run_thread = RunPointsThread(
            motion, points_to_run,
            z_travel=self.spin_z_travel.value(),
            z_down=self.spin_z_down.value(),
            dwell_s=self.spin_dwell.value(),
            dwell_down_s=self.spin_dwell_down.value(),
        )
        self._run_thread.log_line.connect(self._append_log)
        self._run_thread.progress.connect(self._on_run_progress)
        self._run_thread.finished_ok.connect(self._on_run_finished_ok)
        self._run_thread.finished_err.connect(self._on_run_finished_err)
        self._run_thread.finished.connect(self._on_run_thread_finished)
        self._run_thread.start()

    def _on_run_progress(self, current, total):
        # current/total là tiến trình TƯƠNG ĐỐI trong danh sách điểm được
        # giao cho lần chạy này (points_to_run) - quy đổi về tuyệt đối
        # theo toàn bộ CSV để progress bar hiển thị đúng khi đang resume.
        absolute_current = self._run_start_index + current
        absolute_total = self._run_total_points
        self.progress_run.setValue(absolute_current)
        self.progress_run.setFormat(f"{absolute_current}/{absolute_total}")

    def _mark_run_progress(self, completed_in_this_run):
        """Cập nhật _points_done_count (tuyệt đối) sau khi 1 lần chạy kết
        thúc (dù hoàn tất hết, bị dừng, hay bị lỗi giữa chừng). Trả về
        True nếu đã chạy HẾT toàn bộ CSV (không còn gì để tiếp tục)."""
        self._points_done_count = self._run_start_index + completed_in_this_run
        total = self._run_total_points
        if self._points_done_count >= total:
            # Đã xong toàn bộ - xoá cache để lần START kế tiếp load lại
            # từ đầu (một vòng chạy mới hoàn toàn).
            self._csv_points_cache = None
            self._csv_points_path = None
            self._points_done_count = 0
            return True
        return False

    def _on_run_finished_ok(self, completed):
        self.ctx.busy = False
        finished_all = self._mark_run_progress(completed)
        if finished_all:
            self._append_log(f"=== HOÀN TẤT (Done): đã chạy hết toàn bộ {self._run_total_points} điểm. ===")
        else:
            done = self._points_done_count
            total = self._run_total_points
            self._append_log(
                f"⏸ Đã dừng - hoàn tất {done}/{total} điểm. Bấm START để chạy tiếp từ điểm {done + 1}."
            )
        self.btn_run_start.setEnabled(True)
        self.btn_run_stop.setEnabled(False)

    def _on_run_finished_err(self, err, completed):
        self.ctx.busy = False
        finished_all = self._mark_run_progress(completed)
        self._append_log(f"⚠ LỖI: {err}")
        if not finished_all:
            done = self._points_done_count
            total = self._run_total_points
            self._append_log(
                f"⏸ Đã dừng vì lỗi - hoàn tất {done}/{total} điểm. Bấm START để chạy tiếp từ điểm {done + 1}."
            )
        self.btn_run_start.setEnabled(True)
        self.btn_run_stop.setEnabled(False)

    def _on_run_thread_finished(self):
        self._run_thread = None
        self._active_motion = None

    def on_run_stop(self):
        """STOP: dừng vòng chạy điểm NGAY SAU điểm hiện tại (điểm đang
        chạy dở vẫn hoàn tất bình thường, không bị cắt ngang giữa chừng
        một cách nguy hiểm), rồi về HẲN HOME VẬT LÝ (chạm công tắc hành
        trình, chờ READY từ STM32) - bước an toàn cuối cùng khi người
        dùng chủ động dừng.

        Số điểm ĐÃ hoàn tất trước khi dừng được `_on_run_finished_ok`/
        `_on_run_finished_err` (phát ra ngay khi RunPointsThread kết
        thúc, TRƯỚC khi hàm này gọi Home vật lý xong) ghi lại vào
        `_points_done_count`. Nhờ vậy, khi người dùng bấm lại START, hệ
        thống biết chính xác cần chạy tiếp từ điểm nào - không chạy lại
        từ đầu."""
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
            # Đồng bộ lại vị trí PHẦN MỀM (planner.current_pos) khớp với vị
            # trí VẬT LÝ thật sau khi Home: robot vừa chạm công tắc hành
            # trình nên chắc chắn đang ở đúng planner.HOME. Nếu không đồng
            # bộ lại, lần START tiếp theo sẽ tính quãng đường di chuyển từ
            # một current_pos SAI (vị trí trước khi Home), gây giật/lệch ở
            # đoạn di chuyển đầu tiên.
            try:
                self.ctx.planner.current_pos = list(self.ctx.planner.HOME)
            except Exception:
                pass
        else:
            self._append_log("⚠ Không nhận được READY từ STM32.")
        self.btn_run_start.setEnabled(True)

    # ---------------- Đóng cửa sổ ----------------
    def closeEvent(self, event):
        if self._capture_thread is not None:
            self._capture_thread.request_stop()
            self._capture_thread.wait(2000)
        if self._ref_capture_thread is not None:
            self._ref_capture_thread.request_stop()
            self._ref_capture_thread.wait(2000)
        if self._run_thread is not None:
            self._run_thread.request_stop()
            self._run_thread.wait(2000)
        super().closeEvent(event)