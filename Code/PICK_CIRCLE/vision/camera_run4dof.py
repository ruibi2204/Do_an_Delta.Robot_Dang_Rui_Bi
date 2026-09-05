import os
import threading
import time

import cv2
import numpy as np

from vision.Camera_4dof import (
    undistort_image,
    detect_objects,
    detect_frame_holes,
    mold_bounding_region,
    is_inside_region,
    _build_red_mask,
    _CIRCLE_CALIB,
)

# Import các mốc thời gian di chuyển THẬT của robot (đã dùng trong
# move_delta_4dof.py) để tính độ trễ camera->gắp CHUẨN hơn thay vì để
# một con số 0.5s đoán mò. Nếu vì lý do gì đó không import được (thiếu
# file, sai đường dẫn project...) thì rơi về giá trị mặc định an toàn.
try:
    from kinematics.move_delta_4dof import (
        TIME_MOVE_FAST, TIME_MOVE_DOWN, TIME_MOVE_ACTION,
    )
except Exception:
    TIME_MOVE_FAST, TIME_MOVE_DOWN, TIME_MOVE_ACTION = 0.6, 0.15, 0.15

DET_SCALE = 0.5

# Tỉ lệ kích thước ô vuông ROI so với chiều nhỏ nhất của khung hình (giống
# cách dynamic_window.py đang tính ROI: roi_size = min(w, h) // 3).
ROI_SIZE_RATIO = 1.0 / 3.0

# Dịch ô ROI sang trái một khoảng cố định (mm) để bù lệch vị trí thực tế
# của vùng nhận diện so với khung hình camera. Quy đổi sang pixel bằng
# scale mm<->px đang dùng trong _CIRCLE_CALIB (xem chỗ tính shift_left_px
# bên dưới, trong run_camera_run4dof_stream).
ROI_SHIFT_LEFT_MM = 10.0

# Dịch ô ROI xuống DƯỚI một khoảng cố định (mm) - ROI mặc định đặt sát mép
# trên (roi_y = 0), số này > 0 sẽ đẩy ô xuống thấp hơn. Quy đổi mm -> pixel
# giống hệt cách làm với ROI_SHIFT_LEFT_MM.
ROI_SHIFT_DOWN_MM = 10.0

# Màu vẽ khung ROI (BGR) để phân biệt trên ảnh hiển thị.
ROI_OBJECT_COLOR = (255, 0, 0)      # xanh dương - vùng dò VẬT
ROI_FRAME_COLOR = (0, 165, 255)     # cam - vùng dò KHUNG

# Màu vẽ điểm DỰ ĐOÁN (vị trí vật/khung tại thời điểm robot thực sự gắp).
PREDICT_COLOR = (255, 0, 255)       # hồng/magenta - dễ phân biệt với đỏ/xanh lá


class _ThreadedCapture:
    """Đọc frame từ camera trong 1 thread riêng, luôn giữ frame MỚI NHẤT.

    Lý do: cv2.VideoCapture.read() là lệnh BLOCKING - main loop phải đợi
    camera trả frame mới trước khi xử lý tiếp. Nếu tốc độ xử lý (undistort +
    detect) nhanh hơn tốc độ camera gửi frame thì không sao, nhưng nếu có
    độ trễ driver/USB, main loop sẽ bị "ăn" theo tốc độ chậm nhất. Tách
    riêng 1 thread chỉ lo đọc frame liên tục giúp main loop luôn lấy được
    frame mới nhất ngay khi cần, không phải chờ đợi -> tăng FPS thực tế.
    """

    def __init__(self, camera_index, target_fps=30.0, brightness=80.0):
        backend = cv2.CAP_DSHOW if os.name == "nt" else 0
        self.cap = cv2.VideoCapture(camera_index, backend) if os.name == "nt" else cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Không thể mở camera index {camera_index}")

        # Ép định dạng nén MJPG: hầu hết webcam USB truyền MJPG nhanh hơn
        # nhiều so với YUYV mặc định ở cùng độ phân giải, giúp camera thực
        # sự đạt được framerate cao thay vì bị giới hạn băng thông USB.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FPS, target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if brightness != 0.0:
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)

        self._lock = threading.Lock()
        self._frame = None
        self._ret = False
        self._stopped = False
        # Báo hiệu "đã có frame mới" - dùng để main loop CHỜ frame thật sự
        # mới thay vì xử lý lặp lại cùng 1 frame nhiều lần (nguyên nhân
        # khiến FPS đo được bị "ảo" lên rất cao, ví dụ 100+, trong khi
        # camera vật lý chỉ gửi ~30 frame/giây).
        self._new_frame_event = threading.Event()
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

    def _update(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            with self._lock:
                self._ret = ret
                self._frame = frame
            if ret:
                self._new_frame_event.set()
            else:
                # Tránh vòng lặp busy khi mất kết nối camera.
                time.sleep(0.05)

    def read(self, timeout=1.0):
        """Chờ tới khi có frame MỚI (khác với lần read() trước) rồi trả về.
        Nhờ vậy main loop không bao giờ xử lý trùng 1 frame -> FPS đo được
        phản ánh đúng tốc độ camera thật, không bị thổi phồng."""
        got_new = self._new_frame_event.wait(timeout=timeout)
        with self._lock:
            self._new_frame_event.clear()
            if self._frame is None:
                return False, None
            return self._ret, self._frame.copy()

    def release(self):
        self._stopped = True
        self._thread.join(timeout=1.0)
        self.cap.release()


# ==== Dự đoán vị trí/góc gắp vật khi vật đặt trên BÀN XOAY tốc độ không đổi ====
# Bàn xoay 10 vòng/phút -> tốc độ góc omega = 10*360/60 = 60 deg/s.
TABLE_RPM = 10.0
TABLE_OMEGA_DEG_S = TABLE_RPM * 360.0 / 60.0  # = 60 deg/s

# TODO: đo/calib 2 giá trị này (tọa độ TÂM bàn xoay, cùng hệ mm với x_mm,
# y_mm mà detect_objects/detect_frame_holes trả về - tức đã quy chiếu theo
# cx, cy, fx, fy của camera). Cách đo đơn giản: đặt 1 vật cố định trên bàn,
# quay bàn, ghi lại (x_mm, y_mm) ở nhiều góc quay khác nhau rồi fit tâm
# đường tròn (least-squares circle fit) - báo mình nếu muốn code phần này.
#
# LƯU Ý QUAN TRỌNG: độ lớn của toạ độ dự đoán (predicted) phụ thuộc TRỰC
# TIẾP vào bán kính = khoảng cách từ (x_mm, y_mm) tới (TABLE_CENTER_X_MM,
# TABLE_CENTER_Y_MM). Nếu tâm bàn khai báo ở đây SAI (ví dụ để tạm 0,0
# trong khi tâm thật cách xa cỡ chục-trăm mm), bán kính tính ra sẽ sai lệch
# rất nhiều -> toạ độ dự đoán bị lệch hẳn (có thể quá nhỏ HOẶC quá lớn một
# cách vô nghĩa). Đây thường là nguyên nhân số 1 khiến kết quả "dự đoán quá
# nhỏ" - ưu tiên calib đúng tâm bàn trước khi chỉnh các hằng số khác.
TABLE_CENTER_X_MM = 0
TABLE_CENTER_Y_MM = 0.0

# Chiều quay của bàn nhìn từ camera. +1 = ngược chiều kim đồng hồ (CCW,
# góc toán học dương trong hệ x_mm/y_mm), -1 = cùng chiều kim đồng hồ (CW).
#
# ĐÃ XÁC NHẬN BẰNG MẮT qua marker hồng vẽ trực tiếp lên ảnh: để +1 thì điểm
# dự đoán bị lệch sang PHẢI màn hình (sai) - đảo lại -1 để điểm dự đoán
# lệch sang TRÁI màn hình như đúng chiều quay thực tế của bàn xoay.
TABLE_ROTATION_DIR = -1

# Độ trễ (giây) từ lúc camera PHÁT HIỆN vật tới lúc tay robot THỰC SỰ
# chạm/gắp vật. Trước đây để cứng 0.5s (đoán mò) khiến độ lệch dự đoán quá
# nhỏ so với thực tế. Giờ tính dựa trên CHÍNH các mốc thời gian di chuyển
# thật của robot (TIME_MOVE_FAST: chạy tới điểm A, TIME_MOVE_DOWN: hạ Z,
# TIME_MOVE_ACTION: tiến sâu gắp - xem move_delta_4dof.py) cộng thêm một
# khoảng đệm cho xử lý ảnh + truyền lệnh UART, nên số này VỪA lớn hơn VỪA
# đúng thực tế hơn con số cũ.
VISION_PROCESSING_DELAY_S = 0.15  # đệm cho xử lý ảnh + gửi lệnh UART
GRASP_DELAY_S = TIME_MOVE_FAST + TIME_MOVE_DOWN + TIME_MOVE_ACTION + VISION_PROCESSING_DELAY_S

# "Núm" khuếch đại thủ công: nếu sau khi calib tâm bàn + GRASP_DELAY_S vẫn
# thấy độ lệch dự đoán chưa đủ (do robot của bạn có thêm độ trễ cơ khí
# chưa tính hết), có thể tăng số này lên (vd 1.2, 1.5) để khuếch đại thêm
# mà KHÔNG cần sửa lại công thức xoay. Để 1.0 nếu không cần khuếch đại.
PREDICTION_LEAD_SCALE = 1.0


def predict_grasp_pose(x_mm, y_mm, angle_deg, delay_s=GRASP_DELAY_S,
                        table_cx=TABLE_CENTER_X_MM, table_cy=TABLE_CENTER_Y_MM,
                        omega_deg_s=TABLE_OMEGA_DEG_S, direction=TABLE_ROTATION_DIR,
                        lead_scale=PREDICTION_LEAD_SCALE):
    """Dự đoán (x_mm, y_mm, angle_deg) của vật/khung tại thời điểm robot
    THỰC SỰ gắp, dựa trên vị trí phát hiện hiện tại + tốc độ quay bàn cố
    định. Áp dụng được cho cả vật rời lẫn khung (khung không tự quay nếu
    đã cố định trên bàn, nhưng nếu khung cũng đặt trên bàn xoay thì gọi
    hàm này y hệt).

    Công thức là ma trận xoay 2D chuẩn áp dụng lên vector (dx, dy) =
    (điểm hiện tại) - (tâm bàn), rồi cộng lại tâm bàn để ra toạ độ tuyệt
    đối đã xoay:
        x' =  dx*cos(θ) - dy*sin(θ)
        y' =  dx*sin(θ) + dy*cos(θ)
    """
    delta_deg = direction * omega_deg_s * delay_s * lead_scale
    delta_rad = np.deg2rad(delta_deg)

    dx = x_mm - table_cx
    dy = y_mm - table_cy

    cos_d, sin_d = np.cos(delta_rad), np.sin(delta_rad)

    x_rel = dx * cos_d - dy * sin_d
    y_rel = dx * sin_d + dy * cos_d

    x_pred = table_cx + x_rel
    y_pred = table_cy + y_rel
    angle_pred = (angle_deg + delta_deg) % 360.0

    return x_pred, y_pred, angle_pred


def _mm_to_px(x_mm, y_mm, cx, cy, fx, fy, calib_key):
    """Chiều NGƯỢC LẠI của _px_to_mm_position trong Camera_4dof.py - dùng để
    vẽ điểm dự đoán (chỉ tính bằng mm) trở lại lên ảnh pixel.

    Công thức gốc (Camera_4dof.py):
        dx_px = cx - px            ;  x_mm = dx_px * scale
        dy_px = cy - py            ;  y_mm = dy_px * scale * (fx/fy)
    => đảo ngược:
        dx_px = x_mm / scale       ;  px = cx - dx_px
        dy_px = y_mm * fy / (fx*scale) ; py = cy - dy_px
    """
    scale = _CIRCLE_CALIB[calib_key]["scale"]
    dx_px = x_mm / scale
    dy_px = (y_mm * fy) / (fx * scale) if fx else y_mm / scale
    px = cx - dx_px
    py = cy - dy_px
    return px, py


def _draw_predicted_marker(display, cur_px, pred_px, x_mm_grasp, y_mm_grasp, angle_deg_grasp, tag):
    """Vẽ điểm DỰ ĐOÁN (vị trí lúc robot thực sự gắp) lên ảnh, kèm mũi tên
    nối từ vị trí hiện tại -> vị trí dự đoán để thấy rõ HƯỚNG dự đoán đang
    lệch về phía nào trên màn hình - dùng để kiểm chứng/chỉnh TABLE_ROTATION_DIR
    bằng mắt khi camera đang chạy thực tế."""
    px_pred, py_pred = int(round(pred_px[0])), int(round(pred_px[1]))
    px_cur, py_cur = int(round(cur_px[0])), int(round(cur_px[1]))

    cv2.arrowedLine(display, (px_cur, py_cur), (px_pred, py_pred), PREDICT_COLOR, 2, tipLength=0.25)
    cv2.drawMarker(display, (px_pred, py_pred), PREDICT_COLOR, cv2.MARKER_TILTED_CROSS, 16, 2)
    label = f"Du doan {tag} ({x_mm_grasp:.1f},{y_mm_grasp:.1f}) {angle_deg_grasp:.0f}deg"
    cv2.putText(display, label, (px_pred - 70, py_pred + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, PREDICT_COLOR, 1)


def _compute_two_rois(frame_w, frame_h, shift_left_px=0, shift_down_px=0):
    """Tính một ô vuông ROI duy nhất đặt ở phía trên, giữa màn hình, có thể
    dịch sang trái/phải một khoảng shift_left_px (pixel dương = dịch sang
    trái, âm = dịch sang phải) và dịch xuống/lên một khoảng shift_down_px
    (pixel dương = dịch xuống, âm = dịch lên). Trả về cùng một ROI cho cả
    vật và khung, để hợp nhất hai vùng thành một."""
    roi_size = int(min(frame_w, frame_h) * ROI_SIZE_RATIO)
    roi_x = (frame_w - roi_size) // 2 - shift_left_px
    roi_x = max(0, min(roi_x, frame_w - roi_size))  # tránh ROI tràn ra ngoài khung hình
    roi_y = 0 + shift_down_px  # mặc định sát mép trên, dịch xuống theo shift_down_px
    roi_y = max(0, min(roi_y, frame_h - roi_size))  # tránh ROI tràn ra ngoài khung hình
    roi = (roi_x, roi_y, roi_size, roi_size)
    return roi, roi


def _mask_restricted_to_roi(mask_small, roi_full_px, det_scale):
    x0, y0, w, h = roi_full_px
    x0s = max(0, int(round(x0 * det_scale)))
    y0s = max(0, int(round(y0 * det_scale)))
    x1s = min(mask_small.shape[1], int(round((x0 + w) * det_scale)))
    y1s = min(mask_small.shape[0], int(round((y0 + h) * det_scale)))

    restricted = np.zeros_like(mask_small)
    if x1s > x0s and y1s > y0s:
        restricted[y0s:y1s, x0s:x1s] = mask_small[y0s:y1s, x0s:x1s]
    return restricted


def _draw_roi_box(display, roi, color, label):
    x0, y0, w, h = roi
    cv2.rectangle(display, (x0, y0), (x0 + w, y0 + h), color, 2)
    cv2.putText(display, label, (x0, max(0, y0 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def run_camera_run4dof_stream(camera_index, calib_path, is_running, brightness=80.0):
    data = np.load(calib_path)
    camera_matrix, dist_coeffs = data["camera_matrix"], data["dist_coeffs"]

    cap = _ThreadedCapture(camera_index, target_fps=30.0, brightness=brightness)

    fps = 0.0
    frame_count = 0
    start_time = cv2.getTickCount()

    try:
        while is_running():
            ret, frame = cap.read()
            if not ret or frame is None:
                # Chưa có frame đầu tiên hoặc camera tạm mất tín hiệu.
                time.sleep(0.01)
                continue

            # ---- Undistort: LẤY cx, cy, fx, fy từ new_camera_matrix, dùng để
            #      quy đổi px -> mm - giống hệt Camera_4dof.py ----
            undistorted, new_cm = undistort_image(frame, camera_matrix, dist_coeffs)
            fx, fy = new_cm[0, 0], new_cm[1, 1]
            cx, cy = new_cm[0, 2], new_cm[1, 2]

            display = undistorted.copy()
            origin_px = (int(round(cx)), int(round(cy)))
            cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            # ---- 2 ô vuông ROI riêng: 1 để dò VẬT, 1 để dò KHUNG ----
            # (dùng chung 1 vùng, dịch sang trái ROI_SHIFT_LEFT_MM và xuống
            # dưới ROI_SHIFT_DOWN_MM để bù lệch thực tế của vùng nhận diện
            # so với khung hình camera)
            h_img, w_img = undistorted.shape[:2]
            # Quy đổi mm -> pixel dựa trên scale mm<->px hiện có
            # (dx_px = x_mm / scale  =>  pixel/mm = 1/scale).
            _roi_scale = _CIRCLE_CALIB["white"]["scale"]
            shift_left_px = int(round(ROI_SHIFT_LEFT_MM / _roi_scale)) if _roi_scale else 0
            shift_down_px = int(round(ROI_SHIFT_DOWN_MM / _roi_scale)) if _roi_scale else 0
            roi_object, roi_frame = _compute_two_rois(w_img, h_img, shift_left_px, shift_down_px)
            _draw_roi_box(display, roi_object, ROI_OBJECT_COLOR, "Vung nhan dien VAT")
            _draw_roi_box(display, roi_frame, ROI_FRAME_COLOR, "Vung nhan dien KHUNG")

            # ---- Dò mask đỏ trên ảnh thu nhỏ (DET_SCALE) rồi upscale contour
            #      về toạ độ ảnh gốc - đúng cách Camera_4dof.py đang làm ----
            small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
            small_min_area = max(20, int(round(80 * DET_SCALE * DET_SCALE)))
            red_mask = _build_red_mask(small, min_component_area=small_min_area)
            upscale = 1.0 / DET_SCALE

            # ---- Tách mask riêng cho từng ROI: mask ngoài ô vuông tương ứng
            #      bị tô đen -> detect_* chỉ còn thấy phần bên trong ô ----
            mask_object = _mask_restricted_to_roi(red_mask, roi_object, DET_SCALE)
            mask_frame = _mask_restricted_to_roi(red_mask, roi_frame, DET_SCALE)

            # ---- Khung 8 lỗ (frame holes): CHỈ dò trong roi_frame ----
            frame_result = detect_frame_holes(undistorted, cx, cy, fx, fy, mask=mask_frame, upscale=upscale)
            holes = frame_result["holes"] if frame_result["frame_found"] else []

            # ---- Vật thể rời (object): CHỈ dò trong roi_object (mask đã bị
            #      giới hạn + truyền thêm roi= để lọc lần nữa cho chắc) ----
            objects = detect_objects(
                undistorted, cx, cy, fx, fy,
                mask=mask_object, upscale=upscale, refine_frame=undistorted,
                roi=roi_object,
            )
            if holes:
                region = mold_bounding_region(holes, margin_mm=15.0)
                objects = [o for o in objects if not is_inside_region(o["x_mm"], o["y_mm"], region)]

            # ---- Vẽ annotation lên ảnh hiển thị ----
            for o in objects:
                px, py = int(round(o["cx_px"])), int(round(o["cy_px"]))
                cv2.circle(display, (px, py), 5, (0, 0, 255), -1)
                label = f"Vat ({o['x_mm']:.1f},{o['y_mm']:.1f}) {o['angle_deg']:.0f}deg"
                cv2.putText(display, label, (px - 65, py - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

            if not holes:
                cv2.putText(
                    display,
                    "Dang quet khung... (khong tim thay hoac chua co khung)",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2,
                )
            else:
                for h in holes:
                    px, py = int(round(h["cx_px"])), int(round(h["cy_px"]))
                    cv2.circle(display, (px, py), 5, (0, 255, 0), -1)
                    cv2.putText(display, f"Lo#{h['slot_id']}", (px - 28, py - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                anchor_x = sum(h["x_mm"] for h in holes) / len(holes)
                anchor_y = sum(h["y_mm"] for h in holes) / len(holes)
                anchor_angle = sum(h["angle_deg"] for h in holes) / len(holes)
                cv2.putText(
                    display,
                    f"Khung: {len(holes)} lo trong, X={anchor_x:.1f} Y={anchor_y:.1f} Goc={anchor_angle:.1f}deg",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2,
                )

            # ---- Gom items trả về cho GUI + VẼ điểm dự đoán lên ảnh ----
            # (TRƯỚC ĐÂY chỉ tính x_mm_grasp/y_mm_grasp rồi bỏ vào items,
            # KHÔNG hề vẽ lên display -> không thấy được gì trên màn hình.
            # Giờ vẽ hẳn ra bằng mũi tên + marker màu hồng để có thể xác
            # nhận trực tiếp bằng mắt khi camera đang chạy thực tế.)
            items = []
            for o in objects:
                item = {**o, "diameter_mm": None, "slot_id": None}
                x_pred, y_pred, angle_pred = predict_grasp_pose(
                    o["x_mm"], o["y_mm"], o["angle_deg"])
                item["x_mm_grasp"] = x_pred
                item["y_mm_grasp"] = y_pred
                item["angle_deg_grasp"] = angle_pred

                px_pred, py_pred = _mm_to_px(x_pred, y_pred, cx, cy, fx, fy, calib_key="white")
                item["cx_px_grasp"] = px_pred
                item["cy_px_grasp"] = py_pred
                _draw_predicted_marker(
                    display, (o["cx_px"], o["cy_px"]), (px_pred, py_pred),
                    x_pred, y_pred, angle_pred, tag="VAT",
                )
                items.append(item)

            for h in holes:
                item = {**h, "diameter_mm": None}
                x_pred, y_pred, angle_pred = predict_grasp_pose(
                    h["x_mm"], h["y_mm"], h["angle_deg"])
                item["x_mm_grasp"] = x_pred
                item["y_mm_grasp"] = y_pred
                item["angle_deg_grasp"] = angle_pred

                px_pred, py_pred = _mm_to_px(x_pred, y_pred, cx, cy, fx, fy, calib_key="black")
                item["cx_px_grasp"] = px_pred
                item["cy_px_grasp"] = py_pred
                _draw_predicted_marker(
                    display, (h["cx_px"], h["cy_px"]), (px_pred, py_pred),
                    x_pred, y_pred, angle_pred, tag="LO",
                )
                items.append(item)

            frame_count += 1
            if frame_count >= 10:
                end_time = cv2.getTickCount()
                seconds = (end_time - start_time) / cv2.getTickFrequency()
                fps = frame_count / seconds if seconds > 0 else 0.0
                frame_count = 0
                start_time = cv2.getTickCount()

            yield display, items, fps
    finally:
        cap.release()