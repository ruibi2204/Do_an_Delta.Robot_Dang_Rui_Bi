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

try:
    from kinematics.move_delta_4dof import (
        TIME_MOVE_FAST, TIME_MOVE_DOWN, TIME_MOVE_ACTION,
    )
except Exception:
    TIME_MOVE_FAST, TIME_MOVE_DOWN, TIME_MOVE_ACTION = 0.6, 0.15, 0.15

DET_SCALE = 0.5
ROI_SIZE_RATIO = (1.0 / 3.0)
ROI_SHIFT_LEFT_MM = 0.0
ROI_SHIFT_DOWN_MM = 0.0
ROI_OBJECT_COLOR = (255, 0, 0)
ROI_FRAME_COLOR = (0, 165, 255)
PREDICT_COLOR = (255, 0, 255)

# Bù lệch thủ công cho tọa độ Y của VẬT (mm). CHỈ áp dụng cho danh sách
# `objects` (vật rời) - KHÔNG áp dụng cho `holes` (khung), khung phải giữ
# nguyên tọa độ gốc do camera đo được. Được cộng vào o["y_mm"] ngay sau
# khi detect + lọc vật nằm trong vùng khung, nên mọi thứ tính từ đây trở
# đi (nhãn hiển thị, dự đoán vị trí gắp predict_grasp_pose, tọa độ gửi
# cho robot qua x_mm_grasp/y_mm_grasp) đều phản ánh đúng offset này.
OBJECT_Y_OFFSET_MM = 3.0


class _ThreadedCapture:
    """Đọc frame từ camera trong 1 thread riêng, luôn giữ frame MỚI NHẤT."""

    def __init__(self, camera_index, target_fps=30.0, brightness=80.0):
        backend = cv2.CAP_DSHOW if os.name == "nt" else 0
        self.cap = cv2.VideoCapture(camera_index, backend) if os.name == "nt" else cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Không thể mở camera index {camera_index}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FPS, target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if brightness != 0.0:
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)

        self._lock = threading.Lock()
        self._frame = None
        self._ret = False
        self._stopped = False
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
                time.sleep(0.05)

    def read(self, timeout=1.0):
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
_table_rpm_lock = threading.Lock()
_current_table_rpm = 0.0  # 0 = coi như bàn đang đứng yên cho tới khi GUI báo tốc độ thật


def set_table_rpm(rpm: float):
    """GUI gọi hàm này mỗi khi bật bàn xoay (kèm RPM đang chỉnh), đổi RPM
    lúc đang bật, hoặc tắt bàn xoay (gọi với rpm=0.0)."""
    global _current_table_rpm
    with _table_rpm_lock:
        _current_table_rpm = float(rpm)


def get_table_rpm() -> float:
    with _table_rpm_lock:
        return _current_table_rpm


TABLE_CENTER_X_MM = 0
TABLE_CENTER_Y_MM = 0.0

TABLE_ROTATION_DIR = -1

VISION_PROCESSING_DELAY_S = 0.15
GRASP_DELAY_S = TIME_MOVE_FAST + TIME_MOVE_DOWN + TIME_MOVE_ACTION + VISION_PROCESSING_DELAY_S

PREDICTION_LEAD_SCALE = 1.0


def predict_grasp_pose(x_mm, y_mm, angle_deg, delay_s=GRASP_DELAY_S,
                        table_cx=TABLE_CENTER_X_MM, table_cy=TABLE_CENTER_Y_MM,
                        omega_deg_s=None, direction=TABLE_ROTATION_DIR,
                        lead_scale=PREDICTION_LEAD_SCALE):
    """Dự đoán (x_mm, y_mm, angle_deg) tại thời điểm robot THỰC SỰ gắp.

    omega_deg_s=None (mặc định): tự lấy tốc độ góc từ RPM THỰC TẾ đang
    chạy (do GUI cập nhật qua set_table_rpm)."""
    if omega_deg_s is None:
        omega_deg_s = get_table_rpm() * 360.0 / 60.0

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
    scale = _CIRCLE_CALIB[calib_key]["scale"]
    dx_px = x_mm / scale
    dy_px = (y_mm * fy) / (fx * scale) if fx else y_mm / scale
    px = cx - dx_px
    py = cy - dy_px
    return px, py


def _draw_predicted_marker(display, cur_px, pred_px, x_mm_grasp, y_mm_grasp, angle_deg_grasp, tag):
    px_pred, py_pred = int(round(pred_px[0])), int(round(pred_px[1]))
    px_cur, py_cur = int(round(cur_px[0])), int(round(cur_px[1]))

    cv2.arrowedLine(display, (px_cur, py_cur), (px_pred, py_pred), PREDICT_COLOR, 2, tipLength=0.25)
    cv2.drawMarker(display, (px_pred, py_pred), PREDICT_COLOR, cv2.MARKER_TILTED_CROSS, 16, 2)
    label = f"Du doan {tag} ({x_mm_grasp:.1f},{y_mm_grasp:.1f}) {angle_deg_grasp:.0f}deg"
    cv2.putText(display, label, (px_pred - 70, py_pred + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, PREDICT_COLOR, 1)


def _compute_two_rois(frame_w, frame_h, shift_left_px=0, shift_down_px=0):
    roi_size = int(min(frame_w, frame_h) * ROI_SIZE_RATIO)
    roi_x = (frame_w - roi_size) // 2 - shift_left_px
    roi_x = max(0, min(roi_x, frame_w - roi_size))
    roi_y = 0 + shift_down_px
    roi_y = max(0, min(roi_y, frame_h - roi_size))
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
                time.sleep(0.01)
                continue

            undistorted, new_cm = undistort_image(frame, camera_matrix, dist_coeffs)
            fx, fy = new_cm[0, 0], new_cm[1, 1]
            cx, cy = new_cm[0, 2], new_cm[1, 2]

            display = undistorted.copy()
            origin_px = (int(round(cx)), int(round(cy)))
            cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            h_img, w_img = undistorted.shape[:2]
            _roi_scale = _CIRCLE_CALIB["white"]["scale"]
            shift_left_px = int(round(ROI_SHIFT_LEFT_MM / _roi_scale)) if _roi_scale else 0
            shift_down_px = int(round(ROI_SHIFT_DOWN_MM / _roi_scale)) if _roi_scale else 0
            roi_object, roi_frame = _compute_two_rois(w_img, h_img, shift_left_px, shift_down_px)
            _draw_roi_box(display, roi_object, ROI_OBJECT_COLOR, "Vung nhan dien VAT")
            _draw_roi_box(display, roi_frame, ROI_FRAME_COLOR, "Vung nhan dien KHUNG")

            small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
            small_min_area = max(20, int(round(80 * DET_SCALE * DET_SCALE)))
            red_mask = _build_red_mask(small, min_component_area=small_min_area)
            upscale = 1.0 / DET_SCALE

            mask_object = _mask_restricted_to_roi(red_mask, roi_object, DET_SCALE)
            mask_frame = _mask_restricted_to_roi(red_mask, roi_frame, DET_SCALE)

            # ---- Khung 8 lỗ (frame holes): CHỈ dò trong roi_frame. KHÔNG
            #      áp dụng OBJECT_Y_OFFSET_MM ở đây - khung giữ nguyên
            #      tọa độ gốc do camera đo được. ----
            frame_result = detect_frame_holes(undistorted, cx, cy, fx, fy, mask=mask_frame, upscale=upscale)
            holes = frame_result["holes"] if frame_result["frame_found"] else []

            # ---- Vật thể rời (object): CHỈ dò trong roi_object ----
            objects = detect_objects(
                undistorted, cx, cy, fx, fy,
                mask=mask_object, upscale=upscale, refine_frame=undistorted,
                roi=roi_object,
            )
            if holes:
                region = mold_bounding_region(holes, margin_mm=15.0)
                objects = [o for o in objects if not is_inside_region(o["x_mm"], o["y_mm"], region)]

            # ---- BÙ LỆCH +2mm CHO TỌA ĐỘ Y CỦA VẬT (chỉ objects, không
            #      đụng tới holes) - áp dụng NGAY SAU khi detect + lọc,
            #      TRƯỚC khi vẽ nhãn và tính dự đoán gắp, để mọi bước phía
            #      sau (nhãn hiển thị, predict_grasp_pose, x_mm_grasp/
            #      y_mm_grasp gửi cho robot) đều dùng giá trị Y đã bù. ----
            for o in objects:
                o["y_mm"] = o["y_mm"] + OBJECT_Y_OFFSET_MM

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

            # ---- Khung (holes): giữ nguyên hoàn toàn, KHÔNG bù offset ----
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