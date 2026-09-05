import argparse
import os
import sys
import math
import cv2
import numpy as np

RECT_WIDTH_MM = 10.0
RECT_HEIGHT_MM = 20.0
RECT_RATIO_EXPECTED = RECT_WIDTH_MM / RECT_HEIGHT_MM
RECT_RATIO_TOLERANCE = 0.15

FRAME_HOLE_WIDTH_MM = 10
FRAME_HOLE_HEIGHT_MM = 20
FRAME_HOLE_RATIO_EXPECTED = FRAME_HOLE_WIDTH_MM / FRAME_HOLE_HEIGHT_MM
FRAME_HOLE_RATIO_TOLERANCE = 0.15
FRAME_HOLE_COUNT_EXPECTED = 8

# Bảng hiệu chỉnh mm/px cố định theo màu (thay cho việc tự hiệu chỉnh động
# mỗi khung hình bằng cách so kích thước đo được với kích thước thật).
#   scale     : mm trên mỗi px (dùng để quy đổi cả toạ độ và kích thước)
#   intercept : hằng số bù (mm) cộng thêm khi quy đổi KÍCH THƯỚC, để sửa
#               sai lệch hệ thống do biên mask co/giãn theo màu vật.
# Quy ước dùng trong file này (vật và khung dùng 2 màu tham chiếu khác nhau):
#   'white' -> vật thể (hình chữ nhật đỏ)
#   'black' -> lỗ trên khung
_CIRCLE_CALIB = {
    'white': {'scale': 0.3118, 'intercept': 1.0},
    'black': {'scale': 0.3125, 'intercept': 1.0},
}

DET_SCALE = 0.5

# Do sang camera (cv2.CAP_PROP_BRIGHTNESS). Khoang gia tri phu thuoc
# tung camera/driver (thuong 0-255, co camera dung -1.0..1.0), nen coi
# day la gia tri khoi tao, roi chinh truc tiep bang phim '+'/'-' khi
# dang chay run_live_display de tim gia tri phu hop voi camera cua ban.
CAMERA_BRIGHTNESS_DEFAULT = 50
CAMERA_BRIGHTNESS_STEP = 10


def _wrap_angle_180(angle_deg):
    a = angle_deg % 180.0
    if a >= 90.0:
        a -= 180.0
    return a


def _long_edge_angle_deg(box_points):
    best_len = -1.0
    best_angle = 0.0
    n = len(box_points)
    for i in range(n):
        p1 = box_points[i]
        p2 = box_points[(i + 1) % n]
        dx = float(p2[0] - p1[0])
        dy = float(p2[1] - p1[1])
        length = math.hypot(dx, dy)
        if length > best_len:
            best_len = length
            best_angle = math.degrees(math.atan2(dy, dx))
    return _wrap_angle_180(best_angle), best_len


def _calib_mm_from_px(px_value, calib_key):
    """Quy đổi 1 kích thước theo px sang mm bằng bảng _CIRCLE_CALIB (scale*px + intercept)."""
    calib = _CIRCLE_CALIB[calib_key]
    return px_value * calib['scale'] + calib['intercept']


def _px_to_mm_position(item_px, scale, fx=1.0, fy=1.0, cx=0.0, cy=0.0):
    dx_px = -item_px['cx'] + cx
    dy_px = cy - item_px['cy']
    x_mm = dx_px * scale
    y_mm = dy_px * scale * (fx / fy if fy else 1.0)
    angle_robot = _wrap_angle_180(item_px['angle_deg'])
    return x_mm, y_mm, angle_robot


_undistort_cache = {}


def build_undistort_maps(camera_matrix, dist_coeffs, image_size):
    w, h = image_size
    new_camera_matrix, _roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1)
    return map1, map2, new_camera_matrix


def undistort_image(img, camera_matrix, dist_coeffs):
    h, w = img.shape[:2]
    key = (camera_matrix.tobytes(), dist_coeffs.tobytes(), w, h)
    cached = _undistort_cache.get(key)
    if cached is None:
        map1, map2, new_camera_matrix = build_undistort_maps(camera_matrix, dist_coeffs, (w, h))
        _undistort_cache[key] = (map1, map2, new_camera_matrix)
        if len(_undistort_cache) > 4:
            _undistort_cache.pop(next(iter(_undistort_cache)))
    else:
        map1, map2, new_camera_matrix = cached
    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted, new_camera_matrix


def _remove_small_components(mask, min_area=150):
    num_labels, labels, stats, _c = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    cleaned = np.zeros_like(mask)
    for lbl in range(1, num_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == lbl] = 255
    return cleaned


def _build_red_mask(frame, min_component_area=150):
    blurred = cv2.GaussianBlur(frame, (5, 5), 1.5)
    b, g, r = cv2.split(blurred.astype(np.int16))
    red_condition = (r - g > 35) & (r - b > 35) & (r > 80)
    red_mask = np.uint8(red_condition * 255)
    kernel_close = np.ones((7, 7), np.uint8)
    kernel_open = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)
    red_mask = _remove_small_components(red_mask, min_area=min_component_area)
    return red_mask


def _build_red_mask_precise(frame):
    b, g, r = cv2.split(frame.astype(np.int16))
    red_condition = (r - g > 35) & (r - b > 35) & (r > 80)
    red_mask = np.uint8(red_condition * 255)
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    return red_mask


def _scale_contours(contours, upscale):
    if upscale == 1.0:
        return contours
    return [c.astype(np.float32) * upscale for c in contours]


def _refine_object_contour_in_roi(full_frame, cx_approx, cy_approx, w_approx, h_approx, pad_px=18):
    h_img, w_img = full_frame.shape[:2]
    half_w = max(w_approx, h_approx) / 2.0 + pad_px
    half_h = half_w
    x0 = max(0, int(cx_approx - half_w))
    y0 = max(0, int(cy_approx - half_h))
    x1 = min(w_img, int(cx_approx + half_w))
    y1 = min(h_img, int(cy_approx + half_h))
    if x1 - x0 < 5 or y1 - y0 < 5:
        return None
    crop = full_frame[y0:y1, x0:x1]
    crop_mask = _build_red_mask_precise(crop)
    contours, _hier = cv2.findContours(crop_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = max(contours, key=cv2.contourArea)
    if cv2.contourArea(best) < 20:
        return None
    best = best.astype(np.float32)
    best[:, 0, 0] += x0
    best[:, 0, 1] += y0
    return best


def _detect_red_rectangles(frame, mask=None, upscale=1.0, refine_frame=None, refine_pad=18):
    red_mask = mask if mask is not None else _build_red_mask(frame)
    contours, _hier = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = _scale_contours(contours, upscale)
    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 150:
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), _raw_angle = rect
        if refine_frame is not None:
            refined = _refine_object_contour_in_roi(refine_frame, cx, cy, rw, rh, pad_px=refine_pad)
            if refined is not None:
                cnt = refined
                area = cv2.contourArea(cnt)
                if area < 150:
                    continue
                rect = cv2.minAreaRect(cnt)
                (cx, cy), (rw, rh), _raw_angle = rect
        box = cv2.boxPoints(rect)
        long_side_px = max(rw, rh)
        short_side_px = min(rw, rh)
        if short_side_px <= 0:
            continue
        box_area = rw * rh
        if box_area <= 0:
            continue
        extent = area / box_area
        if extent < 0.80:
            continue
        ratio = short_side_px / long_side_px
        if abs(ratio - RECT_RATIO_EXPECTED) > RECT_RATIO_TOLERANCE:
            continue
        angle_deg, _ = _long_edge_angle_deg(box)
        results.append({
            'cx': float(cx), 'cy': float(cy),
            'w_px': float(long_side_px), 'h_px': float(short_side_px),
            'angle_deg': float(angle_deg), 'area_px': float(area),
        })
    return results


def _filter_objects_by_physical_size(raw_rects):
    """Gán kích thước mm cho từng vật bằng bảng hiệu chỉnh cố định _CIRCLE_CALIB['white'].
    Việc lọc hợp lệ (tỉ lệ cạnh, độ đặc contour) đã thực hiện ở _detect_red_rectangles,
    nên ở đây không cần dò lại kích thước thật để loại nhiễu như trước nữa."""
    kept = []
    scale = _CIRCLE_CALIB['white']['scale']
    for r in raw_rects:
        r = dict(r)
        r['w_mm'] = _calib_mm_from_px(r['w_px'], 'white')
        r['h_mm'] = _calib_mm_from_px(r['h_px'], 'white')
        r['scale_mm_per_px'] = scale
        kept.append(r)
    return kept


def detect_objects(frame, cx, cy, fx=1.0, fy=1.0, mask=None, upscale=1.0, refine_frame=None, roi=None):
    """
    Phát hiện vật thể (hình chữ nhật đỏ) và trả về danh sách các vật.
    Nếu roi (x, y, w, h) được chỉ định, chỉ những vật có tâm nằm trong ROI mới được giữ lại.
    """
    raw = _detect_red_rectangles(frame, mask=mask, upscale=upscale, refine_frame=refine_frame)
    raw = _filter_objects_by_physical_size(raw)
    objects = []
    for o in raw:
        scale = o['scale_mm_per_px']
        x_mm, y_mm, angle_robot = _px_to_mm_position(o, scale, fx, fy, cx, cy)
        # Lọc theo ROI nếu có
        if roi is not None:
            x0, y0, w, h = roi
            if not (x0 <= o['cx'] <= x0 + w and y0 <= o['cy'] <= y0 + h):
                continue
        objects.append({
            'type': 'object',
            'x_mm': round(x_mm, 2),
            'y_mm': round(y_mm, 2),
            'angle_deg': round(angle_robot, 2),
            'width_mm': round(o['w_mm'], 2),
            'height_mm': round(o['h_mm'], 2),
            'cx_px': o['cx'],
            'cy_px': o['cy'],
        })
    return objects


def _detect_frame_holes_raw(frame, mask=None, upscale=1.0):
    red_mask = mask if mask is not None else _build_red_mask(frame)
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None or len(contours) == 0:
        return None, []
    contours = _scale_contours(contours, upscale)
    hierarchy = hierarchy[0]

    frame_idx, frame_area = -1, -1.0
    for i, cnt in enumerate(contours):
        if hierarchy[i][3] != -1:
            continue
        area = cv2.contourArea(cnt)
        if area > frame_area:
            frame_area, frame_idx = area, i
    if frame_idx == -1 or frame_area < 500:
        return None, []

    frame_rect = cv2.minAreaRect(contours[frame_idx])

    holes = []
    for i, cnt in enumerate(contours):
        if hierarchy[i][3] != frame_idx:
            continue
        area = cv2.contourArea(cnt)
        if area < 80:
            continue
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), _raw_angle = rect
        if min(rw, rh) <= 0:
            continue
        box_area = rw * rh
        if box_area <= 0:
            continue
        extent = area / box_area
        if extent < 0.75:
            continue
        long_side_px = max(rw, rh)
        short_side_px = min(rw, rh)
        ratio = short_side_px / long_side_px
        if abs(ratio - FRAME_HOLE_RATIO_EXPECTED) > FRAME_HOLE_RATIO_TOLERANCE:
            continue
        box = cv2.boxPoints(rect)
        angle_deg, _ = _long_edge_angle_deg(box)
        holes.append({
            'cx': float(cx), 'cy': float(cy),
            'w_px': float(long_side_px), 'h_px': float(short_side_px),
            'angle_deg': float(angle_deg), 'area_px': float(area),
        })
    return frame_rect, holes


def _filter_frame_holes_by_physical_size(raw_holes):
    """Gán kích thước mm cho từng lỗ bằng bảng hiệu chỉnh cố định _CIRCLE_CALIB['black'].
    Trả về (kept, frame_scale) như cũ để detect_frame_holes dùng trực tiếp."""
    calib = _CIRCLE_CALIB['black']
    kept = []
    for h in raw_holes:
        hh = dict(h)
        hh['scale_mm_per_px'] = calib['scale']
        hh['w_mm'] = _calib_mm_from_px(h['w_px'], 'black')
        hh['h_mm'] = _calib_mm_from_px(h['h_px'], 'black')
        kept.append(hh)
    frame_scale = calib['scale']
    return kept, frame_scale


def _sort_holes_grid_order(holes_mm, row_tolerance_mm=8.0):
    if not holes_mm:
        return []
    ordered = sorted(holes_mm, key=lambda h: h['y_mm'])
    rows = []
    current_row = [ordered[0]]
    for h in ordered[1:]:
        if abs(h['y_mm'] - current_row[-1]['y_mm']) <= row_tolerance_mm:
            current_row.append(h)
        else:
            rows.append(current_row)
            current_row = [h]
    rows.append(current_row)
    result = []
    for row in rows:
        result.extend(sorted(row, key=lambda h: h['x_mm']))
    return result


def detect_frame_holes(frame, cx, cy, fx=1.0, fy=1.0, mask=None, upscale=1.0):
    frame_rect, raw_holes = _detect_frame_holes_raw(frame, mask=mask, upscale=upscale)
    if frame_rect is None:
        return {
            'frame_found': False, 'holes': [], 'empty_count': 0,
            'expected_count': FRAME_HOLE_COUNT_EXPECTED, 'occupied_estimate': None,
        }

    kept, frame_scale = _filter_frame_holes_by_physical_size(raw_holes)
    scale_to_use = frame_scale

    holes_mm = []
    for h in kept:
        x_mm, y_mm, angle_robot = _px_to_mm_position(h, scale_to_use, fx, fy, cx, cy)
        holes_mm.append({
            'type': 'hole',
            'x_mm': round(x_mm, 2),
            'y_mm': round(y_mm, 2),
            'angle_deg': round(angle_robot, 2),
            'width_mm': round(h['w_mm'], 2),
            'height_mm': round(h['h_mm'], 2),
            'cx_px': h['cx'],
            'cy_px': h['cy'],
        })

    holes_sorted = _sort_holes_grid_order(holes_mm)
    for idx, h in enumerate(holes_sorted):
        h['slot_id'] = idx + 1

    empty_count = len(holes_sorted)
    occupied_estimate = max(0, FRAME_HOLE_COUNT_EXPECTED - empty_count)

    return {
        'frame_found': True,
        'holes': holes_sorted,
        'empty_count': empty_count,
        'expected_count': FRAME_HOLE_COUNT_EXPECTED,
        'occupied_estimate': occupied_estimate,
    }


def mold_bounding_region(holes, margin_mm=15.0):
    if not holes:
        return None
    xs = [h['x_mm'] for h in holes]
    ys = [h['y_mm'] for h in holes]
    return {'x_min': min(xs) - margin_mm, 'x_max': max(xs) + margin_mm,
            'y_min': min(ys) - margin_mm, 'y_max': max(ys) + margin_mm}


def is_inside_region(x_mm, y_mm, region):
    if region is None:
        return False
    return region['x_min'] <= x_mm <= region['x_max'] and region['y_min'] <= y_mm <= region['y_max']


def required_dof4_rotation(object_angle_deg, target_angle_deg=90.0):
    return _wrap_angle_180(target_angle_deg - object_angle_deg)


def run_live_display(camera_index, camera_matrix, dist_coeffs, brightness=CAMERA_BRIGHTNESS_DEFAULT):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Khong the mo camera index {camera_index}")
        return
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    current_brightness = brightness
    cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
    print("Dang chay Camera_4dof. 'q' de thoat. '+'/'-' de chinh do sang camera.")

    fps = 0.0
    frame_count = 0
    start_time = cv2.getTickCount()

    # Biến lưu ROI (ô vuông cố định) - sẽ được tính lại mỗi frame nếu kích thước thay đổi
    roi = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted, new_cm = undistort_image(frame, camera_matrix, dist_coeffs)
        fx, fy = new_cm[0, 0], new_cm[1, 1]
        cx, cy = new_cm[0, 2], new_cm[1, 2]

        small = cv2.resize(undistorted, None, fx=DET_SCALE, fy=DET_SCALE, interpolation=cv2.INTER_AREA)
        small_min_area = max(20, int(round(150 * DET_SCALE * DET_SCALE)))
        red_mask = _build_red_mask(small, min_component_area=small_min_area)
        upscale = 1.0 / DET_SCALE

        # --- Xác định ROI (ô vuông cố định) ---
        h, w = undistorted.shape[:2]
        roi_size = min(w, h) // 3          # kích thước ô vuông = 1/3 chiều nhỏ nhất
        roi = (w//2 - roi_size//2, h//2 - roi_size//2, roi_size, roi_size)
        # ------------------------------------

        frame_result = detect_frame_holes(undistorted, cx, cy, fx, fy, mask=red_mask, upscale=upscale)
        # Truyền roi vào detect_objects
        objects = detect_objects(
            undistorted, cx, cy, fx, fy,
            mask=red_mask,
            upscale=upscale,
            refine_frame=undistorted,
            roi=roi
        )

        display = undistorted.copy()
        origin_px = (int(round(cx)), int(round(cy)))
        cv2.drawMarker(display, origin_px, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # Vẽ ô vuông ROI lên ảnh hiển thị
        cv2.rectangle(display, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (255, 0, 0), 2)
        cv2.putText(display, "Vung tim vat", (roi[0], roi[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

        for o in objects:
            center_px = (int(o['cx_px']), int(o['cy_px']))
            rot_needed = required_dof4_rotation(o['angle_deg'], target_angle_deg=90.0)
            cv2.circle(display, center_px, 3, (0, 255, 0), -1)
            txt1 = f"X:{o['x_mm']:.1f} Y:{o['y_mm']:.1f} mm  Goc:{o['angle_deg']:.1f}deg"
            txt2 = f"{o['width_mm']:.1f}x{o['height_mm']:.1f}mm  Xoay_can:{rot_needed:.1f}deg"
            cv2.putText(display, txt1, (center_px[0] - 90, center_px[1] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(display, txt2, (center_px[0] - 90, center_px[1] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        if not frame_result['frame_found']:
            cv2.putText(display, "Khong thay khung", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            for h in frame_result['holes']:
                center_px = (int(round(h['cx_px'])), int(round(h['cy_px'])))
                cv2.circle(display, center_px, 4, (0, 255, 255), -1)
                cv2.putText(display, f"#{h['slot_id']}", (center_px[0] - 10, center_px[1] + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                txt = f"({h['x_mm']:.1f},{h['y_mm']:.1f}) {h['angle_deg']:.0f}deg"
                cv2.putText(display, txt, (center_px[0] - 70, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.putText(
                display,
                f"Khung: {frame_result['empty_count']}/{frame_result['expected_count']} lo trong",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
            )

        frame_count += 1
        if frame_count >= 10:
            end_time = cv2.getTickCount()
            seconds = (end_time - start_time) / cv2.getTickFrequency()
            fps = frame_count / seconds if seconds > 0 else 0.0
            frame_count = 0
            start_time = cv2.getTickCount()
        cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display, f"Do sang: {current_brightness:.0f} (+/- de chinh)", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.imshow("Camera_4dof - Red Rectangle 10x20mm", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):
            current_brightness += CAMERA_BRIGHTNESS_STEP
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)
        elif key in (ord('-'), ord('_')):
            current_brightness -= CAMERA_BRIGHTNESS_STEP
            cap.set(cv2.CAP_PROP_BRIGHTNESS, current_brightness)

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Camera_4dof - nhan dien hinh chu nhat do + khung")
    parser.add_argument('--calib', type=str, default='calibration_result.npz')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--brightness', type=float, default=CAMERA_BRIGHTNESS_DEFAULT,
                         help="Do sang camera khoi tao (cv2.CAP_PROP_BRIGHTNESS); chinh them bang phim +/- khi dang chay.")
    args = parser.parse_args()
    if not os.path.isfile(args.calib):
        print(f"LOI: Khong tim thay file calib '{args.calib}'.")
        sys.exit(1)
    data = np.load(args.calib)
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    run_live_display(args.camera, camera_matrix, dist_coeffs, brightness=args.brightness)


if __name__ == '__main__':
    main()