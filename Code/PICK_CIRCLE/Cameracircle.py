import argparse
import os
import sys

import cv2
import numpy as np


class CircleTracker:
    """Lớp quản lý và làm mượt vị trí các vòng tròn qua các frame để chống chớp giật."""

    def __init__(self, alpha=0.35, max_disappeared=15, dist_threshold=30,
                 min_hits=3, max_radius_jump_ratio=0.1):
        self.alpha = alpha
        self.max_disappeared = max_disappeared
        self.dist_threshold = dist_threshold
        self.min_hits = min_hits
        self.max_radius_jump_ratio = max_radius_jump_ratio
        self.tracked_circles = []

    def update(self, detected_circles):
        for tracked in self.tracked_circles:
            tracked['disappeared'] += 1

        for new_c in detected_circles:
            new_x, new_y, new_r, c_type = new_c
            matched = False

            for tracked in self.tracked_circles:
                if tracked['type'] == c_type:
                    dist = np.hypot(
                        tracked['center'][0] - new_x, tracked['center'][1] - new_y
                    )
                    if dist < self.dist_threshold:
                        old_r = tracked['radius']
                        if old_r > 0 and abs(new_r - old_r) / old_r > self.max_radius_jump_ratio:
                            a = self.alpha * 0.25
                        else:
                            a = self.alpha

                        tracked['center'][0] = a * new_x + (1 - a) * tracked['center'][0]
                        tracked['center'][1] = a * new_y + (1 - a) * tracked['center'][1]
                        tracked['radius'] = a * new_r + (1 - a) * old_r
                        tracked['disappeared'] = 0
                        tracked['hits'] = tracked.get('hits', 0) + 1
                        matched = True
                        break

            if not matched:
                self.tracked_circles.append({
                    'center': [float(new_x), float(new_y)],
                    'radius': float(new_r),
                    'type': c_type,
                    'disappeared': 0,
                    'hits': 1,
                })

        self.tracked_circles = [
            t for t in self.tracked_circles if t['disappeared'] < self.max_disappeared
        ]

        confirmed = [t for t in self.tracked_circles if t['hits'] >= self.min_hits]
        return confirmed


# =====================================================================
# HIỆU CHUẨN KÍCH THƯỚC (pixel -> mm)
# Cập nhật từ hồi quy tuyến tính trên số liệu đo thực tế (xem calibrate_circle.py):
#   white: 3 điểm đo (25mm, 30mm, 30mm)  -> R²=0.9993, sai số lớn nhất 0.08mm
#   black: 3 điểm đo (26mm, 31mm, 31mm)  -> R²=0.9998, sai số lớn nhất 0.04mm
# LƯU Ý: số liệu hiệu chuẩn hiện chỉ trải trong khoảng ~25-31mm. Nếu dùng vật
# có kích thước ngoài khoảng này (nhỏ hơn ~20mm hoặc lớn hơn ~40mm), nên đo
# thêm điểm ở dải đó và hồi quy lại để tránh sai số ngoại suy.
# =====================================================================
_CIRCLE_CALIB = {
    'white': {'scale': 0.325, 'intercept': 1.0},
    'black': {'scale': 0.32, 'intercept': 1.0},
}


def _get_calib(circle_type):
    key = 'white' if 'white' in circle_type else 'black'
    return _CIRCLE_CALIB[key]


def calculate_real_properties(radius_pixel, circle_type):
    diameter_pixel = radius_pixel * 2
    c = _get_calib(circle_type)
    return (c['scale'] * diameter_pixel) + c['intercept']


def px_to_mm_scale(circle_type):
    return _get_calib(circle_type)['scale']


# =====================================================================
# HIỆU CHỈNH TỌA ĐỘ X/Y: camera -> hệ tọa độ robot
# Mặc định là ma trận đơn vị + offset 0 (chưa hiệu chỉnh gì thêm), vì cần
# số liệu đo thực tế (xem calibrate_xy.py) mới tính ra được. Sau khi có
# số liệu, dán kết quả từ calibrate_xy.py vào đây.
#
# - Nếu chỉ có offset đơn giản (lệch tâm thuần túy): giữ XY_CALIB_MATRIX là
#   ma trận đơn vị, chỉ đổi XY_CALIB_OFFSET.
# - Nếu camera bị lệch xoay/tỉ lệ so với trục robot: dùng cả XY_CALIB_MATRIX
#   (không phải ma trận đơn vị nữa) lẫn XY_CALIB_OFFSET.
# =====================================================================
XY_CALIB_MATRIX = np.array([[1.0, 0.0], [0.0, 1.0]])
XY_CALIB_OFFSET = np.array([0.0, 0.0])


def apply_xy_calibration(x_mm, y_mm):
    """Áp ma trận hiệu chỉnh (xoay/tỉ lệ) + offset (lệch tâm) đã đo thực tế
    lên tọa độ X/Y tính từ hình học camera, để khớp đúng hệ tọa độ robot.
    """
    v = XY_CALIB_MATRIX @ np.array([x_mm, y_mm]) + XY_CALIB_OFFSET
    return float(v[0]), float(v[1])


# =====================================================================
# KHỬ MÉO ẢNH - CACHE & FIX TÂM ẢNH
# =====================================================================
_undistort_cache = {}


def build_undistort_maps(camera_matrix, dist_coeffs, image_size):
    w, h = image_size
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 0, (w, h)
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1
    )
    return map1, map2, new_camera_matrix


def undistort_image(img, camera_matrix, dist_coeffs):
    h, w = img.shape[:2]
    key = (camera_matrix.tobytes(), dist_coeffs.tobytes(), w, h)
    cached = _undistort_cache.get(key)
    if cached is None:
        map1, map2, new_camera_matrix = build_undistort_maps(
            camera_matrix, dist_coeffs, (w, h)
        )
        _undistort_cache[key] = (map1, map2, new_camera_matrix)
        if len(_undistort_cache) > 4:
            _undistort_cache.pop(next(iter(_undistort_cache)))
    else:
        map1, map2, new_camera_matrix = cached

    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted, new_camera_matrix


def _refine_center_radius(cnt, area):
    m = cv2.moments(cnt)
    if m['m00'] == 0:
        (x, y), r = cv2.minEnclosingCircle(cnt)
        return float(x), float(y), float(r)

    cx = m['m10'] / m['m00']
    cy = m['m01'] / m['m00']
    radius_area = float(np.sqrt(area / np.pi))

    if len(cnt) >= 5:
        (ex, ey), (minor_ax, major_ax), _angle = cv2.fitEllipse(cnt)
        radius_ellipse = (minor_ax + major_ax) / 4.0
        x = (cx + ex) / 2.0
        y = (cy + ey) / 2.0
        radius = (radius_area + radius_ellipse) / 2.0
    else:
        x, y = cx, cy
        radius = radius_area

    return float(x), float(y), float(radius)


def _remove_small_components(mask, min_area=200):
    num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )
    if num_labels <= 1:
        return mask

    cleaned = np.zeros_like(mask)
    for lbl in range(1, num_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == lbl] = 255
    return cleaned


# =====================================================================
# THUẬT TOÁN NHẬN DIỆN RGB (GIỮ NGUYÊN TÊN HÀM CŨ THEO YÊU CẦU)
# =====================================================================
def detect_circles_hsv_optimized(frame):
    max_allowed_radius = 110

    blurred = cv2.GaussianBlur(frame, (5, 5), 1.5)
    b, g, r = cv2.split(blurred.astype(np.int16))

    red_condition = (r - g > 35) & (r - b > 35) & (r > 80)
    red_mask = np.uint8(red_condition * 255)

    blue_condition = (b - r > 30) & (b - g > 15) & (b > 90)
    blue_mask = np.uint8(blue_condition * 255)

    kernel_close = np.ones((7, 7), np.uint8)
    kernel_open = np.ones((5, 5), np.uint8)

    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel_close)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel_open)

    detected_this_frame = []

    def process_contours(mask, color_name, max_count=4):
        mask = _remove_small_components(mask, min_area=200)
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        if hierarchy is None:
            return
        hierarchy = hierarchy[0]
        candidates = []

        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < 200:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if circularity < 0.72:
                continue

            if len(cnt) >= 5:
                _, (minor_ax, major_ax), _ = cv2.fitEllipse(cnt)
                if major_ax > 0 and (minor_ax / major_ax) < 0.70:
                    continue

            parent_idx = hierarchy[i][3]
            x, y, radius = _refine_center_radius(cnt, area)

            if radius < 10 or radius > max_allowed_radius:
                continue

            sub_type = 'white' if parent_idx == -1 else 'black'
            circle_type = f'{color_name}_{sub_type}'
            temp_diameter = calculate_real_properties(radius, circle_type)

            if temp_diameter < 18.0 or temp_diameter > 65.0:
                continue

            candidates.append((x, y, radius, circle_type))

        candidates.sort(key=lambda c: c[2] * c[2], reverse=True)
        detected_this_frame.extend(candidates[:max_count])

    process_contours(red_mask, 'red', max_count=4)
    process_contours(blue_mask, 'blue', max_count=4)

    return detected_this_frame


# =====================================================================
# VÒNG LẶP LIVE DISPLAY CẢI TIẾN TOÁN HỌC KHÔNG GIAN
# =====================================================================
def run_live_display(camera_index, camera_matrix, dist_coeffs):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Không thể mở camera index {camera_index}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    tracker = CircleTracker(alpha=0.35, max_disappeared=15, dist_threshold=30)
    print("Đang chạy chế độ Live Camera. Nhấn 'q' để thoát.")

    fps = 0
    frame_count = 0
    start_time = cv2.getTickCount()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted, new_camera_matrix = undistort_image(
            frame, camera_matrix, dist_coeffs
        )

        detected = detect_circles_hsv_optimized(undistorted)
        stable_circles = tracker.update(detected)

        display = undistorted.copy()

        # 1. Trích xuất Tiêu cự (Focal Lengths) và Tâm (Optical Center)
        fx = new_camera_matrix[0, 0]
        fy = new_camera_matrix[1, 1]
        cx = new_camera_matrix[0, 2]
        cy = new_camera_matrix[1, 2]

        origin_px = (int(round(cx)), int(round(cy)))

        # Vẽ hệ trục
        axis_len_px = 60
        cv2.arrowedLine(display, origin_px, (origin_px[0] - axis_len_px, origin_px[1]), (0, 0, 255), 2, tipLength=0.2)
        cv2.arrowedLine(display, origin_px, (origin_px[0], origin_px[1] - axis_len_px), (255, 0, 0), 2, tipLength=0.2)
        cv2.circle(display, origin_px, 4, (0, 255, 255), -1)
        cv2.putText(display, 'O(0,0)', (origin_px[0] + 6, origin_px[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        for item in stable_circles:
            center_f = item['center']
            radius_f = item['radius']
            c_type = item['type']

            diameter_mm = calculate_real_properties(radius_f, c_type)
            scale_mm_per_px = px_to_mm_scale(c_type)

            # 2. Ước lượng khoảng cách Z (từ camera tới mặt mâm)
            # Dựa vào mô hình Pinhole: Z = scale_thực_tế * Tiêu_cự
            Z_mm = scale_mm_per_px * fx

            # 3. Tính tọa độ phẳng chính xác bù trừ chênh lệch tỉ lệ trục fx, fy
            # Gốc tọa độ O(0,0) đặt tại tâm camera.
            # Trục X hướng sang trái (giữ nguyên logic gốc của bạn), Trục Y hướng lên trên.
            x_apparent = (-center_f[0] + cx) * scale_mm_per_px
            y_apparent = (cy - center_f[1]) * scale_mm_per_px * (fx / fy)

            # 4. Hiệu chỉnh Parallax Error (Sai số thị sai) cho Ball and Plate
            # Loại bỏ sự dịch chuyển giả mạo khi bóng lăn ra xa rìa mâm
            R_ball_mm = diameter_mm / 2.0
            parallax_factor = (Z_mm - R_ball_mm) / Z_mm if Z_mm > 0 else 1.0

            x_mm = x_apparent * parallax_factor
            y_mm = y_apparent * parallax_factor

            # 5. Hiệu chỉnh camera -> hệ tọa độ robot (offset lệch tâm, và cả
            # xoay/tỉ lệ nếu có — xem calibrate_xy.py để đo và tính ra
            # XY_CALIB_MATRIX / XY_CALIB_OFFSET từ số liệu thực tế).
            x_mm, y_mm = apply_xy_calibration(x_mm, y_mm)

            center = (int(round(center_f[0])), int(round(center_f[1])))
            radius = int(round(radius_f))

            if 'red' in c_type:
                main_color = (0, 0, 255)
                label_prefix = 'Do'
            else:
                main_color = (255, 0, 0)
                label_prefix = 'Xanh'

            cv2.circle(display, center, radius, main_color, 2)
            cv2.circle(display, center, 2, (0, 0, 255), 3)

            sub_txt = 'Trang' if 'white' in c_type else 'Den'
            text_d = f'{label_prefix}-{sub_txt}: {diameter_mm:.1f}mm'
            text_pos = f'X:{x_mm:.1f}, Y:{y_mm:.1f} mm'

            cv2.putText(display, text_d, (center[0] - 60, center[1] - radius - 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, main_color, 2)
            cv2.putText(display, text_pos, (center[0] - 60, center[1] - radius - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        frame_count += 1
        if frame_count >= 10:
            end_time = cv2.getTickCount()
            seconds = (end_time - start_time) / cv2.getTickFrequency()
            fps = frame_count / seconds
            frame_count = 0
            start_time = cv2.getTickCount()

        cv2.putText(display, f'FPS: {fps:.1f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Live Camera - Tracking', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='Live camera nhận diện')
    parser.add_argument('--calib', type=str, default='calibration_result.npz',
                        help='File .npz kết quả calib')
    parser.add_argument('--camera', type=int, default=0, help='Chỉ số webcam')
    args = parser.parse_args()

    if not os.path.isfile(args.calib):
        print(f"LỖI: Không tìm thấy file calib '{args.calib}'.")
        sys.exit(1)

    data = np.load(args.calib)
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']

    run_live_display(args.camera, camera_matrix, dist_coeffs)


if __name__ == '__main__':
    main()