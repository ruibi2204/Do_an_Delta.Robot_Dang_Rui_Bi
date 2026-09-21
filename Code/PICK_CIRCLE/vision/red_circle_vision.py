"""
red_circle_vision.py
---------------------
Module VISION ĐỘC LẬP: đọc file calib, phát hiện vòng tròn ĐỎ (mặc định
đường kính thật 30mm) trên 1 khung hình, và tính tọa độ (X, Y) tính bằng
mm so với gốc (0,0) của camera.

KHÔNG dùng PySide6/Qt - chỉ OpenCV + numpy - để tách biệt hoàn toàn khỏi
code giao diện (xem camera_tab.py, chỉ gọi sang module này).

KHÔNG import gì từ shared_state.py hay vision/Cameracircle.py - đây là 1
pipeline nhận diện + tính tọa độ RIÊNG, tự khai báo bảng hiệu chuẩn của
chính nó (xem CIRCLE_CALIB bên dưới), để không phụ thuộc / không đụng vào
các file production khác trong hệ thống.

Cách dùng (xem thêm ví dụ trong camera_tab.py):

    vision = RedCircleVision()
    vision.reset(calib_path="calibration_result.npz")   # gọi lại mỗi khi BẬT camera

    # Với mỗi khung hình mới (frame là ảnh BGR - numpy array từ OpenCV):
    result = vision.process_frame(frame, offset_x=..., offset_y=...)
    # result là (x_mm, y_mm) nếu tìm thấy vòng tròn đỏ, hoặc None nếu chưa
    # thấy. Hàm này VẼ LUÔN chú thích (khoanh vòng tròn + tọa độ) lên
    # chính `frame` (sửa tại chỗ).
"""

import os

import cv2
import numpy as np


# =====================================================================
# BẢNG HIỆU CHUẨN KÍCH THƯỚC (pixel -> mm)
#
# 'white' = vòng tròn ĐẶC, không có lỗ/viền rỗng ở giữa (contour ngoài
#           cùng, không có contour cha) - vòng tròn đỏ Ø30mm dùng loại
#           này.
# 'black' = vòng tròn CÓ LỖ/viền rỗng ở giữa dạng nhẫn (contour có cha) -
#           để dự phòng nếu sau này cần nhận diện thêm loại vật này.
#
# Hệ số scale/intercept lấy đúng theo số liệu hiệu chuẩn thực tế đã đo
# (dải ~25-31mm, khớp với vòng Ø30mm cần tìm ở đây):
#   diameter_mm = scale * diameter_px + intercept
# Nếu đo lại/hiệu chuẩn lại vòng tròn thực tế, CHỈ cần sửa 2 số ở đây.
# =====================================================================
CIRCLE_CALIB = {
    'white': {'scale': 0.32, 'intercept': 1.0},
}

# Đường kính thật mặc định cần tìm (mm). Dùng để lọc bỏ các vòng tròn có
# kích thước không hợp lý (không phải vòng Ø30mm cần tìm).
TARGET_DIAMETER_MM = 30.0
# Cho phép sai lệch +/- bao nhiêu mm quanh TARGET_DIAMETER_MM vẫn được
# chấp nhận là "đúng vòng tròn cần tìm" (bù cho sai số đo + rung ảnh).
DIAMETER_TOLERANCE_MM = 6.0

# Bán kính tối thiểu/tối đa hợp lý trên ảnh (px) - lọc nhiễu quá nhỏ và
# vật quá to bất thường (không phải vòng tròn thật).
MIN_RADIUS_PX = 8
MAX_RADIUS_PX = 140

# Diện tích tối thiểu (px^2) để không coi các đốm nhiễu li ti là vòng tròn.
MIN_AREA_PX = 200

# Độ tròn tối thiểu (1.0 = tròn hoàn hảo) - loại bỏ hình chữ nhật/vật có
# góc cạnh (ví dụ HCN đỏ 10x20mm mà DOF4 đang nhận diện song song).
MIN_CIRCULARITY = 0.72


def _get_calib(shape_type):
    return CIRCLE_CALIB['white'] if shape_type == 'white' else CIRCLE_CALIB['black']


def calculate_real_diameter_mm(radius_px, shape_type):
    """diameter_mm = scale * diameter_px + intercept, theo CIRCLE_CALIB."""
    diameter_px = radius_px * 2.0
    c = _get_calib(shape_type)
    return c['scale'] * diameter_px + c['intercept']


def px_to_mm_scale(shape_type):
    return _get_calib(shape_type)['scale']


def _refine_center_radius(cnt, area):
    """Tính tâm/bán kính chính xác hơn minEnclosingCircle thô, bằng cách
    kết hợp moments (tâm khối lượng) + fitEllipse (khi đủ điểm)."""
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
    num_labels, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask
    cleaned = np.zeros_like(mask)
    for lbl in range(1, num_labels):
        if stats[lbl, cv2.CC_STAT_AREA] >= min_area:
            cleaned[labels == lbl] = 255
    return cleaned


def detect_red_circles(frame_bgr):
    """
    Tìm tất cả vòng tròn ĐỎ hợp lệ trong khung hình.
    Trả về list [(x_px, y_px, radius_px, shape_type), ...] với shape_type
    là 'white' (đặc) hoặc 'black' (có lỗ giữa).
    """
    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 1.5)
    b, g, r = cv2.split(blurred.astype(np.int16))
    red_condition = (r - g > 35) & (r - b > 35) & (r > 80)
    red_mask = np.uint8(red_condition * 255)

    kernel_close = np.ones((7, 7), np.uint8)
    kernel_open = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)
    red_mask = _remove_small_components(red_mask, min_area=MIN_AREA_PX)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]

    results = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < MIN_AREA_PX:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * np.pi * (area / (perimeter * perimeter))
        if circularity < MIN_CIRCULARITY:
            continue

        if len(cnt) >= 5:
            _, (minor_ax, major_ax), _ = cv2.fitEllipse(cnt)
            if major_ax > 0 and (minor_ax / major_ax) < 0.70:
                continue

        parent_idx = hierarchy[i][3]
        x, y, radius = _refine_center_radius(cnt, area)
        if radius < MIN_RADIUS_PX or radius > MAX_RADIUS_PX:
            continue

        shape_type = 'white' if parent_idx == -1 else 'black'
        diameter_mm = calculate_real_diameter_mm(radius, shape_type)
        if abs(diameter_mm - TARGET_DIAMETER_MM) > DIAMETER_TOLERANCE_MM:
            continue

        results.append((x, y, radius, shape_type))

    # Chỉ giữ lại vòng tròn lớn nhất (ổn định nhất) nếu có nhiều hơn 1 -
    # đủ dùng cho mục đích canh 1 vòng tròn Ø30mm.
    results.sort(key=lambda c: c[2] * c[2], reverse=True)
    return results[:4]


class _CircleTracker:
    """Làm mượt vị trí vòng tròn qua các frame để chống chớp giật, và chỉ
    báo "đã phát hiện" sau khi thấy ổn định vài khung hình liên tiếp
    (min_hits) để tránh báo nhầm do nhiễu 1 khung hình."""

    def __init__(self, alpha=0.35, max_disappeared=15, dist_threshold=30, min_hits=3):
        self.alpha = alpha
        self.max_disappeared = max_disappeared
        self.dist_threshold = dist_threshold
        self.min_hits = min_hits
        self.tracked = []

    def update(self, detected):
        for t in self.tracked:
            t['disappeared'] += 1

        for x, y, r, shape_type in detected:
            matched = False
            for t in self.tracked:
                if t['type'] != shape_type:
                    continue
                dist = float(np.hypot(t['center'][0] - x, t['center'][1] - y))
                if dist < self.dist_threshold:
                    a = self.alpha
                    t['center'][0] = a * x + (1 - a) * t['center'][0]
                    t['center'][1] = a * y + (1 - a) * t['center'][1]
                    t['radius'] = a * r + (1 - a) * t['radius']
                    t['disappeared'] = 0
                    t['hits'] = t.get('hits', 0) + 1
                    matched = True
                    break
            if not matched:
                self.tracked.append({
                    'center': [float(x), float(y)], 'radius': float(r),
                    'type': shape_type, 'disappeared': 0, 'hits': 1,
                })

        self.tracked = [t for t in self.tracked if t['disappeared'] < self.max_disappeared]
        return [t for t in self.tracked if t['hits'] >= self.min_hits]


class RedCircleVision:
    """
    Đối tượng xử lý VISION cho 1 camera: giữ trạng thái calib + tracker
    riêng, xử lý từng khung hình để ra tọa độ (X, Y mm) của vòng tròn đỏ.
    """

    def __init__(self):
        self._tracker = _CircleTracker()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None

    def reset(self, calib_path=None):
        """Gọi lại mỗi khi BẬT camera: xóa tracker cũ, đọc lại file calib.
        Trả về chuỗi lỗi nếu đọc calib thất bại, hoặc None nếu OK/không có
        calib (sẽ dùng tâm hình học làm gốc tọa độ khi đó)."""
        self._tracker = _CircleTracker()
        self._fx = self._fy = self._cx = self._cy = None
        self._camera_matrix = None
        self._dist_coeffs = None

        if calib_path and os.path.isfile(calib_path):
            try:
                data = np.load(calib_path)
                self._camera_matrix = data["camera_matrix"]
                self._dist_coeffs = data["dist_coeffs"]
            except Exception as e:
                return f"Không đọc được file calib: {e}"
        return None

    def _ensure_intrinsics(self, frame_shape):
        if self._fx is not None:
            return
        h, w = frame_shape[:2]
        if self._camera_matrix is not None and self._dist_coeffs is not None:
            new_cm, _roi = cv2.getOptimalNewCameraMatrix(
                self._camera_matrix, self._dist_coeffs, (w, h), 0, (w, h)
            )
            self._fx = float(new_cm[0, 0])
            self._fy = float(new_cm[1, 1])
            self._cx = float(new_cm[0, 2])
            self._cy = float(new_cm[1, 2])
        else:
            self._fx = 1.0
            self._fy = 1.0
            self._cx = w / 2.0
            self._cy = h / 2.0

    def process_frame(self, frame, offset_x=0.0, offset_y=0.0, draw=True):
        """
        Xử lý 1 khung hình (BGR, đã undistort sẵn nếu bên gọi đã hiệu
        chỉnh). Trả về (x_mm, y_mm) nếu tìm thấy vòng tròn đỏ Ø30mm, hoặc
        None nếu chưa thấy. Nếu draw=True, vẽ chú thích lên `frame` (sửa
        tại chỗ - in-place).
        """
        self._ensure_intrinsics(frame.shape)
        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        detected = detect_red_circles(frame)
        stable = self._tracker.update(detected)

        if not stable:
            if draw:
                cv2.putText(
                    frame, "Vong tron do O30mm: KHONG THAY", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA,
                )
            return None

        item = max(stable, key=lambda c: c['radius'])
        center_f = item['center']
        radius_f = item['radius']
        shape_type = item['type']

        diameter_mm = calculate_real_diameter_mm(radius_f, shape_type)
        scale_mm_per_px = px_to_mm_scale(shape_type)

        # Ước lượng khoảng cách Z (mô hình pinhole).
        Z_mm = scale_mm_per_px * fx

        # Tọa độ phẳng: trục X hướng sang trái, trục Y hướng lên trên, gốc
        # (0,0) tại tâm quang học đã hiệu chỉnh.
        x_apparent = (-center_f[0] + cx) * scale_mm_per_px
        y_apparent = (cy - center_f[1]) * scale_mm_per_px * (fx / fy)

        # Bù sai số thị sai (parallax) theo bán kính vật.
        R_mm = diameter_mm / 2.0
        parallax_factor = (Z_mm - R_mm) / Z_mm if Z_mm > 0 else 1.0
        x_mm = x_apparent * parallax_factor
        y_mm = y_apparent * parallax_factor

        # Cộng offset do bên gọi truyền vào (bù lệch giữa gốc tọa độ camera
        # và gốc tọa độ thực tế - ví dụ offset đang chỉnh trên giao diện).
        x_mm += offset_x
        y_mm += offset_y

        if draw:
            center_px = (int(round(center_f[0])), int(round(center_f[1])))
            radius_px = int(round(radius_f))
            cv2.circle(frame, center_px, radius_px, (0, 0, 255), 2)
            cv2.circle(frame, center_px, 2, (0, 0, 255), 3)
            cv2.putText(
                frame, f"({x_mm:.1f}, {y_mm:.1f}) mm",
                (center_px[0] - 60, center_px[1] - radius_px - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA,
            )
            corner_text = f"Vong tron do O30mm:  X = {x_mm:.1f} mm   Y = {y_mm:.1f} mm"
            cv2.putText(frame, corner_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, corner_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

        return x_mm, y_mm