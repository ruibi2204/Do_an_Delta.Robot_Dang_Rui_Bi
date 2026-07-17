import numpy as np
import cv2
def detect_object_pixel(frame_bgr, hsv_lower, hsv_upper, min_area=200):

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None, mask

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask

    px = M["m10"] / M["m00"]
    py = M["m01"] / M["m00"]
    return (px, py), mask


class CameraCalibration:
    """Luu cac cap diem hieu chinh (pixel <-> robot) va tinh ma tran bien
    doi Affine 2D de doi toa do anh (px, py) sang toa do thuc (X, Y) mm
    tren mat ban robot."""

    def __init__(self):
        self.pixel_points = []   # list[(px, py)]
        self.robot_points = []   # list[(X, Y)]
        self.matrix = None       # ma tran affine 2x3 (numpy array)

    def add_point_pair(self, pixel_xy, robot_xy):
        self.pixel_points.append(tuple(pixel_xy))
        self.robot_points.append(tuple(robot_xy))

    def clear(self):
        self.pixel_points.clear()
        self.robot_points.clear()
        self.matrix = None

    def compute(self):
        n = len(self.pixel_points)
        if n < 3:
            raise ValueError("Can it nhat 3 cap diem hieu chinh de tinh toan!")

        src = np.array(self.pixel_points, dtype=np.float64)
        dst = np.array(self.robot_points, dtype=np.float64)

        if n == 3:
            self.matrix = cv2.getAffineTransform(
                src.astype(np.float32), dst.astype(np.float32)
            )
        else:
            ones = np.ones((n, 1), dtype=np.float64)
            A = np.hstack([src, ones])
            sol_x, _, _, _ = np.linalg.lstsq(A, dst[:, 0], rcond=None)
            sol_y, _, _, _ = np.linalg.lstsq(A, dst[:, 1], rcond=None)
            self.matrix = np.array([sol_x, sol_y], dtype=np.float64)

        return self.matrix

    def pixel_to_robot(self, px, py):
        if self.matrix is None:
            # Fallback sang công thức giải tích với tâm tại (0, -80)
            scale_x = 120.0 / 307.6
            scale_y = 160.0 / 379.38
            X = (240.0 - py) * scale_x
            Y = -80.0 + (320.0 - px) * scale_y
            return X, Y
        vec = np.array([px, py, 1.0], dtype=np.float64)
        X = float(np.dot(self.matrix[0], vec))
        Y = float(np.dot(self.matrix[1], vec))
        return X, Y

    # ----- CÁC HÀM LƯU / LOAD ĐÃ ĐƯỢC SỬA -----
    def save(self, path, format='auto'):
        """
        Lưu ma trận hiệu chỉnh ra file.
        - Nếu path kết thúc bằng '.npz' -> lưu dạng .npz (nén).
        - Nếu path kết thúc bằng '.csv' hoặc '.txt' -> lưu dạng CSV (dùng savetxt).
        - Nếu format='npz' -> bắt buộc lưu .npz (có thể thêm hậu tố nếu cần).
        """
        if self.matrix is None:
            raise ValueError("Chua co ma tran hieu chinh de luu.")

        # Tự động chọn định dạng theo phần mở rộng nếu format='auto'
        if format == 'auto':
            if path.lower().endswith('.npz'):
                fmt = 'npz'
            else:
                fmt = 'csv'
        else:
            fmt = format

        if fmt == 'npz':
            # Lưu ma trận vào file .npz (có thể nén)
            np.savez_compressed(path, matrix=self.matrix)
        else:
            # Mặc định lưu CSV
            np.savetxt(path, self.matrix, delimiter=",")

    def load(self, path):
        """
        Đọc ma trận hiệu chỉnh từ file.
        Tự động nhận dạng:
        - .npz  -> dùng numpy.load
        - .csv / .txt -> dùng numpy.loadtxt
        """
        if path.lower().endswith('camera_calib_XY.npz'):
            # Đọc file .npz (có thể chứa nhiều mảng)
            data = np.load(path)
            # Giả sử ma trận được lưu với key 'matrix'
            if 'matrix' not in data:
                raise ValueError("File .npz khong chua key 'matrix'. Hay kiem tra lai.")
            self.matrix = data['matrix']
        else:
            # Mặc định đọc dạng CSV
            self.matrix = np.loadtxt(path, delimiter=",")

        return self.matrix

    # (Tùy chọn) Hàm lưu toàn bộ dữ liệu (cả điểm và ma trận) vào .npz
    def save_full(self, path):
        """Lưu tất cả dữ liệu (pixel_points, robot_points, matrix) vào file .npz."""
        data = {
            'pixel_points': np.array(self.pixel_points, dtype=np.float64),
            'robot_points': np.array(self.robot_points, dtype=np.float64),
            'matrix': self.matrix
        }
        np.savez_compressed(path, **data)

    def load_full(self, path):
        """Đọc toàn bộ dữ liệu từ file .npz (được lưu bởi save_full)."""
        data = np.load(path)
        self.pixel_points = [tuple(p) for p in data['pixel_points']]
        self.robot_points = [tuple(p) for p in data['robot_points']]
        self.matrix = data['matrix']
        return self.matrix
