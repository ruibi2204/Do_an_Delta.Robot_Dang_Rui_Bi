"""
camera_module.py - Module quản lý camera và phát hiện vật thể
Hỗ trợ phát hiện màu đỏ và các màu khác, xác định tọa độ tâm vật thể
"""

import cv2
import numpy as np
import threading
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage
from typing import Tuple, Optional, List, Dict, Any
from enum import Enum


class ColorRange:
    """Lớp lưu trữ khoảng màu HSV"""

    def __init__(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int], name: str = ""):
        self.lower = np.array(lower, dtype=np.uint8)
        self.upper = np.array(upper, dtype=np.uint8)
        self.name = name

    def get_mask(self, hsv_image: np.ndarray) -> np.ndarray:
        """Tạo mask từ ảnh HSV"""
        return cv2.inRange(hsv_image, self.lower, self.upper)


class ColorPresets:
    """Các khoảng màu HSV thông dụng"""

    # Màu đỏ (có 2 khoảng do màu đỏ nằm ở 2 đầu của trục H)
    RED_LOW = ColorRange((0, 100, 100), (10, 255, 255), "Đỏ (thấp)")
    RED_HIGH = ColorRange((160, 100, 100), (179, 255, 255), "Đỏ (cao)")

    # Các màu khác
    BLUE = ColorRange((100, 100, 100), (130, 255, 255), "Xanh dương")
    GREEN = ColorRange((40, 100, 100), (80, 255, 255), "Xanh lá")
    YELLOW = ColorRange((20, 100, 100), (30, 255, 255), "Vàng")
    ORANGE = ColorRange((10, 100, 100), (20, 255, 255), "Cam")
    PURPLE = ColorRange((130, 100, 100), (160, 255, 255), "Tím")
    CYAN = ColorRange((80, 100, 100), (100, 255, 255), "Xanh lơ")

    @classmethod
    def get_red_mask(cls, hsv_image: np.ndarray) -> np.ndarray:
        """Tạo mask cho màu đỏ (kết hợp 2 khoảng)"""
        mask1 = cls.RED_LOW.get_mask(hsv_image)
        mask2 = cls.RED_HIGH.get_mask(hsv_image)
        return cv2.bitwise_or(mask1, mask2)

    @classmethod
    def get_all_presets(cls) -> Dict[str, List[ColorRange]]:
        """Trả về tất cả các khoảng màu"""
        return {
            "Đỏ": [cls.RED_LOW, cls.RED_HIGH],
            "Xanh dương": [cls.BLUE],
            "Xanh lá": [cls.GREEN],
            "Vàng": [cls.YELLOW],
            "Cam": [cls.ORANGE],
            "Tím": [cls.PURPLE],
            "Xanh lơ": [cls.CYAN],
        }


class DetectedObject:
    """Lớp lưu thông tin vật thể phát hiện được"""

    def __init__(self,
                 center_x: float,
                 center_y: float,
                 width: float,
                 height: float,
                 area: float,
                 contour: np.ndarray,
                 color_name: str = "",
                 confidence: float = 1.0):
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.area = area
        self.contour = contour
        self.color_name = color_name
        self.confidence = confidence

    @property
    def center(self) -> Tuple[float, float]:
        return (self.center_x, self.center_y)

    @property
    def rect(self) -> Tuple[float, float, float, float]:
        return (self.center_x, self.center_y, self.width, self.height)

    def to_dict(self) -> Dict[str, Any]:
        """Chuyển đổi thành dictionary để dễ xử lý"""
        return {
            'center_x': self.center_x,
            'center_y': self.center_y,
            'width': self.width,
            'height': self.height,
            'area': self.area,
            'color_name': self.color_name,
            'confidence': self.confidence,
        }


class ObjectDetector:
    """Lớp phát hiện vật thể từ ảnh"""

    def __init__(self,
                 min_area: float = 100,
                 max_area: float = 50000,
                 min_aspect_ratio: float = 0.3,
                 max_aspect_ratio: float = 3.0,
                 use_contour_approximation: bool = True,
                 show_debug: bool = False):
        """
        Khởi tạo bộ phát hiện vật thể

        Args:
            min_area: Diện tích tối thiểu của vật thể (pixel²)
            max_area: Diện tích tối đa của vật thể (pixel²)
            min_aspect_ratio: Tỉ lệ khung hình tối thiểu (width/height)
            max_aspect_ratio: Tỉ lệ khung hình tối đa (width/height)
            use_contour_approximation: Sử dụng xấp xỉ contour (giảm nhiễu)
            show_debug: Hiển thị debug (mask, contours)
        """
        self.min_area = min_area
        self.max_area = max_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.use_contour_approximation = use_contour_approximation
        self.show_debug = show_debug

        # Bộ lọc màu tùy chỉnh
        self.color_ranges: List[ColorRange] = []

        # Ảnh debug
        self.debug_mask = None
        self.debug_contours = None

    def add_color_range(self, color_range: ColorRange):
        """Thêm khoảng màu cần phát hiện"""
        self.color_ranges.append(color_range)

    def add_red_color(self):
        """Thêm khoảng màu đỏ (2 khoảng)"""
        self.color_ranges.append(ColorPresets.RED_LOW)
        self.color_ranges.append(ColorPresets.RED_HIGH)

    def clear_color_ranges(self):
        """Xóa tất cả khoảng màu"""
        self.color_ranges.clear()

    def set_color_ranges(self, color_ranges: List[ColorRange]):
        """Thiết lập danh sách khoảng màu"""
        self.color_ranges = color_ranges

    def detect(self, image: np.ndarray, return_all: bool = False) -> Optional[DetectedObject]:
        """
        Phát hiện vật thể trong ảnh

        Args:
            image: Ảnh BGR
            return_all: Trả về tất cả vật thể tìm thấy (mặc định chỉ trả về vật thể lớn nhất)

        Returns:
            DetectedObject hoặc List[DetectedObject] nếu return_all=True
        """
        if image is None:
            return None

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Tạo mask từ các khoảng màu
        if len(self.color_ranges) == 0:
            # Mặc định phát hiện màu đỏ
            mask = ColorPresets.get_red_mask(hsv)
        else:
            masks = []
            for color_range in self.color_ranges:
                mask = color_range.get_mask(hsv)
                masks.append(mask)
            mask = cv2.bitwise_or(*masks) if len(masks) > 1 else masks[0]

        # Lưu mask để debug
        self.debug_mask = mask.copy()

        # Xử lý mask: loại bỏ nhiễu
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Tìm contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Lưu contours để debug
        self.debug_contours = contours

        # Lọc các vật thể
        objects = []
        for contour in contours:
            # Tính diện tích
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            # Lọc theo tỉ lệ khung hình
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue

            # Tìm tâm
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]

            # Xấp xỉ contour để giảm nhiễu
            if self.use_contour_approximation:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                contour = cv2.approxPolyDP(contour, epsilon, True)

            # Xác định màu sắc (lấy màu trung bình của vùng)
            mask_roi = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask_roi, [contour], -1, 255, -1)
            mean_color = cv2.mean(image, mask=mask_roi)[:3]

            # Tìm tên màu gần nhất
            color_name = self._get_closest_color_name(mean_color)

            obj = DetectedObject(
                center_x=cx,
                center_y=cy,
                width=w,
                height=h,
                area=area,
                contour=contour,
                color_name=color_name,
                confidence=area / self.max_area if area < self.max_area else 1.0
            )
            objects.append(obj)

        if not objects:
            return None

        # Sắp xếp theo diện tích giảm dần
        objects.sort(key=lambda o: o.area, reverse=True)

        if return_all:
            return objects
        return objects[0]

    def _get_closest_color_name(self, bgr_color: Tuple[float, float, float]) -> str:
        """Tìm tên màu gần nhất với BGR color"""
        # Chuyển BGR sang HSV
        bgr = np.uint8([[bgr_color]])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        # Định nghĩa các khoảng màu
        color_ranges = [
            ("Đỏ", 0, 10, 100, 255, 100, 255),
            ("Đỏ", 160, 179, 100, 255, 100, 255),
            ("Cam", 10, 20, 100, 255, 100, 255),
            ("Vàng", 20, 30, 100, 255, 100, 255),
            ("Xanh lá", 40, 80, 100, 255, 100, 255),
            ("Xanh dương", 100, 130, 100, 255, 100, 255),
            ("Xanh lơ", 80, 100, 100, 255, 100, 255),
            ("Tím", 130, 160, 100, 255, 100, 255),
        ]

        for name, h_min, h_max, s_min, s_max, v_min, v_max in color_ranges:
            if h_min <= h <= h_max and s_min <= s <= s_max and v_min <= v <= v_max:
                return name

        return "Khác"

    def draw_detection(self, image: np.ndarray, objects: List[DetectedObject]) -> np.ndarray:
        """Vẽ các vật thể phát hiện lên ảnh"""
        img = image.copy()

        for obj in objects:
            # Vẽ bounding box
            x = int(obj.center_x - obj.width / 2)
            y = int(obj.center_y - obj.height / 2)
            cv2.rectangle(img, (x, y),
                          (x + int(obj.width), y + int(obj.height)),
                          (0, 255, 0), 2)

            # Vẽ tâm
            cv2.circle(img, (int(obj.center_x), int(obj.center_y)), 5, (0, 0, 255), -1)

            # Vẽ contour
            cv2.drawContours(img, [obj.contour], -1, (255, 0, 0), 2)

            # Hiển thị thông tin
            label = f"{obj.color_name} ({obj.area:.0f})"
            cv2.putText(img, label,
                        (int(obj.center_x - 30), int(obj.center_y - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Hiển thị tọa độ
            coord_text = f"({obj.center_x:.1f}, {obj.center_y:.1f})"
            cv2.putText(img, coord_text,
                        (int(obj.center_x - 30), int(obj.center_y + 30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        return img

    def get_debug_images(self) -> Dict[str, np.ndarray]:
        """Trả về ảnh debug (mask, contours)"""
        debug_images = {}

        if self.debug_mask is not None:
            # Chuyển mask thành 3 kênh để hiển thị
            debug_images['mask'] = cv2.cvtColor(self.debug_mask, cv2.COLOR_GRAY2BGR)

        if self.debug_contours is not None:
            # Tạo ảnh contours
            contour_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.drawContours(contour_img, self.debug_contours, -1, (0, 255, 0), 2)
            debug_images['contours'] = contour_img

        return debug_images


class CameraThread(QThread):
    """Thread camera cải tiến với tích hợp phát hiện vật thể"""

    frame_ready = pyqtSignal(QImage)  # Ảnh raw
    frame_with_detection = pyqtSignal(QImage)  # Ảnh có vẽ detection
    detection_ready = pyqtSignal(object)  # Vật thể phát hiện được
    log_signal = pyqtSignal(str)

    def __init__(self, cam_index=0, enable_detection=True):
        super().__init__()
        self.cam_index = cam_index
        self._running = False
        self.cap = None
        self.latest_frame = None  # Ảnh BGR (numpy array)
        self.latest_detection = None  # Vật thể phát hiện gần nhất
        self.enable_detection = enable_detection

        # Bộ phát hiện vật thể
        self.detector = ObjectDetector(
            min_area=100,
            max_area=50000,
            min_aspect_ratio=0.3,
            max_aspect_ratio=3.0,
            show_debug=False
        )
        # Mặc định phát hiện màu đỏ
        self.detector.add_red_color()

        # Cài đặt xử lý frame
        self.process_every_n_frames = 3  # Xử lý detection mỗi N frame
        self.frame_counter = 0
        self.show_detection = True  # Hiển thị kết quả detection trên ảnh

    def run(self):
        """Vòng lặp chính của thread camera"""
        self.cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)

        if not self.cap.isOpened():
            self.log_signal.emit(f"[LOI] Khong mo duoc camera index {self.cam_index}")
            return

        # Thiết lập độ phân giải
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self._running = True
        self.log_signal.emit("[OK] Camera da khoi dong")

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            self.latest_frame = frame
            self.frame_counter += 1

            # Phát hiện vật thể (nếu bật và đến lượt xử lý)
            if self.enable_detection and self.frame_counter % self.process_every_n_frames == 0:
                try:
                    detection = self.detector.detect(frame, return_all=False)
                    if detection is not None:
                        self.latest_detection = detection
                        self.detection_ready.emit(detection)

                        # Log khi phát hiện vật thể mới (hạn chế spam)
                        if hasattr(self, '_last_detection_log') or True:
                            self.log_signal.emit(
                                f"[DETECT] {detection.color_name} tại "
                                f"({detection.center_x:.1f}, {detection.center_y:.1f}) - "
                                f"Diện tích: {detection.area:.0f}"
                            )
                            self._last_detection_log = True
                except Exception as e:
                    self.log_signal.emit(f"[ERROR] Lỗi phát hiện: {e}")

            # Gửi ảnh raw
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            self.frame_ready.emit(qimg.copy())

            # Gửi ảnh có vẽ detection
            if self.show_detection and self.latest_detection is not None:
                try:
                    # Vẽ detection lên frame
                    display_frame = frame.copy()
                    display_frame = self.detector.draw_detection(
                        display_frame, [self.latest_detection]
                    )
                    # Chuyển sang RGB và gửi
                    rgb_display = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                    h_d, w_d, ch_d = rgb_display.shape
                    qimg_display = QImage(rgb_display.data, w_d, h_d, ch_d * w_d,
                                          QImage.Format_RGB888)
                    self.frame_with_detection.emit(qimg_display.copy())
                except Exception as e:
                    self.log_signal.emit(f"[ERROR] Lỗi vẽ detection: {e}")
            else:
                # Nếu không có detection hoặc tắt hiển thị, gửi ảnh raw
                self.frame_with_detection.emit(qimg.copy())

            self.msleep(30)  # ~33 FPS

        self.cap.release()
        self.log_signal.emit("[INFO] Camera da dung")

    def stop(self):
        """Dừng thread camera"""
        self._running = False
        self.wait(3000)

    def set_detection_enabled(self, enabled: bool):
        """Bật/tắt chức năng phát hiện vật thể"""
        self.enable_detection = enabled

    def set_color_ranges(self, color_ranges: List[ColorRange]):
        """Thiết lập khoảng màu cần phát hiện"""
        self.detector.clear_color_ranges()
        for color_range in color_ranges:
            self.detector.add_color_range(color_range)

    def set_detection_params(self, **kwargs):
        """Thiết lập tham số phát hiện"""
        for key, value in kwargs.items():
            if hasattr(self.detector, key):
                setattr(self.detector, key, value)

    def get_latest_detection(self) -> Optional[DetectedObject]:
        """Lấy vật thể phát hiện gần nhất"""
        return self.latest_detection


def capture_single_frame(cam_index=0):
    """Chụp một frame đơn"""
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None


def detect_object_in_frame(frame: np.ndarray,
                           color_ranges: Optional[List[ColorRange]] = None) -> Optional[DetectedObject]:
    """
    Phát hiện vật thể trong một frame

    Args:
        frame: Ảnh BGR
        color_ranges: Danh sách khoảng màu (mặc định: màu đỏ)

    Returns:
        DetectedObject hoặc None
    """
    if frame is None:
        return None

    detector = ObjectDetector()
    if color_ranges:
        for cr in color_ranges:
            detector.add_color_range(cr)
    else:
        detector.add_red_color()

    return detector.detect(frame)


# =========================================================================
#  HÀM TIỆN ÍCH CHO MAIN GUI
# =========================================================================

def get_color_presets() -> Dict[str, List[ColorRange]]:
    """Trả về các preset màu sắc"""
    return ColorPresets.get_all_presets()


def create_color_range_from_hsv(lower: Tuple[int, int, int],
                                upper: Tuple[int, int, int],
                                name: str = "") -> ColorRange:
    """Tạo ColorRange từ giá trị HSV"""
    return ColorRange(lower, upper, name)


def red_color_ranges() -> List[ColorRange]:
    """Trả về khoảng màu đỏ"""
    return [ColorPresets.RED_LOW, ColorPresets.RED_HIGH]


# =========================================================================
#  DEMO / TEST
# =========================================================================

def demo_camera_with_detection():
    """Demo camera với phát hiện vật thể"""
    import time

    # Tạo thread camera
    camera = CameraThread(cam_index=0, enable_detection=True)

    # Kết nối signal
    def on_detection(detected_obj):
        print(f"[DEMO] Phát hiện: {detected_obj.color_name} tại "
              f"({detected_obj.center_x:.1f}, {detected_obj.center_y:.1f})")

    camera.detection_ready.connect(on_detection)

    # Khởi động camera
    camera.start()
    print("[DEMO] Camera đã khởi động. Nhấn Ctrl+C để dừng.")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[DEMO] Đang dừng camera...")

    camera.stop()
    print("[DEMO] Đã dừng camera")


def demo_detect_red_object():
    """Demo phát hiện vật thể màu đỏ từ ảnh tĩnh"""
    import sys
    import os

    print("[DEMO] Phát hiện vật thể màu đỏ")
    print("Sử dụng webcam để chụp ảnh...")

    # Chụp ảnh từ webcam
    frame = capture_single_frame(0)
    if frame is None:
        print("[ERROR] Không thể chụp ảnh từ webcam")
        return

    # Phát hiện
    detector = ObjectDetector()
    detector.add_red_color()

    objects = detector.detect(frame, return_all=True)

    if objects:
        print(f"\nPhát hiện {len(objects)} vật thể:")
        for i, obj in enumerate(objects):
            print(f"  {i + 1}. {obj.color_name} tại ({obj.center_x:.1f}, {obj.center_y:.1f})")
            print(f"     Kích thước: {obj.width:.1f}x{obj.height:.1f}, Diện tích: {obj.area:.0f}")
    else:
        print("Không phát hiện vật thể nào")

    # Vẽ kết quả
    result_img = detector.draw_detection(frame, objects if objects else [])

    # Hiển thị ảnh
    cv2.imshow("Detection Result", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Chạy demo
    print("Chọn chế độ demo:")
    print("1. Camera real-time với phát hiện")
    print("2. Phát hiện từ ảnh tĩnh")

    choice = input("Nhập lựa chọn (1/2): ")

    if choice == "1":
        demo_camera_with_detection()
    elif choice == "2":
        demo_detect_red_object()
    else:
        print("Lựa chọn không hợp lệ")