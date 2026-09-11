"""
camera_center_dot.py
---------------------
Mở camera và hiển thị một CHẤM MÀU CAM cố định tại chính giữa khung hình,
đánh dấu tọa độ (0, 0) - dùng để canh chỉnh tâm bàn xoay ngay dưới ống kính.

Cách dùng:
    python camera_center_dot.py                # mở camera mặc định (index 0)
    python camera_center_dot.py --index 1       # chọn camera khác
    python camera_center_dot.py --width 1280 --height 720
    python camera_center_dot.py --dot-radius 8  # chỉnh cỡ chấm

Phím tắt trong cửa sổ hiển thị:
    q hoặc ESC  -> thoát
    c           -> ẩn/hiện đường chữ thập (crosshair) phụ trợ
"""

import argparse
import sys

import cv2

ORANGE_BGR = (0, 140, 255)  # OpenCV dùng BGR: cam đậm, dễ nhìn trên nền bất kỳ
WHITE = (255, 255, 255)

# Bảng chọn backend camera. "auto" -> dùng DSHOW trên Windows (mở nhanh, ổn
# định hơn MSMF mặc định), để mặc định (0) trên các hệ điều hành khác.
BACKEND_MAP = {
    "auto": None,  # xử lý riêng trong main()
    "dshow": cv2.CAP_DSHOW,
    "msmf": getattr(cv2, "CAP_MSMF", 0),
    "default": cv2.CAP_ANY,
}

# Giảm độ sáng khung hình hiển thị (áp dụng bằng phần mềm nên luôn có tác
# dụng, không phụ thuộc camera có hỗ trợ CAP_PROP_BRIGHTNESS hay không).
# Mỗi "mức" giảm BRIGHTNESS_STEP; ở đây giảm sẵn 2 mức. Chỉnh trực tiếp ở đây.
BRIGHTNESS_STEP = 0.15
BRIGHTNESS_LEVELS_DOWN = 2
BRIGHTNESS_SCALE = 1.0 - (BRIGHTNESS_STEP * BRIGHTNESS_LEVELS_DOWN)  # = 0.7

# Cửa sổ hiển thị camera được thu nhỏ lại còn tỉ lệ này so với độ phân giải
# khung hình thật (ảnh gửi vẽ chấm vẫn giữ nguyên, chỉ cửa sổ hiển thị nhỏ
# hơn cho gọn màn hình). Chỉnh trực tiếp ở đây.
WINDOW_SCALE = 0.6


def parse_args():
    p = argparse.ArgumentParser(description="Hiển thị chấm cam tại tâm khung hình camera.")
    p.add_argument("--index", type=int, default=0, help="Chỉ số camera (mặc định 0).")
    p.add_argument("--width", type=int, default=720, help="Độ rộng khung hình mong muốn.")
    p.add_argument("--height", type=int, default=480, help="Độ cao khung hình mong muốn.")
    p.add_argument("--dot-radius", type=int, default=6, help="Bán kính chấm cam (px).")
    p.add_argument("--no-crosshair", action="store_true", help="Không vẽ đường chữ thập phụ trợ lúc khởi động.")
    p.add_argument(
        "--backend", choices=list(BACKEND_MAP.keys()), default="auto",
        help="Backend mở camera. 'auto' (mặc định) dùng DirectShow (dshow) trên Windows, "
             "backend hệ thống mặc định trên các OS khác.",
    )
    return p.parse_args()


def open_camera(index, backend_name):
    if backend_name == "auto":
        backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    else:
        backend = BACKEND_MAP[backend_name]
    return cv2.VideoCapture(index, backend)


def main():
    args = parse_args()

    cap = open_camera(args.index, args.backend)
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"[LỖI] Không mở được camera index={args.index}.", file=sys.stderr)
        sys.exit(1)

    show_crosshair = not args.no_crosshair
    window_name = "Canh tam ban xoay - Camera Center (0,0)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    window_sized = False  # chỉ resize cửa sổ 1 lần, sau khi biết kích thước khung hình thật

    print("Đang chạy... Nhấn 'q' hoặc ESC để thoát, 'c' để bật/tắt chữ thập.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[LỖI] Không đọc được khung hình từ camera.", file=sys.stderr)
            break

        if BRIGHTNESS_SCALE != 1.0:
            frame = cv2.convertScaleAbs(frame, alpha=BRIGHTNESS_SCALE, beta=0)

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        if not window_sized:
            cv2.resizeWindow(window_name, int(w * WINDOW_SCALE), int(h * WINDOW_SCALE))
            window_sized = True

        if show_crosshair:
            line_len = max(15, args.dot_radius * 3)
            cv2.line(frame, (cx - line_len, cy), (cx + line_len, cy), ORANGE_BGR, 1)
            cv2.line(frame, (cx, cy - line_len), (cx, cy + line_len), ORANGE_BGR, 1)

        # Chấm cam tại tâm - đánh dấu tọa độ (0, 0)
        cv2.circle(frame, (cx, cy), args.dot_radius, ORANGE_BGR, thickness=-1)
        cv2.circle(frame, (cx, cy), args.dot_radius + 1, WHITE, thickness=1)  # viền trắng cho dễ thấy

        cv2.putText(
            frame, "(0, 0)", (cx + args.dot_radius + 6, cy - args.dot_radius - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA,
        )
        cv2.putText(
            frame, "(0, 0)", (cx + args.dot_radius + 6, cy - args.dot_radius - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, ORANGE_BGR, 1, cv2.LINE_AA,
        )

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # 'q' hoặc ESC
            break
        elif key == ord("c"):
            show_crosshair = not show_crosshair

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()