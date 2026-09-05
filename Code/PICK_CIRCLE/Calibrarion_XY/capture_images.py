import argparse
import os
import cv2


def try_find(gray, board_w, board_h):
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    return cv2.findChessboardCorners(gray, (board_w, board_h), flags)

def main():
    parser = argparse.ArgumentParser(description="Chup anh ban co de calib camera")
    parser.add_argument("--board_w", type=int, default=9, help="So goc trong theo chieu ngang")
    parser.add_argument("--board_h", type=int, default=7, help="So goc trong theo chieu doc")
    parser.add_argument("--out_dir", type=str, default="images", help="Thu muc luu anh")
    parser.add_argument("--camera", type=int, default=0, help="Chi so camera (0 la mac dinh)")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Khong mo duoc camera index {args.camera}")
        return

    # Hai cach hieu kich thuoc bo cach (9x7 o vuong -> 8x6 goc trong)
    candidates = [
        (args.board_w, args.board_h),
        (args.board_w - 1, args.board_h - 1),
    ]

    count = 0
    print("Nhan SPACE/'s' de luu anh, 'q'/ESC de thoat.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame tu camera.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found = False
        used_size = None
        corners = None
        for size in candidates:
            ok, c = try_find(gray, size[0], size[1])
            if ok:
                found = True
                used_size = size
                corners = c
                break

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, used_size, corners, found)
            cv2.putText(
                display,
                f"Da tim thay ban co: {used_size[0]}x{used_size[1]} goc trong",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                display,
                "Chua tim thay ban co - dieu chinh goc do/anh sang",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        cv2.putText(
            display,
            f"Da luu: {count} anh",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        cv2.imshow("Capture calibration images", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):  # q hoac ESC
            break
        elif key in (ord("s"), 32):  # s hoac space
            if found:
                filename = os.path.join(args.out_dir, f"calib_{count:03d}.png")
                cv2.imwrite(filename, frame)
                print(f"Da luu: {filename}")
                count += 1
            else:
                print("Khong the luu: chua thay bang cach ro trong khung hinh.")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Hoan tat. Tong cong da chup {count} anh vao thu muc '{args.out_dir}'.")


if __name__ == "__main__":
    main()
