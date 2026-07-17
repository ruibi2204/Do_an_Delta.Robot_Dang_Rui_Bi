import argparse
import csv
import os
import sys

import cv2
import numpy as np


def find_corners_auto(gray, board_w, board_h):
    """Thu ca (board_w, board_h) va (board_w-1, board_h-1) de tu dong phat hien
    dung so goc trong, phong truong hop nguoi dung nhap so o vuong thay vi
    so goc trong."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    candidates = [(board_w, board_h), (board_w - 1, board_h - 1)]
    for size in candidates:
        if size[0] < 2 or size[1] < 2:
            continue
        ok, corners = cv2.findChessboardCorners(gray, size, flags)
        if ok:
            return ok, corners, size
    return False, None, None


def undistort_image(img, camera_matrix, dist_coeffs, crop=False):
    """Khu meo 1 anh. Tra ve (anh_da_khu_meo, camera_matrix_moi).

    MAC DINH KHONG CROP (crop=False): giu nguyen kich thuoc khung hinh goc
    (vi du 640x480) sau khi khu meo. Ly do: neu crop theo ROI, vung ROI
    thuong bi cat LECH (khong doi xung), khien tam anh sau crop KHONG con
    trung voi tam quang hoc that cua camera -> sai lech goc toa do (0,0).
    Giu nguyen kich thuoc goc dam bao tam anh (w/2, h/2) luon on dinh."""
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    if crop:
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undistorted = undistorted[y:y + rh, x:x + rw]

    return undistorted, new_camera_matrix


def detect_corners_pixel(img, board_w, board_h):
    """Tim toa do pixel cac goc ban co tren anh (da khu meo).
    Truoc khi tim, tien hanh nhi phan hoa anh xam de tang do tuong phan.
    Tra ve (found, corners_refined, used_size)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Nhi phan hoa anh xam bang phuong phap Otsu
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Tim goc tren anh nhi phan
    ok, corners, size = find_corners_auto(binary, board_w, board_h)
    if not ok:
        return False, None, None

    # Tinh chinh lai goc tren anh xam goc de dat do chinh xac cao
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners_refined, size


def compute_mm_coords(corners, size, square_size, image_shape):
    """Tinh toa do THUC (mm) cua moi goc ban co, lay goc (0,0) la PIXEL
    TRUNG TAM cua khung hinh, dung phep bien doi homography.

    **LUU Y**: Truc x duoc dao chieu (nhan -1) de phu hop voi he quy chieu
    cua robot (x dương sang trai hoac phai tuy thiet lap).

    Tra ve: world_mm (Nx2 array, mm, da tru di tam), H (ma tran homography
    pixel->mm tren mat phang ban co), world_center (toa do mm cua tam anh
    truoc khi tru, chi de tham khao/debug)."""
    board_w, board_h = size
    h, w = image_shape[:2]

    # Toa do mm LY THUYET cua tung goc, dua vao vi tri luoi (col, row) va
    # kich thuoc that cua 1 o vuong. Truc y lay dau am de sau nay tuong ung
    # chieu "duong huong len" (giong quy uoc truc y cua toa do pixel da dung).
    # Truc x duoc dao chieu (nhan -1) theo yeu cau.
    world_pts = np.zeros((len(corners), 2), dtype=np.float32)
    for idx in range(len(corners)):
        col = idx % board_w
        row = idx // board_w
        world_pts[idx, 0] = -col * square_size      # DAO CHIEU TRUC X
        world_pts[idx, 1] = -row * square_size

    image_pts = corners.reshape(-1, 2).astype(np.float32)

    # H: phep bien doi phoi canh pixel -> mm tren mat phang ban co.
    H, _ = cv2.findHomography(image_pts, world_pts, method=0)

    # Tim toa do mm cua pixel TRUNG TAM khung hinh tren mat phang ban co.
    cx, cy = w / 2.0, h / 2.0
    center_px = np.array([[[cx, cy]]], dtype=np.float32)
    world_center = cv2.perspectiveTransform(center_px, H)[0][0]

    # Toa do mm cuoi cung = toa do ly thuyet - toa do mm cua tam anh.
    world_mm = world_pts - world_center

    return world_mm, H, world_center


def save_corners_csv(path, corners, size, image_shape, square_size):
    """Luu toa do cac goc ra file CSV: pixel goc, pixel da doi ve tam anh,
    va toa do THUC (mm) voi goc la pixel trung tam (dung homography)."""
    board_w, board_h = size
    h, w = image_shape[:2]
    cx, cy = w / 2.0, h / 2.0

    world_mm, H, world_center = compute_mm_coords(corners, size, square_size, image_shape)

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["corner_index", "col", "row", "pixel_x", "pixel_y",
                          "x_px", "y_px", "x_mm", "y_mm"])
        for idx, pt in enumerate(corners):
            col = idx % board_w
            row = idx // board_w
            px, py = pt.ravel()
            x_px = px - cx
            y_px = cy - py
            x_mm, y_mm = world_mm[idx]
            writer.writerow([idx, col, row, f"{px:.3f}", f"{py:.3f}",
                              f"{x_px:.3f}", f"{y_px:.3f}",
                              f"{x_mm:.3f}", f"{y_mm:.3f}"])
    print(f"Da luu toa do vao: {path}")
    print(f"(Goc mm cua tam anh truoc khi tru, chi de tham khao: "
          f"world_center = ({world_center[0]:.3f}, {world_center[1]:.3f}) mm)")

    return H


def annotate_corners(img, corners, size, world_mm=None):
    """Ve cac goc len anh kem so thu tu (va toa do mm neu co), ve truc toa do
    tam anh de kiem tra truc quan."""
    vis = img.copy()
    h, w = vis.shape[:2]
    cx, cy = w // 2, h // 2

    cv2.line(vis, (0, cy), (w, cy), (0, 0, 255), 1, cv2.LINE_AA)
    cv2.line(vis, (cx, 0), (cx, h), (255, 0, 0), 1, cv2.LINE_AA)
    cv2.arrowedLine(vis, (cx, cy), (min(cx + 40, w - 1), cy), (0, 0, 255), 1, cv2.LINE_AA, tipLength=0.3)
    cv2.arrowedLine(vis, (cx, cy), (cx, max(cy - 40, 0)), (255, 0, 0), 1, cv2.LINE_AA, tipLength=0.3)
    cv2.circle(vis, (cx, cy), 4, (0, 255, 255), -1)
    cv2.putText(vis, "O(0,0)", (cx + 6, cy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    cv2.drawChessboardCorners(vis, size, corners, True)
    for idx, pt in enumerate(corners):
        x, y = pt.ravel()
        cv2.putText(
            vis, str(idx), (int(x) + 4, int(y) - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA
        )
    return vis


def process_single_image(img, camera_matrix, dist_coeffs, board_w, board_h, square_size,
                          out_undistorted, out_csv, out_annotated):
    undistorted, new_camera_matrix = undistort_image(img, camera_matrix, dist_coeffs)
    cv2.imwrite(out_undistorted, undistorted)
    print(f"Da luu anh da khu meo vao: {out_undistorted}")

    found, corners, size = detect_corners_pixel(undistorted, board_w, board_h)
    if not found:
        print("Khong phat hien duoc ban co tren anh da khu meo.")
        return

    print(f"Phat hien ban co kich thuoc {size[0]} x {size[1]} goc trong, tong {len(corners)} diem.")
    save_corners_csv(out_csv, corners, size, undistorted.shape, square_size)

    vis = annotate_corners(undistorted, corners, size)
    cv2.imwrite(out_annotated, vis)
    print(f"Da luu anh co danh so goc vao: {out_annotated}")


def run_webcam(camera_index, camera_matrix, dist_coeffs, board_w, board_h, square_size, out_prefix):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Khong mo duoc camera index {camera_index}")
        return

    print("Nhan SPACE/'s' de chup + tim goc ban co + luu ket qua, 'q'/ESC de thoat.")
    shot_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame tu camera.")
            break

        undistorted, _ = undistort_image(frame, camera_matrix, dist_coeffs)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        # Nhi phan hoa de tim goc
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        found, corners, size = find_corners_auto(binary, board_w, board_h)

        display = undistorted.copy()
        if found:
            cv2.drawChessboardCorners(display, size, corners, found)
            cv2.putText(display, f"Da tim thay ban co: {size[0]}x{size[1]}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Chua tim thay ban co", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Da luu: {shot_count} lan", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("Khu meo + tim goc ban co (undistorted)", display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key in (ord("s"), 32):
            if not found:
                print("Khong the luu: chua thay ban co trong khung hinh.")
                continue
            # Tinh chinh lai goc tren anh xam goc
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            out_undistorted = f"{out_prefix}_{shot_count:03d}.jpg"
            out_csv = f"{out_prefix}_{shot_count:03d}_corners.csv"
            out_annotated = f"{out_prefix}_{shot_count:03d}_annotated.jpg"

            cv2.imwrite(out_undistorted, undistorted)
            save_corners_csv(out_csv, corners_refined, size, undistorted.shape, square_size)
            vis = annotate_corners(undistorted, corners_refined, size)
            cv2.imwrite(out_annotated, vis)

            print(f"Da luu: {out_undistorted}, {out_csv}, {out_annotated}")
            shot_count += 1

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Khu meo anh + xac dinh toa do THUC (mm) cac goc ban co, goc la tam anh")
    parser.add_argument("--calib", type=str, default="calibration_result.npz",
                         help="File .npz ket qua calib (mac dinh: calibration_result.npz)")
    parser.add_argument("--image", type=str, default=None, help="Duong dan anh dau vao (neu khong dung webcam)")
    parser.add_argument("--camera", type=int, default=0, help="Chi so webcam (mac dinh 0), dung khi khong truyen --image")
    parser.add_argument("--board_w", type=int, default=9, help="So goc trong / o vuong chieu ngang cua ban co (mac dinh 9)")
    parser.add_argument("--board_h", type=int, default=7, help="So goc trong / o vuong chieu doc cua ban co (mac dinh 7)")
    parser.add_argument("--square_size", type=float, default=None,
                         help="Kich thuoc 1 o vuong (mm). Neu khong truyen, se tu doc tu file .npz (neu co), "
                              "neu khong co trong .npz thi mac dinh 10mm")
    parser.add_argument("--out_prefix", type=str, default="result",
                         help="Tien to ten file ket qua (mac dinh: 'result')")
    args = parser.parse_args()

    if not os.path.isfile(args.calib):
        print(f"LOI: Khong tim thay file calib '{args.calib}'.")
        print("Hay chay calibrate_camera.py truoc de tao ra file nay, hoac truyen dung duong dan bang --calib <duong_dan>")
        sys.exit(1)

    data = np.load(args.calib)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    print(f"Da doc file calib: {args.calib}")

    # Xac dinh square_size (mm): uu tien tham so dong lenh, sau do lay tu npz,
    # cuoi cung fallback 10mm neu khong co thong tin nao.
    if args.square_size is not None:
        square_size = args.square_size
        print(f"Su dung square_size tu tham so dong lenh: {square_size} mm")
    elif "square_size" in data:
        square_size = float(data["square_size"])
        print(f"Su dung square_size doc tu file calib: {square_size} mm")
    else:
        square_size = 10.0
        print(f"CANH BAO: khong tim thay square_size trong file calib, dung mac dinh {square_size} mm. "
              f"Neu sai, hay truyen --square_size <gia_tri_dung>")

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"Khong doc duoc anh: {args.image}")
            sys.exit(1)

        out_undistorted = f"{args.out_prefix}_undistorted.jpg"
        out_csv = f"{args.out_prefix}_corners.csv"
        out_annotated = f"{args.out_prefix}_annotated.jpg"

        process_single_image(
            img, camera_matrix, dist_coeffs, args.board_w, args.board_h, square_size,
            out_undistorted, out_csv, out_annotated
        )
    else:
        run_webcam(args.camera, camera_matrix, dist_coeffs, args.board_w, args.board_h, square_size, args.out_prefix)


if __name__ == "__main__":
    main()