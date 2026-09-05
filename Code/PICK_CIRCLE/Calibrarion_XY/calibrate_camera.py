import argparse
import glob
import os
import sys
import cv2
import numpy as np
def find_corners_auto(gray, board_w, board_h):
    """Thu ca (board_w, board_h) va (board_w-1, board_h-1) de tu do phat hien
    dung so goc trong, phong truong hop nguoi dung nhap so o vuong thay vi
    so goc trong."""
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    candidates = [(board_w, board_h), (board_w - 1, board_h - 1)]
    for size in candidates:
        if size[0] < 2 or size[1] < 2:
            continue
        ok, corners = cv2.findChessboardCorners(gray, size, flags)
        if ok:
            return ok, corners, size
    return False, None, None


def main():
    parser = argparse.ArgumentParser(description="Calib camera bang ban co")
    parser.add_argument("--images_dir", type=str, default="images", help="Thu muc chua anh ban co (mac dinh: 'images')")
    parser.add_argument("--board_w", type=int, default=9, help="So goc trong / o vuong chieu ngang (mac dinh 9)")
    parser.add_argument("--board_h", type=int, default=7, help="So goc trong / o vuong chieu doc (mac dinh 7)")
    parser.add_argument("--square_size", type=float, default=9.96, help="Kich thuoc 1 o vuong tinh bang mm (mac dinh 10mm)")
    parser.add_argument("--output", type=str, default="calibration_result", help="Ten file output (khong can duoi)")
    parser.add_argument("--show", action="store_true", help="Hien thi anh phat hien goc de kiem tra truc quan")
    args = parser.parse_args()

    print(f"Dang tim anh trong thu muc: {os.path.abspath(args.images_dir)}")
    if not os.path.isdir(args.images_dir):
        print(f"LOI: Thu muc '{args.images_dir}' khong ton tai.")
        print("Hay chup anh truoc bang capture_images.py, hoac truyen dung duong dan bang --images_dir <duong_dan>")
        sys.exit(1)

    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    image_paths = []
    for ext in exts:
        image_paths.extend(glob.glob(os.path.join(args.images_dir, ext)))
    image_paths = sorted(image_paths)

    if len(image_paths) < 5:
        print(f"Canh bao: chi tim thay {len(image_paths)} anh trong '{args.images_dir}'.")
        print("Nen co it nhat 10-20 anh o cac goc do/khoang cach khac nhau de calib chinh xac.")
        if len(image_paths) == 0:
            sys.exit(1)

    # Dem so lan phat hien theo tung kich thuoc de tu chon size pho bien nhat
    size_votes = {}
    detections = []  # list of (path, gray_shape, corners, size)

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"Bo qua (khong doc duoc): {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners, size = find_corners_auto(gray, args.board_w, args.board_h)
        if ok:
            size_votes[size] = size_votes.get(size, 0) + 1
            detections.append((path, gray.shape[::-1], corners, size, img))
        else:
            print(f"Khong phat hien duoc ban co trong: {path}")

    if not detections:
        print("Khong phat hien duoc ban co trong bat ky anh nao. Kiem tra lai:")
        print(" - Kich thuoc board_w/board_h co dung khong")
        print(" - Anh co ro net, du sang, thay day du bang co khong")
        sys.exit(1)

    # Chon kich thuoc pho bien nhat
    best_size = max(size_votes, key=size_votes.get)
    print(f"\nKich thuoc goc trong duoc su dung: {best_size[0]} x {best_size[1]}")
    print(f"(Phat hien thanh cong {size_votes[best_size]}/{len(image_paths)} anh voi kich thuoc nay)\n")

    board_w, board_h = best_size

    # Chuan bi object points (toa do 3D thuc te, Z=0) cho 1 goc nhin
    objp = np.zeros((board_h * board_w, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []  # diem 3D trong khong gian thuc
    imgpoints = []  # diem 2D tuong ung tren anh
    image_size = None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    used_count = 0
    for path, shape, corners, size, img in detections:
        if size != best_size:
            continue  # bo qua anh phat hien sai kich thuoc de dam bao dong nhat
        image_size = shape
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria
        )
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        used_count += 1

        if args.show:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, size, corners_refined, True)
            cv2.imshow("corners", vis)
            cv2.waitKey(200)

    if args.show:
        cv2.destroyAllWindows()

    print(f"Su dung {used_count} anh de calib.\n")

    if used_count < 5:
        print("Canh bao: so anh hop le qua it (<5), ket qua calib co the khong chinh xac.")

    # ---- Calib camera ----
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    # ---- Tinh reprojection error ----
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(objpoints)

    print("===== KET QUA CALIB =====")
    print(f"RMS reprojection error (OpenCV): {ret:.4f}")
    print(f"Mean reprojection error (tinh tay): {mean_error:.4f} pixel")
    print("\nCamera matrix (ma tran noi tai):")
    print(camera_matrix)
    print("\nDistortion coefficients [k1, k2, p1, p2, k3]:")
    print(dist_coeffs.ravel())

    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    print(f"\nfx={fx:.2f}  fy={fy:.2f}  cx={cx:.2f}  cy={cy:.2f}")

    if mean_error < 0.5:
        print("\nDanh gia: Calib TOT (sai so < 0.5 pixel)")
    elif mean_error < 1.0:
        print("\nDanh gia: Calib CHAP NHAN DUOC (sai so < 1.0 pixel)")
    else:
        print("\nDanh gia: Sai so cao (>1.0 pixel), nen chup them anh o nhieu goc do/khoang cach hon.")

    # ---- Luu ket qua ----
    npz_path = f"{args.output}.npz"
    np.savez(
        npz_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        mean_error=mean_error,
        board_w=board_w,
        board_h=board_h,
        square_size=args.square_size,
    )
    print(f"\nDa luu ket qua (numpy) vao: {npz_path}")

    yaml_path = f"{args.output}.yaml"
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("mean_reprojection_error", mean_error)
    fs.release()
    print(f"Da luu ket qua (OpenCV YAML) vao: {yaml_path}")

    print("\nDe dung ket qua nay undistort anh/video sau nay, tham khao file undistort_demo.py")


if __name__ == "__main__":
    main()