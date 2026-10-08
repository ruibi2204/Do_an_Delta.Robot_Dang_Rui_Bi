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
    parser.add_argument("--square_size", type=float, default=9.96, help="Kich thuoc 1 o vuong tinh bang mm (mac dinh 9.96mm)")
    parser.add_argument("--output", type=str, default="calibration_result", help="Ten file output (khong can duoi)")
    parser.add_argument("--show", action="store_true", help="Hien thi anh phat hien goc de kiem tra truc quan")
    parser.add_argument("--ref_image", type=str, default=None,
                        help="Anh ban co dat dung tren mat mam (anh chuan). "
                             "Neu co, se tinh them R, T cua anh nay bang solvePnP")
    parser.add_argument("--fix_k3", action="store_true",
                        help="Co dinh k3 = 0 khi calib (dung khi he so meo k2, k3 qua lon / nghi bi overfit)")
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
    detections = []  # list of (path, gray_shape, corners, size, img)

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
    used_paths = []  # duong dan cac anh THUC SU dung de calib (khop thu tu rvecs/tvecs)
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
        used_paths.append(path)
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
    calib_flags = cv2.CALIB_FIX_K3 if args.fix_k3 else 0
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=calib_flags
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

    # ---- R, T cho TUNG anh calib (lay tu rvecs, tvecs) ----
    R_list, T_list = [], []
    print("===== MA TRAN NGOAI (R, T) THEO TUNG ANH =====")
    for path, rvec, tvec in zip(used_paths, rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)          # R: 3x3
        T = tvec.reshape(3, 1)              # T: 3x1 (mm)
        R_list.append(R)
        T_list.append(T)
        print(f"--- {os.path.basename(path)} ---")
        print("R =\n", np.round(R, 5))
        print("T (mm) =", np.round(T.ravel(), 4))

    # ---- R, T cua ANH CHUAN (ban co nam tren mat mam) ----
    R_ref = T_ref = None
    if args.ref_image:
        ref_img = cv2.imread(args.ref_image)
        if ref_img is None:
            print(f"\nLOI: khong doc duoc anh chuan '{args.ref_image}'")
        else:
            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            ok, ref_corners, ref_size = find_corners_auto(
                ref_gray, args.board_w, args.board_h
            )
            if not ok or ref_size != best_size:
                print("\nLOI: khong tim thay ban co (dung kich thuoc) trong anh chuan.")
            else:
                ref_corners = cv2.cornerSubPix(
                    ref_gray, ref_corners, (11, 11), (-1, -1), criteria
                )
                okp, rvec_ref, tvec_ref = cv2.solvePnP(
                    objp, ref_corners, camera_matrix, dist_coeffs
                )
                if okp:
                    R_ref, _ = cv2.Rodrigues(rvec_ref)
                    T_ref = tvec_ref.reshape(3, 1)
                    print("\n===== R, T CUA ANH CHUAN (MAT MAM) =====")
                    print("R =\n", np.round(R_ref, 5))
                    print("T (mm) =", np.round(T_ref.ravel(), 4))
                else:
                    print("\nLOI: solvePnP that bai tren anh chuan.")

    print("\n===== KET QUA CALIB =====")
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
    save_dict = dict(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        mean_error=mean_error,
        board_w=board_w,
        board_h=board_h,
        square_size=args.square_size,
        R_all=np.array(R_list),   # (N, 3, 3)
        T_all=np.array(T_list),   # (N, 3, 1)
    )
    if R_ref is not None:
        save_dict["R_ref"] = R_ref
        save_dict["T_ref"] = T_ref
    np.savez(npz_path, **save_dict)
    print(f"\nDa luu ket qua (numpy) vao: {npz_path}")

    yaml_path = f"{args.output}.yaml"
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("mean_reprojection_error", mean_error)
    if R_ref is not None:
        fs.write("R_ref", R_ref)
        fs.write("T_ref", T_ref)
    fs.release()
    print(f"Da luu ket qua (OpenCV YAML) vao: {yaml_path}")

    print("\nDe dung ket qua nay undistort anh/video sau nay, tham khao file undistort_demo.py")


if __name__ == "__main__":
    main()