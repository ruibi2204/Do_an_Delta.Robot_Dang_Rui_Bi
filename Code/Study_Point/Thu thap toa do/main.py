import argparse
import csv
import os
import sys

import cv2
import numpy as np


def find_corners_auto(gray_or_binary, board_w, board_h, flags):
    """Thu ca (board_w, board_h) va (board_w-1, board_h-1) de tu dong phat hien
    dung so goc trong, phong truong hop nguoi dung nhap so o vuong thay vi
    so goc trong. Dung cho phuong phap findChessboardCorners CO DIEN."""
    candidates = [(board_w, board_h), (board_w - 1, board_h - 1)]
    for size in candidates:
        if size[0] < 2 or size[1] < 2:
            continue
        ok, corners = cv2.findChessboardCorners(gray_or_binary, size, flags)
        if ok:
            return True, corners, size
    return False, None, None


def detect_corners_raw_subpix(img, board_w, board_h):
    """Phat hien goc ban co voi do chinh xac sub-pixel CAO NHAT co the,
    thuc hien tren ANH GOC (CHUA khu meo).

    QUAN TRONG VE DO CHINH XAC: co tinh KHONG khu meo ca anh roi moi tim
    goc, vi buoc undistort() phai noi suy (resample) pixel, lam "mo" nhe
    vi tri that cua goc truoc khi do. Tim goc tren pixel GOC (chua bi noi
    suy lai) cho ket qua sub-pixel chinh xac hon. Viec khu meo se duoc ap
    dung SAU, chi cho toa do diem (xem undistort_corner_points), khong
    anh huong den buoc phat hien nay.

    Uu tien dung cv2.findChessboardCornersSB (thuat toan moi hon, chinh
    xac va on dinh hon nhieu so voi findChessboardCorners co dien, dac
    biet voi anh hoi mo hoac anh sang khong deu). Neu khong kha dung hoac
    that bai, fallback ve phuong phap co dien (Otsu + cornerSubPix).

    Tra ve: (found, corners (N,1,2) float64, size (board_w,board_h) da dung)
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    candidates = [(board_w, board_h), (board_w - 1, board_h - 1)]

    # ---- Uu tien: findChessboardCornersSB ----
    if hasattr(cv2, "findChessboardCornersSB"):
        sb_flags = (cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
                    + cv2.CALIB_CB_NORMALIZE_IMAGE)
        for size in candidates:
            if size[0] < 2 or size[1] < 2:
                continue
            ok, corners = cv2.findChessboardCornersSB(gray, size, flags=sb_flags)
            if ok:
                return True, corners.reshape(-1, 1, 2).astype(np.float64), size

    # ---- Fallback: phuong phap co dien (Otsu + adaptive) + cornerSubPix ----
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    classic_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                      + cv2.CALIB_CB_FILTER_QUADS)
    found, corners, size = find_corners_auto(binary, board_w, board_h, classic_flags)
    if not found:
        return False, None, None

    # Tieu chi hoi tu chat hon de dat do chinh xac sub-pixel cao hon
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-4)
    corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners_refined.reshape(-1, 1, 2).astype(np.float64), size


def undistort_corner_points(corners_raw, camera_matrix, dist_coeffs, new_camera_matrix):
    """Khu meo CHO TOA DO DIEM (khong resample anh) - chinh xac hon nhieu
    so voi khu meo ca anh roi tim goc lai tren anh da bi noi suy.
    Tra ve corners cung dinh dang (N,1,2) nhu findChessboardCorners, nam
    trong he pixel cua new_camera_matrix (tuc he pixel cua anh da khu meo
    dung de hien thi/luu)."""
    pts = corners_raw.reshape(-1, 1, 2).astype(np.float64)
    undist = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=new_camera_matrix)
    return undist.astype(np.float32)


def undistort_image(img, camera_matrix, dist_coeffs, crop=False):
    """Khu meo 1 anh (chi de HIEN THI / LUU FILE anh, khong dung de tim
    goc). Tra ve (anh_da_khu_meo, camera_matrix_moi).

    MAC DINH KHONG CROP (crop=False): giu nguyen kich thuoc khung hinh goc
    (vi du 640x480) sau khi khu meo. Ly do: neu crop theo ROI, vung ROI
    thuong bi cat LECH (khong doi xung), khien tam anh sau crop KHONG con
    trung voi tam quang hoc that cua camera -> sai lech goc toa do (0,0).
    Giu nguyen kich thuoc goc dam bao new_camera_matrix (cx, cy) - tam quang
    hoc THAT su cua he thong sau khu meo - khong bi xe dich."""
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    # INTER_CUBIC cho chat luong noi suy anh tot hon INTER_LINEAR mac dinh
    # (chi anh huong chat luong hinh anh hien thi/luu, khong anh huong toa
    # do vi toa do da duoc tinh rieng qua undistort_corner_points).
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1
    )
    undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC)

    if crop:
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undistorted = undistorted[y:y + rh, x:x + rw]

    return undistorted, new_camera_matrix


def build_object_points(size, square_size):
    """Toa do THUC (mm) LY THUYET cua tung goc trong HE TOA DO RIENG CUA
    BAN CO (mat phang Z=0), dung lam object points cho solvePnP.

    Quy uoc truc:
    - Truc X duong khi di TU PHAI SANG TRAI (theo yeu cau): vi thu tu goc
      duoc OpenCV danh so tang dan TU TRAI SANG PHAI theo cot (col), nen de
      X tang khi sang trai ta lay x = -col * square_size.
    - Truc Y duong khi di TU DUOI LEN TREN (theo yeu cau): vi row tang dan
      TU TREN XUONG DUOI trong anh, nen de Y tang khi len tren ta lay
      y = -row * square_size.

    Luu y: day chi la toa do NOI BO cua mat phang ban co (dung de solvePnP
    tim tu the R, t so voi camera). Gia tri cuoi cung tra ve nguoi dung se
    duoc doi ve goc la tam quang hoc that cua camera (xem compute_mm_coords).
    """
    board_w, board_h = size
    n = board_w * board_h
    objp = np.zeros((n, 3), dtype=np.float64)
    for idx in range(n):
        col = idx % board_w
        row = idx // board_w
        objp[idx, 0] = -col * square_size
        objp[idx, 1] = -row * square_size
        objp[idx, 2] = 0.0
    return objp


def compute_mm_coords(corners, size, square_size, new_camera_matrix):
    """Tinh toa do THUC (mm) cua moi goc ban co, theo HE TOA DO CO DINH
    THEO CAMERA/MAN HINH (khong xoay theo mat phang ban co).

    LUU Y: corners truyen vao day PHAI la toa do da duoc khu meo (qua
    undistort_corner_points), va new_camera_matrix la ma tran camera moi
    tuong ung (khong con he so meo, dist=0).

    Cach lam:
    1) Dung solvePnP tim tu the (R, t) cua ban co so voi camera, sau do
       tinh chinh lai bang solvePnPRefineLM (Levenberg-Marquardt) de tang
       do chinh xac tu the.
    2) Chuyen tung goc tu he ban co sang HE CAMERA (mm that, co goc la
       chinh tam quang hoc cua camera):
           P_cam = R @ P_board + t
       Day la toa do 3D THAT cua tung goc trong khong gian, tinh theo
       dung hinh hoc phoi canh (tieu cu, do sau, tu the nghieng that cua
       ban co) - khong con phu thuoc thuan tuy vao square_size nhu cach cu.
    3) Vi truc quang hoc (tia (0,0,s) xuat phat tu tam camera) luon co
       X_cam = Y_cam = 0 tai MOI do sau s, nen goc (0,0) cua he toa do nay
       TU NHIEN la diem ma truc quang hoc cat mat phang ban co - dung y
       nghia "toa do noi suy tu tam camera di ra" - ma KHONG CAN tinh toan
       giao diem rieng nhu truoc.
    4) He truc X_cam, Y_cam nay LUON SONG SONG VOI TRUC NGANG/DOC CUA ANH
       (co dinh theo man hinh camera), KHONG xoay theo do nghieng cua ban
       co trong khong gian - dung nhu yeu cau.
    5) Doi dau truc cho dung quy uoc: X DUONG khi di TU PHAI SANG TRAI
       (nguoc voi X_cam chuan cua OpenCV, von duong sang phai) va Y DUONG
       khi di TU DUOI LEN TREN (nguoc voi Y_cam chuan, von duong xuong
       duoi) -> x_out = -X_cam, y_out = -Y_cam.

    Tra ve: world_mm (Nx2, mm; X duong = phai->trai, Y duong = duoi->tren),
    (rvec, tvec) tu solvePnP (da refine).
    """
    objp = build_object_points(size, square_size)
    imgp = corners.reshape(-1, 2).astype(np.float64)

    # corners da duoc khu meo tu truoc (undistort_corner_points) -> khong
    # con he so meo (dist = 0), va dung dung new_camera_matrix.
    dist_zero = np.zeros((5, 1), dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        objp, imgp, new_camera_matrix, dist_zero, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        raise RuntimeError("solvePnP that bai, khong tim duoc tu the ban co.")

    # Tinh chinh them tu the bang Levenberg-Marquardt de tang do chinh xac
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(objp, imgp, new_camera_matrix, dist_zero, rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)

    # P_cam (Nx3) = R @ P_board^T + t, cho tat ca cac goc cung luc.
    P_cam = (R @ objp.T).T + t

    x_cam = -P_cam[:, 0]  # duong = phai -> trai
    y_cam = -P_cam[:, 1]  # duong = duoi -> tren

    world_mm = np.stack([x_cam, y_cam], axis=1)

    return world_mm, (rvec, tvec)


def save_corners_csv(path, corners, size, square_size, new_camera_matrix):
    """Luu toa do cac goc ra file CSV: pixel goc (da khu meo), pixel so voi
    tam quang hoc THAT cua camera (cx, cy tu new_camera_matrix), va toa do
    THUC (mm) theo he truc CO DINH THEO CAMERA/MAN HINH (x_mm, y_mm)."""
    cx = new_camera_matrix[0, 2]
    cy = new_camera_matrix[1, 2]

    world_mm, (rvec, tvec) = compute_mm_coords(corners, size, square_size, new_camera_matrix)

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["corner_index", "col", "row", "pixel_x", "pixel_y",
                          "x_px", "y_px", "x_mm", "y_mm"])
        board_w, board_h = size
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
    print(f"(Tam quang hoc that cua camera: cx={cx:.3f}, cy={cy:.3f} px)")
    print(f"(He truc x_mm, y_mm co dinh theo camera/man hinh: X duong = "
          f"phai -> trai, Y duong = duoi -> tren; goc (0,0) la diem truc "
          f"quang hoc cat mat phang ban co)")


def annotate_corners(img, corners, size, new_camera_matrix):
    """Ve cac goc len anh kem so thu tu, ve truc toa do CO DINH THEO
    MAN HINH/CAMERA (khong xoay theo do nghieng cua ban co):
    - Truc X (do): mui ten huong SANG TRAI (duong = phai -> trai).
    - Truc Y (xanh duong): mui ten huong LEN TREN (duong = duoi -> tren).
    Goc (0,0) LUON la pixel (cx, cy) - tam quang hoc that cua camera lay
    tu new_camera_matrix - vi day chinh la pixel ma truc quang hoc chieu
    qua, bat ke ban co nghieng the nao. Khong can rvec/tvec de ve truc nay."""
    vis = img.copy()
    h, w = vis.shape[:2]
    cx = new_camera_matrix[0, 2]
    cy = new_camera_matrix[1, 2]
    origin_px = (int(round(cx)), int(round(cy)))

    axis_len_px = max(40, min(w, h) // 8)
    x_axis_px = (origin_px[0] - axis_len_px, origin_px[1])   # X: sang trai
    y_axis_px = (origin_px[0],  origin_px[1] - axis_len_px)   # Y: len tren

    cv2.arrowedLine(vis, origin_px, x_axis_px, (0, 0, 255), 2, cv2.LINE_AA, tipLength=0.2)
    cv2.arrowedLine(vis, origin_px, y_axis_px, (255, 0, 0), 2, cv2.LINE_AA, tipLength=0.2)
    cv2.circle(vis, origin_px, 5, (0, 255, 255), -1)
    cv2.putText(vis, "O(0,0)", (origin_px[0] + 6, origin_px[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, "X", (x_axis_px[0] - 15, x_axis_px[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, "Y", (y_axis_px[0] + 6, y_axis_px[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

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
    # 1) Tim goc tren ANH GOC (chua khu meo) de dat do chinh xac sub-pixel
    #    cao nhat - khong bi anh huong boi noi suy remap.
    found, corners_raw, size = detect_corners_raw_subpix(img, board_w, board_h)
    if not found:
        print("Khong phat hien duoc ban co tren anh.")
        return
    print(f"Phat hien ban co kich thuoc {size[0]} x {size[1]} goc trong, tong {len(corners_raw)} diem.")

    # 2) Khu meo anh CHI de xuat file hien thi/luu (khong dung de do dac).
    undistorted, new_camera_matrix = undistort_image(img, camera_matrix, dist_coeffs)
    cv2.imwrite(out_undistorted, undistorted)
    print(f"Da luu anh da khu meo vao: {out_undistorted}")

    # 3) Khu meo CHO TOA DO DIEM da tim duoc o buoc 1 (chinh xac hon nhieu
    #    so voi tim lai goc tren anh da khu meo/resample).
    corners = undistort_corner_points(corners_raw, camera_matrix, dist_coeffs, new_camera_matrix)

    save_corners_csv(out_csv, corners, size, square_size, new_camera_matrix)

    vis = annotate_corners(undistorted, corners, size, new_camera_matrix)
    cv2.imwrite(out_annotated, vis)
    print(f"Da luu anh co danh so goc vao: {out_annotated}")


def run_webcam(camera_index, camera_matrix, dist_coeffs, board_w, board_h, square_size, out_prefix):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Khong mo duoc camera index {camera_index}")
        return

    print("Nhan SPACE/'s' de chup + tim goc ban co + luu ket qua, 'q'/ESC de thoat.")
    shot_count = 0

    # Preview: dung phuong phap co dien (nhanh) chi de hien thi khung hinh
    # song, KHONG dung de luu du lieu -> khong anh huong do chinh xac.
    preview_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame tu camera.")
            break
        undistorted_preview, _ = undistort_image(frame, camera_matrix, dist_coeffs)

        gray_preview = cv2.cvtColor(undistorted_preview, cv2.COLOR_BGR2GRAY)
        found, corners_preview, size = find_corners_auto(gray_preview, board_w, board_h, preview_flags)

        display = undistorted_preview.copy()
        if found:
            cv2.drawChessboardCorners(display, size, corners_preview, found)
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
            # Khi chup that: dung pipeline CHINH XAC - tim goc tren FRAME
            # GOC (chua khu meo), roi khu meo TOA DO DIEM. Khong dung ket
            # qua preview o tren.
            found_precise, corners_raw, size_precise = detect_corners_raw_subpix(frame, board_w, board_h)
            if not found_precise:
                print("Khong the luu: chua thay ban co trong khung hinh (do chinh xac cao).")
                continue

            undistorted, new_camera_matrix = undistort_image(frame, camera_matrix, dist_coeffs)
            corners_refined = undistort_corner_points(corners_raw, camera_matrix, dist_coeffs, new_camera_matrix)

            out_undistorted = f"{out_prefix}_{shot_count:03d}.jpg"
            out_csv = f"{out_prefix}_{shot_count:03d}_corners.csv"
            out_annotated = f"{out_prefix}_{shot_count:03d}_annotated.jpg"

            cv2.imwrite(out_undistorted, undistorted)
            save_corners_csv(out_csv, corners_refined, size_precise, square_size, new_camera_matrix)
            vis = annotate_corners(undistorted, corners_refined, size_precise, new_camera_matrix)
            cv2.imwrite(out_annotated, vis)

            print(f"Da luu: {out_undistorted}, {out_csv}, {out_annotated}")
            shot_count += 1

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Khu meo anh + xac dinh toa do THUC (mm) cac goc ban co, goc la giao diem truc quang hoc voi mat phang ban co")
    parser.add_argument("--calib", type=str, default="calibration_result.npz",
                         help="File .npz ket qua calib (mac dinh: calibration_result.npz)")
    parser.add_argument("--image", type=str, default=None, help="Duong dan anh dau vao (neu khong dung webcam)")
    parser.add_argument("--camera", type=int, default=0, help="Chi so webcam (mac dinh 0), dung khi khong truyen --image")
    parser.add_argument("--board_w", type=int, default=9, help="So goc trong / o vuong chieu ngang cua ban co (mac dinh 9)")
    parser.add_argument("--board_h", type=int, default=7, help="So goc trong / o vuong chieu doc cua ban co (mac dinh 7)")
    parser.add_argument("--square_size", type=float, default=9.94,
                         help="Kich thuoc 1 o vuong (mm). Neu khong truyen, se tu doc tu file .npz (neu co), "
                              "neu khong co trong .npz thi mac dinh 10 mm")
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
    # cuoi cung fallback 9.96mm neu khong co thong tin nao.
    if args.square_size is not None:
        square_size = args.square_size
        print(f"Su dung square_size tu tham so dong lenh: {square_size} mm")
    elif "square_size" in data:
        square_size = float(data["square_size"])
        print(f"Su dung square_size doc tu file calib: {square_size} mm")
    else:
        square_size = 9.94
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