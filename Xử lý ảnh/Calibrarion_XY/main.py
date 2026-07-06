import cv2
import numpy as np
import glob
import json
import os

# --- Cau hinh ---
CHECKERBOARD = (9, 7)        # so goc trong (10x8 o vuong -> 9x7 goc)
square_size = 20.0           # mm - ban co moi (luoi 20x20mm, in tren A4)
TARGET_NUM_IMAGES = 20       # so anh muc tieu can co truoc khi chay calibration
IMAGES_FOLDER = 'calib_images2'
CAMERA_ID = 0                # doi lai neu may co nhieu camera

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def capture_more_images(folder, target_count, camera_id=0):
    """Mo camera, cho phep chup them anh (SPACE = chup, ESC = dung som) cho den khi du target_count anh."""
    os.makedirs(folder, exist_ok=True)
    existing = sorted(glob.glob(os.path.join(folder, '*.jpg')))
    count = len(existing)
    next_index = count + 1

    print(f"Hien co {count}/{target_count} anh trong '{folder}'.")
    print(f"Can chup them {max(0, target_count - count)} anh.")
    print("Nhan [SPACE] de chup, [ESC] de dung som.")
    print("Luu y: chup o NHIEU GOC DO/VI TRI khac nhau (nghieng, xoay, gan/xa, cac goc khung hinh).")

    # CAP_DSHOW (DirectShow) tren Windows giup mo camera nhanh hon nhieu (tu ~3s xuong <1s)
    # so voi backend mac dinh (MSMF). Tren Linux/Mac dung backend mac dinh.
    if os.name == "nt":
        cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        raise RuntimeError("Khong mo duoc camera!")

    # Giam buffer xuong 1 frame de luon lay duoc frame moi nhat (giam do tre hinh anh)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    while count < target_count:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame tu camera.")
            break

        display = frame.copy()
        cv2.putText(display, f"Da chup: {count}/{target_count}  (SPACE=chup, ESC=dung)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Chup anh ban co - SPACE de chup, ESC de dung", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("Da dung chup som theo yeu cau nguoi dung.")
            break
        elif key == 32:  # SPACE
            filename = os.path.join(folder, f"img_{next_index:03d}.jpg")
            cv2.imwrite(filename, frame)
            count += 1
            next_index += 1
            print(f"Da luu: {filename} ({count}/{target_count})")

    cap.release()
    cv2.destroyAllWindows()
    return count


def run_calibration(folder):
    """Chay toan bo quy trinh intrinsic calibration tren anh trong 'folder'."""
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []
    img_shape = None
    used_files = []

    images = sorted(glob.glob(f'{folder}/*.jpg'))
    print(f"\nTong so anh: {len(images)}")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            used_files.append(fname)
        else:
            print(f"KHONG phat hien duoc bang co: {fname}")

    print(f"Phat hien thanh cong: {len(used_files)}/{len(images)} anh")

    if len(used_files) < 5:
        print("Qua it anh hop le, dung lai.")
        return

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None)

    print("\n=== KET QUA CALIBRATION ===")
    print("RMS reprojection error (tong the):", ret)
    print("Camera matrix K:\n", K)
    print("Distortion coeffs [k1,k2,p1,p2,k3]:\n", dist.ravel())

    print("\n=== SAI SO TUNG ANH ===")
    per_image_errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        per_image_errors.append(error)
        print(f"{used_files[i]}: {error:.4f} px")

    # Luu ket qua
    result = {
        "image_width": img_shape[0],
        "image_height": img_shape[1],
        "square_size_mm": square_size,
        "checkerboard_inner_corners": CHECKERBOARD,
        "rms_reprojection_error": ret,
        "camera_matrix_K": K.tolist(),
        "distortion_coefficients": dist.ravel().tolist(),
        "num_images_used": len(used_files),
        "per_image_errors": {used_files[i]: per_image_errors[i] for i in range(len(used_files))}
    }

    with open('camera_calib_result.json', 'w') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    np.savez('camera_calib_XY.npz', K=K, dist=dist, rvecs=rvecs, tvecs=tvecs)

    # Undistort anh mau de kiem tra truc quan
    test_img = cv2.imread(used_files[0])
    h, w = test_img.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(test_img, K, dist, None, new_K)
    cv2.imwrite('undistorted_sample.jpg', undistorted)
    # ghep 2 anh canh nhau de so sanh
    comparison = np.hstack([test_img, undistorted])
    cv2.imwrite('comparison_before_after.jpg', comparison)

    print("\nDa luu: camera_calib_result.json, camera_calib_XY.npz, comparison_before_after.jpg")


def main():
    existing = glob.glob(os.path.join(IMAGES_FOLDER, '*.jpg'))

    if len(existing) < TARGET_NUM_IMAGES:
        final_count = capture_more_images(IMAGES_FOLDER, TARGET_NUM_IMAGES, CAMERA_ID)
        if final_count < 5:
            print(f"\nChi co {final_count} anh - qua it de calibration (can toi thieu 5). Dung chuong trinh.")
            return
        if final_count < TARGET_NUM_IMAGES:
            print(f"\n[Luu y] Dung som voi {final_count}/{TARGET_NUM_IMAGES} anh - van du toi thieu de tiep tuc.")
    else:
        print(f"Da co du {len(existing)}/{TARGET_NUM_IMAGES} anh trong '{IMAGES_FOLDER}', bo qua buoc chup.")

    run_calibration(IMAGES_FOLDER)


if __name__ == "__main__":
    main()