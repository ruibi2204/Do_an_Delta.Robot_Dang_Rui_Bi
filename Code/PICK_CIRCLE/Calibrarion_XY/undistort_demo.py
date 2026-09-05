import argparse
import os
import sys
import cv2
import numpy as np
def main():
    parser = argparse.ArgumentParser(description="Khu meo anh/video dung ket qua calib")
    parser.add_argument("--calib", type=str, default="calibration_result.npz", help="File .npz ket qua calib (mac dinh: calibration_result.npz)")
    parser.add_argument("--image", type=str, default=None, help="Duong dan anh can khu meo")
    parser.add_argument("--out", type=str, default="undistorted.jpg", help="Duong dan luu anh ket qua")
    parser.add_argument("--camera", type=int, default=0, help="Chi so webcam de khu meo truc tiep (mac dinh 0)")
    args = parser.parse_args()

    if not os.path.isfile(args.calib):
        print(f"LOI: Khong tim thay file calib '{args.calib}'.")
        print("Hay chay calibrate_camera.py truoc de tao ra file nay, hoac truyen dung duong dan bang --calib <duong_dan>")
        sys.exit(1)

    data = np.load(args.calib)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"Khong doc duoc anh: {args.image}")
            return
        h, w = img.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(
            img, camera_matrix, dist_coeffs, None, new_camera_matrix
        )
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undistorted_cropped = undistorted[y:y + rh, x:x + rw]
        else:
            undistorted_cropped = undistorted

        cv2.imwrite(args.out, undistorted_cropped)
        print(f"Da luu anh da khu meo vao: {args.out}")

    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"Khong mo duoc camera {args.camera}")
            return
        print("Nhan 'q' de thoat.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            h, w = frame.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h)
            )
            undistorted = cv2.undistort(
                frame, camera_matrix, dist_coeffs, None, new_camera_matrix
            )
            combined = np.hstack([frame, undistorted])
            cv2.imshow("Goc (trai) vs Da khu meo (phai)", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Can chi dinh --image hoac --camera")


if __name__ == "__main__":
    main()