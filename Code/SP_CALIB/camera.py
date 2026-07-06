import cv2
import numpy as np
import os
import json
import threading

# ============ CAU HINH ============
CHECKERBOARD = (9, 7)       # so giao diem: 9 cot x 7 hang (goc trong)

# Kich thuoc thuc te cua 1 o ban co theo TUNG TRUC (khong phai hinh vuong chuan
# do sai so scale khi in/gia cong ban co): 20mm x 21mm
SQUARE_SIZE_X_MM = 21      # kich thuoc o theo truc X (doc - theo huong row)
SQUARE_SIZE_Y_MM = 20      # kich thuoc o theo truc Y (ngang - theo huong col)

BOARD_Z_MM = 380.0           # Z co dinh cho toan bo mat phang ban co

# Chi so (row, col) cua giao diem CHINH GIUA - voi luoi 9 cot (0..8) va 7 hang (0..6),
# trung tam chinh xac la col=4, row=3 (khong can noi suy vi 9 va 7 deu la so le)
CENTER_ROW = (CHECKERBOARD[1] - 1) // 2   # = 3
CENTER_COL = (CHECKERBOARD[0] - 1) // 2   # = 4

# Quy uoc truc toa do robot:
#   - Truc X: duong TU DUOI LEN (row tang -> x tang)
#   - Truc Y: duong TU PHAI SANG TRAI (col giam -> y tang)
# Doi dau neu chieu truc anh nguoc voi chieu truc robot thuc te
ROW_SIGN = -1    # da xac nhan qua anh test: can dao dau de x duong TU DUOI LEN (row=0 o tren -> x duong)
COL_SIGN = 1     # da xac nhan: y dang dung chieu, duong TU PHAI SANG TRAI (khong can doi)

CAMERA_ID = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
INTRINSIC_CALIB_FILE = "camera_calib_XY.npz"   # neu co, se tu dong khu meo truoc khi nhan dien


class Camera:
    def __init__(self, camera_id=None, width=None, height=None, intrinsic_file=None):
        self.camera_id = camera_id if camera_id is not None else CAMERA_ID
        self.width = width or CAMERA_WIDTH
        self.height = height or CAMERA_HEIGHT
        self.intrinsic_file = intrinsic_file or INTRINSIC_CALIB_FILE
        self.cap = None
        self.K = None
        self.dist = None
        self._load_intrinsics()

    def _load_intrinsics(self):
        if os.path.exists(self.intrinsic_file):
            data = np.load(self.intrinsic_file)
            self.K = data["K"]
            self.dist = data["dist"]
            print(f"Da nap thong so intrinsic tu {self.intrinsic_file} - anh se duoc khu meo.")
        else:
            print(f"[Canh bao] Khong tim thay {self.intrinsic_file} - anh se KHONG duoc khu meo "
                  f"(toa do se kem chinh xac hon o vung ria anh).")

    def open(self):
        if os.name == "nt":
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"Khong mo duoc camera id={self.camera_id}")
        return self

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def read(self, undistort=True):
        if self.cap is None:
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        if undistort and self.K is not None:
            frame = cv2.undistort(frame, self.K, self.dist)
        return frame

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def detect_board_corners(frame, checkerboard=None, square_size_x=None, square_size_y=None,
                          board_z=None, draw=True):

    checkerboard = checkerboard or CHECKERBOARD
    square_size_x = square_size_x if square_size_x is not None else SQUARE_SIZE_X_MM
    square_size_y = square_size_y if square_size_y is not None else SQUARE_SIZE_Y_MM
    board_z = board_z if board_z is not None else BOARD_Z_MM

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, checkerboard, flags)

    if not found:
        return (frame.copy() if draw else None), False, []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    corners = corners.reshape(-1, 2)  # (N,2) pixel toa do, N = n_cols*n_rows

    n_cols, n_rows = checkerboard
    corners_data = []
    for idx in range(len(corners)):
        row = idx // n_cols
        col = idx % n_cols
        u, v = corners[idx]

        # ====== CACH TINH TOA DO ======
        # Truc x (doc): duong tu duoi len (row tang -> x tang), dung kich thuoc o theo X
        x = ROW_SIGN * (row - CENTER_ROW) * square_size_x

        # Truc y (ngang): duong tu phai sang trai (col giam -> y tang), dung kich thuoc o theo Y
        # Su dung dau am de dao chieu so voi cach thong thuong
        y = -COL_SIGN * (col - CENTER_COL) * square_size_y
        # Cach viet khac: y = COL_SIGN * (CENTER_COL - col) * square_size_y

        z = board_z

        corners_data.append({
            "row": row, "col": col,
            "pixel_u": float(u), "pixel_v": float(v),
            "robot_x": float(x), "robot_y": float(y), "robot_z": float(z),
            "is_center": (row == CENTER_ROW and col == CENTER_COL),
        })

    annotated = None
    if draw:
        annotated = frame.copy()
        cv2.drawChessboardCorners(annotated, checkerboard, corners.reshape(-1, 1, 2), found)
        for d in corners_data:
            u, v = int(d["pixel_u"]), int(d["pixel_v"])
            # Ve vong tron cho tat ca cac diem
            if d["is_center"]:
                # Diem (0,0) - mau do
                cv2.circle(annotated, (u, v), 5, (0, 0, 255), -1)
                cv2.circle(annotated, (u, v), 6, (0, 0, 255), 2)
            else:
                # Cac diem khac - mau xanh la
                cv2.circle(annotated, (u, v), 4, (0, 255, 0), 1)
            # KHONG hien thi toa do text

    return annotated, found, corners_data


def save_corners_json(corners_data, path="board_corners.json"):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(corners_data, f, indent=2, ensure_ascii=False)
    print(f"Da luu {len(corners_data)} giao diem vao {path}")


# ============ TEST NHANH (chay truc tiep file nay) ============
if __name__ == "__main__":
    print(f"Trung tam ban co tai (row={CENTER_ROW}, col={CENTER_COL}) -> gan toa do (0,0,{BOARD_Z_MM})")
    print(f"Kich thuoc o: X={SQUARE_SIZE_X_MM}mm, Y={SQUARE_SIZE_Y_MM}mm")
    print("Quy uoc truc toa do: x duong tu duoi len, y duong tu phai sang trai.")
    print("Nhan [SPACE] de luu danh sach toa do ra file 'board_corners.json'.")
    print("Nhan [ESC] de thoat.\n")

    with Camera() as cam:
        while True:
            frame = cam.read()
            if frame is None:
                print("Khong doc duoc frame tu camera.")
                break

            display, found, corners_data = detect_board_corners(frame)

            status = f"Phat hien {len(corners_data)}/{CHECKERBOARD[0]*CHECKERBOARD[1]} giao diem" if found else "KHONG thay ban co"
            cv2.putText(display, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if found else (0, 0, 255), 2)

            cv2.imshow("Nhan dien giao diem ban co - SPACE=luu, ESC=thoat", display)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == 32 and found:  # SPACE
                save_corners_json(corners_data)

    cv2.destroyAllWindows()