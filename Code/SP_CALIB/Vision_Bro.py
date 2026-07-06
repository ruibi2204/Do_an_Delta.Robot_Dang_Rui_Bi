import cv2
import time

# ============ CAU HINH ============
CAMERA_ID = 1
FRAME_WIDTH = 480
FRAME_HEIGHT = 720
TARGET_FPS = 30   # camera toi da 30fps @ 720p


def open_camera(cam_id):
    import os
    cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(cam_id)

    if not cap.isOpened():
        raise RuntimeError(f"Khong mo duoc camera id={cam_id}")

    # MJPG + BUFFERSIZE=1 la 2 yeu to quan trong nhat de anh muot, do tre thap
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    return cap


def main():
    cap = open_camera(CAMERA_ID)

    prev_time = time.time()
    fps_smooth = 0.0

    print(f"Camera 2 (id={CAMERA_ID}) - CHE DO QUAN SAT | ESC de thoat")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame tu camera.")
            break

        now = time.time()
        dt = now - prev_time
        prev_time = now
        if dt > 0:
            fps_smooth = fps_smooth * 0.9 + (1.0 / dt) * 0.1

        cv2.putText(frame, f"FPS: {fps_smooth:.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow(f"Camera 2 (id={CAMERA_ID}) - ESC=thoat", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()