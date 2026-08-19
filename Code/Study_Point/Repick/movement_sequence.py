#!/usr/bin/env python3
# movement_sequence.py
# Thực hiện chuỗi di chuyển robot Delta theo kịch bản

import time
import sys
from kinematics import inverse_kinematics
from uart_comm import UARTComm
import threading
# ---- Cấu hình ----
HOME_POS = (0.0, 0.0, 300.0)        # Vị trí về nhà (z cao)
PICK_Z = 360                # Chiều cao khi hạ xuống (giữ nguyên x,y)
LIFT_OFFSET = 4.0                 # mm nâng lên sau khi giữ tại PICK_Z (z giảm đi = nâng lên)
WAIT_HOME = 2                     # giây đứng yên tại HOME
WAIT_PICK = 1                     # giây đứng yên tại PICK

# Danh sách các điểm cần đến (mỗi điểm là tuple (x,y,z))
# Ở đây tôi định nghĩa vài điểm mẫu, bạn có thể thay đổi.
POINTS = [
    (0.0, 0.0, 300.0)
]

def move_to(comm: UARTComm, x: float, y: float, z: float):
    """Tính góc và gửi lệnh di chuyển đến (x,y,z)."""
    try:
        t1, t2, t3 = inverse_kinematics(x, y, z)
        comm.send_angles(t1, t2, t3)
        return True
    except Exception as e:
        comm.log(f"[ERR] Không thể di chuyển đến ({x},{y},{z}): {e}")
        return False

def run_sequence(comm: UARTComm, points=None):
    """
    Thực hiện chuỗi di chuyển:
      - Về HOME, đợi WAIT_HOME giây
      - Với mỗi điểm trong danh sách:
          + Di chuyển đến điểm đó (giữ nguyên z=300 nếu điểm đã có z=300)
          + Hạ xuống PICK_Z (giữ nguyên x,y), đợi WAIT_PICK giây
          + Nâng lên thêm LIFT_OFFSET mm (giữ nguyên x,y)
          + Về lại HOME
    """
    if points is None:
        points = POINTS

    comm.log("[INFO] Bắt đầu chuỗi di chuyển")

    # Về HOME lần đầu
    comm.log(f"[INFO] Về vị trí HOME {HOME_POS}")
    move_to(comm, *HOME_POS)
    time.sleep(WAIT_HOME)

    for idx, (px, py, pz) in enumerate(points, 1):
        comm.log(f"[INFO] Điểm {idx}: ({px},{py},{pz})")

        # Di chuyển đến điểm (với z hiện tại)
        move_to(comm, px, py, pz)
        time.sleep(2)  # chờ ổn định

        # Hạ xuống độ cao PICK_Z (giữ nguyên x,y)
        move_to(comm, px, py, PICK_Z)
        time.sleep(WAIT_PICK)

        # Nâng lên thêm LIFT_OFFSET mm trước khi về HOME (giữ nguyên x,y)
        lift_z = PICK_Z - LIFT_OFFSET
        comm.log(f"[INFO] Nâng lên {LIFT_OFFSET}mm -> z={lift_z}")
        move_to(comm, px, py, lift_z)
        time.sleep(WAIT_PICK)
        # Về HOME
        move_to(comm, *HOME_POS)
        time.sleep(WAIT_HOME)

    comm.log("[INFO] Hoàn thành chuỗi di chuyển")

if __name__ == "__main__":
    # Chạy thử với dry-run
    comm = UARTComm(port="COM6", dry_run=True)
    comm.connect()
    run_sequence(comm)
    comm.disconnect()