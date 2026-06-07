import socket
import time
import json
import numpy as np

# Cấu hình kết nối tới Viewer
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_position(x, y, z):
    data = {"x": float(x), "y": float(y), "z": float(z)}
    sock.sendto(json.dumps(data).encode('utf-8'), (UDP_IP, UDP_PORT))
    time.sleep(0.015)  # Thời gian trễ nhỏ tạo vận tốc di chuyển mượt mà


def clear_screen():
    sock.sendto(json.dumps({"cmd": "clear"}).encode('utf-8'), (UDP_IP, UDP_PORT))


# --- HÀM PHÁT QUỸ ĐẠO HÌNH TRÒN ---
def generate_circle(radius=70.0, center_z=-220.0):
    clear_screen()
    print("-> Đang vẽ HÌNH TRÒN...")
    for theta in np.linspace(0, 2 * np.pi, 200):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        send_position(x, y, center_z)


# --- HÀM PHÁT QUỸ ĐẠO HÌNH VUÔNG ---
def generate_square(side=110.0, center_z=-220.0):
    clear_screen()
    print("-> Đang vẽ HÌNH VUÔNG...")
    half = side / 2.0
    # Định nghĩa 4 đỉnh hình vuông
    corners = [
        [-half, -half],
        [half, -half],
        [half, half],
        [-half, half],
        [-half, -half]
    ]
    for i in range(4):
        p1 = corners[i]
        p2 = corners[i + 1]
        for t in np.linspace(0, 1, 50):
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            send_position(x, y, center_z)


# --- HÀM PHÁT QUỸ ĐẠO HÌNH TAM GIÁC ---
def generate_triangle(side=120.0, center_z=-220.0):
    clear_screen()
    print("-> Đang vẽ HÌNH TAM GIÁC...")
    h = side * np.sqrt(3) / 2.0
    # Định nghĩa 3 đỉnh tam giác đều cân đối tâm
    corners = [
        [0.0, 2.0 * h / 3.0],
        [-side / 2.0, -h / 3.0],
        [side / 2.0, -h / 3.0],
        [0.0, 2.0 * h / 3.0]
    ]
    for i in range(3):
        p1 = corners[i]
        p2 = corners[i + 1]
        for t in np.linspace(0, 1, 60):
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            send_position(x, y, center_z)


# --- MENU ĐIỀU KHIỂN CHÍNH ---
if __name__ == "__main__":
    while True:
        print("\n=== BAN DIEU KHIEN QUY DAO ROBOT DELTA ===")
        print("1. Yêu cầu vẽ HÌNH TRÒN")
        print("2. Yêu cầu vẽ HÌNH VUÔNG")
        print("3. Yêu cầu vẽ HÌNH TAM GIÁC")
        print("4. Thoát chương trình")

        choice = input("Nhập lựa chọn của bạn (1-4): ")

        if choice == "1":
            generate_circle()
        elif choice == "2":
            generate_square()
        elif choice == "3":
            generate_triangle()
        elif choice == "4":
            print("Đã đóng bộ điều khiển.")
            break
        else:
            print("Lựa chọn không hợp lệ, vui lòng nhập lại!")