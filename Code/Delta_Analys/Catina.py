import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import socket
import json

# ==========================================
# 1. THÔNG SỐ KỸ THUẬT VÀ ĐỘNG HỌC NGƯỢC
# ==========================================
sb = 180.0
sp = 50.0
L = 120.0
l = 250.0
angles = np.array([0, 120, 240]) * np.pi / 180.0


def delta_calc_inverse(x0, y0, z0):
    theta = [0.0, 0.0, 0.0]
    for i in range(3):
        X = x0 * np.cos(angles[i]) + y0 * np.sin(angles[i])
        Y = -x0 * np.sin(angles[i]) + y0 * np.cos(angles[i])
        Z = z0
        X -= sb - sp
        a = 2 * L * X
        b = 2 * L * Z
        c = X ** 2 + Y ** 2 + Z ** 2 + L ** 2 - l ** 2
        discriminant = a ** 2 + b ** 2 - c ** 2
        if discriminant < 0: return None
        t = (-b - np.sqrt(discriminant)) / (c - a)
        theta[i] = 2 * np.arctan(t)
    return theta


# ==========================================
# 2. HÀM VẼ CƠ KHÍ
# ==========================================
def draw_cylinder(radius, length, color):
    glColor3f(*color)
    quadric = gluNewQuadric()
    gluCylinder(quadric, radius, radius, length, 16, 16)
    gluDisk(quadric, 0, radius, 16, 1)
    glTranslatef(0, 0, length)
    gluDisk(quadric, 0, radius, 16, 1)
    glTranslatef(0, 0, -length)


def draw_robot(ee_pos, angles_deg):
    # --- VẼ BASE ---
    glColor3f(0.3, 0.3, 0.3)
    glBegin(GL_TRIANGLES)
    for a in angles: glVertex3f(sb * np.cos(a), sb * np.sin(a), 0)
    glEnd()

    for i in range(3):
        bx, by, bz = sb * np.cos(angles[i]), sb * np.sin(angles[i]), 0
        th = angles_deg[i]
        ax_local = (sb - sp) + L * np.cos(th)
        az_local = L * np.sin(th)
        ax = ax_local * np.cos(angles[i])
        ay = ax_local * np.sin(angles[i])
        az = az_local
        px = ee_pos[0] + sp * np.cos(angles[i])
        py = ee_pos[1] + sp * np.sin(angles[i])
        pz = ee_pos[2]

        # Cánh tay chủ động
        glPushMatrix()
        glTranslatef(bx, by, bz)
        dx, dy, dz = ax - bx, ay - by, az - bz
        dist = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        glRotatef(np.degrees(np.arctan2(np.sqrt(dx ** 2 + dy ** 2), dz)), -dy, dx, 0)
        draw_cylinder(6.0, dist, (0.1, 0.4, 0.8))
        glPopMatrix()

        # Cánh tay bị động
        glPushMatrix()
        glTranslatef(ax, ay, az)
        dx, dy, dz = px - ax, py - ay, pz - az
        dist = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        glRotatef(np.degrees(np.arctan2(np.sqrt(dx ** 2 + dy ** 2), dz)), -dy, dx, 0)
        draw_cylinder(3.0, dist, (0.2, 0.2, 0.2))
        glPopMatrix()

    # --- VẼ MOVING PLATFORM ---
    glPushMatrix()
    glTranslatef(ee_pos[0], ee_pos[1], ee_pos[2])
    glColor3f(0.6, 0.1, 0.1)
    glBegin(GL_TRIANGLES)
    for a in angles: glVertex3f(sp * np.cos(a), sp * np.sin(a), 0)
    glEnd()
    glPopMatrix()


# --- HÀM VẼ ĐƯỜNG QUỸ ĐẠO ĐÃ ĐI QUA ---
def draw_trajectory(trail_points):
    if len(trail_points) < 2: return
    glLineWidth(2.5)
    glColor3f(0.0, 1.0, 0.8)  # Màu xanh Neon đặc trưng sinh động
    glBegin(GL_LINE_STRIP)
    for p in trail_points:
        glVertex3f(p[0], p[1], p[2])
    glEnd()


# ==========================================
# 3. VÒNG LẶP CHƯƠNG TRÌNH CHÍNH
# ==========================================
def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Mô Phỏng Robot Delta - Viewer")

    glEnable(GL_DEPTH_TEST)
    clock = pygame.time.Clock()

    # Cấu hình Socket UDP nhận dữ liệu từ file điều khiển độc lập
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)  # Chế độ non-blocking để tránh treo đồ họa khi chưa có dữ liệu

    # Vị trí đứng yên mặc định ban đầu khi chưa nhận lệnh
    current_pos = [0.0, 0.0, -220.0]
    trajectory_trail = []  # Mảng lưu vết đường đi

    pitch, yaw, distance = 25.0, 40.0, 650.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        # Nhận phím điều chỉnh góc nhìn camera
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:  yaw -= 1.5
        if keys[pygame.K_RIGHT]: yaw += 1.5
        if keys[pygame.K_UP]:    pitch += 1.5
        if keys[pygame.K_DOWN]:  pitch -= 1.5
        pitch = max(5.0, min(85.0, pitch))

        # --- KIỂM TRA VÀ CẬP NHẬT DỮ LIỆU TỪ FILE ĐIỀU KHIỂN ---
        try:
            data, addr = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))

            # Nếu nhận được lệnh xóa vệt vẽ cũ
            if msg.get("cmd") == "clear":
                trajectory_trail.clear()
            else:
                current_pos = [msg['x'], msg['y'], msg['z']]
                trajectory_trail.append(list(current_pos))
                if len(trajectory_trail) > 1000:  # Giới hạn bộ nhớ vết vẽ
                    trajectory_trail.pop(0)
        except BlockingIOError:
            pass  # Chưa có tọa độ mới, tiếp tục giữ nguyên vị trí cũ

        # --- THIẾT LẬP CAMERA (GÓC NHÌN CHUẨN ĐÃ THỐNG NHẤT) ---
        glViewport(0, 0, display[0], display[1])
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (display[0] / display[1]), 1.0, 2000.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        target_z = -150.0
        rad_pitch, rad_yaw = np.radians(pitch), np.radians(yaw)
        cam_x = distance * np.cos(rad_pitch) * np.sin(rad_yaw)
        cam_y = distance * np.sin(rad_pitch)
        cam_z = target_z + distance * np.cos(rad_pitch) * np.cos(rad_yaw)

        gluLookAt(cam_x, -cam_z, cam_y, 0.0, 0.0, target_z, 0.0, 0.0, 1.0)

        glClearColor(0.12, 0.12, 0.12, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Lưới không gian nền
        glColor3f(0.2, 0.2, 0.2)
        glBegin(GL_LINES)
        for grid_i in range(-300, 301, 50):
            glVertex3f(grid_i, -300, -350);
            glVertex3f(grid_i, 300, -350)
            glVertex3f(-300, grid_i, -350);
            glVertex3f(300, grid_i, -350)
        glEnd()

        # Tiến hành vẽ vệt quỹ đạo và vẽ mô hình robot
        draw_trajectory(trajectory_trail)

        joint_angles = delta_calc_inverse(*current_pos)
        if joint_angles is not None:
            draw_robot(current_pos, joint_angles)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()