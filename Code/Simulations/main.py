# pyrefly: ignore [missing-import]
import vtk
import math
import random
import sys

# --- THÔNG SỐ CƠ KHÍ ROBOT DELTA VÀ QUY ƯỚC CHUẨN (Đơn vị: mm) ---
sb = 160.0  # Cạnh tam giác đều của Đế cố định (Base)
sp = 50.0   # Cạnh tam giác đều của Đầu gắp di động (Platform)
L1 = 110.0  # Chiều dài Cánh tay trên chủ động (Upper Arm)
L2 = 240.0  # Chiều dài Cánh tay dưới song song (Lower Arm)

f = sb / math.sqrt(3)
e = sp / math.sqrt(3)
ALPHA = [0.0, 120.0, 240.0]

# --- HÀM TÍNH KINEMATICS NGƯỢC CHUẨN ĐỒ ĐỒNG BỘ ---
def delta_calcInverse(x0, y0, z0):
    thetas = [0.0, 0.0, 0.0]
    a = (f - e) / 2.0
    for i in range(3):
        phi = math.radians(ALPHA[i])
        x = x0 * math.cos(phi) + y0 * math.sin(phi)
        y = -x0 * math.sin(phi) + y0 * math.cos(phi)
        z = z0

        E = 2.0 * L1 * (x + a)
        F = 2.0 * z * L1
        G = (x + a) ** 2 + y ** 2 + z ** 2 + L1 ** 2 - L2 ** 2

        dist = E ** 2 + F ** 2 - G ** 2
        if dist < 0:
            return None

        t = (-F - math.sqrt(dist)) / (G - E)
        thetas[i] = math.degrees(2.0 * math.atan(t))
    return thetas

# --- HÀM ĐỊNH HƯỚNG VÀ KHÓA CHẶT 2 ĐẦU KHỚP THANH TRỤ (3D TRANSFORM) ---
def orient_cylinder(actor, p1, p2, radius):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.hypot(dx, math.hypot(dy, dz))

    if length < 1e-6:
        return

    cylinder_source = actor.GetMapper().GetInputConnection(0, 0).GetProducer()
    cylinder_source.SetHeight(length)
    cylinder_source.SetRadius(radius)

    transform = vtk.vtkTransform()
    transform.Translate(p1[0] + dx / 2.0, p1[1] + dy / 2.0, p1[2] + dz / 2.0)

    v_source = [0.0, 1.0, 0.0]
    v_target = [dx / length, dy / length, dz / length]

    cross_v = [
        v_source[1] * v_target[2] - v_source[2] * v_target[1],
        v_source[2] * v_target[0] - v_source[0] * v_target[2],
        v_source[0] * v_target[1] - v_source[1] * v_target[0]
    ]
    sin_a = math.hypot(cross_v[0], math.hypot(cross_v[1], cross_v[2]))
    cos_a = v_source[0] * v_target[0] + v_source[1] * v_target[1] + v_source[2] * v_target[2]
    angle = math.degrees(math.atan2(sin_a, cos_a))

    if sin_a > 1e-6:
        transform.RotateWXYZ(angle, cross_v[0], cross_v[1], cross_v[2])
    elif cos_a < 0:
        transform.RotateX(180)

    actor.SetUserTransform(transform)

def create_cylinder_actor():
    source = vtk.vtkCylinderSource()
    source.SetResolution(32)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor

def create_sphere_actor(radius, color):
    source = vtk.vtkSphereSource()
    source.SetRadius(radius)
    source.SetThetaResolution(32)
    source.SetPhiResolution(32)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)
    return actor

# --- CHƯƠNG TRÌNH CHÍNH ---
def main():
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.92, 0.93, 0.95)

    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1100, 750)
    renderWindow.SetWindowName("Delta Robot 3D Simulation - Phân loại màu sắc")

    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)
    interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    light = vtk.vtkLight()
    light.SetFocalPoint(0, 0, -200)
    light.SetPosition(300, 400, 500)
    renderer.AddLight(light)

    # 1. Đế trên cố định (Fixed Base)
    base_mesh = vtk.vtkCylinderSource()
    base_mesh.SetRadius(f * 1.3)
    base_mesh.SetHeight(15)
    base_mesh.SetResolution(3)
    base_mapper = vtk.vtkPolyDataMapper()
    base_mapper.SetInputConnection(base_mesh.GetOutputPort())
    base_actor = vtk.vtkActor()
    base_actor.SetMapper(base_mapper)
    base_actor.GetProperty().SetColor(0.15, 0.15, 0.15)
    base_actor.GetProperty().SetSpecular(0.4)

    t_base = vtk.vtkTransform()
    t_base.RotateX(90)
    t_base.RotateY(30)
    base_actor.SetUserTransform(t_base)
    renderer.AddActor(base_actor)

    # 2. Bàn máy làm việc (Working Table)
    table = vtk.vtkCubeSource()
    table.SetBounds(-300, 300, -300, 300, -350, -340)
    table_mapper = vtk.vtkPolyDataMapper()
    table_mapper.SetInputConnection(table.GetOutputPort())
    table_actor = vtk.vtkActor()
    table_actor.SetMapper(table_mapper)
    table_actor.GetProperty().SetColor(0.65, 0.65, 0.65)
    renderer.AddActor(table_actor)

    # Các khay chứa vật phân loại
    drop_positions = {
        (0.86, 0.2, 0.27): (140, -80, -335),  # Đỏ
        (0.16, 0.65, 0.27): (140, 30, -335),   # Xanh lá
        (0.0, 0.48, 1.0): (140, 140, -335)    # Xanh dương
    }
    for color, pos in drop_positions.items():
        box = vtk.vtkCubeSource()
        box.SetBounds(pos[0] - 25, pos[0] + 25, pos[1] - 25, pos[1] + 25, -340, -332)
        b_mapper = vtk.vtkPolyDataMapper()
        b_mapper.SetInputConnection(box.GetOutputPort())
        b_actor = vtk.vtkActor()
        b_actor.SetMapper(b_mapper)
        b_actor.GetProperty().SetColor(color)
        b_actor.GetProperty().SetOpacity(0.5)
        renderer.AddActor(b_actor)

    # 3. Khâu liên kết cấu trúc cơ khí
    robot_components = {}
    for i in range(3):
        # Upper Arm
        robot_components[f'upper_{i}'] = create_cylinder_actor()
        robot_components[f'upper_{i}'].GetProperty().SetColor(0.1, 0.45, 0.75)
        renderer.AddActor(robot_components[f'upper_{i}'])

        # Lower Arm cặp song song
        for j in range(2):
            robot_components[f'lower_{i}_{j}'] = create_cylinder_actor()
            robot_components[f'lower_{i}_{j}'].GetProperty().SetColor(0.3, 0.3, 0.3)
            renderer.AddActor(robot_components[f'lower_{i}_{j}'])

            robot_components[f'joint_u_{i}_{j}'] = create_sphere_actor(radius=5.5, color=(0.1, 0.1, 0.1))
            robot_components[f'joint_l_{i}_{j}'] = create_sphere_actor(radius=5.5, color=(0.1, 0.1, 0.1))
            renderer.AddActor(robot_components[f'joint_u_{i}_{j}'])
            renderer.AddActor(robot_components[f'joint_l_{i}_{j}'])

    # 4. Đầu gắp tam giác (End-Effector Platform)
    ee_mesh = vtk.vtkCylinderSource()
    ee_mesh.SetRadius(e * 1.3)
    ee_mesh.SetHeight(8)
    ee_mesh.SetResolution(3)
    ee_mapper = vtk.vtkPolyDataMapper()
    ee_mapper.SetInputConnection(ee_mesh.GetOutputPort())
    ee_actor = vtk.vtkActor()
    ee_actor.SetMapper(ee_mapper)
    ee_actor.GetProperty().SetColor(0.2, 0.2, 0.2)
    renderer.AddActor(ee_actor)

    # 5. Khởi tạo phôi phân loại mẫu
    item_source = vtk.vtkCylinderSource()
    item_source.SetRadius(10)
    item_source.SetHeight(12)
    item_source.SetResolution(24)
    item_mapper = vtk.vtkPolyDataMapper()
    item_mapper.SetInputConnection(item_source.GetOutputPort())
    item_actor = vtk.vtkActor()
    item_actor.SetMapper(item_mapper)
    renderer.AddActor(item_actor)

    # --- HÀM ĐỒNG BỘ VỊ TRÍ & XOAY CỦA PHÔI (FIX LỖI) ---
    def move_item_to(x, y, z):
        """Hàm gộp chung phép tịnh tiến và phép xoay khống chế để Actor cập nhật chuẩn xác"""
        t = vtk.vtkTransform()
        t.Translate(x, y, z)
        t.RotateX(90) # Giữ phôi hình trụ đứng thẳng trên bàn máy
        item_actor.SetUserTransform(t)

    # --- HÀM CẬP NHẬT ĐỘNG HỌC TOÀN CỤC ---
    def update_robot_geometry(x0, y0, z0):
        t_ee = vtk.vtkTransform()
        t_ee.Translate(x0, y0, z0)
        t_ee.RotateX(90)
        t_ee.RotateY(30)
        ee_actor.SetUserTransform(t_ee)

        angles = delta_calcInverse(x0, y0, z0)
        if not angles:
            return False

        d_offset = 18.0

        for i in range(3):
            theta_rad = math.radians(angles[i])
            alpha_rad = math.radians(ALPHA[i])

            perp_x = -math.sin(alpha_rad)
            perp_y = math.cos(alpha_rad)

            B_i = (f * math.cos(alpha_rad), f * math.sin(alpha_rad), 0.0)

            J_i_center = ((f + L1 * math.cos(theta_rad)) * math.cos(alpha_rad),
                          (f + L1 * math.cos(theta_rad)) * math.sin(alpha_rad),
                          -L1 * math.sin(theta_rad))

            orient_cylinder(robot_components[f'upper_{i}'], B_i, J_i_center, radius=4.5)

            for j in range(2):
                sign = 1.0 if j == 0 else -1.0
                offset_x = sign * (d_offset / 2.0) * perp_x
                offset_y = sign * (d_offset / 2.0) * perp_y

                J_u_offset = (J_i_center[0] + offset_x, J_i_center[1] + offset_y, J_i_center[2])
                P_l_offset = (x0 + e * math.cos(alpha_rad) + offset_x,
                              y0 + e * math.sin(alpha_rad) + offset_y,
                              z0)

                orient_cylinder(robot_components[f'lower_{i}_{j}'], J_u_offset, P_l_offset, radius=2.0)
                robot_components[f'joint_u_{i}_{j}'].SetPosition(J_u_offset)
                robot_components[f'joint_l_{i}_{j}'].SetPosition(P_l_offset)
        return True

    camera = renderer.GetActiveCamera()
    camera.SetPosition(450, -450, 150)
    camera.SetFocalPoint(0, 0, -220)
    camera.SetViewUp(0, 0, 1)
    renderer.ResetCamera()

    # --- ĐỊNH NGHĨA QUỸ ĐẠO MÁY TRẠNG THÁI ---
    pick_pos = (-120, -20, -334)  # Tọa độ thực tế phôi nằm trên mặt bàn xám
    current_color = random.choice(list(drop_positions.keys()))

    item_actor.GetProperty().SetColor(current_color)
    move_item_to(pick_pos[0], pick_pos[1], pick_pos[2] + 6.0) # Đưa phôi về đúng vị trí đón gắp

    ee_pos = [0.0, 0.0, -180.0]
    target_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 40.0]

    state = "APPROACH_PICK"
    speed = 4.0  # mm/frame

    interactor.Initialize()

    while True:
        interactor.ProcessEvents()

        dx = target_pos[0] - ee_pos[0]
        dy = target_pos[1] - ee_pos[1]
        dz = target_pos[2] - ee_pos[2]
        dist = math.hypot(dx, math.hypot(dy, dz))

        if dist > speed:
            ee_pos[0] += (dx / dist) * speed
            ee_pos[1] += (dy / dist) * speed
            ee_pos[2] += (dz / dist) * speed
        else:
            ee_pos = list(target_pos)

            if state == "APPROACH_PICK":
                state = "GO_DOWN_PICK"
                target_pos = list(pick_pos)
            elif state == "GO_DOWN_PICK":
                state = "GO_UP_PICK"
                target_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 50.0]
            elif state == "GO_UP_PICK":
                state = "APPROACH_DROP"
                drop_target = drop_positions[current_color]
                target_pos = [drop_target[0], drop_target[1], drop_target[2] + 50.0]
            elif state == "APPROACH_DROP":
                state = "GO_DOWN_DROP"
                target_pos = drop_positions[current_color]
            elif state == "GO_DOWN_DROP":
                state = "GO_UP_DROP"
                target_pos = [target_pos[0], target_pos[1], target_pos[2] + 50.0]

                # Reset vị trí phôi mới về lại vị trí Bàn chờ gắp ngay sau khi thả khay
                current_color = random.choice(list(drop_positions.keys()))
                item_actor.GetProperty().SetColor(current_color)
                move_item_to(pick_pos[0], pick_pos[1], pick_pos[2] + 6.0)
            elif state == "GO_UP_DROP":
                state = "APPROACH_PICK"
                target_pos = [pick_pos[0], pick_pos[1], pick_pos[2] + 40.0]

        # Cục phôi bám dính di chuyển theo đầu gắp nếu đang được hút/gắp đi phân loại
        if state in ["GO_UP_PICK", "APPROACH_DROP", "GO_DOWN_DROP"]:
            move_item_to(ee_pos[0], ee_pos[1], ee_pos[2] - 6.0)

        update_robot_geometry(ee_pos[0], ee_pos[1], ee_pos[2])
        renderWindow.Render()

if __name__ == "__main__":
    main()