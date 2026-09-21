# pyrefly: ignore [missing-import]
"""
Mô phỏng Robot Delta 3D (VTK) - PHIÊN BẢN CHUYỂN ĐỘNG & HÌNH ẢNH CHÂN THỰC HƠN
================================================================================
So với bản gốc, file này cải tiến 2 nhóm chính mà không đổi công thức động
học nghịch (delta_calcInverse) - vẫn ra đúng góc khớp như cũ:

1) CHUYỂN ĐỘNG CHÂN THỰC HƠN
   - Vòng lặp render cũ dùng "while True: interactor.ProcessEvents()" chạy
     nhanh hết mức CPU cho phép -> tốc độ mô phỏng phụ thuộc máy, giật/lag
     tùy thời điểm. Bản mới dùng TIMER THẬT của VTK (CreateRepeatingTimer)
     + đo thời gian thực (dt) mỗi khung hình -> tốc độ luôn ổn định, đúng
     đơn vị mm/giây, không phụ thuộc cấu hình máy.
   - Trước đây robot di chuyển với TỐC ĐỘ HẰNG SỐ và đổi hướng đột ngột tại
     mỗi điểm mốc (rất "máy móc"/giật). Bản mới áp dụng PROFILE TĂNG/GIẢM
     TỐC (trapezoidal, gia tốc không đổi) cho từng đoạn di
     chuyển: robot tăng tốc từ từ, chạy đều ở giữa, rồi giảm tốc êm khi
     sắp tới điểm đích - giống hệt chuyển động thật của robot công nghiệp.
   - Thêm các bước DỪNG NGẮN (dwell) để "gắp" và "thả" vật, thay vì gắp/thả
     tức thời như trước - có hoạt cảnh đầu hút co/giãn nhẹ khi gắp/thả.
   - Vật sau khi thả được GIỮ LẠI trong khay (tối đa N vật/khay, vật cũ
     nhất tự động biến mất khi khay đầy) thay vì biến mất ngay lập tức,
     mô phỏng đúng quá trình phân loại tích lũy vật trong thực tế.
   - Vị trí gắp/thả có thêm nhiễu ngẫu nhiên nhỏ mỗi chu kỳ để trông tự
     nhiên hơn, thay vì lặp lại y hệt một quỹ đạo mãi mãi.

2) HÌNH ẢNH ROBOT CHÂN THỰC HƠN
   - Vật liệu (Ambient/Diffuse/Specular/SpecularPower) được set riêng cho
     từng nhóm chi tiết: đế nhựa mờ, tay trên sơn kim loại bóng, thanh
     song song carbon mờ đen, khớp cầu inox sáng bóng, đầu gắp nhựa mờ.
   - Thêm 3 KHỐI ĐỘNG CƠ (mô phỏng servo) tại 3 vai của robot - trước đây
     chỉ có tay + khớp, thiếu phần "động cơ" nên nhìn thiếu chi tiết.
   - Thêm ĐẦU HÚT CHÂN KHÔNG (vòi + đế cao su) ở tâm đầu gắp, có hoạt cảnh
     co lại khi gắp và giãn ra khi thả.
   - Thêm sàn nền + khử răng cưa (anti-aliasing, multi-sampling) cho hình
     ảnh mượt hơn; thêm đèn phụ (fill light) dịu hơn để đổ bóng tự nhiên
     hơn thay vì chỉ có 1 đèn chiếu thẳng gây bệt màu; thử bật đổ bóng
     (shadow map) - nếu bản VTK cài trên máy không hỗ trợ thì tự bỏ qua,
     không làm crash chương trình.
"""

import collections
import math
import random
import time

# pyrefly: ignore [missing-import]
import vtk

# --- THÔNG SỐ CƠ KHÍ ROBOT DELTA VÀ QUY ƯỚC CHUẨN (Đơn vị: mm) ---
sb = 160.0  # Cạnh tam giác đều của Đế cố định (Base)
sp = 50.0   # Cạnh tam giác đều của Đầu gắp di động (Platform)
L1 = 110.0  # Chiều dài Cánh tay trên chủ động (Upper Arm)
L2 = 240.0  # Chiều dài Cánh tay dưới song song (Lower Arm)

f = sb / math.sqrt(3)
e = sp / math.sqrt(3)
ALPHA = [0.0, 120.0, 240.0]

# --- THAM SỐ CHUYỂN ĐỘNG (mm/s) - dùng cho profile tăng/giảm tốc ---
SPEED_TRAVEL = 260.0     # tốc độ di chuyển ngang/đi xa (nhanh)
SPEED_APPROACH = 150.0   # tốc độ khi lên/xuống ở vùng an toàn (vừa)
SPEED_ENGAGE = 90.0      # tốc độ khi hạ xuống gắp/thả (chậm, chính xác)
DWELL_GRIP_TIME = 0.28   # thời gian dừng để "hút" vật (giây)
DWELL_RELEASE_TIME = 0.28  # thời gian dừng để "nhả" vật (giây)
MAX_ITEMS_PER_TRAY = 8   # số vật tối đa tích lũy trong 1 khay trước khi vật cũ nhất biến mất


# --- HÀM TÍNH KINEMATICS NGƯỢC CHUẨN ĐỒ ĐỒNG BỘ (GIỮ NGUYÊN CÔNG THỨC GỐC) ---
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


# --- HÀM GÁN VẬT LIỆU (Ambient/Diffuse/Specular) CHO ẤN TƯỢNG CHÂN THỰC HƠN ---
def set_material(actor, color, ambient=0.25, diffuse=0.75, specular=0.25, specular_power=20.0):
    prop = actor.GetProperty()
    prop.SetColor(color)
    prop.SetAmbient(ambient)
    prop.SetDiffuse(diffuse)
    prop.SetSpecular(specular)
    prop.SetSpecularPower(specular_power)
    return actor


def dist3(p1, p2):
    return math.hypot(p1[0] - p2[0], math.hypot(p1[1] - p2[1], p1[2] - p2[2]))


class MotionSegment:
    """Quản lý 1 đoạn di chuyển thẳng từ start -> target theo PROFILE HÌNH
    THANG CHUẨN (trapezoidal velocity profile) - đúng như cách robot công
    nghiệp thực tế di chuyển: tăng tốc với gia tốc không đổi, chạy đều ở
    tốc độ tối đa, rồi giảm tốc với cùng gia tốc để dừng êm tại đích. Tốc
    độ được tính theo THỜI GIAN đã trôi qua trong đoạn (không phải theo
    quãng đường đã đi), tránh hiện tượng "ì" ở đầu đoạn."""

    def __init__(self, start, target, max_speed, accel=900.0):
        self.start = tuple(start)
        self.target = tuple(target)
        self.accel = accel
        self.length = dist3(self.start, self.target)
        if self.length > 1e-6:
            self.direction = tuple((self.target[i] - self.start[i]) / self.length for i in range(3))
        else:
            self.direction = (0.0, 0.0, 0.0)

        L = self.length
        if L <= 1e-6:
            self.total_time = 0.0
            self.t_acc = 0.0
            self.t_cruise_end = 0.0
            self.peak_speed = 0.0
            self.d_acc = 0.0
        else:
            t_acc_full = max_speed / accel
            d_acc_full = 0.5 * accel * t_acc_full * t_acc_full
            if 2.0 * d_acc_full <= L:
                # Đủ dài để đạt tốc độ tối đa: tăng tốc - chạy đều - giảm tốc.
                self.peak_speed = max_speed
                self.t_acc = t_acc_full
                self.d_acc = d_acc_full
                cruise_dist = L - 2.0 * d_acc_full
                cruise_time = cruise_dist / max_speed
                self.t_cruise_end = self.t_acc + cruise_time
                self.total_time = self.t_cruise_end + self.t_acc
            else:
                # Đoạn quá ngắn để đạt tốc độ tối đa: profile tam giác
                # (chỉ tăng tốc rồi giảm tốc ngay, không có đoạn chạy đều).
                peak = math.sqrt(accel * L)
                self.peak_speed = peak
                self.t_acc = peak / accel
                self.d_acc = L / 2.0
                self.t_cruise_end = self.t_acc
                self.total_time = 2.0 * self.t_acc

        self.elapsed = 0.0

    def advance(self, dt):
        """Tiến thêm dt giây, trả về True nếu đã tới đích."""
        self.elapsed += dt
        if self.elapsed >= self.total_time:
            self.elapsed = self.total_time
            return True
        return False

    def _distance_at(self, t):
        if self.length <= 1e-6:
            return 0.0
        if t <= self.t_acc:
            return 0.5 * self.accel * t * t
        elif t <= self.t_cruise_end:
            return self.d_acc + self.peak_speed * (t - self.t_acc)
        else:
            t_dec = max(0.0, self.total_time - t)
            return self.length - 0.5 * self.accel * t_dec * t_dec

    def position(self):
        s = self._distance_at(self.elapsed)
        return tuple(self.start[i] + self.direction[i] * s for i in range(3))


# --- CHƯƠNG TRÌNH CHÍNH ---
def main():
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.90, 0.92, 0.95)
    renderer.SetBackground2(0.78, 0.82, 0.88)
    renderer.GradientBackgroundOn()

    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1100, 750)
    renderWindow.SetWindowName("Delta Robot 3D Simulation - Phân loại màu sắc (chuyển động chân thực)")
    # Khử răng cưa để hình ảnh mượt, đỡ "gãy khúc" ở các cạnh cong.
    renderWindow.SetMultiSamples(8)

    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)
    interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    # ---- Ánh sáng: 1 đèn chính (key light) + 1 đèn phụ dịu (fill light) +
    # ánh sáng môi trường nhẹ, thay vì chỉ 1 đèn duy nhất gây bệt màu/bóng
    # gắt như bản gốc. Tắt đèn tự động mặc định của VTK (headlight) để
    # toàn quyền kiểm soát ánh sáng cho đúng ý đồ.
    renderer.AutomaticLightCreationOff()

    key_light = vtk.vtkLight()
    key_light.SetLightTypeToSceneLight()
    key_light.SetFocalPoint(0, 0, -200)
    key_light.SetPosition(350, 450, 550)
    key_light.SetIntensity(1.0)
    key_light.SetColor(1.0, 0.98, 0.94)
    renderer.AddLight(key_light)

    fill_light = vtk.vtkLight()
    fill_light.SetLightTypeToSceneLight()
    fill_light.SetFocalPoint(0, 0, -200)
    fill_light.SetPosition(-400, -300, 250)
    fill_light.SetIntensity(0.35)
    fill_light.SetColor(0.75, 0.82, 1.0)
    renderer.AddLight(fill_light)

    # Thử bật đổ bóng (shadow map). Không phải bản VTK/driver GPU nào
    # cũng hỗ trợ - bọc trong try/except để nếu lỗi thì bỏ qua êm, không
    # làm crash chương trình (chỉ mất hiệu ứng bóng đổ, mọi thứ khác vẫn
    # chạy bình thường).
    try:
        shadows_baker = vtk.vtkShadowMapBakerPass()
        shadows_pass = vtk.vtkShadowMapPass()
        shadows_pass.SetShadowMapBakerPass(shadows_baker)

        seq = vtk.vtkSequencePass()
        passes = vtk.vtkRenderPassCollection()
        passes.AddItem(shadows_baker)
        passes.AddItem(shadows_pass)
        seq.SetPasses(passes)

        camera_pass = vtk.vtkCameraPass()
        camera_pass.SetDelegatePass(seq)
        renderer.SetPass(camera_pass)
    except Exception:
        pass

    # 1. Đế trên cố định (Fixed Base) - vật liệu nhựa mờ đen
    base_mesh = vtk.vtkCylinderSource()
    base_mesh.SetRadius(f * 1.3)
    base_mesh.SetHeight(15)
    base_mesh.SetResolution(48)
    base_mapper = vtk.vtkPolyDataMapper()
    base_mapper.SetInputConnection(base_mesh.GetOutputPort())
    base_actor = vtk.vtkActor()
    base_actor.SetMapper(base_mapper)
    set_material(base_actor, (0.13, 0.13, 0.14), ambient=0.25, diffuse=0.8, specular=0.35, specular_power=25)

    t_base = vtk.vtkTransform()
    t_base.RotateX(90)
    t_base.RotateY(30)
    base_actor.SetUserTransform(t_base)
    renderer.AddActor(base_actor)

    # 1b. 3 khối ĐỘNG CƠ (servo) tại 3 vai của robot - chi tiết còn thiếu
    # ở bản gốc, giúp robot nhìn "có động lực" thay vì chỉ có tay không.
    # Các điểm vai B_i KHÔNG phụ thuộc vị trí đầu gắp nên chỉ cần tạo 1 lần.
    motor_actors = []
    for i in range(3):
        alpha_rad = math.radians(ALPHA[i])
        Bx, By = f * math.cos(alpha_rad), f * math.sin(alpha_rad)

        motor = create_cylinder_actor()
        set_material(motor, (0.08, 0.08, 0.09), ambient=0.2, diffuse=0.7, specular=0.55, specular_power=45)
        motor_h = 34.0
        motor_source = motor.GetMapper().GetInputConnection(0, 0).GetProducer()
        motor_source.SetHeight(motor_h)
        motor_source.SetRadius(11.0)
        t_motor = vtk.vtkTransform()
        t_motor.Translate(Bx, By, motor_h / 2.0)
        t_motor.RotateX(90)
        motor.SetUserTransform(t_motor)
        renderer.AddActor(motor)
        motor_actors.append(motor)

        # Vòng đai màu cam nhỏ ở đáy động cơ (chi tiết "cổ trục") cho có
        # điểm nhấn màu, giống các servo thực tế thường có tem/viền màu.
        collar = create_cylinder_actor()
        set_material(collar, (0.85, 0.35, 0.05), ambient=0.3, diffuse=0.7, specular=0.4, specular_power=25)
        collar_source = collar.GetMapper().GetInputConnection(0, 0).GetProducer()
        collar_source.SetHeight(4.0)
        collar_source.SetRadius(11.5)
        t_collar = vtk.vtkTransform()
        t_collar.Translate(Bx, By, 3.0)
        t_collar.RotateX(90)
        collar.SetUserTransform(t_collar)
        renderer.AddActor(collar)

    # 2. Bàn máy làm việc (Working Table) - vật liệu nhựa/nhôm mờ
    table = vtk.vtkCubeSource()
    table.SetBounds(-300, 300, -300, 300, -350, -340)
    table_mapper = vtk.vtkPolyDataMapper()
    table_mapper.SetInputConnection(table.GetOutputPort())
    table_actor = vtk.vtkActor()
    table_actor.SetMapper(table_mapper)
    set_material(table_actor, (0.62, 0.63, 0.65), ambient=0.3, diffuse=0.75, specular=0.15, specular_power=8)
    renderer.AddActor(table_actor)

    # 2b. Sàn nền lớn hơn phía dưới bàn máy, tạo chiều sâu không gian và
    # làm nền tương phản cho robot - bản gốc chỉ có nền màu phẳng tuyệt đối.
    floor = vtk.vtkCubeSource()
    floor.SetBounds(-900, 900, -900, 900, -352, -351)
    floor_mapper = vtk.vtkPolyDataMapper()
    floor_mapper.SetInputConnection(floor.GetOutputPort())
    floor_actor = vtk.vtkActor()
    floor_actor.SetMapper(floor_mapper)
    set_material(floor_actor, (0.80, 0.81, 0.83), ambient=0.35, diffuse=0.7, specular=0.05, specular_power=4)
    renderer.AddActor(floor_actor)

    # Các khay chứa vật phân loại
    drop_positions = {
        (0.86, 0.20, 0.27): (140, -80, -335),  # Đỏ
        (0.16, 0.65, 0.27): (140, 30, -335),   # Xanh lá
        (0.0, 0.48, 1.0): (140, 140, -335),    # Xanh dương
    }
    for color, pos in drop_positions.items():
        box = vtk.vtkCubeSource()
        box.SetBounds(pos[0] - 25, pos[0] + 25, pos[1] - 25, pos[1] + 25, -340, -332)
        b_mapper = vtk.vtkPolyDataMapper()
        b_mapper.SetInputConnection(box.GetOutputPort())
        b_actor = vtk.vtkActor()
        b_actor.SetMapper(b_mapper)
        set_material(b_actor, color, ambient=0.35, diffuse=0.65, specular=0.3, specular_power=15)
        b_actor.GetProperty().SetOpacity(0.45)
        renderer.AddActor(b_actor)

    # 3. Khâu liên kết cấu trúc cơ khí
    robot_components = {}
    for i in range(3):
        # Upper Arm - sơn kim loại xanh bóng
        robot_components[f'upper_{i}'] = create_cylinder_actor()
        set_material(robot_components[f'upper_{i}'], (0.08, 0.42, 0.78),
                     ambient=0.2, diffuse=0.7, specular=0.6, specular_power=50)
        renderer.AddActor(robot_components[f'upper_{i}'])

        # Lower Arm cặp song song - thanh carbon mờ, gần như đen
        for j in range(2):
            robot_components[f'lower_{i}_{j}'] = create_cylinder_actor()
            set_material(robot_components[f'lower_{i}_{j}'], (0.12, 0.12, 0.13),
                         ambient=0.18, diffuse=0.75, specular=0.25, specular_power=15)
            renderer.AddActor(robot_components[f'lower_{i}_{j}'])

            # Khớp cầu - inox sáng bóng (đặc trưng khớp bi thật)
            robot_components[f'joint_u_{i}_{j}'] = create_sphere_actor(radius=5.5, color=(0.75, 0.76, 0.78))
            robot_components[f'joint_l_{i}_{j}'] = create_sphere_actor(radius=5.5, color=(0.75, 0.76, 0.78))
            for jname in (f'joint_u_{i}_{j}', f'joint_l_{i}_{j}'):
                set_material(robot_components[jname], (0.75, 0.76, 0.78),
                             ambient=0.15, diffuse=0.5, specular=0.9, specular_power=90)
            renderer.AddActor(robot_components[f'joint_u_{i}_{j}'])
            renderer.AddActor(robot_components[f'joint_l_{i}_{j}'])

    # 4. Đầu gắp tam giác (End-Effector Platform) - nhựa mờ đen
    ee_mesh = vtk.vtkCylinderSource()
    ee_mesh.SetRadius(e * 1.3)
    ee_mesh.SetHeight(8)
    ee_mesh.SetResolution(48)
    ee_mapper = vtk.vtkPolyDataMapper()
    ee_mapper.SetInputConnection(ee_mesh.GetOutputPort())
    ee_actor = vtk.vtkActor()
    ee_actor.SetMapper(ee_mapper)
    set_material(ee_actor, (0.16, 0.16, 0.17), ambient=0.22, diffuse=0.75, specular=0.4, specular_power=30)
    renderer.AddActor(ee_actor)

    # 4b. ĐẦU HÚT CHÂN KHÔNG (gripper) ở tâm đầu gắp - vòi kim loại mảnh +
    # đế cao su tròn ở mũi. Chi tiết này bản gốc chưa có (đầu gắp trước
    # đây chỉ là 1 đĩa trơn, thiếu "cơ cấu gắp" thực tế).
    nozzle_actor = create_cylinder_actor()
    set_material(nozzle_actor, (0.35, 0.35, 0.37), ambient=0.2, diffuse=0.7, specular=0.5, specular_power=40)
    nozzle_source = nozzle_actor.GetMapper().GetInputConnection(0, 0).GetProducer()
    NOZZLE_LEN = 22.0
    nozzle_source.SetHeight(NOZZLE_LEN)
    nozzle_source.SetRadius(3.2)
    renderer.AddActor(nozzle_actor)

    cup_actor = create_cylinder_actor()
    set_material(cup_actor, (0.10, 0.10, 0.11), ambient=0.25, diffuse=0.8, specular=0.15, specular_power=8)
    cup_source = cup_actor.GetMapper().GetInputConnection(0, 0).GetProducer()
    CUP_HEIGHT = 5.0
    cup_source.SetHeight(CUP_HEIGHT)
    cup_source.SetRadius(8.5)
    renderer.AddActor(cup_actor)

    def update_gripper(x0, y0, z0, cup_squeeze=1.0):
        """Đặt vòi hút + đế cao su ngay dưới tâm đầu gắp. cup_squeeze < 1.0
        mô phỏng đế cao su hơi co lại khi đang hút/nhả vật (hiệu ứng nhỏ
        nhưng giúp cảm giác "đang thao tác" chân thực hơn)."""
        t_nozzle = vtk.vtkTransform()
        t_nozzle.Translate(x0, y0, z0 - NOZZLE_LEN / 2.0)
        t_nozzle.RotateX(90)
        nozzle_actor.SetUserTransform(t_nozzle)

        t_cup = vtk.vtkTransform()
        t_cup.Translate(x0, y0, z0 - NOZZLE_LEN - CUP_HEIGHT / 2.0)
        t_cup.RotateX(90)
        t_cup.Scale(cup_squeeze, 1.0, cup_squeeze)
        cup_actor.SetUserTransform(t_cup)

    # 5. Khởi tạo phôi phân loại mẫu (vật đang được xử lý)
    item_source = vtk.vtkCylinderSource()
    item_source.SetRadius(10)
    item_source.SetHeight(12)
    item_source.SetResolution(32)
    item_mapper = vtk.vtkPolyDataMapper()
    item_mapper.SetInputConnection(item_source.GetOutputPort())
    item_actor = vtk.vtkActor()
    item_actor.SetMapper(item_mapper)
    set_material(item_actor, (0.5, 0.5, 0.5), ambient=0.3, diffuse=0.7, specular=0.35, specular_power=25)
    renderer.AddActor(item_actor)

    def move_item_to(x, y, z):
        """Gộp chung phép tịnh tiến và phép xoay khống chế để Actor cập nhật
        chuẩn xác (giữ phôi hình trụ đứng thẳng)."""
        t = vtk.vtkTransform()
        t.Translate(x, y, z)
        t.RotateX(90)
        item_actor.SetUserTransform(t)

    # 5b. Danh sách các vật ĐÃ ĐƯỢC PHÂN LOẠI, nằm lại trong khay (thay vì
    # biến mất ngay khi thả như bản gốc) - mô phỏng đúng thực tế "vật tích
    # lũy dần trong khay". Giới hạn số lượng để không phình bộ nhớ vô hạn.
    dropped_actors = collections.deque()

    def spawn_dropped_marker(color, drop_center):
        jitter_x = random.uniform(-14.0, 14.0)
        jitter_y = random.uniform(-14.0, 14.0)
        spin = random.uniform(0.0, 360.0)

        marker_source = vtk.vtkCylinderSource()
        marker_source.SetRadius(10)
        marker_source.SetHeight(12)
        marker_source.SetResolution(20)
        marker_mapper = vtk.vtkPolyDataMapper()
        marker_mapper.SetInputConnection(marker_source.GetOutputPort())
        marker_actor = vtk.vtkActor()
        marker_actor.SetMapper(marker_mapper)
        set_material(marker_actor, color, ambient=0.3, diffuse=0.7, specular=0.3, specular_power=20)

        t_marker = vtk.vtkTransform()
        t_marker.Translate(drop_center[0] + jitter_x, drop_center[1] + jitter_y, drop_center[2] + 6.0)
        t_marker.RotateZ(spin)
        t_marker.RotateX(90)
        marker_actor.SetUserTransform(t_marker)

        renderer.AddActor(marker_actor)
        dropped_actors.append(marker_actor)
        if len(dropped_actors) > MAX_ITEMS_PER_TRAY * len(drop_positions):
            oldest = dropped_actors.popleft()
            renderer.RemoveActor(oldest)

    # --- HÀM CẬP NHẬT ĐỘNG HỌC TOÀN CỤC (GIỮ NGUYÊN CÔNG THỨC GỐC) ---
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

    # =================================================================
    # MÁY TRẠNG THÁI CHUYỂN ĐỘNG: mỗi chu kỳ gắp-thả gồm 8 bước thay vì
    # 6 bước như bản gốc - có thêm 2 bước DỪNG (dwell) để gắp/thả, và mỗi
    # đoạn di chuyển có tốc độ tăng/giảm êm (xem lớp MotionSegment).
    # =================================================================
    BASE_PICK_POS = (-120, -20, -334)  # Tọa độ thực tế phôi nằm trên mặt bàn xám

    class Animator:
        def __init__(self):
            self.current_color = random.choice(list(drop_positions.keys()))
            self.pick_pos = self._jittered_pick()
            item_actor.GetProperty().SetColor(self.current_color)
            move_item_to(self.pick_pos[0], self.pick_pos[1], self.pick_pos[2] + 6.0)

            self.ee_pos = (0.0, 0.0, -180.0)
            self.carrying = False
            self.cup_squeeze = 1.0
            self.drop_target = None  # được chốt khi bắt đầu ASCEND_PICK

            self.state = "APPROACH_PICK"
            self.segment = MotionSegment(
                self.ee_pos,
                (self.pick_pos[0], self.pick_pos[1], self.pick_pos[2] + 40.0),
                SPEED_TRAVEL,
            )
            self.dwell_elapsed = 0.0

        @staticmethod
        def _jittered_pick():
            return (
                BASE_PICK_POS[0] + random.uniform(-8.0, 8.0),
                BASE_PICK_POS[1] + random.uniform(-8.0, 8.0),
                BASE_PICK_POS[2],
            )

        def _start_segment(self, target, speed, next_state):
            self.segment = MotionSegment(self.ee_pos, target, speed)
            self.state = next_state

        def _start_dwell(self, next_state):
            self.dwell_elapsed = 0.0
            self.state = next_state

        def step(self, dt):
            if self.state in ("DWELL_GRIP", "DWELL_RELEASE"):
                self.dwell_elapsed += dt
                duration = DWELL_GRIP_TIME if self.state == "DWELL_GRIP" else DWELL_RELEASE_TIME
                progress = min(1.0, self.dwell_elapsed / duration)
                # Đế cao su co nhẹ giữa chừng dwell rồi giãn lại - mô phỏng
                # động tác "hút/nhả" thay vì đứng yên vô hồn.
                self.cup_squeeze = 1.0 - 0.22 * math.sin(progress * math.pi)

                if progress >= 1.0:
                    if self.state == "DWELL_GRIP":
                        self.carrying = True
                        target = (self.pick_pos[0], self.pick_pos[1], self.pick_pos[2] + 50.0)
                        self._start_segment(target, SPEED_APPROACH, "ASCEND_PICK")
                    else:  # DWELL_RELEASE
                        spawn_dropped_marker(self.current_color, self.drop_target)
                        self.carrying = False
                        # Chuẩn bị phôi mới cho chu kỳ kế tiếp.
                        self.current_color = random.choice(list(drop_positions.keys()))
                        self.pick_pos = self._jittered_pick()
                        item_actor.GetProperty().SetColor(self.current_color)
                        move_item_to(self.pick_pos[0], self.pick_pos[1], self.pick_pos[2] + 6.0)

                        target = (self.drop_target[0], self.drop_target[1], self.drop_target[2] + 50.0)
                        self._start_segment(target, SPEED_APPROACH, "ASCEND_DROP")
                return

            # Các state còn lại đều là di chuyển theo MotionSegment.
            reached = self.segment.advance(dt)
            self.ee_pos = self.segment.position()

            if reached:
                if self.state == "APPROACH_PICK":
                    self._start_segment(self.pick_pos, SPEED_ENGAGE, "DESCEND_PICK")
                elif self.state == "DESCEND_PICK":
                    self._start_dwell("DWELL_GRIP")
                elif self.state == "ASCEND_PICK":
                    base_drop = drop_positions[self.current_color]
                    jitter = (random.uniform(-14.0, 14.0), random.uniform(-14.0, 14.0))
                    self.drop_target = (base_drop[0] + jitter[0], base_drop[1] + jitter[1], base_drop[2])
                    target = (self.drop_target[0], self.drop_target[1], self.drop_target[2] + 50.0)
                    self._start_segment(target, SPEED_TRAVEL, "APPROACH_DROP")
                elif self.state == "APPROACH_DROP":
                    self._start_segment(self.drop_target, SPEED_ENGAGE, "DESCEND_DROP")
                elif self.state == "DESCEND_DROP":
                    self._start_dwell("DWELL_RELEASE")
                elif self.state == "ASCEND_DROP":
                    target = (self.pick_pos[0], self.pick_pos[1], self.pick_pos[2] + 40.0)
                    self._start_segment(target, SPEED_TRAVEL, "APPROACH_PICK")

            # Vật bám theo đầu gắp trong lúc đang "được hút đi phân loại".
            if self.carrying:
                move_item_to(self.ee_pos[0], self.ee_pos[1], self.ee_pos[2] - 6.0)

    animator = Animator()

    # =================================================================
    # VÒNG LẶP RENDER: dùng TIMER THẬT của VTK (đúng chuẩn, không block,
    # vẫn xử lý được tương tác chuột/phím của người dùng mượt mà) thay vì
    # vòng lặp while True thủ công của bản gốc.
    # =================================================================
    state_time = {"last": time.perf_counter()}

    def on_timer(obj, event):
        now = time.perf_counter()
        dt = now - state_time["last"]
        state_time["last"] = now
        # Giới hạn dt để tránh bước nhảy lớn bất thường (ví dụ khi cửa sổ
        # bị kéo/resize khiến 1 khung hình mất nhiều thời gian xử lý).
        dt = min(dt, 0.05)

        animator.step(dt)
        update_robot_geometry(animator.ee_pos[0], animator.ee_pos[1], animator.ee_pos[2])
        update_gripper(animator.ee_pos[0], animator.ee_pos[1], animator.ee_pos[2], animator.cup_squeeze)

        renderWindow.Render()

    interactor.Initialize()
    interactor.AddObserver('TimerEvent', on_timer)
    interactor.CreateRepeatingTimer(16)  # ~60 khung hình/giây

    renderWindow.Render()
    interactor.Start()


if __name__ == "__main__":
    main()