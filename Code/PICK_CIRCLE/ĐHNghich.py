import numpy as np
import threading
import math
R = 122
r = 40.0
a = 130.0
b = 298.0
GOC_LAP_DEG = [0.0, 120.0, 240.0]
THETA_OFFSET_DEG = [0.0, 0.0, 0.0]
def inverse_kinematics(Px: float, Py: float, Pz: float):
    phi = np.radians(GOC_LAP_DEG)
    theta_list = []

    for i in range(3):
        cos_phi = np.cos(phi[i])
        sin_phi = np.sin(phi[i])

        # Biến đổi tọa độ về hệ quy chiếu của cánh tay i
        Pxi = Px * cos_phi + Py * sin_phi
        Pyi = -Px * sin_phi + Py * cos_phi
        Pzi = Pz

        Cxi = Pxi + r - R
        Cyi = Pyi
        Czi = Pzi

        # Kiểm tra vùng làm việc cơ bản
        if abs(Cyi) > b:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) vượt vùng làm việc cơ khí (cánh {i+1})!")

        RHS = Cxi**2 + Cyi**2 + Czi**2 + a**2 - b**2
        A_coeff = RHS + 2 * a * Cxi
        if A_coeff == 0:
            raise ValueError(f"Điểm kỳ dị toán học tại cánh {i+1}!")
        B_coeff = -4 * a * Czi
        C_coeff = RHS - 2 * a * Cxi

        delta = B_coeff**2 - 4 * A_coeff * C_coeff
        if delta < 0:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) không hợp lệ về mặt hình học (cánh {i+1})!")

        t = (-B_coeff - np.sqrt(delta)) / (2 * A_coeff)   # Chọn nghiệm nhỏ hơn
        theta_1i = 2 * np.arctan(t)
        theta_deg = np.degrees(theta_1i) + THETA_OFFSET_DEG[i]
        theta_list.append(theta_deg)

    return theta_list[0], theta_list[1], theta_list[2]


if __name__ == "__main__":
    # Test nhanh: điểm home lý thuyết (0,0,300) mm (điều chỉnh theo robot của bạn)
    t1, t2, t3 = inverse_kinematics(0.0, 0.0, 350.0)
    print(f"theta1={t1:.3f}°  theta2={t2:.3f}°  theta3={t3:.3f}°")