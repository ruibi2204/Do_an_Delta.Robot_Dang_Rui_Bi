import numpy as np
def inverse_kinematics(Px: float, Py: float, Pz: float):
    R = 95.0
    a = 130.0
    b = 298.0
    r = 40
    phi = np.radians([0.0, 120.0, 240.0])
    theta_list = []
    for i in range(3):
        cos_phi = np.cos(phi[i])
        sin_phi = np.sin(phi[i])
        Pxi = Px * cos_phi + Py * sin_phi
        Pyi = -Px * sin_phi + Py * cos_phi
        Pzi = Pz
        Cxi = Pxi + r - R
        Cyi = Pyi
        Czi = Pzi
        if abs(Cyi) > b:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) vượt vùng làm việc cơ khí (cánh {i+1})!")
        RHS = Cxi ** 2 + Cyi ** 2 + Czi ** 2 + a ** 2 - b ** 2
        A_coeff = RHS + 2 * a * Cxi
        if A_coeff == 0:
            raise ValueError(f"Điểm kỳ dị toán học tại cánh {i + 1}!")
        B_coeff = -4 * a * Czi
        C_coeff = RHS - 2 * a * Cxi
        delta = B_coeff ** 2 - 4 * A_coeff * C_coeff
        if delta < 0:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) không hợp lệ về mặt hình học (cánh {i + 1})!")
        t = (-B_coeff - np.sqrt(delta)) / (2 * A_coeff)
        theta_1i = 2 * np.arctan(t)
        theta_list.append(np.degrees(theta_1i))
    return theta_list[0], theta_list[1], theta_list[2]