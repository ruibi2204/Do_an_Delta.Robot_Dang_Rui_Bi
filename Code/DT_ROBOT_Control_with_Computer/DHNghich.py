import numpy as np
def tinh_dhn(Px, Py, Pz):

    R = 100.0  # OA_i (mm)
    a = 130.0  # A_iB_i (mm)
    b = 298.0  # B_iC_i (mm)
    r = 35.30  # C_iP_i (mm)

    # Góc phi_i của 3 cánh tay
    phi = np.radians([0.0, 120.0, 240.0])

    theta_1_angles = []

    for i in range(3):
        # 1. Biến đổi tọa độ sang hệ riêng Ox_iy_iz_i của cánh thứ i
        cos_phi = np.cos(phi[i])
        sin_phi = np.sin(phi[i])

        Pxi = Px * cos_phi + Py * sin_phi
        Pyi = -Px * sin_phi + Py * cos_phi
        Pzi = Pz

        # Tọa độ điểm khớp C_i
        Cxi = Pxi - r
        Cyi = Pyi
        Czi = Pzi

        # 2. Giải góc phụ theta_3i (Phương trình 2.8)
        cos_theta3 = Cyi / b
        if abs(cos_theta3) > 1.0:
            return None  # Vượt quá vùng làm việc cơ khí

        theta3 = np.arccos(cos_theta3)
        sin_theta3 = np.sin(theta3)

        # 3. Áp dụng phép thế Weierstrass giải phương trình bậc 2 (Phương trình 2.11)
        # Vế phải phương trình (2.10)
        RHS = Cxi ** 2 + Czi ** 2 + a ** 2 - (b ** 2) * (sin_theta3 ** 2)

        # Hệ số phương trình bậc 2: A*t^2 + B*t + C_const = 0
        A_coeff = RHS - 2 * a * (R - Cxi)
        B_coeff = -4 * a * Czi
        C_coeff = RHS - 2 * a * (R + Cxi)

        # Giải tìm nghiệm t
        delta = B_coeff ** 2 - 4 * A_coeff * C_coeff
        if delta < 0:
            return None  # Vị trí không hợp lệ về mặt hình học

        # Chọn nghiệm phù hợp cấu hình robot thực tế
        t = (-B_coeff - np.sqrt(delta)) / (2 * A_coeff)

        # 4. Tính góc khớp chủ động theta_1i theo ĐỘ (Phương trình 2.12)
        theta_1i = 2 * np.arctan(t)*3.7
        theta_1_angles.append(np.degrees(theta_1i))

    return theta_1_angles[0], theta_1_angles[1], theta_1_angles[2]