# ============================================================
#  kinematics/inverse_kinematics.py — Động học ngược Robot Delta
# ============================================================
# pyrefly: ignore [missing-import]
import numpy as np


def inverse_kinematics(Px: float, Py: float, Pz: float):
    """
    Tính động học ngược Robot Delta.

    Tham số:
        Px, Py, Pz : Toạ độ điểm đến của end-effector (mm)

    Trả về:
        (theta1, theta2, theta3) : Góc 3 khớp THỰC TẾ (độ)
        Chưa nhân tỉ số truyền đai — việc đó thực hiện trong uart_comm.py

    Ngoại lệ:
        ValueError nếu điểm nằm ngoài vùng làm việc.
    """

    R = 100.0   # OA_i   — Bán kính mâm tĩnh (mm)
    a = 130.0   # A_iB_i — Chiều dài cánh tay trên (mm)
    b = 298.0   # B_iC_i — Chiều dài cánh tay dưới (mm)
    r = 35.30   # C_iP_i — Bán kính mâm động (mm)

    # Góc phi_i của 3 cánh tay (0°, 120°, 240°)
    phi = np.radians([0.0, 120.0, 240.0])

    theta_list = []

    for i in range(3):
        # 1. Biến đổi toạ độ sang hệ riêng Ox_iy_iz_i của cánh tay thứ i
        cos_phi = np.cos(phi[i])
        sin_phi = np.sin(phi[i])

        Pxi = Px * cos_phi + Py * sin_phi
        Pyi = -Px * sin_phi + Py * cos_phi
        Pzi = Pz

        # Toạ độ điểm khớp C_i
        Cxi = Pxi + r - R
        Cyi = Pyi
        Czi = Pzi

        # 2. Kiểm tra điều kiện vùng làm việc (Thay vì dùng lượng giác)
        if abs(Cyi) > b:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) vượt vùng làm việc cơ khí (cánh {i+1})!")

        # 3. Áp dụng phép thế Weierstrass — giải phương trình bậc 2 (Phương trình 2.11)
        # Tính RHS đã được tối giản triệt để hàm lượng giác
        RHS = Cxi ** 2 + Cyi ** 2 + Czi ** 2 + a ** 2 - b ** 2

        # Sửa lại hệ số A, B, C chuẩn xác theo biến đổi toán học từ hình ảnh
        A_coeff = RHS + 2 * a * Cxi
        
        # Phòng hờ trường hợp A_coeff = 0 gây lỗi chia cho 0 (ZeroDivisionError)
        if A_coeff == 0:
            raise ValueError(f"Điểm kỳ dị toán học tại cánh {i + 1}!")
        B_coeff = -4 * a * Czi
        C_coeff = RHS - 2 * a * Cxi

        delta = B_coeff ** 2 - 4 * A_coeff * C_coeff
        if delta < 0:
            raise ValueError(f"Điểm ({Px},{Py},{Pz}) không hợp lệ về mặt hình học (cánh {i + 1})!")

        # Chọn nghiệm phù hợp cấu hình robot thực tế (tùy thuộc vào robot bạn chọn dấu + hay -)
        t = (-B_coeff - np.sqrt(delta)) / (2 * A_coeff)

        # 4. Tính góc khớp chủ động theta_1i (Phương trình 2.12)
        theta_1i = 2 * np.arctan(t)
        theta_list.append(np.degrees(theta_1i))

    return theta_list[0], theta_list[1], theta_list[2]