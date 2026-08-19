#!/usr/bin/env python3
# kinematics.py
# Động học nghịch robot Delta

import numpy as np

# ==============================================================
# THAM SỐ CƠ KHÍ - CHỈNH TẠI ĐÂY KHI HIỆU CHỈNH (CALIBRATION)
# ==============================================================
# R: bán kính bệ CỐ ĐỊNH, đo từ TÂM bệ đến TÂM khớp xoay vai (không
#    phải đến mép tấm, không phải đến cạnh tam giác).
# r: bán kính bệ DI ĐỘNG (effector), đo từ TÂM mâm đến TÂM khớp cổ tay.
# a: chiều dài cánh tay trên (bicep), tâm khớp vai -> tâm khớp khuỷu.
# b: chiều dài thanh dưới (rod/forearm song song).
#
# Đây là các thông số dễ sai nhất trong thực tế (xem phân tích trước).
# Nếu robot chạy lệch X,Y theo kiểu co giãn đều theo bán kính,
# hãy hiệu chỉnh lại R và r trước tiên.
R = 122
r = 40.0
a = 130.0
b = 298.0

# GOC_LAP_DEG: góc lắp thực tế của 3 cánh tay quanh tâm (độ).
# Mặc định lý thuyết là [0, 120, 240], nhưng nếu khung cơ khí lắp
# lệch (sai số gia công/lắp ráp), hãy đo góc thực tế và sửa tại đây.
# Dấu hiệu nhận biết: sai số X,Y KHÔNG đối xứng đều 3 hướng mà lệch
# rõ rệt về 1-2 hướng cụ thể.
GOC_LAP_DEG = [0.0, 120.0, 240.0]

# Offset góc "zero" của từng động cơ (độ), cộng thêm vào theta tính
# được trước khi gửi xuống driver, dùng để bù lệch home/encoder.
# Nếu robot lệch X,Y theo kiểu xoay/hệ thống (không tỉ lệ bán kính),
# hãy hiệu chỉnh tại đây thay vì R, r.
THETA_OFFSET_DEG = [0.0, 0.0, 0.0]


def inverse_kinematics(Px: float, Py: float, Pz: float):
    """
    Tính góc khớp (độ) từ tọa độ Cartesian (Px, Py, Pz) [mm].
    Sử dụng các tham số cơ khí ở đầu file: R, r, a, b, GOC_LAP_DEG,
    THETA_OFFSET_DEG.

    Trả về: (theta1, theta2, theta3) [độ]
    Nếu điểm không hợp lệ, ném ngoại lệ với thông báo.
    """
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