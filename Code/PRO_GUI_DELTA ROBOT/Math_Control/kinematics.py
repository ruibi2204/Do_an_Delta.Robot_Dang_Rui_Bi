import numpy as np

# ---- Thong so co khi cua robot (SUA O DAY neu doi kich thuoc robot) ----
R = 100.0
A_ARM = 130.0   # chieu dai canh tren (bien tay quay)
B_ARM = 298.0   # chieu dai canh duoi (thanh truyen)
R_EE = 40.0     # ban kinh mam di dong (r)

# Goc lap dat cua 3 canh quanh truc Z (0, 120, 240 do)
_PHI = np.radians([0.0, 120.0, 240.0])

def inverse_kinematics(Px: float, Py: float, Pz: float):
    """Tinh dong hoc nguoc cho robot delta.

    Tra ve tuple (theta1, theta2, theta3) - goc KHOP tinh bang DO.
    Nem ValueError neu diem (Px, Py, Pz) nam ngoai vung lam viec co khi
    hoac vo nghiem ve mat hinh hoc.
    """
    R_ = R
    a = A_ARM
    b = B_ARM
    r = R_EE
    phi = _PHI

    theta_list = []
    for i in range(3):
        cos_phi = np.cos(phi[i])
        sin_phi = np.sin(phi[i])

        # Doi toa do dau cong tac sang he toa do rieng cua canh i
        Pxi = Px * cos_phi + Py * sin_phi
        Pyi = -Px * sin_phi + Py * cos_phi
        Pzi = Pz

        Cxi = Pxi + r - R_
        Cyi = Pyi
        Czi = Pzi

        if abs(Cyi) > b:
            raise ValueError(
                f"Diem ({Px},{Py},{Pz}) vuot vung lam viec co khi (canh {i + 1})!"
            )

        RHS = Cxi ** 2 + Cyi ** 2 + Czi ** 2 + a ** 2 - b ** 2
        A_coeff = RHS + 2 * a * Cxi
        if A_coeff == 0:
            raise ValueError(f"Diem ky di toan hoc tai canh {i + 1}!")

        B_coeff = -4 * a * Czi
        C_coeff = RHS - 2 * a * Cxi
        delta = B_coeff ** 2 - 4 * A_coeff * C_coeff

        if delta < 0:
            raise ValueError(
                f"Diem ({Px},{Py},{Pz}) khong hop le ve mat hinh hoc (canh {i + 1})!"
            )

        t = (-B_coeff - np.sqrt(delta)) / (2 * A_coeff)
        theta_1i = 2 * np.arctan(t)
        theta_list.append(float(np.degrees(theta_1i)))

    return theta_list[0], theta_list[1], theta_list[2]


if __name__ == "__main__":
    # Test nhanh khi chay truc tiep file nay: python kinematics.py
    test_points = [(0, 0, -250), (20, 10, -260), (-30, -15, -240)]
    for p in test_points:
        try:
            t1, t2, t3 = inverse_kinematics(*p)
            print(f"P={p} -> theta1={t1:.3f}, theta2={t2:.3f}, theta3={t3:.3f}")
        except ValueError as e:
            print(f"P={p} -> LOI: {e}")
