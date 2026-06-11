# ============================================================
#  trajectory/generator.py — Sinh quỹ đạo Robot Delta
# ============================================================
"""
Mỗi hàm trả về list các điểm [(Px, Py, Pz), ...] trong không gian
Cartesian (mm) mà end-effector cần đi qua.

Quy ước trục:
    X, Y : mặt phẳng nằm ngang
    Z    : âm hướng xuống dưới (vùng làm việc thường -200 ~ -350 mm)
"""

import numpy as np
from typing import List, Tuple

Point3D = Tuple[float, float, float]


# ─────────────────────────────────────────────
# 1. HÌNH TRÒN
# ─────────────────────────────────────────────
def generate_circle(
    radius: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = -250.0,
    n_points: int = 100,
) -> List[Point3D]:
    """
    Sinh quỹ đạo hình tròn.

    Tham số:
        radius   : Bán kính (mm)
        cx, cy   : Tâm hình tròn trong mặt phẳng XY (mm)
        z        : Độ cao cố định của end-effector (mm, âm)
        n_points : Số điểm nội suy trên vòng tròn

    Trả về:
        List[Point3D] — điểm đầu == điểm cuối (khép kín)
    """
    if radius <= 0:
        raise ValueError("Bán kính phải > 0")

    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    pts = [(cx + radius * np.cos(a), cy + radius * np.sin(a), z) for a in angles]
    pts.append(pts[0])          # khép kín vòng
    return pts


# ─────────────────────────────────────────────
# 2. HÌNH VUÔNG
# ─────────────────────────────────────────────
def generate_square(
    side: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = -250.0,
    n_per_side: int = 20,
) -> List[Point3D]:
    """
    Sinh quỹ đạo hình vuông (cạnh song song với trục X, Y).

    Tham số:
        side       : Độ dài một cạnh (mm)
        cx, cy     : Tâm hình vuông (mm)
        z          : Độ cao cố định (mm)
        n_per_side : Số điểm nội suy trên mỗi cạnh

    Trả về:
        List[Point3D] — khép kín (điểm đầu == điểm cuối)
    """
    if side <= 0:
        raise ValueError("Độ dài cạnh phải > 0")

    h = side / 2.0
    # 4 đỉnh theo chiều ngược kim đồng hồ bắt đầu từ góc dưới-trái
    corners = [
        (cx - h, cy - h),
        (cx + h, cy - h),
        (cx + h, cy + h),
        (cx - h, cy + h),
    ]

    pts: List[Point3D] = []
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        ts = np.linspace(0, 1, n_per_side, endpoint=False)
        for t in ts:
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, z))

    pts.append(pts[0])          # khép kín
    return pts


# ─────────────────────────────────────────────
# 3. TAM GIÁC ĐỀU
# ─────────────────────────────────────────────
def generate_triangle(
    side: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = -250.0,
    n_per_side: int = 20,
) -> List[Point3D]:
    """
    Sinh quỹ đạo tam giác đều, một đỉnh hướng lên trên (+Y).

    Tham số:
        side       : Độ dài một cạnh (mm)
        cx, cy     : Tâm tam giác (mm)
        z          : Độ cao cố định (mm)
        n_per_side : Số điểm nội suy trên mỗi cạnh

    Trả về:
        List[Point3D] — khép kín
    """
    if side <= 0:
        raise ValueError("Độ dài cạnh phải > 0")

    # Bán kính ngoại tiếp
    R_circ = side / np.sqrt(3)

    # 3 đỉnh, đỉnh đầu tiên hướng lên (+Y), xoay ngược chiều kim đồng hồ
    corners = [
        (cx + R_circ * np.cos(np.pi / 2 + i * 2 * np.pi / 3),
         cy + R_circ * np.sin(np.pi / 2 + i * 2 * np.pi / 3))
        for i in range(3)
    ]

    pts: List[Point3D] = []
    for i in range(3):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 3]
        ts = np.linspace(0, 1, n_per_side, endpoint=False)
        for t in ts:
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, z))

    pts.append(pts[0])          # khép kín
    return pts
