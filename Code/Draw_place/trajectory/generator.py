import numpy as np
from typing import List, Tuple
Point3D = Tuple[float, float, float]
def generate_circle(
    radius: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = -250.0,
    n_points: int = 100,
) -> List[Point3D]:
    if radius <= 0:
        raise ValueError("Bán kính phải > 0")

    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    pts = [(cx + radius * np.cos(a), cy + radius * np.sin(a), z) for a in angles]
    pts.append(pts[0])          # khép kín vòng
    return pts
def generate_square(
    side: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = 250.0,
    n_per_side: int = 20,
) -> List[Point3D]:
    if side <= 0:
        raise ValueError("Độ dài cạnh phải > 0")
    h = side / 2.0
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
    pts.append(pts[0])
    return pts
def generate_triangle(
    side: float,
    cx: float = 0.0,
    cy: float = 0.0,
    z: float = 250.0,
    n_per_side: int = 20,
) -> List[Point3D]:
    if side <= 0:
        raise ValueError("Độ dài cạnh phải > 0")
    R_circ = side / np.sqrt(3)
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

    pts.append(pts[0])
    return pts
