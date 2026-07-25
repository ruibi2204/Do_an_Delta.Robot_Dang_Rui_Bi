import numpy as np
from typing import List, Tuple

Point3D = Tuple[float, float, float]

def generate_line(
    start: Point3D,
    end: Point3D,
    n_points: int = 50,
    include_endpoint: bool = True
) -> List[Point3D]:
    x1, y1, z1 = start
    x2, y2, z2 = end

    if include_endpoint:
        total = n_points
    else:
        total = n_points + 1

    t_values = np.linspace(0, 1, total, endpoint=include_endpoint)

    points = []
    for t in t_values:
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        z = z1 + (z2 - z1) * t
        points.append((x, y, z))

    return points