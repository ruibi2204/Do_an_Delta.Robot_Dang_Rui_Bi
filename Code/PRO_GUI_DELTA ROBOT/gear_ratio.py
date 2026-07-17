
GEAR_RATIO = 3.0  # u = 3 (SUA O DAY neu robot dung ty so truyen khac)


def joint_to_motor_angle(theta_joint_deg: float, gear_ratio: float = GEAR_RATIO) -> float:
    """Doi goc KHOP (do) sang goc TRUC DONG CO (do) can quay."""
    return theta_joint_deg * gear_ratio


def motor_to_joint_angle(theta_motor_deg: float, gear_ratio: float = GEAR_RATIO) -> float:
    """Doi goc TRUC DONG CO (do) nguoc lai goc KHOP (do)."""
    return theta_motor_deg / gear_ratio


def joints_to_motors(theta1: float, theta2: float, theta3: float,
                      gear_ratio: float = GEAR_RATIO):
    """Doi ca 3 goc khop (theta1, theta2, theta3) sang goc dong co tuong ung."""
    return (
        joint_to_motor_angle(theta1, gear_ratio),
        joint_to_motor_angle(theta2, gear_ratio),
        joint_to_motor_angle(theta3, gear_ratio),
    )


def motors_to_joints(m1: float, m2: float, m3: float, gear_ratio: float = GEAR_RATIO):
    """Doi nguoc lai: goc dong co -> goc khop."""
    return (
        motor_to_joint_angle(m1, gear_ratio),
        motor_to_joint_angle(m2, gear_ratio),
        motor_to_joint_angle(m3, gear_ratio),
    )


if __name__ == "__main__":
    t1, t2, t3 = 12.5, -8.3, 20.0
    m1, m2, m3 = joints_to_motors(t1, t2, t3)
    print(f"Joint: {t1}, {t2}, {t3}  ->  Motor (u={GEAR_RATIO}): {m1}, {m2}, {m3}")
