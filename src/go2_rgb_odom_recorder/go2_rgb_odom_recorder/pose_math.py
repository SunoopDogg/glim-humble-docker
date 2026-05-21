"""Pure-logic pose helpers (no ROS imports, host-unit-testable)."""
import math


def quat_to_yaw(x, y, z, w):
    """Yaw (rotation about Z, rad) from a quaternion. Range (-pi, pi].

    Standard ZYX-yaw extraction: atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
