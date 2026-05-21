import math

from go2_rgb_odom_recorder.pose_math import quat_to_yaw


def test_identity_is_zero_yaw():
    assert quat_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_ninety_deg_about_z():
    s = math.sqrt(0.5)  # sin(45deg) = cos(45deg)
    assert math.isclose(quat_to_yaw(0.0, 0.0, s, s), math.pi / 2, abs_tol=1e-9)


def test_one_eighty_about_z():
    # qz=1, qw=0 (the 180deg-Z case real Ouster units exhibit)
    assert math.isclose(quat_to_yaw(0.0, 0.0, 1.0, 0.0), math.pi, abs_tol=1e-9)


def test_negative_ninety_about_z():
    s = math.sqrt(0.5)
    assert math.isclose(quat_to_yaw(0.0, 0.0, -s, s), -math.pi / 2, abs_tol=1e-9)
