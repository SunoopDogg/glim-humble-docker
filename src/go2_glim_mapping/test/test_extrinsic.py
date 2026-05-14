import math

import numpy as np
import pytest

from go2_glim_mapping.extrinsic import (
    compose_t_lidar_imu,
    rotation_to_quaternion,
)


def _rz(deg):
    """4x4 row-major homogeneous, rotation about Z by deg, zero translation."""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [c, -s, 0, 0,
            s,  c, 0, 0,
            0,  0, 1, 0,
            0,  0, 0, 1]


def test_identity_metadata_gives_identity_tum():
    md = {'imu_to_sensor_transform': _rz(0), 'lidar_to_sensor_transform': _rz(0)}
    tum = compose_t_lidar_imu(md, frame='os_lidar')
    assert tum == pytest.approx([0, 0, 0, 0, 0, 0, 1], abs=1e-9)


def test_newer_ouster_180deg_z_rotation():
    # LiDAR frame rotated 180 deg about Z relative to sensor; IMU aligned to sensor.
    md = {'imu_to_sensor_transform': _rz(0), 'lidar_to_sensor_transform': _rz(180)}
    tum = compose_t_lidar_imu(md, frame='os_lidar')
    # quaternion ~ [0,0,1,0] (qw=0); translation 0
    assert tum[:3] == pytest.approx([0, 0, 0], abs=1e-9)
    assert abs(tum[3]) == pytest.approx(0, abs=1e-9)   # qx
    assert abs(tum[4]) == pytest.approx(0, abs=1e-9)   # qy
    assert abs(tum[5]) == pytest.approx(1, abs=1e-9)   # qz
    assert abs(tum[6]) == pytest.approx(0, abs=1e-9)   # qw


def test_translation_is_converted_mm_to_m():
    md = {'imu_to_sensor_transform': [1, 0, 0, 1000,
                                      0, 1, 0, 2000,
                                      0, 0, 1, 3000,
                                      0, 0, 0, 1],
          'lidar_to_sensor_transform': _rz(0)}
    tum = compose_t_lidar_imu(md, frame='os_lidar')
    assert tum[:3] == pytest.approx([1.0, 2.0, 3.0], abs=1e-9)


def test_os_sensor_frame_uses_imu_to_sensor_directly():
    md = {'imu_to_sensor_transform': _rz(90), 'lidar_to_sensor_transform': _rz(180)}
    tum = compose_t_lidar_imu(md, frame='os_sensor')
    q = rotation_to_quaternion(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float))
    assert tum[3:] == pytest.approx(list(q), abs=1e-9)


def test_unknown_frame_raises():
    md = {'imu_to_sensor_transform': _rz(0), 'lidar_to_sensor_transform': _rz(0)}
    with pytest.raises(ValueError):
        compose_t_lidar_imu(md, frame='base_link')
