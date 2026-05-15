"""Derive GLIM's T_lidar_imu (TUM [x,y,z,qx,qy,qz,qw]) from Ouster sensor metadata.

GLIM defines T_lidar_imu as the transform from the IMU frame to the LiDAR frame.
Ouster metadata provides 4x4 row-major homogeneous transforms with translation in
MILLIMETERS:
  imu_to_sensor_transform   : T_sensor_imu   (IMU   -> sensor frame)
  lidar_to_sensor_transform : T_sensor_lidar (LiDAR -> sensor frame)

ouster-ros publishes points in os_lidar by default (point_cloud_frame); some setups
use os_sensor. The frame decides the composition:
  os_lidar  : T_lidar_imu = inv(T_sensor_lidar) @ T_sensor_imu
  os_sensor : T_lidar_imu = T_sensor_imu      (the cloud frame IS the sensor frame)
The newer-Ouster 180-deg Z rotation lives inside lidar_to_sensor_transform, so it is
captured automatically. ALWAYS validate the result with glim_ext libimu_validator.so.
"""
import json

import numpy as np


def _mat(transform_16):
    m = np.array(transform_16, dtype=float).reshape(4, 4)
    m[:3, 3] /= 1000.0  # Ouster translations are in millimeters
    return m


def rotation_to_quaternion(R):
    """3x3 rotation matrix -> normalized [qx, qy, qz, qw] (Shepperd's method)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        qw, qx, qy, qz = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw, qx, qy, qz = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw, qx, qy, qz = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw, qx, qy, qz = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def _get_transform(metadata, key):
    """Extract a 16-element transform list from metadata, handling nested format.

    Supports both flat format (metadata[key]) and nested format introduced in
    firmware 2.5.x (metadata['imu_intrinsics']['imu_to_sensor_transform'], etc.).
    """
    if key in metadata:
        return metadata[key]
    nested = {'imu_to_sensor_transform': ('imu_intrinsics', 'imu_to_sensor_transform'),
              'lidar_to_sensor_transform': ('lidar_intrinsics', 'lidar_to_sensor_transform')}
    if key in nested:
        parent, child = nested[key]
        if parent in metadata and child in metadata[parent]:
            return metadata[parent][child]
    raise KeyError(f"Transform key {key!r} not found in metadata (tried flat and nested format)")


def compose_t_lidar_imu(metadata, frame='os_lidar'):
    """metadata dict (Ouster) -> GLIM T_lidar_imu as TUM list [x,y,z,qx,qy,qz,qw]."""
    T_sensor_imu = _mat(_get_transform(metadata, 'imu_to_sensor_transform'))
    if frame == 'os_sensor':
        T = T_sensor_imu
    elif frame == 'os_lidar':
        T_sensor_lidar = _mat(_get_transform(metadata, 'lidar_to_sensor_transform'))
        T = np.linalg.inv(T_sensor_lidar) @ T_sensor_imu
    else:
        raise ValueError(f"unknown frame {frame!r}; expected 'os_lidar' or 'os_sensor'")
    xyz = T[:3, 3]
    quat = rotation_to_quaternion(T[:3, :3])
    return [float(v) for v in (*xyz, *quat)]


def load_metadata(path):
    """Read an Ouster metadata JSON file into a dict."""
    with open(path) as f:
        return json.load(f)


def main(argv=None):
    """CLI: derive T_lidar_imu from an Ouster metadata JSON and write a calib YAML."""
    import argparse

    parser = argparse.ArgumentParser(description='Derive GLIM T_lidar_imu from Ouster metadata.')
    parser.add_argument('--metadata', required=True, help='Ouster metadata JSON file')
    parser.add_argument('--frame', default='os_lidar', choices=['os_lidar', 'os_sensor'],
                        help='Frame ouster-ros publishes points in (point_cloud_frame)')
    parser.add_argument('--out', help='Output calib YAML path (default: stdout)')
    a = parser.parse_args(argv)

    tum = compose_t_lidar_imu(load_metadata(a.metadata), frame=a.frame)
    line = 'T_lidar_imu: [' + ', '.join(repr(v) for v in tum) + ']\n'
    if a.out:
        with open(a.out, 'w') as f:
            f.write('# Derived from %s (frame=%s). VALIDATE with libimu_validator.\n' % (a.metadata, a.frame))
            f.write(line)
        print('wrote', a.out)
    else:
        print(line, end='')


if __name__ == '__main__':
    main()
