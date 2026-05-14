"""Build the effective GLIM config dir for a launch profile (sim/real, +viewer).

GLIM reads its config from files in a directory, not from ROS params, so per-launch
overrides are applied by copying the committed config to a temp dir and patching the
JSON there. The committed config stays sim-validated and headless; this never edits
it. Pure file/JSON ops so it is unit-testable without launching ROS.
"""
import json
import os
import shutil


def prepare_config(src_dir, dst_dir, profile='sim', viewer=False, t_lidar_imu=None):
    """Copy src_dir->dst_dir and patch for profile/viewer; return the dir GLIM should use.

    Returns src_dir unchanged when nothing needs patching (sim + no viewer).
    profile 'real': set global_shutter_lidar=False and (if given) T_lidar_imu.
    viewer True: insert libstandard_viewer.so into config_ros.json extension_modules.
    """
    if profile == 'sim' and not viewer:
        return src_dir

    shutil.rmtree(dst_dir, ignore_errors=True)
    shutil.copytree(src_dir, dst_dir)

    if profile == 'real':
        sensors_path = os.path.join(dst_dir, 'config_sensors.json')
        with open(sensors_path) as f:
            sensors = json.load(f)
        sensors['sensors']['global_shutter_lidar'] = False
        if t_lidar_imu is not None:
            if len(t_lidar_imu) != 7:
                raise ValueError(f"t_lidar_imu must be 7 elems (TUM), got {len(t_lidar_imu)}")
            sensors['sensors']['T_lidar_imu'] = [float(v) for v in t_lidar_imu]
        with open(sensors_path, 'w') as f:
            json.dump(sensors, f, indent=2)

    if viewer:
        ros_path = os.path.join(dst_dir, 'config_ros.json')
        with open(ros_path) as f:
            ros = json.load(f)
        mods = ros['glim_ros']['extension_modules']
        if not any('standard_viewer' in m for m in mods):
            mods.insert(1, 'libstandard_viewer.so')
            ros['glim_ros']['extension_modules'] = mods
            with open(ros_path, 'w') as f:
                json.dump(ros, f, indent=2)

    return dst_dir


def load_extrinsic_yaml(path):
    """Read T_lidar_imu (7-elem TUM list) from a calib yaml of form 'T_lidar_imu: [...]'."""
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['T_lidar_imu']
