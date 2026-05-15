"""Real-hardware E2E: ouster-ros driver + GLIM mapping (profile:=real).

Symmetric to sim_mapping.launch.py. Brings up the Ouster OS1-32 via ouster-ros with
INTERNAL_OSC timestamps and the 'native' point type (per-point t for deskew), then
runs the mapping pipeline against /ouster/points + /ouster/imu with the real config
profile (global_shutter_lidar=false, calibrated T_lidar_imu).

Drive the Go2 with its joystick to build the map -- teleop is NOT in the GLIM data
path. Example:
  ros2 launch go2_glim_mapping real_mapping.launch.py sensor_hostname:=os1-xxxx.local
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_glim_mapping')
    ouster_share = get_package_share_directory('ouster_ros')

    args = [
        DeclareLaunchArgument('sensor_hostname', description='Ouster hostname or IP (e.g. os1-xxxx.local)'),
        DeclareLaunchArgument('udp_dest', default_value='',
                              description='Host IP for sensor UDP data (blank = driver auto-detect)'),
        DeclareLaunchArgument('lidar_port', default_value='0',
                              description='UDP port for lidar data (0 = auto-assign)'),
        DeclareLaunchArgument('imu_port', default_value='0',
                              description='UDP port for IMU data (0 = auto-assign)'),
        DeclareLaunchArgument('point_type', default_value='native',
                              description='Ouster point type (native/xyz/xyzi)'),
        DeclareLaunchArgument('udp_profile_lidar', default_value='',
                              description='Ouster UDP lidar profile (blank = sensor default)'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Enable GLIM Iridescence live viewer (needs X11+GL)'),
    ]

    ouster = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(ouster_share, 'launch', 'sensor.launch.xml')),
        launch_arguments={
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'udp_dest': LaunchConfiguration('udp_dest'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'timestamp_mode': 'TIME_FROM_INTERNAL_OSC',
            'point_type': LaunchConfiguration('point_type'),
            'udp_profile_lidar': LaunchConfiguration('udp_profile_lidar'),
            'viz': 'false',
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'profile': 'real',
            'use_sim_time': 'false',
            'points_topic': '/ouster/points',
            'imu_topic': '/ouster/imu',
            'viewer': LaunchConfiguration('viewer'),
        }.items(),
    )

    return LaunchDescription(args + [ouster, mapping])
