"""Real-hardware autonomous navigation: ouster-ros driver + glim localization + Nav2.

Symmetric to go2_glim_mapping/real_mapping.launch.py. Brings up the Ouster OS1-32
(INTERNAL_OSC, native point type) and runs navigation.launch.py with profile:=real
against /ouster/points + /ouster/imu inside the prebuilt GLIM map. Nav2 emits
/cmd_vel; the existing go2_bringup bridge drives the robot. Set the initial pose
(RViz /initialpose or the localizer's relocalize service) after bring-up.

  ros2 launch go2_glim_navigation real_navigation.launch.py \
      sensor_hostname:=os1-xxxx.local \
      map_path:=/root/glim-humble-docker/maps/dump \
      costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml
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
    pkg_share = get_package_share_directory('go2_glim_navigation')
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
                              description='Ouster point type (native gives per-point t for deskew)'),
        DeclareLaunchArgument('udp_profile_lidar', default_value='',
                              description='Ouster UDP lidar profile (blank = sensor default)'),
        DeclareLaunchArgument('map_path', description='GLIM dump dir to localize against'),
        DeclareLaunchArgument('costmap_yaml',
                              description='Exported 2D costmap (map_server yaml) for the global costmap'),
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'profile': 'real',
            'use_sim_time': 'false',
            'points_topic': '/ouster/points',
            'imu_topic': '/ouster/imu',
            'map_path': LaunchConfiguration('map_path'),
            'costmap_yaml': LaunchConfiguration('costmap_yaml'),
        }.items(),
    )

    return LaunchDescription(args + [ouster, navigation])
