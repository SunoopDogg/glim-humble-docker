"""Real-hardware autonomous navigation: ouster driver + rko_lio + icp_localization + Nav2.

Symmetric to go2_glim_mapping/real_mapping.launch.py. Brings up the Ouster OS1-32
(INTERNAL_OSC, native point type), publishes the base_link->os_sensor mounting
transform that navigation needs (mapping never required base_link), and runs
navigation.launch.py (rko_lio odometry + icp_localization map->odom + Nav2) against
/ouster/points + /ouster/imu. Nav2 emits /cmd_vel; the existing go2_bringup bridge
drives the robot. Set the initial pose (RViz /initialpose) after bring-up.

The base_link->os_sensor offset (mount_xyz / mount_rpy) is the physical LiDAR
mounting on the Go2 — measure it for your rig. Confirm the Ouster cloud frame_id
(os_sensor vs os_lidar) and point the static transform at the right child frame.

  ros2 launch go2_glim_navigation real_navigation.launch.py \
      sensor_hostname:=os1-xxxx.local \
      map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
      costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(context, *args, **kwargs):
    xyz = LaunchConfiguration('mount_xyz').perform(context).split()
    rpy = LaunchConfiguration('mount_rpy').perform(context).split()
    child = LaunchConfiguration('lidar_frame').perform(context)
    return [Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                   '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                   '--frame-id', 'base_link', '--child-frame-id', child],
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_glim_navigation')
    ouster_share = get_package_share_directory('ouster_ros')

    args = [
        DeclareLaunchArgument('sensor_hostname', description='Ouster hostname or IP (e.g. os1-xxxx.local)'),
        DeclareLaunchArgument('udp_dest', default_value='',
                              description='Host IP for sensor UDP data (blank = driver auto-detect)'),
        DeclareLaunchArgument('lidar_port', default_value='0'),
        DeclareLaunchArgument('imu_port', default_value='0'),
        DeclareLaunchArgument('point_type', default_value='native',
                              description='native gives per-point t for deskew'),
        DeclareLaunchArgument('udp_profile_lidar', default_value=''),
        DeclareLaunchArgument('map_pcd', description='Reference .pcd map for icp_localization'),
        DeclareLaunchArgument('costmap_yaml',
                              description='Offline-exported 2D costmap (map_server yaml)'),
        DeclareLaunchArgument('lidar_frame', default_value='os_sensor',
                              description='Ouster cloud frame_id (os_sensor vs os_lidar) to mount under base_link'),
        DeclareLaunchArgument('mount_xyz', default_value='0.0 0.0 0.0',
                              description='base_link->lidar translation "x y z" (measure for your rig)'),
        DeclareLaunchArgument('mount_rpy', default_value='0.0 0.0 0.0',
                              description='base_link->lidar rotation "roll pitch yaw" rad'),
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
            'use_sim_time': 'false',
            'points_topic': '/ouster/points',
            'imu_topic': '/ouster/imu',
            'map_pcd': LaunchConfiguration('map_pcd'),
            'costmap_yaml': LaunchConfiguration('costmap_yaml'),
        }.items(),
    )

    return LaunchDescription(args + [ouster, OpaqueFunction(function=_static_tf), navigation])
