"""End-to-end sim test: headless Gazebo + diff-drive sensor rig + glim localization + Nav2.

Reuses go2_glim_mapping's validated sim rig (room.world + sensor_bot.urdf), which
provides the base_link<->sensor TF. Runs navigation instead of mapping: rko_lio +
icp_localization localize against a prebuilt sim .pcd map and Nav2 drives the rig to
goals via /cmd_vel. Build the sim map first with go2_glim_mapping/mapping.launch.py mode:=sim
(it writes glim_map.pcd), then point map_pcd/costmap_yaml at it.

  ros2 launch go2_glim_navigation sim_navigation.launch.py \
      map_pcd:=/tmp/sim_glim_map.pcd costmap_yaml:=/tmp/sim_costmap.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('go2_glim_navigation')
    mapping_share = get_package_share_directory('go2_glim_mapping')
    world = os.path.join(mapping_share, 'sim', 'room.world')
    urdf = os.path.join(mapping_share, 'sim', 'sensor_bot.urdf')

    args = [
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
        DeclareLaunchArgument('map_pcd', description='Prebuilt sim .pcd reference map'),
        DeclareLaunchArgument('costmap_yaml', description='Prebuilt sim 2D costmap (map_server yaml)'),
    ]

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-file', urdf, '-entity', 'sensor_bot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.2'],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'points_topic': '/points',
            'imu_topic': '/imu',
            'map_pcd': LaunchConfiguration('map_pcd'),
            'costmap_yaml': LaunchConfiguration('costmap_yaml'),
        }.items(),
    )

    return LaunchDescription(args + [gzserver, spawn, navigation])
