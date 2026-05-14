"""End-to-end sim test: headless Gazebo + diff-drive sensor rig + GLIM mapping.

Brings up the validated diff-drive rig (OS1-ish 32-beam LiDAR + 200 Hz IMU) in a
structured room and runs the mapping pipeline. Drive it with, e.g.:
  ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.5}, angular: {z: 0.25}}"
then Ctrl-C the launch; map_saver writes the map and glim writes /tmp/dump.

This is test scaffolding (not the Go2). Go2 locomotion in Gazebo is a separate
task; the mapping package itself is robot-agnostic on its /points + /imu inputs.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_glim_mapping')
    world = os.path.join(pkg_share, 'sim', 'room.world')
    urdf = os.path.join(pkg_share, 'sim', 'sensor_bot.urdf')

    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')

    args = [
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Enable GLIM Iridescence live map viewer (needs X11+GL)'),
    ]

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-file', urdf, '-entity', 'sensor_bot', '-x', spawn_x, '-y', spawn_y, '-z', '0.2'],
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'points_topic': '/points',
            'imu_topic': '/imu',
            'viewer': LaunchConfiguration('viewer'),
        }.items(),
    )

    return LaunchDescription(args + [gzserver, spawn, mapping])
