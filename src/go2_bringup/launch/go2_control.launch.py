"""Launch Go2 EDU control stack: joy_node + teleop_twist_joy + go2_sport_bridge.

Sets CYCLONEDDS_URI to the package's cyclone_dds.xml before starting nodes so
the DDS middleware uses the correct network interface for Go2 communication.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_bringup')
    dds_xml = os.path.join(pkg, 'config', 'dds', 'cyclone_dds.xml')
    joy_cfg = os.path.join(pkg, 'config', 'joy', 'ps4_teleop.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('CYCLONEDDS_URI', dds_xml),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'device': '/dev/input/js0', 'autorepeat_rate': 20.0}],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[joy_cfg],
        ),
        Node(
            package='go2_bringup', executable='go2_sport_bridge',
            name='go2_sport_bridge', output='screen',
        ),
    ])
