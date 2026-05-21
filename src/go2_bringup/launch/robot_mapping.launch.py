"""Single entry point: Go2 control + Ouster GLIM mapping session.

Usage:
  ros2 launch go2_bringup robot_mapping.launch.py sensor_hostname:=<IP> udp_dest:=<Jetson-IP>

Prerequisites:
  M3 done — T_lidar_imu in config/calib/ouster_os1_32.yaml must not be identity.
  Set NetworkInterfaceAddress in config/dds/cyclone_dds.xml to Go2 interface (192.168.123.x).
  Verify: ip a | grep 192.168.123
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'sensor_hostname',
            description='Ouster hostname or IP (e.g. os1-xxxx.local or 169.254.x.x)',
        ),
        DeclareLaunchArgument('udp_dest', default_value='',
                              description='Host IP for sensor UDP data (blank = auto-detect)'),
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

    go2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('go2_bringup'), 'launch', 'go2_control.launch.py']
            )
        ),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('go2_glim_mapping'), 'launch', 'mapping.launch.py']
            )
        ),
        launch_arguments={
            'mode': 'real',
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'udp_dest': LaunchConfiguration('udp_dest'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'point_type': LaunchConfiguration('point_type'),
            'udp_profile_lidar': LaunchConfiguration('udp_profile_lidar'),
            'viewer': LaunchConfiguration('viewer'),
        }.items(),
    )

    return LaunchDescription(args + [go2_control, mapping])
