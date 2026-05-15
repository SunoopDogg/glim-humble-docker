"""Single entry point: Go2 control + Ouster GLIM mapping session.

Usage:
  ros2 launch go2_bringup robot_mapping.launch.py sensor_hostname:=os1-xxxx.local

Prerequisites (Phase 3 runbook):
  M3 완료 필수 — config/calib/ouster_os1_32.yaml의 T_lidar_imu가 identity이면
  GLIM 궤적이 발산한다. derive_extrinsic을 먼저 실행할 것.

  Jetson 이더넷 인터페이스명을 config/dds/cyclone_dds.xml에 설정할 것.
  확인: ip a | grep 192.168.123
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_hostname = LaunchConfiguration('sensor_hostname')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_hostname',
            description='Ouster hostname or IP (e.g. os1-xxxx.local or 192.168.123.10)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('go2_bringup'), 'launch', 'go2_control.launch.py']
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('go2_glim_mapping'), 'launch', 'real_mapping.launch.py']
                )
            ),
            launch_arguments={'sensor_hostname': sensor_hostname}.items(),
        ),
    ])
