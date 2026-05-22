"""Record RealSense D435i RGB + Go2 map-frame pose (x,y,theta) into one rosbag2.

Teleop-drive the Go2 (joystick via go2_bringup) inside a prebuilt GLIM map; this
launch brings up the localization stack (map->base_link), the RealSense color stream,
the pose_from_tf republisher, and a `ros2 bag record` process.

EVERYTHING runs on CycloneDDS bound to eno1 -- the localization TF originates on the
Go2's cyclonedds graph, so the RealSense node and the bag-record process MUST inherit
the same RMW + CYCLONEDDS_URI or they discover 0 topics and the bag is empty. The two
SetEnvironmentVariable actions at the TOP of the description set that for every action
below (included launches + ExecuteProcess inherit the launch-process environment).

  ros2 launch go2_rgb_odom_recorder record.launch.py \
      sensor_hostname:=os1-xxxx.local \
      udp_dest:=<Jetson-eno1-IP> lidar_port:=7502 imu_port:=7503 udp_profile_lidar:=LEGACY \
      map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
      mount_xyz:="x y z" mount_rpy:="r p y" lidar_frame:=os_sensor \
      output_dir:=/root/glim-humble-docker/bags bag_name:=session1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    recorder_share = get_package_share_directory('go2_rgb_odom_recorder')
    realsense_share = get_package_share_directory('realsense2_camera')
    go2_share = get_package_share_directory('go2_bringup')
    dds_xml = os.path.join(go2_share, 'config', 'dds', 'cyclone_dds.xml')

    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds = SetEnvironmentVariable('CYCLONEDDS_URI', dds_xml)

    args = [
        # localization passthrough
        DeclareLaunchArgument('sensor_hostname', description='Ouster hostname or IP'),
        DeclareLaunchArgument('udp_dest', default_value=''),
        DeclareLaunchArgument('lidar_port', default_value='0'),
        DeclareLaunchArgument('imu_port', default_value='0'),
        DeclareLaunchArgument('udp_profile_lidar', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME'),
        DeclareLaunchArgument('map_pcd', description='Reference .pcd map for icp_localization'),
        DeclareLaunchArgument('lidar_frame', default_value='os_sensor'),
        DeclareLaunchArgument('mount_xyz', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument('mount_rpy', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        # recorder-specific
        DeclareLaunchArgument('rgb_topic',
                              default_value='/camera/camera/color/image_raw/compressed',
                              description='JPEG CompressedImage auto-advertised by the realsense '
                                          'node via image_transport (needs ros-humble-compressed-'
                                          'image-transport). ~10x smaller than raw. Use '
                                          '/camera/camera/color/image_raw for raw sensor_msgs/Image.'),
        DeclareLaunchArgument('pose_topic', default_value='/go2/map_pose'),
        DeclareLaunchArgument('rate', default_value='10.0',
                              description='pose_from_tf publish rate (Hz)'),
        DeclareLaunchArgument('color_profile', default_value='640,480,15',
                              description='RealSense color WIDTH,HEIGHT,FPS. D435i color FPS '
                                          'must be one of {6,15,30,60} -- 10 is NOT a valid '
                                          'sensor rate and silently falls back to 1280x720x30. '
                                          '15 is the closest valid rate to the 10 Hz pose.'),
        DeclareLaunchArgument('output_dir', default_value='bags',
                              description='Parent dir for the bag (must be writable)'),
        DeclareLaunchArgument('bag_name', default_value='go2_rgb_odom',
                              description='Bag dir name; must NOT already exist'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(recorder_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'udp_dest': LaunchConfiguration('udp_dest'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'udp_profile_lidar': LaunchConfiguration('udp_profile_lidar'),
            'timestamp_mode': LaunchConfiguration('timestamp_mode'),
            'map_pcd': LaunchConfiguration('map_pcd'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'mount_xyz': LaunchConfiguration('mount_xyz'),
            'mount_rpy': LaunchConfiguration('mount_rpy'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items(),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        }.items(),
    )

    pose_node = Node(
        package='go2_rgb_odom_recorder', executable='pose_from_tf', name='pose_from_tf',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': LaunchConfiguration('base_frame'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'rate': LaunchConfiguration('rate'),
        }],
    )

    bag_path = PathJoinSubstitution([LaunchConfiguration('output_dir'),
                                     LaunchConfiguration('bag_name')])
    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path,
             LaunchConfiguration('rgb_topic'), LaunchConfiguration('pose_topic')],
        output='screen',
    )

    return LaunchDescription(
        [set_rmw, set_dds] + args + [localization, realsense, pose_node, record])
