"""Self-contained map-frame localization stack for the RGB+pose recorder.

PORTED from go2_glim_navigation/launch/real_navigation.launch.py + navigation.launch.py,
with Nav2 / sport_bridge / costmap OMITTED (recording is teleop-driven, no autonomy).
The validated Ouster params (TIME_FROM_ROS_TIME, native point type, LEGACY profile) and
the rko_lio/icp wiring are the source of truth in go2_glim_navigation -- mirror any
change there here.

Brings up:
  ouster sensor.launch.xml          -> /ouster/points + /ouster/imu
  base_link->lidar static TF        -> (mount_xyz / mount_rpy / lidar_frame)
  rko_lio online_node               -> odom -> base_link + /rko_lio/odometry
  icp_localization                  -> map -> odom (scan-to-.pcd, eats map_pcd)

Net result: TF map -> base_link, which pose_from_tf republishes as /go2/map_pose.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_glim_navigation.nav_config import prepare_icp_params, validate_pcd_map

RKO_LIO_ODOM_TOPIC = '/rko_lio/odometry'
POINTS_TOPIC = '/ouster/points'
IMU_TOPIC = '/ouster/imu'


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


def _localization(context, *args, **kwargs):
    base_frame = LaunchConfiguration('base_frame').perform(context)
    map_pcd = validate_pcd_map(LaunchConfiguration('map_pcd').perform(context))

    rko_lio = Node(
        package='rko_lio', executable='online_node', name='rko_lio', output='screen',
        parameters=[{
            'lidar_topic': POINTS_TOPIC,
            'imu_topic': IMU_TOPIC,
            'base_frame': base_frame,
            'use_sim_time': False,
        }],
    )
    # rko_lio hard-aborts (exit -6, "Number of correspondences are 0") if it grabs a
    # cold first scan before os_driver streams -- the launch does NOT respawn it, so the
    # whole map->odom->base_link chain never appears. Delay its start so os_driver is warm.
    rko_lio = TimerAction(period=8.0, actions=[rko_lio])

    icp_share = get_package_share_directory('icp_localization_ros2')
    icp_node_params = prepare_icp_params(
        os.path.join(icp_share, 'config', 'node_params.yaml'),
        '/tmp/recorder_icp_node_params_effective.yaml',
        pcd_path=map_pcd, points_topic=POINTS_TOPIC, imu_topic=IMU_TOPIC,
        odom_topic=RKO_LIO_ODOM_TOPIC,
        input_filters_path=os.path.join(icp_share, 'config', 'input_filters_ouster_os1.yaml'))
    icp = Node(
        package='icp_localization_ros2', executable='icp_localization',
        name='icp_localization', output='screen',
        parameters=[icp_node_params,
                    {'icp_config_path': os.path.join(icp_share, 'config', 'icp.yaml'),
                     'use_sim_time': False}],
    )
    return [rko_lio, icp]


def generate_launch_description():
    ouster_share = get_package_share_directory('ouster_ros')

    args = [
        DeclareLaunchArgument('sensor_hostname', description='Ouster hostname or IP'),
        DeclareLaunchArgument('udp_dest', default_value=''),
        DeclareLaunchArgument('lidar_port', default_value='0'),
        DeclareLaunchArgument('imu_port', default_value='0'),
        DeclareLaunchArgument('point_type', default_value='native'),
        DeclareLaunchArgument('udp_profile_lidar', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME'),
        DeclareLaunchArgument('map_pcd', description='Reference .pcd map for icp_localization'),
        DeclareLaunchArgument('lidar_frame', default_value='os_sensor'),
        DeclareLaunchArgument('mount_xyz', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument('mount_rpy', default_value='0.0 0.0 0.0'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
    ]

    ouster = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(ouster_share, 'launch', 'sensor.launch.xml')),
        launch_arguments={
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'udp_dest': LaunchConfiguration('udp_dest'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'timestamp_mode': LaunchConfiguration('timestamp_mode'),
            'point_type': LaunchConfiguration('point_type'),
            'udp_profile_lidar': LaunchConfiguration('udp_profile_lidar'),
            'viz': 'false',
        }.items(),
    )

    return LaunchDescription(
        args + [ouster, OpaqueFunction(function=_static_tf),
                OpaqueFunction(function=_localization)])
