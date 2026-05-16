"""Autonomous navigation inside a prebuilt GLIM map: glim localization + Nav2.

Robot-agnostic on /points + /imu + a GLIM dump map_path. Runs glim_rosnode in
localization mode (publishes map->odom->base_link) and Nav2 (Smac planner + RPP
controller + STVL local layer) which emits /cmd_vel. Symmetric to
go2_glim_mapping/mapping.launch.py.

  ros2 launch go2_glim_navigation navigation.launch.py \
      map_path:=/root/glim-humble-docker/maps/dump \
      costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml \
      points_topic:=/ouster/points imu_topic:=/ouster/imu use_sim_time:=false profile:=real
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_glim_mapping.launch_config import load_extrinsic_yaml, prepare_config
from go2_glim_navigation.nav_config import prepare_nav2_params, validate_map_path


def _setup(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ('true', '1', 'yes')
    profile = LaunchConfiguration('profile').perform(context).lower()
    map_path = validate_map_path(LaunchConfiguration('map_path').perform(context))

    glim_cfg = LaunchConfiguration('config_path').perform(context)
    t_lidar_imu = None
    if profile == 'real':
        t_lidar_imu = load_extrinsic_yaml(LaunchConfiguration('calib_path').perform(context))
    glim_cfg = prepare_config(glim_cfg, '/tmp/glim_nav_cfg_effective',
                              profile=profile, viewer=False, t_lidar_imu=t_lidar_imu)

    nav2_params = prepare_nav2_params(
        LaunchConfiguration('nav2_params').perform(context),
        '/tmp/nav2_params_effective.yaml', use_sim_time=use_sim_time)

    glim = Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{
            'config_path': glim_cfg,
            'map_path': map_path,
            'localization': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/points', LaunchConfiguration('points_topic')),
            ('/imu', LaunchConfiguration('imu_topic')),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time_str,
            'params_file': nav2_params,
            'map': LaunchConfiguration('costmap_yaml'),
            'slam': 'false',
        }.items(),
    )
    return [glim, nav2]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('profile', default_value='sim',
                              description="'sim' or 'real' (GLIM global_shutter+T_lidar_imu)"),
        DeclareLaunchArgument('map_path', description='GLIM dump dir to localize against'),
        DeclareLaunchArgument('costmap_yaml',
                              description='Exported 2D costmap (map_server yaml) for the global costmap'),
        DeclareLaunchArgument('points_topic', default_value='/points'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_navigation'), 'config', 'glim_loc']),
            description='GLIM localization config dir'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_navigation'), 'config', 'nav2', 'nav2_params.yaml'])),
        DeclareLaunchArgument(
            'calib_path',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_mapping'), 'config', 'calib', 'ouster_os1_32.yaml'])),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
