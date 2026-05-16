"""Autonomous navigation inside a prebuilt map: rko_lio odometry + icp_localization + Nav2.

Robot-agnostic on /points + /imu + a .pcd reference map. Decoupled localization stack
(fallback "B", chosen over the glim_localization fork to avoid a GLIM 1.2.1->1.0.4
downgrade of the validated mapping pipeline):

  rko_lio (odometry.launch.py)   -> publishes  odom -> base_link
  icp_localization (bringup)      -> publishes  map  -> odom   (scan-to-.pcd, /initialpose)
  Nav2 (Smac + RPP + STVL)        -> /cmd_vel
  map_server                      -> static global costmap (offline pcd->OccupancyGrid)

TF must resolve base_link<->sensor: sim provides it via the rig URDF; the real path
adds a base_link->os_sensor static transform (see real_navigation.launch.py).

NOTE (Task 10): the exact launch-arg names of rko_lio/icp_localization are reconciled
against the BUILT packages in the container; the wiring below follows their READMEs and
is adjusted there. icp must be configured to publish ONLY map->odom (rko_lio owns
odom->base_link) — verify with `ros2 run tf2_tools view_frames` (one publisher per edge).

  ros2 launch go2_glim_navigation navigation.launch.py \
      map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
      costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml \
      points_topic:=/ouster/points imu_topic:=/ouster/imu use_sim_time:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from go2_glim_navigation.nav_config import prepare_nav2_params, validate_pcd_map


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = use_sim_time.lower() in ('true', '1', 'yes')
    points_topic = LaunchConfiguration('points_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    map_pcd = validate_pcd_map(LaunchConfiguration('map_pcd').perform(context))

    nav2_params = prepare_nav2_params(
        LaunchConfiguration('nav2_params').perform(context),
        '/tmp/nav2_params_effective.yaml', use_sim_time=use_sim_time_bool)

    # Odometry: rko_lio (apt ros-humble-rko-lio) -> odom -> base_frame.
    rko_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rko_lio'), 'launch', 'odometry.launch.py')),
        launch_arguments={
            'lidar_topic': points_topic,
            'imu_topic': imu_topic,
            'base_frame': base_frame,
        }.items(),
    )

    # Prior-map correction: icp_localization (scan-to-.pcd) -> map -> odom.
    icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('icp_localization'), 'launch', 'bringup.launch.py')),
        launch_arguments={
            'pcd_filepath': map_pcd,
            'range_data_topic': points_topic,
            'imu_data_topic': imu_topic,
            'odometry_data_topic': '/rko_lio/odometry',
            'is_use_odometry': 'true',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': LaunchConfiguration('costmap_yaml'),
            'slam': 'false',
        }.items(),
    )
    return [rko_lio, icp, nav2]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_pcd', description='Reference .pcd map for icp_localization'),
        DeclareLaunchArgument('costmap_yaml',
                              description='Offline-exported 2D costmap (map_server yaml) for the global costmap'),
        DeclareLaunchArgument('points_topic', default_value='/points'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_navigation'), 'config', 'nav2', 'nav2_params.yaml'])),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
