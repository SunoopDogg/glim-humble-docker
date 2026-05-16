"""Autonomous navigation inside a prebuilt map: rko_lio odometry + icp_localization + Nav2.

Robot-agnostic on /points + /imu + a .pcd reference map. Decoupled localization stack
(fallback "B", chosen over the glim_localization fork to avoid a GLIM 1.2.1->1.0.4
downgrade of the validated mapping pipeline):

  rko_lio (odometry.launch.py)   -> publishes  odom -> base_link  + /rko_lio/odometry
  icp_localization (Node)         -> map -> odom (scan-to-.pcd, consumes /rko_lio/odometry)
  Nav2 (Smac + RPP + STVL)        -> /cmd_vel
  map_server                      -> static global costmap (offline pcd->OccupancyGrid)

TF must resolve base_link<->sensor: sim provides it via the rig URDF; the real path
adds a base_link->os_sensor static transform (see real_navigation.launch.py).

TF COMPOSITION (verify on hardware with `ros2 run tf2_tools view_frames`, Phase 3 N2):
icp is configured with is_use_odometry=true consuming rko_lio's /rko_lio/odometry. The
exact single-publisher-per-edge wiring (icp is_provide_odom_frame, and whether rko_lio's
odom->base_link TF is kept or icp owns the full chain) is resolved empirically against
live odometry+lidar data; the committed icp config (icp_localization_ros2/config) holds
the calibration/ICP tuning and is patched only for map + topics here.

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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_glim_navigation.nav_config import (
    prepare_icp_params,
    prepare_nav2_params,
    validate_pcd_map,
)

RKO_LIO_ODOM_TOPIC = '/rko_lio/odometry'


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

    # Odometry: rko_lio (apt ros-humble-rko-lio) -> odom -> base_frame + /rko_lio/odometry.
    # rko_lio's odometry.launch.py only honors args found in the raw CLI argv (it inspects
    # context.argv for "name:=value"), so IncludeLaunchDescription cannot forward them --
    # run its node (online_node) directly. The lidar<->imu extrinsic is looked up from TF
    # (ouster_ros static TF + base_link), so only topics + base_frame are set here.
    rko_lio = Node(
        package='rko_lio', executable='online_node', name='rko_lio', output='screen',
        parameters=[{
            'lidar_topic': points_topic,
            'imu_topic': imu_topic,
            'base_frame': base_frame,
            'use_sim_time': use_sim_time_bool,
        }],
    )

    # Prior-map correction: icp_localization (scan-to-.pcd) -> map -> odom.
    # icp is config-file driven (no launch args); patch the committed node_params for
    # our map + topics + the Ouster OS1 input filter, keep its ICP/calib tuning.
    icp_share = get_package_share_directory('icp_localization_ros2')
    icp_node_params = prepare_icp_params(
        os.path.join(icp_share, 'config', 'node_params.yaml'),
        '/tmp/icp_node_params_effective.yaml',
        pcd_path=map_pcd, points_topic=points_topic, imu_topic=imu_topic,
        odom_topic=RKO_LIO_ODOM_TOPIC,
        input_filters_path=os.path.join(icp_share, 'config', 'input_filters_ouster_os1.yaml'))
    icp = Node(
        package='icp_localization_ros2', executable='icp_localization',
        name='icp_localization', output='screen',
        parameters=[icp_node_params,
                    {'icp_config_path': os.path.join(icp_share, 'config', 'icp.yaml'),
                     'use_sim_time': use_sim_time_bool}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': LaunchConfiguration('costmap_yaml'),
            # 'slam' defaults to 'False' (localization mode) -- do NOT pass lowercase
            # 'false'; nav2 evaluates PythonExpression(['not ', slam]) which needs
            # Python-cased False/True.
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
