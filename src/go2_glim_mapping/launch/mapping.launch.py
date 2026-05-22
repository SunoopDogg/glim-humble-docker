"""Unified GLIM mapping launch: bring up a sensor source by mode, run GLIM + map_saver.

One entry point for all mapping. Pick the source with `mode`:
  mode:=sim    headless Gazebo diff-drive rig (default) -> /points + /imu, sim profile
  mode:=real   ouster-ros driver -> /ouster/points + /ouster/imu, real profile
  mode:=topics no source; map an existing /points + /imu (another sim, a bag, ...)

Name the map with `map_name` -> <maps_root>/<map_name>/glim_map.{ply,pcd}
plus the GLIM factor-graph dump at <maps_root>/<map_name>/dump/. Each name is an
isolated directory, so a new run never overwrites a differently-named map.

Examples:
  ros2 launch go2_glim_mapping mapping.launch.py mode:=sim map_name:=room_a
  ros2 launch go2_glim_mapping mapping.launch.py mode:=real map_name:=lab \\
      sensor_hostname:=os1-xxxx.local udp_dest:=<host-IP> lidar_port:=7502 imu_port:=7503
  ros2 launch go2_glim_mapping mapping.launch.py mode:=topics \\
      points_topic:=/ouster/points imu_topic:=/ouster/imu
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_glim_mapping.launch_config import (
    load_extrinsic_yaml,
    prepare_config,
    resolve_map_paths,
    resolve_mode,
)


def _source_actions(mode):
    """Source-bringup actions for the chosen mode (sim: gazebo, real: ouster, topics: none)."""
    pkg_share = get_package_share_directory('go2_glim_mapping')
    if mode == 'sim':
        world = os.path.join(pkg_share, 'sim', 'room.world')
        urdf = os.path.join(pkg_share, 'sim', 'sensor_bot.urdf')
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
        return [gzserver, spawn]
    if mode == 'real':
        ouster_share = get_package_share_directory('ouster_ros')
        ouster = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(ouster_share, 'launch', 'sensor.launch.xml')),
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
        return [ouster]
    return []  # topics: external source, nothing to bring up


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).lower()
    src_points, src_imu, mode_profile, use_sim_time = resolve_mode(mode)

    # topics mode (src_*=None) falls back to the user-provided topic args
    points_topic = src_points or LaunchConfiguration('points_topic').perform(context)
    imu_topic = src_imu or LaunchConfiguration('imu_topic').perform(context)

    # explicit profile arg overrides the mode default (blank = use mode default)
    profile = LaunchConfiguration('profile').perform(context).lower() or mode_profile

    maps_root = LaunchConfiguration('maps_root').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    output_dir, dump_path = resolve_map_paths(maps_root, map_name)

    config_path = LaunchConfiguration('config_path').perform(context)
    viewer = LaunchConfiguration('viewer').perform(context).lower() in ('true', '1', 'yes')
    t_lidar_imu = None
    if profile == 'real':
        t_lidar_imu = load_extrinsic_yaml(LaunchConfiguration('calib_path').perform(context))
    eff_config = prepare_config(config_path, '/tmp/glim_cfg_effective',
                                profile=profile, viewer=viewer, t_lidar_imu=t_lidar_imu)

    glim_node = Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{
            'config_path': eff_config,
            'dump_path': dump_path,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/points', points_topic), ('/imu', imu_topic)],
    )
    map_saver = Node(
        package='go2_glim_mapping', executable='map_saver', name='map_saver', output='screen',
        parameters=[{
            'map_topic': '/glim_ros/map',
            'output_dir': output_dir,
            'basename': 'glim_map',
            'use_sim_time': use_sim_time,
        }],
    )
    nodes = _source_actions(mode) + [glim_node, map_saver]
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes'):
        rviz_cfg = os.path.join(
            get_package_share_directory('go2_glim_mapping'), 'config', 'rviz', 'mapping.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode', default_value='sim',
                              description="Sensor source: 'sim' (Gazebo), 'real' (Ouster), "
                                          "or 'topics' (external /points + /imu)"),
        DeclareLaunchArgument('map_name', default_value='glim_map',
                              description='Map identity -> <maps_root>/<map_name>/glim_map.{ply,pcd}'),
        DeclareLaunchArgument('maps_root', default_value='/root/glim-humble-docker/maps',
                              description='Root dir for map output (bind-mounted repo by default)'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Enable GLIM Iridescence live viewer (needs X11+GL)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz2 with the mapping view (config/rviz/mapping.rviz)'),
        DeclareLaunchArgument('profile', default_value='',
                              description="GLIM config profile override; blank = derived from mode "
                                          "(sim->sim, real/topics->real)"),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_mapping'), 'config', 'glim']),
            description='Absolute path to the GLIM config dir'),
        DeclareLaunchArgument(
            'calib_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('go2_glim_mapping'), 'config', 'calib', 'ouster_os1_32.yaml']),
            description='YAML holding the calibrated T_lidar_imu (used when profile resolves to real)'),
        # real / topics
        DeclareLaunchArgument('sensor_hostname', default_value='',
                              description='Ouster hostname or IP (required for mode:=real)'),
        DeclareLaunchArgument('udp_dest', default_value='',
                              description='Host IP for sensor UDP data (blank = driver auto-detect)'),
        DeclareLaunchArgument('lidar_port', default_value='0',
                              description='UDP port for lidar data (0 = auto-assign)'),
        DeclareLaunchArgument('imu_port', default_value='0',
                              description='UDP port for IMU data (0 = auto-assign)'),
        DeclareLaunchArgument('point_type', default_value='native',
                              description='Ouster point type (native/xyz/xyzi)'),
        DeclareLaunchArgument('udp_profile_lidar', default_value='',
                              description='Ouster UDP lidar profile (blank = sensor default)'),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_INTERNAL_OSC',
                              description='Ouster timestamp mode (mapping wants INTERNAL_OSC)'),
        # sim
        DeclareLaunchArgument('x', default_value='0.0', description='sim spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='sim spawn y'),
        # topics
        DeclareLaunchArgument('points_topic', default_value='/points',
                              description='Source PointCloud2 topic (mode:=topics)'),
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='Source Imu topic (mode:=topics)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
