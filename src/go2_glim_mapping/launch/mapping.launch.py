"""Launch GLIM 3D mapping + map saver against configurable sensor topics.

Runs glim_rosnode (ROS2 node name "glim_ros") with this package's GLIM config,
remapping the configured input topics (/points, /imu) to the actual source
topics, and a map_saver that writes the global map to PLY+PCD on shutdown or on
the ~/save_map service.

Args:
  profile:=real  -> patch config_sensors.json (global_shutter_lidar=false +
                    calibrated T_lidar_imu from calib_path) for real hardware.
                    Default 'sim' uses the committed config unchanged.
  viewer:=true   -> enable GLIM's Iridescence live map viewer (needs X11 + GL).
                    The committed config stays headless; when true we copy it to
                    a temp dir and add libstandard_viewer.so (no file duplication
                    in the package).

Sim defaults match the validated diff-drive rig. For a real Ouster:
  ros2 launch go2_glim_mapping mapping.launch.py \
      points_topic:=/ouster/points imu_topic:=/ouster/imu use_sim_time:=false
(and set T_lidar_imu in config/glim/config_sensors.json to your mounting).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2_glim_mapping.launch_config import load_extrinsic_yaml, prepare_config


def _effective_config_path(context):
    """Resolve config_path, applying the launch profile (sim/real) and viewer patches."""
    config_path = LaunchConfiguration('config_path').perform(context)
    profile = LaunchConfiguration('profile').perform(context).lower()
    viewer = LaunchConfiguration('viewer').perform(context).lower() in ('true', '1', 'yes')
    t_lidar_imu = None
    if profile == 'real':
        t_lidar_imu = load_extrinsic_yaml(LaunchConfiguration('calib_path').perform(context))
    return prepare_config(config_path, '/tmp/glim_cfg_effective',
                          profile=profile, viewer=viewer, t_lidar_imu=t_lidar_imu)


def _nodes(context, *args, **kwargs):
    config_path = _effective_config_path(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    glim_node = Node(
        package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
        parameters=[{
            'config_path': config_path,
            'dump_path': LaunchConfiguration('dump_path'),
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/points', LaunchConfiguration('points_topic')),
            ('/imu', LaunchConfiguration('imu_topic')),
        ],
    )
    map_saver = Node(
        package='go2_glim_mapping', executable='map_saver', name='map_saver', output='screen',
        parameters=[{
            'map_topic': '/glim_ros/map',
            'output_dir': LaunchConfiguration('output_dir'),
            'basename': LaunchConfiguration('map_basename'),
            'use_sim_time': use_sim_time,
        }],
    )
    return [glim_node, map_saver]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([FindPackageShare('go2_glim_mapping'), 'config', 'glim']),
            description='Absolute path to the GLIM config dir'),
        DeclareLaunchArgument('points_topic', default_value='/points',
                              description='Source PointCloud2 topic (remapped into GLIM)'),
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='Source Imu topic (remapped into GLIM)'),
        DeclareLaunchArgument('dump_path', default_value='/root/glim-humble-docker/maps/dump',
                              description='GLIM factor-graph dump dir (written on shutdown). '
                                          'Default is on the bind-mounted repo so it persists on the host.'),
        DeclareLaunchArgument('output_dir', default_value='/root/glim-humble-docker/maps',
                              description='Where map_saver writes <map_basename>.{ply,pcd}. '
                                          'Default is on the bind-mounted repo (host: ~/projects/glim-humble-docker/maps).'),
        DeclareLaunchArgument('map_basename', default_value='glim_map'),
        DeclareLaunchArgument('viewer', default_value='false',
                              description='Enable GLIM Iridescence live map viewer (needs X11+GL)'),
        DeclareLaunchArgument('profile', default_value='sim',
                              description="Config profile: 'sim' (committed, headless) or "
                                          "'real' (global_shutter_lidar=false + calibrated T_lidar_imu)"),
        DeclareLaunchArgument(
            'calib_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('go2_glim_mapping'), 'config', 'calib', 'ouster_os1_32.yaml']),
            description='YAML holding the calibrated T_lidar_imu (used when profile:=real)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
