"""Real-hardware autonomous navigation: ouster driver + rko_lio + icp_localization + Nav2.

Symmetric to go2_glim_mapping/real_mapping.launch.py. Brings up the Ouster OS1-32
(INTERNAL_OSC, native point type), publishes the base_link->os_sensor mounting
transform that navigation needs (mapping never required base_link), and runs
navigation.launch.py (rko_lio odometry + icp_localization map->odom + Nav2) against
/ouster/points + /ouster/imu. Nav2 emits /cmd_vel; the existing go2_bringup bridge
drives the robot. Set the initial pose (RViz /initialpose) after bring-up.

The base_link->os_sensor offset (mount_xyz / mount_rpy) is the physical LiDAR
mounting on the Go2 — measure it for your rig. Confirm the Ouster cloud frame_id
(os_sensor vs os_lidar) and point the static transform at the right child frame.

  ros2 launch go2_glim_navigation real_navigation.launch.py \
      sensor_hostname:=os1-xxxx.local \
      map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
      costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def generate_launch_description():
    pkg_share = get_package_share_directory('go2_glim_navigation')
    ouster_share = get_package_share_directory('ouster_ros')

    args = [
        DeclareLaunchArgument('sensor_hostname', description='Ouster hostname or IP (e.g. os1-xxxx.local)'),
        DeclareLaunchArgument('udp_dest', default_value='',
                              description='Host IP for sensor UDP data (blank = driver auto-detect)'),
        DeclareLaunchArgument('lidar_port', default_value='0'),
        DeclareLaunchArgument('imu_port', default_value='0'),
        DeclareLaunchArgument('point_type', default_value='native',
                              description='native gives per-point t for deskew'),
        DeclareLaunchArgument('udp_profile_lidar', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME',
                              description='Nav needs wall-clock cloud stamps so Nav2 costmaps can '
                                          'look up base_link->map at the cloud header time. '
                                          'INTERNAL_OSC (mapping default) is since-boot -> costmap '
                                          'TF lookups fail with ~56yr extrapolation. Use PTP for both.'),
        DeclareLaunchArgument('map_pcd', description='Reference .pcd map for icp_localization'),
        DeclareLaunchArgument('costmap_yaml',
                              description='Offline-exported 2D costmap (map_server yaml)'),
        DeclareLaunchArgument('lidar_frame', default_value='os_sensor',
                              description='Ouster cloud frame_id (os_sensor vs os_lidar) to mount under base_link'),
        DeclareLaunchArgument('mount_xyz', default_value='0.0 0.0 0.0',
                              description='base_link->lidar translation "x y z" (measure for your rig)'),
        DeclareLaunchArgument('mount_rpy', default_value='0.0 0.0 0.0',
                              description='base_link->lidar rotation "roll pitch yaw" rad'),
        DeclareLaunchArgument('drive_enabled', default_value='false',
                              description='Start the Go2 sport bridge ENABLED (default '
                                          'false = safe; robot will not move until '
                                          '`ros2 service call /go2_sport_bridge/enable '
                                          'std_srvs/srv/SetBool "{data: true}"`).'),
        DeclareLaunchArgument('max_vx', default_value='0.3',
                              description='Forward velocity cap (m/s) applied to Nav2 cmd_vel.'),
        DeclareLaunchArgument('max_vyaw', default_value='0.5',
                              description='Yaw velocity cap (rad/s) applied to Nav2 cmd_vel.'),
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

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'points_topic': '/ouster/points',
            'imu_topic': '/ouster/imu',
            'map_pcd': LaunchConfiguration('map_pcd'),
            'costmap_yaml': LaunchConfiguration('costmap_yaml'),
        }.items(),
    )

    go2_share = get_package_share_directory('go2_bringup')
    dds_xml = os.path.join(go2_share, 'config', 'dds', 'cyclone_dds.xml')

    # Whole stack on ONE RMW: the bridge must both receive Nav2 /cmd_vel and reach the
    # Go2 (Unitree SDK = CycloneDDS). ROS2 does not interoperate across RMW vendors, so
    # force rmw_cyclonedds_cpp + the Go2 DDS config for every node launched below.
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    set_dds = SetEnvironmentVariable('CYCLONEDDS_URI', dds_xml)

    sport_bridge = Node(
        package='go2_bringup', executable='go2_sport_bridge', name='go2_sport_bridge',
        output='screen',
        parameters=[{
            # value_type casts the launch-arg STRING to the right type. Without it, the
            # string "false" is truthy and the gate would default OPEN (cf. the nav2
            # slam:='false' footgun in CLAUDE.md).
            'enabled': ParameterValue(LaunchConfiguration('drive_enabled'), value_type=bool),
            'max_vx': ParameterValue(LaunchConfiguration('max_vx'), value_type=float),
            'max_vyaw': ParameterValue(LaunchConfiguration('max_vyaw'), value_type=float),
        }],
    )

    return LaunchDescription(
        [set_rmw, set_dds] + args
        + [ouster, OpaqueFunction(function=_static_tf), navigation, sport_bridge]
    )
