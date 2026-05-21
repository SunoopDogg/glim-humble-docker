# Jetson Nav — Validated Execution Order (real Go2 + Ouster OS1-32)

Verified live on the Jetson AGX Orin, 2026-06-15. Brings up the decoupled nav stack
(rko_lio + icp_localization + Nav2) against the prebuilt GLIM map. Run on the Jetson **host**;
all ROS work happens **inside** the `glim-humble-docker-jetson` container (host network).

## Real values discovered this session
- **Sensor:** OS-1-32-U0, sn `122413000316`, fw `2.5.3`. mDNS `os-122413000316.local`.
- **Sensor IP:** `192.168.2.32` (on the `192.168.2.0/24` subnet, NOT link-local this session).
- **Ouster iface:** `eno1`. Host needs an IP on the sensor subnet → add `192.168.2.1/24`.
- **udp_dest:** `192.168.2.1` (the host IP the sensor streams to).
- **Profile:** `LEGACY` (fw 2.5.x crashes on RNG19/WINDOW), ports `7502`/`7503`, `TIME_FROM_INTERNAL_OSC`, mode `1024x10`.
- **Map:** `maps/glim_map.pcd` (ascii) — used for BOTH icp and costmap. (clean/binary variants unreadable by `pcd_to_costmap`.)
- **Cloud frame_id:** `os_lidar`; IMU `os_imu`; ouster publishes `os_sensor→{os_lidar,os_imu}`.

> Sensor IP/subnet can change between power cycles. Re-check with `getent hosts os-122413000316.local`.
> If it returns a `192.168.x` addr, add that subnet to `eno1` (step 2) and set `udp_dest` to the host IP.

## One-time setup (per fresh container)
```bash
# host: restart the EXISTING container (never `compose up` recreate — loses GTSAM/gtsam_points in /usr/local)
docker start glim-humble-docker-jetson
docker update --restart=unless-stopped glim-humble-docker-jetson   # survive PID1 death (137)

# inside container:
apt-get install -y ros-humble-rko-lio ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-spatio-temporal-voxel-layer ros-humble-libpointmatcher ros-humble-grid-map-msgs \
    ros-humble-rviz2 iproute2 iputils-ping
# build icp + nav (icp has a QoS fix in src — must build from this tree)
cd /root/glim-humble-docker && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-up-to icp_localization_ros2 go2_glim_navigation \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
```

## Run order (every session)
```bash
# --- 1. network (host netns; run in container since it has `ip`) ---
ip addr add 192.168.2.1/24 dev eno1            # match sensor subnet (skip if already present)
bash scripts/net_setup_go2.sh                  # Go2 subnet on eno1 + mcast route + rp_filter
sysctl -w net.ipv4.conf.eno1.rp_filter=0 && sysctl -w net.ipv4.conf.all.rp_filter=0
ping -c2 192.168.2.32                           # GATE: sensor replies

# (one-time / when map changes) static costmap from the ascii map
ros2 run go2_glim_navigation pcd_to_costmap --pcd maps/glim_map.pcd --out maps/glim_costmap \
    --resolution 0.05 --z-min 0.0 --z-max 1.5

# source first in every shell:
cd /root/glim-humble-docker && source /opt/ros/humble/setup.bash && source install/setup.bash

# --- 2. Ouster driver ---
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.2.32 udp_dest:=192.168.2.1 \
    lidar_port:=7502 imu_port:=7503 timestamp_mode:=TIME_FROM_INTERNAL_OSC \
    udp_profile_lidar:=LEGACY point_type:=native viz:=false
# GATE (N1a): ros2 topic hz /ouster/points (~10) and /ouster/imu (~100)

# --- 3. base_link -> os_sensor static TF (identity ok for test; measure mount for production) ---
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id os_sensor

# --- 4. rko_lio odometry (run the NODE, not its launch) ---
ros2 run rko_lio online_node --ros-args -p lidar_topic:=/ouster/points -p imu_topic:=/ouster/imu \
    -p base_frame:=base_link -p use_sim_time:=false
# GATE (N1): /rko_lio/odometry ~10 Hz, odom->base_link TF tracks when robot moves

# --- 5. icp_localization (map->odom). Generate effective params first ---
python3 -c "from go2_glim_navigation.nav_config import prepare_icp_params; \
from ament_index_python.packages import get_package_share_directory as g; import os; \
s=g('icp_localization_ros2'); \
prepare_icp_params(os.path.join(s,'config','node_params.yaml'),'/tmp/icp_node_params_effective.yaml', \
pcd_path='/root/glim-humble-docker/maps/glim_map.pcd',points_topic='/ouster/points', \
imu_topic='/ouster/imu',odom_topic='/rko_lio/odometry', \
input_filters_path=os.path.join(s,'config','input_filters_ouster_os1.yaml'))"
ICP=$(ros2 pkg prefix icp_localization_ros2)/share/icp_localization_ros2/config/icp.yaml
ros2 run icp_localization_ros2 icp_localization --ros-args \
    --params-file /tmp/icp_node_params_effective.yaml -p icp_config_path:=$ICP -p use_sim_time:=false
# GATE (N2): map->odom stable at rest (icp converges from origin to true pose);
#   /registered_cloud ~10 Hz; NO "incompatible QoS" warning.

# --- 6. visual verify (RViz on host display :1) ---
# (host) xhost +local:root
python3 maps/pub_map.py maps/glim_map.pcd          # latched /prior_map for overlay
DISPLAY=:1 rviz2 -d maps/nav_check.rviz            # grey=prior_map, green=registered_cloud
# GATE (N2 visual): green registered scan overlays grey prior map. If offset -> RViz
#   "2D Pose Estimate" at the robot's true pose (icp does NOT auto-relocalize globally).

# --- 7. Nav2 + goal (N3: FIRST ROBOT MOTION). Run real_navigation ALONE. ---
# Do NOT also run steps 2-5: real_navigation bundles ouster+rko_lio+icp+nav2+go2_sport_bridge.
# Double-launching Ouster causes the os_driver port-7503 collision + rko_lio cold-start race.
ros2 launch go2_glim_navigation real_navigation.launch.py \
    sensor_hostname:=192.168.2.32 udp_dest:=192.168.2.1 lidar_port:=7502 imu_port:=7503 \
    udp_profile_lidar:=LEGACY \
    map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
    costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml \
    lidar_frame:=os_sensor mount_xyz:="0.0 0.0 0.0" mount_rpy:="0.0 0.0 0.0"
# Set /initialpose in RViz, then "Nav2 Goal".

# --- 8. enable real robot motion (N3) — Go2 must already be STANDING (sport mode) ---
# In a SEPARATE operator shell, source the DDS env FIRST or the service is invisible
# (cross-RMW -> "waiting for service to become available..." forever):
source /opt/ros/humble/setup.bash && source install/setup.bash && source scripts/go2_env.sh
ps aux | grep go2_sport_bridge | grep -v grep        # GATE: exactly one bridge process
ros2 topic echo /sportmodestate                      # GATE: live Go2 telemetry (eno1 READ ok)
ros2 service call /go2_sport_bridge/enable std_srvs/srv/SetBool "{data: true}"
# send a SMALL goal; robot moves. Stop: enable {data: false} (active stop), or Ctrl-C.
# Caps: max_vx/max_vyaw launch args (default 0.3 m/s / 0.5 rad/s). Watchdog stops on
# cmd_vel timeout. Whole stack runs on CycloneDDS (set by the launch + go2_env.sh).
```

## Known issues / risks
- **Goal not roughly ahead → robot spins in place / circles, "Failed to make progress"** (root-caused
  + fixed 2026-06-17). RPP `use_rotate_to_heading` (default true) rotates in place before driving
  (`/cmd_vel` linear.x=0); `SimpleProgressChecker` counts only linear motion → aborts mid-turn → loop.
  Fixed in `config/nav2/nav2_params.yaml`: `controller_server.FollowPath.use_rotate_to_heading: false`
  (SmacHybrid DUBIN path is already a feasible forward arc → turn-while-driving), plus explicit
  `progress_checker`/`goal_checker` and `goal_checker.yaw_goal_tolerance: 3.14` (ignore final heading,
  else it circles the goal). Diagnose motion-free: goal with bridge gated OFF, rclpy-sub `/cmd_vel` —
  linear.x≈0+angular≠0 = the bug; goal straight ahead gives linear.x=desired_linear_vel.
- **Map covers only ~65%** — the save was truncated (`glim_map.pcd` 1.89M of 2.91M pts). icp will
  lose lock in unmapped regions. Test inside the mapped area; for full coverage re-export from `maps/dump/`.
- **icp QoS fix** — Ouster publishes BEST_EFFORT; icp subscribers were RELIABLE → 0 msgs. Fixed in
  `RangeDataAccumulator.cpp` (cloud) + `transform/TfPublisher.cpp` (imu) → `.best_effort()`. Rebuild required.
- **icp_map not viewable directly** — published once, VOLATILE; late RViz misses it. Use `maps/pub_map.py`.
- **map→odom rest jitter ~±15 cm** on the 32-beam sensor in the partial map — confirm acceptable for nav footprint.
- **mount_xyz/rpy identity** is test-only; measure base_link→lidar for production and set the matching
  icp `odometry_source_to_range_sensor`.
- **Container exit 137** (not OOM) — set restart policy; deps survive `start`/`stop`, lost only on recreate.
