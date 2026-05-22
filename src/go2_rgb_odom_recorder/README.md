# go2_rgb_odom_recorder

Record RealSense D435i RGB + Go2 map-frame pose (x, y, θ) into one rosbag2 while
teleop-driving inside a prebuilt GLIM map.

## What it records
- `/camera/camera/color/image_raw` — raw `sensor_msgs/Image`, color only, 15 fps (640×480).
  (D435i color FPS must be one of {6,15,30,60}; 10 is invalid and silently falls back to
  1280×720×30. 15 is the closest valid rate to the 10 Hz pose; `trim_bag` tolerance pairs them.)
- `/go2/map_pose` — `geometry_msgs/PoseStamped` in the `map` frame, 10 Hz (θ = yaw of the quaternion).

## How it works
```
prebuilt glim_map.pcd ─► icp_localization ─map→odom─┐
                                                     ├─► /tf ─► pose_from_tf ─/go2/map_pose─┐ (10 Hz)
ouster ─/ouster/{points,imu}─► rko_lio ─odom→base_link┘                                     │
RealSense D435i ─/camera/camera/color/image_raw (10 fps raw)──────────────────────► ros2 bag record
```
`pose_from_tf` looks up TF `map→base_link` at 10 Hz and republishes it as `PoseStamped`,
stamped with the transform's own time (not wall-clock) so RGB↔pose alignment stays honest.

## Prerequisites (this rig / container)
- Built workspace incl. `go2_glim_navigation` (the recorder imports its `nav_config` icp helper).
- A prebuilt `.pcd` map (`maps/glim_map.pcd`).
- **`ros-humble-realsense2-camera` apt-installed in the container** — NOT in the base image:
  `apt-get install -y ros-humble-realsense2-camera` (belongs in `scripts/install-deps.sh`).
- `ros-humble-rmw-cyclonedds-cpp` installed (Go2 path is CycloneDDS-only).
- `ufw allow in on eno1` + `ufw allow 7502/udp 7503/udp` (see project CLAUDE.md).
- `sysctl -w net.ipv4.conf.eno1.rp_filter=0 && sysctl -w net.ipv4.conf.all.rp_filter=0`.

## Run
```bash
ros2 launch go2_rgb_odom_recorder record.launch.py \
    sensor_hostname:=os1-xxxx.local \
    udp_dest:=<Jetson-eno1-IP> lidar_port:=7502 imu_port:=7503 udp_profile_lidar:=LEGACY \
    map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
    mount_xyz:="<x y z>" mount_rpy:="<r p y>" lidar_frame:=os_sensor \
    output_dir:=/root/glim-humble-docker/bags bag_name:=session1
```
Then set the initial pose (RViz `2D Pose Estimate` / `/initialpose`) so icp converges, and
teleop-drive with the joystick. `bag_name` must NOT already exist.

The whole stack runs on CycloneDDS bound to `eno1` (set automatically by the launch). The
RealSense node and the `ros2 bag record` process inherit `RMW_IMPLEMENTATION` +
`CYCLONEDDS_URI` from the two `SetEnvironmentVariable` actions at the top of the launch —
without them they discover 0 topics and the bag is empty.

## Tests
Pure-logic (host or container). The system `launch_testing` pytest plugin clashes with the
uv-provisioned pytest, so disable plugin autoload:
```bash
cd src/go2_rgb_odom_recorder
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  uv run --no-project --with pytest python -m pytest test/ -v
```

## Post-processing: trim pose-less RGB
Recording starts at launch (t=0), but `/go2/map_pose` only appears AFTER icp converges
(initial pose set). So the raw bag has RGB frames with no pose: the **front** (before
convergence), any **mid-run gaps** (icp tracking loss), and the **tail** (after the last
pose). `trim_bag` drops them in one pass — it keeps an RGB frame iff a pose exists within
±tolerance of its stamp, and copies all pose messages verbatim:
```bash
ros2 run go2_rgb_odom_recorder trim_bag \
    --in bags/session1 --out bags/session1_trimmed --tolerance 0.1
```
`--tolerance` is seconds (default 0.1 = one 10 Hz period); `--out` must not exist. Drive
inside an existing map → set the initial pose right after launch to minimise the dropped
front segment.

## Acceptance (hardware)
1. **Topics live** — with the stack up + initial pose set + icp converged, verify with a
   short rclpy subscriber (NOT `ros2 topic hz`, unreliable under cyclonedds), and
   `export CYCLONEDDS_URI=<…>/cyclone_dds.xml` in the check shell:
   `/go2/map_pose` ~10 Hz (non-zero x/y once converged) and
   `/camera/camera/color/image_raw` ~10 fps.
2. **Bag non-empty (env-propagation gate)** — after a short record + Ctrl-C:
   `ros2 bag info bags/session1` → BOTH topics listed with `Count > 0`. An empty/missing
   topic means RMW/`CYCLONEDDS_URI` did not reach the record process.
