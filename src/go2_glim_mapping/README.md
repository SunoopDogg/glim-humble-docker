# go2_glim_mapping

ROS2 Humble (ament_python) orchestration for **3D LiDAR mapping with [GLIM](https://github.com/koide3/glim)**, targeting a Unitree Go2 + Ouster, **simulation-first**. This package builds no SLAM of its own — it launches `glim_ros`, feeds it sensor topics, supplies GLIM config, and saves the resulting map. It is **robot-agnostic** on its `/points` + `/imu` inputs.

Validated end-to-end in Gazebo Classic (headless): a moving LiDAR+IMU produces a metric-accurate map (room walls reconstructed to ~1 cm at their true positions). See `../../docs/go2-ouster-glim-design.md` for the full design + validation record.

## Layout

```
go2_glim_mapping/
├── launch/
│   └── mapping.launch.py      # unified: mode:=sim|real|topics, map_name:=<name>
├── config/
│   ├── glim/                  # GLIM config (validated overrides; see "Config" below)
│   └── rviz/
├── sim/                       # test scaffolding (NOT the Go2)
│   ├── sensor_bot.urdf        # diff-drive base + OS1-ish 32-beam LiDAR + 200 Hz IMU
│   └── room.world             # 16×16 m room with pillars
└── go2_glim_mapping/
    ├── map_saver.py           # subscribe /glim_ros/map (latched) → PLY + PCD
    └── ply_to_pcd.py          # offline PLY → PCD (for the offline_viewer export route)
```

## Build

```bash
# in the colcon workspace root (sibling of src/glim, src/glim_ros2)
colcon build --packages-select go2_glim_mapping
source install/setup.bash
```
Requires `glim_ros` already built (GLIM deps via `scripts/install-deps.sh` + `colcon build`).
`mode:=sim` additionally needs `gazebo_ros` + `velodyne_gazebo_plugins`; `mode:=real` needs `ouster_ros`.

## Run

All mapping runs go through one launch — pick the source with `mode` and name the
map with `map_name` (output isolates to `<maps_root>/<map_name>/`).

### End-to-end sim test (no real robot)
```bash
ros2 launch go2_glim_mapping mapping.launch.py mode:=sim map_name:=room_a
# drive it (separate shell):
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.25}}"
# Ctrl-C the launch when done.
```

**Live GLIM map viewer** (Iridescence window — needs X11 + GL):
```bash
ros2 launch go2_glim_mapping mapping.launch.py mode:=sim viewer:=true
```
The committed config stays headless; `viewer:=true` copies it to `/tmp/glim_cfg_viewer`
and adds `libstandard_viewer.so` at launch (no committed file is changed). To watch
the Gazebo sim itself instead, run `gzclient` (DISPLAY set) in another shell while the
launch is up.

> Multi-container hosts: if other ROS2 distros (e.g. Jazzy) run on the same host
> network, set a unique `ROS_DOMAIN_ID` (e.g. `export ROS_DOMAIN_ID=42`) in every
> shell — cross-distro DDS discovery otherwise breaks `ros2` CLI (TypeHash errors)
> and spams TF "jump back in time".

### External source (`mode:=topics`) — another sim, a bag, or a running Ouster driver
```bash
ros2 launch go2_glim_mapping mapping.launch.py mode:=topics \
    points_topic:=/ouster/points imu_topic:=/ouster/imu
```
No source is brought up; GLIM subscribes to the topics you name (real profile by
default — pass `profile:=sim` if the source is a sim/bag with no per-point times).

### Real Ouster on a Go2 EDU (onboard AGX Orin)

One-shot driver + mapping (Ouster internal IMU, no PTP for v1):
```bash
ros2 launch go2_glim_mapping mapping.launch.py mode:=real map_name:=lab \
    sensor_hostname:=os1-xxxx.local udp_dest:=<host-IP> lidar_port:=7502 imu_port:=7503
```
This runs `ouster-ros` with `timestamp_mode:=TIME_FROM_INTERNAL_OSC` + `point_type:=native`
and the GLIM `real` profile (`global_shutter_lidar=false`, calibrated `T_lidar_imu`).
**Drive the Go2 with its joystick** to build the map — teleop is not in the GLIM data path.

Calibrate `T_lidar_imu` from the live sensor metadata before the first real run:
```bash
# copy the driver's metadata (ouster-ros writes one, or save the published /ouster/metadata)
ros2 run go2_glim_mapping derive_extrinsic --metadata /tmp/ouster_metadata.json \
    --frame os_lidar --out config/calib/ouster_os1_32.yaml
```
Then **validate** with glim_ext `libimu_validator.so` (see the plan runbook M3) — newer
Ouster units need a 180° Z rotation (`qz≈1, qw≈0`); the identity placeholder will drift.

> Time sync: PTP is deferred for v1 — points and IMU share one Ouster clock, so
> INTERNAL_OSC keeps them consistent. INTERNAL_OSC stamps are NOT Unix wall-clock,
> so tf/rviz against system-time nodes is awkward; add PTP (`ptp4l`/`phc2sys`) if you
> need wall-clock alignment.

### Map output
`map_name` gives each map its own directory under the **bind-mounted repo**
(`maps_root` default `/root/glim-humble-docker/maps` → host
`~/projects/glim-humble-docker/maps/`, gitignored) so maps persist on the host,
survive container restarts, and a new run never overwrites a differently-named map.
Override the root with `maps_root:=`.
- **Live global map** → `map_saver` writes `<maps_root>/<map_name>/glim_map.{ply,pcd}`
  (default `maps/glim_map/glim_map.{ply,pcd}`) on shutdown, or on demand:
  `ros2 service call /map_saver/save_map std_srvs/srv/Trigger`
- **Factor-graph dump** → `glim_rosnode` writes `<maps_root>/<map_name>/dump` on shutdown.
  Re-optimize / export PLY in the GUI: `ros2 run glim_ros offline_viewer` → File > Save > Export Points.
  Convert that PLY: `ros2 run go2_glim_mapping ply_to_pcd map.ply map.pcd`.
- **Downstream nav** reads `map_pcd:=maps/<map_name>/glim_map.pcd` (default path is now
  `maps/glim_map/glim_map.pcd`, not `maps/glim_map.pcd`).

## Config (`config/glim/`)

Copied from GLIM defaults with these overrides (in `config_sensors.json` / `config_ros.json` / `config_sub_mapping_gpu.json`):

| Key | Value | Why |
|-----|-------|-----|
| `points_topic` / `imu_topic` | `/points` / `/imu` | neutral names; launch remaps to the real source |
| `global_shutter_lidar` | `true` | sim/Gazebo LiDAR has no per-point timestamps; disable deskew (point order ≠ scan-time order). **Set `false` for a real Ouster** — ouster-ros publishes a per-point `t` field and you WANT deskewing on a moving robot; leaving it `true` zeroes those times and quietly degrades the map. |
| `T_lidar_imu` | `[-0.1, 0, -0.1, 0,0,0,1]` | **sim rig mounting — set to YOUR LiDAR↔IMU extrinsic for a real robot** |
| `extension_modules` | (no `libstandard_viewer.so`) | headless default; use `viewer:=true` for the Iridescence GUI (needs X11+GL) |
| `max_num_keyframes` | `5` | so submaps finalize and `/glim_ros/map` populates during short/enclosed runs |

## Gotchas

- **Node name is `glim_ros`** (not `glim_rosnode`, which is the executable). Published topics are `/glim_ros/map`, `/glim_ros/odom`, etc.
- **`/glim_ros/map` (latched) only publishes after a submap finalizes.** In enclosed spaces at low speed, scan overlap stays high → with stock `max_num_keyframes=15` no submap finalizes → live map stays empty until shutdown. The `/tmp/dump` is always written. (We lowered `max_num_keyframes` to 5.)
- **Non-monotonic time → GLIM throws** `IndexedSlidingWindow: index out of range`. Use `use_sim_time:=true` and restart `glim_rosnode` after a sim reset / bag loop.
- **CT odometry is unusable** with timestamp-less clouds (`config_odometry_ct.json` indexes per-point times). The default GPU IMU odometry is used.
- For CPU-only machines, set `gtsam_points` `BUILD_WITH_CUDA=OFF` and switch config to the `*_cpu.json` variants.
