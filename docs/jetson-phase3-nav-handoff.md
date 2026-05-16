# Jetson AGX Orin — Navigation Phase 3 Bring-up Handoff

Continuing **autonomous Nav2 navigation on the prebuilt GLIM map** for the real Go2 EDU + Ouster OS1-32, on the onboard **Jetson AGX Orin**. Software (Phase 1 pure logic + Phase 2 scaffolding + build/interface conformance) is done and committed on branch `design/go2-glim-navigation`; Phase 3 is the hardware runtime, here on the robot.

> Design spec / plan live under `docs/superpowers/{specs,plans}/2026-06-15-go2-glim-navigation*` but are **local-only (untracked)** — a fresh clone won't have them. This file is the self-contained handoff.

## Locked decisions (Approach B — decoupled, no GLIM change)

The `glim_localization` fork was rejected: its package `glim_ros` collides with the upstream submodules AND it is GLIM **1.0.4** vs this repo's **1.2.1** (would downgrade the validated mapping pipeline). Instead, navigation is built from independent pieces and **mapping stays on GLIM 1.2.1, untouched**:

- **Odometry — `rko_lio`** (apt `ros-humble-rko-lio`): publishes `odom → base_link` + `/rko_lio/odometry`.
- **Prior-map localization — `icp_localization_ros2`** (submodule `src/icp_localization_ros2`): scan-to-`.pcd` ICP, publishes `map → odom`. Consumes the existing `maps/glim_map.pcd` (no conversion).
- **Static costmap — `pcd_to_costmap`** (this package): offline `pcd → OccupancyGrid` (pgm+yaml), served by `nav2_map_server`. Live dynamics via STVL on the local costmap.
- **Nav2**: Smac Hybrid-A* planner + RPP controller → `/cmd_vel` → existing `go2_bringup` bridge.

## Validated in Phase 2 (container `glim-humble-docker-gpu`, no robot)

- **icp_localization_ros2 BUILDS** with apt `ros-humble-libpointmatcher` only (no `pointmatcher_ros`). The #8 build risk is resolved — B is viable, no hdl_localization fallback needed.
- `pcd_to_costmap` produced `maps/glim_costmap.{pgm,yaml}` (318×318 @ 0.05 m, ~3% occupied); `nav2_map_server` loads it.
- `navigation.launch.py` composes cleanly: `/rko_lio` + `/icp_localization` + `/nav2_container` all spawn, no launch exceptions; rko_lio publishes `/rko_lio/odometry` (odom) + deskew on.
- 14 pure-logic unit tests green (`cd src/go2_glim_navigation && uv run --no-project --with pyyaml --with pytest python -m pytest test/`).

### Interface facts baked into the launch (reconciled against built pkgs)
- **rko_lio is run as a `Node`** (`executable: online_node`), NOT via its `odometry.launch.py` — that launch only honors args found in the raw `context.argv` (`name:=value`), so `IncludeLaunchDescription` can't forward them. It has **no `publish_tf` knob** (always broadcasts `odom→base_link`; only `invert_odom_tf` flips parent/child).
- **icp** is `package: icp_localization_ros2`, `executable: icp_localization`, configured by `node_params.yaml` (no launch args). `prepare_icp_params` patches `pcd_file_path` + topics + `input_filters_ouster_os1.yaml` into a temp copy.
- **Do NOT pass `slam:=false`** to nav2 bringup (lowercase) — nav2 evaluates `PythonExpression(['not ', slam])`; the default `'False'` = localization mode is what we want, so omit it.

## TF composition — DESIGN INTENT (tree B), verify at runtime

rko_lio cannot suppress its TF, so the tree is fixed:
```
map --(icp publishMapToOdom)--> odom --(rko_lio)--> base_link --(static, real_navigation)--> os_sensor
                                  └--(icp, is_provide_odom_frame)--> odom_source --> range_sensor
```
- Nav2's chain `map→odom→base_link` is complete: `map→odom` from icp, `odom→base_link` from rko_lio. **No edge has two publishers** (icp's `odom→odom_source` is a separate child of `odom`; harmless to Nav2).
- **CRITICAL CALIBRATION:** icp interprets `/rko_lio/odometry` (which is `odom→base_link`) as `odom→odom_source`. For the computed `map→odom` to correctly place `base_link`, set icp's `calibration.odometry_source_to_range_sensor` = **`base_link → range_sensor` (the lidar mount offset)** — the SAME transform as the `base_link→os_sensor` static TF in `real_navigation.launch.py`. Identity (committed default) is only correct if the lidar sits at `base_link` (sim). On the real Go2 the mount offset is non-trivial; a wrong value shows up as localization bias.
- Verify on hardware: `ros2 run tf2_tools view_frames` → exactly one publisher per edge, `map→odom` (icp) and `odom→base_link` (rko_lio) present, no duplicate `odom→base_link`.

## Phase 3 runbook (DO IN ORDER — gates blocking)

**Setup (inside the privileged container):**
```bash
apt-get install -y ros-humble-rko-lio ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-spatio-temporal-voxel-layer ros-humble-libpointmatcher ros-humble-grid-map-msgs
# rp_filter for link-local Ouster (resets on reboot):
sysctl -w net.ipv4.conf.eno1.rp_filter=0 && sysctl -w net.ipv4.conf.all.rp_filter=0
source /opt/ros/humble/setup.bash
colcon build --packages-up-to icp_localization_ros2 go2_glim_navigation --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# (Re)generate the static costmap if the map changed:
ros2 run go2_glim_navigation pcd_to_costmap --pcd maps/glim_map.pcd --out maps/glim_costmap \
    --resolution 0.05 --z-min 0.0 --z-max 1.5
```

**N1 — rko_lio odometry on Orin.** Bring up the Ouster (`real_navigation.launch.py` brings up ouster + nav, or run ouster alone first). Hand-carry slowly.
Gate: `/rko_lio/odometry` publishes, `odom→base_link` tracks, sensor rate sustained, no frame drops.

**N2 — icp_localization + initial pose + TF.** Set the `base_link→lidar` mount: measure it, set `mount_xyz`/`mount_rpy`/`lidar_frame` in `real_navigation.launch.py` AND the matching `calibration.odometry_source_to_range_sensor` for icp. Launch, set `/initialpose` in RViz at the robot's real start pose.
Gate: `map→odom` stable at rest; `view_frames` shows the tree above with one publisher per edge; no divergence / no "indeterminant" errors. (icp needs an initial pose — it does NOT auto-relocalize globally.)

**N3 — static-obstacle goal nav.** Send a goal in a known static area.
Gate: plans a path, drives, `/cmd_vel` reaches the Go2 via the bridge, arrives within tolerance.

**N4 — STVL dynamic obstacles + gait robustness (#1 risk).** Walk a person into the path while the Go2 walks a loop.
Gate: dynamic obstacle marked + cleared (STVL time-decay); localization drift bounded during gait (vibration / narrow 32-beam FoV). Mitigate: damped mount; rko_lio + icp tuning; STVL params.

**N5 — E2E autonomous loop.** Multi-goal run in a room comparable to the mapping baseline.
Gate: repeatable goal-to-goal navigation without manual intervention.

## Key commands
```bash
# Full real navigation (ouster + rko_lio + icp + Nav2):
ros2 launch go2_glim_navigation real_navigation.launch.py \
    sensor_hostname:=os1-xxxx.local \
    map_pcd:=/root/glim-humble-docker/maps/glim_map.pcd \
    costmap_yaml:=/root/glim-humble-docker/maps/glim_costmap.yaml \
    lidar_frame:=os_sensor mount_xyz:="0.0 0.0 0.0" mount_rpy:="0.0 0.0 0.0"
# Send a goal: RViz "Nav2 Goal", or:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"
```

## Out of scope / fallback
- Out: 3D/terrain planning, auto global relocalization (initial pose manual), multi-goal BT, perf tuning, physical mount/damping (but the `base_link→lidar` value MUST be measured — see N2).
- Fallback (not needed, icp builds): `hdl_localization` ROS2 port if icp proves unstable on the Orin.
