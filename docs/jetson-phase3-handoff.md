# Jetson AGX Orin — Phase 3 Bring-up Handoff

Context handoff for continuing the **real Go2 EDU + Ouster OS1-32 GLIM mapping** bring-up on the onboard **Jetson AGX Orin**. Phases 1–2 (software scaffolding) are done and committed; Phase 3 is the hardware bring-up and runs here, on the robot.

> The full design spec / implementation plan live under `docs/superpowers/{specs,plans}/` but are **local-only (untracked, not in git history)** — a fresh clone won't have them. This file is the self-contained handoff.

## Locked decisions

- **Go2 EDU** (DDS open via `unitree_ros2`), **Ouster OS1-32**, **IMU = Ouster internal** (`/ouster/imu`), compute = **onboard AGX Orin** (repo `jetson` docker profile), no tether.
- The pipeline is robot-agnostic on `/points` + `/imu`; `unitree_ros2`/joystick is only for **driving** the robot, not in the GLIM data path.

## What's already in the repo (Phase 1–2, committed)

- `src/go2_glim_mapping/launch/real_mapping.launch.py` — brings up `ouster-ros` (`timestamp_mode:=TIME_FROM_INTERNAL_OSC`, `point_type:=native`) + `mapping.launch.py profile:=real`. Run: `ros2 launch go2_glim_mapping real_mapping.launch.py sensor_hostname:=os1-xxxx.local`.
- `mapping.launch.py profile:=real` — patches `config_sensors.json` (`global_shutter_lidar=false` + calibrated `T_lidar_imu` from `config/calib/ouster_os1_32.yaml`) into `/tmp/glim_cfg_effective` at launch; committed sim config untouched.
- `config/calib/ouster_os1_32.yaml` — `T_lidar_imu` (TUM). **Currently an identity placeholder — must be replaced in M3.**
- `ros2 run go2_glim_mapping derive_extrinsic --metadata <json> --frame os_lidar --out config/calib/ouster_os1_32.yaml` — derives `T_lidar_imu` from Ouster metadata (`inv(lidar_to_sensor)·imu_to_sensor`, captures the newer-unit 180° Z rotation).
- `src/ouster-ros` submodule (ros2 branch), `ros-humble-pcl-conversions`/`libpcl-dev`/`libtins-dev`/`libpcap-dev` in `install-deps.sh`.
- Pure-logic unit tests (10): `cd src/go2_glim_mapping && uv run --no-project --with numpy --with pyyaml --with pytest python -m pytest test/`.

## Phase 3 runbook (do IN ORDER — gates are blocking)

**M1 — Orin compute validation (HIGHEST RISK, FIRST).** All prior validation was on an RTX 4090, never the Orin.
```bash
docker compose --profile jetson build
docker compose up -d glim-humble-docker-jetson && docker exec -it glim-humble-docker-jetson bash
# inside: bash scripts/install-deps.sh   (gtsam_points BUILD_WITH_CUDA=ON)
#         source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
ros2 run glim_ros glim_rosnode   # must start, no CUDA/arch error
```
Gate: all GLIM deps build; `glim_rosnode` sustains the OS1-32 rate (~10–20 Hz) with no frame drops / no "large time gap between consecutive LiDAR frames". Jetson base is `dustynv/cudnn:8.9-r36.2.0` (JetPack 6.0) vs GLIM-tested 6.1 — if it fails, bump the base image. **Do not proceed until GLIM runs on the Orin.**

**M2 — Ouster network + driver.** Dedicated ethernet, host static IP, `ping os1-xxxx.local`. Launch `real_mapping.launch.py`.
Gate: `ros2 topic hz /ouster/points` ≈ 10–20 Hz, `/ouster/imu` ≈ 100 Hz; `ros2 topic echo --once /ouster/points` shows `t` + `ring` fields. **Record `header.frame_id` (`os_lidar` vs `os_sensor`) → that is the `--frame` for M3.**

**M3 — Derive + validate T_lidar_imu.**
```bash
ros2 run go2_glim_mapping derive_extrinsic --metadata <metadata.json> --frame <from M2> --out config/calib/ouster_os1_32.yaml
colcon build --packages-select go2_glim_mapping   # refresh the installed calib
```
Gate: glim_ext `libimu_validator.so` reports the transform consistent AND a short slow pass shows no trajectory divergence / no "indeterminant linear system". Fallback: `unmannedlab/imu_lidar_calibration` (separate ROS1 env, excite all DoF), write result to the yaml, re-validate.

**M4 — Static / slow mapping sanity.** `real_mapping.launch.py`; move slowly; Ctrl-C.
Gate: `maps/glim_map.{ply,pcd}` written, non-empty, walls visibly planar.

**M5 — Gait-vibration spike (#1 unvalidated algorithm risk).** Mount on the Go2, joystick-walk a loop in a room comparable to the sim baseline; `ros2 bag record /ouster/points /ouster/imu`; save the map.
Gate (pass/fail): walking-map wall std within a reasonable multiple of the **sim baseline ~1 cm** (see `docs/go2-ouster-glim-design.md`), bounded drift. If it fails: vibration-damped mount and/or raise `imu_acc_noise`/`imu_gyro_noise` in `config_sensors.json`; re-run. Record the outcome.

## Key gotchas (also in CLAUDE.md)

- `timestamp_mode` MUST be `TIME_FROM_INTERNAL_OSC` (NOT `TIME_FROM_ROS_TIME` — host-receive jitter decouples LiDAR↔IMU). PTP deferred for v1 (one Ouster clock feeds both points+imu); INTERNAL_OSC stamps aren't Unix wall-clock → add PTP only if tf/rviz wall-clock alignment is needed.
- `global_shutter_lidar=false` on real hardware (per-point `t` → deskew); the sim profile uses `true`.
- **Must drive first** — `/glim_ros/map` only publishes after a submap finalizes; tune `max_num_keyframes` in `config_sub_mapping_*.json`.
- Native deps (GTSAM/iridescence/gtsam_points) live in the running container's `/usr/local`, not the image → don't `docker compose up`-recreate needlessly.
- Multi-distro on host net → set unique `ROS_DOMAIN_ID` (e.g. 42) per shell.

## Out of scope (deferred, user decisions)

Physical mount/damping, LiDAR power (Go2 tap vs battery), `base_link`↔lidar tf, consumer-Go2 (`go2_ros2_sdk` WebRTC) path, PTP upgrade.
