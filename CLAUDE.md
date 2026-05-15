# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A **Docker harness** for building and running [GLIM](https://github.com/koide3/glim) (a GPU-accelerated 3D LiDAR SLAM framework by koide3) on **ROS2 Humble**. The target deployment is robot navigation on a **Unitree Go2** quadruped using an **Ouster** LiDAR for perception. It is the reproducible build/run environment plus the GLIM source pulled in as submodules, plus `src/go2_glim_mapping` (the mapping application — see "Mapping package").

## Architecture (the big picture)

The environment is assembled in three layers that must be understood together:

1. **`Dockerfile`** builds a base image (`cuda13.1.2:humble-uv-nvm`) on top of an NVIDIA CUDA image. It installs the ROS2 Humble base, the colcon/rosdep/vcstool build toolchain, and two language runtimes: **`uv`** (Python) and **`nvm`** (Node). It deliberately does *not* build GLIM or its heavy C++ dependencies — those are built inside the running container (layer 2) to keep the image build fast and arch-flexible.

2. **`scripts/install-deps.sh`** runs *inside the container* and builds GLIM's native dependencies from source: **GTSAM 4.3a0**, **iridescence** (OpenGL viewer), and **gtsam_points** (built with `BUILD_WITH_CUDA=ON` — flip to `OFF` on machines without a GPU). It also `apt install`s system libs (boost, metis, fmt, spdlog, glm, glfw, OpenCV, ROS image_transport/cv_bridge). This is the slow, heavy step.

3. **`src/glim`**, **`src/glim_ros2`** (SLAM core + ROS2 wrapper), **`src/ouster-ros`** (real Ouster ROS2 driver, `ros2` branch), **`src/unitree-go2-ros2`** (Go2 Gazebo model + CHAMP — sim only, not in the mapping data path), and **`src/unitree_ros2`** (Go2 EDU DDS SDK — provides `unitree_api` messages used by `go2_bringup`) are **git submodules**, built with **colcon** alongside `src/go2_glim_mapping` and **`src/go2_bringup`** (ament_python, not a submodule — PS4 joystick + Go2 DDS bridge + `real_mapping` entry point). Submodules are empty until initialized.

### Python ↔ ROS2 bridge

`uv` manages an isolated `.venv`, but ROS2's Python packages (rclpy, etc.) live in the system `dist-packages`. **`scripts/link_ros_to_venv.sh`** bridges the two: it runs `uv sync`, writes a `ros2.pth` file into the venv's `site-packages` pointing at the ROS2 install paths (so the uv venv can import ROS2 modules), then activates the venv. Run it after sourcing the ROS2 setup. It accepts an optional venv-dir argument (`bash scripts/link_ros_to_venv.sh <dir>`, default `.venv`).

### Multi-arch

`docker-compose.yaml` defines three services for two architectures:
- `glim-humble-docker` — default **amd64/x86** (CUDA 13.1.2).
- `glim-humble-docker-gpu` — same as above but reserves NVIDIA GPUs; enabled via the `gpu` profile.
- `glim-humble-docker-jetson` — **Jetson ARM64** (base `dustynv/cudnn:8.9-r36.2.0`, `runtime: nvidia`, mounts `/tmp/argus_socket`); enabled via the `jetson` profile.

All services run `privileged`, `network_mode: host`, `ipc: host` (required for ROS2 DDS + LiDAR hardware access) and bind-mount the repo to `/root/glim-humble-docker`.

## Mapping package (`go2_glim_mapping`)

ament_python orchestration around `glim_ros` (launch + GLIM config + map save). Robot-agnostic on `/points` + `/imu`.
- Sim E2E (headless): `ros2 launch go2_glim_mapping sim_mapping.launch.py` (`+ viewer:=true` for the GLIM Iridescence GUI). Drive with `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.25}}"`.
- Real Ouster (one-shot driver + mapping): `ros2 launch go2_glim_mapping real_mapping.launch.py sensor_hostname:=os1-xxxx.local` (Ouster internal IMU, `profile:=real`). Or point mapping at an existing source: `mapping.launch.py points_topic:=/ouster/points imu_topic:=/ouster/imu use_sim_time:=false`.
- Map out → bind-mounted `maps/` (`glim_map.{ply,pcd}` + `dump/`); trigger via `/map_saver/save_map` (std_srvs/Trigger) or Ctrl-C. **Must drive first** — `/glim_ros/map` only publishes after a submap finalizes (tune `max_num_keyframes` in `config_sub_mapping_*.json`).

## Commands

```bash
# Pull GLIM source (required before building — submodules start empty)
git submodule update --init --recursive

# Build the image (pick the target architecture)
docker compose build                          # amd64
docker compose --profile gpu build            # amd64 + GPU reservation
docker compose --profile jetson build         # Jetson ARM64

# Start a container and shell in (X11 forwarding is preconfigured for the GLIM viewer)
docker compose up -d glim-humble-docker
docker exec -it glim-humble-docker bash

# --- inside the container ---
apt-get install -y libzip-dev                 # required for ouster_ros (missing from install-deps.sh)
bash scripts/install-deps.sh                  # build GTSAM / iridescence / gtsam_points (slow, one-time)
source /opt/ros/humble/setup.bash
# Recommended Jetson colcon build (unitree_go has Foxy-only dep → cascade-aborts glim/ouster_ros without --ignore)
colcon build --packages-ignore unitree_go unitree_hg unitree_ros2_example \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 4 --continue-on-error
source install/setup.bash
bash scripts/link_ros_to_venv.sh              # uv sync + write ROS2 import bridge + activate venv

# run go2_glim_mapping pure-logic unit tests (extrinsic/launch_config — no ROS needed, host or container)
cd src/go2_glim_mapping && uv run --no-project --with numpy --with pyyaml --with pytest python -m pytest test/

# Jetson real-hardware mapping session (M3 derive_extrinsic must be done first)
sysctl -w net.ipv4.conf.eno1.rp_filter=0 && sysctl -w net.ipv4.conf.all.rp_filter=0   # inside container
ros2 launch go2_bringup robot_mapping.launch.py \
    sensor_hostname:=<Ouster-IP> udp_dest:=<Jetson-eno1-IP> \
    lidar_port:=7502 imu_port:=7503 udp_profile_lidar:=LEGACY
# Save map (separate terminal): ros2 service call /map_saver/save_map std_srvs/srv/Trigger '{}'
```

## Gotchas

- The four submodules (`src/glim`, `src/glim_ros2`, `src/ouster-ros`, `src/unitree-go2-ros2`) are **empty checkouts** until `git submodule update --init --recursive` is run. Build commands fail silently-ish without this.
- `gtsam_points` defaults to `BUILD_WITH_CUDA=ON` in `install-deps.sh`. On a GPU-less machine you must edit that flag to `OFF` or the build fails.
- The GLIM real-time viewer needs working X11 forwarding — the compose files already mount `/tmp/.X11-unix` and set `DISPLAY`; the host must allow X connections (`xhost +local:`).
- GLIM's ROS2 node is named **`glim_ros`** (NOT the `glim_rosnode` executable) → topics are `/glim_ros/{map,odom,...}`; `config_path`/`use_sim_time`/`dump_path` are params of node `glim_ros`.
- `config_sensors.json` `global_shutter_lidar`: **`true`** for sim/Gazebo LiDAR (no per-point timestamps), **`false`** for a real Ouster (has per-point `t` → want deskewing). CT odometry needs per-point times — use the default IMU (GPU/CPU) odometry, not `config_odometry_ct.json`.
- Native deps (GTSAM/iridescence/gtsam_points) live in the **running container's `/usr/local`, not the image** → `docker compose up`-recreating the container loses them (slow rebuild); avoid needless recreation.
- Multi-distro DDS clash: with other ROS2 distros (e.g. Jazzy containers) on the same `network_mode: host`, set a unique `ROS_DOMAIN_ID` per project — cross-distro discovery breaks the `ros2` CLI (`unknown tag 'rclpy.type_hash.TypeHash'`) and spams tf "jump back in time".
- Files created by root-in-container tools (`ros2 pkg create`, copies) are root-owned on the bind mount → `chown` to your host uid before editing them locally.
- Real Ouster bring-up uses `real_mapping.launch.py` (ouster-ros + `profile:=real`). `timestamp_mode` MUST be `TIME_FROM_INTERNAL_OSC` (NOT `TIME_FROM_ROS_TIME` — host-receive jitter decouples LiDAR↔IMU); `point_type:=native` gives per-point `t` for deskew. PTP is deferred for v1 (Ouster supplies both points + imu on one clock; INTERNAL_OSC stamps aren't Unix wall-clock, so add PTP if you need tf/rviz wall-clock alignment).
- Real-hardware `T_lidar_imu` is **derived from Ouster metadata** (`ros2 run go2_glim_mapping derive_extrinsic --metadata <json> --frame os_lidar`), not guessed — newer units have a 180° Z rotation (`qz≈1,qw≈0`); identity causes drift / "indeterminant linear system". Confirm the cloud `frame_id` (`os_lidar` vs `os_sensor`) and always validate with glim_ext `libimu_validator.so`.
- `mapping.launch.py profile:=real` patches `config_sensors.json` (`global_shutter_lidar=false` + `T_lidar_imu` from `config/calib/ouster_os1_32.yaml`) into `/tmp/glim_cfg_effective` at launch; the committed sim config is never edited (same temp-copy trick as `viewer:=true`).
- `ouster_ros` rosdeps (`ros-humble-pcl-conversions`, `libpcl-dev`, `libtins-dev`, `libpcap-dev`) are in `install-deps.sh`; if missing in a running container, `rosdep install --from-paths src/ouster-ros --ignore-src -r -y`.
- Build the real Ouster driver with `colcon build --packages-up-to ouster_ros` (pulls the `ouster-sensor-msgs` dep; `--packages-select` alone misses it).
- `ament_flake8` is NOT a passing gate here — committed code uses ~108-char lines (E501). Match existing style; don't reflow to 99.
- **Jetson ufw blocks Ouster UDP**: ufw is active by default on Jetson Ubuntu — symptom: raw AF_PACKET socket sees packets but UDP socket receives 0. Fix: `sudo ufw allow 7502/udp && sudo ufw allow 7503/udp`.
- **rp_filter=2 drops link-local Ouster packets**: reverse-path filter on the Ouster interface (`eno1`) rejects link-local (`169.254.x.x`) UDP before it reaches any socket. Fix inside privileged container: `sysctl -w net.ipv4.conf.eno1.rp_filter=0 && sysctl -w net.ipv4.conf.all.rp_filter=0`. Resets on reboot — set at session start before launch.
- **libzip-dev missing causes ouster_ros build failure**: not in `install-deps.sh` — `apt-get install -y libzip-dev` inside the container before building.
- **firmware 2.5.x nested metadata format**: `derive_extrinsic` raises `KeyError: 'imu_to_sensor_transform'` because firmware 2.5.x wraps transforms under `imu_intrinsics.imu_to_sensor_transform`. `extrinsic.py` is patched to handle both flat and nested formats.
- **ouster-ros 0.16.2 + firmware 2.5.x WINDOW field crash**: `RNG19_RFL8_SIG16_NIR16` profile causes `Field 'WINDOW' not found in LidarScan` — os_driver dies on activation. Fix: pass `udp_profile_lidar:=LEGACY` to `real_mapping.launch.py`.
- **real_mapping.launch.py extended args**: `lidar_port` (default 0=auto), `imu_port` (default 0=auto), `udp_profile_lidar` (default empty=sensor default), `point_type` (default native). For link-local Ouster, always set `udp_dest:=<Jetson-eno1-IP> lidar_port:=7502 imu_port:=7503`.
- **go2_bringup package**: `src/go2_bringup` (ament_python) — single entry point for PS4 joystick + Go2 DDS bridge + real_mapping. `config/dds/cyclone_dds.xml` `NetworkInterfaceAddress` must be set to the Go2 subnet interface (e.g. `enx...`, 192.168.123.x network).
