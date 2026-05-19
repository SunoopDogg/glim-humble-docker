# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A **Docker harness** for building and running [GLIM](https://github.com/koide3/glim) (a GPU-accelerated 3D LiDAR SLAM framework by koide3) on **ROS2 Humble**. The target deployment is robot navigation on a **Unitree Go2** quadruped using an **Ouster** LiDAR for perception. It is the reproducible build/run environment plus the GLIM source pulled in as submodules, plus `src/go2_glim_mapping` (the mapping application — see "Mapping package") and `src/go2_glim_navigation` (Nav2 autonomous navigation on the built map — see "Navigation package").

## Architecture (the big picture)

The environment is assembled in three layers that must be understood together:

1. **`Dockerfile`** builds a base image (`cuda13.1.2:humble-uv-nvm`) on top of an NVIDIA CUDA image. It installs the ROS2 Humble base, the colcon/rosdep/vcstool build toolchain, and two language runtimes: **`uv`** (Python) and **`nvm`** (Node). It deliberately does *not* build GLIM or its heavy C++ dependencies — those are built inside the running container (layer 2) to keep the image build fast and arch-flexible.

2. **`scripts/install-deps.sh`** runs *inside the container* and builds GLIM's native dependencies from source: **GTSAM 4.3a0**, **iridescence** (OpenGL viewer), and **gtsam_points** (built with `BUILD_WITH_CUDA=ON` — flip to `OFF` on machines without a GPU). It also `apt install`s system libs (boost, metis, fmt, spdlog, glm, glfw, OpenCV, ROS image_transport/cv_bridge). This is the slow, heavy step.

3. **`src/glim`**, **`src/glim_ros2`** (SLAM core + ROS2 wrapper), **`src/ouster-ros`** (real Ouster ROS2 driver, `ros2` branch), **`src/unitree-go2-ros2`** (Go2 Gazebo model + CHAMP — sim only, not in the mapping data path), **`src/unitree_ros2`** (Go2 EDU DDS SDK — provides `unitree_api` messages used by `go2_bringup`), and **`src/icp_localization_ros2`** (prior-map ICP localizer for navigation) are **git submodules**, built with **colcon** alongside `src/go2_glim_mapping`, `src/go2_glim_navigation`, and **`src/go2_bringup`** (ament_python, not submodules — PS4 joystick + Go2 DDS bridge + `real_mapping` entry point). Submodules are empty until initialized.

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

## Navigation package (`go2_glim_navigation`)

Nav2 autonomous goal navigation inside the prebuilt GLIM map. **Decoupled localizer (Approach B)** — the `glim_localization` fork was rejected (its pkg `glim_ros` collides with the submodules AND it is GLIM 1.0.4 vs repo's 1.2.1, downgrading mapping). Stack: **rko_lio** (apt `ros-humble-rko-lio`, `odom→base_link`) + **icp_localization_ros2** (submodule, scan-to-`.pcd` `map→odom`, eats `maps/glim_map.pcd` directly) + **`pcd_to_costmap`** (offline static OccupancyGrid) + Nav2 (Smac Hybrid-A* + RPP + STVL).
- Real E2E: `ros2 launch go2_glim_navigation real_navigation.launch.py sensor_hostname:=os1-xxxx.local map_pcd:=maps/glim_map.pcd costmap_yaml:=maps/glim_costmap.yaml` (measure `mount_xyz`/`mount_rpy`/`lidar_frame` for the base_link→lidar static TF).
- Static costmap: `ros2 run go2_glim_navigation pcd_to_costmap --pcd maps/glim_map.pcd --out maps/glim_costmap --z-min 0.0 --z-max 1.5` (regenerate after remapping; `maps/` is gitignored).
- Phase 3 hardware runbook: `docs/jetson-phase3-nav-handoff.md`. Pure-logic tests: same uv command from `src/go2_glim_navigation`.
- **base_link is required for nav** (mapping deferred it): `real_navigation.launch.py` publishes `base_link→<lidar_frame>` static TF; icp's `calibration.odometry_source_to_range_sensor` MUST equal that same base_link→lidar mount (identity only ok for sim). TF tree: `map→odom` (icp) + `odom→base_link` (rko_lio).

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

- The git submodules (`.gitmodules`: `src/glim`, `src/glim_ros2`, `src/ouster-ros`, `src/unitree-go2-ros2`, `src/unitree_ros2`, `src/icp_localization_ros2`) are **empty checkouts** until `git submodule update --init --recursive` is run. Build commands fail silently-ish without this.
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
- **ros-humble-joy missing from image**: `joy_node` and `teleop_twist_joy` not pre-installed — `apt-get install -y ros-humble-joy ros-humble-teleop-twist-joy` inside container before first `robot_mapping.launch.py` run.
- **Stale os_driver intercepts IMU UDP**: if a previous launch was killed without cleanup, the old `os_driver` process keeps binding to port 7503 and steals IMU packets — new launch gets `num_imu=0` on all LiDAR frames. Symptom: `/ouster/imu` has publisher count=1 but zero messages. Fix: `pkill -f os_driver` (and `pkill -f glim_rosnode`) before restarting launch. Check with `ps aux | grep os_driver | grep -v grep | wc -l` — must be 1.
- **GLIM odom stuck at ~0 despite robot moving**: if `/glim_ros/odom` position stays near origin after robot motion, check `ps aux | grep os_driver | wc -l` first — stale os_driver is the common cause (IMU deprivation breaks scan matching initialization).
- **rko_lio: run the node, not its launch** — use `Node(executable='online_node')` / `ros2 run rko_lio online_node`, NOT `odometry.launch.py`. That launch only honors args present in the raw CLI `context.argv` (`name:=value`), so `IncludeLaunchDescription(launch_arguments=...)` silently fails its required-param check ("missing required parameter(s): imu_topic, lidar_topic, base_frame"). No `publish_tf` knob (always broadcasts `odom→base_link`).
- **nav2 bringup: never pass `slam:='false'` (lowercase)** — nav2 evaluates `IfCondition(PythonExpression(['not ', slam]))` → `NameError: name 'false' is not defined`. Omit `slam` (default `'False'` = localization mode) or pass capitalized `'False'`.
- **icp_localization_ros2 builds with apt `ros-humble-libpointmatcher` alone** — no separate `pointmatcher_ros` despite its package.xml dep. Nav apt deps: `ros-humble-{navigation2,nav2-bringup,spatio-temporal-voxel-layer,libpointmatcher,grid-map-msgs}`. Its TF frames are hardcoded (`map`/`odom`/`odom_source`/`range_sensor`) and config is via `node_params.yaml` (its `bringup.launch.py` takes no args) — patch the params file, don't pass launch args.
- **Nav needs `TIME_FROM_ROS_TIME`, NOT INTERNAL_OSC** (opposite of mapping): Ouster INTERNAL_OSC stamps cloud as since-boot (~568s) while rko_lio/icp TF use wall clock (~1.78e9) → Nav2 costmap `base_link→map` lookup fails with ~56yr "extrapolation into the past" → `bt_navigator` never reaches `active`. `real_navigation.launch.py` exposes `timestamp_mode` arg (default `TIME_FROM_ROS_TIME`); mapping keeps INTERNAL_OSC. PTP is the proper fix for both. Residual sub-second extrapolation after the flip is a `transform_tolerance` issue, categorically different.
- **rko_lio startup race in bundled nav launch**: `real_navigation.launch.py` starts os_driver + rko_lio together → rko_lio grabs a cold first scan and hard-aborts `std::runtime_error: Number of correspondences are 0` (exit -6, launch does NOT respawn). Survives when started standalone against an already-warm os_driver. Workaround: let the bundle bring up os_driver/icp/nav2, then `ros2 run rko_lio online_node ...`. Proper fix: TimerAction delay on rko_lio.
- **Find the Ouster when unreachable**: `avahi-browse -atr | grep -i ouster` or `getent hosts os-<serial>.local` resolves the sensor IP via mDNS even across subnets. If it returns e.g. `192.168.2.32` but `eno1` is on another subnet (Go2 `192.168.123.x`), add a matching secondary IP: `ip addr add 192.168.2.1/24 dev eno1` (inside privileged container — host netns), set `udp_dest` to that host IP. IP/subnet can change per power-cycle.
- **RViz on the Jetson physical display**: host is headless-SSH but a local seat session lives on `:1`. Show GUI: host `xhost +local:root` (with `DISPLAY=:1 XAUTHORITY=/home/<user>/.Xauthority`), then in container `DISPLAY=:1 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz`. Container already mounts `/tmp/.X11-unix`.
- **Driving the real Go2 from Nav2 needs the WHOLE nav stack on CycloneDDS**: the Go2 SDK is CycloneDDS-only and ROS2 doesn't interoperate across RMW vendors, so `real_navigation.launch.py` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=<go2_bringup>/config/dds/cyclone_dds.xml` for every node, and runs `go2_sport_bridge` (`/cmd_vel`→`/api/sport/request`, MOVE 1008). Motion is gated: default OFF, `ros2 service call /go2_sport_bridge/enable std_srvs/srv/SetBool "{data: true}"` to drive (caps `max_vx`/`max_vyaw`, watchdog stop on cmd_vel timeout). Robot must be STANDING first (operator joystick). Verify `cyclone_dds.xml` `<NetworkInterface name=...>` matches the Go2 interface (`ip a`), and pass `enabled` via `ParameterValue(value_type=bool)` — the string "false" is truthy.
- **`rmw_cyclonedds_cpp` is NOT in the base image**: only `cyclonedds-tools`/`libcycloneddsidl` (the DDS lib, not the ROS RMW). The Go2 path silently can't reach the robot without it. `apt-get install -y ros-humble-rmw-cyclonedds-cpp` inside the container (verify `/opt/ros/humble/lib/librmw_cyclonedds_cpp.so` exists). Belongs in `install-deps.sh`.
- **CycloneDDS for the Go2 must bind EXACTLY ONE enx interface — NO `lo`, NO Peers** (reversed 2026-06-16 after deep research, 3-0 verified vs official `unitree_ros2/setup.sh`). A prior fix added `<NetworkInterface name="lo"/>` + a localhost `<Peer>` "for same-host discovery" — that multi-homed lo+enx binding is the ROOT CAUSE the Go2 was never discovered (0 subs on `/api/sport/request`; `ddsi_udp_conn_write ... retcode -3` when a Go2 unicast peer was added). Official Unitree + autonomy_stack_go2 + go2_rl_ws all run full multi-node stacks same-host on a single enx interface (`multicast="default"`), no lo, no Peers. The old "round-trip VERIFIED" used a FAKE subscriber + the `ros2` CLI that hangs under cyclonedds regardless — invalid test. `go2_bringup/config/dds/cyclone_dds.xml` is now single-enx; use the modern `<Interfaces>` form (deprecated `<NetworkInterfaceAddress>` breaks binding). `ROS_DOMAIN_ID=0` (Go2 default). RMW is fine as apt-installed (CycloneDDS 0.10.x = Go2 SDK2 0.10.2; do NOT rebuild from source on Humble). Verify the link via the staged protocol in `docs/jetson-go2-dds-link-test.md`: demo talker/listener for same-host (NOT `ros2 node list`), then `ros2 topic echo /sportmodestate` to prove Go2 READ before any WRITE/motion. (Bind `eno1` not `enx` — see the USB-eth gotcha below; full plan in `docs/go2-drive-activation-design.md`.)
- **[SUPERSEDED 2026-06-17 — it was UFW, not the adapter; see `docs/go2-ouster-ufw-rootcause.md` and the next bullet]** ~~USB-eth adapter (`enx...`) drops ALL inbound UDP — use onboard `eno1` for Go2 AND Ouster~~: the symptom (raw socket / `tcpdump -i enx` sees frames; UDP socket gets ~0; ping works) was blamed on the USB-Ethernet adapter. A hands-on `~/ros2_ws` session proved the real cause is **UFW `INPUT policy DROP` dropping non-allowed UDP on every interface** — not enx. With its ports allowed the **Ouster runs fine on the `enx` dongle** (link-local `169.254.x`, unicast 7502/7503) and the **Go2 runs on `eno1`** at the same time — verified end-to-end (stand + drive 0.3 m + one synchronized rosbag). Go2 and Ouster do NOT have to share `eno1`; separate interfaces work. (The `multicast 0/38`, `unicast 2011→19` numbers were ufw dropping un-allowed UDP.)
- **UFW is THE root cause of "Go2 DDS never discovered / `/lowstate` rate 0 / `/api/sport/request` 0 subs"** (2026-06-17 empirical — `docs/go2-ouster-ufw-rootcause.md`): Jetson Ubuntu ships `ufw` active with `INPUT policy DROP`; it drops the Go2 DDS multicast (`239.255.0.1` SPDP+data and the Go2 data group `230.1.1.1`). AF_PACKET raw sees the robot streaming; a joined UDP socket gets 0; `IpInReceives` climbs while `IpInDelivers` stays flat; `iptables -L INPUT` shows `policy DROP` with no rule matching `239.255.0.1`. **Fix (persistent): `sudo ufw allow in on eno1`** (+ `ufw allow 7502/udp 7503/udp` for the Ouster). After it: `/lowstate` 500 Hz, `/joint_states` 500 Hz, `/tf` 518 Hz, robot drives. **This makes the `lo`+Peers / `rp_filter=0` / `224.0.0.0/4`-route / `allmulticast` workarounds in the bullets above & in `scripts/net_setup_go2.sh` UNNECESSARY** — `CYCLONEDDS_URI` collapses to the Unitree-standard single `eno1` interface (`multicast="default"`, no `lo`, no `<Peers>`); same-host multicast loopback works through ufw on one NIC (the earlier "self-loopback breaks same-host" / "lo+eno1 needed" findings were ufw dropping the looped/peer multicast on `eno1` INPUT). General rule on this box: raw-sniff-sees but socket-doesn't ⇒ **ufw / IP-input**, not the device.
- **Multicast RX debug protocol (tcpdump-sees ≠ socket-receives)**: when DDS discovers nothing, tcpdump (promisc, device-layer tap) is NOT proof the socket gets packets. Confirm delivery with a raw `socket.IP_ADD_MEMBERSHIP` join vs tcpdump in the SAME window, and watch `nstat UdpInDatagrams` delta. Check `ip route get 239.255.0.1` points at the Go2's iface — a stale Ouster `224.0.0.0/4 dev eno1` static route hijacks multicast egress/loopback (delete it + `ip route flush cache`). rp_filter=0 all ifaces, promisc, MAC filter, RX-offload-off do NOT fix a dead USB-eth.
- **Identify Go2 vs Ouster by traffic signature, not host iface IPs**: device IPs ≠ Jetson interface IPs (easy to mis-attribute). Go2 = DDS multicast `239.255.0.1` (+ `230.1.1.1:1720` stream), at 192.168.123.161. Ouster = UDP `7502`(lidar)/`7503`(imu) + `TCP 7501` open, at 192.168.2.32. `tcpdump -i <iface> udp` then match ports.
- **`timeout` does NOT kill `ros2` CLI python children**: `timeout 8 ros2 topic echo/pub` leaves zombie processes that pollute DDS (stale participants break discovery). Use a python subprocess with `preexec_fn=os.setsid` + `os.killpg(SIGKILL)`, or `pkill -9 -f "ros2 topic"`. Also `ros2 node/topic list`, `lifecycle get`, `param get` HANG under cyclonedds — use pub/echo/service-call + log/tf, never node-graph introspection.
- **Reference Go2 stacks on this host**: `~/projects/go2-humble-docker` (unitree_ros2, builds CycloneDDS 0.10.x from source, `NETWORK_INTERFACE`-driven setup.sh) and `~/ros2_ws` (go2_driver DDS + ouster_ros via `go2_bringup go2.launch.py`, proven **Go2 on `eno1` + Ouster on the `enx` dongle simultaneous**, driven + recorded — once `ufw allow in on eno1` is set; see `docs/go2-ouster-ufw-rootcause.md`). Validates single-iface `multicast="default"` on `eno1` for the Go2 (no lo/Peers needed once ufw is open).
