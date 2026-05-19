# Go2 + Ouster DDS/UDP — UFW is the root cause (empirical, supersedes the "enx is broken" theory)

Date: 2026-06-17. Source: a full hands-on bring-up + drive + record session in `~/ros2_ws`
(the IntelligentRoboticsLabs `go2_driver` workspace) on **this Jetson**, with the Go2 **EDU**
on the onboard `eno1` and the Ouster **OS-1-32** on the USB-Ethernet adapter `enx32aababacca0`.
Everything below was observed directly (packet captures + rosbag), not inferred.

## TL;DR for navigation

- **The single root cause of "Go2 DDS never discovered / `/lowstate` rate 0 / `/api/sport/request` 0 subs" is UFW.**
  Ubuntu's `ufw` is active with `INPUT policy DROP`; it silently drops the Go2's DDS **multicast**
  (`239.255.0.1` SPDP + data, plus the Go2 data group `230.1.1.1`). Raw `AF_PACKET` sockets see the
  frames (pre-firewall); joined UDP sockets receive 0 (post-firewall). One command fixes it:
  ```bash
  sudo ufw allow in on eno1
  ```
  After this: `/lowstate` 500 Hz, `/joint_states` 500 Hz, `/tf` 518 Hz, robot drives. Persistent (ufw saves rules).

- **`CYCLONEDDS_URI` should be the Unitree standard: `eno1` only, `multicast="default"`.** Once ufw is
  open, `lo` interfaces, localhost/robot `<Peer>`s, `rp_filter=0`, the `224.0.0.0/4` route override, and
  `allmulticast on` are all **unnecessary** — they were attempts at the wrong cause and add fragility.
  (Same-host multicast loopback works fine through ufw on a single `eno1`; the earlier "self-loopback
  breaks same-host" was also ufw dropping the looped multicast on `eno1`'s INPUT.)

- **Go2 and Ouster run simultaneously on SEPARATE interfaces** — Go2 on `eno1`, Ouster on the `enx`
  USB-Ethernet dongle. This contradicts the prior CLAUDE.md conclusion ("enx drops ALL inbound UDP, put
  both on eno1 via a switch"). The Ouster's unicast LiDAR/IMU UDP works on `enx` because its ports are
  ufw-allowed (`ufw allow 7502/udp && ufw allow 7503/udp`, already in the gotchas). The "enx is broken"
  reading was UFW (non-allowed ports dropped), not the adapter.

## What was proven, with evidence

1. **Robot publishes DDS fine — host firewall was eating it.** With ufw still blocking, an `AF_PACKET`
   raw sniffer on `eno1` showed the Go2 streaming hard:
   ```
   192.168.123.161 -> 239.255.0.1:7400  (SPDP, domain 0)
   192.168.123.161 -> 239.255.0.1:7401  (DDS data)
   192.168.123.161 -> 230.1.1.1:1720    (Go2 data group, ~thousands/s)
   ```
   A UDP socket joined to `239.255.0.1:7400` on `eno1` received **0**. `IpInReceives` climbed while
   `IpInDelivers` stayed flat → dropped at IP input. `iptables -L INPUT` = `policy DROP` + ufw chains
   with no rule matching `239.255.0.1`. `sudo ufw allow in on eno1` → socket immediately received the
   robot; `/lowstate` came up at 500 Hz.

2. **Things that did NOT matter** (ruled out by direct test, in this order): robot model/posture (EDU,
   publishing regardless of low-down), ROS domain (robot is domain 0, SPDP on 7400), `rp_filter`
   (set 0, no change), the `224.0.0.0/4` multicast route (deleted/re-added to `eno1`, no change),
   `allmulticast` (raw saw frames either way), `lo`-as-interface / localhost `<Peer>` / robot `<Peer>`
   (full-unicast attempt: send to `.161` worked once `eno1` had higher `priority`, but data never
   arrived because the robot still answers on multicast → only fixing ufw helped). The whole
   lo/Peers/route/rp_filter rabbit hole was chasing a firewall drop.

3. **Simultaneous Go2 + Ouster, driven and recorded.** With `eno1`-only CycloneDDS + ufw open + Ouster
   on `enx`: a stand → 0.1 m/s × 3 s (~0.3 m) → stop sequence physically moved the robot and recorded
   one rosbag (24 s, 50450 msgs): `/ouster/points` 225 (~10 Hz), `/lowstate` 11944-ish, `/joint_states`
   11944, `/tf` 12391, `/odom` 1, `/api/sport/request`+`/response` 28 each (robot acked the commands).

## Reusable for nav (concrete)

- Pre-session, once per boot (ufw is persistent; these usually are NOT needed but harmless):
  ```bash
  sudo ufw allow in on eno1          # the actual fix — persistent
  # ouster ports if not already saved:
  sudo ufw allow 7502/udp && sudo ufw allow 7503/udp
  ```
- `CYCLONEDDS_URI`: single `<NetworkInterface name="eno1" priority="default" multicast="default"/>`,
  no `lo`, no `<Peers>`. `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- Ouster bring-up needs **fixed UDP ports** (`lidar_port:=7502 imu_port:=7503`) + explicit
  `udp_dest:=<host-IP-on-ouster-subnet>` — random ports trigger a driver poll-timeout → self-reset
  loop that starves the cloud (symptom: `sensor::poll_client() timed out`, `/ouster/points` never
  publishes though `enx` RX climbs).
- Diagnosing "topic exists but rate 0" on this box: run an `AF_PACKET` raw sniff (sees pre-firewall) vs
  a joined UDP socket (post-firewall). If raw sees the source and the socket doesn't → it's ufw/IP-input,
  not the device. (`~/ros2_ws/mc_sniff.py` is such a sniffer.)
- Test hygiene with CycloneDDS: `ros2 topic hz` ignores SIGTERM → use `timeout -s KILL N ros2 topic hz`;
  stop CLI tools with **Ctrl-C, never Ctrl-Z** (a suspended `ros2` keeps its DDS participant → pollutes
  discovery). `ros2 node/topic list` can hang under cyclonedds — prefer `hz`/`echo`/`service call`.

## Open / not verified here

- Whether `eno1`-only also fully serves the *nav stack* (rko_lio + icp + Nav2 + go2_sport_bridge) was not
  re-tested in this session — but the DDS substrate (discovery + `/cmd_vel`→robot + robot state) is now
  proven on the simplified config, so the nav-specific gotchas in `CLAUDE.md` still apply on top.
- `go2_driver` (the `~/ros2_ws` one) publishes `/odom` only once (latched) and never publishes `/imu`
  (IMU lives inside `/lowstate`); the nav stack here uses rko_lio/icp for odometry instead, so this is
  informational only.
