# Go2 DDS link — staged verification (READ before WRITE, motion-gated)

Design from deep research (2026-06-16) + advisor review. Resolves the blocker where
the Go2 broadcasts SPDP multicast and pings OK, but `/api/sport/request` had 0
subscribers and no Go2 topics were ever discovered.

## Root cause (3-0 verified vs official unitree_ros2/setup.sh)

The container's `cyclone_dds.xml` bound **two** interfaces (`enx` + `lo`) plus a
localhost `<Peer>`. Official Unitree config + two working community Go2 stacks
(autonomy_stack_go2, go2_rl_ws) all bind **exactly one** enx interface, no lo, no
Peers, no ParticipantIndex — and run full multi-node stacks same-host. The
multi-homed lo+enx binding is what blocked Go2 discovery and produced
`ddsi_udp_conn_write ... retcode -3` when a Go2 unicast peer was added.

Fixed in `src/go2_bringup/config/dds/cyclone_dds.xml` → single enx interface.
RMW is correct as-is: apt `ros-humble-rmw-cyclonedds-cpp` ships CycloneDDS 0.10.x,
matching the Go2 SDK2 0.10.2. **Do not rebuild CycloneDDS from source on Humble.**

## Pre-reqs (every session)

- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `export CYCLONEDDS_URI=file://$PWD/src/go2_bringup/config/dds/cyclone_dds.xml`
- `export ROS_DOMAIN_ID=0`  (Go2 default — must be 0)
- Confirm enx name in the XML matches `ip a` (interface on 192.168.123.x). Go2 at .161.
- **Bridge stays OFF.** Steps 1–2 prove the link with zero motion risk.

## Step 1 — same-host discovery (NOT `ros2 node list`)

`ros2 node list` / `topic list` hang under cyclonedds regardless of health — the
trap that caused the original misdiagnosis. Use real pub/sub:

```
# shell A
ros2 run demo_nodes_cpp talker
# shell B
ros2 run demo_nodes_cpp listener
```

Listener prints → same-host discovery works on single-enx; the `lo` addition was
a misdiagnosis, safe to leave removed. **If listener is silent → STOP.** Both Go2
and same-host links must work; do not force the revert — revisit the config.

## Step 2 — READ Go2 state (green light, still no motion)

```
ros2 topic echo /sportmodestate --no-daemon       # or /lowstate
```

Frames arrive → DDS link is bidirectional, MOVE writes will land. **This is the
go/no-go gate.** Prove READ before WRITE.

If silent despite step 1 passing → multicast group-join failed on the USB-eth
adapter (tcpdump sees packets without joining the IGMP group; DDS must actually
join). `AllowMulticast=spdp` will NOT fix this (spdp still discovers by multicast).
Fallback → step 3.

## Step 3 — unicast fallback (only if step 2 silent, step 1 green)

Add to the XML `<Domain>`, WITHOUT re-adding lo:

```xml
<Discovery><Peers><Peer address="192.168.123.161"/></Peers></Discovery>
```

This directly tests the research claim that the `-3` write error was lo+enx
routing, not the peer itself. Re-run step 2.

## Step 4 — motion (gated, first real autonomous motion)

Only after step 2 green AND operator confirms: **robot STANDING (sport mode),
space clear, e-stop in hand.**

```
# bridge is launched OFF by real_navigation.launch.py; enable explicitly:
ros2 service call /go2_sport_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

Velocity caps (`max_vx`/`max_vyaw`) + cmd_vel watchdog already enforced in the
bridge. Disable (`{data: false}`) stops the robot.
