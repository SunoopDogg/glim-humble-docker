# Go2 drive activation — design (resolved 2026-06-16)

How to make the real Go2 move from Nav2, now that the DDS blocker root cause is
confirmed. Supersedes the multicast guesswork in `jetson-go2-dds-link-test.md`.

## Root cause (confirmed, not theory)

The USB-Ethernet adapter `enx32aababacca0` **does not deliver inbound multicast
to the kernel socket layer**. Proven DDS-independently: a raw python UDP socket
joined to 239.255.0.1:7400 on enx receives **0** packets while tcpdump (promisc)
sees 35-38 Go2 SPDP frames in the same window. Immune to every software fix
tried: correct single multicast route on enx, rp_filter=0 all ifaces, PROMISC on,
manual multicast MAC, RX checksum/GRO offload off, eno1 down (no interface
interference). Local multicast works (lo, docker0, enx self-loopback after the
route fix), so the kernel mcast stack is fine — it is the USB-eth driver/HW RX.

The Go2 EDU publishes SPDP discovery + all state via multicast → unreachable over
enx. Unicast-only discovery also failed to matter (write path needs the Go2 to
discover us; even with a unicast `<Peer>` the matched-subscriber count stayed 0).

**Why the reference worked:** the `~/projects/go2-humble-docker` setup drove this
**same Jetson** with the Go2 on the **onboard eno1** NIC (Tegra MGBE), Ouster not
connected (user-confirmed). The onboard NIC delivers inbound multicast correctly;
the USB adapter does not. enx was only introduced in glim to run Go2 + Ouster at
once — and it is the wrong NIC for the multicast-dependent Go2.

## Reference recipe (`~/projects/go2-humble-docker`)

- `scripts/install_deps.sh`: apt `ros-humble-rmw-cyclonedds-cpp` +
  `rosidl-generator-dds-idl`, clones `unitree_ros2`, builds **CycloneDDS
  releases/0.10.x + rmw_cyclonedds (humble) from source** in a `cyclonedds_ws`,
  generates `setup.sh`.
- generated `setup.sh`: sources ROS2 + `cyclonedds_ws/install`, sets
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and
  `CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface
  name="${NETWORK_INTERFACE}" priority="default" multicast="default"/></Interfaces>
  </General></Domain></CycloneDDS>'` — single iface, multicast=default, NO lo,
  NO Peers (matches our deep-research conclusion).
- `NETWORK_INTERFACE` default `eth0` → set to the actual Go2 NIC.

The from-source CycloneDDS is NOT what fixed multicast (raw-socket evidence rules
out the DDS lib). It is belt-and-suspenders from the Foxy-era official guide.
glim already ships apt rmw_cyclonedds_cpp + the unitree_ros2 submodule (messages),
so **apt is the default; from-source cyclonedds_ws is the fallback** only if eno1
discovery flakes.

## PROVEN working topology (`~/ros2_ws`, this Jetson, 2026-03-01)

The user's `~/ros2_ws` ran **Go2 (go2_driver, DDS) + Ouster (ouster_ros)
SIMULTANEOUSLY** on this Jetson via `go2_bringup/launch/go2.launch.py`. The Mar-1
build/run log captures the exact working env:

```
CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="eno1" priority="default" multicast="default"/>
</Interfaces></General></Domain></CycloneDDS>'
ROS_LOCALHOST_ONLY=0
```

Ouster metadata `os-122412001934-metadata.json`: OS-1-32-U0, fw v2.5.3,
`udp_dest=169.254.155.31` (link-local), lidar 46626 / imu 33182.

So the **whole working setup used the single onboard NIC `eno1`**: Go2 DDS on the
192.168.123.x subnet (multicast — works on eno1) AND the Ouster on 169.254.x
link-local, both reaching eno1. The USB-eth adapter was **not used at all**. This
is host-native (desktop session), not the glim container, but the NIC conclusion
holds: eno1 delivers inbound multicast + the Ouster's UDP; enx delivers neither.

**Likely L1 topology:** the Ouster is on the Go2's payload network, so ONE cable
from the Go2 to the Jetson `eno1` carries both (the Go2 has an internal switch).
Confirm with the user. If the Ouster is separately cabled, an external switch into
eno1 achieves the same.

## Design: everything on onboard eno1 (drop the USB adapter)

| Device | Was (glim) | Becomes | Why |
|--------|-----|---------|-----|
| **Go2** | enx (USB) | **eno1 (onboard)** | needs multicast; onboard NIC delivers it (proven Mar-1) |
| **Ouster** | eno1 | **eno1 (stays)** | UDP works on eno1; both share the one onboard NIC (proven Mar-1) |
| **USB-eth (enx)** | Go2 | **unused / removed** | drops ALL inbound UDP (mcast 0/38, unicast 2011→19 socket) |

NOTE: do NOT move the Ouster to enx — enx drops inbound UDP, so the Ouster
(receives LiDAR/IMU UDP) would break there too. Both devices ride eno1, exactly
as the Mar-1 working setup did.

### Network (eno1 carries BOTH subnets)
- Go2: `ip addr add 192.168.123.222/24 dev eno1` (Go2 at .161)
- Ouster: keep its link-local on eno1 (`169.254.x`, auto) or its assigned subnet;
  `udp_dest` = the eno1 host IP on the Ouster's subnet (Mar-1 used 169.254.155.31).
- `ip route add 224.0.0.0/4 dev eno1` (Go2 multicast route)
- `sysctl -w net.ipv4.conf.eno1.rp_filter=0 net.ipv4.conf.all.rp_filter=0`
- One NIC, two subnets (192.168.123.x + 169.254.x) — eno1 handles both.

### DDS
- `go2_bringup/config/dds/cyclone_dds.xml`: `<NetworkInterface name="eno1"
  multicast="default"/>` — single iface, no lo, no Peers. (Working-tree reversal
  already single-iface; change the name enx→eno1.)
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (apt), `ROS_DOMAIN_ID=0`.

### glim nav integration (already mostly wired)
`real_navigation.launch.py` already forces RMW=cyclonedds + CYCLONEDDS_URI and runs
`go2_sport_bridge` (gated OFF). Only deltas: cyclone_dds.xml binds eno1; Ouster
launch args point at the enx interface/IP. The Ouster driver uses raw UDP (not
DDS), so it is unaffected by the DDS interface binding.

## Verification gates (in order, motion last)

0. **eno1 inbound multicast** (the whole premise): Go2 on eno1, then raw socket /
   `ros2 topic echo /sportmodestate` shows frames. If 0 here, escalate — but the
   reference + user history say this works.
1. **same-host**: demo talker/listener (NOT `ros2 node list`).
2. **Go2 READ**: `ros2 topic echo /sportmodestate` → live telemetry (gate restored,
   unlike the enx unicast-only plan which lost it).
3. **write match**: publisher on `/api/sport/request` shows matched-subscriber ≥1.
4. **round-trip**: benign sport query → reply on `/api/sport/response`.
5. **motion**: operator stands Go2, clear space, e-stop; `enable` service; small goal.

## Persistence (all the above is runtime, resets on reboot)

Put the eno1 addr + 224.0.0.0/4 route + rp_filter into a session-start script
(e.g. extend `go2_bringup` bringup or a `scripts/net_setup_go2.sh`). Find what set
`224.0.0.0/4 dev eno1` originally so it lands on the right NIC after the swap.
NICs/IPs reshuffle per power-cycle — verify `ip a` each session.
