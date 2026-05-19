# Jetson Go2 Nav Goal-Drive Runbook

First real-hardware autonomous motion. READ before WRITE. Robot STANDING before
drive-enable. ≥1 m clear ahead, physical e-stop in hand. Bridge defaults OFF.

## 0. One-time HOST setup (persists across reboot)

UFW drops the Go2 DDS multicast — this is THE blocker. On the HOST (not container):

    sudo ufw allow in on eno1
    sudo ufw allow 7502/udp     # Ouster lidar
    sudo ufw allow 7503/udp     # Ouster imu
    sudo ufw status verbose     # confirm the eno1 + 7502/7503 rules

## 1. Per-session network (inside the privileged container)

    bash scripts/net_setup_go2.sh          # eno1 addr 192.168.123.222 + rp_filter
    ip a | grep -A2 eno1                    # confirm 192.168.123.222/24 + Go2 at .161
    # DDS env for EVERY operator shell:
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/root/glim-humble-docker/src/go2_bringup/config/dds/cyclone_dds.xml
    export ROS_DOMAIN_ID=0

## G1 — DDS link (READ + WRITE), bridge OFF, zero motion risk

READ — Go2 telemetry must stream (use echo, NOT `ros2 topic list`/`node list` which
hang under cyclonedds):

    ros2 topic echo /lowstate --once        # or /sportmodestate — must print a message

WRITE — our request topic must show the Go2's onboard subscriber as a matched sub.
Start the bridge (still gated OFF) and check matched count > 0:

    ros2 run go2_bringup go2_sport_bridge &
    ros2 topic info /api/sport/request -v   # "Subscription count" >= 1 (the Go2)

GO/NO-GO: both READ rate > 0 and WRITE matched-sub >= 1 before proceeding.

## G2 — manual drive

Stand the robot first (operator), clear space, e-stop ready:

    ros2 topic pub --once /go2_sport_bridge/mode std_msgs/String "{data: stand_up}"
    sleep 4
    ros2 topic pub --once /go2_sport_bridge/mode std_msgs/String "{data: balance_stand}"
    sleep 4

Enable drive and command a small forward move (~0.3 m), then stop:

    ros2 service call /go2_sport_bridge/enable std_srvs/srv/SetBool "{data: true}"
    timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ros2 service call /go2_sport_bridge/enable std_srvs/srv/SetBool "{data: false}"

PASS: robot moves forward ~0.3 m and stops on disable + on cmd_vel timeout.

## G3 — E2E (RViz goal → Nav2 → drive)

Bring up the full nav stack (per docs/jetson-nav-run-order.md), stand + enable as G2,
then set an RViz "2D Goal Pose" INSIDE the mapped area. Expect: Nav2 plans, `/cmd_vel`
flows through the bridge, the Go2 drives toward the goal. Disable + e-stop to end.
