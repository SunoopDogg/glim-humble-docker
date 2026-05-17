# Source (do NOT execute) in EVERY operator terminal that talks to the nav stack —
# `ros2 service call .../enable`, `ros2 topic echo /cmd_vel`, `ros2 topic echo /sportmodestate`.
#
# real_navigation.launch.py sets RMW_IMPLEMENTATION + CYCLONEDDS_URI only for the nodes
# IT spawns. A plain shell defaults to rmw_fastrtps_cpp; cross-RMW does not discover, so
# the enable service hangs "waiting for service to become available..." even when the
# bridge is alive. Sourcing this lines the operator shell up with the launch.
#
# CycloneDDS CLI caveat: `ros2 node/topic list`, `param get`, `lifecycle get` HANG under
# cyclonedds — use pub / echo / service-call only. Never wrap `ros2` in `timeout`.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/root/glim-humble-docker/src/go2_bringup/config/dds/cyclone_dds.xml
export ROS_DOMAIN_ID=0
echo "go2_env: RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID URI=$CYCLONEDDS_URI"
