#!/bin/bash
# Restart the solbot5 localization layer standalone, without touching the
# hardware/sim layer. Brings up:
#   relposned_heading, navsat_transform, navsat_init, ekf_filter_node_odom,
#   map->odom static TF
#
# navsat_init is part of this layer, so it re-sets navsat_transform's /datum on
# every restart (navsat_transform comes up datum-less). That closes the gap
# where a localization-only restart left navsat without a datum.
#
# Usage:
#   bash run_localization.sh                 # start (real robot)
#   bash run_localization.sh restart         # kill existing localization + start
#   SIM=1 bash run_localization.sh restart   # against a running sim
#
# Environment:
#   SIM=1               use sim time + sim topic wiring (gps_odom_topic=odometry/gps_raw)
#   HEADING_OFFSET=<deg> antenna mounting offset (default 0.0)

set -e

if [ "${1:-}" = "restart" ]; then
    echo "Killing existing localization nodes..."
    pkill -f "relposned_heading|navsat_transform|navsat_init|ekf_node|ekf_filter_node_odom|map_to_odom" 2>/dev/null || true
    sleep 2
    shift   # consume 'restart' so it isn't passed to ros2 launch
fi

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

HEADING_OFFSET="${HEADING_OFFSET:-0.0}"

if [ "${SIM:-0}" = "1" ]; then
    # Match the sim CLI DDS so we can join a running Gazebo session.
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery>\
<ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
    USE_SIM_TIME=true
    GPS_ODOM_TOPIC=odometry/gps_raw
else
    # Real robot: loopback-only domain (matches the core stack).
    export CYCLONEDDS_URI="<CycloneDDS><Domain>\
<General><AllowMulticast>false</AllowMulticast>\
<NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General>\
<Discovery><ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>\
<Peers><Peer address=\"localhost\"/></Peers></Discovery></Domain></CycloneDDS>"
    USE_SIM_TIME=false
    GPS_ODOM_TOPIC=odometry/gps
fi

echo "=========================================="
echo "  solbot5 localization layer"
echo "=========================================="
echo "  SIM            : ${SIM:-0}"
echo "  use_sim_time   : $USE_SIM_TIME"
echo "  gps_odom_topic : $GPS_ODOM_TOPIC"
echo "  heading_offset : $HEADING_OFFSET deg"
echo "=========================================="

ros2 launch solbot5_localization ekf_relposned.launch.py \
    use_sim_time:=$USE_SIM_TIME \
    gps_odom_topic:=$GPS_ODOM_TOPIC \
    heading_offset_deg:=$HEADING_OFFSET \
    "$@"
