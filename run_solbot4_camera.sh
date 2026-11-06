#!/bin/bash
# Runs ON pd600 (solbot4): OAK-D camera (solbot4_toolbar/oakd.launch.py) on the robot
# loopback domain 0, plus a domain_bridge forwarding /oakd/rgb/image_raw/compressed
# and /oakd/rgb/camera_info to domain 1 on tailscale0 (hp-zbook: bash pd600.sh view).
# Normally started from the laptop:  bash pd600.sh camera
#
# Start it AFTER the core stack: run_core_um982.sh pkills anything matching "ros2"
# (and "domain_bridge" when it exits), which includes this camera.
#
#   CAMERA_MODEL  OAK-D-LITE (default) | OAK-D | OAK-D-PRO ...
#   REMOTE_PEER   laptop Tailscale name (default hp-zbook)

set -e
WS5=/home/aa/ros2_ws5
REMOTE_PEER="${REMOTE_PEER:-hp-zbook}"
CAMERA_MODEL="${CAMERA_MODEL:-OAK-D-LITE}"

source "$HOME/ros2_ws4/pd600_env.sh"      # jazzy + ws4, cyclone loopback domain 0
source "$WS5/install/setup.bash"

lsusb | grep -q '03e7:' || echo "WARNING: no Luxonis (03e7) USB device found; check the OAK-D cable"

BRIDGE_CYCLONEDDS_URI="<CycloneDDS>\
<Domain id=\"0\"><General><AllowMulticast>false</AllowMulticast>\
<NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General>\
<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>\
<Peers><Peer address=\"localhost\"/></Peers></Discovery>\
<Internal><HeartbeatInterval min=\"2s\" max=\"10s\"/></Internal></Domain>\
<Domain id=\"1\"><General><AllowMulticast>false</AllowMulticast>\
<NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress></General>\
<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>\
<Peers><Peer address=\"${REMOTE_PEER}\"/></Peers></Discovery>\
<Internal><HeartbeatInterval min=\"2s\" max=\"10s\"/></Internal></Domain>\
</CycloneDDS>"

BRIDGE_CONFIG="$(ros2 pkg prefix solbot4_toolbar)/share/solbot4_toolbar/config/camera_bridge.yaml"
CYCLONEDDS_URI="$BRIDGE_CYCLONEDDS_URI" ros2 run domain_bridge domain_bridge "$BRIDGE_CONFIG" &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null || true' EXIT
echo "camera domain_bridge 0 -> 1 ($REMOTE_PEER), PID $BRIDGE_PID"

ros2 launch solbot4_toolbar oakd.launch.py camera_model:="$CAMERA_MODEL" "$@"
