#!/bin/bash
# Runs ON pd600 (solbot4): everything for the bar-follower test in ONE launch / tmux session:
#   core (ros2_ws4 run_core_um982.sh: UM982, EKF, Nav2 field_nav_hw params, rosbridge, Mapviz bridge, ...)
#   + toolbar_service + toolbar_outer_bridge (phone MQTT)
#   + OAK-D camera + camera domain_bridge to the laptop
#   + bar_follower (dry_run; toolbar_service starts disabled)
# = solbot4_toolbar/core_bar_follower.launch.py. Normally started from the laptop:
#   bash pd600.sh start_bar [launch args, e.g. use_bar_follower:=false]
# The field nav task (ws4) is started separately, as before.
#
# Replaces `pd600.sh start` + `pd600.sh camera` + `pd600.sh follow`; do not run those alongside.
# Same "ros2" pkill caveat as run_solbot4_toolbar.sh: start it from an interactive shell
# (pd600.sh types it into tmux).
#
#   CAMERA_MODEL  OAK-D-LITE (default) | OAK-D | ...
#   REMOTE_PEER   laptop Tailscale name (default hp-zbook); env of run_core_um982.sh also applies

set -e

WS4=/home/aa/ros2_ws4
WS5=/home/aa/ros2_ws5

grep -q 'CORE_PKG' "$WS4/run_core_um982.sh" || {
    echo "ERROR: $WS4/run_core_um982.sh has no CORE_PKG/OVERLAY_SETUP hook."
    echo "       cd ~/ros2_ws4 && git checkout feature/toolbar-service"
    exit 1
}
[[ -f "$WS5/install/solbot4_toolbar/share/solbot4_toolbar/launch/core_bar_follower.launch.py" ]] || {
    echo "ERROR: overlay not built / outdated. On the laptop: bash ~/ros2_ws5/pd600.sh deploy"
    exit 1
}
[[ -f "$WS5/DEPLOYED" ]] && echo "ros2_ws5 overlay: $(cat "$WS5/DEPLOYED")"
lsusb | grep -q '03e7:' || echo "WARNING: no Luxonis (03e7) USB device found; check the OAK-D cable"

export REMOTE_PEER="${REMOTE_PEER:-hp-zbook}"
CAMERA_MODEL="${CAMERA_MODEL:-OAK-D-LITE}"

# camera domain_bridge: domain 0 on loopback (robot graph) -> domain 1 on tailscale0 (laptop)
export CAMERA_BRIDGE_CYCLONEDDS_URI="<CycloneDDS>\
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

export CORE_PKG=solbot4_toolbar
export CORE_LAUNCH=core_bar_follower.launch.py
export OVERLAY_SETUP="$WS5/install/setup.bash"
export NAV2_PARAMS="${NAV2_PARAMS:-$WS4/install/solbot4_nav2_bringup/share/solbot4_nav2_bringup/params/nav2_params_field_nav_hw.yaml}"
export BROKER_IP="${BROKER_IP:-pd600}"

cd "$WS4"
# relative path: after exec this process reads "bash run_core_um982.sh" (no "ros2")
exec bash run_core_um982.sh camera_model:="$CAMERA_MODEL" "$@"
