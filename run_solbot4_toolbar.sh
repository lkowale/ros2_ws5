#!/bin/bash
# Runs ON pd600 (solbot4): core stack from ros2_ws4 + sliding toolbar from the
# ros2_ws5 overlay. Normally started from the laptop:  bash pd600.sh start
#
# = the last field session's core start (run_core_um982.sh, field_nav_hw Nav2
#   params, pd600 broker, hp-zbook peer), launching solbot4_toolbar/core_toolbar.launch.py
#   (pd600_core.launch.py + toolbar_service) with ~/ros2_ws5/install sourced after ros2_ws4.
#
# Env vars of run_core_um982.sh still apply (BROKER_IP, REMOTE_PEER, NAV2_PARAMS, ...).
#
# Do not start this with "ros2" anywhere in the command line of this process or its
# parents (e.g. `bash ~/ros2_ws5/run_solbot4_toolbar.sh` from a non-interactive shell):
# run_core_um982.sh begins with `pkill -9 -f "ros2|..."`. pd600.sh start types
# `bash run_solbot4_toolbar.sh` into an interactive tmux shell, which is safe.

set -e

WS4=/home/aa/ros2_ws4
WS5=/home/aa/ros2_ws5

grep -q 'CORE_PKG' "$WS4/run_core_um982.sh" || {
    echo "ERROR: $WS4/run_core_um982.sh has no CORE_PKG/OVERLAY_SETUP hook."
    echo "       cd ~/ros2_ws4 && git checkout feature/toolbar-service"
    exit 1
}
[[ -f "$WS5/install/setup.bash" ]] || {
    echo "ERROR: overlay not built. On the laptop: bash ~/ros2_ws5/pd600.sh deploy"
    exit 1
}
[[ -f "$WS5/DEPLOYED" ]] && echo "ros2_ws5 overlay: $(cat "$WS5/DEPLOYED")"

export CORE_PKG=solbot4_toolbar
export CORE_LAUNCH=core_toolbar.launch.py
export OVERLAY_SETUP="$WS5/install/setup.bash"
export NAV2_PARAMS="${NAV2_PARAMS:-$WS4/install/solbot4_nav2_bringup/share/solbot4_nav2_bringup/params/nav2_params_field_nav_hw.yaml}"
export BROKER_IP="${BROKER_IP:-pd600}"
export REMOTE_PEER="${REMOTE_PEER:-hp-zbook}"

cd "$WS4"
# relative path: after exec this process reads "bash run_core_um982.sh" (no "ros2")
exec bash run_core_um982.sh "$@"
