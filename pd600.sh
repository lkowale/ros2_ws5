#!/bin/bash
# Laptop-side control of solbot4 (pd600) running the ros2_ws5 sliding toolbar.
# Source of truth is this laptop's ros2_ws5; pd600:~/ros2_ws5 is a build-only
# overlay on pd600:~/ros2_ws4 (not a git checkout, don't edit it there).
#
#   bash pd600.sh deploy          rsync toolbar packages + run script to pd600, build there
#   bash pd600.sh start [args]    start core + toolbar in tmux session "solbot4" on pd600
#                                 (args go to ros2 launch, e.g. use_rviz:=false)
#   bash pd600.sh start_bar [args]
#                                 ONE session "solbot4": core + toolbar + OAK-D + camera bridge + bar_follower
#                                 (run_solbot4_bar_follower.sh); replaces start + camera + follow
#   bash pd600.sh attach          watch the stack (detach: Ctrl-b d; Ctrl-c stops it)
#   bash pd600.sh stop            Ctrl-c the stack, wait for it to exit
#   bash pd600.sh status          tmux session + toolbar_service process + deployed version
#   bash pd600.sh state           one /toolbar/state message
#   bash pd600.sh toolbar <cmd> [pos_m]
#                                 enable | disable | set_position <m> | center | hold | auto |
#                                 reset_fault | get_status
#   bash pd600.sh ros2 <args...>  any ros2 CLI command on pd600 (robot DDS domain + overlay)
#   bash pd600.sh camera [args]   start the OAK-D + image bridge in tmux "oakd" (after start)
#   bash pd600.sh camera_stop     stop it
#   bash pd600.sh view [topic]    on the laptop: oakd_view.py window of /oakd/rgb/image_raw/compressed
#   bash pd600.sh follow          bar_follower (red bar -> /tool_bar_position_cmd) in tmux "follow"
#                                 after camera; starts dry_run, see config/toolbar_cam.yaml
#   bash pd600.sh follow_stop     stop it (toolbar_service holds when the stream stops)
#   bash pd600.sh follow_view     on the laptop: view of /bar_follower/image/compressed
#   bash pd600.sh gui             on the laptop: toolbar_gui.py slider window (ssh pipe, no DDS)
#
# The stack runs in tmux on pd600, so a dropped laptop connection does not stop the robot.

set -e

HOST="${PD600_HOST:-pd600}"
SESSION=solbot4
CAM_SESSION=oakd
WS5_LOCAL="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# packages deployed to pd600 (add tool_slider crop_row_vision when running the vision loop there)
PKGS="${PKGS:-toolbar_msgs toolbar solbot4_toolbar}"

# remote environment for ros2 CLI: robot loopback DDS (ws4) + ws5 overlay
REMOTE_ENV='source ~/ros2_ws4/pd600_env.sh && source ~/ros2_ws5/install/setup.bash'

rsh() { ssh -o ConnectTimeout=5 "$HOST" "$@"; }

remote_ros2() {
    local q
    q=$(printf '%q ' "$@")
    ssh -o ConnectTimeout=5 -t "$HOST" "$REMOTE_ENV && ros2 $q"
}

session_running() { rsh "tmux has-session -t $SESSION 2>/dev/null"; }

cmd="${1:-}"; shift || true
case "$cmd" in
deploy)
    cd "$WS5_LOCAL"
    ver="$(git describe --always --dirty) $(git branch --show-current) $(date '+%Y-%m-%d %H:%M:%S') from $(hostname)"
    echo "Deploying ros2_ws5 [$ver] -> $HOST: $PKGS"
    rsh "mkdir -p ~/ros2_ws5/src"
    for p in $PKGS; do
        rsync -az --delete --exclude __pycache__ "src/$p/" "$HOST:ros2_ws5/src/$p/"
    done
    rsync -az run_solbot4_toolbar.sh run_solbot4_bar_follower.sh run_solbot4_camera.sh toolbar_gui_remote.py "$HOST:ros2_ws5/"
    rsh "cat > ~/ros2_ws5/README_PD600 <<'EOF'
Build-only overlay on ~/ros2_ws4, deployed from the laptop's ros2_ws5 by pd600.sh deploy.
Do not edit here: changes are overwritten on the next deploy.
EOF
echo '$ver' > ~/ros2_ws5/DEPLOYED"
    rsh "grep -q CORE_PKG ~/ros2_ws4/run_core_um982.sh" || \
        echo "WARNING: pd600 ros2_ws4 lacks the CORE_PKG hook: cd ~/ros2_ws4 && git checkout feature/toolbar-service"
    rsh "cd ~/ros2_ws5 && source /opt/ros/jazzy/setup.bash && source ~/ros2_ws4/install/setup.bash && \
         colcon build --packages-select $PKGS 2>&1 | tail -n 4"
    if session_running; then echo "NOTE: stack is running; restart it to load the new build (stop + start)."; fi
    ;;
start)
    if session_running; then
        echo "tmux session '$SESSION' already exists on $HOST (bash pd600.sh attach / stop)"; exit 1
    fi
    # neutral tmux argv and a typed command: run_core_um982.sh pkills anything matching "ros2"
    rsh "tmux new-session -d -s $SESSION -x 220 -y 50"
    args=""
    [[ $# -gt 0 ]] && args=$(printf '%q ' "$@")
    rsh "tmux send-keys -t $SESSION 'cd ~/ros2_ws5 && bash run_solbot4_toolbar.sh $args' Enter"
    echo "Started in tmux '$SESSION' on $HOST.  Watch: bash pd600.sh attach   Toolbar: bash pd600.sh state"
    ;;
start_bar)
    if session_running; then
        echo "tmux session '$SESSION' already exists on $HOST (bash pd600.sh attach / stop)"; exit 1
    fi
    for s2 in $CAM_SESSION follow; do
        if rsh "tmux has-session -t $s2 2>/dev/null"; then
            echo "tmux session '$s2' is running on $HOST; stop it first (camera_stop / follow_stop)"; exit 1
        fi
    done
    rsh "tmux new-session -d -s $SESSION -x 220 -y 50"
    args=""
    [[ $# -gt 0 ]] && args=$(printf '%q ' "$@")
    rsh "tmux send-keys -t $SESSION 'cd ~/ros2_ws5 && bash run_solbot4_bar_follower.sh $args' Enter"
    echo "Started core + toolbar + OAK-D + bar_follower in tmux '$SESSION' on $HOST."
    echo "  Watch: bash pd600.sh attach   Camera: bash pd600.sh view   Follower: bash pd600.sh follow_view"
    echo "  Commanding: bash pd600.sh toolbar enable; bash pd600.sh toolbar auto;"
    echo "              bash pd600.sh ros2 param set /bar_follower dry_run false"
    ;;
attach)
    ssh -t "$HOST" "tmux attach -t $SESSION"
    ;;
stop)
    session_running || { echo "not running"; exit 0; }
    rsh "tmux send-keys -t $SESSION C-c"
    for _ in $(seq 1 30); do
        rsh "pgrep -f 'core_(toolbar|bar_follower).launch.p[y]' >/dev/null" || break   # [y]: don't match this shell
        sleep 1
    done
    rsh "tmux kill-session -t $SESSION"
    echo "stopped"
    ;;
status)
    rsh "tmux ls 2>/dev/null | grep '^$SESSION:' || echo 'tmux: not running'; \
         pgrep -fa 'lib/toolbar/toolbar_servic[e]' || echo 'toolbar_service: not running'; \
         echo \"overlay: \$(cat ~/ros2_ws5/DEPLOYED 2>/dev/null || echo not deployed)\"; \
         echo \"ros2_ws4: \$(cd ~/ros2_ws4 && git branch --show-current) \$(cd ~/ros2_ws4 && git log --oneline -1)\""
    ;;
state)
    remote_ros2 topic echo --once /toolbar/state
    ;;
toolbar)
    c="${1:?usage: pd600.sh toolbar <command> [position_m]}"
    pos="${2:-0.0}"
    remote_ros2 service call /toolbar_control toolbar_msgs/srv/ToolbarControl \
        "{command: $c, position: $pos}"
    ;;
ros2)
    remote_ros2 "$@"
    ;;
camera)
    session_running || echo "NOTE: core not running; start it first (bash pd600.sh start), its startup pkill would kill the camera"
    if rsh "tmux has-session -t $CAM_SESSION 2>/dev/null"; then
        echo "tmux session '$CAM_SESSION' already exists on $HOST (bash pd600.sh camera_stop)"; exit 1
    fi
    args=""
    [[ $# -gt 0 ]] && args=$(printf '%q ' "$@")
    rsh "tmux new-session -d -s $CAM_SESSION -x 220 -y 50"
    rsh "tmux send-keys -t $CAM_SESSION 'cd ~/ros2_ws5 && bash run_solbot4_camera.sh $args' Enter"
    echo "Camera started in tmux '$CAM_SESSION' on $HOST.  Logs: ssh -t $HOST tmux attach -t $CAM_SESSION   View: bash pd600.sh view"
    ;;
follow)
    if rsh "tmux has-session -t follow 2>/dev/null"; then
        echo "tmux session 'follow' already exists on $HOST (bash pd600.sh follow_stop)"; exit 1
    fi
    rsh "tmux new-session -d -s follow -x 220 -y 50"
    cal=~/ros2_ws5/install/solbot4_toolbar/share/solbot4_toolbar/config/toolbar_cam.yaml
    rsh "tmux send-keys -t follow '$REMOTE_ENV && ros2 run solbot4_toolbar bar_follower.py --ros-args --params-file $cal $*' Enter"
    echo "bar_follower started in tmux 'follow'.  Logs: ssh -t $HOST tmux attach -t follow  View: bash pd600.sh follow_view"
    echo "Commanding: bash pd600.sh ros2 param set /bar_follower dry_run false ; bash pd600.sh toolbar auto"
    ;;
follow_stop)
    rsh "tmux has-session -t follow 2>/dev/null" || { echo "not running"; exit 0; }
    rsh "tmux send-keys -t follow C-c"; sleep 2; rsh "tmux kill-session -t follow"
    ;;
follow_view)
    exec bash "$0" view /bar_follower/image/compressed
    ;;
camera_stop)
    rsh "tmux has-session -t $CAM_SESSION 2>/dev/null" || { echo "not running"; exit 0; }
    rsh "tmux send-keys -t $CAM_SESSION C-c"
    for _ in $(seq 1 15); do
        rsh "pgrep -f 'oakd.launch.p[y]' >/dev/null" || break
        sleep 1
    done
    rsh "tmux kill-session -t $CAM_SESSION"
    echo "camera stopped"
    ;;
view)
    # laptop side: domain 1 over Tailscale, peer = robot (same as ws4 run_mapviz_remote.sh)
    # env -i: a VS Code snap shell leaks GTK/GIO vars that crash rqt (GLib-GIO-ERROR settings schema)
    env -i PATH=/usr/bin:/bin HOME="$HOME" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
        RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=1 \
        CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><NetworkInterfaceAddress>tailscale0</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex><Peers><Peer address=\"$HOST\"/></Peers></Discovery><Internal><HeartbeatInterval min=\"2s\" max=\"10s\"/></Internal></Domain></CycloneDDS>" \
        WS5="$WS5_LOCAL" bash -c 'source /opt/ros/jazzy/setup.bash && exec python3 "$WS5/oakd_view.py" "$@"' view "$@"
    ;;
gui)
    env -i PATH=/usr/bin:/bin HOME="$HOME" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
        SSH_AUTH_SOCK="${SSH_AUTH_SOCK:-}" PD600_HOST="$HOST" python3 "$WS5_LOCAL/toolbar_gui.py"
    ;;
*)
    sed -n '2,24p' "$0"; exit 1
    ;;
esac
