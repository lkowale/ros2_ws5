#!/bin/bash
# M4 — Vision-guided tool slider sim for solbot5.
#
# ── Launch ────────────────────────────────────────────────────────────────────
#   bash run_m4_sim.sh
#   HEADLESS=False bash run_m4_sim.sh          # show Gazebo GUI
#
# ── Send a single goal (new terminal) ────────────────────────────────────────
#   bash run_m4_sim.sh goal <x> <y> [yaw_deg]
#
# Environment:
#   HEADLESS=True|False       Gazebo GUI            (default: True)
#   HEADING_OFFSET=<deg>      antenna offset        (default: 0.0)

set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH ROS_PACKAGE_PATH LD_LIBRARY_PATH

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_set_cli_cyclone() {
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery>\
<ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
}

_deg2quat_z() {
    python3 -c "
import math, sys
d=float(sys.argv[1]); y=math.radians(d)
print(f'{math.sin(y/2):.6f} {math.cos(y/2):.6f}')
" "$1"
}

if [[ "${1:-}" == "goal" ]]; then
    _set_cli_cyclone
    X="${2:?Usage: run_m4_sim.sh goal <x> <y> [yaw_deg]}"
    Y="${3:?Usage: run_m4_sim.sh goal <x> <y> [yaw_deg]}"
    YAW="${4:-90}"
    read QZ QW < <(_deg2quat_z "$YAW")
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
        --feedback
    exit 0
fi

if [[ "${1:-}" == "cancel" ]]; then
    _set_cli_cyclone
    ros2 action cancel /navigate_to_pose 2>/dev/null || true
    ros2 action cancel /run_field 2>/dev/null || true
    exit 0
fi

# ── Launch ────────────────────────────────────────────────────────────────────
HEADLESS="${HEADLESS:-True}"
HEADING_OFFSET="${HEADING_OFFSET:-0.0}"

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':')

echo "Cleaning up existing sim/nav processes..."
pkill -9 -f "gz sim|ruby.*gz" 2>/dev/null || true
pkill -9 -f "ekf_node|ekf_filter_node_odom|navsat_transform|relposned_heading" 2>/dev/null || true
pkill -9 -f "sim_relposned|sim_gps_fix|navsat_init|covariance_injector|parameter_bridge|robot_state_pub" 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|nav2_container" 2>/dev/null || true
pkill -f "plant_row_detector_sim\|tool_slider_controller\|tool_actuator_sim" 2>/dev/null || true
sleep 2

LOG_DIR="$HOME/ros2_ws5/logs/m4_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/m4_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 M4 — Tool slider sim"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Single goal : bash run_m4_sim.sh goal <x> <y> [yaw_deg]"
echo "  Field run   : bash run_m4_field_sim.sh field <field_name>"
echo "  Cancel      : bash run_m4_sim.sh cancel"
echo "=========================================="

{
    echo "=== solbot5 M4 sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD 2>/dev/null) $(git -C "$HOME/ros2_ws5" log -1 --format='%s' 2>/dev/null)"
} | tee "$LOG_FILE"

ros2 launch gazebo_spawn m4_nav_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    "$@" 2>&1 | tee -a "$LOG_FILE"
