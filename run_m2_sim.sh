#!/bin/bash
# M2 — Nav2 drive sim for solbot5.
#
# M1 localization sim + Nav2, so the robot can navigate to goals.
#
# ── Launch ────────────────────────────────────────────────────────────────────
#   bash run_m2_sim.sh
#   HEADLESS=False bash run_m2_sim.sh                 # show Gazebo GUI
#
# ── Send a goal (new terminal) ────────────────────────────────────────────────
#   bash run_m2_sim.sh goal 10 0          # x=10 y=0 (map frame)
#   bash run_m2_sim.sh goal 10 5
#
# ── Cancel ────────────────────────────────────────────────────────────────────
#   bash run_m2_sim.sh cancel
#
# Environment:
#   HEADLESS=True|False       Gazebo GUI            (default: True)
#   HEADING_OFFSET=<deg>      antenna offset        (default: 0.0)

set -e

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

_set_cli_cyclone() {
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery>\
<ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
}

# ── Sub-commands ──────────────────────────────────────────────────────────────
if [[ "${1:-}" == "goal" ]]; then
    _set_cli_cyclone
    X="${2:?Usage: run_m2_sim.sh goal <x> <y>}"
    Y="${3:?Usage: run_m2_sim.sh goal <x> <y>}"
    echo "Sending NavigateToPose goal: x=$X y=$Y (map frame)"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}" \
        --feedback
    exit 0
fi

if [[ "${1:-}" == "cancel" ]]; then
    _set_cli_cyclone
    echo "Cancelling all /navigate_to_pose goals..."
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}}" &
    sleep 1; kill %1 2>/dev/null || true
    exit 0
fi

# ── Launch ────────────────────────────────────────────────────────────────────
HEADLESS="${HEADLESS:-True}"
HEADING_OFFSET="${HEADING_OFFSET:-0.0}"

# Remove snap paths to avoid libpthread conflict in Gazebo/RViz.
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':')

echo "Cleaning up existing sim/nav processes..."
pkill -9 -f "gz sim|ruby.*gz" 2>/dev/null || true
pkill -9 -f "ekf_node|ekf_filter_node_odom|navsat_transform|relposned_heading" 2>/dev/null || true
pkill -9 -f "sim_relposned|sim_gps_fix|navsat_init|covariance_injector|parameter_bridge|robot_state_pub" 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|nav2_container" 2>/dev/null || true
sleep 2

LOG_DIR="$HOME/ros2_ws5/logs/m2_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/m2_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 M2 — Nav2 drive sim"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Send goal: bash run_m2_sim.sh goal 10 0"
echo "  Cancel   : bash run_m2_sim.sh cancel"
echo "=========================================="

{
    echo "=== solbot5 M2 sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD) $(git -C "$HOME/ros2_ws5" log -1 --format='%s')"
} | tee "$LOG_FILE"

ros2 launch solbot5_gazebo_spawn m2_nav_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    "$@" 2>&1 | tee -a "$LOG_FILE"
