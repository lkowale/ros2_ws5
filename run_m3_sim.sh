#!/bin/bash
# M3 — Reeds-Shepp planner sim for solbot5.
#
# ── Launch ────────────────────────────────────────────────────────────────────
#   bash run_m3_sim.sh
#   HEADLESS=False bash run_m3_sim.sh          # show Gazebo GUI
#
# ── Send a single goal (new terminal) ────────────────────────────────────────
#   bash run_m3_sim.sh goal <x> <y> [yaw_deg]
#     yaw_deg: goal heading in degrees ENU (0=East, 90=North). Default: 90
#
# ── Run the automated RS test suite ──────────────────────────────────────────
#   bash run_m3_sim.sh test
#
# ── Run one-line swath navigator ─────────────────────────────────────────────
#   bash run_m3_sim.sh line [field_name]
#     field_name: directory under src/fields/ containing line.json (default: test_line)
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

_deg2quat_z() {
    # Convert heading degrees to quaternion z,w (yaw only).
    # Returns "z w" string.
    python3 -c "
import math, sys
d=float(sys.argv[1]); y=math.radians(d)
print(f'{math.sin(y/2):.6f} {math.cos(y/2):.6f}')
" "$1"
}

# ── Sub-commands ──────────────────────────────────────────────────────────────
if [[ "${1:-}" == "goal" ]]; then
    _set_cli_cyclone
    X="${2:?Usage: run_m3_sim.sh goal <x> <y> [yaw_deg]}"
    Y="${3:?Usage: run_m3_sim.sh goal <x> <y> [yaw_deg]}"
    YAW="${4:-90}"
    read QZ QW < <(_deg2quat_z "$YAW")
    LOG_FILE="$HOME/ros2_ws5/logs/m3_sim/latest.log"
    echo "NavigateToPose: x=$X y=$Y yaw=${YAW}° (qz=$QZ qw=$QW)" | tee -a "$LOG_FILE"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
        --feedback 2>&1 | tee -a "$LOG_FILE"
    exit 0
fi

if [[ "${1:-}" == "cancel" ]]; then
    _set_cli_cyclone
    echo "Cancelling navigation..."
    ros2 action cancel /navigate_to_pose 2>/dev/null || true
    ros2 action cancel /run_one_line 2>/dev/null || true
    exit 0
fi

if [[ "${1:-}" == "line" ]]; then
    _set_cli_cyclone
    FIELD="${2:-test_line}"
    LOG_FILE="$HOME/ros2_ws5/logs/m3_sim/latest.log"
    echo "RunOneLine: field=$FIELD" | tee -a "$LOG_FILE"
    ros2 action send_goal /run_one_line solbot5_msgs/action/RunOneLine \
        "{field_name: '$FIELD'}" \
        --feedback 2>&1 | tee -a "$LOG_FILE" || true
    exit 0
fi

if [[ "${1:-}" == "test" ]]; then
    _set_cli_cyclone
    LOG_FILE="$HOME/ros2_ws5/logs/m3_sim/latest.log"
    {
    echo ""
    echo "═══════════════════════════════════════════════════"
    echo "  solbot5 M3 — Reeds-Shepp planner test suite"
    echo "  Robot starts at map (0,0), heading=90° (North)"
    echo "═══════════════════════════════════════════════════"
    echo ""
    } | tee -a "$LOG_FILE"

    _goal() {
        local label="$1" x="$2" y="$3" yaw="${4:-90}"
        read QZ QW < <(_deg2quat_z "$yaw")
        {
        echo ""
        echo "────────────────────────────────────────────"
        echo "  GOAL: $label"
        echo "  x=$x  y=$y  yaw=${yaw}° → qz=$QZ qw=$QW"
        echo "────────────────────────────────────────────"
        } | tee -a "$LOG_FILE"
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
            "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" \
            --feedback 2>&1 | tee -a "$LOG_FILE" || true
    }

    # Test 1: straight ahead (should be a short straight RS path)
    _goal "1: straight ahead (North, 15m)" 0 15 90

    # Test 2: forward + turn right (CSC path expected)
    _goal "2: forward + 45° right, 10m diagonal" 10 10 0

    # Test 3: goal behind robot (reverse required, CCC or reverse-CSC)
    _goal "3: directly behind, same heading" 0 -8 90

    # Test 4: sharp left turn, large distance (CSC left arc + straight)
    _goal "4: far left, 90° left turn" -15 10 180

    # Test 5: goal close + rotated 180° (tight CCC path)
    _goal "5: close (3m), 180° flip" 0 3 270

    # Test 6: diagonal far, arbitrary heading
    _goal "6: far diagonal SE, heading East" 20 -5 0

    # Test 7: return to origin heading North
    _goal "7: back to origin, heading North" 0 0 90

    {
    echo ""
    echo "═══════════════════════════════════════════════════"
    echo "  Test suite complete."
    echo "═══════════════════════════════════════════════════"
    } | tee -a "$LOG_FILE"
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
sleep 2

LOG_DIR="$HOME/ros2_ws5/logs/m3_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/m3_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 M3 — RS planner sim"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Single goal : bash run_m3_sim.sh goal <x> <y> [yaw_deg]"
echo "  Test suite  : bash run_m3_sim.sh test"
echo "  Swath line  : bash run_m3_sim.sh line [field_name]"
echo "  Cancel      : bash run_m3_sim.sh cancel"
echo "=========================================="

{
    echo "=== solbot5 M3 sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD 2>/dev/null) $(git -C "$HOME/ros2_ws5" log -1 --format='%s' 2>/dev/null)"
} | tee "$LOG_FILE"

ros2 launch gazebo_spawn m3_nav_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    "$@" 2>&1 | tee -a "$LOG_FILE"
