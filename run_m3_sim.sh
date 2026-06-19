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

# Reset ROS environment to avoid contamination from other workspaces (e.g. ws4).
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH ROS_PACKAGE_PATH

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
    echo "  Robot starts at map (0,0), heading=0° (East)"
    echo "═══════════════════════════════════════════════════"
    echo ""
    } | tee -a "$LOG_FILE"

    # Start the thorough logger in the background; let it connect before goals.
    python3 "$HOME/ros2_ws5/m3_sim_logger.py" --hz 2 &
    LOGGER_PID=$!
    sleep 2

    # Build goal list as YAML for RunRsTest action.
    # Each entry: {header: {frame_id: map}, pose: {position: {x,y,z}, orientation: {x,y,z,w}}}
    _pose() {
        local x="$1" y="$2" yaw="${3:-90}"
        read QZ QW < <(_deg2quat_z "$yaw")
        echo "{header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}"
    }

    G1=$(_pose  0  15  90)   # straight ahead North
    G2=$(_pose 10  10   0)   # forward + right, heading East
    G3=$(_pose  0  -8  90)   # behind robot, same heading
    G4=$(_pose -15 10 180)   # far left, heading West
    G5=$(_pose  0   3 270)   # close, 180° flip
    G6=$(_pose 20  -5   0)   # far diagonal SE, heading East
    G7=$(_pose  0   0  90)   # back to origin, heading North

    GOALS="[$G1, $G2, $G3, $G4, $G5, $G6, $G7]"
    LABELS='["1:North 15m", "2:SE diagonal", "3:behind", "4:far left", "5:180flip", "6:far SE", "7:origin"]'

    echo "Sending RunRsTest (7 goals)..." | tee -a "$LOG_FILE"
    ros2 action send_goal /run_rs_test solbot5_msgs/action/RunRsTest \
        "{goals: $GOALS, labels: $LABELS}" \
        --feedback 2>&1 | tee -a "$LOG_FILE" || true

    {
    echo ""
    echo "═══════════════════════════════════════════════════"
    echo "  Test suite complete."
    echo "═══════════════════════════════════════════════════"
    } | tee -a "$LOG_FILE"

    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
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
