#!/bin/bash
# Field navigator sim for solbot5 — RS planner headland turns.
#
# ── Launch core sim ───────────────────────────────────────────────────────────
#   bash run_field_sim.sh
#   HEADLESS=False bash run_field_sim.sh        # show Gazebo GUI
#
# ── Send field mission ────────────────────────────────────────────────────────
#   bash run_field_sim.sh field <field_name> [start_line]
#     field_name : directory under src/fields/ with *_directed_turns.geojson
#     start_line : line index to start from (default: 0; -1 = resume from progress)
#
# ── Cancel current mission ────────────────────────────────────────────────────
#   bash run_field_sim.sh cancel
#
# Environment:
#   HEADLESS=True|False       Gazebo GUI   (default: True)
#   HEADING_OFFSET=<deg>      GPS antenna heading offset (default: 0.0)

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

FIELDS_DIR="/home/aa/ros2_ws5/src/fields"

# ── Sub-commands ──────────────────────────────────────────────────────────────
if [[ "${1:-}" == "field" ]]; then
    _set_cli_cyclone
    FIELD="${2:?Usage: run_field_sim.sh field <field_name> [start_line]}"
    START="${3:-0}"
    GEOJSON=$(find "$FIELDS_DIR/$FIELD" -maxdepth 1 -name "*_directed_turns.geojson" 2>/dev/null | head -1)
    if [[ -z "$GEOJSON" ]]; then
        echo "ERROR: No *_directed_turns.geojson found in $FIELDS_DIR/$FIELD"
        echo "Available fields:"
        find "$FIELDS_DIR" -maxdepth 2 -name "*_directed_turns.geojson" \
            | sed "s|$FIELDS_DIR/||" | sed 's|/.*||' | sort -u | sed 's/^/  /'
        exit 1
    fi
    NLINES=$(python3 -c "import json,sys; d=json.load(open('$GEOJSON')); print(len(d['features']))" 2>/dev/null || echo "?")

    python3 "$HOME/ros2_ws5/rs_controller_logger.py" &
    LOGGER_PID=$!
    sleep 2

    echo "RunField: field=$FIELD  start_line=$START  lines=$NLINES  ($GEOJSON)"
    ros2 action send_goal /run_field solbot5_msgs/action/RunField \
        "{field_name: '$FIELD', start_line_index: $START}" \
        --feedback || true

    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
    exit 0
fi

if [[ "${1:-}" == "cancel" ]]; then
    _set_cli_cyclone
    echo "Cancelling field navigation..."
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
pkill -f "field_nav_logger.py" 2>/dev/null || true
sleep 2

LOG_DIR="$HOME/ros2_ws5/logs/field_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/field_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 — Field Navigator Sim"
echo "  RS planner headland turns"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Send mission : bash run_field_sim.sh field <field_name> [start_line]"
echo "  Cancel       : bash run_field_sim.sh cancel"
echo ""
echo "Available fields:"
find "$FIELDS_DIR" -maxdepth 2 -name "*_directed_turns.geojson" \
    | sed "s|$FIELDS_DIR/||" | sed 's|/.*||' | sort -u | sed 's/^/  /'
echo "=========================================="

{
    echo "=== solbot5 field-nav sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD 2>/dev/null) $(git -C "$HOME/ros2_ws5" log -1 --format='%s' 2>/dev/null)"
} | tee "$LOG_FILE"

cleanup() {
    echo ""
    echo "Shutting down field-nav logger..."
    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Start CSV logger — waits for Nav2 to come up before it gets any data, harmless.
python3 "$HOME/ros2_ws5/field_nav_logger.py" &
LOGGER_PID=$!
echo "Started field_nav_logger (PID: $LOGGER_PID) → $LOG_DIR/field_nav_*.csv"

ros2 launch gazebo_spawn m3_nav_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    2>&1 | tee -a "$LOG_FILE"
