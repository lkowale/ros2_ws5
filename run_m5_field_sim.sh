#!/bin/bash
# M4 Field navigator sim — tool slider + RS planner headland turns.
#
# ── Launch core sim ───────────────────────────────────────────────────────────
#   bash run_m4_field_sim.sh
#   HEADLESS=False bash run_m4_field_sim.sh     # show Gazebo GUI
#
# ── Send field mission ────────────────────────────────────────────────────────
#   bash run_m4_field_sim.sh field <field_name> [start_line]
#
# ── Cancel current mission ────────────────────────────────────────────────────
#   bash run_m4_field_sim.sh cancel
#
# Environment:
#   HEADLESS=True|False       Gazebo GUI   (default: True)
#   HEADING_OFFSET=<deg>      GPS antenna heading offset (default: 0.0)
#   VISION_DEBUG=True|False   Publish crop_row_vision_node's annotated debug
#                             image on crop_row_vision/debug_image (default: False)
#                             View with: ros2 run image_view image_view --ros-args \
#                               -r image:=/crop_row_vision/debug_image
#   ROW_BIAS_STEP_M=<m>       Simulated GPS/chassis imperfection: both crop
#                             rows in a pair drift together by ±this much per
#                             spawned segment (triangle wave, bounded to
#                             ±ROW_BIAS_MAX_M). Set to 0 for perfectly
#                             straight rows. (default: 0.02)
#   ROW_BIAS_MAX_M=<m>        Bound for the drift above. (default: 0.05)
#   SPAWN_FIELD=<field_name>  Spawn robot at a swath start instead of (0,0,0).
#   SPAWN_LINE=<line_index>   Swath (geojson feature) index to spawn at, e.g.
#                             SPAWN_FIELD=house_short SPAWN_LINE=6 bash run_m4_field_sim.sh
#                             Pose is computed by spawn_at_swath.py using the
#                             same lat/lon->world projection sim_gps_fix_publisher.py
#                             uses at runtime, so it matches where the EKF/GPS
#                             pipeline will itself place the robot on that swath
#                             — not Gazebo's separate (unfused) wheel odometry.
#                             After spawning, send the mission with the matching
#                             start_line so the BT resumes at the same swath:
#                             bash run_m4_field_sim.sh field house_short 6

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

if [[ "${1:-}" == "field" ]]; then
    _set_cli_cyclone
    FIELD="${2:?Usage: run_m4_field_sim.sh field <field_name> [start_line]}"
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
    ros2 action cancel /run_field 2>/dev/null || true
    exit 0
fi

# ── Launch ────────────────────────────────────────────────────────────────────
# Apply the same CycloneDDS participant-index override the field/cancel CLI
# helpers use — the main launch tree spawns many nodes (nav2 container, tool
# slider, crop row spawner, loggers, ...) and without this the default
# participant pool can run out ("Failed to find a free participant index"),
# which is what silently killed a later `field` mission-send in the past.
_set_cli_cyclone

HEADLESS="${HEADLESS:-True}"
HEADING_OFFSET="${HEADING_OFFSET:-0.0}"
VISION_DEBUG="${VISION_DEBUG:-False}"

SPAWN_ARGS=()
if [[ -n "${SPAWN_FIELD:-}" && -n "${SPAWN_LINE:-}" ]]; then
    read -r X_POSE Y_POSE YAW < <(
        python3 "$HOME/ros2_ws5/spawn_at_swath.py" "$SPAWN_FIELD" "$SPAWN_LINE" \
            | tail -1 | sed 's/x_pose:=//; s/y_pose:=//; s/yaw:=//'
    )
    echo "Spawning at $SPAWN_FIELD line $SPAWN_LINE: x=$X_POSE y=$Y_POSE yaw=$YAW"
    SPAWN_ARGS=("x_pose:=$X_POSE" "y_pose:=$Y_POSE" "yaw:=$YAW")
fi

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':')

echo "Cleaning up existing sim/nav processes..."
pkill -9 -f "gz sim|ruby.*gz" 2>/dev/null || true
pkill -9 -f "ekf_node|ekf_filter_node_odom|navsat_transform|relposned_heading" 2>/dev/null || true
pkill -9 -f "sim_relposned|sim_gps_fix|navsat_init|covariance_injector|parameter_bridge|robot_state_pub" 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|nav2_container" 2>/dev/null || true
pkill -f "plant_row_detector_sim\|tool_slider_controller\|tool_actuator_sim\|field_nav_logger.py\|tool_slider_logger.py\|crop_row_spawner" 2>/dev/null || true
sleep 2

LOG_DIR="$HOME/ros2_ws5/logs/m5_field_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/m5_field_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 M4 — Field Navigator + Tool Slider Sim"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Vision debug   : $VISION_DEBUG  (crop_row_vision/debug_image)"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Send mission : bash run_m4_field_sim.sh field <field_name> [start_line]"
echo "  Cancel       : bash run_m4_field_sim.sh cancel"
echo ""
echo "Available fields:"
find "$FIELDS_DIR" -maxdepth 2 -name "*_directed_turns.geojson" \
    | sed "s|$FIELDS_DIR/||" | sed 's|/.*||' | sort -u | sed 's/^/  /'
echo "=========================================="

{
    echo "=== solbot5 M4 field-nav sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD 2>/dev/null) $(git -C "$HOME/ros2_ws5" log -1 --format='%s' 2>/dev/null)"
} | tee "$LOG_FILE"

cleanup() {
    echo ""
    echo "Shutting down field-nav logger, tool-slider logger, and crop row spawner..."
    kill "$LOGGER_PID" 2>/dev/null || true
    wait "$LOGGER_PID" 2>/dev/null || true
    kill "$TOOL_SLIDER_LOGGER_PID" 2>/dev/null || true
    wait "$TOOL_SLIDER_LOGGER_PID" 2>/dev/null || true
    kill "$CROP_ROW_PID" 2>/dev/null || true
    wait "$CROP_ROW_PID" 2>/dev/null || true
    pkill -f "crop_row_spawner" 2>/dev/null || true
}
trap cleanup EXIT

python3 "$HOME/ros2_ws5/field_nav_logger.py" &
LOGGER_PID=$!
echo "Started field_nav_logger (PID: $LOGGER_PID) → $LOG_DIR/"

python3 "$HOME/ros2_ws5/tool_slider_logger.py" &
TOOL_SLIDER_LOGGER_PID=$!
echo "Started tool_slider_logger (PID: $TOOL_SLIDER_LOGGER_PID) → logs/tool_slider/"

# Rolling crop-row spawner: keeps one row segment under the robot's camera on
# its current swath, spawned ahead and removed once passed. Waits for /fromLL
# itself, so no fixed startup delay needed here — just background it now.
(
    sleep 15
    ros2 run gazebo_crop_rows crop_row_spawner --ros-args \
        -p field_file:="$HOME/ros2_ws5/src/fields/house_short/house_short_lines_0.77m_directed_turns.geojson" \
        -p world_name:=house_short_crop_rows \
        -p spawn_all:=false \
        -p spawn_ahead_m:=4.0 \
        -p remove_behind_m:=3.0 \
        -p row_offset_m:=0.18 \
        -p row_width_m:=0.06 \
        -p segment_length_m:=2.0 \
        -p min_move_m:=1.5 \
        -p bias_step_m:="${ROW_BIAS_STEP_M:-0.02}" \
        -p bias_max_m:="${ROW_BIAS_MAX_M:-0.05}"
) >> "$LOG_FILE" 2>&1 &
CROP_ROW_PID=$!
echo "Started rolling crop_row_spawner (PID: $CROP_ROW_PID, rolling window)"

ros2 launch gazebo_spawn m4_nav_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    use_vision_detector:=true \
    vision_debug_publish:=$VISION_DEBUG \
    "${SPAWN_ARGS[@]}" \
    2>&1 | tee -a "$LOG_FILE"
