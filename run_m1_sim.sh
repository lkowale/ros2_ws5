#!/bin/bash
# M1 — minimal localization sim for solbot5 (dual-antenna heading).
#
# Brings up Gazebo + robot + the localization path only (no Nav2), to validate
# the relposned_heading pipeline and calibrate the antenna mounting offset.
#
# ── Launch ────────────────────────────────────────────────────────────────────
#   bash run_m1_sim.sh
#   HEADLESS=False bash run_m1_sim.sh                 # show Gazebo GUI
#   HEADING_OFFSET=90 bash run_m1_sim.sh              # apply calibration offset
#
# ── Drive (new terminal) ──────────────────────────────────────────────────────
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#     --ros-args -r cmd_vel:=/cmd_vel_ackermann
#
# ── Calibrate heading (new terminal) ──────────────────────────────────────────
#   bash run_m1_sim.sh calibrate     # prints EKF yaw vs Gazebo ground-truth yaw
#
# Environment:
#   HEADLESS=True|False       Gazebo GUI            (default: True)
#   HEADING_OFFSET=<deg>      antenna offset        (default: 0.0)

set -e

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── calibrate sub-command: compare EKF yaw to Gazebo ground-truth yaw ──────────
if [[ "${1:-}" == "calibrate" ]]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><Discovery>\
<ParticipantIndex>auto</ParticipantIndex>\
<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
    exec python3 - <<'PY'
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def yaw(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

class Cal(Node):
    def __init__(self):
        super().__init__('m1_calibrate')
        self.truth = None
        self.est = None
        self.create_subscription(Odometry, '/odometry/gazebo', self._t, 10)
        self.create_subscription(Odometry, '/odom', self._e, 10)
        self.create_timer(0.5, self._report)
    def _t(self, m): self.truth = yaw(m.pose.pose.orientation)
    def _e(self, m): self.est = yaw(m.pose.pose.orientation)
    def _report(self):
        if self.truth is None or self.est is None:
            self.get_logger().info('waiting for /odometry/gazebo and /odom ...')
            return
        err = (self.est - self.truth + 180) % 360 - 180
        self.get_logger().info(
            f'truth={self.truth:7.2f}  ekf={self.est:7.2f}  '
            f'error={err:7.2f} deg  -> set HEADING_OFFSET={-err:.1f} to correct')

rclpy.init()
try:
    rclpy.spin(Cal())
except KeyboardInterrupt:
    pass
PY
fi

# ── Launch ────────────────────────────────────────────────────────────────────
HEADLESS="${HEADLESS:-True}"
HEADING_OFFSET="${HEADING_OFFSET:-0.0}"

# Remove snap paths to avoid libpthread conflict in Gazebo/RViz.
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':')

echo "Cleaning up existing sim processes..."
pkill -9 -f "gz sim|ruby.*gz" 2>/dev/null || true
pkill -9 -f "ekf_node|ekf_filter_node_odom|navsat_transform|relposned_heading" 2>/dev/null || true
pkill -9 -f "sim_relposned|navsat_init|covariance_injector|parameter_bridge|robot_state_pub" 2>/dev/null || true
sleep 2

# Log every run to a timestamped file (+ a 'latest' symlink) so the full
# stdout/stderr of the stack is reviewable after the fact.
LOG_DIR="$HOME/ros2_ws5/logs/m1_sim"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/m1_sim_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

echo "=========================================="
echo "  solbot5 M1 — minimal localization sim"
echo "=========================================="
echo "  Headless       : $HEADLESS"
echo "  Heading offset : $HEADING_OFFSET deg"
echo "  Log            : $LOG_FILE"
echo ""
echo "  Drive:     ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
echo "               --ros-args -r cmd_vel:=/cmd_vel_ackermann"
echo "  Calibrate: bash run_m1_sim.sh calibrate"
echo "=========================================="

{
    echo "=== solbot5 M1 sim — $(date) ==="
    echo "headless=$HEADLESS  heading_offset_deg=$HEADING_OFFSET"
    echo "git: $(git -C "$HOME/ros2_ws5" rev-parse --short HEAD) $(git -C "$HOME/ros2_ws5" log -1 --format='%s')"
    echo "dirty: $(git -C "$HOME/ros2_ws5" status --porcelain | wc -l) modified/untracked"
} | tee "$LOG_FILE"

ros2 launch solbot5_gazebo_spawn m1_localization_sim.launch.py \
    headless:=$HEADLESS \
    heading_offset_deg:=$HEADING_OFFSET \
    "$@" 2>&1 | tee -a "$LOG_FILE"
