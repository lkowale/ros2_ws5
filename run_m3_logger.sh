#!/bin/bash
# Launch the M3 sim thorough logger.
# Run this in a separate terminal AFTER bash run_m3_sim.sh is up.
#
# Usage:
#   bash run_m3_logger.sh            # 2 Hz sampling
#   bash run_m3_logger.sh --hz 5     # 5 Hz sampling
#   bash run_m3_logger.sh --all      # every message, no rate-limiting
#
# Tail the latest log from yet another terminal:
#   tail -f ~/ros2_ws5/logs/m3_sim/logger_latest.log

set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH ROS_PACKAGE_PATH

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec python3 /home/aa/ros2_ws5/m3_sim_logger.py "$@"
