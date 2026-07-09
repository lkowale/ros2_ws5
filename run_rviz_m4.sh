#!/bin/bash
# Launch RViz2 for M4 tool slider sim.
#
# Shows: robot model (tool_bar + OAK-D camera), TF frames, odometry,
#        nav path, OAK-D RGB camera image, tool_bar position topic.
#
# Usage:
#   bash run_rviz_m4.sh
#   USE_SIM_TIME=false bash run_rviz_m4.sh   # real robot

set -e

unset LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v snap | tr '\n' ':')
export __GLX_VENDOR_LIBRARY_NAME=mesa
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json

if [ -z "${RVIZ_WAYLAND:-}" ]; then
    export QT_QPA_PLATFORM=xcb
    export GDK_BACKEND=x11
fi

USE_SIM_TIME="${USE_SIM_TIME:-true}"
CONFIG="$HOME/ros2_ws5/src/navigation/nav2_bringup/config/rviz_m4.rviz"

echo "Launching RViz2 — M4 tool slider"
echo "  Config : $CONFIG"
echo "  SimTime: $USE_SIM_TIME"

rviz2 -d "$CONFIG" --ros-args -p use_sim_time:=$USE_SIM_TIME
