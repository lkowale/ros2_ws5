#!/bin/bash
# Launch Mapviz to observe a running solbot5 sim (or real robot).
#
# Shows: satellite/OSM tiles, /gps/fix (green), EKF /odom (red arrows),
# Nav2 /plan (blue), base_footprint frame (yellow). The map origin comes from
# /local_xy_origin, published by origin_publisher (started in the M2 sim launch).
#
# Usage:
#   bash run_mapviz.sh                      # M=3 for m3, M=2 for m2 (default)
#   bash run_mapviz.sh mapviz_rs_test.mvc   # RS planner test suite view
#
# Attaches to whatever sim/robot is already running on the default DDS domain.

set -e

# Start from a CLEAN library/exec path. If this terminal had ros2_ws4 sourced,
# its install/mapviz_plugins (a different build than system Jazzy) gets loaded
# alongside system mapviz — that mismatch is what breaks tile rendering and
# causes the lag. Wipe inherited overlay paths, then source ONLY ros2_ws5.
unset LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin

source /opt/ros/jazzy/setup.bash
source /home/aa/ros2_ws5/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Use the SAME DDS discovery as the sim launch (which sets no CYCLONEDDS_URI =
# default multicast on localhost). A custom URI here changed discovery and made
# Mapviz see topic names but receive NO data ("No messages received" on every
# display). Match the sim: leave CYCLONEDDS_URI unset.
unset CYCLONEDDS_URI

# Remove snap paths to avoid the libpthread conflict.
export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v snap | tr '\n' ':')

# This machine has both Mesa and NVIDIA EGL/GLX; under Wayland the mixed GL
# context gives blank tiles. Force the Mesa GLX vendor for a consistent context.
export __GLX_VENDOR_LIBRARY_NAME=mesa
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json

# Force XWayland (xcb): Mapviz's Qt/OpenGL tile canvas renders blank under native
# Wayland on this machine. xcb + the X11 GLX path is the reliable combo. Set
# MAPVIZ_WAYLAND=1 to skip this if you ever want native Wayland.
if [ -z "${MAPVIZ_WAYLAND:-}" ]; then
    export QT_QPA_PLATFORM=xcb
    export GDK_BACKEND=x11
fi

# CONFIG can be set explicitly, else M=2/3 selects mapviz_m<N>.mvc.
if [ -n "${CONFIG:-}" ]; then
    : # already set by caller
elif [ -n "${1:-}" ] && [ -f "$HOME/ros2_ws5/src/navigation/nav2_bringup/config/${1}" ]; then
    CONFIG="$HOME/ros2_ws5/src/navigation/nav2_bringup/config/${1}"
else
    M="${M:-2}"
    CONFIG="$HOME/ros2_ws5/src/navigation/nav2_bringup/config/mapviz_m${M}.mvc"
fi

LOG_DIR="$HOME/ros2_ws5/logs/mapviz"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/mapviz_$(date +%Y%m%d_%H%M%S).log"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

# Mapviz ignores -c/config_uri in this build and always loads ~/.mapviz_config.
# So install our config there (back up any existing one first).
if [ -f "$HOME/.mapviz_config" ] && ! cmp -s "$CONFIG" "$HOME/.mapviz_config"; then
    cp "$HOME/.mapviz_config" "$HOME/.mapviz_config.bak.$(date +%s)"
fi
cp "$CONFIG" "$HOME/.mapviz_config"

echo "Launching Mapviz (config installed to ~/.mapviz_config from $CONFIG)"
echo "Log: $LOG_FILE"
# use_sim_time MUST be true when observing the sim: the sim stamps messages with
# sim time (seconds since Gazebo start), but a wall-clock Mapviz sees them as
# ~1.8e9 s old and discards everything ("No messages received" on every display).
# Set USE_SIM_TIME=false when pointing Mapviz at the real robot.
USE_SIM_TIME="${USE_SIM_TIME:-true}"

# Default info log level; MAPVIZ_DEBUG=1 for verbose diagnosis.
LOG_LEVEL="${MAPVIZ_DEBUG:+debug}"; LOG_LEVEL="${LOG_LEVEL:-info}"
ros2 run mapviz mapviz --ros-args \
    -p use_sim_time:=$USE_SIM_TIME \
    --log-level "$LOG_LEVEL" 2>&1 | tee "$LOG_FILE"
