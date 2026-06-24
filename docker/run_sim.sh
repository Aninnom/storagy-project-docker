#!/bin/bash
# Launches the Storagy simulation inside the noVNC desktop session.
# Run it manually from a terminal inside the desktop (http://localhost:6080),
# or wire it into an autostart entry like the other Storagy repos do.
set -e

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1   # software OpenGL: works without a GPU

# Wait until the VNC X server (:1) is up before starting GUI apps.
for _ in $(seq 1 60); do
    [ -e /tmp/.X11-unix/X1 ] && break
    sleep 1
done
sleep 3

source /opt/ros/humble/setup.bash
source /opt/storagy_project_ws/install/setup.bash

cd /opt/storagy_project_ws
echo "==================================================================="
echo " Starting Storagy project simulation"
echo "==================================================================="
exec ros2 launch storagy bringup.launch.py
