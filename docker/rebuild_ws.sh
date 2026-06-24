#!/bin/bash
# Rebuilds the ROS 2 workspace after editing files in the volume-mounted ./src.
# Run from the host:           docker compose exec storagy-project rebuild_ws.sh
# Or inside the noVNC desktop: rebuild_ws.sh
set -e

source /opt/ros/humble/setup.bash

cd /opt/storagy_project_ws
colcon build --symlink-install
echo
echo "Rebuild complete. In existing terminals, run:"
echo "  source /opt/storagy_project_ws/install/setup.bash"
echo "or open a new terminal so the changes take effect."
