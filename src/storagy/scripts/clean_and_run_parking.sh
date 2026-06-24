#!/bin/bash
# Clean up ANY running parkinglot sim / ROS stack, then launch exactly ONE
# parking demo. Re-running `ros2 launch` without this leaves the previous
# Gazebo server alive; several stacked servers each publish /clock and /tf,
# which makes sim time jump around (TF_OLD_DATA warnings) and breaks the slot
# occupancy node. Always start the demo through this script.
#
# Run it INSIDE the container, from a noVNC desktop terminal so the GUI shows:
#     ~/... $ /opt/storagy_project_ws/src/storagy/scripts/clean_and_run_parking.sh
# or from the host:
#     docker compose exec storagy-project \
#         /opt/storagy_project_ws/src/storagy/scripts/clean_and_run_parking.sh
#
# Pass extra launch args through, e.g. `... clean_and_run_parking.sh use_rviz2:=false`.

# NOTE: do NOT use `set -u`. ROS 2's /opt/ros/humble/setup.bash references unset
# variables (e.g. AMENT_TRACE_SETUP_FILES), which would abort the script before
# it can launch.

PATTERNS='gz sim|/usr/bin/ruby|parameter_bridge|robot_state_publisher|static_transform_publisher|rviz2|slot_occupancy_node|ros_gz_sim|ros2 launch storagy'

echo "[clean] stopping any existing parkinglot sim / ROS stack..."
# Rosetta wraps binaries in /run/rosetta/rosetta, which makes `pkill -f` unreliable,
# so kill by PID resolved from the full command line instead. NOTE: use `ps -eww`
# (no column truncation) — plain `ps -eo pid,args` cuts lines at ~80 cols, and the
# long Rosetta paths push the distinguishing name (e.g. slot_occupancy_node.py)
# past the cut, so the grep silently misses processes. Loop to catch launch
# children as their parent dies.
for _ in 1 2 3 4 5; do
    pids=$(ps -eww -o pid,args | grep -E "$PATTERNS" | grep -v grep | awk '{print $1}')
    [ -z "$pids" ] && break
    for pid in $pids; do kill -9 "$pid" 2>/dev/null; done
    sleep 2
done

remaining=$(ps -eww -o args | grep -E "$PATTERNS" | grep -v grep | wc -l)
if [ "$remaining" -ne 0 ]; then
    echo "[clean] WARNING: $remaining sim process(es) still alive:"
    ps -eww -o pid,args | grep -E "$PATTERNS" | grep -v grep | cut -c1-100
else
    echo "[clean] clean — no sim processes left."
fi

export DISPLAY="${DISPLAY:-:1}"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source /opt/storagy_project_ws/install/setup.bash

echo "[run] launching one parking demo (Ctrl-C to stop)..."
exec ros2 launch storagy parking_demo.launch.py "$@"
