#!/bin/bash
# Clean up ALL stale storagy/Nav2/hardware ROS nodes, then launch exactly one
# real-robot parking stack. Re-running the launch without this leaves orphan
# nodes alive -- the SICK lidar node and motor_driver2 in particular do NOT die
# on Ctrl-C, and they hold exclusive resources (lidar TCP socket, /dev/ttyUSB0),
# so the next launch fails with SIGSEGV / "multiple access on port". Always start
# the real parking demo through this script.
#
# Run from the robot's DESKTOP terminal (RViz needs a display):
#   ~/Desktop/storagy_project_ws/src/storagy/scripts/run_parking_real.sh
#   ... run_parking_real.sh use_line_detector:=false      # lidar-only fallback
#
# Pass any parking_real.launch.py args straight through.

# NOTE: no `set -u` -- ROS setup.bash references unset vars.

PATTERNS='motor_driver2|sick_generic_caller|nav2_|lifecycle_manager|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|map_server|component_container|robot_state_publisher|slot_occupancy_node|line_detector_node|parking_manager_node|rviz2|ros2 launch storagy'

echo "[clean] killing stale ROS nodes..."
for _ in 1 2 3 4 5; do
    pids=$(ps -eww -o pid,args | grep -E "$PATTERNS" | grep -v grep \
           | grep -v run_parking_real | awk '{print $1}')
    [ -z "$pids" ] && break
    for p in $pids; do kill -9 "$p" 2>/dev/null; done
    sleep 2
done

remaining=$(ps -eww -o args | grep -E "$PATTERNS" | grep -v grep \
            | grep -v run_parking_real | wc -l)
if [ "$remaining" -ne 0 ]; then
    echo "[clean] WARNING: $remaining node(s) still alive:"
    ps -eww -o pid,args | grep -E "$PATTERNS" | grep -v grep \
        | grep -v run_parking_real | cut -c1-90
else
    echo "[clean] clean -- no stale nodes left."
fi

source /opt/ros/humble/setup.bash
source /home/storagy/Desktop/storagy_project_ws/install/setup.bash

echo "[run] launching parking_real (Ctrl-C to stop)..."
exec ros2 launch storagy parking_real.launch.py "$@"
