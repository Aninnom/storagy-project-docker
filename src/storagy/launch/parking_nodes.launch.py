"""parking_nodes.launch.py -- ONLY the parking application nodes.

Run this AFTER `ros2 launch storagy bringup.launch.py` is up AND you have set a
2D Pose Estimate in RViz (robot localized). Starting the parking manager before
localization makes the robot navigate from a wrong pose (spins in place).

  ros2 launch storagy bringup.launch.py          # terminal 1: hardware+nav2+rf2o+rviz
  # ... set 2D Pose Estimate, confirm localized ...
  ros2 launch storagy parking_nodes.launch.py    # terminal 2: occupancy + manager
  ros2 launch storagy parking_nodes.launch.py use_line_detector:=true   # + camera

Watch:  ros2 topic echo /parking/state   /  /parking/occupancy
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("storagy")
    occ = os.path.join(pkg, "scripts", "slot_occupancy_node.py")
    line = os.path.join(pkg, "scripts", "line_detector_node.py")
    mgr = os.path.join(pkg, "scripts", "parking_manager_node.py")
    yaml = os.path.join(pkg, "param", "parking_spaces.yaml")

    use_line = LaunchConfiguration("use_line_detector")
    points_topic = LaunchConfiguration("points_topic")
    mgr_delay = LaunchConfiguration("manager_delay")

    occupancy = ExecuteProcess(
        cmd=["python3", occ, "--ros-args",
             "-p", "yaml_path:=" + yaml,
             "-p", "use_sim_time:=false",
             "-p", "scan_topic:=/scan",
             "-p", "map_frame:=map"],
        output="screen",
    )

    line_detector = ExecuteProcess(
        cmd=["python3", line, "--ros-args",
             "-p", "yaml_path:=" + yaml,
             "-p", "use_sim_time:=false",
             "-p", "map_frame:=map",
             "-p", ["points_topic:=", points_topic]],
        condition=IfCondition(use_line),
        output="screen",
    )

    # Manager starts after a delay so you have time to confirm localization and
    # occupancy first. Increase manager_delay if you want more time.
    parking_manager = TimerAction(
        period=LaunchConfiguration("manager_delay"),
        actions=[ExecuteProcess(
            cmd=["python3", mgr, "--ros-args",
                 "-p", "yaml_path:=" + yaml,
                 "-p", "use_sim_time:=false",
                 "-p", "map_frame:=map",
                 "-p", "base_frame:=base_footprint",
                 "-p", "scan_topic:=/scan",
                 # Entry is on the +Y side; approach the bay from +Y facing -Y.
                 "-p", "approach_yaw:=-1.5708",
                 "-p", "approach_offset:=0.70"],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_line_detector", default_value="false"),
        DeclareLaunchArgument("points_topic", default_value="/camera/depth/points"),
        DeclareLaunchArgument("manager_delay", default_value="5.0"),
        occupancy,
        line_detector,
        parking_manager,
    ])
