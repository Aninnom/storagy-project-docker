"""parking_real.launch.py -- autonomous parking on the REAL robot (no Gazebo).

Runs the parking application (lidar slot-occupancy + camera line-detector +
parking-manager state machine) on top of the real hardware + Nav2 stack from
bringup.launch.py:
  - SICK 2D lidar  -> /scan
  - Orbbec camera  -> /camera/depth/points (colored cloud, see note below)
  - motor_driver2  -> /odom + odom->base_footprint
  - AMCL on map/parkinglot.pgm -> map->odom  (so `map` == parking_spaces.yaml coords)
  - Nav2 (planner/controller/bt) -> navigate_to_pose, velocity_smoother(/cmd_vel_nav)

Bay coordinates live in param/parking_spaces.yaml, expressed in the `map` frame.
PREREQ: the robot must be localized on the parkinglot map first -- set a
"2D Pose Estimate" in RViz so map->odom is correct before parking starts.

The camera line-detector needs an XYZ+RGB cloud. hardware_bringup.launch.py has
been set to launch the Orbbec with enable_colored_point_cloud:=true and
depth_registration:=true; the registered cloud is published in the color optical
frame (which exists in the robot TF tree). If the camera can't deliver a colored
cloud, launch with use_line_detector:=false -- approach/dock still run on lidar
occupancy alone (only the camera-based lateral centering is lost).

Usage:
  ros2 launch storagy parking_real.launch.py
  ros2 launch storagy parking_real.launch.py use_line_detector:=false
  # watch:  ros2 topic echo /parking/state
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("storagy")
    occ_script = os.path.join(pkg, "scripts", "slot_occupancy_node.py")
    line_script = os.path.join(pkg, "scripts", "line_detector_node.py")
    mgr_script = os.path.join(pkg, "scripts", "parking_manager_node.py")
    parking_yaml = os.path.join(pkg, "param", "parking_spaces.yaml")
    parking_rviz = os.path.join(pkg, "rviz", "parking.rviz")

    use_rviz2 = LaunchConfiguration("use_rviz2")
    use_line = LaunchConfiguration("use_line_detector")
    points_topic = LaunchConfiguration("points_topic")

    # Real hardware + Nav2 + AMCL + parkinglot map. Its own RViz is disabled;
    # we run the parking-specific RViz below instead.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "bringup.launch.py")
        ),
        launch_arguments={"use_rviz2": "false"}.items(),
    )

    # Lidar slot occupancy (needs map <- ... <- scan TF; real time).
    occupancy = ExecuteProcess(
        cmd=[
            "python3", occ_script, "--ros-args",
            "-p", "yaml_path:=" + parking_yaml,
            "-p", "use_sim_time:=false",
            "-p", "scan_topic:=/scan",
            "-p", "map_frame:=map",
        ],
        output="screen",
    )

    # Camera line detector (optional; needs colored cloud).
    line_detector = ExecuteProcess(
        cmd=[
            "python3", line_script, "--ros-args",
            "-p", "yaml_path:=" + parking_yaml,
            "-p", "use_sim_time:=false",
            "-p", "map_frame:=map",
            "-p", ["points_topic:=", points_topic],
        ],
        condition=IfCondition(use_line),
        output="screen",
    )

    # Parking-manager state machine. Delayed so Nav2's navigate_to_pose action
    # server is up (it also waits internally, but this trims startup noise).
    parking_manager = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3", mgr_script, "--ros-args",
                    "-p", "yaml_path:=" + parking_yaml,
                    "-p", "use_sim_time:=false",
                    "-p", "map_frame:=map",
                    "-p", "base_frame:=base_footprint",
                    "-p", "scan_topic:=/scan",
                ],
                output="screen",
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", parking_rviz, "--ros-args", "--log-level", "ERROR"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz2),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz2", default_value="true"),
        DeclareLaunchArgument("use_line_detector", default_value="true"),
        DeclareLaunchArgument(
            "points_topic", default_value="/camera/depth/points",
            description="RGB+XYZ cloud topic for the line detector."),
        bringup,
        occupancy,
        line_detector,
        parking_manager,
        rviz,
    ])
