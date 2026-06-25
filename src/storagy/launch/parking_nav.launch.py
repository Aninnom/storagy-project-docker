"""
Parking navigation bringup (Phase 0 + 1 + 2 + 3).

Extends parking_demo.launch.py with the Nav2 navigation stack (planner +
controller + bt_navigator + behaviours, NO AMCL / NO map_server) and the
parking-manager state machine.

Localisation is ground-truth odom: the gz OdometryPublisher reports the robot
pose in the world frame, and a static identity map->odom keeps the map frame ==
the gz world == param/parking_spaces.yaml coords. Nav2 therefore needs no map
server; the global costmap is a rolling /scan-based window (see
param/navigation2/storagy_parking.yaml). This is sim-only but removes any map<->
world alignment risk and keeps every node on one consistent frame.

Flow: slot_occupancy_node marks bays free/occupied -> parking_manager_node picks
a free bay and drives (NavigateToPose) to an approach pose just outside it. The
final dock into the bay is Phase 4.

Usage:
  ros2 launch storagy parking_nav.launch.py
  # watch:  ros2 topic echo /parking/state
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_storagy = get_package_share_directory("storagy")

    occupancy_script = os.path.join(pkg_storagy, "scripts", "slot_occupancy_node.py")
    line_script = os.path.join(pkg_storagy, "scripts", "line_detector_node.py")
    manager_script = os.path.join(pkg_storagy, "scripts", "parking_manager_node.py")
    parking_yaml = os.path.join(pkg_storagy, "param", "parking_spaces.yaml")

    nav2_params = os.path.join(
        pkg_storagy, "param", "navigation2", "storagy_parking.yaml"
    )
    nav2_launch_dir = os.path.join(pkg_storagy, "launch", "navigation2")
    bt_dir = os.path.join(pkg_storagy, "behavior_trees")
    nav_to_pose_bt = os.path.join(
        bt_dir, "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    nav_through_poses_bt = os.path.join(
        bt_dir, "navigate_through_poses_w_replanning_and_recovery.xml"
    )

    # 1. Simulation (SLAM off; Nav2 off -- we bring nav up ourselves below).
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_storagy, "launch", "simulation_bringup.launch.py")
        ),
        launch_arguments={
            "use_slam": "false",
            "use_nav2": "false",
            "use_rviz2": LaunchConfiguration("use_rviz2"),
            "use_gui": LaunchConfiguration("use_gui"),
        }.items(),
    )

    # 2. Ground-truth localisation: static map -> odom = identity.
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        output="screen",
    )

    # 3. Lidar slot occupancy node.
    occupancy = ExecuteProcess(
        cmd=[
            "python3", occupancy_script,
            "--ros-args",
            "-p", f"yaml_path:={parking_yaml}",
            "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    # 4. Camera line detector node.
    line_detector = ExecuteProcess(
        cmd=[
            "python3", line_script,
            "--ros-args",
            "-p", f"yaml_path:={parking_yaml}",
            "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    # 5. Nav2 navigation stack (no localisation, no map server).
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params,
            "use_composition": "False",
            "default_nav_to_pose_bt_xml": nav_to_pose_bt,
            "default_nav_through_poses_bt_xml": nav_through_poses_bt,
        }.items(),
    )

    # 6. Parking-manager state machine. Delayed so Nav2 has time to come up
    #    (it also waits for the action server itself, but this avoids noise).
    parking_manager = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3", manager_script,
                    "--ros-args",
                    "-p", f"yaml_path:={parking_yaml}",
                    "-p", "use_sim_time:=true",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz2", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        simulation,
        static_map_odom,
        occupancy,
        line_detector,
        navigation,
        parking_manager,
    ])
