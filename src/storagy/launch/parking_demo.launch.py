"""
Parking demo bringup (Phase 0 + 1).

Brings up the parkinglot simulation WITHOUT slam_toolbox, provides a lightweight
static map->odom transform so the `map` frame coincides with the Gazebo world
frame (and therefore with the coordinates in param/parking_spaces.yaml), and
starts the lidar-based slot occupancy node.

The gz OdometryPublisher reports the robot pose relative to the WORLD origin
(verified empirically: odom->base_footprint already equals the true world pose),
and the Gazebo world frame == the map frame used by param/parking_spaces.yaml.
So map->odom is simply the identity. (An earlier version offset it by the spawn
pose and double-counted it, placing every scan point in the wrong spot so all
slots read "free".) This assumes negligible odom drift, which holds in sim for
this short demo. In a later phase AMCL replaces this static transform.

Usage:
  ros2 launch storagy parking_demo.launch.py
  # then watch:  ros2 topic echo /parking/occupancy
  # in RViz add a MarkerArray display on /parking/occupancy_markers
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_storagy = get_package_share_directory("storagy")

    occupancy_script = os.path.join(pkg_storagy, "scripts", "slot_occupancy_node.py")
    parking_yaml = os.path.join(pkg_storagy, "param", "parking_spaces.yaml")

    # 1. Simulation (SLAM off; Nav2 off). RViz stays on for visualisation.
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

    # 2. Lightweight localisation: static map -> odom = identity (odom is already
    #    reported in world == map coordinates).
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        output="screen",
    )

    # 3. Lidar-based slot occupancy node.
    occupancy = ExecuteProcess(
        cmd=[
            "python3", occupancy_script,
            "--ros-args",
            "-p", f"yaml_path:={parking_yaml}",
            "-p", "use_sim_time:=true",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz2", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        simulation,
        static_map_odom,
        occupancy,
    ])
