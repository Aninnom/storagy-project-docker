#!/usr/bin/env python3
"""Top-level bringup launch file for the Storagy project.

Loads a pre-built map (mapping was done elsewhere) and brings up Nav2
localization + navigation. Drop your map files into
`src/storagy/map/` and point the `map` argument at the .yaml:

  ros2 launch storagy bringup.launch.py map:=<name>.yaml

The default is `map/<DEFAULT_MAP>` below — change it once your map is ready.

Note: this skeleton wires up map loading + Nav2 only. A running robot /
Gazebo simulation (or a real robot) must still publish /scan and TF for
AMCL to localize against the map — add that here as the project grows.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Default map file name (lives in src/storagy/map/). Update once mapping is done.
DEFAULT_MAP = 'map.yaml'


def generate_launch_description():
    storagy_share = get_package_share_directory('storagy')
    default_map_path = os.path.join(storagy_share, 'map', DEFAULT_MAP)

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_nav2 = LaunchConfiguration('use_nav2')

    # Nav2's stock bringup: map_server + AMCL localization + the nav stack,
    # all reading the map passed via `map:=`.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py',
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to the map .yaml to load',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Bring up Nav2 (map_server + AMCL + navigation)',
        ),
        # TODO: add Gazebo / robot_state_publisher / ros_gz bridge (or real
        # robot drivers) so /scan and TF are published for AMCL.
        nav2_bringup,
    ])
