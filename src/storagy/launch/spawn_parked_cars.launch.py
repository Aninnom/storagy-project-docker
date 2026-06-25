"""
Spawn parked-car boxes into chosen bays to build occupancy scenarios.

The cars are no longer baked into worlds/parkinglot.sdf; this launch spawns one
`models/parked_car` box at the centre of each bay named in the `occupied`
argument (comma-separated, e.g. `occupied:=P2,P4`). Default is `P1,P3`.
Use `occupied:=` (empty) for an empty lot.

Included by parking_demo.launch.py and parking_nav.launch.py. Standalone:
  ros2 launch storagy spawn_parked_cars.launch.py occupied:=P1,P2,P3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Bay centre x [m] in the map/world frame (matches param/parking_spaces.yaml).
SLOT_X = {"P1": -1.925, "P2": -1.475, "P3": -1.025, "P4": -0.575}
BAY_Y = 1.378   # bay centre y
CAR_Z = 0.15    # box half-height -> rests on the floor
WORLD = "parkinglot"


def _spawn_cars(context, *args, **kwargs):
    pkg = get_package_share_directory("storagy")
    model = os.path.join(pkg, "models", "parked_car", "model.sdf")

    raw = LaunchConfiguration("occupied").perform(context)
    slots = [s.strip().upper() for s in raw.split(",") if s.strip()]

    nodes = []
    for slot in slots:
        if slot not in SLOT_X:
            print(f"[spawn_parked_cars] WARNING: unknown bay '{slot}', skipping "
                  f"(valid: {', '.join(SLOT_X)})")
            continue
        nodes.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_car_{slot}",
            arguments=[
                "-world", WORLD,
                "-file", model,
                "-name", f"car_{slot}",
                "-x", str(SLOT_X[slot]),
                "-y", str(BAY_Y),
                "-z", str(CAR_Z),
            ],
            output="screen",
        ))
    if slots and not nodes:
        print("[spawn_parked_cars] no valid bays -> empty lot")
    # Delay so the gz world is up before we spawn into it.
    return [TimerAction(period=5.0, actions=nodes)] if nodes else []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "occupied",
            default_value="P1,P3",
            description="Comma-separated bays to fill with parked cars, e.g. P2,P4. "
                        "Empty for no cars.",
        ),
        OpaqueFunction(function=_spawn_cars),
    ])
