# Real-robot parking (`real-robot` branch)

This branch runs the parking application **on the physical Storagy robot** (no
Gazebo). `main` is the Docker/Gazebo simulation; this branch adds the real
hardware bring-up, lidar odometry, and bays re-measured in the real map frame.
Most files are shared with `main`; the diff is the real-robot delta below.

## What differs from `main`

| Area | Change |
|---|---|
| `src/motor_driver2/` | Real wheel/motor driver package. Its `odom->base_footprint` TF broadcast is **disabled** (see odometry note). |
| `launch/bringup.launch.py` | Adds an **rf2o_laser_odometry** node that now owns `odom->base_footprint`. |
| `launch/hardware_bringup.launch.py` | Real SICK lidar + Orbbec camera + motor driver bring-up. Orbbec set to `enable_colored_point_cloud:=true` + `depth_registration:=true` for the line detector. |
| `launch/parking_real.launch.py` | One-shot real-robot parking: bring-up + the 3 parking nodes. |
| `launch/parking_nodes.launch.py` | Just the 3 parking nodes (run after bring-up + a 2D Pose Estimate). |
| `scripts/run_parking_real.sh` | Kills orphan nodes (SICK/motor/AMCL don't die on Ctrl-C and hold serial/lidar/USB), then launches. **Always start the real demo through this.** |
| `param/parking_spaces.yaml` | 4 bays measured in the **real** map frame (`map/parkinglot.pgm`), not the Gazebo SDF frame. |
| `config/sick_scan_xd/` | SICK TiM front-lidar config. |

## Prerequisites

- ROS 2 Humble on the robot (`/opt/ros/humble`).
- External deps are **not vendored** — pull them with vcstool:
  ```bash
  cd <workspace>            # dir containing src/
  vcs import src < real-robot.repos
  colcon build --symlink-install
  ```
  (Currently just `rf2o_laser_odometry` @ MAPIRlab.)

## Run

```bash
# from the robot's desktop terminal (RViz needs a display)
src/storagy/scripts/run_parking_real.sh
#   ... run_parking_real.sh use_line_detector:=false   # lidar-only fallback
```
Then set a **2D Pose Estimate** in RViz so `map->odom` is correct before parking
starts. Watch state with `ros2 topic echo /parking/state`.

> Teleop must remap `/cmd_vel:=/cmd_vel_nav` (Nav2's velocity_smoother owns
> `/cmd_vel`).

## Odometry note (why rf2o)

The wheel odometry is miscalibrated in the **MCU firmware** (robot drives straight
but odom reports a large false yaw) and the firmware is not modifiable here. There
is no usable IMU. The fix is **rf2o_laser_odometry** (lidar scan-matching), which
publishes `odom->base_footprint`; `motor_driver2`'s own odom TF broadcast is
disabled so the two don't fight. Verified: 2 m straight → ~2° error (was ~113°).

## Bay coordinates

`param/parking_spaces.yaml` holds 4 bays in the `map` frame, measured by
teleop-parking the robot in each bay and averaging `map->base_footprint`. P3/P4
were re-measured 2026-06-26 (they had been extrapolated while the lidar was down).
See the comment block at the top of that file for the per-bay method and pitch.

## Background

See [`docs/real-robot-debug-2026-06-25.md`](docs/real-robot-debug-2026-06-25.md)
for the full bring-up / debugging log (odometry root-cause, rf2o integration,
first end-to-end docked run).
