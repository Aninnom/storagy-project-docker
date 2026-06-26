#!/usr/bin/env python3
"""
Parking Manager state machine for Storagy parking demo (Phase 3 + 4).

Orchestrates the high-level parking behaviour:

  1. Wait for /parking/occupancy (from the lidar slot_occupancy_node).
  2. Pick a FREE slot (nearest to the robot by default).
  3. (Phase 3) Drive with Nav2 (NavigateToPose) to an APPROACH pose just outside
     the bay entry (bays open toward -Y, so the pose sits on the -Y side facing
     +Y into the bay).
  4. (Phase 4) DOCK: a closed-loop controller eases the robot straight into the
     bay. Lateral centering uses the camera line detector's measured divider x
     (the two dividers bounding the target bay -> their midpoint = bay centre);
     when the dividers leave the camera FOV near full insertion the last good
     centre is held. Forward depth uses the 2D lidar: stop when the forward cone
     range to the bay's back wall reaches `dock_standoff`.

Dock velocity commands are published to /cmd_vel_nav (the velocity_smoother's
input), NOT /cmd_vel, so the smoother stays the single writer of /cmd_vel and
there is no contention with idle Nav2 output.

Publishes:
  /parking/state   std_msgs/String   "state=DOCKING,target=P2,..."
  <dock_cmd_topic> geometry_msgs/Twist  (default /cmd_vel_nav, dock only)

Subscribes:
  /parking/occupancy   std_msgs/String   "P1=occupied,P2=free,..."
  /parking/lines       std_msgs/String   "D-1.70=seen(905,x=-1.70),..."
  /scan                sensor_msgs/LaserScan

Run:
  python3 src/storagy/scripts/parking_manager_node.py \
      --ros-args -p yaml_path:=<abs path to parking_spaces.yaml>
"""

import math
import os
import re

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# "D-1.70=seen(905,x=-1.70)"  ->  name, measured x  (miss -> not captured)
_LINE_RE = re.compile(r"(D[+-]\d+\.\d+)=seen\(\d+,x=([+-]?\d+\.\d+)\)")


class ParkingManagerNode(Node):
    # High-level states
    IDLE = "IDLE"
    SENDING = "SENDING"
    WAIT_ACCEPT = "WAIT_ACCEPT"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"        # staged at approach pose
    DOCKING = "DOCKING"        # easing into the bay
    DOCKED = "DOCKED"          # parked
    FAILED = "FAILED"

    def __init__(self):
        super().__init__("parking_manager_node")

        default_yaml = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "param",
            "parking_spaces.yaml",
        )
        self.declare_parameter("yaml_path", default_yaml)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("approach_offset", 0.45)
        self.declare_parameter("approach_yaw", math.pi / 2.0)
        self.declare_parameter("selection", "nearest")
        # --- docking ---
        self.declare_parameter("dock_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("dock_vx", 0.08)          # forward speed [m/s]
        # Forward lidar range at which to stop. The lidar sits 0.1525 m ahead of
        # base; the bays' back-wall face is at y=1.668 and parked_pose.y=1.378,
        # so stopping at 1.668-(1.378+0.1525) ~= 0.14 m centres the robot at the
        # parked depth (fully inside the 0.5 m-deep bay).
        self.declare_parameter("dock_standoff", 0.14)    # stop dist to wall [m]
        self.declare_parameter("dock_kp_ct", 1.5)        # cross-track gain
        self.declare_parameter("dock_kp_yaw", 1.2)       # heading gain
        self.declare_parameter("dock_w_max", 0.5)        # max yaw rate [rad/s]
        self.declare_parameter("forward_cone_deg", 10.0)

        yaml_path = self.get_parameter("yaml_path").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.approach_yaw = float(self.get_parameter("approach_yaw").value)
        self.selection = self.get_parameter("selection").value
        self.dock_cmd_topic = self.get_parameter("dock_cmd_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        self.dock_vx = float(self.get_parameter("dock_vx").value)
        self.dock_standoff = float(self.get_parameter("dock_standoff").value)
        self.dock_kp_ct = float(self.get_parameter("dock_kp_ct").value)
        self.dock_kp_yaw = float(self.get_parameter("dock_kp_yaw").value)
        self.dock_w_max = float(self.get_parameter("dock_w_max").value)
        self.forward_cone = math.radians(
            float(self.get_parameter("forward_cone_deg").value)
        )

        self.slots = self._load_slots(yaml_path)
        self.get_logger().info(
            f"Loaded {len(self.slots)} slots: {', '.join(self.slots.keys())}"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.occupancy = {}
        self.line_x = {}           # divider name -> measured x (latest)
        self.latest_scan = None
        self.dock_center = None     # cached bay-centre x from camera
        self.state = self.IDLE
        self.target = None
        self.tried = set()
        self.goal_handle = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/parking/occupancy", self.occupancy_callback, 10
        )
        self.create_subscription(
            String, "/parking/lines", self.lines_callback, 10
        )
        self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos
        )
        self.state_pub = self.create_publisher(String, "/parking/state", 10)
        self.cmd_pub = self.create_publisher(Twist, self.dock_cmd_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.timer = self.create_timer(1.0, self.tick)            # high-level FSM
        self.dock_timer = self.create_timer(0.1, self.dock_tick)  # dock loop 10Hz
        self.get_logger().info("Parking manager ready.")

    def _load_slots(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        slots = {}
        for name, info in data["parking_spaces"].items():
            b = info["bounds"]
            slots[name] = {
                "center": (float(info["center"]["x"]), float(info["center"]["y"])),
                "x_min": float(b["x_min"]),
                "x_max": float(b["x_max"]),
                "y_min": float(b["y_min"]),
            }
        return slots

    # ---- callbacks -------------------------------------------------------
    def occupancy_callback(self, msg: String):
        occ = {}
        for tok in msg.data.split(","):
            tok = tok.strip()
            if "=" in tok:
                name, state = tok.split("=", 1)
                occ[name.strip()] = state.strip()
        self.occupancy = occ

    def lines_callback(self, msg: String):
        self.line_x = {m.group(1): float(m.group(2)) for m in _LINE_RE.finditer(msg.data)}

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    # ---- geometry helpers ------------------------------------------------
    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            return (t.x, t.y, quat_to_yaw(q.x, q.y, q.z, q.w))
        except tf2_ros.TransformException:
            return None

    def _approach_pose(self, name):
        # Generic: approach pose sits `approach_offset` metres OUT from the bay
        # centre on the entry side, facing into the bay (approach_yaw). Works for
        # either entry side (+/- pi/2) instead of assuming the -Y entry.
        cx, cy = self.slots[name]["center"]
        yaw = self.approach_yaw
        d = self.approach_offset
        x = cx - d * math.cos(yaw)
        y = cy - d * math.sin(yaw)
        return (x, y, yaw)

    def _bay_center_x(self, name):
        """Camera-measured bay centre x (mean of the two bounding dividers).

        Falls back to a single divider +/- half-width, then to the cached value.
        Returns None only if never measured."""
        s = self.slots[name]
        dn_lo = f"D{s['x_min']:+.2f}"
        dn_hi = f"D{s['x_max']:+.2f}"
        half = (s["x_max"] - s["x_min"]) / 2.0
        lo = self.line_x.get(dn_lo)
        hi = self.line_x.get(dn_hi)
        if lo is not None and hi is not None:
            self.dock_center = 0.5 * (lo + hi)
        elif lo is not None:
            self.dock_center = lo + half
        elif hi is not None:
            self.dock_center = hi - half
        # else: keep the cached self.dock_center (dividers out of FOV)
        return self.dock_center

    def _forward_distance(self):
        """Min lidar range within +/- forward_cone of straight ahead, or None."""
        scan = self.latest_scan
        if scan is None:
            return None
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        n = ranges.shape[0]
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
            & (np.abs(angles) <= self.forward_cone)
        )
        if not np.any(valid):
            return None
        return float(np.min(ranges[valid]))

    def _select_slot(self):
        free = [
            n for n in self.slots
            if self.occupancy.get(n) == "free" and n not in self.tried
        ]
        if not free:
            return None
        if self.selection == "first":
            return sorted(free)[0]
        pose = self._robot_pose()
        if pose is None:
            return sorted(free)[0]
        rx, ry = pose[0], pose[1]

        def dist(n):
            ax, ay, _ = self._approach_pose(n)
            return math.hypot(ax - rx, ay - ry)

        return min(free, key=dist)

    # ---- high-level FSM --------------------------------------------------
    def tick(self):
        if self.state == self.IDLE:
            if self.occupancy:
                target = self._select_slot()
                if target is None:
                    if all(self.occupancy.get(n) != "free" for n in self.slots):
                        self.get_logger().warn(
                            "No free slot available.", throttle_duration_sec=5.0
                        )
                else:
                    self.target = target
                    self.state = self.SENDING
        elif self.state == self.SENDING:
            self._send_goal()
        self._publish_state()

    def _send_goal(self):
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.1)
            if not self.nav_client.server_is_ready():
                self.get_logger().info(
                    "Waiting for navigate_to_pose action server...",
                    throttle_duration_sec=3.0,
                )
                return

        x, y, yaw = self._approach_pose(self.target)
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Navigating to approach pose of {self.target}: "
            f"x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )
        self.state = self.WAIT_ACCEPT
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f"Goal for {self.target} was rejected.")
            self._on_failure()
            return
        self.goal_handle = handle
        self.state = self.NAVIGATING
        handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(
                f"Arrived at approach pose of {self.target}; starting dock."
            )
            self.dock_center = None
            self.state = self.DOCKING
        else:
            self.get_logger().warn(
                f"Navigation to {self.target} failed (status {status})."
            )
            self._on_failure()

    def _on_failure(self):
        if self.target is not None:
            self.tried.add(self.target)
        self.target = None
        self.goal_handle = None
        self.state = self.IDLE if self._select_slot() is not None else self.FAILED

    # ---- docking loop ----------------------------------------------------
    def dock_tick(self):
        if self.state != self.DOCKING:
            return

        fwd = self._forward_distance()
        if fwd is not None and fwd <= self.dock_standoff:
            self._stop()
            self.get_logger().info(
                f"Docked in {self.target} (front wall {fwd:.3f} m)."
            )
            self.state = self.DOCKED
            return

        pose = self._robot_pose()
        if pose is None:
            self._stop()
            return
        rx, ry, yaw = pose

        center = self._bay_center_x(self.target)
        if center is None:
            center = self.slots[self.target]["center"][0]  # yaml fallback

        # Cross-track: robot's left is -X world (heading ~ +Y), so a +X offset
        # (rx > center) is corrected by turning CCW (positive w).
        e_ct = (rx - center) * math.copysign(1.0, math.sin(self.approach_yaw))
        e_yaw = math.atan2(
            math.sin(self.approach_yaw - yaw), math.cos(self.approach_yaw - yaw)
        )
        w = clamp(
            self.dock_kp_ct * e_ct + self.dock_kp_yaw * e_yaw,
            -self.dock_w_max, self.dock_w_max,
        )

        # Ease forward speed down as the wall approaches.
        if fwd is not None:
            vx = self.dock_vx * clamp((fwd - self.dock_standoff) / 0.20, 0.25, 1.0)
        else:
            vx = self.dock_vx
        self._drive(vx, w)

    def _drive(self, vx, w):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ---- status ----------------------------------------------------------
    def _publish_state(self):
        parts = [f"state={self.state}"]
        if self.target:
            parts.append(f"target={self.target}")
            if self.state in (self.SENDING, self.WAIT_ACCEPT, self.NAVIGATING):
                x, y, yaw = self._approach_pose(self.target)
                parts.append(f"approach=({x:.2f},{y:.2f},{yaw:.2f})")
            elif self.state in (self.DOCKING, self.DOCKED) and self.dock_center is not None:
                parts.append(f"center_x={self.dock_center:.3f}")
        self.state_pub.publish(String(data=",".join(parts)))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down parking manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
