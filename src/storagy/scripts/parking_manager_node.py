#!/usr/bin/env python3
"""
Parking Manager state machine for Storagy parking demo (Phase 3).

Orchestrates the high-level parking behaviour:

  1. Wait for /parking/occupancy (from the lidar slot_occupancy_node).
  2. Pick a FREE slot (nearest to the robot by default).
  3. Compute an APPROACH pose just outside the bay entry and drive there with
     Nav2 (NavigateToPose).  The bays open toward -Y, so the approach pose sits
     at the slot's x, a little on the -Y side of the entry, facing +Y (into the
     bay).  The final precise insertion is left to Phase 4 (dock controller).

This node does NOT do the centimetre-level docking; it gets the robot reliably
staged in front of the chosen bay.  If a goal is aborted it tries the next free
slot.

Publishes:
  /parking/state   std_msgs/String   "state=NAVIGATING,target=P2,..."

Subscribes:
  /parking/occupancy   std_msgs/String   "P1=occupied,P2=free,..."

Run:
  python3 src/storagy/scripts/parking_manager_node.py \
      --ros-args -p yaml_path:=<abs path to parking_spaces.yaml>
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf2_ros


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class ParkingManagerNode(Node):
    # High-level states
    IDLE = "IDLE"              # waiting for occupancy / free slot
    SENDING = "SENDING"        # about to send a nav goal
    WAIT_ACCEPT = "WAIT_ACCEPT"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"        # staged at approach pose (Phase 4 takes over)
    FAILED = "FAILED"          # no reachable free slot

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
        # Distance on the -Y (entry) side of the bay to stage the robot [m].
        self.declare_parameter("approach_offset", 0.45)
        # Heading at the approach pose; bays open toward -Y so face +Y (pi/2).
        self.declare_parameter("approach_yaw", math.pi / 2.0)
        # "nearest" (to the robot) or "first" (P1..P4 order).
        self.declare_parameter("selection", "nearest")

        yaml_path = self.get_parameter("yaml_path").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.approach_offset = float(self.get_parameter("approach_offset").value)
        self.approach_yaw = float(self.get_parameter("approach_yaw").value)
        self.selection = self.get_parameter("selection").value

        self.slots = self._load_slots(yaml_path)
        self.get_logger().info(
            f"Loaded {len(self.slots)} slots: {', '.join(self.slots.keys())}"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.occupancy = {}        # name -> "occupied"/"free"
        self.state = self.IDLE
        self.target = None
        self.tried = set()         # slots already attempted (and failed)
        self.goal_handle = None

        self.occ_sub = self.create_subscription(
            String, "/parking/occupancy", self.occupancy_callback, 10
        )
        self.state_pub = self.create_publisher(String, "/parking/state", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info("Parking manager ready.")

    def _load_slots(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        slots = {}
        for name, info in data["parking_spaces"].items():
            b = info["bounds"]
            slots[name] = {
                "center": (float(info["center"]["x"]), float(info["center"]["y"])),
                "y_min": float(b["y_min"]),
            }
        return slots

    def occupancy_callback(self, msg: String):
        occ = {}
        for tok in msg.data.split(","):
            tok = tok.strip()
            if "=" in tok:
                name, state = tok.split("=", 1)
                occ[name.strip()] = state.strip()
        self.occupancy = occ

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except tf2_ros.TransformException:
            return None

    def _approach_pose(self, name):
        cx, _cy = self.slots[name]["center"]
        y = self.slots[name]["y_min"] - self.approach_offset
        return (cx, y, self.approach_yaw)

    def _select_slot(self):
        free = [
            n for n in self.slots
            if self.occupancy.get(n) == "free" and n not in self.tried
        ]
        if not free:
            return None
        if self.selection == "first":
            return sorted(free)[0]
        # nearest to the robot's current approach point
        robot = self._robot_xy()
        if robot is None:
            return sorted(free)[0]
        rx, ry = robot

        def dist(n):
            ax, ay, _ = self._approach_pose(n)
            return math.hypot(ax - rx, ay - ry)

        return min(free, key=dist)

    # ---- FSM -------------------------------------------------------------
    def tick(self):
        if self.state == self.IDLE:
            if self.occupancy:
                target = self._select_slot()
                if target is None:
                    if self.occupancy and all(
                        self.occupancy.get(n) != "free" for n in self.slots
                    ):
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
        # 4 == STATUS_SUCCEEDED
        if status == 4:
            self.get_logger().info(
                f"Arrived at approach pose of {self.target}. "
                f"(Phase 4 dock controller takes over.)"
            )
            self.state = self.ARRIVED
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
        # Try the next free slot, else give up.
        if self._select_slot() is not None:
            self.state = self.IDLE
        else:
            self.state = self.FAILED

    def _publish_state(self):
        parts = [f"state={self.state}"]
        if self.target:
            x, y, yaw = self._approach_pose(self.target)
            parts.append(f"target={self.target}")
            parts.append(f"approach=({x:.2f},{y:.2f},{yaw:.2f})")
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
