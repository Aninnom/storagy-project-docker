#!/usr/bin/env python3
"""
Slot Occupancy Node for Storagy parking demo (Phase 1).

Decides, for each parking slot defined in param/parking_spaces.yaml, whether it
is OCCUPIED or FREE using the 2D lidar (/scan):

  1. Transform every valid scan beam into the map frame (via TF).
  2. For each slot, count beam endpoints that fall inside the slot's bounds,
     shrunk inward by `inset` to avoid catching walls / divider tape.
  3. If the count >= `point_threshold` the slot is OCCUPIED, else FREE.

Publishes:
  /parking/occupancy           std_msgs/String   "P1=occupied,P2=free,..."
  /parking/occupancy_markers   visualization_msgs/MarkerArray  (RViz overlay:
                               green=free, red=occupied, text shows hit count)

This node only needs a valid map->base_footprint->...-><scan frame> TF chain.
In the Phase-1 demo that is provided by a static map->odom transform (see
parking_demo.launch.py); later it will come from AMCL.

Run:
  python3 src/storagy/scripts/slot_occupancy_node.py \
      --ros-args -p yaml_path:=<abs path to parking_spaces.yaml>
"""

import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


def quat_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    """Build a 4x4 homogeneous transform from translation + quaternion."""
    # Normalize quaternion defensively.
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    m = np.identity(4)
    m[0, 0] = 1.0 - 2.0 * (yy + zz)
    m[0, 1] = 2.0 * (xy - wz)
    m[0, 2] = 2.0 * (xz + wy)
    m[1, 0] = 2.0 * (xy + wz)
    m[1, 1] = 1.0 - 2.0 * (xx + zz)
    m[1, 2] = 2.0 * (yz - wx)
    m[2, 0] = 2.0 * (xz - wy)
    m[2, 1] = 2.0 * (yz + wx)
    m[2, 2] = 1.0 - 2.0 * (xx + yy)
    m[0, 3], m[1, 3], m[2, 3] = tx, ty, tz
    return m


class SlotOccupancyNode(Node):
    def __init__(self):
        super().__init__("slot_occupancy_node")

        default_yaml = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "param",
            "parking_spaces.yaml",
        )
        self.declare_parameter("yaml_path", default_yaml)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("scan_topic", "/scan")
        # Shrink bounds inward [m]. Must stay below the gap between a parked car
        # and the slot line (~0.025 m here) so the car's front face — the main
        # surface the lidar sees — is still counted, while points from a car in
        # the adjacent slot are excluded.
        self.declare_parameter("inset", 0.02)
        self.declare_parameter("point_threshold", 5)   # hits to call OCCUPIED
        self.declare_parameter("publish_period", 0.5)  # status/markers rate [s]

        yaml_path = self.get_parameter("yaml_path").value
        self.map_frame = self.get_parameter("map_frame").value
        scan_topic = self.get_parameter("scan_topic").value
        self.inset = float(self.get_parameter("inset").value)
        self.point_threshold = int(self.get_parameter("point_threshold").value)
        period = float(self.get_parameter("publish_period").value)

        self.slots = self._load_slots(yaml_path)
        self.get_logger().info(
            f"Loaded {len(self.slots)} slots from {yaml_path}: "
            f"{', '.join(self.slots.keys())}"
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest scan + computed occupancy
        self.latest_scan = None
        self.occupancy = {name: ("unknown", 0) for name in self.slots}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, sensor_qos
        )
        self.status_pub = self.create_publisher(String, "/parking/occupancy", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/parking/occupancy_markers", 10
        )

        self.timer = self.create_timer(period, self.update)
        self.get_logger().info("Slot occupancy node ready.")

    def _load_slots(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        slots = {}
        for name, info in data["parking_spaces"].items():
            b = info["bounds"]
            slots[name] = {
                "x_min": float(b["x_min"]),
                "x_max": float(b["x_max"]),
                "y_min": float(b["y_min"]),
                "y_max": float(b["y_max"]),
                "center": (float(info["center"]["x"]), float(info["center"]["y"])),
            }
        return slots

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _scan_points_in_map(self, scan: LaserScan):
        """Return (N,2) array of valid beam endpoints in the map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF {self.map_frame} <- {scan.header.frame_id} unavailable: {e}",
                throttle_duration_sec=2.0,
            )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        m = quat_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        n = ranges.shape[0]
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
        )
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size == 0:
            return np.empty((0, 2))

        # Points in the scan (laser) frame, z=0.
        pts = np.ones((ranges.size, 4))
        pts[:, 0] = ranges * np.cos(angles)
        pts[:, 1] = ranges * np.sin(angles)
        pts[:, 2] = 0.0
        mapped = (m @ pts.T).T  # (N,4)
        return mapped[:, :2]

    def update(self):
        scan = self.latest_scan
        if scan is None:
            return
        pts = self._scan_points_in_map(scan)
        if pts is None:
            return

        d = self.inset
        if pts.shape[0] > 0:
            px, py = pts[:, 0], pts[:, 1]
        for name, s in self.slots.items():
            if pts.shape[0] == 0:
                count = 0
            else:
                # Inset the two side lines (avoid a neighbouring car's points)
                # and the back wall, but NOT the entry (y_min, -Y): the entry is
                # open, and the parked car's front face sits just inside it — the
                # main surface the lidar sees — so insetting it would drop the
                # detection. A free bay yields ~0 points here because beams pass
                # through and stop on the back wall (outside y_max).
                inside = (
                    (px >= s["x_min"] + d)
                    & (px <= s["x_max"] - d)
                    & (py >= s["y_min"])
                    & (py <= s["y_max"] - d)
                )
                count = int(np.count_nonzero(inside))
            state = "occupied" if count >= self.point_threshold else "free"
            self.occupancy[name] = (state, count)

        self._publish_status()
        self._publish_markers()

    def _publish_status(self):
        parts = [f"{name}={state}" for name, (state, _) in sorted(self.occupancy.items())]
        self.status_pub.publish(String(data=",".join(parts)))
        table = ", ".join(
            f"{name}:{state}({c})" for name, (state, c) in sorted(self.occupancy.items())
        )
        self.get_logger().info(table, throttle_duration_sec=2.0)

    def _publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, s) in enumerate(sorted(self.slots.items())):
            state, count = self.occupancy[name]
            cx, cy = s["center"]
            occupied = state == "occupied"

            box = Marker()
            box.header.frame_id = self.map_frame
            box.header.stamp = now
            box.ns = "slots"
            box.id = i
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = cx
            box.pose.position.y = cy
            box.pose.position.z = 0.02
            box.pose.orientation.w = 1.0
            box.scale.x = s["x_max"] - s["x_min"]
            box.scale.y = s["y_max"] - s["y_min"]
            box.scale.z = 0.02
            box.color.a = 0.35
            box.color.r = 0.9 if occupied else 0.1
            box.color.g = 0.1 if occupied else 0.9
            box.color.b = 0.1
            arr.markers.append(box)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = now
            label.ns = "slot_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.a = 1.0
            label.color.r = label.color.g = label.color.b = 1.0
            label.text = f"{name}: {state} ({count})"
            arr.markers.append(label)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SlotOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down slot occupancy node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
