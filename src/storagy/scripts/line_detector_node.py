#!/usr/bin/env python3
"""
Camera Line Detector Node for Storagy parking demo (Phase 2).

Detects the black parking-tape lines on the ground from the RGB-D camera and
matches them to the slot dividers defined in param/parking_spaces.yaml.

Approach B (depth/RGB point cloud, NOT inverse-perspective mapping):

  1. Subscribe to /camera/depth/points, the rgbd_camera's XYZ+RGB cloud.
  2. Transform every point into the map frame (via TF).  The cloud's frame_id
     is camera_link (x forward, z up) -- see the <gz_frame_id> note in
     storagy.urdf; gz emits the points in that convention, not the optical one.
  3. Keep points that are (a) dark (mean RGB < dark_lum_max -> black tape, floor
     is ~0.8 grey) and (b) near the ground (map z in [ground_z_min, z_max], so
     walls / parked-car boxes are rejected) and (c) inside the parking region.
  4. The vertical dividers sit at known x positions (the slot bounds' x edges).
     For each expected divider, count tape points within +/- divider_tol in x
     and report DETECTED/missing plus the measured mean x (residual vs yaml).

The forward-facing camera (mounted horizontal at z=0.23 m) only sees the bays
when the robot is turned toward them; at the spawn pose they are outside the
field of view, so this node reports all dividers "missing" until the robot
faces the bay row.

Publishes:
  /parking/lines          std_msgs/String   "D-2.15=seen(123,x=-2.14),..."
  /parking/line_markers   visualization_msgs/MarkerArray  (RViz overlay:
                          green vertical segment per detected divider at the
                          measured x, plus the raw tape points as a cloud)

Run:
  python3 src/storagy/scripts/line_detector_node.py \
      --ros-args -p yaml_path:=<abs path to parking_spaces.yaml>
"""

import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros


def quat_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    """Build a 4x4 homogeneous transform from translation + quaternion."""
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


class LineDetectorNode(Node):
    def __init__(self):
        super().__init__("line_detector_node")

        default_yaml = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "param",
            "parking_spaces.yaml",
        )
        self.declare_parameter("yaml_path", default_yaml)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("points_topic", "/camera/depth/points")
        # Mean RGB (0-255) below which a point is "black tape". Floor renders
        # ~0.8 grey (~200); tape material is 0.02-0.03 (~very dark). 70 leaves
        # margin against soft shadows.
        self.declare_parameter("dark_lum_max", 70.0)
        # Ground band in map z [m]: tape sits at z~0.026; keep a small band and
        # reject anything tall (walls, parked-car boxes).
        self.declare_parameter("ground_z_min", -0.05)
        self.declare_parameter("ground_z_max", 0.12)
        # How close (in x) a tape point must be to an expected divider to count,
        # and how many such points make a divider "detected". Tape is 0.08 wide.
        self.declare_parameter("divider_tol", 0.06)
        self.declare_parameter("min_line_points", 15)
        self.declare_parameter("publish_period", 0.5)

        yaml_path = self.get_parameter("yaml_path").value
        self.map_frame = self.get_parameter("map_frame").value
        points_topic = self.get_parameter("points_topic").value
        self.dark_lum_max = float(self.get_parameter("dark_lum_max").value)
        self.ground_z_min = float(self.get_parameter("ground_z_min").value)
        self.ground_z_max = float(self.get_parameter("ground_z_max").value)
        self.divider_tol = float(self.get_parameter("divider_tol").value)
        self.min_line_points = int(self.get_parameter("min_line_points").value)
        period = float(self.get_parameter("publish_period").value)

        self.dividers, self.bay_y = self._load_dividers(yaml_path)
        self.get_logger().info(
            f"Expected {len(self.dividers)} dividers at x="
            f"{[round(x, 3) for x in self.dividers]}, bay y "
            f"{self.bay_y[0]:.3f}..{self.bay_y[1]:.3f}"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_cloud = None
        # name -> (detected: bool, count: int, mean_x: float|None)
        self.lines = {self._dname(x): (False, 0, None) for x in self.dividers}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, points_topic, self.cloud_callback, sensor_qos
        )
        self.status_pub = self.create_publisher(String, "/parking/lines", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/parking/line_markers", 10
        )

        self.timer = self.create_timer(period, self.update)
        self.get_logger().info("Line detector node ready.")

    @staticmethod
    def _dname(x):
        return f"D{x:+.2f}"

    def _load_dividers(self, path):
        """Vertical divider x positions (unique slot-bound edges) + bay y-span."""
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        xs = set()
        y_mins, y_maxs = [], []
        for info in data["parking_spaces"].values():
            b = info["bounds"]
            xs.add(round(float(b["x_min"]), 3))
            xs.add(round(float(b["x_max"]), 3))
            y_mins.append(float(b["y_min"]))
            y_maxs.append(float(b["y_max"]))
        return sorted(xs), (min(y_mins), max(y_maxs))

    def cloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    def _tape_points_in_map(self, cloud: PointCloud2):
        """Return (N,2) map-frame x,y of dark, near-ground points, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                cloud.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF {self.map_frame} <- {cloud.header.frame_id} unavailable: {e}",
                throttle_duration_sec=2.0,
            )
            return None

        raw = pc2.read_points_numpy(
            cloud, field_names=("x", "y", "z", "rgb"), reshape_organized_cloud=False
        )
        if raw.shape[0] == 0:
            return np.empty((0, 2))

        xyz = raw[:, :3]
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        rgb_f = raw[finite, 3].astype(np.float32)
        if xyz.shape[0] == 0:
            return np.empty((0, 2))

        # Decode packed float32 RGB -> mean luminance, mask the dark (tape) pts
        # before transforming, so we move far fewer points.
        rgb_i = rgb_f.view(np.uint32)
        r = (rgb_i >> 16) & 0xFF
        g = (rgb_i >> 8) & 0xFF
        b = rgb_i & 0xFF
        lum = (r.astype(np.float32) + g + b) / 3.0
        dark = lum < self.dark_lum_max
        if not np.any(dark):
            return np.empty((0, 2))

        pts = np.ones((int(np.count_nonzero(dark)), 4))
        pts[:, :3] = xyz[dark]

        t = tf.transform.translation
        q = tf.transform.rotation
        m = quat_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        mapped = (m @ pts.T).T  # (N,4)

        # Keep ground band + parking region (a little wider than the bays).
        mx, my, mz = mapped[:, 0], mapped[:, 1], mapped[:, 2]
        x_lo = self.dividers[0] - 0.20
        x_hi = self.dividers[-1] + 0.20
        y_lo = self.bay_y[0] - 0.15
        y_hi = self.bay_y[1] + 0.15
        keep = (
            (mz >= self.ground_z_min)
            & (mz <= self.ground_z_max)
            & (mx >= x_lo)
            & (mx <= x_hi)
            & (my >= y_lo)
            & (my <= y_hi)
        )
        return mapped[keep, :2]

    def update(self):
        cloud = self.latest_cloud
        if cloud is None:
            return
        tape = self._tape_points_in_map(cloud)
        if tape is None:
            return

        if tape.shape[0] > 0:
            tx, ty = tape[:, 0], tape[:, 1]
        for x_d in self.dividers:
            name = self._dname(x_d)
            if tape.shape[0] == 0:
                self.lines[name] = (False, 0, None)
                continue
            sel = np.abs(tx - x_d) <= self.divider_tol
            count = int(np.count_nonzero(sel))
            if count >= self.min_line_points:
                self.lines[name] = (True, count, float(np.mean(tx[sel])))
            else:
                self.lines[name] = (False, count, None)

        self._publish_status()
        self._publish_markers(tape)

    def _publish_status(self):
        parts = []
        for name, (det, count, mx) in self.lines.items():
            if det:
                parts.append(f"{name}=seen({count},x={mx:+.2f})")
            else:
                parts.append(f"{name}=miss({count})")
        self.status_pub.publish(String(data=",".join(parts)))
        self.get_logger().info(", ".join(parts), throttle_duration_sec=2.0)

    def _publish_markers(self, tape):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Raw tape points (yellow) for sanity.
        pts = Marker()
        pts.header.frame_id = self.map_frame
        pts.header.stamp = now
        pts.ns = "tape_points"
        pts.id = 0
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        pts.scale.x = pts.scale.y = 0.02
        pts.color = ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0)
        pts.pose.orientation.w = 1.0
        for x, y in tape:
            pts.points.append(Point(x=float(x), y=float(y), z=0.03))
        arr.markers.append(pts)

        # One vertical segment per divider at the measured x (green) or the
        # expected x (red, dim) when missing.
        for i, x_d in enumerate(self.dividers):
            name = self._dname(x_d)
            det, _count, mx = self.lines[name]
            x_draw = mx if (det and mx is not None) else x_d
            seg = Marker()
            seg.header.frame_id = self.map_frame
            seg.header.stamp = now
            seg.ns = "dividers"
            seg.id = i
            seg.type = Marker.LINE_STRIP
            seg.action = Marker.ADD
            seg.scale.x = 0.03
            if det:
                seg.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=1.0)
            else:
                seg.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.4)
            seg.pose.orientation.w = 1.0
            seg.points.append(Point(x=float(x_draw), y=float(self.bay_y[0]), z=0.04))
            seg.points.append(Point(x=float(x_draw), y=float(self.bay_y[1]), z=0.04))
            arr.markers.append(seg)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down line detector node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
