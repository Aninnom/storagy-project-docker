#!/usr/bin/env python3

import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_numbers(text, expected_count=None):
    values = [float(value) for value in text.split()]
    if expected_count is not None and len(values) < expected_count:
        raise ValueError(f'Expected {expected_count} numbers, got {len(values)}: {text}')
    return values


def child_text(element, path, default=None):
    child = element.find(path)
    if child is None or child.text is None:
        return default
    return child.text.strip()


class AutoParking(Node):
    """Patrol parking slots and park in the first slot that sensors report empty."""

    def __init__(self):
        super().__init__('auto_parking')

        self.declare_parameter('parking_slot', 0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('world_file', '')
        self.declare_parameter('parking_lines_model', 'parking_lines')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('slot_scan_offset', 0.28)
        self.declare_parameter('approach_offset', 0.18)
        self.declare_parameter('forward_parking_speed', 0.035)
        self.declare_parameter('forward_parking_timeout_sec', 18.0)
        self.declare_parameter('forward_parking_goal_tolerance', 0.035)
        self.declare_parameter('forward_parking_yaw_gain', 0.8)
        self.declare_parameter('robot_length', 0.40)
        self.declare_parameter('front_clearance', 0.01)
        self.declare_parameter('occupied_detection_radius', 0.35)
        self.declare_parameter('detection_duration_sec', 1.2)
        self.declare_parameter('occupancy_min_points', 4)
        self.declare_parameter('camera_min_score', 0.08)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('world_occupancy_fallback', True)
        self.declare_parameter('slot_width_margin', 0.04)
        self.declare_parameter('slot_depth_margin', 0.05)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('initial_pose_delay_sec', 2.0)
        self.declare_parameter('goal_pause_sec', 0.3)

        self.frame_id = self.get_parameter('frame_id').value
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.scan_count = 0
        self.latest_image = None
        self.image_count = 0

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.slots_pub = self.create_publisher(PoseArray, 'parking_slots', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10,
        )

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_count += 1

    def image_callback(self, msg):
        self.latest_image = msg
        self.image_count += 1

    def default_world_file(self):
        pkg_storagy = get_package_share_directory('storagy')
        return os.path.join(pkg_storagy, 'worlds', 'parkinglot.sdf')

    def load_world_root(self):
        world_file = self.get_parameter('world_file').value
        if not world_file:
            world_file = self.default_world_file()

        self.get_logger().info(f'Reading parking layout from: {world_file}')
        return ET.parse(world_file).getroot()

    def model_name(self, model):
        return model.attrib.get('name', '')

    def pose_xy(self, element):
        pose_text = child_text(element, 'pose', '0 0 0 0 0 0')
        pose = parse_numbers(pose_text, 2)
        return pose[0], pose[1]

    def box_size_xy(self, element):
        size_text = child_text(element, 'geometry/box/size')
        if size_text is None:
            return None
        size = parse_numbers(size_text, 2)
        return size[0], size[1]

    def find_parking_slots(self, root):
        lines_model_name = self.get_parameter('parking_lines_model').value
        lines_model = None
        for model in root.findall('.//model'):
            if self.model_name(model) == lines_model_name:
                lines_model = model
                break

        if lines_model is None:
            raise RuntimeError(f'Cannot find model "{lines_model_name}" in SDF.')

        vertical_lines = []
        horizontal_lines = []
        for visual in lines_model.findall('.//visual'):
            size = self.box_size_xy(visual)
            if size is None:
                continue
            x, y = self.pose_xy(visual)
            sx, sy = size
            line = {'x': x, 'y': y, 'sx': sx, 'sy': sy}
            if sy > sx:
                vertical_lines.append(line)
            else:
                horizontal_lines.append(line)

        vertical_lines.sort(key=lambda line: line['x'])
        if len(vertical_lines) < 2:
            raise RuntimeError('Need at least 2 parking divider lines to infer slots.')

        divider_y = sum(line['y'] for line in vertical_lines) / len(vertical_lines)
        slot_depth = max(line['sy'] for line in vertical_lines)
        wall_line = max(
            horizontal_lines,
            key=lambda line: abs(line['y'] - divider_y),
            default={'y': divider_y + slot_depth * 0.5},
        )

        yaw = math.pi / 2.0 if wall_line['y'] >= divider_y else -math.pi / 2.0
        slots = []
        for index in range(len(vertical_lines) - 1):
            left = vertical_lines[index]
            right = vertical_lines[index + 1]
            center_x = (left['x'] + right['x']) * 0.5
            center_y = divider_y
            width = right['x'] - left['x']
            slots.append({
                'number': index + 1,
                'x': center_x,
                'y': center_y,
                'yaw': yaw,
                'width': width,
                'depth': slot_depth,
                'occupied': False,
                'lidar_points': 0,
                'camera_score': 0.0,
            })

        return slots

    def apply_world_occupancy_fallback(self, root, slots):
        if not self.get_parameter('world_occupancy_fallback').value:
            return slots

        parked_points = []
        for model in root.findall('.//model'):
            name = self.model_name(model)
            if not name.startswith('parked_robot_'):
                continue
            x, y = self.pose_xy(model)
            parked_points.append((name, x, y))

        for name, x, y in parked_points:
            best_slot = min(
                slots,
                key=lambda slot: math.hypot(slot['x'] - x, slot['y'] - y),
            )
            if math.hypot(best_slot['x'] - x, best_slot['y'] - y) <= 0.30:
                best_slot['world_occupied'] = True
                self.get_logger().info(
                    f"World fallback: slot {best_slot['number']} occupied by {name}.")

        return slots

    def quaternion_yaw(self, rotation):
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                'base_footprint',
                Time(),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    'base_link',
                    Time(),
                )
            except TransformException as error:
                self.get_logger().warn(f'Cannot get robot pose in {self.frame_id}: {error}')
                return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.quaternion_yaw(transform.transform.rotation),
        )

    def scan_to_map_points(self, scan):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                scan.header.frame_id,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warn(f'Cannot transform scan to {self.frame_id}: {error}')
            return []

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        yaw = self.quaternion_yaw(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                laser_x = distance * math.cos(angle)
                laser_y = distance * math.sin(angle)
                map_x = tx + cos_yaw * laser_x - sin_yaw * laser_y
                map_y = ty + sin_yaw * laser_x + cos_yaw * laser_y
                points.append((map_x, map_y))
            angle += scan.angle_increment

        return points

    def collect_lidar_points(self):
        duration = float(self.get_parameter('detection_duration_sec').value)
        deadline = time.monotonic() + duration
        start_scan_count = self.scan_count
        processed_scan_count = self.scan_count
        points = []

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.08)
            if self.latest_scan is None or self.scan_count == processed_scan_count:
                continue
            processed_scan_count = self.scan_count
            points.extend(self.scan_to_map_points(self.latest_scan))

        if self.scan_count == start_scan_count:
            raise RuntimeError('No LaserScan messages received. Check /scan bridge and lidar.')

        return points

    def latest_camera_score(self):
        if not self.get_parameter('camera_enabled').value:
            return 0.0
        if self.latest_image is None:
            self.get_logger().warn('No camera image received. Continuing with lidar only.')
            return 0.0

        image = self.latest_image
        encoding = image.encoding.lower()
        if encoding in ('rgb8', 'bgr8'):
            channels = 3
            rgb_order = encoding == 'rgb8'
        elif encoding in ('rgba8', 'bgra8'):
            channels = 4
            rgb_order = encoding == 'rgba8'
        else:
            self.get_logger().warn(f'Unsupported camera encoding: {image.encoding}')
            return 0.0

        width = image.width
        height = image.height
        data = image.data
        step = image.step
        x0 = int(width * 0.30)
        x1 = int(width * 0.70)
        y0 = int(height * 0.25)
        y1 = int(height * 0.85)

        samples = 0
        visual_hits = 0
        stride = 4
        for y in range(y0, y1, stride):
            row = y * step
            for x in range(x0, x1, stride):
                idx = row + x * channels
                if idx + 2 >= len(data):
                    continue
                if rgb_order:
                    r, g, b = data[idx], data[idx + 1], data[idx + 2]
                else:
                    b, g, r = data[idx], data[idx + 1], data[idx + 2]

                high = max(r, g, b)
                low = min(r, g, b)
                mean = (r + g + b) / 3.0
                saturation = high - low
                samples += 1
                if saturation > 35 or mean < 70 or mean > 205:
                    visual_hits += 1

        if samples == 0:
            return 0.0
        return visual_hits / samples

    def point_inside_slot(self, point, slot):
        dx = point[0] - slot['x']
        dy = point[1] - slot['y']
        yaw = slot['yaw']

        depth_axis = dx * math.cos(yaw) + dy * math.sin(yaw)
        width_axis = -dx * math.sin(yaw) + dy * math.cos(yaw)

        half_width = max(0.0, abs(slot['width']) * 0.5 -
                         float(self.get_parameter('slot_width_margin').value))
        half_depth = max(0.0, slot['depth'] * 0.5 -
                         float(self.get_parameter('slot_depth_margin').value))
        return abs(width_axis) <= half_width and abs(depth_axis) <= half_depth

    def inspect_slot(self, slot):
        if slot.get('world_occupied', False):
            slot['occupied'] = True
            self.get_logger().info(
                f"Slot {slot['number']} is occupied by world fallback.")
            return True

        self.get_logger().info(f"Inspecting slot {slot['number']} with lidar + camera...")
        lidar_points = self.collect_lidar_points()
        lidar_count = sum(1 for point in lidar_points if self.point_inside_slot(point, slot))
        near_count = self.nearby_slot_obstacle_count(lidar_points, slot)
        camera_score = self.latest_camera_score()

        lidar_occupied = lidar_count >= int(self.get_parameter('occupancy_min_points').value)
        near_occupied = near_count >= int(self.get_parameter('occupancy_min_points').value)
        camera_occupied = (
            camera_score >= float(self.get_parameter('camera_min_score').value) and
            (lidar_count + near_count) > 0
        )
        occupied = lidar_occupied or near_occupied or camera_occupied

        slot['lidar_points'] = lidar_count
        slot['near_points'] = near_count
        slot['camera_score'] = camera_score
        slot['occupied'] = occupied

        state = 'occupied' if occupied else 'empty'
        self.get_logger().info(
            f"Slot {slot['number']} is {state}: "
            f"lidar_points={lidar_count}, near_points={near_count}, "
            f"camera_score={camera_score:.3f}")
        return occupied

    def nearby_slot_obstacle_count(self, points, slot):
        pose = self.robot_pose()
        if pose is None:
            return 0

        robot_x, robot_y, _ = pose
        radius = float(self.get_parameter('occupied_detection_radius').value)
        count = 0
        for point in points:
            if math.hypot(point[0] - robot_x, point[1] - robot_y) > radius:
                continue
            if self.point_inside_slot(point, slot):
                count += 1
        return count

    def discover_parking_slots(self):
        root = self.load_world_root()
        slots = self.find_parking_slots(root)
        slots = self.apply_world_occupancy_fallback(root, slots)
        for slot in slots:
            state = 'occupied' if slot.get('world_occupied', False) else 'candidate'
            self.get_logger().info(
                f"Slot {slot['number']} {state}: "
                f"x={slot['x']:.3f}, y={slot['y']:.3f}, yaw={slot['yaw']:.3f}")
        return slots

    def make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        quat = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']
        return pose

    def scan_pose_for_slot(self, slot, offset=None):
        if offset is None:
            offset = float(self.get_parameter('slot_scan_offset').value)
        distance = slot['depth'] * 0.5 + offset
        return self.make_pose_stamped(
            slot['x'] - math.cos(slot['yaw']) * distance,
            slot['y'] - math.sin(slot['yaw']) * distance,
            slot['yaw'],
        )

    def publish_initial_pose(self):
        if not self.get_parameter('publish_initial_pose').value:
            return

        x = self.get_parameter('initial_x').value
        y = self.get_parameter('initial_y').value
        yaw = self.get_parameter('initial_yaw').value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose = self.make_pose_stamped(x, y, yaw).pose
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.1

        self.get_logger().info(
            f'Publishing initial pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        delay = self.get_parameter('initial_pose_delay_sec').value
        time.sleep(float(delay))

    def publish_slot_preview(self, slots):
        preview = PoseArray()
        preview.header.frame_id = self.frame_id
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.poses = [
            self.make_pose_stamped(slot['x'], slot['y'], slot['yaw']).pose
            for slot in slots
        ]
        self.slots_pub.publish(preview)

    def wait_for_nav2(self):
        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        if self.action_client.wait_for_server(timeout_sec=120.0):
            return True

        self.get_logger().error('Nav2 navigate_to_pose action server is not available.')
        return False

    def send_goal(self, pose, label):
        goal = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        self.get_logger().info(
            f'Navigating to {label}: x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}')

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {label}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded: {label}')
            time.sleep(float(self.get_parameter('goal_pause_sec').value))
            return True

        self.get_logger().error(f'Goal failed: {label}, status={result.status}')
        return False

    def choose_manual_slot(self, slots):
        requested_slot = int(self.get_parameter('parking_slot').value)
        if requested_slot <= 0:
            return None
        for slot in slots:
            if slot['number'] == requested_slot:
                return slot
        raise RuntimeError(f'Slot {requested_slot} does not exist.')

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def forward_park_in_slot(self, slot):
        approach = self.scan_pose_for_slot(
            slot,
            offset=float(self.get_parameter('approach_offset').value),
        )
        if not self.send_goal(approach, f"slot {slot['number']} approach"):
            return False

        center_to_target = max(
            0.0,
            slot['depth'] * 0.5 -
            float(self.get_parameter('robot_length').value) * 0.5 -
            float(self.get_parameter('front_clearance').value),
        )
        target_x = slot['x'] + math.cos(slot['yaw']) * center_to_target
        target_y = slot['y'] + math.sin(slot['yaw']) * center_to_target
        target_yaw = slot['yaw']
        speed = abs(float(self.get_parameter('forward_parking_speed').value))
        timeout = float(self.get_parameter('forward_parking_timeout_sec').value)
        tolerance = float(self.get_parameter('forward_parking_goal_tolerance').value)
        yaw_gain = float(self.get_parameter('forward_parking_yaw_gain').value)
        deadline = time.monotonic() + timeout

        self.get_logger().info(
            f"Driving forward into slot {slot['number']}: "
            f"target x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}"
        )

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.robot_pose()
            if pose is None:
                continue

            x, y, yaw = pose
            distance = math.hypot(target_x - x, target_y - y)
            yaw_error = normalize_angle(target_yaw - yaw)
            if distance <= tolerance:
                self.stop_robot()
                self.get_logger().info(f"Forward parking reached slot {slot['number']}.")
                return True

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = max(-0.22, min(0.22, yaw_gain * yaw_error))
            self.cmd_vel_pub.publish(cmd)

        self.stop_robot()
        self.get_logger().error(f"Forward parking timed out for slot {slot['number']}.")
        return False

    def sorted_patrol_slots(self, slots, manual_slot):
        if manual_slot is not None:
            return [manual_slot]

        pose = self.robot_pose()
        if pose is None:
            robot_x = float(self.get_parameter('initial_x').value)
            robot_y = float(self.get_parameter('initial_y').value)
        else:
            robot_x, robot_y, _ = pose

        return sorted(
            slots,
            key=lambda slot: math.hypot(
                self.scan_pose_for_slot(slot).pose.position.x - robot_x,
                self.scan_pose_for_slot(slot).pose.position.y - robot_y,
            ),
        )

    def run(self):
        try:
            slots = self.discover_parking_slots()
            manual_slot = self.choose_manual_slot(slots)
        except Exception as error:
            self.get_logger().error(str(error))
            return False

        self.publish_slot_preview(slots)
        if not self.wait_for_nav2():
            return False
        self.publish_initial_pose()

        patrol_slots = self.sorted_patrol_slots(slots, manual_slot)
        for slot in patrol_slots:
            if slot.get('world_occupied', False):
                self.get_logger().info(
                    f"Slot {slot['number']} occupied. Passing it before approach.")
                continue

            scan_pose = self.scan_pose_for_slot(slot)
            if not self.send_goal(scan_pose, f"slot {slot['number']} inspection pose"):
                return False

            try:
                occupied = self.inspect_slot(slot)
            except Exception as error:
                self.get_logger().error(str(error))
                return False

            if occupied:
                self.get_logger().info(f"Slot {slot['number']} occupied. Passing it.")
                continue

            self.get_logger().info(f"Slot {slot['number']} empty. Forward parking now.")
            if not self.forward_park_in_slot(slot):
                return False
            self.get_logger().info('Auto parking finished.')
            return True

        self.get_logger().error('No empty parking slot found.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoParking()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
