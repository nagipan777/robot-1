#!/usr/bin/env python3
import json
import math
import signal
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_point(homography, x, y):
    denominator = homography[6] * x + homography[7] * y + homography[8]
    if abs(denominator) < 1e-9:
        raise ValueError("homography denominator is zero")
    return (
        (homography[0] * x + homography[1] * y + homography[2]) / denominator,
        (homography[3] * x + homography[4] * y + homography[5]) / denominator,
    )


def is_identity_homography(homography):
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    return len(homography) == 9 and all(
        abs(float(actual) - expected) <= 1e-9
        for actual, expected in zip(homography, identity)
    )


class ShutdownRequested(Exception):
    pass


class VisionTurnController(Node):
    def __init__(self):
        super().__init__("vision_turn_controller")
        self.declare_parameter("control_enabled", False)
        self.declare_parameter("calibration_ready", False)
        self.declare_parameter("pose_source", "odom")
        self.declare_parameter("pose_topic", "/odom")
        self.declare_parameter("detections_topic", "/webcam/detections")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("target_label", "litter")
        self.declare_parameter("detection_target_frame", "odom")
        self.declare_parameter("detection_target_topic", "/litter/target_odom")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("target_topic", "/litter/target")
        self.declare_parameter("target_marker_topic", "/litter/target_marker")
        self.declare_parameter("reached_topic", "/litter/reached")
        self.declare_parameter("image_to_odom_homography", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("calibration_image_width", 0)
        self.declare_parameter("calibration_image_height", 0)
        self.declare_parameter("angular_gain", 1.2)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("angle_tolerance", 0.08)
        self.declare_parameter("arrival_distance", 0.15)
        self.declare_parameter("input_timeout", 1.0)
        self.declare_parameter("control_mode", "turn_only")
        self.declare_parameter("linear_gain", 0.4)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("forward_angle_tolerance", 0.15)

        self.robot_pose = None
        self.pose_received_at = 0.0
        self.target_pixel = None
        self.target_odom = None
        self.control_target = None
        self.target_received_at = 0.0
        self.detection_received_at = 0.0
        self.last_target_log_at = 0.0
        self.marker_visible = False
        self.warned_not_ready = False
        self.output_active = False

        detections_topic = self.get_parameter("detections_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        pose_source = self.get_parameter("pose_source").value
        pose_topic = self.get_parameter("pose_topic").value

        self.create_subscription(String, detections_topic, self.on_detections, 10)
        self.create_subscription(
            PointStamped,
            self.get_parameter("target_topic").value,
            self.on_target,
            10,
        )
        if pose_source == "odom":
            self.create_subscription(Odometry, pose_topic, self.on_odom, 10)
        elif pose_source == "pose_stamped":
            self.create_subscription(PoseStamped, pose_topic, self.on_pose_stamped, 10)
        else:
            raise ValueError("pose_source must be 'odom' or 'pose_stamped'")
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.target_publisher = self.create_publisher(
            PointStamped, self.get_parameter("detection_target_topic").value, 10
        )
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_publisher = self.create_publisher(
            Marker, self.get_parameter("target_marker_topic").value, marker_qos
        )
        self.reached_publisher = self.create_publisher(
            Bool, self.get_parameter("reached_topic").value, 10
        )
        self.create_timer(0.1, self.update_control)
        self.get_logger().info(
            f"pose_source={pose_source}:{pose_topic}, detections={detections_topic}, output={cmd_vel_topic}"
        )

    def store_pose(self, pose):
        self.robot_pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.pose_received_at = time.monotonic()

    def on_odom(self, message):
        self.store_pose(message.pose.pose)

    def on_pose_stamped(self, message):
        expected_frame = self.get_parameter("target_frame").value
        if message.header.frame_id != expected_frame:
            self.robot_pose = None
            self.pose_received_at = 0.0
            self.stop_if_active()
            self.get_logger().warning(
                f"Ignoring pose in frame {message.header.frame_id!r}; "
                f"expected {expected_frame!r}"
            )
            return
        self.store_pose(message.pose)

    def on_target(self, message):
        expected_frame = self.get_parameter("target_frame").value
        if message.header.frame_id != expected_frame:
            self.control_target = None
            self.target_received_at = 0.0
            self.stop_if_active()
            self.get_logger().warning(
                f"Ignoring target in frame {message.header.frame_id!r}; "
                f"expected {expected_frame!r}"
            )
            return
        if not math.isfinite(message.point.x) or not math.isfinite(message.point.y):
            self.control_target = None
            self.target_received_at = 0.0
            self.stop_if_active()
            self.get_logger().warning("Ignoring non-finite target coordinates")
            return
        self.control_target = (message.point.x, message.point.y)
        self.target_received_at = time.monotonic()
        self.publish_marker(message)

    def on_detections(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            self.detection_received_at = time.monotonic()
            expected_width = self.get_parameter("calibration_image_width").value
            expected_height = self.get_parameter("calibration_image_height").value
            if expected_width > 0 and expected_height > 0 and (
                int(payload.get("image_width", -1)) != expected_width
                or int(payload.get("image_height", -1)) != expected_height
            ):
                raise ValueError(
                    f"image size differs from calibration: expected "
                    f"{expected_width}x{expected_height}, got "
                    f"{payload.get('image_width')}x{payload.get('image_height')}"
                )
            target_label = self.get_parameter("target_label").value.lower()
            litter = [
                item for item in detections
                if str(item.get("label", "")).lower() == target_label
            ]
            best = max(litter, key=lambda item: float(item.get("confidence", 0.0)), default=None)
            if best is None:
                self.clear_target()
                self.stop_if_active()
                return
            self.target_pixel = (float(best["center_x"]), float(best["center_y"]))
            homography = self.get_parameter("image_to_odom_homography").value
            if len(homography) != 9:
                raise ValueError("image_to_odom_homography must contain 9 values")
            self.target_odom = transform_point(homography, *self.target_pixel)
            self.publish_target(float(best.get("confidence", 0.0)))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.clear_target()
            self.stop_if_active()
            self.get_logger().warning(f"Invalid detection JSON: {exc}")

    def publish_target(self, confidence):
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("detection_target_frame").value
        target_x, target_y = self.target_odom

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame
        point.point.x = target_x
        point.point.y = target_y
        self.target_publisher.publish(point)

        now = time.monotonic()
        if now - self.last_target_log_at >= 1.0:
            pixel_x, pixel_y = self.target_pixel
            self.get_logger().info(
                f"litter pixel=({pixel_x:.1f}, {pixel_y:.1f}) -> "
                f"{frame}=({target_x:.3f}, {target_y:.3f}), confidence={confidence:.3f}"
            )
            self.last_target_log_at = now

    def publish_marker(self, target):
        marker = Marker()
        marker.header = target.header
        marker.ns = "litter_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target.point.x
        marker.pose.position.y = target.point.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.35
        marker.color.a = 0.9
        self.marker_publisher.publish(marker)
        self.marker_visible = True

    def clear_target(self):
        self.target_pixel = None
        self.target_odom = None
        self.control_target = None
        self.target_received_at = 0.0
        if self.marker_visible:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.get_parameter("target_frame").value
            marker.ns = "litter_target"
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_publisher.publish(marker)
            self.marker_visible = False

    def publish_stop(self):
        self.publisher.publish(Twist())
        self.output_active = False

    def stop_if_active(self):
        if self.output_active:
            self.publish_stop()

    def update_control(self):
        now = time.monotonic()
        timeout = self.get_parameter("input_timeout").value
        if self.target_odom is not None and now - self.detection_received_at > timeout:
            self.clear_target()
        inputs_fresh = (
            self.robot_pose is not None
            and self.control_target is not None
            and now - self.pose_received_at <= timeout
            and now - self.target_received_at <= timeout
        )
        if not inputs_fresh:
            self.stop_if_active()
            self.reached_publisher.publish(Bool(data=False))
            return

        robot_x, robot_y, robot_yaw = self.robot_pose
        target_x, target_y = self.control_target
        distance = math.hypot(target_x - robot_x, target_y - robot_y)
        reached = distance <= self.get_parameter("arrival_distance").value
        self.reached_publisher.publish(Bool(data=reached))

        enabled = self.get_parameter("control_enabled").value
        calibrated = self.get_parameter("calibration_ready").value
        homography = self.get_parameter("image_to_odom_homography").value
        if not enabled or not calibrated or is_identity_homography(homography):
            self.stop_if_active()
            if not self.warned_not_ready:
                self.get_logger().warning(
                    "Automatic turning is disabled. A non-identity calibration and both safety flags are required."
                )
                self.warned_not_ready = True
            return

        self.warned_not_ready = False
        if reached:
            self.publish_stop()
            return

        mode = self.get_parameter("control_mode").value
        if mode not in ("turn_only", "approach"):
            self.stop_if_active()
            self.get_logger().error(
                f"Unknown control_mode={mode!r}; expected 'turn_only' or 'approach'"
            )
            return

        target_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
        error = normalize_angle(target_yaw - robot_yaw)
        command = Twist()
        if abs(error) > self.get_parameter("angle_tolerance").value:
            gain = self.get_parameter("angular_gain").value
            limit = self.get_parameter("max_angular_speed").value
            command.angular.z = max(-limit, min(limit, gain * error))
        if (
            mode == "approach"
            and abs(error) <= self.get_parameter("forward_angle_tolerance").value
        ):
            gain = self.get_parameter("linear_gain").value
            limit = self.get_parameter("max_linear_speed").value
            remaining = distance - self.get_parameter("arrival_distance").value
            command.linear.x = max(0.0, min(limit, gain * remaining))
        self.publisher.publish(command)
        self.output_active = True


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisionTurnController()

    def shutdown_handler(_signum, _frame):
        raise ShutdownRequested

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, ShutdownRequested):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
