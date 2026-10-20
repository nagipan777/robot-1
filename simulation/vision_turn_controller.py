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

from target_manager import (
    SafeTargetManager,
    evaluate_robot_guard,
    format_robot_proximity_log,
    require_fresh_timestamp,
    select_best_detection,
)


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
        self.declare_parameter("dry_run_enabled", False)
        self.declare_parameter("pose_source", "odom")
        self.declare_parameter("pose_topic", "/odom")
        self.declare_parameter("detections_topic", "/webcam/detections")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("dry_run_cmd_vel_topic", "/nav/cmd_vel_unstamped")
        self.declare_parameter("target_label", "litter")
        self.declare_parameter("detection_target_frame", "odom")
        self.declare_parameter("detection_target_topic", "/litter/target_candidate_odom")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("candidate_target_topic", "/litter/target_candidate")
        self.declare_parameter("target_topic", "/litter/target")
        self.declare_parameter("decision_topic", "/litter/target_decision")
        self.declare_parameter("require_detection_metadata", False)
        self.declare_parameter("target_marker_topic", "/litter/target_marker")
        self.declare_parameter("reached_topic", "/litter/reached")
        self.declare_parameter("image_to_odom_homography", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("calibration_image_width", 0)
        self.declare_parameter("calibration_image_height", 0)
        self.declare_parameter("angular_gain", 1.2)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("angle_tolerance", 0.08)
        self.declare_parameter("arrival_distance", 0.15)
        self.declare_parameter("arrival_epsilon", 0.0)
        self.declare_parameter("input_timeout", 1.0)
        self.declare_parameter("max_detection_age", 0.0)
        self.declare_parameter("safe_target_management_enabled", False)
        self.declare_parameter("target_change_distance", 0.0)
        self.declare_parameter("min_detection_confidence", 0.0)
        self.declare_parameter("control_mode", "turn_only")
        self.declare_parameter("linear_gain", 0.4)
        self.declare_parameter("max_linear_speed", 0.08)
        self.declare_parameter("forward_angle_tolerance", 0.15)
        self.declare_parameter("robot_guard_radius", 0.5)
        self.declare_parameter("max_robot_pose_age", 1.0)

        self.robot_pose = None
        self.pose_received_at = 0.0
        self.pose_stamp_seconds = None
        self.target_pixel = None
        self.target_odom = None
        self.control_target = None
        self.target_received_at = 0.0
        self.detection_received_at = 0.0
        self.last_target_log_at = 0.0
        self.marker_visible = False
        self.warned_not_ready = False
        self.output_active = False
        self.target_reached = False
        self.target_manager = SafeTargetManager()
        self.pending_detection_metadata = None

        detections_topic = self.get_parameter("detections_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        dry_run_enabled = bool(self.get_parameter("dry_run_enabled").value)
        if dry_run_enabled and self.get_parameter("control_enabled").value:
            raise ValueError("dry_run_enabled and control_enabled cannot both be true")
        pose_source = self.get_parameter("pose_source").value
        pose_topic = self.get_parameter("pose_topic").value

        self.create_subscription(String, detections_topic, self.on_detections, 10)
        self.create_subscription(
            PointStamped,
            self.get_parameter("candidate_target_topic").value,
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
        self.dry_run_publisher = self.create_publisher(
            Twist, self.get_parameter("dry_run_cmd_vel_topic").value, 10
        )
        self.target_publisher = self.create_publisher(
            PointStamped, self.get_parameter("detection_target_topic").value, 10
        )
        self.approved_target_publisher = self.create_publisher(
            PointStamped, self.get_parameter("target_topic").value, 10
        )
        self.decision_publisher = self.create_publisher(
            String, self.get_parameter("decision_topic").value, 10
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
            f"pose_source={pose_source}:{pose_topic}, detections={detections_topic}, "
            f"output={self.get_parameter('dry_run_cmd_vel_topic').value if dry_run_enabled else cmd_vel_topic}, "
            f"dry_run={dry_run_enabled}"
        )

    def publish_decision(self, accepted, reason, message=None, metadata=None, distance=None):
        record = {
            "accepted": bool(accepted),
            "reason": reason,
            "timestamp": time.time(),
        }
        if message is not None:
            record.update({
                "frame_id": message.header.frame_id,
                "x": message.point.x,
                "y": message.point.y,
            })
        if metadata is not None:
            record["confidence"] = metadata["confidence"]
            record["detection_timestamp"] = metadata["timestamp"]
        if distance is not None:
            record["robot_distance_m"] = distance
        self.decision_publisher.publish(String(data=json.dumps(record, sort_keys=True)))

    def store_pose(self, pose, stamp):
        self.robot_pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.pose_received_at = time.monotonic()
        self.pose_stamp_seconds = stamp.sec + stamp.nanosec / 1_000_000_000.0

    def on_odom(self, message):
        self.store_pose(message.pose.pose, message.header.stamp)

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
        self.store_pose(message.pose, message.header.stamp)

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
            self.publish_decision(False, "non_finite_target", message)
            return
        metadata = self.pending_detection_metadata
        stamp_key = (message.header.stamp.sec, message.header.stamp.nanosec)
        metadata_matches = metadata is not None and metadata["stamp_key"] == stamp_key
        if self.get_parameter("require_detection_metadata").value and not metadata_matches:
            self.lose_detection()
            self.get_logger().warning("Detection rejected: reason=missing_detection_metadata")
            self.publish_decision(False, "missing_detection_metadata", message)
            return
        if metadata_matches:
            max_pose_age = float(self.get_parameter("max_robot_pose_age").value)
            if max_pose_age <= 0.0 or not math.isfinite(max_pose_age):
                self.lose_detection()
                self.get_logger().error(
                    "Detection rejected: reason=invalid_max_robot_pose_age"
                )
                self.publish_decision(False, "invalid_max_robot_pose_age", message, metadata)
                return
            receive_age = time.monotonic() - self.pose_received_at
            if self.robot_pose is None:
                self.lose_detection()
                self.get_logger().warning(
                    "Detection rejected: reason=robot_pose_unavailable "
                    f"target_{expected_frame}=({message.point.x:.3f}, {message.point.y:.3f}) "
                    f"confidence={metadata['confidence']:.3f} "
                    f"timestamp={metadata['timestamp']:.6f}"
                )
                self.publish_decision(False, "robot_pose_unavailable", message, metadata)
                return
            ros_now = self.get_clock().now().nanoseconds / 1_000_000_000.0
            stamp_age = (
                math.inf
                if self.pose_stamp_seconds is None
                else ros_now - self.pose_stamp_seconds
            )
            pose_age = max(receive_age, stamp_age)
            if pose_age < -0.1 or pose_age > max_pose_age:
                self.lose_detection()
                self.get_logger().warning(
                    "Detection rejected: reason=robot_pose_stale "
                    f"pose_age={pose_age:.3f}s receive_age={receive_age:.3f}s "
                    f"stamp_age={stamp_age:.3f}s limit={max_pose_age:.3f}s "
                    f"target_{expected_frame}=({message.point.x:.3f}, {message.point.y:.3f}) "
                    f"confidence={metadata['confidence']:.3f} "
                    f"timestamp={metadata['timestamp']:.6f}"
                )
                self.publish_decision(False, "robot_pose_stale", message, metadata)
                return
            try:
                guard = evaluate_robot_guard(
                    (message.point.x, message.point.y),
                    self.robot_pose[:2],
                    self.get_parameter("robot_guard_radius").value,
                )
            except ValueError as exc:
                self.lose_detection()
                self.get_logger().error(f"Detection rejected: reason=invalid_guard {exc}")
                self.publish_decision(False, "invalid_guard", message, metadata)
                return
            if not guard["accepted"]:
                self.lose_detection()
                self.get_logger().warning(
                    format_robot_proximity_log(
                        expected_frame,
                        (message.point.x, message.point.y),
                        metadata["confidence"],
                        guard["distance"],
                        metadata["timestamp"],
                    )
                )
                self.publish_decision(
                    False, "robot_proximity", message, metadata, guard["distance"]
                )
                return
        new_target = (message.point.x, message.point.y)
        if self.get_parameter("safe_target_management_enabled").value:
            decision = self.target_manager.consider(
                new_target,
                float(self.get_parameter("target_change_distance").value),
            )
            if decision == "blocked_same":
                if (
                    self.target_manager.reached
                    and not self.target_manager.suspended
                    and self.control_target is not None
                ):
                    self.target_received_at = time.monotonic()
                    self.publish_marker(message)
                self.stop_if_active()
                return
            if decision == "new":
                self.target_reached = False
                self.control_target = new_target
            # Keep the first accepted coordinate for an active target so bbox
            # jitter cannot move the goal or release the arrival latch.
        else:
            if self.control_target != new_target:
                self.target_reached = False
            self.control_target = new_target
        self.target_received_at = time.monotonic()
        self.approved_target_publisher.publish(message)
        self.publish_decision(True, "accepted", message, metadata)
        self.publish_marker(message)

    def on_detections(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            max_detection_age = self.get_parameter("max_detection_age").value
            if max_detection_age > 0.0:
                require_fresh_timestamp(
                    payload["timestamp"], time.time(), max_detection_age
                )
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
            min_confidence = float(
                self.get_parameter("min_detection_confidence").value
            )
            best = select_best_detection(
                detections, target_label, min_confidence
            )
            if best is None:
                self.lose_detection()
                return
            homography = self.get_parameter("image_to_odom_homography").value
            calibration_allowed = (
                self.get_parameter("calibration_ready").value
                or self.get_parameter("dry_run_enabled").value
            )
            if (
                not calibration_allowed
                or len(homography) != 9
                or is_identity_homography(homography)
                or expected_width <= 0
                or expected_height <= 0
            ):
                self.lose_detection()
                if not self.warned_not_ready:
                    self.get_logger().warning(
                        "Floor calibration is not valid for this mode; refusing to publish a metric litter target"
                    )
                    self.warned_not_ready = True
                return
            self.target_pixel = (float(best["center_x"]), float(best["center_y"]))
            self.target_odom = transform_point(homography, *self.target_pixel)
            self.detection_received_at = time.monotonic()
            detection_timestamp = float(payload.get("timestamp", time.time()))
            if not math.isfinite(detection_timestamp):
                raise ValueError("detection timestamp must be finite")
            self.publish_target(
                float(best.get("confidence", 0.0)), detection_timestamp
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.lose_detection()
            self.get_logger().warning(f"Invalid detection JSON: {exc}")

    def publish_target(self, confidence, detection_timestamp):
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("detection_target_frame").value
        target_x, target_y = self.target_odom

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame
        point.point.x = target_x
        point.point.y = target_y
        self.pending_detection_metadata = {
            "stamp_key": (stamp.sec, stamp.nanosec),
            "confidence": confidence,
            "timestamp": detection_timestamp,
        }
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

    def clear_target(self, preserve_safe_state=False):
        self.target_pixel = None
        self.target_odom = None
        self.control_target = None
        self.target_reached = False
        self.target_received_at = 0.0
        self.pending_detection_metadata = None
        if not preserve_safe_state:
            self.target_manager.reset()
        if self.marker_visible:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = self.get_parameter("target_frame").value
            marker.ns = "litter_target"
            marker.id = 0
            marker.action = Marker.DELETE
            self.marker_publisher.publish(marker)
            self.marker_visible = False

    def lose_detection(self):
        safe_management = self.get_parameter(
            "safe_target_management_enabled"
        ).value
        control_enabled = self.get_parameter("control_enabled").value
        preserve_safe_state = safe_management and control_enabled
        if preserve_safe_state:
            self.target_manager.suspend()
        self.clear_target(preserve_safe_state=preserve_safe_state)
        self.stop_if_active()

    def publish_stop(self):
        self.active_publisher().publish(Twist())
        self.output_active = False

    def active_publisher(self):
        if self.get_parameter("dry_run_enabled").value:
            return self.dry_run_publisher
        return self.publisher

    def stop_if_active(self):
        if self.output_active:
            self.publish_stop()

    def update_control(self):
        now = time.monotonic()
        timeout = self.get_parameter("input_timeout").value
        if self.target_odom is not None and now - self.detection_received_at > timeout:
            self.lose_detection()
        inputs_fresh = (
            self.robot_pose is not None
            and self.control_target is not None
            and now - self.pose_received_at
            <= float(self.get_parameter("max_robot_pose_age").value)
            and now - self.target_received_at <= timeout
        )
        if not inputs_fresh:
            self.stop_if_active()
            self.reached_publisher.publish(Bool(data=False))
            return

        robot_x, robot_y, robot_yaw = self.robot_pose
        target_x, target_y = self.control_target
        distance = math.hypot(target_x - robot_x, target_y - robot_y)
        arrival_distance = self.get_parameter("arrival_distance").value
        arrival_epsilon = self.get_parameter("arrival_epsilon").value
        if arrival_epsilon < 0.0:
            self.stop_if_active()
            self.get_logger().error("arrival_epsilon must be non-negative")
            return
        reached_now = distance <= arrival_distance + arrival_epsilon
        if reached_now and not self.target_reached:
            self.target_reached = True
            if self.get_parameter("safe_target_management_enabled").value:
                self.target_manager.mark_reached()
            self.get_logger().info(
                f"Target reached: distance={distance:.6f} m, "
                f"threshold={arrival_distance + arrival_epsilon:.6f} m"
            )
        reached = self.target_reached
        self.reached_publisher.publish(Bool(data=reached))

        enabled = self.get_parameter("control_enabled").value
        dry_run_enabled = self.get_parameter("dry_run_enabled").value
        calibrated = self.get_parameter("calibration_ready").value
        homography = self.get_parameter("image_to_odom_homography").value
        output_allowed = dry_run_enabled or (enabled and calibrated)
        if (enabled and dry_run_enabled) or not output_allowed or is_identity_homography(homography):
            self.stop_if_active()
            if not self.warned_not_ready:
                self.get_logger().warning(
                    "Command calculation is disabled. Physical control requires both safety flags; dry-run requires dry_run_enabled."
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
            remaining = distance - arrival_distance
            command.linear.x = max(0.0, min(limit, gain * remaining))
        self.active_publisher().publish(command)
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
