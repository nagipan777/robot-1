#!/usr/bin/env python3
"""Read-only ROS 2 topic summary for recording a PAI demonstration video."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class RecordingStatus(Node):
    """Subscribe to the proof-of-flow topics and print a compact summary."""

    def __init__(self):
        super().__init__("pai_recording_status")
        self.values = {
            "detections": "waiting",
            "target": "waiting",
            "pose": "waiting",
            "command": "waiting",
            "reached": "waiting",
        }
        self.updated_at = {name: None for name in self.values}
        self.create_subscription(
            String, "/webcam/detections", self.on_detections, 10
        )
        self.create_subscription(PointStamped, "/litter/target", self.on_target, 10)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)
        self.create_subscription(Bool, "/litter/reached", self.on_reached, 10)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            "Read-only monitor started; it does not publish control commands."
        )

    def store(self, name, value):
        self.values[name] = value
        self.updated_at[name] = time.monotonic()

    def on_detections(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get("detections", [])
            labels = [str(item.get("label", "?")) for item in detections]
            label_text = ",".join(labels[:3]) if labels else "none"
            self.store("detections", f"{len(detections)}[{label_text}]")
        except (TypeError, ValueError, json.JSONDecodeError):
            self.store("detections", "invalid-json")

    def on_target(self, message):
        self.store(
            "target",
            f"({message.point.x:.2f},{message.point.y:.2f}) "
            f"frame={message.header.frame_id}",
        )

    def on_pose(self, message):
        yaw = yaw_from_quaternion(message.pose.orientation)
        self.store(
            "pose",
            f"({message.pose.position.x:.2f},{message.pose.position.y:.2f},"
            f"{yaw:.2f}) frame={message.header.frame_id}",
        )

    def on_command(self, message):
        self.store(
            "command",
            f"linear={message.linear.x:.3f} angular={message.angular.z:.3f}",
        )

    def on_reached(self, message):
        self.store("reached", str(bool(message.data)).lower())

    def format_value(self, name, now):
        updated = self.updated_at[name]
        age = "--" if updated is None else f"{now - updated:.1f}s"
        return f"{name}={self.values[name]} age={age}"

    def report(self):
        now = time.monotonic()
        self.get_logger().info(
            "ROS2 FLOW | "
            + " | ".join(self.format_value(name, now) for name in self.values)
        )


def main():
    rclpy.init()
    node = RecordingStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
