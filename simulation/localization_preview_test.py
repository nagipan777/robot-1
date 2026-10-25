#!/usr/bin/env python3
"""Single-process ROS integration test for mock wheel odometry and fusion."""

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from localization_fusion_node import LocalizationFusionNode
from wheel_odometry_node import WheelOdometryNode


class Harness(Node):
    def __init__(self):
        super().__init__("localization_preview_test")
        self.counts = self.create_publisher(
            String, "/localization/encoder_counts_mock", 10
        )
        self.aruco = self.create_publisher(
            PoseStamped, "/localization/aruco_pose", 10
        )
        self.create_subscription(
            Odometry, "/localization/wheel_odom", self.on_odom, 10
        )
        self.create_subscription(
            String, "/localization/status", self.on_status, 10
        )
        self.odom = None
        self.status = None

    def on_odom(self, message):
        self.odom = message

    def on_status(self, message):
        payload = json.loads(message.data)
        if payload.get("state") == "FUSED":
            self.status = payload

    def publish_counts(self, left, right):
        self.counts.publish(
            String(
                data=json.dumps(
                    {"left": left, "right": right, "timestamp": time.time()},
                    separators=(",", ":"),
                )
            )
        )

    def publish_aruco(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = 1.0
        message.pose.position.y = 2.0
        message.pose.orientation.w = 1.0
        self.aruco.publish(message)


def spin_until(executor, predicate, deadline):
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    wheel = WheelOdometryNode()
    fusion = LocalizationFusionNode()
    harness = Harness()
    executor = SingleThreadedExecutor()
    for node in (wheel, fusion, harness):
        executor.add_node(node)
    result = {"passed": False}
    try:
        deadline = time.monotonic() + 8.0
        for _ in range(5):
            harness.publish_counts(1000, 1000)
            if spin_until(executor, lambda: harness.odom is not None, time.monotonic() + 0.5):
                break
        if harness.odom is None:
            raise RuntimeError("wheel odometry was not received")
        harness.odom = None
        time.sleep(0.02)
        harness.publish_counts(1010, 1010)
        if not spin_until(executor, lambda: harness.odom is not None, deadline):
            raise RuntimeError("updated wheel odometry was not received")
        if harness.odom.header.frame_id != "odom" or harness.odom.child_frame_id != "base_link":
            raise RuntimeError("wheel odometry frame mismatch")
        for _ in range(5):
            harness.publish_aruco()
            if spin_until(executor, lambda: harness.status is not None, time.monotonic() + 0.5):
                break
        if harness.status is None:
            raise RuntimeError("FUSED status was not received")
        result = {
            "passed": True,
            "wheel_odom": {
                "frame_id": harness.odom.header.frame_id,
                "child_frame_id": harness.odom.child_frame_id,
                "x": harness.odom.pose.pose.position.x,
            },
            "fusion_status": harness.status,
            "pose_output_enabled": False,
            "transport": "mock_only",
        }
        print(json.dumps(result, sort_keys=True))
        return 0
    except Exception as error:
        result["error"] = str(error)
        print(json.dumps(result, sort_keys=True))
        return 1
    finally:
        for node in (harness, fusion, wheel):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
