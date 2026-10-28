#!/usr/bin/env python3
"""Deterministic nonphysical gate test for the live-camera controller path."""

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class GateProbe(Node):
    def __init__(self):
        super().__init__("ai_dry_run_gate_test")
        self.pose_publisher = self.create_publisher(PoseStamped, "/robot/pose_candidate", 10)
        self.detection_publisher = self.create_publisher(String, "/webcam/detections", 10)
        self.nav = []
        self.dry = []
        self.create_subscription(Twist, "/nav/cmd_vel_unstamped", self.on_nav, 10)
        self.create_subscription(Twist, "/roomba/cmd_vel_dry_run", self.on_dry, 10)

    @staticmethod
    def values(message):
        return {
            "at": time.monotonic(),
            "linear_x": float(message.linear.x),
            "angular_z": float(message.angular.z),
        }

    def on_nav(self, message):
        self.nav.append(self.values(message))

    def on_dry(self, message):
        self.dry.append(self.values(message))

    def publish_pose(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.orientation.w = 1.0
        self.pose_publisher.publish(message)

    def publish_detection(self):
        now = time.time()
        message = String()
        message.data = json.dumps(
            {
                "timestamp": now,
                "capture_timestamp": now,
                "processing_completed_timestamp": now,
                "processing_latency_s": 0.0,
                "image_width": 800,
                "image_height": 600,
                "detections": [
                    {
                        "label": "litter",
                        "confidence": 0.90,
                        "center_x": 708.0,
                        "center_y": 213.0,
                        "width": 80.0,
                        "height": 80.0,
                    }
                ],
                "aruco_detections": [],
            },
            separators=(",", ":"),
        )
        self.detection_publisher.publish(message)


def spin_for(executor, node, duration, publish_detection=False):
    deadline = time.monotonic() + duration
    next_publish = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            node.publish_pose()
            if publish_detection:
                node.publish_detection()
            next_publish = now + 0.05
        executor.spin_once(timeout_sec=0.01)


def nonzero(messages):
    return [
        item
        for item in messages
        if abs(item["linear_x"]) > 1e-9 or abs(item["angular_z"]) > 1e-9
    ]


def zero_after(messages, timestamp):
    return [
        item
        for item in messages
        if item["at"] >= timestamp
        and abs(item["linear_x"]) <= 1e-9
        and abs(item["angular_z"]) <= 1e-9
    ]


def main():
    rclpy.init()
    node = GateProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # Allow DDS endpoint discovery before publishing evidence messages.
        spin_for(executor, node, 2.0, publish_detection=False)
        spin_for(executor, node, 2.0, publish_detection=True)
        initial_nav_nonzero = nonzero(node.nav)
        initial_dry_nonzero = nonzero(node.dry)
        if not initial_nav_nonzero or not initial_dry_nonzero:
            raise RuntimeError("stable detection did not produce dry-run nonzero candidates")

        detection_stopped_at = time.monotonic()
        spin_for(executor, node, 1.4, publish_detection=False)
        nav_stops = zero_after(node.nav, detection_stopped_at)
        dry_stops = zero_after(node.dry, detection_stopped_at)
        if not nav_stops or not dry_stops:
            raise RuntimeError("input timeout did not produce STOP on both dry-run stages")

        before_reconfirm_nav = len(node.nav)
        before_reconfirm_dry = len(node.dry)
        spin_for(executor, node, 0.8, publish_detection=True)
        reconfirm_nav = node.nav[before_reconfirm_nav:]
        reconfirm_dry = node.dry[before_reconfirm_dry:]
        if nonzero(reconfirm_nav) or nonzero(reconfirm_dry):
            raise RuntimeError("same-target reconfirmation restarted without authorization")

        result = {
            "pass": True,
            "evidence_class": "synthetic_ros_nonphysical",
            "physical_output": False,
            "initial_nav_nonzero_count": len(initial_nav_nonzero),
            "initial_dry_nonzero_count": len(initial_dry_nonzero),
            "nav_stop_latency_s": nav_stops[0]["at"] - detection_stopped_at,
            "dry_stop_latency_s": dry_stops[0]["at"] - detection_stopped_at,
            "same_target_reconfirm_nav_nonzero_count": len(nonzero(reconfirm_nav)),
            "same_target_reconfirm_dry_nonzero_count": len(nonzero(reconfirm_dry)),
        }
        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
