#!/usr/bin/env python3
"""Read-only timestamped recorder for the Create3 command/odometry path."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


HAZARD_NAMES = {
    0: "backup_limit",
    1: "bump",
    2: "cliff",
    3: "stall",
    4: "wheel_drop",
    5: "object_proximity",
}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Recorder(Node):
    def __init__(self, output):
        super().__init__("oscillation_diagnostics")
        self.output = output
        self.started_ns = time.monotonic_ns()
        self.counts = {}
        self.events = []
        self.last_odom = None
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_unstamped, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_stamped, 10)
        self.create_subscription(
            TwistStamped, "/diffdrive_controller/cmd_vel", self.on_motion_output, 10
        )
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.on_hazard,
            qos_profile_sensor_data,
        )

    def record(self, topic, **values):
        mono_ns = time.monotonic_ns()
        event = {
            "topic": topic,
            "monotonic_s": (mono_ns - self.started_ns) / 1e9,
            "wall_time_ns": time.time_ns(),
            "ros_receive_time_ns": self.get_clock().now().nanoseconds,
            **values,
        }
        self.output.write(json.dumps(event, separators=(",", ":")) + "\n")
        self.output.flush()
        self.events.append(event)
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def on_unstamped(self, message):
        self.record(
            "/cmd_vel_unstamped", linear_x=message.linear.x, angular_z=message.angular.z
        )

    def record_stamped(self, topic, message):
        self.record(
            topic,
            header_time_ns=stamp_ns(message.header.stamp),
            linear_x=message.twist.linear.x,
            angular_z=message.twist.angular.z,
        )

    def on_stamped(self, message):
        self.record_stamped("/cmd_vel", message)

    def on_motion_output(self, message):
        self.record_stamped("/diffdrive_controller/cmd_vel", message)

    def on_odom(self, message):
        pose = message.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        self.record(
            "/odom",
            header_time_ns=stamp_ns(message.header.stamp),
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            linear_x=message.twist.twist.linear.x,
            angular_z=message.twist.twist.angular.z,
        )

    def on_hazard(self, message):
        detections = [
            {"type": int(item.type), "name": HAZARD_NAMES.get(int(item.type), "unknown"),
             "frame_id": item.header.frame_id, "header_time_ns": stamp_ns(item.header.stamp)}
            for item in message.detections
        ]
        self.record(
            "/hazard_detection",
            header_time_ns=stamp_ns(message.header.stamp),
            detections=detections,
        )


def graph_snapshot(node, topics):
    snapshot = {}
    for topic in topics:
        snapshot[topic] = {
            "publishers": [
                {"node": info.node_name, "namespace": info.node_namespace,
                 "type": info.topic_type}
                for info in node.get_publishers_info_by_topic(topic)
            ],
            "subscriptions": [
                {"node": info.node_name, "namespace": info.node_namespace,
                 "type": info.topic_type}
                for info in node.get_subscriptions_info_by_topic(topic)
            ],
        }
    return snapshot


def sign_reversals(events, topic, threshold=0.005):
    reversals = []
    previous = None
    for event in events:
        if event["topic"] != topic:
            continue
        value = event.get("linear_x", 0.0)
        sign = 1 if value > threshold else -1 if value < -threshold else 0
        if sign and previous and sign != previous[0]:
            reversals.append(
                {"monotonic_s": event["monotonic_s"], "from": previous[1], "to": value}
            )
        if sign:
            previous = (sign, value)
    return reversals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--output", default="/tmp/oscillation_events.jsonl")
    parser.add_argument("--summary", default="/tmp/oscillation_summary.json")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    output_path = Path(args.output)
    with output_path.open("w") as output:
        node = Recorder(output)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

        topics = [
            "/cmd_vel_unstamped", "/cmd_vel", "/diffdrive_controller/cmd_vel",
            "/odom", "/hazard_detection",
        ]
        hazards = [event for event in node.events
                   if event["topic"] == "/hazard_detection" and event["detections"]]
        summary = {
            "duration_s": args.duration,
            "counts": node.counts,
            "graph": graph_snapshot(node, topics),
            "command_reversals": sign_reversals(node.events, "/cmd_vel_unstamped"),
            "stamped_command_reversals": sign_reversals(node.events, "/cmd_vel"),
            "motion_output_reversals": sign_reversals(
                node.events, "/diffdrive_controller/cmd_vel"
            ),
            "odom_velocity_reversals": sign_reversals(node.events, "/odom"),
            "nonempty_hazards": hazards,
        }
        Path(args.summary).write_text(json.dumps(summary, indent=2) + "\n")
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
