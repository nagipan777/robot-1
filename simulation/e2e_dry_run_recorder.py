#!/usr/bin/env python3
"""Record ROS evidence for one real-camera/Pico dry-run scenario."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


class EvidenceRecorder(Node):
    def __init__(self, scenario):
        super().__init__("e2e_dry_run_recorder")
        self.scenario = scenario
        self.events = []
        self.counts = {}
        self.nonzero_nav = 0
        self.nonzero_dry = 0
        self.stop_dry = 0
        self.create_subscription(String, "/webcam/detections", self.on_detection, 10)
        self.create_subscription(PointStamped, "/litter/target_candidate", self.on_candidate, 10)
        self.create_subscription(PointStamped, "/litter/target", self.on_approved, 10)
        self.create_subscription(String, "/litter/target_decision", self.on_decision, 10)
        self.create_subscription(PoseStamped, "/robot/pose_dry_run", self.on_pose, 10)
        self.create_subscription(
            PoseStamped, "/robot/pose_candidate", self.on_pose_candidate, 10
        )
        self.create_subscription(Twist, "/nav/cmd_vel_unstamped", self.on_nav, 10)
        self.create_subscription(Twist, "/roomba/cmd_vel_dry_run", self.on_dry, 10)
        self.create_subscription(String, "/pico/dry_run_state", self.on_pico_state, 10)

    def add(self, topic, data):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.events.append({"received_at": time.time(), "topic": topic, "data": data})

    def on_detection(self, message):
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"invalid_json": True}
        self.add("/webcam/detections", payload)

    def point(self, topic, message):
        self.add(topic, {"frame_id": message.header.frame_id, "x": message.point.x, "y": message.point.y})

    def on_candidate(self, message):
        self.point("/litter/target_candidate", message)

    def on_approved(self, message):
        self.point("/litter/target", message)

    def on_decision(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            data = {"raw": message.data}
        self.add("/litter/target_decision", data)

    def on_pose(self, message):
        self.pose("/robot/pose_dry_run", message)

    def on_pose_candidate(self, message):
        self.pose("/robot/pose_candidate", message)

    def pose(self, topic, message):
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.add(
            topic,
            {
                "frame_id": message.header.frame_id,
                "x": message.pose.position.x,
                "y": message.pose.position.y,
                "yaw": yaw,
            },
        )

    @staticmethod
    def twist_data(message):
        return {"linear_x": message.linear.x, "angular_z": message.angular.z}

    def on_nav(self, message):
        data = self.twist_data(message)
        if not math.isclose(data["linear_x"], 0.0) or not math.isclose(data["angular_z"], 0.0):
            self.nonzero_nav += 1
        self.add("/nav/cmd_vel_unstamped", data)

    def on_dry(self, message):
        data = self.twist_data(message)
        if math.isclose(data["linear_x"], 0.0) and math.isclose(data["angular_z"], 0.0):
            self.stop_dry += 1
        else:
            self.nonzero_dry += 1
        self.add("/roomba/cmd_vel_dry_run", data)

    def on_pico_state(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            data = {"raw": message.data}
        self.add("/pico/dry_run_state", data)

    def result(self):
        decisions = [e["data"].get("reason") for e in self.events if e["topic"] == "/litter/target_decision"]
        return {
            "scenario": self.scenario,
            "recorded_at": time.time(),
            "counts": self.counts,
            "decision_reasons": decisions,
            "nonzero_nav_commands": self.nonzero_nav,
            "nonzero_dry_run_commands": self.nonzero_dry,
            "dry_run_stop_commands": self.stop_dry,
            "events": self.events,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=list("ABCDE"))
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    rclpy.init()
    node = EvidenceRecorder(args.scenario)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(node.result(), ensure_ascii=False, indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
