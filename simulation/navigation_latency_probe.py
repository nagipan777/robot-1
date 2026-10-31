#!/usr/bin/env python3
"""Trace actual Navigation Twist creation into the stop-only bridge."""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node

from latency_trace import LatencyTrace
from roomba_latency_probe import MockPico, read_events


class InputPublisher(Node):
    def __init__(self):
        super().__init__("navigation_latency_input")
        self.pose = self.create_publisher(PoseStamped, "/latency_probe/pose", 1)
        self.target = self.create_publisher(PointStamped, "/latency_probe/target", 1)

    def wait_for_subscribers(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose.get_subscription_count() and self.target.get_subscription_count():
                return
        raise TimeoutError("Navigation input subscriptions were not discovered")

    def publish_inputs(self):
        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.orientation.w = 1.0
        target = PointStamped()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.point.x = 1.0
        self.pose.publish(pose)
        self.target.publish(target)


def terminate(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=3.0)


def summarize(path, label, samples):
    events = read_events(path)
    created = [
        event for event in events
        if event.get("stage") == "navigation_command_created"
        and event.get("command_kind") == "nonzero"
    ][:samples]
    published = [
        event for event in events
        if event.get("stage") == "navigation_publish_return"
        and event.get("command_kind") == "nonzero"
    ][:samples]
    received = [
        event for event in events
        if event.get("stage") == "bridge_receive"
        and (event.get("linear_x") or event.get("angular_z"))
    ][:samples]
    decisions = [
        event for event in events
        if event.get("stage") == "safety_decision"
        and event.get("reason") == "drive_enabled_false"
    ][:samples]
    counts = {
        "created": len(created),
        "published": len(published),
        "received": len(received),
        "rejected": len(decisions),
    }
    if any(count != samples for count in counts.values()):
        raise RuntimeError("trace count mismatch: {}".format(counts))
    rows = []
    for index, (create, publish, receive, decision) in enumerate(
        zip(created, published, received, decisions), start=1
    ):
        base = create["monotonic_ns"]
        rows.append(
            {
                "sample": index,
                "linear_x": create["linear_x"],
                "angular_z": create["angular_z"],
                "create_to_publish_return_ms": round(
                    (publish["monotonic_ns"] - base) / 1_000_000.0, 3
                ),
                "create_to_bridge_receive_ms": round(
                    (receive["monotonic_ns"] - base) / 1_000_000.0, 3
                ),
                "create_to_rejection_ms": round(
                    (decision["monotonic_ns"] - base) / 1_000_000.0, 3
                ),
                "bridge_receive_to_rejection_ms": round(
                    (decision["monotonic_ns"] - receive["monotonic_ns"])
                    / 1_000_000.0,
                    3,
                ),
                "wall_minus_monotonic_ms": round(
                    (create["wall_time_ns"] - create["monotonic_ns"])
                    / 1_000_000.0,
                    3,
                ),
            }
        )
    offsets = [row["wall_minus_monotonic_ms"] for row in rows]
    output = {
        "label": label,
        "counts": counts,
        "clock_offset_span_ms": round(max(offsets) - min(offsets), 6),
        "samples": rows,
    }
    summary_path = os.path.splitext(path)[0] + "-summary.json"
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return summary_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--bridge-script", default="roomba_bridge_tcp.py")
    parser.add_argument("--controller-script", default="vision_turn_controller.py")
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    try:
        os.unlink(args.output)
    except FileNotFoundError:
        pass
    trace = LatencyTrace(args.output, component="navigation_latency_probe")
    mock = MockPico(trace, 0.0)
    mock.start()
    base = os.path.splitext(args.output)[0]
    bridge_log = open(base + "-bridge.log", "w", encoding="utf-8")
    nav_log = open(base + "-navigation.log", "w", encoding="utf-8")
    bridge = subprocess.Popen(
        [
            sys.executable, args.bridge_script, "--ros-args",
            "-p", "host:=127.0.0.1", "-p", "port:={}".format(mock.port),
            "-p", "drive_enabled:=false",
            "-p", "command_topic:=/latency_probe/nav_cmd",
            "-p", "latency_trace_path:={}".format(args.output),
        ],
        stdout=bridge_log,
        stderr=subprocess.STDOUT,
    )
    navigation = subprocess.Popen(
        [
            sys.executable, args.controller_script, "--ros-args",
            "-p", "control_enabled:=false", "-p", "calibration_ready:=false",
            "-p", "dry_run_enabled:=true", "-p", "pose_source:=pose_stamped",
            "-p", "pose_topic:=/latency_probe/pose",
            "-p", "detection_target_topic:=/latency_probe/unused_target",
            "-p", "candidate_target_topic:=/latency_probe/target",
            "-p", "dry_run_cmd_vel_topic:=/latency_probe/nav_cmd",
            "-p", "require_detection_metadata:=false",
            "-p", "safe_target_management_enabled:=false",
            "-p", "control_mode:=approach", "-p", "input_timeout:=10.0",
            "-p", "max_robot_pose_age:=10.0",
            "-p", "image_to_odom_homography:=[2.0,0.0,0.0,0.0,2.0,0.0,0.0,0.0,1.0]",
            "-p", "latency_trace_path:={}".format(args.output),
        ],
        stdout=nav_log,
        stderr=subprocess.STDOUT,
    )
    rclpy.init()
    node = InputPublisher()
    try:
        node.wait_for_subscribers()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            node.publish_inputs()
            time.sleep(0.05)
            count = sum(
                event.get("stage") == "safety_decision"
                and event.get("reason") == "drive_enabled_false"
                for event in read_events(args.output)
            )
            if count >= args.samples:
                break
        else:
            raise TimeoutError("Navigation did not produce enough rejected commands")
        print(summarize(args.output, args.label, args.samples))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        terminate(navigation)
        terminate(bridge)
        bridge_log.close()
        nav_log.close()
        mock.close()
        trace.close()


if __name__ == "__main__":
    main()
