#!/usr/bin/env python3
"""ROS 2 publish/subscribe E2E for observation-only odometry dry-run."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from localization_core import Pose2D, WheelOdometry, compose
from odometry_dryrun_core import DryRunOdometry, load_encoder_csv
from wheel_odometry_node import WheelOdometryNode


class Harness(Node):
    def __init__(self):
        super().__init__("odometry_ros_e2e_harness")
        self.encoder = self.create_publisher(String, "/roomba/encoder_counts", 10)
        self.aruco = self.create_publisher(PoseStamped, "/robot/pose", 10)
        self.messages = []
        self.create_subscription(Odometry, "/roomba/odom", self.messages.append, 10)

    def publish_sample(self, sample, map_pose):
        aruco = PoseStamped()
        seconds = int(sample.timestamp)
        aruco.header.stamp.sec = seconds
        aruco.header.stamp.nanosec = int((sample.timestamp - seconds) * 1e9)
        aruco.header.frame_id = "map"
        aruco.pose.position.x = map_pose.x
        aruco.pose.position.y = map_pose.y
        aruco.pose.orientation.z = math.sin(map_pose.yaw / 2.0)
        aruco.pose.orientation.w = math.cos(map_pose.yaw / 2.0)
        self.aruco.publish(aruco)
        payload = String()
        payload.data = json.dumps(
            {
                "timestamp": sample.timestamp,
                "left_encoder": sample.left_encoder,
                "right_encoder": sample.right_encoder,
                "valid": sample.valid,
                "communication_state": sample.communication_state,
                "sequence": sample.sequence,
                "error": sample.error,
            },
            separators=(",", ":"),
        )
        self.encoder.publish(payload)


def spin_until(executor, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def run(input_path, output_csv, output_json):
    samples = load_encoder_csv(input_path)
    expected = WheelOdometry(508.8, 0.072, 0.235, max_tick_delta=2000)
    expected_processor = DryRunOdometry(expected, max_time_gap_s=1.0)
    expected_poses = []
    expected_by_sequence = {}
    for sample in samples:
        try:
            processed = expected_processor.process(sample)
        except ValueError:
            expected.resynchronize()
            expected_processor.connected = False
            continue
        if processed.update is not None:
            expected_poses.append(processed.update.pose)
            expected_by_sequence[sample.sequence] = processed.update.pose
    rclpy.init()
    wheel = WheelOdometryNode(
        parameter_overrides=[
            Parameter("odometry_enabled", value=True),
            Parameter("csv_path", value=str(output_csv)),
        ]
    )
    harness = Harness()
    executor = SingleThreadedExecutor()
    executor.add_node(wheel)
    executor.add_node(harness)
    result = {"passed": False, "data_kind": "synthetic", "real_uart_used": False}
    try:
        # Allow DDS discovery before the first sample.
        spin_until(executor, lambda: False, 0.3)
        map_origin = Pose2D(1.179, 0.048, 0.12)
        expected_messages = 0
        local_pose = Pose2D()
        for sample in samples:
            if sample.sequence in expected_by_sequence:
                local_pose = expected_by_sequence[sample.sequence]
                expected_messages += 1
            harness.publish_sample(sample, compose(map_origin, local_pose))
            target = expected_messages
            spin_until(executor, lambda: len(harness.messages) >= target, 0.5)
        if not spin_until(executor, lambda: len(harness.messages) >= expected_messages, 2.0):
            raise RuntimeError(
                "expected {} odometry messages, received {}".format(
                    expected_messages, len(harness.messages)
                )
            )
        messages = harness.messages
        if any(message.header.frame_id != "roomba_odom" for message in messages):
            raise RuntimeError("unexpected odometry frame")
        if any(message.child_frame_id != "roomba_base_link" for message in messages):
            raise RuntimeError("unexpected child frame")
        stamps = [message.header.stamp.sec + message.header.stamp.nanosec / 1e9 for message in messages]
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            raise RuntimeError("odometry timestamps are not strictly increasing")
        if any(message.pose.covariance[0] <= 0 for message in messages):
            raise RuntimeError("pose covariance is not defined")
        final = messages[-1]
        expected_final = expected_poses[-1]
        if abs(final.pose.pose.position.x - expected_final.x) > 1e-9:
            raise RuntimeError("final x does not match core")
        csv_rows = list(csv.DictReader(Path(output_csv).open(encoding="utf-8")))
        if len(csv_rows) != len(samples):
            raise RuntimeError("CSV row count does not match input")
        result.update(
            {
                "passed": True,
                "input_samples": len(samples),
                "odom_messages": len(messages),
                "topic": "/roomba/odom",
                "message_type": "nav_msgs/msg/Odometry",
                "frame_id": "roomba_odom",
                "child_frame_id": "roomba_base_link",
                "final_x_m": final.pose.pose.position.x,
                "final_y_m": final.pose.pose.position.y,
                "final_yaw_rad": 2.0 * math.atan2(
                    final.pose.pose.orientation.z, final.pose.pose.orientation.w
                ),
                "tf_published": False,
                "navigation_connected": False,
            }
        )
    except Exception as error:
        result["error"] = str(error)
    finally:
        executor.remove_node(harness)
        executor.remove_node(wheel)
        harness.destroy_node()
        wheel.destroy_node()
        rclpy.shutdown()
    Path(output_json).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args()
    result = run(args.input, args.output_csv, args.output_json)
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
