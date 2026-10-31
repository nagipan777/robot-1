#!/usr/bin/env python3
"""Opt-in CSV replay publisher for the observation-only odometry node."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from odometry_dryrun_core import load_encoder_csv


class OdometryReplayNode(Node):
    def __init__(self):
        super().__init__("odometry_replay")
        self.declare_parameter("replay_enabled", False)
        self.declare_parameter("input_file", "")
        self.declare_parameter("output_topic", "/roomba/encoder_counts")
        self.declare_parameter("replay_rate", 1.0)
        if not self.get_parameter("replay_enabled").value:
            raise RuntimeError("odometry replay is locked; replay_enabled=false")
        input_file = self.get_parameter("input_file").value
        if not input_file:
            raise ValueError("input_file is required")
        self.rate = float(self.get_parameter("replay_rate").value)
        if self.rate < 0:
            raise ValueError("replay_rate must be zero or positive")
        self.samples = load_encoder_csv(input_file)
        self.publisher = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.index = 0
        self.wall_started = None
        self.source_started = self.samples[0].timestamp
        self.done = False
        self.create_timer(0.005 if self.rate == 0 else 0.01, self.tick)

    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.wall_started is None:
            self.wall_started = now
        sample = self.samples[self.index]
        if self.rate > 0:
            target = (sample.timestamp - self.source_started) / self.rate
            if now - self.wall_started < target:
                return
        message = String()
        message.data = json.dumps(
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
        self.publisher.publish(message)
        self.index += 1
        if self.index >= len(self.samples):
            self.done = True
            self.get_logger().info("Odometry replay complete: samples={}".format(self.index))


def main():
    rclpy.init()
    node = OdometryReplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
