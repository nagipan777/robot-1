#!/usr/bin/env python3
"""Preview-only mock encoder input to nav_msgs/Odometry."""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from localization_core import WheelOdometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        self.declare_parameter("input_topic", "/localization/encoder_counts_mock")
        self.declare_parameter("output_topic", "/localization/wheel_odom")
        self.declare_parameter("preview_enabled", False)
        self.declare_parameter("encoder_counts_per_rev", 508.8)
        self.declare_parameter("wheel_diameter_m", 0.072)
        self.declare_parameter("wheel_separation_m", 0.235)
        self.declare_parameter("max_tick_delta", 2000)
        if not self.get_parameter("preview_enabled").value:
            raise RuntimeError("wheel odometry is locked; preview_enabled=false")
        self.estimator = WheelOdometry(
            self.get_parameter("encoder_counts_per_rev").value,
            self.get_parameter("wheel_diameter_m").value,
            self.get_parameter("wheel_separation_m").value,
            self.get_parameter("max_tick_delta").value,
        )
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            String, self.get_parameter("input_topic").value, self.on_counts, 10
        )

    def on_counts(self, message):
        try:
            payload = json.loads(message.data)
            update = self.estimator.update(
                payload["left"], payload["right"], payload["timestamp"]
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning("Rejected encoder sample: {}".format(error))
            return
        output = Odometry()
        seconds = int(update.timestamp)
        output.header.stamp.sec = seconds
        output.header.stamp.nanosec = int((update.timestamp - seconds) * 1e9)
        output.header.frame_id = "odom"
        output.child_frame_id = "base_link"
        output.pose.pose.position.x = update.pose.x
        output.pose.pose.position.y = update.pose.y
        output.pose.pose.orientation.z = math.sin(update.pose.yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(update.pose.yaw / 2.0)
        output.twist.twist.linear.x = update.linear_velocity
        output.twist.twist.angular.z = update.angular_velocity
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = WheelOdometryNode()
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
