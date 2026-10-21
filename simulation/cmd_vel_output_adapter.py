#!/usr/bin/env python3
"""Route navigation Twist output to exactly one safe integration boundary."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


VALID_MODES = ("gazebo", "roomba", "dry_run")


def finite_twist(message):
    return all(
        math.isfinite(value)
        for value in (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
    )


class CmdVelOutputAdapter(Node):
    def __init__(self):
        super().__init__("cmd_vel_output_adapter")
        self.declare_parameter("mode", "dry_run")
        self.declare_parameter("input_topic", "/nav/cmd_vel_unstamped")
        self.declare_parameter("gazebo_topic", "/cmd_vel_unstamped")
        self.declare_parameter("roomba_topic", "/roomba/cmd_vel_unstamped")
        self.declare_parameter("dry_run_topic", "/roomba/cmd_vel_dry_run")
        self.declare_parameter("command_timeout", 0.5)

        self.mode = self.get_parameter("mode").value
        if self.mode not in VALID_MODES:
            raise ValueError(f"mode must be one of {VALID_MODES}")
        timeout = float(self.get_parameter("command_timeout").value)
        if timeout <= 0.0:
            raise ValueError("command_timeout must be positive")

        topic_parameter = {
            "gazebo": "gazebo_topic",
            "roomba": "roomba_topic",
            "dry_run": "dry_run_topic",
        }[self.mode]
        self.publisher = self.create_publisher(
            Twist, self.get_parameter(topic_parameter).value, 10
        )
        self.last_command_at = None
        self.output_active = False
        self.create_subscription(
            Twist, self.get_parameter("input_topic").value, self.on_command, 10
        )
        self.create_timer(0.05, self.check_timeout)
        self.get_logger().info(
            f"mode={self.mode}, input={self.get_parameter('input_topic').value}, "
            f"output={self.get_parameter(topic_parameter).value}"
        )
        if self.mode == "dry_run":
            self.get_logger().warning(
                "DRY-RUN: commands are observable only; no physical command topic is published"
            )

    def publish_stop(self, reason):
        self.publisher.publish(Twist())
        self.output_active = False
        self.get_logger().warning(f"output stopped: {reason}")

    def on_command(self, message):
        if not finite_twist(message):
            self.last_command_at = None
            self.publish_stop("non-finite command")
            return
        self.publisher.publish(message)
        self.last_command_at = time.monotonic()
        self.output_active = any(
            abs(value) > 0.0 for value in (message.linear.x, message.angular.z)
        )
        if self.mode == "dry_run":
            self.get_logger().info(
                f"dry-run command: linear.x={message.linear.x:.3f} m/s, "
                f"angular.z={message.angular.z:.3f} rad/s"
            )

    def check_timeout(self):
        if self.last_command_at is None:
            return
        timeout = float(self.get_parameter("command_timeout").value)
        if time.monotonic() - self.last_command_at <= timeout:
            return
        self.last_command_at = None
        self.publish_stop("navigation command timeout")


def main():
    rclpy.init()
    node = CmdVelOutputAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
