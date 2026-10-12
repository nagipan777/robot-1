#!/usr/bin/env python3
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray


def twist_to_wheels(linear_m_s, angular_rad_s, wheel_separation_m):
    half_track = wheel_separation_m / 2.0
    return (
        (linear_m_s - angular_rad_s * half_track) * 1000.0,
        (linear_m_s + angular_rad_s * half_track) * 1000.0,
    )


def limit_wheels_preserving_ratio(left_mm_s, right_mm_s, limit_mm_s):
    if limit_mm_s <= 0.0:
        raise ValueError("max_wheel_speed_mm_s must be positive")
    peak = max(abs(left_mm_s), abs(right_mm_s))
    if peak <= limit_mm_s:
        return left_mm_s, right_mm_s
    scale = limit_mm_s / peak
    return left_mm_s * scale, right_mm_s * scale


class ShutdownRequested(Exception):
    pass


class RoombaDryRunAdapter(Node):
    def __init__(self):
        super().__init__("roomba_dry_run_adapter")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("wheel_speeds_topic", "/roomba/wheel_speeds_dry_run")
        self.declare_parameter("wheel_separation_m", 0.235)
        self.declare_parameter("max_wheel_speed_mm_s", 100.0)
        self.declare_parameter("command_timeout", 0.5)

        if self.get_parameter("wheel_separation_m").value <= 0.0:
            raise ValueError("wheel_separation_m must be positive")
        if self.get_parameter("max_wheel_speed_mm_s").value <= 0.0:
            raise ValueError("max_wheel_speed_mm_s must be positive")
        if self.get_parameter("command_timeout").value <= 0.0:
            raise ValueError("command_timeout must be positive")

        self.last_command_at = None
        self.last_command = None
        self.stale = True
        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("wheel_speeds_topic").value,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.on_command,
            10,
        )
        self.create_timer(0.05, self.check_timeout)

    def publish_wheels(self, left_mm_s, right_mm_s, reason):
        message = Float64MultiArray()
        message.data = [float(left_mm_s), float(right_mm_s)]
        self.publisher.publish(message)
        self.get_logger().info(
            f"dry-run {reason}: left={left_mm_s:.1f} mm/s, "
            f"right={right_mm_s:.1f} mm/s"
        )

    def on_command(self, message):
        values = (
            message.linear.x,
            message.angular.z,
            self.get_parameter("wheel_separation_m").value,
        )
        if not all(math.isfinite(value) for value in values):
            self.last_command = None
            self.last_command_at = None
            self.stale = True
            self.publish_wheels(0.0, 0.0, "invalid command -> stop")
            return
        left, right = twist_to_wheels(*values)
        left, right = limit_wheels_preserving_ratio(
            left,
            right,
            self.get_parameter("max_wheel_speed_mm_s").value,
        )
        self.last_command = (left, right)
        self.last_command_at = time.monotonic()
        self.stale = False
        self.publish_wheels(left, right, "command")

    def check_timeout(self):
        if self.last_command_at is None:
            return
        if time.monotonic() - self.last_command_at <= self.get_parameter("command_timeout").value:
            return
        self.last_command = None
        self.last_command_at = None
        if not self.stale:
            self.stale = True
            self.publish_wheels(0.0, 0.0, "timeout -> stop")

    def publish_stop(self):
        self.last_command = None
        self.last_command_at = None
        self.stale = True
        self.publish_wheels(0.0, 0.0, "shutdown -> stop")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RoombaDryRunAdapter()

    def shutdown_handler(_signum, _frame):
        raise ShutdownRequested

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, ShutdownRequested):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
