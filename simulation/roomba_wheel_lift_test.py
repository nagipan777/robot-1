#!/usr/bin/env python3
"""Explicit, bounded ROS 2 publisher for a Roomba wheel-lift test."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from roomba_wheel_lift_test_core import (
    SubscriberConnectionGate,
    WATCHDOG_SILENCE_S,
    WheelLiftTestProfile,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class RoombaWheelLiftTest(Node):
    def __init__(self):
        super().__init__("roomba_wheel_lift_test")
        self.declare_parameter("command_topic", "/roomba/cmd_vel_unstamped")
        self.declare_parameter("mode", "stop")
        self.declare_parameter("left_mm_s", 20.0)
        self.declare_parameter("right_mm_s", 20.0)
        self.declare_parameter("drive_duration_s", 1.0)
        self.declare_parameter("wheel_base_m", 0.235)
        self.declare_parameter("allow_rotation", False)
        self.declare_parameter("confirmation", "")
        self.declare_parameter("subscriber_wait_timeout_s", 5.0)

        self.command_topic = self.get_parameter("command_topic").value
        self.profile = WheelLiftTestProfile(
            self.get_parameter("mode").value,
            left_mm_s=self.get_parameter("left_mm_s").value,
            right_mm_s=self.get_parameter("right_mm_s").value,
            drive_duration_s=self.get_parameter("drive_duration_s").value,
            wheel_base_m=self.get_parameter("wheel_base_m").value,
            allow_rotation=self.get_parameter("allow_rotation").value,
            confirmation=self.get_parameter("confirmation").value,
        )
        self.publisher = self.create_publisher(Twist, self.command_topic, COMMAND_QOS)
        now = time.monotonic()
        self.connection_gate = SubscriberConnectionGate(
            self.get_parameter("subscriber_wait_timeout_s").value,
            started_at=now,
        )
        self.started_at = None
        self.finished = False
        self.silence_announced = False
        self.nonzero_announced = False
        self.create_timer(0.05, self.tick)
        self.get_logger().warn(
            "wheel-lift test mode=%s topic=%s; existing bridge/Pico locks are unchanged"
            % (self.profile.mode, self.command_topic)
        )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def publish_test_velocity(self):
        linear_x, angular_z = self.profile.twist()
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.publisher.publish(message)
        if not self.nonzero_announced:
            self.nonzero_announced = True
            self.get_logger().warn(
                "published non-zero Twist for %s: linear.x=%.6f angular.z=%.6f; "
                "bridge rejection is expected"
                % (self.profile.mode, linear_x, angular_z)
            )

    def finish(self):
        if self.finished:
            return
        self.publish_stop()
        self.finished = True
        self.get_logger().info("test complete; STOP published")

    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        if self.started_at is None:
            count = self.publisher.get_subscription_count()
            if not self.connection_gate.ready(count, now):
                return
            self.started_at = now
            self.get_logger().info(
                "matched %d subscriber(s) on %s; type=geometry_msgs/msg/Twist; "
                "qos=reliable/volatile"
                % (count, self.command_topic)
            )
        elapsed = now - self.started_at

        if self.profile.mode == "stop":
            self.finish()
            return

        if elapsed < self.profile.drive_duration_s:
            self.publish_test_velocity()
            return

        if self.profile.mode == "watchdog":
            if not self.silence_announced:
                self.silence_announced = True
                self.get_logger().warn(
                    "watchdog verification window: publishing nothing for %.2fs; "
                    "silence_start_monotonic=%.6f; check Pico watchdog STOP log"
                    % (WATCHDOG_SILENCE_S, time.monotonic())
                )
            if elapsed < self.profile.drive_duration_s + WATCHDOG_SILENCE_S:
                return

        self.finish()

    def emergency_stop(self):
        # The intentional watchdog silence is implemented only in tick().
        # Exceptions and process shutdown still request a STOP.
        self.publish_stop()


def main():
    rclpy.init()
    node = None
    try:
        node = RoombaWheelLiftTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, Exception):
        if node is not None:
            node.emergency_stop()
        raise
    finally:
        if node is not None:
            if not node.finished:
                node.emergency_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
