#!/usr/bin/env python3
"""Explicit, bounded ROS 2 publisher for a Roomba wheel-lift test."""

import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from roomba_wheel_lift_test_core import (
    SubscriberConnectionGate,
    WATCHDOG_SILENCE_S,
    WheelLiftTestProfile,
)


COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    # Match the bridge: newer velocity commands supersede older ones. Reliable
    # delivery is retained so the final STOP is not made best-effort.
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
ARM_ACCEPT_TIMEOUT_S = 5.0


class RoombaWheelLiftTest(Node):
    def __init__(self):
        super().__init__("roomba_wheel_lift_test")
        self.declare_parameter("command_topic", "/roomba/cmd_vel_unstamped")
        self.declare_parameter("arm_topic", "/roomba/arm_generation")
        self.declare_parameter(
            "arm_accepted_topic", "/roomba/arm_accepted_generation"
        )
        self.declare_parameter("arm_generation", 0)
        self.declare_parameter("mode", "stop")
        self.declare_parameter("left_mm_s", 20.0)
        self.declare_parameter("right_mm_s", 20.0)
        self.declare_parameter("drive_duration_s", 1.0)
        self.declare_parameter("wheel_base_m", 0.235)
        self.declare_parameter("allow_rotation", False)
        self.declare_parameter("confirmation", "")
        self.declare_parameter("subscriber_wait_timeout_s", 5.0)
        self.declare_parameter("publish_period_s", 0.05)

        self.command_topic = self.get_parameter("command_topic").value
        self.arm_topic = self.get_parameter("arm_topic").value
        self.arm_accepted_topic = self.get_parameter("arm_accepted_topic").value
        self.arm_generation = self.get_parameter("arm_generation").value
        self.profile = WheelLiftTestProfile(
            self.get_parameter("mode").value,
            left_mm_s=self.get_parameter("left_mm_s").value,
            right_mm_s=self.get_parameter("right_mm_s").value,
            drive_duration_s=self.get_parameter("drive_duration_s").value,
            wheel_base_m=self.get_parameter("wheel_base_m").value,
            allow_rotation=self.get_parameter("allow_rotation").value,
            confirmation=self.get_parameter("confirmation").value,
        )
        self.publish_period_s = self.get_parameter("publish_period_s").value
        if not isinstance(self.publish_period_s, (int, float)) or not (
            0.05 <= self.publish_period_s <= 0.4
        ):
            raise ValueError("publish_period_s must be from 0.05 to 0.4 seconds")
        self.publisher = self.create_publisher(Twist, self.command_topic, COMMAND_QOS)
        self.arm_publisher = self.create_publisher(UInt32, self.arm_topic, COMMAND_QOS)
        self.create_subscription(
            UInt32, self.arm_accepted_topic, self.on_arm_accepted, COMMAND_QOS
        )
        if self.profile.mode in (
            "rotate",
            "watchdog",
            "floor_straight",
            "continuous_straight",
        ) and not (
            isinstance(self.arm_generation, int)
            and not isinstance(self.arm_generation, bool)
            and 1 <= self.arm_generation <= 0xFFFFFFFF
        ):
            raise ValueError(
                "motion test requires arm_generation from 1 to 4294967295"
            )
        now = time.monotonic()
        self.connection_gate = SubscriberConnectionGate(
            self.get_parameter("subscriber_wait_timeout_s").value,
            started_at=now,
        )
        self.started_at = None
        self.arm_sent_at = None
        self.arm_accepted_at = None
        self.drive_started_at = None
        self.finished = False
        self.silence_announced = False
        self.nonzero_announced = False
        self.create_timer(self.publish_period_s, self.tick)
        self.get_logger().warn(
            "wheel-lift test mode=%s topic=%s; existing bridge/Pico locks are unchanged"
            % (self.profile.mode, self.command_topic)
        )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def on_arm_accepted(self, message):
        if int(message.data) != self.arm_generation or self.arm_accepted_at is not None:
            return
        self.arm_accepted_at = time.monotonic()
        self.get_logger().warn(
            "matching ARM acceptance received: generation=%d" % self.arm_generation
        )

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
                "%s"
                % (
                    self.profile.mode,
                    linear_x,
                    angular_z,
                    "bridge rejection is expected"
                    if self.profile.mode == "rejection"
                    else "bounded wheel-lift forwarding is expected",
                )
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

        if self.profile.mode in (
            "rotate",
            "watchdog",
            "floor_straight",
            "continuous_straight",
        ):
            if self.arm_sent_at is None:
                arm = UInt32()
                arm.data = self.arm_generation
                self.arm_publisher.publish(arm)
                self.arm_sent_at = now
                self.get_logger().warn(
                    "explicit ARM published: generation=%d" % self.arm_generation
                )
                return
            if self.arm_accepted_at is None:
                if now - self.arm_sent_at >= ARM_ACCEPT_TIMEOUT_S:
                    self.get_logger().error(
                        "matching ARM acceptance timed out; STOP only"
                    )
                    self.finish()
                return
            if self.drive_started_at is None:
                # Anchor the requested window to the first non-zero publish,
                # after the bridge has validated Pico's matching ARM ACK.
                self.drive_started_at = now
            elapsed = now - self.drive_started_at

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
