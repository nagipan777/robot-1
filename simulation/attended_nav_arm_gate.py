#!/usr/bin/env python3
"""Publish one ARM only after fresh Navigation candidates in an attended trial."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt32

from attended_nav_arm_gate_core import FreshNavigationArmGate


QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class AttendedNavigationArmGate(Node):
    def __init__(self):
        super().__init__("attended_navigation_arm_gate")
        self.declare_parameter("navigation_topic", "/nav/cmd_vel_unstamped")
        self.declare_parameter("arm_topic", "/roomba/arm_generation")
        self.declare_parameter("arm_generation", 0)
        self.declare_parameter("confirmation", "")
        self.declare_parameter("timeout_s", 5.0)
        now = time.monotonic()
        self.gate = FreshNavigationArmGate(
            self.get_parameter("arm_generation").value,
            self.get_parameter("confirmation").value,
            started_at=now,
            timeout_s=self.get_parameter("timeout_s").value,
        )
        self.arm_publisher = self.create_publisher(
            UInt32, self.get_parameter("arm_topic").value, QOS
        )
        self.subscription = self.create_subscription(
            Twist,
            self.get_parameter("navigation_topic").value,
            self.on_navigation,
            QOS,
        )
        self.finished = False
        self.published_at = None
        self.create_timer(0.05, self.on_timer)
        self.get_logger().warn(
            "attended one-shot ARM gate waiting for two fresh Navigation candidates; "
            "it does not publish Drive or retry ARM"
        )

    def on_navigation(self, message):
        generation = self.gate.observe(
            message.linear.x,
            message.angular.z,
            time.monotonic(),
            self.arm_publisher.get_subscription_count(),
        )
        if generation is None:
            return
        arm = UInt32()
        arm.data = generation
        self.arm_publisher.publish(arm)
        self.published_at = time.monotonic()
        self.get_logger().warn(
            "single ARM published after fresh Navigation candidates: generation=%d"
            % generation
        )

    def on_timer(self):
        now = time.monotonic()
        self.gate.check_timeout(now)
        if self.published_at is not None and now - self.published_at >= 0.2:
            self.finished = True


def main():
    rclpy.init()
    node = AttendedNavigationArmGate()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

