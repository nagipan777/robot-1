#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class UnstampedCommandRelay(Node):
    def __init__(self):
        super().__init__("unstamped_cmd_relay")
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.relay, 10)

    def relay(self, message):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = message
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = UnstampedCommandRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
