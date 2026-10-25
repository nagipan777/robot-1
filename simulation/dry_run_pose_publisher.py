#!/usr/bin/env python3
"""Publish a manually measured, fresh map pose for real-camera dry-run only."""

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DryRunPosePublisher(Node):
    def __init__(self, x, y, yaw):
        super().__init__("dry_run_pose_publisher")
        self.x = x
        self.y = y
        self.yaw = yaw
        self.publisher = self.create_publisher(PoseStamped, "/robot/pose_dry_run", 10)
        self.create_timer(0.1, self.publish_pose)
        self.get_logger().warning(
            f"DRY-RUN manual pose: map=({x:.3f}, {y:.3f}, {yaw:.3f} rad)"
        )

    def publish_pose(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = self.x
        message.pose.position.y = self.y
        message.pose.orientation.z = math.sin(self.yaw / 2.0)
        message.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float, help="map frame heading in radians")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw)):
        parser.error("pose values must be finite")
    rclpy.init()
    node = DryRunPosePublisher(args.x, args.y, args.yaw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
