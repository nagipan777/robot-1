#!/usr/bin/env python3
"""Read-only RViz overlays for the PAI demonstration recording."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PresentationVisualizer(Node):
    """Visualize pose, target, command, and recent path without controlling motion."""

    def __init__(self):
        super().__init__("pai_presentation_visualizer")
        self.publisher = self.create_publisher(Marker, "/pai/presentation_markers", 10)
        self.pose = None
        self.target = None
        self.command = Twist()
        self.reached = False
        self.trail = deque(maxlen=300)
        self.create_subscription(PoseStamped, "/robot/pose", self.on_pose, 10)
        self.create_subscription(PointStamped, "/litter/target", self.on_target, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.on_command, 10)
        self.create_subscription(Bool, "/litter/reached", self.on_reached, 10)
        self.create_timer(0.2, self.publish_markers)
        self.get_logger().info("Presentation overlays enabled (read-only).")

    def on_pose(self, message):
        self.pose = message
        point = Point(x=message.pose.position.x, y=message.pose.position.y, z=0.035)
        if not self.trail or math.hypot(point.x - self.trail[-1].x, point.y - self.trail[-1].y) >= 0.02:
            self.trail.append(point)

    def on_target(self, message):
        self.target = message

    def on_command(self, message):
        self.command = message

    def on_reached(self, message):
        self.reached = bool(message.data)

    def marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pai_demo"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.95
        return marker

    def publish_markers(self):
        if self.pose is None:
            return
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        trail = self.marker(0, Marker.LINE_STRIP)
        trail.scale.x = 0.025
        trail.color.r, trail.color.g, trail.color.b = 0.05, 0.75, 1.0
        trail.points = list(self.trail)
        self.publisher.publish(trail)

        heading = self.marker(1, Marker.ARROW)
        heading.pose.position.x, heading.pose.position.y, heading.pose.position.z = x, y, 0.12
        heading.pose.orientation = self.pose.pose.orientation
        heading.scale.x, heading.scale.y, heading.scale.z = 0.48, 0.07, 0.07
        heading.color.r, heading.color.g, heading.color.b = 0.05, 0.9, 1.0
        self.publisher.publish(heading)

        if self.target is not None:
            tx, ty = self.target.point.x, self.target.point.y
            route = self.marker(2, Marker.LINE_LIST)
            route.scale.x = 0.018
            route.color.r, route.color.g, route.color.b = 1.0, 0.55, 0.05
            route.points = [Point(x=x, y=y, z=0.06), Point(x=tx, y=ty, z=0.06)]
            self.publisher.publish(route)
            ring = self.marker(3, Marker.CYLINDER)
            ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = tx, ty, 0.025
            ring.scale.x = ring.scale.y = 0.30
            ring.scale.z = 0.025
            ring.color.r, ring.color.g, ring.color.b = ((0.05, 0.9, 0.25) if self.reached else (1.0, 0.35, 0.05))
            self.publisher.publish(ring)

        status = self.marker(4, Marker.TEXT_VIEW_FACING)
        status.pose.position.x, status.pose.position.y, status.pose.position.z = x, y, 0.72
        status.scale.z = 0.18
        moving = abs(self.command.linear.x) > 0.001 or abs(self.command.angular.z) > 0.001
        state = "REACHED / STOP" if self.reached else ("TRACKING" if moving else "READY / STOP")
        status.text = f"{state} | v={self.command.linear.x:.2f} m/s | w={self.command.angular.z:.2f} rad/s"
        status.color.r, status.color.g, status.color.b = ((0.1, 1.0, 0.25) if self.reached else ((1.0, 0.75, 0.1) if moving else (0.75, 0.85, 1.0)))
        self.publisher.publish(status)


def main():
    rclpy.init()
    node = PresentationVisualizer()
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
