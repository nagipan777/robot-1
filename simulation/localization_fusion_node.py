#!/usr/bin/env python3
"""Fuse accepted ArUco map poses with wheel odometry predictions."""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from localization_core import Pose2D, PoseFusion


def yaw_from_quaternion(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


class LocalizationFusionNode(Node):
    def __init__(self):
        super().__init__("localization_fusion")
        self.declare_parameter("aruco_topic", "/localization/aruco_pose")
        self.declare_parameter("odom_topic", "/localization/wheel_odom")
        self.declare_parameter("pose_topic", "/robot/pose")
        self.declare_parameter("status_topic", "/localization/status")
        self.declare_parameter("pose_output_enabled", False)
        self.declare_parameter("correction_gain", 0.25)
        self.declare_parameter("max_position_innovation_m", 0.5)
        self.declare_parameter("max_yaw_innovation_rad", 0.8)
        self.declare_parameter("max_aruco_age", 0.5)
        self.declare_parameter("max_odom_age", 0.5)
        self.declare_parameter("max_dead_reckon_age", 2.0)
        self.fusion = PoseFusion(
            correction_gain=self.get_parameter("correction_gain").value,
            max_position_innovation_m=self.get_parameter("max_position_innovation_m").value,
            max_yaw_innovation_rad=self.get_parameter("max_yaw_innovation_rad").value,
            max_aruco_age=self.get_parameter("max_aruco_age").value,
            max_odom_age=self.get_parameter("max_odom_age").value,
            max_dead_reckon_age=self.get_parameter("max_dead_reckon_age").value,
        )
        self.output_enabled = self.get_parameter("pose_output_enabled").value
        self.pose_publisher = (
            self.create_publisher(PoseStamped, self.get_parameter("pose_topic").value, 10)
            if self.output_enabled else None
        )
        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.last_status = None
        self.create_subscription(
            PoseStamped, self.get_parameter("aruco_topic").value, self.on_aruco, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10
        )
        self.create_timer(0.1, self.publish_estimate)

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def publish_status(self, state, confidence, reason=""):
        value = (state, round(confidence, 6), reason)
        if value == self.last_status:
            return
        self.last_status = value
        message = String()
        message.data = json.dumps(
            {"state": state, "confidence": confidence, "reason": reason},
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)

    def on_odom(self, message):
        try:
            self.fusion.observe_odom(
                Pose2D(
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw_from_quaternion(message.pose.pose.orientation),
                ),
                self.stamp_seconds(message.header.stamp),
                message.header.frame_id,
            )
        except ValueError as error:
            self.publish_status("REJECTED", 0.0, str(error))

    def on_aruco(self, message):
        accepted, reason = self.fusion.observe_aruco(
            Pose2D(
                message.pose.position.x,
                message.pose.position.y,
                yaw_from_quaternion(message.pose.orientation),
            ),
            self.stamp_seconds(message.header.stamp),
            self.get_clock().now().nanoseconds / 1e9,
            message.header.frame_id,
        )
        if not accepted:
            self.publish_status("REJECTED", 0.0, reason)

    def publish_estimate(self):
        now = self.get_clock().now().nanoseconds / 1e9
        pose, state, confidence = self.fusion.estimate(now)
        if pose is None:
            self.publish_status(state, confidence)
            return
        if self.pose_publisher is None:
            self.publish_status(state, confidence, "pose_output_disabled")
            return
        if len(self.get_publishers_info_by_topic(self.get_parameter("pose_topic").value)) != 1:
            self.publish_status("REJECTED", 0.0, "multiple_pose_publishers")
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.orientation.w = math.cos(pose.yaw / 2.0)
        self.pose_publisher.publish(message)
        self.publish_status(state, confidence)


def main():
    rclpy.init()
    node = LocalizationFusionNode()
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
