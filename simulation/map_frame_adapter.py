#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def transform_xy_yaw(x, y, yaw, translation_x, translation_y, rotation):
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    return (
        translation_x + cosine * x - sine * y,
        translation_y + sine * x + cosine * y,
        math.atan2(math.sin(yaw + rotation), math.cos(yaw + rotation)),
    )


class MapFrameAdapter(Node):
    def __init__(self):
        super().__init__("map_frame_adapter")
        self.declare_parameter("source_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/robot/pose")
        self.declare_parameter("target_input_topic", "/litter/target_odom")
        self.declare_parameter("target_output_topic", "/litter/target")
        self.declare_parameter("map_from_odom_x", 0.0)
        self.declare_parameter("map_from_odom_y", 0.0)
        self.declare_parameter("map_from_odom_yaw", 0.0)

        self.pose_publisher = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, 10
        )
        self.target_publisher = self.create_publisher(
            PointStamped, self.get_parameter("target_output_topic").value, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("target_input_topic").value,
            self.on_target,
            10,
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()
        self.create_timer(1.0, self.publish_static_transform)

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter("map_frame").value
        transform.child_frame_id = self.get_parameter("source_frame").value
        transform.transform.translation.x = self.get_parameter("map_from_odom_x").value
        transform.transform.translation.y = self.get_parameter("map_from_odom_y").value
        _, _, qz, qw = quaternion_from_yaw(
            self.get_parameter("map_from_odom_yaw").value
        )
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def transform(self, x, y, yaw=0.0):
        return transform_xy_yaw(
            x,
            y,
            yaw,
            self.get_parameter("map_from_odom_x").value,
            self.get_parameter("map_from_odom_y").value,
            self.get_parameter("map_from_odom_yaw").value,
        )

    def source_frame_is_valid(self, frame_id):
        expected = self.get_parameter("source_frame").value
        if frame_id != expected:
            self.get_logger().warning(
                f"Ignoring input in frame {frame_id!r}; expected {expected!r}"
            )
            return False
        return True

    def on_odom(self, message):
        if not self.source_frame_is_valid(message.header.frame_id):
            return
        pose = message.pose.pose
        x, y, yaw = self.transform(
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )
        output = PoseStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.get_parameter("map_frame").value
        output.pose.position.x = x
        output.pose.position.y = y
        output.pose.position.z = pose.position.z
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        output.pose.orientation.x = qx
        output.pose.orientation.y = qy
        output.pose.orientation.z = qz
        output.pose.orientation.w = qw
        self.pose_publisher.publish(output)

    def on_target(self, message):
        if not self.source_frame_is_valid(message.header.frame_id):
            return
        x, y, _ = self.transform(message.point.x, message.point.y)
        output = PointStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.get_parameter("map_frame").value
        output.point.x = x
        output.point.y = y
        output.point.z = message.point.z
        self.target_publisher.publish(output)


def main():
    rclpy.init()
    node = MapFrameAdapter()
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
