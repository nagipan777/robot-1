#!/usr/bin/env python3
"""Opt-in encoder observation to /roomba/odom without TF or navigation output."""

import csv
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from localization_core import Pose2D, RelativePoseComparator, WheelOdometry
from odometry_dryrun_core import DryRunOdometry, parse_encoder_sample


class WheelOdometryNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            "wheel_odometry", parameter_overrides=parameter_overrides or []
        )
        self.declare_parameter("input_topic", "/roomba/encoder_counts")
        self.declare_parameter("output_topic", "/roomba/odom")
        self.declare_parameter("aruco_topic", "/robot/pose")
        self.declare_parameter("frame_id", "roomba_odom")
        self.declare_parameter("child_frame_id", "roomba_base_link")
        self.declare_parameter("odometry_enabled", False)
        self.declare_parameter("preview_enabled", False)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("encoder_counts_per_rev", 508.8)
        self.declare_parameter("wheel_diameter_m", 0.072)
        self.declare_parameter("wheel_separation_m", 0.235)
        self.declare_parameter("max_tick_delta", 2000)
        self.declare_parameter("max_time_gap_s", 1.0)
        self.declare_parameter(
            "pose_covariance_diagonal", [0.0025, 0.0025, 999.0, 999.0, 999.0, 0.01]
        )
        self.declare_parameter(
            "twist_covariance_diagonal", [0.01, 999.0, 999.0, 999.0, 999.0, 0.04]
        )
        if not (
            self.get_parameter("odometry_enabled").value
            or self.get_parameter("preview_enabled").value
        ):
            raise RuntimeError(
                "wheel odometry is locked; odometry_enabled=false"
            )
        self.estimator = WheelOdometry(
            self.get_parameter("encoder_counts_per_rev").value,
            self.get_parameter("wheel_diameter_m").value,
            self.get_parameter("wheel_separation_m").value,
            self.get_parameter("max_tick_delta").value,
        )
        self.processor = DryRunOdometry(
            self.estimator, self.get_parameter("max_time_gap_s").value
        )
        self.pose_covariance = self.covariance(
            self.get_parameter("pose_covariance_diagonal").value,
            "pose_covariance_diagonal",
        )
        self.twist_covariance = self.covariance(
            self.get_parameter("twist_covariance_diagonal").value,
            "twist_covariance_diagonal",
        )
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.comparator = RelativePoseComparator()
        self.csv_file = None
        self.csv_writer = None
        csv_path = self.get_parameter("csv_path").value
        if csv_path:
            path = Path(csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file, lineterminator="\n")
            self.csv_writer.writerow(
                (
                    "timestamp_s", "left_ticks", "right_ticks",
                    "left_delta", "right_delta", "valid",
                    "communication_state", "error", "integrated",
                    "odom_x_m", "odom_y_m", "odom_yaw_rad",
                    "linear_m_s", "angular_rad_s",
                    "aruco_relative_x_m", "aruco_relative_y_m",
                    "aruco_relative_yaw_rad", "position_error_m",
                    "yaw_error_rad", "aruco_age_s",
                )
            )
        self.create_subscription(
            String, self.get_parameter("input_topic").value, self.on_counts, 10
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("aruco_topic").value,
            self.on_aruco,
            10,
        )

    @staticmethod
    def covariance(diagonal, name):
        if len(diagonal) != 6 or any(
            not math.isfinite(float(value)) or float(value) < 0
            for value in diagonal
        ):
            raise ValueError("{} must contain six finite non-negative values".format(name))
        result = [0.0] * 36
        for index, value in enumerate(diagonal):
            result[index * 6 + index] = float(value)
        return result

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    @staticmethod
    def yaw_from_quaternion(quaternion):
        return 2.0 * math.atan2(quaternion.z, quaternion.w)

    def on_aruco(self, message):
        if message.header.frame_id != "map":
            self.get_logger().warning("Ignored ArUco comparison pose outside map frame")
            return
        try:
            self.comparator.observe_aruco(
                Pose2D(
                    message.pose.position.x,
                    message.pose.position.y,
                    self.yaw_from_quaternion(message.pose.orientation),
                ),
                self.stamp_seconds(message.header.stamp),
            )
        except ValueError as error:
            self.get_logger().warning("Rejected ArUco comparison sample: {}".format(error))

    def on_counts(self, message):
        try:
            payload = json.loads(message.data)
            if "left_encoder" not in payload and "left" in payload:
                payload["left_encoder"] = payload["left"]
            if "right_encoder" not in payload and "right" in payload:
                payload["right_encoder"] = payload["right"]
            sample = parse_encoder_sample(payload)
            processed = self.processor.process(sample)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.estimator.resynchronize()
            self.processor.connected = False
            self.get_logger().warning("Rejected encoder sample: {}".format(error))
            return
        if processed.update is None:
            self.write_csv(processed)
            self.get_logger().warning(
                "Encoder integration paused: state={} error={}".format(
                    sample.communication_state, sample.error or processed.reason
                )
            )
            return
        update = processed.update
        output = Odometry()
        seconds = int(update.timestamp)
        output.header.stamp.sec = seconds
        output.header.stamp.nanosec = int((update.timestamp - seconds) * 1e9)
        output.header.frame_id = self.get_parameter("frame_id").value
        output.child_frame_id = self.get_parameter("child_frame_id").value
        output.pose.pose.position.x = update.pose.x
        output.pose.pose.position.y = update.pose.y
        output.pose.pose.orientation.z = math.sin(update.pose.yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(update.pose.yaw / 2.0)
        output.pose.covariance = self.pose_covariance
        output.twist.twist.linear.x = update.linear_velocity
        output.twist.twist.angular.z = update.angular_velocity
        output.twist.covariance = self.twist_covariance
        self.publisher.publish(output)
        self.write_csv(processed)

    def write_csv(self, processed):
        if self.csv_writer is not None:
            sample = processed.sample
            update = processed.update
            pose = self.estimator.pose if update is None else update.pose
            comparison = self.comparator.compare(pose, sample.timestamp)
            comparison_values = ("", "", "", "", "", "")
            if comparison is not None:
                comparison_values = (
                    comparison.aruco_relative.x,
                    comparison.aruco_relative.y,
                    comparison.aruco_relative.yaw,
                    comparison.position_error_m,
                    comparison.yaw_error_rad,
                    comparison.aruco_age_s,
                )
            self.csv_writer.writerow(
                (
                    sample.timestamp,
                    "" if sample.left_encoder is None else sample.left_encoder,
                    "" if sample.right_encoder is None else sample.right_encoder,
                    "" if update is None else update.left_delta,
                    "" if update is None else update.right_delta,
                    sample.valid,
                    sample.communication_state,
                    sample.error,
                    processed.integrated,
                    pose.x,
                    pose.y,
                    pose.yaw,
                    "" if update is None else update.linear_velocity,
                    "" if update is None else update.angular_velocity,
                )
                + comparison_values
            )
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = WheelOdometryNode()
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
