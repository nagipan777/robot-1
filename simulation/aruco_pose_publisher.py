#!/usr/bin/env python3
"""Validate camera ArUco observations and publish map-frame robot pose."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from aruco_pose_core import (
    PoseFreshnessGate,
    StreamTimeoutGate,
    marker_pose,
    select_marker_observation,
)


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__("aruco_pose_publisher")
        self.declare_parameter("detections_topic", "/webcam/detections")
        self.declare_parameter("candidate_topic", "/robot/pose_candidate")
        self.declare_parameter("localization_topic", "/localization/aruco_pose")
        self.declare_parameter("pose_topic", "/robot/pose")
        self.declare_parameter("status_topic", "/robot/pose_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size_m", 0.0)
        self.declare_parameter("pose_calibration_ready", False)
        self.declare_parameter("pose_output_enabled", False)
        self.declare_parameter("calibration_image_width", 1280)
        self.declare_parameter("calibration_image_height", 960)
        self.declare_parameter("image_to_map_homography", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("camera_ground_x", 0.0)
        self.declare_parameter("camera_ground_y", 0.0)
        self.declare_parameter("camera_height_m", 0.0)
        self.declare_parameter("marker_height_m", 0.0)
        self.declare_parameter("marker_to_robot_forward_m", 0.0)
        self.declare_parameter("marker_to_robot_left_m", 0.0)
        self.declare_parameter("marker_yaw_offset_rad", 0.0)
        self.declare_parameter("min_marker_pixel_area", 100.0)
        self.declare_parameter("max_pose_age", 0.5)
        self.declare_parameter("min_consecutive_detections", 3)
        self.declare_parameter("map_bounds", [0.0, 3.0, -1.5, 1.5])

        self.candidate = self.create_publisher(
            PoseStamped, self.get_parameter("candidate_topic").value, 10
        )
        if self.get_parameter("marker_size_m").value < 0.0:
            raise ValueError("marker_size_m must be non-negative")
        self.localization = self.create_publisher(
            PoseStamped, self.get_parameter("localization_topic").value, 10
        )
        self.output_enabled = self.get_parameter("pose_output_enabled").value
        self.approved = (
            self.create_publisher(PoseStamped, self.get_parameter("pose_topic").value, 10)
            if self.output_enabled
            else None
        )
        self.status = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.gate = PoseFreshnessGate(
            self.get_parameter("max_pose_age").value,
            self.get_parameter("min_consecutive_detections").value,
        )
        self.stream_gate = StreamTimeoutGate(
            self.get_parameter("max_pose_age").value
        )
        self.last_status = None
        self.create_subscription(
            String, self.get_parameter("detections_topic").value, self.on_detection, 10
        )
        self.create_timer(0.1, self.check_marker_timeout)

    def publish_status(self, state, reason):
        status = (state, reason)
        if status == self.last_status:
            return
        self.last_status = status
        message = String()
        message.data = json.dumps({"state": state, "reason": reason}, separators=(",", ":"))
        self.status.publish(message)

    def on_detection(self, message):
        try:
            payload = json.loads(message.data)
            self.stream_gate.frame_received(time.monotonic())
            timestamp = float(payload["timestamp"])
            if (
                payload.get("image_width") != self.get_parameter("calibration_image_width").value
                or payload.get("image_height") != self.get_parameter("calibration_image_height").value
            ):
                raise ValueError("camera_resolution_mismatch")
            observations = payload.get("aruco_detections", [])
            observation = select_marker_observation(
                observations, self.get_parameter("marker_id").value
            )
            if observation is None:
                raise StopIteration
            calibration_ready = self.get_parameter("pose_calibration_ready").value
            now = self.get_clock().now().nanoseconds / 1e9
            accepted, reason = self.gate.observe(timestamp, now)
            if reason == "stale_or_future" or reason == "non_monotonic":
                raise ValueError(reason)
            x, y, yaw = marker_pose(
                observation["corners"],
                self.get_parameter("image_to_map_homography").value,
                camera_ground=(
                    self.get_parameter("camera_ground_x").value,
                    self.get_parameter("camera_ground_y").value,
                ),
                camera_height=self.get_parameter("camera_height_m").value,
                marker_height=self.get_parameter("marker_height_m").value,
                marker_to_robot_forward=self.get_parameter("marker_to_robot_forward_m").value,
                marker_to_robot_left=self.get_parameter("marker_to_robot_left_m").value,
                yaw_offset=self.get_parameter("marker_yaw_offset_rad").value,
                min_pixel_area=self.get_parameter("min_marker_pixel_area").value,
                map_bounds=self.get_parameter("map_bounds").value,
            )
            pose = PoseStamped()
            seconds = int(timestamp)
            pose.header.stamp.sec = seconds
            pose.header.stamp.nanosec = int((timestamp - seconds) * 1e9)
            pose.header.frame_id = self.get_parameter("map_frame").value
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.candidate.publish(pose)
            # Candidate output is deliberately available for the stationary
            # calibration check.  Unverified geometry must never progress to
            # localization or the authoritative /robot/pose output.
            if not calibration_ready:
                self.publish_status("CANDIDATE_ONLY", "pose_calibration_not_ready")
                return
            if not accepted:
                self.publish_status("REJECTED", reason)
                return
            self.localization.publish(pose)
            if self.approved is None:
                self.publish_status("CANDIDATE_ONLY", "pose_output_disabled")
                return
            publishers = self.get_publishers_info_by_topic(
                self.get_parameter("pose_topic").value
            )
            if len(publishers) != 1:
                self.publish_status("REJECTED", "multiple_pose_publishers")
                return
            self.approved.publish(pose)
            self.publish_status("ACTIVE", "accepted")
        except StopIteration:
            self.gate.missing()
            self.publish_status("LOST", "marker_lost")
        except (KeyError, TypeError, ValueError) as error:
            self.gate.reset()
            self.publish_status("REJECTED", str(error))

    def check_marker_timeout(self):
        if self.stream_gate.check(time.monotonic()):
            self.gate.missing()
            self.publish_status("LOST", "marker_timeout")


def main():
    rclpy.init()
    node = ArucoPosePublisher()
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
