#!/usr/bin/env python3
"""Opt-in ROS recorder for wheel commands versus encoder observations."""

import csv
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt32

from differential_drive import twist_to_wheel_speeds
from odometry_dryrun_core import parse_encoder_sample
from wheel_motion_monitor import MonitorEvent, WheelMotionMonitor


class WheelMotionMonitorNode(Node):
    def __init__(self):
        super().__init__("wheel_motion_monitor")
        self.declare_parameter("monitor_enabled", False)
        self.declare_parameter("encoder_topic", "/roomba/encoder_counts")
        self.declare_parameter("command_topic", "/roomba/cmd_vel_unstamped")
        self.declare_parameter("arm_topic", "/roomba/arm_generation")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("wheel_base_m", 0.235)
        self.declare_parameter("max_speed_mm_s", 20)
        self.declare_parameter("encoder_counts_per_rev", 508.8)
        self.declare_parameter("wheel_diameter_m", 0.072)
        self.declare_parameter("command_no_motion_grace_s", 0.25)
        self.declare_parameter("stop_motion_grace_s", 0.20)
        self.declare_parameter("measurement_stale_s", 0.50)
        self.declare_parameter("encoder_motion_threshold_ticks", 1)
        self.declare_parameter("command_motion_threshold_mm_s", 1)
        self.declare_parameter("max_tick_delta", 2000)
        if not self.get_parameter("monitor_enabled").value:
            raise RuntimeError("wheel motion monitor is locked; monitor_enabled=false")
        csv_path = str(self.get_parameter("csv_path").value)
        if not csv_path:
            raise ValueError("csv_path is required when monitor_enabled=true")
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.max_speed_mm_s = int(self.get_parameter("max_speed_mm_s").value)
        started = time.time()
        self.monitor = WheelMotionMonitor(
            encoder_counts_per_rev=self.get_parameter("encoder_counts_per_rev").value,
            wheel_diameter_m=self.get_parameter("wheel_diameter_m").value,
            wheel_separation_m=self.wheel_base_m,
            command_no_motion_grace_s=self.get_parameter("command_no_motion_grace_s").value,
            stop_motion_grace_s=self.get_parameter("stop_motion_grace_s").value,
            measurement_stale_s=self.get_parameter("measurement_stale_s").value,
            encoder_motion_threshold_ticks=self.get_parameter("encoder_motion_threshold_ticks").value,
            command_motion_threshold_mm_s=self.get_parameter("command_motion_threshold_mm_s").value,
            max_tick_delta=self.get_parameter("max_tick_delta").value,
            started_at_s=started,
        )
        path = Path(csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=list(MonitorEvent.__dataclass_fields__),
            lineterminator="\n",
        )
        self.writer.writeheader()
        self.create_subscription(String, self.get_parameter("encoder_topic").value, self.on_encoder, 10)
        self.create_subscription(Twist, self.get_parameter("command_topic").value, self.on_command, 10)
        self.create_subscription(UInt32, self.get_parameter("arm_topic").value, self.on_arm, 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().warning(
            "observation only: warnings are not connected to STOP; encoder input must come through the existing bridge owner"
        )

    def record(self, event):
        if event is None:
            return
        self.writer.writerow(event.row())
        self.csv_file.flush()
        message = "wheel monitor {}: {}".format(event.event, event.detail)
        if event.severity == "warning":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def on_arm(self, message):
        self.record(self.monitor.observe_arm_request(int(message.data), time.time()))

    def on_command(self, message):
        left, right = twist_to_wheel_speeds(
            message.linear.x,
            message.angular.z,
            track_width_m=self.wheel_base_m,
            max_wheel_speed_mm_s=self.max_speed_mm_s,
        )
        self.record(self.monitor.observe_command(left, right, time.time()))

    def on_encoder(self, message):
        received = time.time()
        try:
            sample = parse_encoder_sample(json.loads(message.data))
            event = self.monitor.observe_encoder(sample, received)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            event = self.monitor.observe_unavailable(received, error)
        self.record(event)

    def on_timer(self):
        self.record(self.monitor.poll(time.time()))

    def destroy_node(self):
        if getattr(self, "csv_file", None) is not None:
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = WheelMotionMonitorNode()
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
