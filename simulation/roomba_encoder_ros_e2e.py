#!/usr/bin/env python3
"""ROS 2 E2E for static encoder topics and observation-only odometry."""

import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, UInt16MultiArray

from roomba_encoder_ros_node import RoombaEncoderRosNode
from wheel_odometry_node import WheelOdometryNode


class FakeTransport:
    def __init__(self, _host, _port):
        self.samples = [(65534, 100), (2, 104), (6, 108)]
        self.index = 0
        self.closed = False
        self.arm_calls = 0
        self.drive_calls = 0

    def connect(self):
        return None

    def verify_stop(self):
        return 7, 0, 0

    def query_oi_mode(self):
        return 2

    def query_encoders(self):
        sample = self.samples[self.index]
        self.index += 1
        return sample

    def close(self):
        self.closed = True


class FailingTransport(FakeTransport):
    def query_encoders(self):
        raise ConnectionError("partial encoder response 3/7")


class Harness(Node):
    def __init__(self):
        super().__init__("roomba_encoder_ros_e2e_harness")
        self.counts = []
        self.status = []
        self.odom = []
        self.invalid_counts = []
        self.invalid_status = []
        self.create_subscription(
            UInt16MultiArray,
            "/roomba/encoder_counts",
            lambda message: self.counts.append(list(message.data)),
            10,
        )
        self.create_subscription(
            String,
            "/roomba/encoder_status",
            lambda message: self.status.append(json.loads(message.data)),
            10,
        )
        self.create_subscription(Odometry, "/roomba/odom", self.odom.append, 10)
        self.create_subscription(
            UInt16MultiArray,
            "/test/invalid_encoder_counts",
            self.invalid_counts.append,
            10,
        )
        self.create_subscription(
            String,
            "/test/invalid_encoder_status",
            lambda message: self.invalid_status.append(json.loads(message.data)),
            10,
        )


def run(output_directory):
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    encoder_csv = output / "encoder_status.csv"
    odom_csv = output / "odometry.csv"
    summary_path = output / "summary.json"
    rclpy.init()
    encoder = RoombaEncoderRosNode(
        parameter_overrides=[
            Parameter("diagnostic_enabled", value=True),
            Parameter("static_stop_first_confirmation", value="STATIC_STOP_FIRST"),
            Parameter("host", value="mock-only"),
            Parameter("sample_count", value=3),
            Parameter("sample_interval_s", value=0.05),
            Parameter("csv_path", value=str(encoder_csv)),
        ],
        transport_factory=FakeTransport,
    )
    wheel = WheelOdometryNode(
        parameter_overrides=[
            Parameter("odometry_enabled", value=True),
            Parameter("input_topic", value="/roomba/encoder_status"),
            Parameter("aruco_topic", value="/unused/aruco"),
            Parameter("csv_path", value=str(odom_csv)),
        ]
    )
    harness = Harness()
    executor = SingleThreadedExecutor()
    for node in (encoder, wheel, harness):
        executor.add_node(node)
    deadline = time.monotonic() + 5.0
    invalid = None
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if encoder.finished and len(harness.odom) >= 2:
                break
        if harness.counts != [[65534, 100], [2, 104], [6, 108]]:
            raise RuntimeError("unexpected encoder counts: {}".format(harness.counts))
        if len(harness.status) != 3 or not all(
            sample["valid"] and sample["communication_state"] == "connected"
            for sample in harness.status
        ):
            raise RuntimeError("valid status samples were not published")
        if len(harness.odom) != 2:
            raise RuntimeError(
                "expected 2 integrated odometry messages, got {}".format(
                    len(harness.odom)
                )
            )
        with odom_csv.open(encoding="utf-8") as source:
            odom_rows = list(csv.DictReader(source))
        if odom_rows[0]["integrated"] != "False":
            raise RuntimeError("first encoder sample was not treated as baseline")
        if (
            odom_rows[1]["left_delta"] != "4"
            or odom_rows[1]["right_delta"] != "4"
        ):
            raise RuntimeError("16-bit wraparound was not integrated as +4/+4")
        transport = encoder.transport
        if transport.arm_calls or transport.drive_calls:
            raise RuntimeError("mock transport observed ARM or Drive")
        invalid = RoombaEncoderRosNode(
            parameter_overrides=[
                Parameter("diagnostic_enabled", value=True),
                Parameter(
                    "static_stop_first_confirmation", value="STATIC_STOP_FIRST"
                ),
                Parameter("host", value="mock-only"),
                Parameter("sample_count", value=1),
                Parameter("sample_interval_s", value=0.05),
                Parameter("counts_topic", value="/test/invalid_encoder_counts"),
                Parameter("status_topic", value="/test/invalid_encoder_status"),
                Parameter("csv_path", value=str(output / "invalid_status.csv")),
            ],
            transport_factory=FailingTransport,
        )
        executor.add_node(invalid)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if invalid.finished and harness.invalid_status:
                break
        if harness.invalid_counts:
            raise RuntimeError("invalid encoder sample was published as counts")
        if len(harness.invalid_status) != 1 or harness.invalid_status[0]["valid"]:
            raise RuntimeError("invalid encoder status was not published")
        result = {
            "passed": True,
            "data_kind": "mock_ros2",
            "real_uart_used": False,
            "counts_topic": "/roomba/encoder_counts",
            "counts_type": "std_msgs/msg/UInt16MultiArray",
            "counts": harness.counts,
            "status_topic": "/roomba/encoder_status",
            "odom_topic": "/roomba/odom",
            "odom_type": "nav_msgs/msg/Odometry",
            "tf_published": False,
            "navigation_connected": False,
            "arm_sent": False,
            "drive_sent": False,
            "invalid_counts_published": 0,
            "invalid_status_published": 1,
        }
        summary_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return result
    finally:
        executor.shutdown()
        if invalid is not None:
            invalid.destroy_node()
        for node in (harness, wheel, encoder):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
