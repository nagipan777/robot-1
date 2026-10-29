#!/usr/bin/env python3
"""Opt-in STOP-first static encoder publisher; never sends ARM or Drive."""

import csv
import json
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String, UInt16MultiArray

from roomba_tcp_transport import RoombaTcpTransport


CSV_FIELDS = (
    "timestamp",
    "wall_time_ns",
    "monotonic_ns",
    "left_encoder",
    "right_encoder",
    "valid",
    "communication_state",
    "sequence",
    "error",
)


class RoombaEncoderRosNode(Node):
    def __init__(self, parameter_overrides=None, transport_factory=None):
        super().__init__(
            "roomba_encoder_diagnostic",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("diagnostic_enabled", False)
        self.declare_parameter("static_stop_first_confirmation", "")
        self.declare_parameter("host", "")
        self.declare_parameter("port", 9000)
        self.declare_parameter("sample_count", 1)
        self.declare_parameter("sample_interval_s", 0.2)
        self.declare_parameter("counts_topic", "/roomba/encoder_counts")
        self.declare_parameter("status_topic", "/roomba/encoder_status")
        self.declare_parameter("csv_path", "")
        if not self.get_parameter("diagnostic_enabled").value:
            raise RuntimeError("encoder diagnostic is locked; diagnostic_enabled=false")
        if (
            self.get_parameter("static_stop_first_confirmation").value
            != "STATIC_STOP_FIRST"
        ):
            raise RuntimeError("STATIC_STOP_FIRST confirmation is required")
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        interval = float(self.get_parameter("sample_interval_s").value)
        csv_path = str(self.get_parameter("csv_path").value)
        if not host:
            raise ValueError("host is required")
        if not 1 <= port <= 65535:
            raise ValueError("port must be from 1 to 65535")
        if self.sample_count < 1:
            raise ValueError("sample_count must be positive")
        if interval <= 0.0:
            raise ValueError("sample_interval_s must be positive")
        if not csv_path:
            raise ValueError("csv_path is required")
        factory = transport_factory or RoombaTcpTransport
        self.transport = factory(host, port)
        self.counts_publisher = self.create_publisher(
            UInt16MultiArray, self.get_parameter("counts_topic").value, 10
        )
        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        path = Path(csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_file, fieldnames=CSV_FIELDS, lineterminator="\n"
        )
        self.csv_writer.writeheader()
        self.sequence = 0
        self.connected = False
        self.finished = False
        self.timer = self.create_timer(interval, self.sample_once)

    def payload(self, *, valid, state, left=None, right=None, error=""):
        wall_time_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        return {
            "timestamp": wall_time_ns / 1e9,
            "wall_time_ns": wall_time_ns,
            "monotonic_ns": monotonic_ns,
            "left_encoder": left,
            "right_encoder": right,
            "valid": bool(valid),
            "communication_state": state,
            "sequence": self.sequence,
            "error": str(error),
        }

    def publish_status(self, payload):
        message = String()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.status_publisher.publish(message)
        self.csv_writer.writerow(payload)
        self.csv_file.flush()

    def sample_once(self):
        if self.finished:
            return
        self.sequence += 1
        try:
            if not self.connected:
                self.transport.connect()
                self.transport.verify_stop()
                mode = self.transport.query_oi_mode()
                if mode != 2:
                    raise RuntimeError("Roomba OI mode is not Safe: {}".format(mode))
                self.connected = True
            left, right = self.transport.query_encoders()
            payload = self.payload(
                valid=True, state="connected", left=left, right=right
            )
            counts = UInt16MultiArray()
            counts.data = [left, right]
            self.status_publisher.publish(
                String(
                    data=json.dumps(
                        payload, sort_keys=True, separators=(",", ":")
                    )
                )
            )
            self.counts_publisher.publish(counts)
            self.csv_writer.writerow(payload)
            self.csv_file.flush()
            self.get_logger().info(
                "encoder sequence={} left={} right={}".format(
                    self.sequence, left, right
                )
            )
        except Exception as error:
            self.connected = False
            self.publish_status(
                self.payload(valid=False, state="disconnected", error=error)
            )
            self.get_logger().error(
                "encoder diagnostic failed: {}".format(error)
            )
            self.finish()
            return
        if self.sequence >= self.sample_count:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        self.transport.close()

    def destroy_node(self):
        self.finish()
        if getattr(self, "csv_file", None) is not None:
            self.csv_file.close()
            self.csv_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = RoombaEncoderRosNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
