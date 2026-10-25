#!/usr/bin/env python3
"""ROS 2 Twist to Pico W log-only TCP bridge; never emits Roomba OI."""

import math
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from pico_dry_run_transport import DryRunTcpTransport, ReconnectBackoff, ReconnectCommandGate


class PicoDryRunBridge(Node):
    def __init__(self):
        super().__init__("pico_dry_run_bridge")
        self.declare_parameter("network_enabled", False)
        self.declare_parameter("host", "")
        self.declare_parameter("port", 9001)
        self.declare_parameter("command_topic", "/roomba/cmd_vel_dry_run")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_linear_m_s", 0.1)
        self.declare_parameter("max_angular_rad_s", 1.0)
        self.declare_parameter("reconnect_initial_delay", 1.0)
        self.declare_parameter("reconnect_max_delay", 8.0)
        self.network_enabled = bool(self.get_parameter("network_enabled").value)
        host = self.get_parameter("host").value
        if self.network_enabled and not host:
            raise ValueError("host is required when network_enabled=true")
        self.transport = DryRunTcpTransport(host, int(self.get_parameter("port").value))
        self.sequence = 0
        self.last_command_at = None
        self.last_sent_at = None
        self.stop_sent = True
        self.next_retry = 0.0
        self.backoff = ReconnectBackoff(
            float(self.get_parameter("reconnect_initial_delay").value),
            float(self.get_parameter("reconnect_max_delay").value),
        )
        self.reconnect_gate = ReconnectCommandGate()
        self.reconnect_warning_logged = False
        self.last_remote_state = None
        self.state_publisher = self.create_publisher(String, "/pico/dry_run_state", 10)
        self.create_subscription(
            Twist, self.get_parameter("command_topic").value, self.on_command, 10
        )
        self.create_timer(0.05, self.tick)
        if self.network_enabled:
            self.connect()
        else:
            self.get_logger().warning("Pico dry-run network disabled; no TCP connection")

    def publish_state(self, event, **details):
        message = String()
        message.data = json.dumps(
            {"event": event, "at": time.time(), **details}, separators=(",", ":")
        )
        self.state_publisher.publish(message)

    def connect(self):
        try:
            reconnect = self.reconnect_gate.has_connected
            session_id = self.transport.connect()
            self.sequence = 0
            state = self.send_values(0.0, 0.0)
            self.reconnect_gate.connected()
            self.backoff.reset()
            self.reconnect_warning_logged = False
            self.get_logger().info(
                f"Pico dry-run receiver connected; session={session_id}; log-only STOP sent"
            )
            self.publish_state(
                "connected",
                session=session_id,
                reconnect=reconnect,
                first_sequence=self.sequence,
                first_state=state,
            )
        except (OSError, ConnectionError, ValueError) as exc:
            self.transport.close()
            delay = self.backoff.failed()
            self.next_retry = time.monotonic() + delay
            self.get_logger().warning(f"Pico dry-run disconnected: {exc}; retry_in={delay:.1f}s")
            self.publish_state("connect_failed", retry_in=delay, error=str(exc))

    def send_values(self, linear_x, angular_z):
        self.sequence += 1
        state = self.transport.send(
            self.sequence, round(linear_x * 1000), round(angular_z * 1000)
        )
        self.stop_sent = state == "STOPPED"
        self.last_sent_at = time.monotonic()
        if state != self.last_remote_state:
            self.publish_state(
                "remote_state",
                session=self.transport.session_id,
                sequence=self.sequence,
                state=state,
            )
            self.last_remote_state = state
        return state

    def on_command(self, message):
        values = (message.linear.x, message.angular.z)
        valid = all(math.isfinite(value) for value in values)
        valid = valid and abs(values[0]) <= float(self.get_parameter("max_linear_m_s").value)
        valid = valid and abs(values[1]) <= float(self.get_parameter("max_angular_rad_s").value)
        try:
            if not valid:
                if self.transport.socket is not None:
                    self.send_values(0.0, 0.0)
                self.last_command_at = None
                self.get_logger().warning("Invalid Twist rejected; dry-run STOP state requested")
                return
            if not self.network_enabled or self.transport.socket is None:
                self.last_command_at = None
                return
            was_locked = self.reconnect_gate.locked
            if not self.reconnect_gate.allow(*values):
                self.last_command_at = None
                if not self.reconnect_warning_logged:
                    self.get_logger().warning(
                        "Post-reconnect non-zero command rejected; a fresh zero command is required"
                    )
                    self.reconnect_warning_logged = True
                    self.publish_state("nonzero_rejected_until_fresh_zero")
                return
            if was_locked and values == (0.0, 0.0):
                self.publish_state("fresh_zero_unlocked")
            self.reconnect_warning_logged = False
            state = self.send_values(*values)
            self.last_command_at = time.monotonic()
            self.get_logger().info(
                f"Pico ACK seq={self.sequence} state={state} "
                f"linear.x={values[0]:.3f}m/s angular.z={values[1]:.3f}rad/s"
            )
        except (OSError, ConnectionError, ValueError) as exc:
            self.transport.close()
            self.last_command_at = None
            self.last_sent_at = None
            self.stop_sent = True
            self.last_remote_state = "STOPPED"
            delay = self.backoff.failed()
            self.next_retry = time.monotonic() + delay
            self.get_logger().warning(
                f"Communication lost; local state STOPPED: {exc}; retry_in={delay:.1f}s"
            )
            self.publish_state("communication_lost", local_state="STOPPED", retry_in=delay)

    def tick(self):
        if not self.network_enabled:
            return
        if self.transport.socket is None:
            if time.monotonic() >= self.next_retry:
                self.connect()
            return
        if (
            self.stop_sent
            and self.last_sent_at is not None
            and time.monotonic() - self.last_sent_at >= 0.1
        ):
            try:
                self.send_values(0.0, 0.0)
            except (OSError, ConnectionError, ValueError) as exc:
                self.transport.close()
                self.last_sent_at = None
                delay = self.backoff.failed()
                self.next_retry = time.monotonic() + delay
                self.publish_state("communication_lost", local_state="STOPPED", retry_in=delay)
                self.get_logger().warning(
                    f"STOP keepalive failed; connection closed: {exc}; retry_in={delay:.1f}s"
                )
            return
        timeout = float(self.get_parameter("command_timeout").value)
        if self.last_command_at is not None and time.monotonic() - self.last_command_at > timeout:
            self.last_command_at = None
            try:
                self.send_values(0.0, 0.0)
                self.get_logger().warning("ROS command timeout; Pico dry-run STOP sent")
            except (OSError, ConnectionError, ValueError) as exc:
                self.transport.close()
                self.stop_sent = True
                self.last_sent_at = None
                self.last_remote_state = "STOPPED"
                delay = self.backoff.failed()
                self.next_retry = time.monotonic() + delay
                self.get_logger().warning(
                    f"STOP delivery failed; connection closed: {exc}; retry_in={delay:.1f}s"
                )
                self.publish_state("stop_delivery_failed", retry_in=delay)

    def shutdown(self):
        if self.transport.socket is not None:
            try:
                self.send_values(0.0, 0.0)
            except (OSError, ConnectionError, ValueError):
                pass
        self.transport.close()


def main():
    rclpy.init()
    node = PicoDryRunBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
