#!/usr/bin/env python3
"""Measure ROS-to-mock-UART latency without connecting to a real Pico/Roomba."""

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt32

from latency_trace import LatencyTrace
from roomba_tcp_transport import (
    ARM_ACCEPTED,
    CONTROL_ARM,
    CONTROL_ARM_RESPONSE,
    DIAGNOSTIC_MAGIC,
    DIAGNOSTIC_OI_MODE,
    DIAGNOSTIC_OI_MODE_LIVE,
    DIAGNOSTIC_OI_MODE_RESPONSE,
    DIAGNOSTIC_STOP,
    DIAGNOSTIC_STOP_RESPONSE,
    DRIVE_ACK_COMPLETE,
    OI_DRIVE_DIRECT,
    RELAY_READY,
    SEQUENCED_DRIVE,
    SEQUENCED_DRIVE_RESPONSE,
    STOP_ACK_COMPLETE,
    with_crc,
)


class MockPico:
    """Protocol-v2 endpoint whose UART stage is a timestamp only."""

    def __init__(self, trace, ack_delay_s=0.003):
        self.trace = trace
        self.ack_delay_s = ack_delay_s
        self.listener = socket.socket()
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.stop_event = threading.Event()
        self.generation = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        try:
            self.listener.close()
        except OSError:
            pass
        self.thread.join(timeout=2.0)

    def run(self):
        try:
            connection, _ = self.listener.accept()
        except OSError:
            return
        with connection:
            connection.settimeout(0.2)
            connection.sendall(RELAY_READY)
            buffer = bytearray()
            while not self.stop_event.is_set():
                try:
                    chunk = connection.recv(256)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer.extend(chunk)
                while self.consume(connection, buffer):
                    pass

    def consume(self, connection, buffer):
        if not buffer:
            return False
        if buffer[0] == OI_DRIVE_DIRECT:
            if len(buffer) < 5:
                return False
            packet = bytes(buffer[:5])
            del buffer[:5]
            right, left = struct.unpack(">hh", packet[1:])
            self.trace.record(
                "mock_uart_stop_write",
                generation=self.generation,
                right_mm_s=right,
                left_mm_s=left,
            )
            return True
        if buffer[0] != DIAGNOSTIC_MAGIC:
            del buffer[0]
            return True
        if len(buffer) < 2:
            return False
        opcode = buffer[1]
        sizes = {
            DIAGNOSTIC_STOP: 2,
            DIAGNOSTIC_OI_MODE: 2,
            DIAGNOSTIC_OI_MODE_LIVE: 2,
            CONTROL_ARM: 6,
            SEQUENCED_DRIVE: 17,
        }
        size = sizes.get(opcode)
        if size is None:
            del buffer[0]
            return True
        if len(buffer) < size:
            return False
        frame = bytes(buffer[:size])
        del buffer[:size]
        if opcode == DIAGNOSTIC_STOP:
            connection.sendall(
                bytes(
                    [DIAGNOSTIC_MAGIC, DIAGNOSTIC_STOP_RESPONSE, STOP_ACK_COMPLETE]
                )
                + struct.pack(">hh", 0, 0)
            )
        elif opcode in (DIAGNOSTIC_OI_MODE, DIAGNOSTIC_OI_MODE_LIVE):
            connection.sendall(
                bytes([DIAGNOSTIC_MAGIC, DIAGNOSTIC_OI_MODE_RESPONSE, 2])
            )
        elif opcode == CONTROL_ARM:
            generation = struct.unpack(">I", frame[2:])[0]
            self.generation = generation
            connection.sendall(
                bytes([DIAGNOSTIC_MAGIC, CONTROL_ARM_RESPONSE, ARM_ACCEPTED])
                + struct.pack(">I", generation)
            )
        elif opcode == SEQUENCED_DRIVE:
            generation, sequence = struct.unpack(">II", frame[2:10])
            _, right, left = struct.unpack(">Bhh", frame[10:15])
            self.trace.record(
                "mock_drive_received",
                generation=generation,
                sequence=sequence,
                right_mm_s=right,
                left_mm_s=left,
            )
            # This is deliberately not a real machine.UART write.
            self.trace.record(
                "mock_uart_write",
                generation=generation,
                sequence=sequence,
                bytes=5,
            )
            if self.ack_delay_s:
                time.sleep(self.ack_delay_s)
            response = with_crc(
                bytes(
                    [
                        DIAGNOSTIC_MAGIC,
                        SEQUENCED_DRIVE_RESPONSE,
                        DRIVE_ACK_COMPLETE,
                    ]
                )
                + struct.pack(">II", generation, sequence)
            )
            connection.sendall(response)
            self.trace.record(
                "mock_ack_sent", generation=generation, sequence=sequence
            )
        return True


class ProbePublisher(Node):
    def __init__(self):
        super().__init__("roomba_latency_probe")
        self.command = self.create_publisher(
            Twist, "/latency_probe/cmd_vel", 1
        )
        self.arm = self.create_publisher(UInt32, "/latency_probe/arm", 1)

    def wait_for_bridge(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.command.get_subscription_count() >= 1
                and self.arm.get_subscription_count() >= 1
            ):
                return
        raise TimeoutError("mock bridge subscriptions were not discovered")


def read_events(path):
    try:
        with open(path, encoding="utf-8") as stream:
            return [json.loads(line) for line in stream if line.strip()]
    except FileNotFoundError:
        return []


def wait_for_event(path, stage, generation, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for event in read_events(path):
            if event.get("stage") == stage and event.get("generation") == generation:
                return event
        time.sleep(0.01)
    raise TimeoutError("{} was not recorded for generation {}".format(stage, generation))


def wait_for_rejection(path, linear_x, after_ns, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for event in read_events(path):
            if (
                event.get("stage") == "safety_decision"
                and event.get("reason") == "drive_enabled_false"
                and event.get("linear_x") == linear_x
                and event["monotonic_ns"] >= after_ns
            ):
                return event
        time.sleep(0.01)
    raise TimeoutError("drive_enabled=false rejection was not recorded")


def summarize(path, label, generations):
    events = read_events(path)
    rows = []
    for generation in generations:
        matching = [e for e in events if e.get("generation") == generation]
        first = {}
        for event in matching:
            first.setdefault(event["stage"], event)
        stages = (
            "publisher_publish_return",
            "bridge_receive",
            "safety_decision",
            "transport_dispatch",
            "tcp_drive_sent",
            "mock_drive_received",
            "mock_uart_write",
            "mock_ack_sent",
            "tcp_drive_ack_received",
            "tcp_drive_ack_validated",
        )
        missing = [stage for stage in stages if stage not in first]
        if missing:
            raise RuntimeError("missing trace stages: {}".format(", ".join(missing)))
        base = first[stages[0]]["monotonic_ns"]
        row = {"generation": generation}
        for stage in stages:
            row[stage + "_ms"] = round(
                (first[stage]["monotonic_ns"] - base) / 1_000_000.0, 3
            )
        stop_publish = first.get("publisher_stop_return")
        stop_receive = None
        if stop_publish:
            stop_receive = next(
                (
                    event
                    for event in matching
                    if event.get("stage") == "bridge_receive"
                    and event.get("linear_x") == 0.0
                    and event["monotonic_ns"] >= stop_publish["monotonic_ns"]
                ),
                None,
            )
        stop_decision = next(
            (
                event
                for event in matching
                if event.get("stage") == "safety_decision"
                and event.get("reason") == "stop"
            ),
            None,
        )
        if stop_publish and stop_receive and stop_decision:
            stop_base = stop_publish["monotonic_ns"]
            row["stop_bridge_receive_ms"] = round(
                (stop_receive["monotonic_ns"] - stop_base) / 1_000_000.0, 3
            )
            row["stop_safety_decision_ms"] = round(
                (stop_decision["monotonic_ns"] - stop_base) / 1_000_000.0, 3
            )
            stop_uart = next(
                (
                    event
                    for event in matching
                    if event.get("stage") == "mock_uart_stop_write"
                    and event["monotonic_ns"] >= stop_decision["monotonic_ns"]
                ),
                None,
            )
            if stop_uart:
                row["stop_mock_uart_write_ms"] = round(
                    (stop_uart["monotonic_ns"] - stop_base) / 1_000_000.0, 3
                )
        rows.append(row)
    summary = {"label": label, "samples": rows}
    summary_path = os.path.splitext(path)[0] + "-summary.json"
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return summary_path


def summarize_rejections(path, label, samples):
    events = read_events(path)
    rows = []
    for generation, linear_x, publish_ns in samples:
        receive = next(
            event
            for event in events
            if event.get("stage") == "bridge_receive"
            and event.get("linear_x") == linear_x
            and event["monotonic_ns"] >= publish_ns
        )
        decision = next(
            event
            for event in events
            if event.get("stage") == "safety_decision"
            and event.get("reason") == "drive_enabled_false"
            and event.get("linear_x") == linear_x
            and event["monotonic_ns"] >= receive["monotonic_ns"]
        )
        rows.append(
            {
                "generation": generation,
                "linear_x": linear_x,
                "bridge_receive_ms": round(
                    (receive["monotonic_ns"] - publish_ns) / 1_000_000.0, 3
                ),
                "safety_decision_ms": round(
                    (decision["monotonic_ns"] - publish_ns) / 1_000_000.0, 3
                ),
                "receive_to_safety_ms": round(
                    (decision["monotonic_ns"] - receive["monotonic_ns"])
                    / 1_000_000.0,
                    3,
                ),
            }
        )
    summary_path = os.path.splitext(path)[0] + "-summary.json"
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump({"label": label, "samples": rows}, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return summary_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--bridge-script", default="roomba_bridge_tcp.py")
    parser.add_argument("--ack-delay-ms", type=float, default=3.0)
    parser.add_argument("--mode", choices=("pipeline", "rejection"), default="pipeline")
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    try:
        os.unlink(args.output)
    except FileNotFoundError:
        pass
    trace = LatencyTrace(args.output, component="latency_probe")
    mock = MockPico(trace, args.ack_delay_ms / 1000.0)
    mock.start()
    log_path = os.path.splitext(args.output)[0] + "-bridge.log"
    log_stream = open(log_path, "w", encoding="utf-8")
    command = [
        sys.executable,
        args.bridge_script,
        "--ros-args",
        "-p", "host:=127.0.0.1",
        "-p", "port:={}".format(mock.port),
        "-p", "drive_enabled:={}".format("true" if args.mode == "pipeline" else "false"),
        "-p", "diagnostic_on_connect:={}".format("true" if args.mode == "pipeline" else "false"),
        "-p", "command_topic:=/latency_probe/cmd_vel",
        "-p", "arm_topic:=/latency_probe/arm",
        "-p", "max_speed_mm_s:=20",
        "-p", "command_timeout:=0.5",
        "-p", "safe_check_interval:=2.0",
        "-p", "latency_trace_path:={}".format(args.output),
    ]
    bridge = subprocess.Popen(command, stdout=log_stream, stderr=subprocess.STDOUT)
    rclpy.init()
    node = ProbePublisher()
    generations = []
    rejection_samples = []
    try:
        node.wait_for_bridge()
        for index in range(args.samples):
            generation = 1000 + index
            generations.append(generation)
            message = Twist()
            message.linear.x = 0.010 + index * 0.0001
            if args.mode == "pipeline":
                arm = UInt32()
                arm.data = generation
                node.arm.publish(arm)
                wait_for_event(args.output, "arm_accepted", generation)
            publish_ns = time.monotonic_ns()
            trace.record(
                "publisher_publish_start",
                generation=generation,
                linear_x=message.linear.x,
                angular_z=0.0,
            )
            node.command.publish(message)
            trace.record(
                "publisher_publish_return",
                generation=generation,
                linear_x=message.linear.x,
                angular_z=0.0,
            )
            if args.mode == "pipeline":
                wait_for_event(args.output, "tcp_drive_ack_validated", generation)
            else:
                wait_for_rejection(args.output, message.linear.x, publish_ns)
                rejection_samples.append((generation, message.linear.x, publish_ns))
            stop = Twist()
            trace.record("publisher_stop_start", generation=generation)
            node.command.publish(stop)
            trace.record("publisher_stop_return", generation=generation)
            time.sleep(0.10)
        if args.mode == "pipeline":
            print(summarize(args.output, args.label, generations))
        else:
            print(summarize_rejections(args.output, args.label, rejection_samples))
    finally:
        try:
            node.command.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            bridge.terminate()
            bridge.wait(timeout=3.0)
        log_stream.close()
        mock.close()
        trace.close()


if __name__ == "__main__":
    main()
