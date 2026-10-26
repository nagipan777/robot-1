#!/usr/bin/env python3
"""Probe locked ARM ACK and reconnect without sending any Drive frame."""

import argparse
import json
import socket
import statistics
import struct
import time
from pathlib import Path


READY = b"READY\n"
STOP = bytes([145, 0, 0, 0, 0])
ARM = bytes([0xD7, 0x04])
ARM_RESPONSE_PREFIX = bytes([0xD7, 0x84])
ARM_REJECTED_LOCK = 0x02


class ReceiveFailure(Exception):
    def __init__(self, reason, received, expected, raw):
        super().__init__(
            "{} after {}/{} bytes; raw={}".format(
                reason, received, expected, raw.hex()
            )
        )
        self.reason = reason
        self.received = received
        self.expected = expected
        self.raw = raw


def receive_exact(connection, size):
    received = bytearray()
    while len(received) < size:
        try:
            chunk = connection.recv(size - len(received))
        except socket.timeout as error:
            raise ReceiveFailure(
                "timeout", len(received), size, bytes(received)
            ) from error
        if not chunk:
            raise ReceiveFailure(
                "closed", len(received), size, bytes(received)
            )
        received.extend(chunk)
    return bytes(received)


def connect_ready(host, port, timeout):
    started = time.monotonic()
    connection = socket.create_connection((host, port), timeout=timeout)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.settimeout(timeout)
    ready = receive_exact(connection, len(READY))
    if ready != READY:
        connection.close()
        raise RuntimeError("unexpected READY: {!r}".format(ready))
    connection.sendall(STOP)
    return connection, time.monotonic() - started


def close_with_stop(connection):
    if connection is None:
        return
    try:
        connection.sendall(STOP)
    except OSError:
        pass
    connection.close()


def run_cycle(host, port, timeout, generation, cycle_id=None):
    result = {
        "generation": generation,
        "connection_id": cycle_id,
        "ready": False,
        "arm_rejected_lock": False,
        "reconnect_ready": False,
        "error": None,
    }
    first = None
    second = None
    try:
        first, result["ready_s"] = connect_ready(host, port, timeout)
        result["ready"] = True
        arm_started = time.monotonic()
        first.sendall(ARM + struct.pack(">I", generation))
        response = receive_exact(first, 7)
        result["arm_ack_s"] = time.monotonic() - arm_started
        result["arm_ack_raw"] = response.hex()
        expected = ARM_RESPONSE_PREFIX + bytes([ARM_REJECTED_LOCK])
        highest = struct.unpack(">I", response[3:])[0]
        result["highest_generation"] = highest
        if response[:3] != expected or highest != generation:
            raise RuntimeError("unexpected ARM response: {}".format(response.hex()))
        result["arm_rejected_lock"] = True
        close_with_stop(first)
        first = None
    except ReceiveFailure as error:
        result["error"] = "ReceiveFailure: {}".format(error)
        result["failure_reason"] = error.reason
        result["partial_bytes"] = error.received
    except Exception as error:
        result["error"] = "{}: {}".format(type(error).__name__, error)
    finally:
        close_with_stop(first)

    # Recovery is measured even when READY or ARM ACK failed. It is a new TCP
    # session and still sends only the legacy zero STOP after READY.
    try:
        second, result["reconnect_ready_s"] = connect_ready(host, port, timeout)
        result["reconnect_ready"] = True
    except Exception as error:
        result["reconnect_error"] = "{}: {}".format(type(error).__name__, error)
    finally:
        close_with_stop(second)
    result["pass"] = (
        result["ready"]
        and result["arm_rejected_lock"]
        and result["reconnect_ready"]
    )
    return result


def metric(values, function):
    return function(values) if values else None


def percentile_95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, (len(ordered) * 95 + 99) // 100 - 1)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be at least 1")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    base_generation = int(time.time()) & 0xFFFFFFFF
    results = []
    for index in range(args.count):
        result = run_cycle(
            args.host,
            args.port,
            args.timeout,
            base_generation + index,
            cycle_id=index + 1,
        )
        results.append(result)
        print(
            "cycle={}/{} pass={} ready_s={} arm_ack_s={} reconnect_s={} error={}".format(
                index + 1,
                args.count,
                result["pass"],
                result.get("ready_s"),
                result.get("arm_ack_s"),
                result.get("reconnect_ready_s"),
                result["error"],
            )
        )

    ready_times = [item["ready_s"] for item in results if "ready_s" in item]
    ack_times = [item["arm_ack_s"] for item in results if "arm_ack_s" in item]
    reconnect_times = [
        item["reconnect_ready_s"]
        for item in results
        if "reconnect_ready_s" in item
    ]
    report = {
        "safety": {
            "nonzero_drive_sent": False,
            "expected_arm_status": "ARM_REJECTED_LOCK (0x02)",
        },
        "host": args.host,
        "port": args.port,
        "count": args.count,
        "passed": sum(item["pass"] for item in results),
        "ready_successes": sum(item["ready"] for item in results),
        "arm_ack_successes": sum(item["arm_rejected_lock"] for item in results),
        "reconnect_successes": sum(item["reconnect_ready"] for item in results),
        "timeouts": sum(item.get("failure_reason") == "timeout" for item in results),
        "partial_receives": sum(item.get("partial_bytes", 0) > 0 for item in results),
        "mean_ready_s": metric(ready_times, statistics.fmean),
        "max_ready_s": metric(ready_times, max),
        "p95_ready_s": percentile_95(ready_times),
        "mean_arm_ack_s": metric(ack_times, statistics.fmean),
        "max_arm_ack_s": metric(ack_times, max),
        "p95_arm_ack_s": percentile_95(ack_times),
        "mean_reconnect_s": metric(reconnect_times, statistics.fmean),
        "max_reconnect_s": metric(reconnect_times, max),
        "p95_reconnect_s": percentile_95(reconnect_times),
        "results": results,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                key: report[key]
                for key in (
                    "count",
                    "passed",
                    "ready_successes",
                    "arm_ack_successes",
                    "reconnect_successes",
                    "timeouts",
                    "partial_receives",
                )
            }
        )
    )
    raise SystemExit(0 if report["passed"] == report["count"] else 1)


if __name__ == "__main__":
    main()
