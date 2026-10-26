#!/usr/bin/env python3
"""Probe locked ARM ACK and reconnect without sending any Drive frame."""

import argparse
import json
import socket
import struct
import time
from pathlib import Path


READY = b"READY\n"
STOP = bytes([145, 0, 0, 0, 0])
ARM = bytes([0xD7, 0x04])
ARM_RESPONSE_PREFIX = bytes([0xD7, 0x84])
ARM_REJECTED_LOCK = 0x02


def receive_exact(connection, size):
    received = bytearray()
    while len(received) < size:
        chunk = connection.recv(size - len(received))
        if not chunk:
            raise ConnectionError(
                "connection closed after {}/{} bytes".format(len(received), size)
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


def run_cycle(host, port, timeout, generation):
    result = {
        "generation": generation,
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
        first.sendall(STOP)
        first.close()
        first = None

        second, result["reconnect_ready_s"] = connect_ready(host, port, timeout)
        result["reconnect_ready"] = True
        second.sendall(STOP)
    except Exception as error:
        result["error"] = "{}: {}".format(type(error).__name__, error)
    finally:
        for connection in (first, second):
            if connection is not None:
                try:
                    connection.sendall(STOP)
                except OSError:
                    pass
                connection.close()
    result["pass"] = (
        result["ready"]
        and result["arm_rejected_lock"]
        and result["reconnect_ready"]
    )
    return result


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
            args.host, args.port, args.timeout, base_generation + index
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

    report = {
        "safety": {
            "nonzero_drive_sent": False,
            "expected_arm_status": "ARM_REJECTED_LOCK (0x02)",
        },
        "host": args.host,
        "port": args.port,
        "count": args.count,
        "passed": sum(item["pass"] for item in results),
        "timeouts": sum("timeout" in (item["error"] or "").lower() for item in results),
        "results": results,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("count", "passed", "timeouts")}))
    raise SystemExit(0 if report["passed"] == report["count"] else 1)


if __name__ == "__main__":
    main()
