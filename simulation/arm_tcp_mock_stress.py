#!/usr/bin/env python3
"""Non-driving READY/ARM ACK/reconnect stress test using a local mock Pico."""

import argparse
import json
import socket
import statistics
import struct
import threading
import time
from pathlib import Path


READY = b"READY\n"
STOP = bytes([145, 0, 0, 0, 0])
ARM_PREFIX = bytes([0xD7, 0x04])
ACK_PREFIX = bytes([0xD7, 0x84, 0x02])  # locked/non-driving rejection


class ReceiveFailure(Exception):
    def __init__(self, kind, received, expected):
        super().__init__("{} after {}/{} bytes".format(kind, received, expected))
        self.kind = kind
        self.received = received


def receive_exact(connection, size):
    received = bytearray()
    started = time.monotonic()
    while len(received) < size:
        try:
            chunk = connection.recv(size - len(received))
        except socket.timeout as error:
            raise ReceiveFailure("timeout", len(received), size) from error
        if not chunk:
            raise ReceiveFailure("closed", len(received), size)
        received.extend(chunk)
    return bytes(received), time.monotonic() - started


def send_chunks(connection, payload, chunk_size):
    for offset in range(0, len(payload), chunk_size):
        connection.sendall(payload[offset : offset + chunk_size])


def server_session(connection, scenario, generation, timeout):
    connection.settimeout(timeout)
    send_chunks(connection, READY, 2 if scenario == "fragmented" else len(READY))
    receive_exact(connection, len(STOP))
    arm, _ = receive_exact(connection, 6)
    if arm != ARM_PREFIX + struct.pack(">I", generation):
        return
    ack = ACK_PREFIX + struct.pack(">I", generation)
    if scenario == "timeout":
        time.sleep(timeout * 1.5)
    elif scenario == "partial_close":
        connection.sendall(ack[:3])
    else:
        send_chunks(connection, ack, 1 if scenario == "fragmented" else len(ack))


def mock_server(listener, scenarios, generations, timeout, errors):
    try:
        for scenario, generation in zip(scenarios, generations):
            connection, _address = listener.accept()
            with connection:
                server_session(connection, scenario, generation, timeout)
    except Exception as error:  # recorded for the calling process
        errors.append("{}: {}".format(type(error).__name__, error))


def percentile_95(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, (len(ordered) * 95 + 99) // 100 - 1)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=300)
    parser.add_argument("--scenario", choices=("normal", "fragmented", "matrix"), default="normal")
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--environment", default="local-mock")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.cycles < 1 or args.timeout <= 0:
        parser.error("cycles and timeout must be positive")

    if args.scenario == "matrix":
        pattern = ("normal", "fragmented", "timeout", "partial_close")
        scenarios = [pattern[index % len(pattern)] for index in range(args.cycles)]
    else:
        scenarios = [args.scenario] * args.cycles
    base = int(time.time()) & 0xFFFFFFFF
    generations = [base + index for index in range(args.cycles)]

    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(8)
    host, port = listener.getsockname()
    server_errors = []
    thread = threading.Thread(
        target=mock_server,
        args=(listener, scenarios, generations, args.timeout, server_errors),
        daemon=True,
    )
    thread.start()

    results = []
    for index, (scenario, generation) in enumerate(zip(scenarios, generations), 1):
        item = {"cycle": index, "scenario": scenario, "generation": generation}
        connection = None
        started = time.monotonic()
        try:
            connect_started = time.monotonic()
            connection = socket.create_connection((host, port), timeout=args.timeout)
            item["connect_s"] = time.monotonic() - connect_started
            connection.settimeout(args.timeout)
            ready, ready_s = receive_exact(connection, len(READY))
            item.update(ready_bytes=len(ready), ready_s=ready_s)
            connection.sendall(STOP)
            connection.sendall(ARM_PREFIX + struct.pack(">I", generation))
            arm_started = time.monotonic()
            ack, arm_ack_s = receive_exact(connection, 7)
            item.update(ack_bytes=len(ack), arm_ack_s=arm_ack_s, ack_raw=ack.hex())
            item["success"] = ack == ACK_PREFIX + struct.pack(">I", generation)
        except ReceiveFailure as error:
            item.update(success=False, failure=error.kind, ack_bytes=error.received)
        except Exception as error:
            item.update(success=False, failure="{}: {}".format(type(error).__name__, error))
        finally:
            item["cycle_s"] = time.monotonic() - started
            if connection is not None:
                try:
                    connection.sendall(STOP)
                except OSError:
                    pass
                connection.close()
        results.append(item)

    thread.join(timeout=max(2.0, args.cycles * args.timeout * 2.0))
    listener.close()
    latencies = [item["arm_ack_s"] for item in results if item.get("success")]
    connect_times = [item["connect_s"] for item in results if "connect_s" in item]
    ready_times = [item["ready_s"] for item in results if item.get("ready_bytes") == 6]
    report = {
        "environment": args.environment,
        "safety": {"nonzero_drive_sent": False, "arm_status": "locked_rejection"},
        "scenario": args.scenario,
        "cycles": args.cycles,
        "successes": sum(bool(item.get("success")) for item in results),
        "connect_successes": len(connect_times),
        "ready_successes": len(ready_times),
        "timeouts": sum(item.get("failure") == "timeout" for item in results),
        "partial_receives": sum(0 < item.get("ack_bytes", 0) < 7 for item in results),
        "mean_arm_ack_s": statistics.fmean(latencies) if latencies else None,
        "max_arm_ack_s": max(latencies) if latencies else None,
        "p95_arm_ack_s": percentile_95(latencies),
        "mean_connect_s": statistics.fmean(connect_times) if connect_times else None,
        "max_connect_s": max(connect_times) if connect_times else None,
        "mean_ready_s": statistics.fmean(ready_times) if ready_times else None,
        "max_ready_s": max(ready_times) if ready_times else None,
        "server_errors": server_errors,
        "results": results,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "results"}, indent=2))
    if server_errors:
        raise SystemExit(1)
    if args.scenario != "matrix" and report["successes"] != args.cycles:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
