#!/usr/bin/env python3
"""STOP-first static Roomba encoder probe; never sends ARM or Drive."""

import argparse
import csv
import time
from pathlib import Path

try:
    from .roomba_tcp_transport import RoombaTcpTransport
except ImportError:
    from roomba_tcp_transport import RoombaTcpTransport


FIELDS = (
    "wall_time_ns",
    "monotonic_ns",
    "left_encoder",
    "right_encoder",
    "valid",
    "communication_state",
    "sequence",
    "error",
)


def run(host, port, count, interval_s, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    transport = RoombaTcpTransport(host, port)
    rows = []
    try:
        transport.connect()
        transport.verify_stop()
        mode = transport.query_oi_mode()
        if mode != 2:
            raise RuntimeError("Roomba OI mode is not Safe: {}".format(mode))
        for sequence in range(1, count + 1):
            wall_time_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            try:
                left, right = transport.query_encoders()
                row = {
                    "wall_time_ns": wall_time_ns,
                    "monotonic_ns": monotonic_ns,
                    "left_encoder": left,
                    "right_encoder": right,
                    "valid": "true",
                    "communication_state": "connected",
                    "sequence": sequence,
                    "error": "",
                }
                print(
                    "encoder sequence={} left={} right={}".format(
                        sequence, left, right
                    )
                )
            except Exception as error:
                row = {
                    "wall_time_ns": wall_time_ns,
                    "monotonic_ns": monotonic_ns,
                    "left_encoder": "",
                    "right_encoder": "",
                    "valid": "false",
                    "communication_state": "error",
                    "sequence": sequence,
                    "error": str(error),
                }
                rows.append(row)
                raise
            rows.append(row)
            if sequence < count:
                time.sleep(interval_s)
    finally:
        transport.close()
        with output_path.open("w", newline="", encoding="utf-8") as target:
            writer = csv.DictWriter(target, fieldnames=FIELDS, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    if args.interval < 0.0:
        parser.error("--interval must be non-negative")
    run(args.host, args.port, args.count, args.interval, args.output)


if __name__ == "__main__":
    main()
