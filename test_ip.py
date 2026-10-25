#!/usr/bin/env python3
"""Pico W TCP connection test, with an optional short drive test."""

import argparse
import socket
import struct
import sys
import time


DEFAULT_HOST = "192.168.0.32"
DEFAULT_PORT = 9000
MAX_SPEED_MM_S = 50
DRIVE_DURATION_S = 1.0
DRIVE_DIRECT = 145


def drive_packet(right_mm_s, left_mm_s):
    return struct.pack(">Bhh", DRIVE_DIRECT, right_mm_s, left_mm_s)


def run(host, port, drive_test=False):
    print(f"connecting to {host}:{port} ...")
    try:
        connection = socket.create_connection((host, port), timeout=5.0)
    except OSError as error:
        print(f"connection failed: {error}", file=sys.stderr)
        return 1

    stop = drive_packet(0, 0)
    try:
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("connected")
        if not drive_test:
            print("connection test only; no drive command was sent")
            return 0

        time.sleep(0.6)
        forward = drive_packet(MAX_SPEED_MM_S, MAX_SPEED_MM_S)
        print(f"forward {DRIVE_DURATION_S:.1f}s at {MAX_SPEED_MM_S} mm/s")
        deadline = time.monotonic() + DRIVE_DURATION_S
        while time.monotonic() < deadline:
            connection.sendall(forward)
            time.sleep(0.05)
        connection.sendall(stop)
        print("stop")
        return 0
    except OSError as error:
        print(f"communication failed: {error}", file=sys.stderr)
        return 1
    finally:
        if drive_test:
            try:
                connection.sendall(stop)
            except OSError:
                pass
        connection.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST, help="Pico W IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--drive-test",
        action="store_true",
        help="explicitly enable a 1 second, 50 mm/s forward test",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return run(args.host, args.port, args.drive_test)


if __name__ == "__main__":
    raise SystemExit(main())
