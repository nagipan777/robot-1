#!/usr/bin/env python3
"""Hardware-free evidence run for the bounded Drive deadline."""

import argparse
import json
import pathlib
import struct
import time

from roomba_tcp_transport import (
    DRIVE_ACK_COMPLETE,
    DIAGNOSTIC_MAGIC,
    DriveDeadlineExpired,
    DriveSessionLimiter,
    RoombaTcpTransport,
    SEQUENCED_DRIVE_RESPONSE,
    with_crc,
)


class MockSocket:
    def __init__(self):
        self.sent = []
        self.response = bytearray()

    def sendall(self, packet):
        self.sent.append(bytes(packet))
        generation, sequence = struct.unpack(">II", packet[2:10])
        body = (
            bytes([DIAGNOSTIC_MAGIC, SEQUENCED_DRIVE_RESPONSE, DRIVE_ACK_COMPLETE])
            + struct.pack(">II", generation, sequence)
        )
        self.response.extend(with_crc(body))

    def recv(self, size):
        result = bytes(self.response[:size])
        del self.response[:size]
        return result


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    events = []

    def trace(stage, **fields):
        events.append({"stage": stage, "monotonic": time.monotonic(), **fields})

    transport = RoombaTcpTransport("mock-only", 0, trace=trace)
    mock_socket = MockSocket()
    transport.socket = mock_socket
    transport.armed_generation = 1
    limiter = DriveSessionLimiter(0.4, time.monotonic)
    limiter.arm()
    started = time.monotonic()
    if limiter.observe_nonzero() != "started":
        raise RuntimeError("deadline did not start")
    deadline = limiter.deadline
    sequence = transport.send_twist(
        0.02,
        0.0,
        track_width_m=0.235,
        max_speed_mm_s=20,
        deadline_monotonic=deadline,
    )
    events.append(
        {
            "stage": "dry_run_drive_ack",
            "monotonic": time.monotonic(),
            "sequence": sequence,
            "right_mm_s": 20,
            "left_mm_s": 20,
        }
    )

    # Deliberately model a blocked executor beyond the deadline.
    time.sleep(max(0.0, deadline - time.monotonic()) + 0.03)
    detected = time.monotonic()
    permitted = limiter.permits_dispatch()
    sent_before = len(mock_socket.sent)
    transport.armed_generation = 1
    transport_deadline_rejected = False
    try:
        transport.send_twist(
            0.02,
            0.0,
            track_width_m=0.235,
            max_speed_mm_s=20,
            deadline_monotonic=deadline,
        )
    except DriveDeadlineExpired:
        transport_deadline_rejected = True
    sent_after = len(mock_socket.sent)
    events.append(
        {
            "stage": "deadline_detected_after_executor_delay",
            "monotonic": detected,
            "deadline_monotonic": deadline,
            "detection_late_s": detected - deadline,
            "limiter_permitted": permitted,
            "transport_deadline_rejected": transport_deadline_rejected,
            "post_deadline_socket_sends": sent_after - sent_before,
        }
    )

    passed = (
        sequence == 1
        and not permitted
        and transport_deadline_rejected
        and sent_after == sent_before
        and limiter.latched
    )
    summary = {
        "pass": passed,
        "hardware_connected": False,
        "arm_sent": False,
        "nonzero_drive_sent_to_hardware": False,
        "duration_limit_s": 0.4,
        "started_monotonic": started,
        "deadline_monotonic": deadline,
        "detected_monotonic": detected,
        "detection_late_s": detected - deadline,
        "post_deadline_socket_sends": sent_after - sent_before,
    }
    with (output_dir / "events.jsonl").open("w") as stream:
        for event in events:
            stream.write(json.dumps(event, sort_keys=True) + "\n")
    write_json(output_dir / "summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
