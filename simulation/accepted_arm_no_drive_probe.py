#!/usr/bin/env python3
"""Verify one accepted ARM round trip, immediate STOP, and stale rejection.

This diagnostic never constructs or sends a Drive frame.
"""

import argparse
import json
import time

try:
    from .roomba_tcp_transport import RoombaDiagnosticError, RoombaTcpTransport
except ImportError:
    from roomba_tcp_transport import RoombaDiagnosticError, RoombaTcpTransport


def run_probe(transport, generation):
    result = {
        "generation": generation,
        "nonzero_drive_sent": False,
        "arm_ack_accepted": False,
        "stop_ack": None,
        "requested_right": None,
        "requested_left": None,
        "oi_mode": None,
        "stale_generation_rejected": False,
    }
    transport.connect()
    try:
        transport.arm(generation)
        result["arm_ack_accepted"] = True

        # Legacy zero STOP revokes the accepted generation on both endpoints.
        transport.send_stop()
        status, right, left = transport.verify_stop()
        result.update(
            stop_ack=status,
            requested_right=right,
            requested_left=left,
            oi_mode=transport.query_oi_mode(),
        )

        try:
            transport.arm(generation)
        except RoombaDiagnosticError:
            result["stale_generation_rejected"] = True
        else:
            raise RuntimeError("stale ARM generation was unexpectedly accepted")

        transport.send_stop()
        result["pass"] = (
            result["arm_ack_accepted"]
            and result["stop_ack"] == 0x07
            and result["requested_right"] == 0
            and result["requested_left"] == 0
            and result["oi_mode"] == 2
            and result["stale_generation_rejected"]
            and not result["nonzero_drive_sent"]
        )
        return result
    finally:
        # Failure paths also request zero before closing. close() repeats STOP.
        try:
            transport.send_stop()
        except (OSError, ConnectionError):
            pass
        transport.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--generation", type=int)
    args = parser.parse_args()
    generation = args.generation or (int(time.time()) & 0xFFFFFFFF)
    transport = RoombaTcpTransport(args.host, args.port)
    result = run_probe(transport, generation)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
