#!/usr/bin/env python3
"""Generate deterministic non-hardware evidence for wheel motion monitoring."""

import argparse
import csv
import json
from pathlib import Path

from odometry_dryrun_core import EncoderSample
from wheel_motion_monitor import MonitorEvent, WheelMotionMonitor


def encoder(at, left, right, sequence, valid=True, state="connected", error=""):
    return EncoderSample(at, left, right, valid, state, sequence, error)


def run(output_directory):
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    events = []

    def monitor():
        return WheelMotionMonitor(
            command_no_motion_grace_s=0.20,
            stop_motion_grace_s=0.15,
            measurement_stale_s=0.50,
            max_tick_delta=100,
            started_at_s=0.0,
        )

    straight = monitor()
    events += [straight.observe_arm_request(10, 1.0), straight.observe_encoder(encoder(1.0, 100, 100, 1), 1.0)]
    events += [straight.observe_command(20, 20, 1.01), straight.observe_encoder(encoder(1.10, 105, 105, 2), 1.10)]

    turn = monitor()
    events += [turn.observe_encoder(encoder(2.0, 200, 200, 1), 2.0), turn.observe_command(-20, 20, 2.01)]
    events += [turn.observe_encoder(encoder(2.10, 195, 205, 2), 2.10)]

    stopped = monitor()
    events += [stopped.observe_encoder(encoder(3.0, 300, 300, 1), 3.0), stopped.observe_command(20, 20, 3.01)]
    events += [stopped.observe_encoder(encoder(3.10, 302, 302, 2), 3.10), stopped.observe_command(0, 0, 3.20)]
    events += [stopped.observe_encoder(encoder(3.30, 303, 303, 3), 3.30), stopped.observe_encoder(encoder(3.50, 304, 304, 4), 3.50)]

    no_change = monitor()
    events += [no_change.observe_encoder(encoder(4.0, 400, 400, 1), 4.0), no_change.observe_command(20, 20, 4.01)]
    events += [no_change.observe_encoder(encoder(4.30, 400, 400, 2), 4.30)]

    missing = monitor()
    events.append(missing.poll(0.60))

    wrapped = monitor()
    events += [wrapped.observe_encoder(encoder(5.0, 65534, 65534, 1), 5.0)]
    events += [wrapped.observe_encoder(encoder(5.10, 2, 2, 2), 5.10)]

    reconnect = monitor()
    events += [reconnect.observe_encoder(encoder(6.0, 10, 10, 1), 6.0)]
    events += [reconnect.observe_encoder(encoder(6.10, None, None, 2, False, "timeout", "rx_timeout"), 6.10)]
    events += [reconnect.observe_encoder(encoder(6.20, 50000, 50000, 3), 6.20)]

    path = output / "wheel_monitor_events.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(MonitorEvent.__dataclass_fields__),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(event.row() for event in events if event is not None)
    names = [event.event for event in events if event is not None]
    required = {
        "encoder_update", "stop_coast_observed", "wheel_change_after_stop",
        "command_without_encoder_change", "measurement_stale",
        "measurement_unavailable", "encoder_baseline",
    }
    result = {
        "pass": required.issubset(names),
        "data_kind": "synthetic",
        "real_uart_used": False,
        "nonzero_drive_sent": False,
        "automatic_stop_connected": False,
        "navigation_connected": False,
        "event_count": len(names),
        "events_observed": sorted(set(names)),
    }
    (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps(result, sort_keys=True))
    raise SystemExit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
