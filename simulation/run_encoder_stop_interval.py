#!/usr/bin/env python3
"""Offline workflow for encoder measurements around a separately run Drive."""

import argparse
import csv
import json
from pathlib import Path

try:
    from encoder_stop_interval_core import (
        ATTENDED_DRIVE_APPROVAL,
        PHYSICAL_STOP_CONFIRMATION,
        evaluate_interval,
        require_after_gate,
        require_drive_approval,
    )
    from odometry_dryrun_core import load_encoder_csv
except ModuleNotFoundError:
    from .encoder_stop_interval_core import (
        ATTENDED_DRIVE_APPROVAL,
        PHYSICAL_STOP_CONFIRMATION,
        evaluate_interval,
        require_after_gate,
        require_drive_approval,
    )
    from .odometry_dryrun_core import load_encoder_csv


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def stable_endpoint(csv_path, which):
    samples = load_encoder_csv(csv_path)
    if not all(sample.valid and sample.communication_state == "connected" for sample in samples):
        raise RuntimeError("{} acquisition contains invalid samples".format(which))
    sample = samples[-1] if which == "before" else samples[0]
    return sample.left_encoder, sample.right_encoder, sample.timestamp


def prepare(args):
    left, right, timestamp = stable_endpoint(args.before_csv, "before")
    write_json(args.session, {
        "schema": "roomba_encoder_stop_interval_v1",
        "state": "before_acquired",
        "drive_approved": False,
        "before_csv": str(Path(args.before_csv).resolve()),
        "before_left_u16": left,
        "before_right_u16": right,
        "before_timestamp": timestamp,
        "motion_label": args.motion_label,
    })


def authorize(args):
    session = read_json(args.session)
    require_drive_approval(session, args.confirmation)
    write_json(args.session, session)


def finalize(args):
    session = read_json(args.session)
    require_after_gate(session, args.confirmation)
    left, right, timestamp = stable_endpoint(args.after_csv, "after")
    result = evaluate_interval(
        session["before_left_u16"], session["before_right_u16"], left, right,
        encoder_counts_per_rev=args.encoder_counts_per_rev,
        wheel_diameter_m=args.wheel_diameter_m,
        wheel_separation_m=args.wheel_separation_m,
        max_tick_delta=args.max_tick_delta,
        measured_distance_m=args.measured_distance_m,
        measured_yaw_rad=args.measured_yaw_rad,
        expected_left_sign=args.expected_left_sign,
        expected_right_sign=args.expected_right_sign,
    )
    row = result.row()
    row.update({
        "motion_label": session.get("motion_label", ""),
        "before_timestamp": session["before_timestamp"],
        "after_timestamp": timestamp,
        "before_csv": session["before_csv"],
        "after_csv": str(Path(args.after_csv).resolve()),
    })
    output = Path(args.output_csv)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=row.keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerow(row)
    session.update({"state": "evaluated", "after_csv": row["after_csv"], "result": row})
    write_json(args.summary, session)


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    before = commands.add_parser("prepare")
    before.add_argument("--before-csv", required=True)
    before.add_argument("--session", required=True)
    before.add_argument("--motion-label", required=True)
    before.set_defaults(run=prepare)
    approval = commands.add_parser("authorize")
    approval.add_argument("--session", required=True)
    approval.add_argument("--confirmation", required=True, choices=[ATTENDED_DRIVE_APPROVAL])
    approval.set_defaults(run=authorize)
    after = commands.add_parser("finalize")
    after.add_argument("--session", required=True)
    after.add_argument("--after-csv", required=True)
    after.add_argument("--output-csv", required=True)
    after.add_argument("--summary", required=True)
    after.add_argument("--confirmation", required=True, choices=[PHYSICAL_STOP_CONFIRMATION])
    after.add_argument("--encoder-counts-per-rev", type=float, default=508.8)
    after.add_argument("--wheel-diameter-m", type=float, default=0.072)
    after.add_argument("--wheel-separation-m", type=float, default=0.235)
    after.add_argument("--max-tick-delta", type=int, default=2000)
    after.add_argument("--measured-distance-m", type=float)
    after.add_argument("--measured-yaw-rad", type=float)
    after.add_argument("--expected-left-sign", type=int, choices=[-1, 0, 1])
    after.add_argument("--expected-right-sign", type=int, choices=[-1, 0, 1])
    after.set_defaults(run=finalize)
    return root


def main():
    args = parser().parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
