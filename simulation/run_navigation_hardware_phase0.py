#!/usr/bin/env python3
"""Generate the non-driving Navigation hardware-unit test matrix."""

import argparse
import csv
import json
import math
from pathlib import Path

try:
    from navigation_hardware_unit_core import evaluate_navigation_case
except ModuleNotFoundError:
    from .navigation_hardware_unit_core import evaluate_navigation_case


CASES = (
    ("forward", dict(target=(1.0, 0.0), mode="approach")),
    ("left_90", dict(target=(0.0, 1.0), mode="turn_only")),
    ("right_90", dict(target=(0.0, -1.0), mode="turn_only")),
    ("left_45", dict(target=(1.0, 1.0), mode="turn_only")),
    ("right_45", dict(target=(1.0, -1.0), mode="turn_only")),
    ("reached", dict(target=(0.10, 0.0), mode="approach")),
    ("pose_stale", dict(target=(1.0, 0.0), pose_fresh=False)),
    ("target_stale", dict(target=(1.0, 0.0), target_fresh=False)),
    ("old_target_after_stop", dict(
        target=(1.0, 0.0), stopped_target_latched=True
    )),
)


def run(output_json, output_csv):
    results = [evaluate_navigation_case(name, **parameters) for name, parameters in CASES]
    rows = [result.row() for result in results]
    output_json = Path(output_json)
    output_csv = Path(output_csv)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "data_kind": "non_driving_calculation",
        "physical_hardware_used": False,
        "arm_sent": False,
        "drive_sent": False,
        "parameters": {
            "angular_gain": 1.2,
            "max_angular_speed_rad_s": 0.8,
            "angle_tolerance_rad": 0.08,
            "arrival_distance_m": 0.15,
            "linear_gain": 0.4,
            "max_linear_speed_m_s": 0.08,
            "forward_angle_tolerance_rad": 0.15,
            "track_width_m": 0.235,
            "max_wheel_speed_mm_s": 20,
        },
        "results": rows,
        "passed": verify(results),
    }
    output_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    with output_csv.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    if not payload["passed"]:
        raise RuntimeError("Navigation phase-0 matrix did not match expectations")
    return payload


def verify(results):
    by_case = {result.case: result for result in results}
    return all((
        (by_case["forward"].left_mm_s, by_case["forward"].right_mm_s) == (20, 20),
        (by_case["left_90"].left_mm_s, by_case["left_90"].right_mm_s) == (-20, 20),
        (by_case["right_90"].left_mm_s, by_case["right_90"].right_mm_s) == (20, -20),
        (by_case["left_45"].left_mm_s, by_case["left_45"].right_mm_s) == (-20, 20),
        (by_case["right_45"].left_mm_s, by_case["right_45"].right_mm_s) == (20, -20),
        all(by_case[name].stop for name in (
            "reached", "pose_stale", "target_stale", "old_target_after_stop"
        )),
        math.isclose(by_case["left_90"].angular_z_rad_s, 0.8),
        math.isclose(by_case["right_90"].angular_z_rad_s, -0.8),
    ))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-csv", required=True)
    args = parser.parse_args()
    result = run(args.output_json, args.output_csv)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
