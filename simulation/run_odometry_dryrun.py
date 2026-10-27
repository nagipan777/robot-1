#!/usr/bin/env python3
"""One-command non-hardware odometry replay and synthetic ArUco A/B report."""

import argparse
import csv
import json
from pathlib import Path

try:
    from localization_core import Pose2D, WheelOdometry, compose
    from odometry_aruco_ab import (
        TimedPose,
        compare_trajectories,
        write_results,
        write_svg,
    )
    from odometry_dryrun_core import DryRunOdometry, load_encoder_csv
except ModuleNotFoundError:
    from .localization_core import Pose2D, WheelOdometry, compose
    from .odometry_aruco_ab import (
        TimedPose,
        compare_trajectories,
        write_results,
        write_svg,
    )
    from .odometry_dryrun_core import DryRunOdometry, load_encoder_csv


ODOM_FIELDS = (
    "timestamp_s", "left_ticks", "right_ticks", "left_delta", "right_delta",
    "valid", "communication_state", "error", "integrated", "odom_x_m",
    "odom_y_m", "odom_yaw_rad", "linear_m_s", "angular_rad_s",
)


def run(input_path, output_directory):
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    estimator = WheelOdometry(508.8, 0.072, 0.235, max_tick_delta=2000)
    processor = DryRunOdometry(estimator, max_time_gap_s=1.0)
    rows = []
    poses = []
    for sample in load_encoder_csv(input_path):
        try:
            processed = processor.process(sample)
            update = processed.update
            pose = estimator.pose if update is None else update.pose
            row = {
                "timestamp_s": sample.timestamp,
                "left_ticks": "" if sample.left_encoder is None else sample.left_encoder,
                "right_ticks": "" if sample.right_encoder is None else sample.right_encoder,
                "left_delta": "" if update is None else update.left_delta,
                "right_delta": "" if update is None else update.right_delta,
                "valid": sample.valid and update is not None,
                "communication_state": sample.communication_state,
                "error": sample.error,
                "integrated": processed.integrated,
                "odom_x_m": pose.x,
                "odom_y_m": pose.y,
                "odom_yaw_rad": pose.yaw,
                "linear_m_s": "" if update is None else update.linear_velocity,
                "angular_rad_s": "" if update is None else update.angular_velocity,
            }
        except ValueError as error:
            estimator.resynchronize()
            processor.connected = False
            row = {
                "timestamp_s": sample.timestamp,
                "left_ticks": sample.left_encoder,
                "right_ticks": sample.right_encoder,
                "left_delta": "",
                "right_delta": "",
                "valid": False,
                "communication_state": sample.communication_state,
                "error": str(error),
                "integrated": False,
                "odom_x_m": estimator.pose.x,
                "odom_y_m": estimator.pose.y,
                "odom_yaw_rad": estimator.pose.yaw,
                "linear_m_s": "",
                "angular_rad_s": "",
            }
        rows.append(row)
        if row["valid"]:
            poses.append(
                TimedPose(
                    sample.timestamp,
                    Pose2D(row["odom_x_m"], row["odom_y_m"], row["odom_yaw_rad"]),
                )
            )
    odom_path = output_directory / "odometry.csv"
    with odom_path.open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=ODOM_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    # Synthetic external poses deliberately start in a non-zero map pose. The
    # A/B tool must remove this map origin and heading before comparison.
    map_origin = Pose2D(1.179, 0.048, 0.12)
    aruco_path = output_directory / "aruco_synthetic.csv"
    with aruco_path.open("w", newline="", encoding="utf-8") as target:
        fields = ("timestamp_s", "x_m", "y_m", "yaw_rad", "valid", "source")
        writer = csv.DictWriter(target, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for item in poses:
            mapped = compose(map_origin, item.pose)
            writer.writerow(
                {
                    "timestamp_s": item.timestamp,
                    "x_m": mapped.x,
                    "y_m": mapped.y,
                    "yaw_rad": mapped.yaw,
                    "valid": True,
                    "source": "synthetic",
                }
            )

    aruco_poses = [TimedPose(item.timestamp, compose(map_origin, item.pose)) for item in poses]
    comparison_rows, metrics = compare_trajectories(poses, aruco_poses)
    comparison_csv = output_directory / "comparison.csv"
    comparison_json = output_directory / "summary.json"
    comparison_svg = output_directory / "trajectory.svg"
    metadata = {
        "data_kind": "synthetic",
        "input": str(input_path),
        "real_uart_used": False,
        "nonzero_drive_sent": False,
    }
    write_results(comparison_rows, metrics, comparison_csv, comparison_json, metadata)
    write_svg(comparison_rows, comparison_svg)
    return dict(metadata, **metrics)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    result = run(args.input, args.output_dir)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
