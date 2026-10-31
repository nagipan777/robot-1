#!/usr/bin/env python3
"""Offline odometry/ArUco relative-trajectory synchronization and metrics."""

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

try:
    from localization_core import Pose2D, compose, inverse, normalize_angle
except ModuleNotFoundError:
    from .localization_core import Pose2D, compose, inverse, normalize_angle


@dataclass(frozen=True)
class TimedPose:
    timestamp: float
    pose: Pose2D


def load_pose_csv(path, *, timestamp="timestamp_s", x="x_m", y="y_m", yaw="yaw_rad"):
    poses = []
    with Path(path).open(newline="", encoding="utf-8") as source:
        for line_number, row in enumerate(csv.DictReader(source), start=2):
            if str(row.get("valid", "true")).lower() in ("false", "0", "no"):
                continue
            try:
                pose = TimedPose(
                    float(row[timestamp]),
                    Pose2D(float(row[x]), float(row[y]), float(row[yaw])),
                )
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("line {}: invalid pose row: {}".format(line_number, error)) from error
            if poses and pose.timestamp <= poses[-1].timestamp:
                raise ValueError("line {}: timestamps must increase".format(line_number))
            poses.append(pose)
    if not poses:
        raise ValueError("pose CSV has no valid samples")
    return poses


def relative_trajectory(poses):
    origin_inverse = inverse(poses[0].pose)
    return [TimedPose(item.timestamp, compose(origin_inverse, item.pose)) for item in poses]


def interpolate(first, second, timestamp):
    if second.timestamp <= first.timestamp:
        raise ValueError("interpolation timestamps must increase")
    ratio = (timestamp - first.timestamp) / (second.timestamp - first.timestamp)
    yaw_delta = normalize_angle(second.pose.yaw - first.pose.yaw)
    return Pose2D(
        first.pose.x + ratio * (second.pose.x - first.pose.x),
        first.pose.y + ratio * (second.pose.y - first.pose.y),
        normalize_angle(first.pose.yaw + ratio * yaw_delta),
    )


def compare_trajectories(odometry, aruco, max_time_delta_s=0.15):
    if max_time_delta_s <= 0:
        raise ValueError("max_time_delta_s must be positive")
    odometry = relative_trajectory(odometry)
    aruco = relative_trajectory(aruco)
    times = [item.timestamp for item in aruco]
    rows = []
    unmatched = 0
    for odom in odometry:
        index = bisect.bisect_left(times, odom.timestamp)
        candidates = []
        if index < len(aruco):
            candidates.append(aruco[index])
        if index > 0:
            candidates.append(aruco[index - 1])
        nearest = min(candidates, key=lambda item: abs(item.timestamp - odom.timestamp))
        offset = nearest.timestamp - odom.timestamp
        if abs(offset) > max_time_delta_s:
            unmatched += 1
            continue
        if 0 < index < len(aruco):
            before, after = aruco[index - 1], aruco[index]
            if (
                odom.timestamp - before.timestamp <= max_time_delta_s
                and after.timestamp - odom.timestamp <= max_time_delta_s
            ):
                aruco_pose = interpolate(before, after, odom.timestamp)
            else:
                aruco_pose = nearest.pose
        else:
            aruco_pose = nearest.pose
        dx = odom.pose.x - aruco_pose.x
        dy = odom.pose.y - aruco_pose.y
        yaw_error = normalize_angle(odom.pose.yaw - aruco_pose.yaw)
        rows.append(
            {
                "timestamp_s": odom.timestamp,
                "odom_x_m": odom.pose.x,
                "odom_y_m": odom.pose.y,
                "odom_yaw_rad": odom.pose.yaw,
                "aruco_x_m": aruco_pose.x,
                "aruco_y_m": aruco_pose.y,
                "aruco_yaw_rad": aruco_pose.yaw,
                "x_error_m": dx,
                "y_error_m": dy,
                "position_error_m": math.hypot(dx, dy),
                "yaw_error_rad": yaw_error,
                "nearest_time_offset_s": offset,
            }
        )
    if not rows:
        raise ValueError("no synchronized pose pairs")
    position_squared = [row["position_error_m"] ** 2 for row in rows]
    yaw_squared = [row["yaw_error_rad"] ** 2 for row in rows]
    offsets = [abs(row["nearest_time_offset_s"]) for row in rows]
    return rows, {
        "comparison_type": "relative_trajectory",
        "matched_samples": len(rows),
        "unmatched_odometry_samples": unmatched,
        "position_rmse_m": math.sqrt(sum(position_squared) / len(rows)),
        "yaw_rmse_rad": math.sqrt(sum(yaw_squared) / len(rows)),
        "max_position_error_m": max(row["position_error_m"] for row in rows),
        "max_yaw_error_rad": max(abs(row["yaw_error_rad"]) for row in rows),
        "terminal_position_error_m": rows[-1]["position_error_m"],
        "terminal_yaw_error_rad": abs(rows[-1]["yaw_error_rad"]),
        "mean_abs_time_offset_s": sum(offsets) / len(offsets),
        "max_abs_time_offset_s": max(offsets),
    }


def write_results(rows, metrics, output_csv, output_json, metadata):
    fields = list(rows[0])
    with Path(output_csv).open("w", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    report = dict(metrics)
    report.update(metadata)
    Path(output_json).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def write_svg(rows, output_path):
    width, height, margin = 800, 500, 40
    points = [
        (row[key_x], row[key_y])
        for row in rows
        for key_x, key_y in (("odom_x_m", "odom_y_m"), ("aruco_x_m", "aruco_y_m"))
    ]
    min_x, max_x = min(x for x, _ in points), max(x for x, _ in points)
    min_y, max_y = min(y for _, y in points), max(y for _, y in points)
    span_x, span_y = max(max_x - min_x, 1e-6), max(max_y - min_y, 1e-6)
    def project(x, y):
        px = margin + (x - min_x) / span_x * (width - 2 * margin)
        py = height - margin - (y - min_y) / span_y * (height - 2 * margin)
        return "{:.2f},{:.2f}".format(px, py)
    odom_points = " ".join(project(row["odom_x_m"], row["odom_y_m"]) for row in rows)
    aruco_points = " ".join(project(row["aruco_x_m"], row["aruco_y_m"]) for row in rows)
    svg = """<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">
<rect width="100%" height="100%" fill="white"/>
<text x="40" y="24" font-family="sans-serif" font-size="16">Relative trajectory A/B (dry-run)</text>
<polyline points="{odom}" fill="none" stroke="#1565c0" stroke-width="3"/>
<polyline points="{aruco}" fill="none" stroke="#2e7d32" stroke-width="2" stroke-dasharray="6 4"/>
<text x="600" y="24" fill="#1565c0" font-family="sans-serif">odometry</text>
<text x="690" y="24" fill="#2e7d32" font-family="sans-serif">ArUco</text>
</svg>
""".format(w=width, h=height, odom=odom_points, aruco=aruco_points)
    Path(output_path).write_text(svg, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odometry", required=True)
    parser.add_argument("--aruco", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-svg", required=True)
    parser.add_argument("--max-time-delta", type=float, default=0.15)
    parser.add_argument("--data-kind", choices=("synthetic", "measured"), required=True)
    args = parser.parse_args()
    odometry = load_pose_csv(
        args.odometry, x="odom_x_m", y="odom_y_m", yaw="odom_yaw_rad"
    )
    aruco = load_pose_csv(args.aruco)
    rows, metrics = compare_trajectories(odometry, aruco, args.max_time_delta)
    write_results(rows, metrics, args.output_csv, args.output_json, {"data_kind": args.data_kind})
    write_svg(rows, args.output_svg)
    print(json.dumps(dict(metrics, data_kind=args.data_kind), sort_keys=True))


if __name__ == "__main__":
    main()
