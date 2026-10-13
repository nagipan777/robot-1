"""Measure rendered calibration-marker pixels at known Gazebo coordinates."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
CALIBRATION_WORLD_POINTS = [(-2.0, -1.0), (2.0, -1.0), (2.0, 1.0), (-2.0, 1.0)]
VALIDATION_WORLD_POINTS = [(-1.2, 1.2), (1.25, 1.15), (0.65, -1.25)]


def find_marker_center(frame):
    blue, green, red = cv2.split(frame)
    mask = (
        (blue.astype(np.int16) - green.astype(np.int16) > 60)
        & (red.astype(np.int16) - green.astype(np.int16) > 60)
        & (blue > 120)
        & (red > 120)
    ).astype(np.uint8) * 255
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates = [index for index in range(1, count) if stats[index, cv2.CC_STAT_AREA] >= 20]
    if not candidates:
        raise ValueError("magenta calibration marker was not found")
    best = max(candidates, key=lambda index: stats[index, cv2.CC_STAT_AREA])
    return float(centroids[best][0]), float(centroids[best][1])


def move_marker(sim_script, x, y):
    subprocess.run(
        [str(sim_script), "calibration-marker-move", str(x), str(y)],
        cwd=ROOT,
        check=True,
    )


def capture_frame(url):
    capture = cv2.VideoCapture(url)
    try:
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"could not read {url}")
        return frame
    finally:
        capture.release()


def measure_points(points, args, label):
    measured = []
    for index, (x, y) in enumerate(points, start=1):
        move_marker(args.sim_script, x, y)
        time.sleep(args.settle_seconds)
        last_error = None
        for _attempt in range(12):
            frame = capture_frame(args.video_url)
            try:
                pixel_x, pixel_y = find_marker_center(frame)
                break
            except ValueError as exc:
                last_error = exc
                time.sleep(1.0)
        else:
            raise last_error
        measured.append([pixel_x, pixel_y, x, y])
        if args.capture_dir:
            args.capture_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.capture_dir / f"{label}_{index}.jpg"), frame)
        print(
            f"{label} {index}: world=({x:.3f}, {y:.3f}) "
            f"pixel=({pixel_x:.2f}, {pixel_y:.2f})"
        )
    return measured


def main():
    parser = argparse.ArgumentParser(description="Collect Gazebo camera calibration points")
    parser.add_argument("--video-url", default="http://127.0.0.1:8766/stream.mjpg")
    parser.add_argument(
        "--sim-script", type=Path, default=ROOT / "simulation/sim.sh"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "simulation/sim_camera_correspondences.json",
    )
    parser.add_argument(
        "--capture-dir", type=Path, default=ROOT / "artifacts/sim_calibration"
    )
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    args = parser.parse_args()

    try:
        calibration = measure_points(
            CALIBRATION_WORLD_POINTS, args, "calibration"
        )
        validation = measure_points(VALIDATION_WORLD_POINTS, args, "validation")
    finally:
        subprocess.run(
            [str(args.sim_script), "calibration-marker-remove"],
            cwd=ROOT,
            check=False,
        )

    payload = {
        "image_width": 640,
        "image_height": 480,
        "calibration_points": calibration,
        "validation_points": validation,
    }
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
