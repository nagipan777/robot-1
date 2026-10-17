"""Calibrate the Gazebo fixed camera without touching real-camera settings."""

import argparse
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_POINTS = ROOT / "simulation/sim_camera_correspondences.json"
DEFAULT_OUTPUT = ROOT / "simulation/vision_controller.sim_camera.yaml"


def parse_correspondence(value):
    try:
        numbers = [float(item) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("use px,py,x,y") from exc
    if len(numbers) != 4 or not all(math.isfinite(item) for item in numbers):
        raise argparse.ArgumentTypeError("use four finite values: px,py,x,y")
    return numbers


def compute_homography(points):
    if len(points) < 4:
        raise ValueError("at least four calibration points are required")
    pixels = np.asarray([[p[0], p[1]] for p in points], dtype=np.float64)
    world = np.asarray([[p[2], p[3]] for p in points], dtype=np.float64)
    homography, _ = cv2.findHomography(pixels, world, method=0)
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("could not compute a finite homography")
    homography /= homography[2, 2]
    return homography


def transform_points(points, homography):
    pixels = np.asarray([[p[0], p[1]] for p in points], dtype=np.float64)
    homogeneous = np.column_stack((pixels, np.ones(len(pixels))))
    projected = (homography @ homogeneous.T).T
    return projected[:, :2] / projected[:, 2, None]


def errors_for(points, homography):
    projected = transform_points(points, homography)
    expected = np.asarray([[p[2], p[3]] for p in points], dtype=np.float64)
    return projected, np.linalg.norm(projected - expected, axis=1)


def summarize_errors(errors):
    return {
        "rmse_m": float(np.sqrt(np.mean(np.square(errors)))),
        "mean_m": float(np.mean(errors)),
        "max_m": float(np.max(errors)),
    }


def format_homography(homography):
    values = []
    for value in homography.reshape(-1):
        rendered = f"{value:.12g}"
        if "." not in rendered and "e" not in rendered.lower():
            rendered += ".0"
        values.append(rendered)
    return ", ".join(values)


def write_sim_config(path, homography, width, height):
    config = path.read_text()
    config, count = re.subn(
        r"image_to_odom_homography:\s*\[[^\]]*\]",
        f"image_to_odom_homography: [{format_homography(homography)}]",
        config,
    )
    if count != 1:
        raise ValueError(f"could not uniquely locate homography in {path}")
    config = re.sub(r"(control_enabled:\s*)\w+", r"\1false", config)
    config = re.sub(r"(calibration_ready:\s*)\w+", r"\1false", config)
    config = re.sub(r"(calibration_image_width:\s*)\d+", rf"\g<1>{width}", config)
    config = re.sub(r"(calibration_image_height:\s*)\d+", rf"\g<1>{height}", config)
    path.write_text(config)


def load_points(path):
    payload = json.loads(path.read_text())
    return (
        payload["image_width"],
        payload["image_height"],
        payload["calibration_points"],
        payload["validation_points"],
    )


def main():
    parser = argparse.ArgumentParser(
        description="Validate and save the Gazebo fixed-camera homography"
    )
    parser.add_argument("--points", type=Path, default=DEFAULT_POINTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--point", action="append", type=parse_correspondence)
    parser.add_argument(
        "--validation-point", action="append", type=parse_correspondence
    )
    parser.add_argument("--max-error", type=float, default=0.05)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    args = parser.parse_args()

    if args.point:
        calibration = args.point
        validation = args.validation_point or []
        width, height = args.image_width, args.image_height
    else:
        width, height, calibration, validation = load_points(args.points)

    if len(calibration) < 4:
        parser.error("four or more calibration points are required")
    if not validation:
        parser.error("one or more independent validation points are required")
    if not math.isfinite(args.max_error) or args.max_error <= 0:
        parser.error("--max-error must be a positive finite value")

    homography = compute_homography(calibration)
    calibration_projected, calibration_errors = errors_for(calibration, homography)
    validation_projected, validation_errors = errors_for(validation, homography)
    calibration_summary = summarize_errors(calibration_errors)
    validation_summary = summarize_errors(validation_errors)

    print(f"homography: [{format_homography(homography)}]")
    for index, (point, actual, error) in enumerate(
        zip(validation, validation_projected, validation_errors), start=1
    ):
        print(
            f"validation {index}: pixel=({point[0]:.2f}, {point[1]:.2f}) "
            f"expected=({point[2]:.3f}, {point[3]:.3f}) "
            f"actual=({actual[0]:.3f}, {actual[1]:.3f}) error={error:.4f} m"
        )
    print(
        "calibration: "
        f"RMSE={calibration_summary['rmse_m']:.4f} m, "
        f"max={calibration_summary['max_m']:.4f} m"
    )
    print(
        "validation: "
        f"RMSE={validation_summary['rmse_m']:.4f} m, "
        f"max={validation_summary['max_m']:.4f} m"
    )

    if validation_summary["max_m"] > args.max_error:
        raise SystemExit(
            f"validation failed: {validation_summary['max_m']:.4f} m exceeds "
            f"{args.max_error:.4f} m; config was not changed"
        )

    write_sim_config(args.output, homography, width, height)
    print(f"saved: {args.output}")
    print("control_enabled=false and calibration_ready=false were preserved")


if __name__ == "__main__":
    main()
