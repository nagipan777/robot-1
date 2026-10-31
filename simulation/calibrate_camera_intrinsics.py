#!/usr/bin/env python3
"""Estimate camera intrinsics from checkerboard images and save JSON evidence."""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", nargs="+", help="Glob(s) for calibration images")
    parser.add_argument("output", type=Path)
    parser.add_argument("--columns", type=int, default=7, help="Inner corners across")
    parser.add_argument("--rows", type=int, default=10, help="Inner corners down")
    parser.add_argument("--square-mm", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pattern_size = (args.columns, args.rows)
    object_template = np.zeros((args.columns * args.rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[
        0 : args.columns, 0 : args.rows
    ].T.reshape(-1, 2)
    object_template *= args.square_mm / 1000.0

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used: list[str] = []
    rejected: list[str] = []
    image_size: tuple[int, int] | None = None
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    filenames = sorted(
        {filename for pattern in args.images for filename in glob.glob(pattern)}
    )
    for filename in filenames:
        image = cv2.imread(filename)
        if image is None:
            rejected.append(filename)
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is not None and current_size != image_size:
            raise ValueError(f"mixed image sizes: {current_size} != {image_size}")
        image_size = current_size
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found:
            rejected.append(filename)
            continue
        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        object_points.append(object_template.copy())
        image_points.append(refined)
        used.append(filename)

    if image_size is None or len(used) < 6:
        raise RuntimeError(f"insufficient usable views: {len(used)} (need at least 6)")

    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    view_errors = []
    for filename, object_view, image_view, rotation, translation in zip(
        used, object_points, image_points, rotations, translations
    ):
        projected, _ = cv2.projectPoints(
            object_view, rotation, translation, matrix, distortion
        )
        observed_xy = image_view.reshape(-1, 2)
        projected_xy = projected.reshape(-1, 2)
        error = (
            cv2.norm(observed_xy, projected_xy, cv2.NORM_L2)
            / len(projected_xy) ** 0.5
        )
        view_errors.append({"image": filename, "rms_px": float(error)})

    result = {
        "model": "opencv_pinhole_radial_tangential",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "checkerboard_inner_corners": [args.columns, args.rows],
        "square_size_m": args.square_mm / 1000.0,
        "usable_view_count": len(used),
        "rejected_view_count": len(rejected),
        "rms_reprojection_error_px": float(rms),
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "per_view_errors": view_errors,
        "used_images": used,
        "rejected_images": rejected,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
