"""Build a scene-separated Gazebo litter pilot dataset."""

import argparse
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np

from diagnose_sim_litter import (
    ROOT,
    capture,
    camera_sequence,
    sim_command,
    wait_for_camera,
    wait_frames,
    wait_for_rendered_litter,
    world_to_pixel,
)


KINDS = ("can", "paper", "bottle")
SCENES = (
    {"id": "scene_00", "split": "test", "x": 1.5, "y": -0.7, "yaw": 0.0, "holdout": "far_right"},
    {"id": "scene_01", "split": "test", "x": -1.2, "y": 1.0, "yaw": 0.75, "holdout": "near_left"},
    {"id": "scene_02", "split": "test", "x": 0.8, "y": 1.4, "yaw": 1.57, "holdout": "upper_left_rotated"},
    {"id": "scene_03", "split": "val", "x": -1.8, "y": -1.2, "yaw": 0.25},
    {"id": "scene_04", "split": "val", "x": 1.8, "y": 1.0, "yaw": 0.9},
    {"id": "scene_05", "split": "train", "x": -0.7, "y": -1.5, "yaw": 0.1},
    {"id": "scene_06", "split": "train", "x": 1.2, "y": 0.6, "yaw": 0.65},
    {"id": "scene_07", "split": "train", "x": -1.7, "y": 0.2, "yaw": 1.2},
    {"id": "scene_08", "split": "train", "x": 0.5, "y": -1.4, "yaw": 1.75},
    {"id": "scene_09", "split": "train", "x": 1.7, "y": -1.5, "yaw": 2.2},
)
POSE_VARIANTS = (
    (0.0, 0.0, 0.0),
    (0.08, 0.04, 0.48),
    (-0.06, 0.07, 0.96),
    (0.04, -0.08, 1.44),
)


def schedule():
    rows = []
    for scene in SCENES:
        for kind in KINDS:
            for variant, (dx, dy, dyaw) in enumerate(POSE_VARIANTS):
                rows.append(
                    {
                        "scene": scene["id"],
                        "split": scene["split"],
                        "kind": kind,
                        "variant": variant,
                        "x": scene["x"] + dx,
                        "y": scene["y"] + dy,
                        "yaw": (scene["yaw"] + dyaw) % (2 * math.pi),
                        "fixed_holdout": variant == 0 and "holdout" in scene,
                        "holdout_name": scene.get("holdout"),
                    }
                )
    return rows


def label_line(box, width, height):
    x1, y1, x2, y2 = box
    center_x = ((x1 + x2) / 2) / width
    center_y = ((y1 + y2) / 2) / height
    box_width = (x2 - x1) / width
    box_height = (y2 - y1) / height
    return f"0 {center_x:.8f} {center_y:.8f} {box_width:.8f} {box_height:.8f}\n"


def quality(box, pixels, expected, shape):
    height, width = shape[:2]
    x1, y1, x2, y2 = box
    box_width, box_height = x2 - x1, y2 - y1
    center = ((x1 + x2) / 2, (y1 + y2) / 2)
    center_error = math.hypot(center[0] - expected[0], center[1] - expected[1])
    fill_ratio = pixels / max(1, box_width * box_height)
    issues = []
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        issues.append("out_of_bounds")
    if not (10 <= box_width <= 100 and 10 <= box_height <= 100):
        issues.append("unexpected_dimensions")
    if pixels < 100:
        issues.append("too_few_changed_pixels")
    if not 0.12 <= fill_ratio <= 1.0:
        issues.append("unexpected_fill_ratio")
    if center_error > 25:
        issues.append("center_far_from_projection")
    return {
        "width_px": box_width,
        "height_px": box_height,
        "changed_pixels": pixels,
        "fill_ratio": fill_ratio,
        "center_error_px": center_error,
        "issues": issues,
    }


def light_transform(image, scene_index):
    # Mild, deterministic exposure/white-balance variation. Split assignment
    # happens at scene level, so variants never leak across sets.
    gains = (1.0, 1.0, 1.0, 0.90, 1.10, 0.84, 0.94, 1.04, 1.14, 0.88)
    gain = gains[scene_index]
    channels = np.array((1.0 + 0.02 * ((scene_index % 3) - 1), 1.0, 1.0 - 0.02 * ((scene_index % 3) - 1)))
    transformed = image.astype(np.float32) * gain * channels.reshape(1, 1, 3)
    return np.clip(transformed, 0, 255).astype(np.uint8), {
        "exposure_gain": gain,
        "bgr_channel_gain": channels.tolist(),
    }


def copy_holdout(row):
    source = ROOT / "artifacts/yolo_diagnostic_stage5b" / row["kind"] / row["holdout_name"]
    result = json.loads((ROOT / "artifacts/yolo_diagnostic_stage5b/results.json").read_text())
    record = next(
        sample for sample in result["samples"]
        if sample["kind"] == row["kind"] and sample["variant"] == row["holdout_name"]
    )
    return cv2.imread(str(source / "litter.jpg")), record["ground_truth_bbox"], record["changed_pixels"]


def write_contact_sheet(records, output):
    cells = []
    for record in records:
        image = cv2.imread(record["image"])
        if record["bbox"]:
            x1, y1, x2, y2 = record["bbox"]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, record["name"], (6, 18), 0, 0.45, (255, 255, 255), 2)
        cells.append(cv2.resize(image, (256, 192)))
    pages = []
    for offset in range(0, len(cells), 20):
        page_cells = cells[offset:offset + 20]
        while len(page_cells) < 20:
            page_cells.append(np.zeros_like(cells[0]))
        pages.append(np.vstack([np.hstack(page_cells[i:i + 5]) for i in range(0, 20, 5)]))
    cv2.imwrite(str(output), np.vstack(pages))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "datasets/gazebo_litter_pilot")
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:8766/snapshot.jpg")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing dataset: {args.output}")
    for split in ("train", "val", "test"):
        (args.output / "images" / split).mkdir(parents=True)
        (args.output / "labels" / split).mkdir(parents=True)

    wait_for_camera(args.snapshot_url)
    sim_command("litter-remove")
    sequence = camera_sequence(args.snapshot_url)
    sequence = wait_frames(args.snapshot_url, sequence, count=3)
    clean_background = capture(args.snapshot_url)
    records = []

    # Three target-free samples per scene, kept with that scene's split.
    for scene_index, scene in enumerate(SCENES):
        background, lighting = light_transform(clean_background, scene_index)
        for variant, delta in enumerate((-0.02, 0.0, 0.02)):
            image = np.clip(background.astype(np.float32) * (1.0 + delta), 0, 255).astype(np.uint8)
            name = f"{scene['id']}_background_{variant:02d}"
            image_path = args.output / "images" / scene["split"] / f"{name}.jpg"
            label_path = args.output / "labels" / scene["split"] / f"{name}.txt"
            cv2.imwrite(str(image_path), image)
            label_path.write_text("")
            records.append({"name": name, "scene": scene["id"], "split": scene["split"], "kind": "background", "bbox": None, "image": str(image_path), "label": str(label_path), "lighting": {**lighting, "frame_gain": 1.0 + delta}, "quality": {"issues": []}})

    current_kind = None
    try:
        for index, row in enumerate(schedule(), 1):
            scene_index = int(row["scene"].split("_")[1])
            if row["fixed_holdout"]:
                raw, box, pixels = copy_holdout(row)
            else:
                if current_kind != row["kind"]:
                    sim_command("litter-remove")
                    sim_command("litter-add", row["kind"], row["x"], row["y"], row["yaw"])
                    current_kind = row["kind"]
                else:
                    sim_command("litter-pose", row["kind"], row["x"], row["y"], row["yaw"])
                expected = world_to_pixel(row["x"], row["y"])
                sequence, raw, box, pixels, _ = wait_for_rendered_litter(
                    args.snapshot_url,
                    sequence,
                    clean_background,
                    expected,
                    timeout=30.0,
                    max_center_error=6.0,
                )
            expected = world_to_pixel(row["x"], row["y"])
            check = quality(box, pixels, expected, raw.shape)
            image, lighting = light_transform(raw, scene_index)
            name = f"{row['scene']}_{row['kind']}_{row['variant']:02d}"
            image_path = args.output / "images" / row["split"] / f"{name}.jpg"
            label_path = args.output / "labels" / row["split"] / f"{name}.txt"
            if row["fixed_holdout"]:
                source = (
                    ROOT
                    / "artifacts/yolo_diagnostic_stage5b"
                    / row["kind"]
                    / row["holdout_name"]
                    / "litter.jpg"
                )
                shutil.copy2(source, image_path)
            else:
                cv2.imwrite(str(image_path), image)
            label_path.write_text(label_line(box, image.shape[1], image.shape[0]))
            records.append({**row, "name": name, "bbox": box, "changed_pixels": pixels, "image": str(image_path), "label": str(label_path), "lighting": lighting, "quality": check})
            print(f"[{index:03d}/120] {name} bbox={box} issues={check['issues']}", flush=True)
    finally:
        sim_command("litter-remove")

    counts = {
        split: {
            kind: sum(1 for item in records if item["split"] == split and item["kind"] == kind)
            for kind in (*KINDS, "background")
        }
        for split in ("train", "val", "test")
    }
    issues = [item for item in records if item["quality"]["issues"]]
    manifest = {
        "class_names": ["litter"],
        "scene_split": {scene["id"]: scene["split"] for scene in SCENES},
        "counts": counts,
        "records": records,
        "quality_issue_count": len(issues),
        "ground_truth": "paired clean-background difference in known projected-coordinate ROI; never model predictions",
        "fixed_holdout": "scene_00..02 variant 00 reuse Stage 5B diagnostic images and annotations",
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "data.yaml").write_text(
        f"path: {args.output.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: litter\n"
    )
    write_contact_sheet(records, args.output / "quality_contact_sheet.jpg")
    print(json.dumps({"counts": counts, "quality_issue_count": len(issues)}, indent=2))
    if issues:
        raise RuntimeError(f"{len(issues)} annotations require manual review")


if __name__ == "__main__":
    main()
