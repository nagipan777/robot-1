"""Collect and score Gazebo litter images without training or robot motion."""

import argparse
import hashlib
import json
import math
import subprocess
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

from compare_sim_litter import box_iou, difference_region


ROOT = Path(__file__).resolve().parent
KINDS = ("can", "paper", "bottle")
THRESHOLDS = (0.1, 0.25, 0.5)
IMAGE_SIZES = (640, 960)
VARIANTS = (
    {"name": "far_right", "x": 1.5, "y": -0.7, "yaw": 0.0},
    {"name": "near_left", "x": -1.2, "y": 1.0, "yaw": 0.75},
    {"name": "upper_left_rotated", "x": 0.8, "y": 1.4, "yaw": 1.57},
)
HOMOGRAPHY = (
    0.0, -0.0124913252039, 2.99167238633,
    -0.0124826624397, 0.0, 3.9882106495,
    0.0, 0.0, 1.0,
)


def sha256(path):
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def world_to_pixel(x, y):
    # This fixed camera calibration is affine and axis-swapped.
    pixel_y = (HOMOGRAPHY[2] - x) / -HOMOGRAPHY[1]
    pixel_x = (HOMOGRAPHY[5] - y) / -HOMOGRAPHY[3]
    return pixel_x, pixel_y


def camera_sequence(url):
    with urllib.request.urlopen(url.replace("/snapshot.jpg", "/health"), timeout=5) as response:
        status = json.loads(response.read().decode("utf-8"))
    if not status.get("ready"):
        raise RuntimeError("simulated camera is not ready")
    return int(status["sequence"])


def wait_for_camera(url, timeout=240.0):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            return camera_sequence(url)
        except Exception as exc:
            last_error = exc
            time.sleep(2)
    raise TimeoutError(f"camera unavailable: {last_error}")


def wait_frames(url, previous, count=3, timeout=120.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sequence = camera_sequence(url)
        if sequence >= previous + count:
            return sequence
        time.sleep(0.25)
    raise TimeoutError("camera sequence did not advance")


def capture(url):
    with urllib.request.urlopen(url, timeout=5) as response:
        encoded = np.frombuffer(response.read(), dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("camera snapshot could not be decoded")
    return image


def wait_for_rendered_litter(
    url,
    previous,
    background,
    expected_pixel,
    timeout=20.0,
    max_center_error=None,
):
    """Wait until Gazebo's camera has rendered the newly created entity."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        previous = wait_frames(url, previous, count=2, timeout=timeout)
        foreground = capture(url)
        try:
            gt_box, changed_pixels, mask = difference_region(
                background, foreground, center=expected_pixel, radius=85
            )
            if max_center_error is not None:
                center_x = (gt_box[0] + gt_box[2]) / 2
                center_y = (gt_box[1] + gt_box[3]) / 2
                center_error = math.hypot(
                    center_x - expected_pixel[0], center_y - expected_pixel[1]
                )
                if center_error > max_center_error:
                    last_error = ValueError(
                        f"stale rendered pose: center error {center_error:.1f}px"
                    )
                    time.sleep(0.5)
                    continue
            return previous, foreground, gt_box, changed_pixels, mask
        except ValueError as exc:
            last_error = exc
            time.sleep(0.5)
    raise TimeoutError(f"litter was not rendered before timeout: {last_error}")


def sim_command(*args):
    subprocess.run(
        [str(ROOT / "simulation/sim.sh"), *map(str, args)],
        cwd=ROOT,
        check=True,
    )


def predictions(result):
    output = []
    for box in result.boxes:
        output.append(
            {
                "class": str(result.names[int(box.cls.item())]),
                "confidence": float(box.conf.item()),
                "bbox": [float(value) for value in box.xyxy[0].cpu().tolist()],
            }
        )
    return output


def infer(model, image, confidence, imgsz):
    started = time.perf_counter()
    result = model.predict(
        image, conf=confidence, imgsz=imgsz, verbose=False
    )[0]
    wall_ms = (time.perf_counter() - started) * 1000.0
    return result, {
        "wall_time_ms": wall_ms,
        "preprocess_ms": float(result.speed.get("preprocess", 0.0)),
        "inference_ms": float(result.speed.get("inference", 0.0)),
        "postprocess_ms": float(result.speed.get("postprocess", 0.0)),
    }


def annotate(image, gt_box, detected, title):
    output = image.copy()
    if gt_box is not None:
        x1, y1, x2, y2 = gt_box
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(output, "GT", (x1, max(14, y1 - 4)), 0, 0.45, (0, 255, 0), 1)
    for item in detected:
        x1, y1, x2, y2 = (int(round(value)) for value in item["bbox"])
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f"{item['class']} {item['confidence']:.2f} IoU {item.get('iou', 0):.2f}"
        cv2.putText(output, label, (x1, min(output.shape[0] - 5, y1 + 15)), 0, 0.42, (0, 0, 255), 1)
    cv2.putText(output, title, (8, 20), 0, 0.52, (255, 255, 255), 2)
    return output


def score_predictions(items, gt_box):
    scored = []
    for item in items:
        copy = dict(item)
        copy["iou"] = box_iou(copy["bbox"], gt_box) if gt_box else 0.0
        scored.append(copy)
    matching = [item for item in scored if item["iou"] >= 0.5]
    tp = 1 if gt_box is not None and matching else 0
    fp = len(scored) - tp
    fn = 1 - tp if gt_box is not None else 0
    return scored, tp, fp, fn


def write_report(path, report):
    lines = [
        "# Stage 5B YOLO diagnostic",
        "",
        f"Weights SHA256: `{report['weights_sha256']}`",
        f"Device: `{report['device']}`",
        "",
        "## Aggregate metrics",
        "",
        "| imgsz | confidence | TP | FP | FN | precision | recall | mean inference ms |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for metric in report["aggregate"]:
        lines.append(
            "| {imgsz} | {confidence:.2f} | {tp} | {fp} | {fn} | {precision:.3f} | {recall:.3f} | {mean_inference_ms:.1f} |".format(**metric)
        )
    lines.extend(
        [
            "",
            "## Background versus object-frame false positives",
            "",
            "| imgsz | confidence | background FP | object-frame FP |",
            "|---:|---:|---:|---:|",
        ]
    )
    for metric in report["aggregate"]:
        selected = [
            evaluation
            for sample in report["samples"]
            for evaluation in sample["evaluations"]
            if evaluation["imgsz"] == metric["imgsz"]
            and math.isclose(
                evaluation["confidence_threshold"], metric["confidence"]
            )
        ]
        background_fp = sum(
            item["fp"] for item in selected if item["condition"] == "background"
        )
        object_fp = sum(
            item["fp"] for item in selected if item["condition"] == "litter"
        )
        lines.append(
            f"| {metric['imgsz']} | {metric['confidence']:.2f} | "
            f"{background_fp} | {object_fp} |"
        )
    lines.extend(
        [
            "",
            "## Object measurements at imgsz=640, confidence=0.10",
            "",
            "| kind | variant | GT bbox | width×height px | changed pixels | best confidence | best IoU |",
            "|---|---|---|---:|---:|---:|---:|",
        ]
    )
    for sample in report["samples"]:
        evaluation = next(
            item for item in sample["evaluations"]
            if item["condition"] == "litter"
            and item["imgsz"] == 640
            and item["confidence_threshold"] == 0.1
        )
        best = max(evaluation["predictions"], key=lambda item: item["iou"], default=None)
        box = sample["ground_truth_bbox"]
        lines.append(
            f"| {sample['kind']} | {sample['variant']} | {box} | "
            f"{box[2] - box[0]}×{box[3] - box[1]} | {sample['changed_pixels']} | "
            f"{(best['confidence'] if best else 0):.3f} | {(best['iou'] if best else 0):.3f} |"
        )
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot-url", default="http://127.0.0.1:8766/snapshot.jpg")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "artifacts/yolo_diagnostic_stage5b",
    )
    args = parser.parse_args()
    weights = ROOT / "runs/detect/taco/weights/best.pt"
    manifest = json.loads((ROOT / "simulation/models/litter_experiment.json").read_text())
    from ultralytics import YOLO

    model = YOLO(weights)
    device = str(next(model.model.parameters()).device)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    wait_for_camera(args.snapshot_url)
    samples = []
    try:
        for kind in KINDS:
            for variant in VARIANTS:
                sample_dir = args.output_dir / kind / variant["name"]
                sample_dir.mkdir(parents=True, exist_ok=True)
                sequence = camera_sequence(args.snapshot_url)
                sim_command("litter-remove")
                sequence = wait_frames(args.snapshot_url, sequence)
                background = capture(args.snapshot_url)
                cv2.imwrite(str(sample_dir / "background.jpg"), background)

                sim_command(
                    "litter-add", kind, variant["x"], variant["y"], variant["yaw"]
                )
                expected_pixel = world_to_pixel(variant["x"], variant["y"])
                sequence, foreground, gt_box, changed_pixels, mask = (
                    wait_for_rendered_litter(
                        args.snapshot_url,
                        sequence,
                        background,
                        expected_pixel,
                    )
                )
                cv2.imwrite(str(sample_dir / "litter.jpg"), foreground)
                cv2.imwrite(str(sample_dir / "difference_mask.png"), mask * 255)

                samples.append(
                    {
                        "kind": kind,
                        "variant": variant["name"],
                        "world_pose": [variant["x"], variant["y"], variant["yaw"]],
                        "expected_pixel": list(expected_pixel),
                        "appearance": manifest[kind],
                        "ground_truth_method": "paired background absolute difference >=25 within expected-position ROI",
                        "ground_truth_bbox": gt_box,
                        "ground_truth_width_px": gt_box[2] - gt_box[0],
                        "ground_truth_height_px": gt_box[3] - gt_box[1],
                        "changed_pixels": changed_pixels,
                        "evaluations": [],
                    }
                )
                print(f"captured {kind}/{variant['name']}: GT={gt_box}, pixels={changed_pixels}")
    finally:
        sim_command("litter-remove")

    # Run inference only after every camera frame is safely on disk. On an
    # 8 GB host, simultaneous Gazebo rendering and YOLO inference can starve
    # the camera stream and invalidate the collection sequence.
    for sample in samples:
        sample_dir = args.output_dir / sample["kind"] / sample["variant"]
        background = cv2.imread(str(sample_dir / "background.jpg"))
        foreground = cv2.imread(str(sample_dir / "litter.jpg"))
        gt_box = sample["ground_truth_bbox"]
        for imgsz in IMAGE_SIZES:
            for threshold in THRESHOLDS:
                for condition, image, gt in (
                    ("background", background, None),
                    ("litter", foreground, gt_box),
                ):
                    result, timing = infer(model, image, threshold, imgsz)
                    detected, tp, fp, fn = score_predictions(predictions(result), gt)
                    sample["evaluations"].append(
                        {
                            "condition": condition,
                            "imgsz": imgsz,
                            "confidence_threshold": threshold,
                            "predictions": detected,
                            "tp": tp,
                            "fp": fp,
                            "fn": fn,
                            "timing": timing,
                        }
                    )
                    image_name = f"{condition}_imgsz{imgsz}_conf{threshold:.2f}.jpg"
                    cv2.imwrite(
                        str(sample_dir / image_name),
                        annotate(
                            image,
                            gt,
                            detected,
                            f"{sample['kind']} {sample['variant']} imgsz={imgsz} conf={threshold:.2f}",
                        ),
                    )

    aggregate = []
    for imgsz in IMAGE_SIZES:
        for threshold in THRESHOLDS:
            selected = [
                evaluation
                for sample in samples
                for evaluation in sample["evaluations"]
                if evaluation["imgsz"] == imgsz
                and math.isclose(evaluation["confidence_threshold"], threshold)
            ]
            tp = sum(item["tp"] for item in selected)
            fp = sum(item["fp"] for item in selected)
            fn = sum(item["fn"] for item in selected)
            aggregate.append(
                {
                    "imgsz": imgsz,
                    "confidence": threshold,
                    "tp": tp,
                    "fp": fp,
                    "fn": fn,
                    "precision": tp / (tp + fp) if tp + fp else 0.0,
                    "recall": tp / (tp + fn) if tp + fn else 0.0,
                    "mean_inference_ms": float(np.mean([item["timing"]["inference_ms"] for item in selected])),
                    "mean_wall_time_ms": float(np.mean([item["timing"]["wall_time_ms"] for item in selected])),
                }
            )
    report = {
        "weights": str(weights),
        "weights_sha256": sha256(weights),
        "weights_modified": False,
        "device": device,
        "image_resolution": [640, 480],
        "confidence_thresholds": list(THRESHOLDS),
        "inference_image_sizes": list(IMAGE_SIZES),
        "iou_match_threshold": 0.5,
        "samples": samples,
        "aggregate": aggregate,
    }
    (args.output_dir / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    write_report(args.output_dir / "REPORT.md", report)
    print(json.dumps(aggregate, indent=2))


if __name__ == "__main__":
    main()
