"""Compare three Gazebo litter appearances with fixed YOLO settings."""

import argparse
import hashlib
import json
import subprocess
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
KINDS = ("can", "paper", "bottle")
CONFIDENCE_THRESHOLD = 0.5
PLACEMENT = (1.5, -0.7)
EXPECTED_PIXEL = (375.0, 119.0)


def box_iou(first, second):
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0 else 0.0


def difference_region(background, foreground, center=EXPECTED_PIXEL, radius=70):
    difference = cv2.absdiff(background, foreground)
    changed = (np.max(difference, axis=2) >= 25).astype(np.uint8)
    roi = np.zeros(changed.shape, dtype=np.uint8)
    center_x, center_y = (int(round(value)) for value in center)
    cv2.rectangle(
        roi,
        (max(0, center_x - radius), max(0, center_y - radius)),
        (
            min(changed.shape[1] - 1, center_x + radius),
            min(changed.shape[0] - 1, center_y + radius),
        ),
        1,
        -1,
    )
    changed &= roi
    changed = cv2.morphologyEx(changed, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ys, xs = np.nonzero(changed)
    if len(xs) < 20:
        raise ValueError("rendered litter difference was too small to measure")
    return (
        [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)],
        int(len(xs)),
        changed,
    )


def detections_from_result(result):
    detections = []
    for box in result.boxes:
        coordinates = [float(value) for value in box.xyxy[0].cpu().tolist()]
        detections.append(
            {
                "label": str(result.names[int(box.cls.item())]),
                "confidence": float(box.conf.item()),
                "box": coordinates,
            }
        )
    return detections


def capture_frame(url):
    with urllib.request.urlopen(url, timeout=5) as response:
        encoded = np.frombuffer(response.read(), dtype=np.uint8)
    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"could not decode {url}")
    return frame


def camera_sequence(snapshot_url):
    health_url = snapshot_url.replace("/snapshot.jpg", "/health")
    with urllib.request.urlopen(health_url, timeout=5) as response:
        status = json.loads(response.read().decode("utf-8"))
    if not status.get("ready"):
        raise RuntimeError("simulated camera is not ready")
    return int(status["sequence"])


def wait_for_new_frames(snapshot_url, previous_sequence, count=1, timeout=240.0):
    deadline = time.monotonic() + timeout
    target = previous_sequence + count
    while time.monotonic() < deadline:
        sequence = camera_sequence(snapshot_url)
        if sequence >= target:
            return sequence
        time.sleep(1.0)
    raise TimeoutError(
        f"camera did not advance from sequence {previous_sequence} to {target}"
    )


def wait_for_camera_ready(snapshot_url, timeout=240.0):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            return camera_sequence(snapshot_url)
        except Exception as exc:
            last_error = exc
            time.sleep(2.0)
    raise TimeoutError(f"simulated camera did not become ready: {last_error}")


def run_sim_command(*arguments):
    subprocess.run(
        [str(ROOT / "simulation/sim.sh"), *map(str, arguments)],
        cwd=ROOT,
        check=True,
    )


def evaluate(model, frame):
    result = model.predict(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]
    return result, detections_from_result(result)


def file_sha256(path):
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def write_markdown(path, report):
    lines = [
        "# Simulated litter YOLO comparison",
        "",
        f"Fixed weights: `{report['weights']}`",
        f"Confidence threshold: `{report['confidence_threshold']}`",
        f"Placement: `{report['placement_m']}`",
        "",
        "| Model | Physical size (m) | Colors / shape | Changed pixels | GT box (px) | Background false positives | Best confidence | Best IoU | Classification |",
        "|---|---|---|---:|---|---:|---:|---:|---|",
    ]
    for item in report["models"]:
        appearance = item["appearance"]
        lines.append(
            "| {kind} | {size} | {colors}; {shape} | {pixels} | {box} | {bg} | {confidence:.4f} | {iou:.4f} | {classification} |".format(
                kind=item["kind"],
                size=" × ".join(str(value) for value in appearance["physical_size_m"]),
                colors=", ".join(appearance["colors"]),
                shape=appearance["shape"],
                pixels=item["changed_pixel_count"],
                box=item["ground_truth_box"],
                bg=len(item["background_detections"]),
                confidence=item["best_confidence"],
                iou=item["best_iou"],
                classification=item["classification"],
            )
        )
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Compare simulated litter without retraining")
    parser.add_argument("--video-url", default="http://127.0.0.1:8766/snapshot.jpg")
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "artifacts/sim_litter_comparison"
    )
    parser.add_argument("--frame-wait-count", type=int, default=1)
    args = parser.parse_args()

    weights = ROOT / "runs/detect/taco/weights/best.pt"
    manifest = json.loads(
        (ROOT / "simulation/models/litter_experiment.json").read_text()
    )
    from ultralytics import YOLO

    model = YOLO(weights)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    wait_for_camera_ready(args.video_url)
    try:
        for kind in KINDS:
            condition_dir = args.output_dir / kind
            condition_dir.mkdir(parents=True, exist_ok=True)

            sequence = camera_sequence(args.video_url)
            run_sim_command("litter-remove")
            sequence = wait_for_new_frames(
                args.video_url, sequence, args.frame_wait_count
            )
            background = capture_frame(args.video_url)
            background_result, background_detections = evaluate(model, background)
            cv2.imwrite(str(condition_dir / "background.jpg"), background)
            cv2.imwrite(
                str(condition_dir / "background_yolo.jpg"), background_result.plot()
            )

            run_sim_command("litter-add", kind, *PLACEMENT)
            sequence = wait_for_new_frames(
                args.video_url, sequence, args.frame_wait_count
            )
            foreground = capture_frame(args.video_url)
            foreground_result, foreground_detections = evaluate(model, foreground)
            cv2.imwrite(str(condition_dir / "litter.jpg"), foreground)
            cv2.imwrite(
                str(condition_dir / "litter_yolo_unscored.jpg"),
                foreground_result.plot(),
            )
            ground_truth_box, pixel_count, mask = difference_region(
                background, foreground
            )
            litter_detections = [
                item for item in foreground_detections
                if item["label"].lower() == "litter"
            ]
            for detection in litter_detections:
                detection["iou"] = box_iou(detection["box"], ground_truth_box)
            best = max(litter_detections, key=lambda item: item["iou"], default=None)
            best_iou = best["iou"] if best else 0.0
            best_confidence = best["confidence"] if best else 0.0
            if best_iou >= 0.5:
                classification = "recognized"
            elif litter_detections:
                classification = "missed; detections are false positives"
            else:
                classification = "missed; no detection"

            annotated = foreground_result.plot()
            x1, y1, x2, y2 = ground_truth_box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                "ground truth",
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
            )
            cv2.imwrite(str(condition_dir / "litter_yolo_iou.jpg"), annotated)
            cv2.imwrite(str(condition_dir / "difference_mask.png"), mask * 255)

            results.append(
                {
                    "kind": kind,
                    "appearance": manifest[kind],
                    "changed_pixel_count": pixel_count,
                    "ground_truth_box": ground_truth_box,
                    "background_detections": background_detections,
                    "litter_detections": foreground_detections,
                    "best_confidence": best_confidence,
                    "best_iou": best_iou,
                    "classification": classification,
                }
            )
            print(
                f"{kind}: pixels={pixel_count}, confidence={best_confidence:.4f}, "
                f"IoU={best_iou:.4f}, {classification}"
            )
    finally:
        run_sim_command("litter-remove")

    report = {
        "weights": str(weights),
        "weights_sha256": file_sha256(weights),
        "confidence_threshold": CONFIDENCE_THRESHOLD,
        "placement_m": list(PLACEMENT),
        "ground_truth_method": "absolute background difference >=25 in a fixed ROI",
        "recognition_iou_threshold": 0.5,
        "models": results,
    }
    (args.output_dir / "results.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    write_markdown(args.output_dir / "results.md", report)
    print(f"saved: {args.output_dir / 'results.md'}")


if __name__ == "__main__":
    main()
