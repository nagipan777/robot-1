"""Compare YOLO weights on the fixed scene-separated holdout."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def iou(first, second):
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    union = ((first[2] - first[0]) * (first[3] - first[1]) +
             (second[2] - second[0]) * (second[3] - second[1]) - intersection)
    return intersection / union if union else 0.0


def read_gt(label_path, width, height):
    text = label_path.read_text().strip()
    if not text:
        return None
    _, cx, cy, box_width, box_height = map(float, text.split())
    return [
        (cx - box_width / 2) * width,
        (cy - box_height / 2) * height,
        (cx + box_width / 2) * width,
        (cy + box_height / 2) * height,
    ]


def average_precision(ranked, positives):
    if not positives:
        return 0.0
    tp = np.cumsum([item["tp"] for item in ranked])
    fp = np.cumsum([1 - item["tp"] for item in ranked])
    recall = tp / positives
    precision = tp / np.maximum(tp + fp, 1)
    recall = np.concatenate(([0.0], recall, [1.0]))
    precision = np.concatenate(([1.0], precision, [0.0]))
    precision = np.maximum.accumulate(precision[::-1])[::-1]
    points = np.linspace(0, 1, 101)
    return float(np.mean([precision[recall >= point].max(initial=0) for point in points]))


def evaluate(
    model_path, dataset, imgsz, confidence=0.25, device=None, visualization_dir=None
):
    from ultralytics import YOLO

    model = YOLO(model_path)
    image_dir = dataset / "images/test"
    label_dir = dataset / "labels/test"
    ranked = []
    fixed_tp = fixed_fp = fixed_fn = 0
    background_fp = 0
    background_images = 0
    positive_images = 0
    matched_ious = []
    all_best_ious = []
    kind_hits = {"can": [0, 0], "paper": [0, 0], "bottle": [0, 0]}
    timings = []
    details = []
    if visualization_dir:
        visualization_dir.mkdir(parents=True, exist_ok=True)
    for image_path in sorted(image_dir.glob("*.jpg")):
        image = cv2.imread(str(image_path))
        gt = read_gt(label_dir / f"{image_path.stem}.txt", image.shape[1], image.shape[0])
        started = time.perf_counter()
        result = model.predict(image, imgsz=imgsz, conf=0.001, verbose=False, device=device)[0]
        timings.append((time.perf_counter() - started) * 1000)
        predictions = []
        for box in result.boxes:
            predictions.append({
                "confidence": float(box.conf.item()),
                "bbox": [float(value) for value in box.xyxy[0].cpu().tolist()],
            })
        predictions.sort(key=lambda item: item["confidence"], reverse=True)
        matched = False
        for prediction in predictions:
            overlap = iou(prediction["bbox"], gt) if gt else 0.0
            is_tp = bool(gt and not matched and overlap >= 0.5)
            matched = matched or is_tp
            ranked.append({"confidence": prediction["confidence"], "tp": int(is_tp)})
            prediction["iou"] = overlap
        selected = [item for item in predictions if item["confidence"] >= confidence]
        best_iou = max((item["iou"] for item in selected), default=0.0)
        hit = best_iou >= 0.5
        if gt:
            positive_images += 1
            fixed_tp += int(hit)
            fixed_fn += int(not hit)
            fixed_fp += len(selected) - int(hit)
            all_best_ious.append(best_iou)
            if hit:
                matched_ious.append(best_iou)
            kind = next(kind for kind in kind_hits if f"_{kind}_" in image_path.stem)
            kind_hits[kind][0] += int(hit)
            kind_hits[kind][1] += 1
        else:
            background_images += 1
            background_fp += len(selected)
            fixed_fp += len(selected)
        if visualization_dir:
            annotated = image.copy()
            if gt:
                x1, y1, x2, y2 = (int(round(value)) for value in gt)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for item in selected:
                x1, y1, x2, y2 = (int(round(value)) for value in item["bbox"])
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(
                    annotated,
                    f"{item['confidence']:.2f} IoU {item['iou']:.2f}",
                    (x1, max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 0, 255),
                    1,
                )
            cv2.imwrite(str(visualization_dir / image_path.name), annotated)
        details.append({"image": image_path.name, "has_gt": gt is not None, "predictions_at_confidence": selected, "best_iou": best_iou})
    ranked.sort(key=lambda item: item["confidence"], reverse=True)
    precision = fixed_tp / (fixed_tp + fixed_fp) if fixed_tp + fixed_fp else 0.0
    recall = fixed_tp / positive_images if positive_images else 0.0
    return {
        "model": str(model_path), "imgsz": imgsz, "confidence": confidence,
        "tp": fixed_tp, "fp": fixed_fp, "fn": fixed_fn,
        "precision": precision, "recall": recall,
        "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0,
        "map50": average_precision(ranked, positive_images),
        "background_fp_per_image": background_fp / background_images,
        "background_fp": background_fp, "background_images": background_images,
        "detection_rate_by_kind": {kind: hits / total for kind, (hits, total) in kind_hits.items()},
        "mean_matched_iou": float(np.mean(matched_ious)) if matched_ious else 0.0,
        "mean_best_iou_all_positive": float(np.mean(all_best_ious)),
        "mean_wall_time_ms": float(np.mean(timings)), "details": details,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--finetuned", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default=None)
    args = parser.parse_args()
    results = [
        evaluate(
            model,
            args.dataset,
            size,
            device=args.device,
            visualization_dir=(
                args.output / "visualizations" / f"{Path(model).stem}_imgsz{size}"
            ),
        )
        for model in (args.original, args.finetuned)
        for size in (640, 960)
    ]
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")
    lines = ["# Gazebo litter holdout comparison", "", "| model | imgsz | precision | recall | F1 | mAP50 | background FP/image | mean IoU | inference ms |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for result in results:
        lines.append(
            f"| {Path(result['model']).name} | {result['imgsz']} | {result['precision']:.3f} | "
            f"{result['recall']:.3f} | {result['f1']:.3f} | {result['map50']:.3f} | "
            f"{result['background_fp_per_image']:.3f} | {result['mean_matched_iou']:.3f} | {result['mean_wall_time_ms']:.1f} |"
        )
    (args.output / "COMPARISON.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
