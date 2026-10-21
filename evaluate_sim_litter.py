"""Capture one simulated-camera frame and record an honest YOLO evaluation."""

import argparse
import json
import time
from pathlib import Path

import cv2


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Evaluate demo_litter with the existing YOLO model")
    parser.add_argument(
        "--video-url", default="http://127.0.0.1:8766/stream.mjpg"
    )
    parser.add_argument(
        "--weights", type=Path, default=root / "runs/detect/taco/weights/best.pt"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=root / "artifacts/sim_litter"
    )
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()

    from ultralytics import YOLO

    capture = cv2.VideoCapture(args.video_url)
    try:
        ok, frame = capture.read()
    finally:
        capture.release()
    if not ok or frame is None or frame.size == 0:
        raise SystemExit(f"could not read {args.video_url}")

    result = YOLO(args.weights).predict(
        frame, conf=args.confidence, verbose=False
    )[0]
    detections = []
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
        detections.append(
            {
                "label": result.names[int(box.cls.item())],
                "confidence": round(box.conf.item(), 6),
                "center_x": round((x1 + x2) / 2, 2),
                "center_y": round((y1 + y2) / 2, 2),
                "width": round(x2 - x1, 2),
                "height": round(y2 - y1, 2),
            }
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "camera_raw.jpg"
    annotated_path = args.output_dir / "yolo_annotated.jpg"
    log_path = args.output_dir / "detections.json"
    cv2.imwrite(str(raw_path), frame)
    cv2.imwrite(str(annotated_path), result.plot())
    log_path.write_text(
        json.dumps(
            {
                "timestamp": time.time(),
                "video_url": args.video_url,
                "weights": str(args.weights),
                "confidence_threshold": args.confidence,
                "image_width": frame.shape[1],
                "image_height": frame.shape[0],
                "detections": detections,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )
    litter = [item for item in detections if item["label"].lower() == "litter"]
    print(f"saved raw image: {raw_path}")
    print(f"saved annotated image: {annotated_path}")
    print(f"saved detection log: {log_path}")
    if litter:
        best = max(litter, key=lambda item: item["confidence"])
        print(
            f"litter detected: confidence={best['confidence']:.4f}, "
            f"center=({best['center_x']:.1f}, {best['center_y']:.1f})"
        )
    else:
        print("litter was not detected; no retraining was performed")


if __name__ == "__main__":
    main()
