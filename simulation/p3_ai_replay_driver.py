#!/usr/bin/env python3
"""Replay saved real-camera frames through the P3 ROS dry-run ingress.

This tool never talks to Pico W and never publishes ROS commands directly.  It
posts YOLO results to the existing HTTP webcam bridge.  Physical-output
isolation must be checked separately before running it.
"""

import argparse
import hashlib
import json
import time
import urllib.request
from pathlib import Path


EXPECTED_MODEL_SHA256 = "a1c863beb4db5048eecc34aaf898d7580518219cc6eb4db8f198ec5a0717a68d"


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def detections_from_result(result, names):
    detections = []
    for box in result.boxes:
        xyxy = [float(value) for value in box.xyxy[0].tolist()]
        detections.append(
            {
                "label": names[int(box.cls[0])],
                "confidence": float(box.conf[0]),
                "center_x": (xyxy[0] + xyxy[2]) / 2.0,
                "center_y": (xyxy[1] + xyxy[3]) / 2.0,
                "bbox": xyxy,
            }
        )
    return detections


def infer(model, image_path, confidence, imgsz, replay_width, replay_height):
    import cv2

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"cannot read replay image: {image_path}")
    original_height, original_width = image.shape[:2]
    if (original_width, original_height) != (replay_width, replay_height):
        image = cv2.resize(image, (replay_width, replay_height))
    captured_at = time.time()
    result = model.predict(
        source=image, conf=confidence, imgsz=imgsz, verbose=False
    )[0]
    completed_at = time.time()
    height, width = result.orig_shape
    return {
        # Compatibility field intentionally means acquisition time.
        "timestamp": captured_at,
        "capture_timestamp": captured_at,
        "processing_completed_timestamp": completed_at,
        "processing_latency_s": completed_at - captured_at,
        "image_width": int(width),
        "image_height": int(height),
        "replay_source_width": int(original_width),
        "replay_source_height": int(original_height),
        "replay_resized_for_calibration": (original_width, original_height)
        != (replay_width, replay_height),
        "detections": detections_from_result(result, model.names),
    }


def post_json(url, payload):
    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    request = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        if response.status != 204:
            raise RuntimeError(f"unexpected HTTP status {response.status}")


def fresh_copy(payload, timestamp=None):
    copied = json.loads(json.dumps(payload))
    now = time.time() if timestamp is None else timestamp
    copied["timestamp"] = now
    copied["capture_timestamp"] = now
    copied["processing_completed_timestamp"] = now
    copied["processing_latency_s"] = 0.0
    return copied


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8765/detections")
    parser.add_argument("--model", default="runs/detect/real_finetune/weights/best.pt")
    parser.add_argument(
        "--near-image", default="artifacts/yolo_diagnostic_stage5b/paper/near_left/litter.jpg"
    )
    parser.add_argument(
        "--far-image", default="artifacts/yolo_diagnostic_stage5b/paper/far_right/litter.jpg"
    )
    parser.add_argument(
        "--background-image",
        default="artifacts/yolo_diagnostic_stage5b/paper/near_left/background.jpg",
    )
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--replay-width", type=int, default=1280)
    parser.add_argument("--replay-height", type=int, default=960)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    from ultralytics import YOLO

    model_path = Path(args.model)
    actual_sha = sha256_file(model_path)
    if actual_sha != EXPECTED_MODEL_SHA256:
        raise RuntimeError(
            f"model SHA256 mismatch: expected={EXPECTED_MODEL_SHA256} actual={actual_sha}"
        )
    model = YOLO(str(model_path))
    payloads = {
        "background": infer(model, Path(args.background_image), args.confidence, args.imgsz, args.replay_width, args.replay_height),
        "near": infer(model, Path(args.near_image), args.confidence, args.imgsz, args.replay_width, args.replay_height),
        "far": infer(model, Path(args.far_image), args.confidence, args.imgsz, args.replay_width, args.replay_height),
    }
    if payloads["background"]["detections"]:
        raise RuntimeError("background replay unexpectedly contains a detection")
    for name in ("near", "far"):
        if not payloads[name]["detections"]:
            raise RuntimeError(f"{name} replay has no detection")

    sent = []

    def send(name, payload, delay=0.12):
        message = fresh_copy(payload)
        post_json(args.url, message)
        sent.append({"name": name, "sent_at": time.time(), "payload": message})
        time.sleep(delay)

    send("initial_no_detection", payloads["background"])
    for index in range(3):
        send(f"near_confirmation_{index + 1}", payloads["near"])
    # Half-second silence exercises the sub-timeout boundary without asserting
    # that a repeated still frame means a frozen camera.
    time.sleep(0.50)
    send("explicit_detection_loss", payloads["background"])
    for index in range(3):
        send(f"same_target_reconfirmation_{index + 1}", payloads["near"])
    for index in range(3):
        send(f"different_target_confirmation_{index + 1}", payloads["far"])
    stale = fresh_copy(payloads["near"], timestamp=time.time() - 2.0)
    post_json(args.url, stale)
    sent.append({"name": "stale_image", "sent_at": time.time(), "payload": stale})
    time.sleep(1.10)
    send("final_no_detection", payloads["background"], delay=0.0)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "evidence_class": "saved-real-image-replay-not-live-camera",
                "physical_output": False,
                "pico_contact": False,
                "model": str(model_path),
                "model_sha256": actual_sha,
                "confidence": args.confidence,
                "imgsz": args.imgsz,
                "inference_payloads": payloads,
                "sent_events": sent,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
