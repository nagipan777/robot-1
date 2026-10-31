#!/usr/bin/env python3
"""Finite, read-only YOLO inference load for latency comparisons."""

import argparse
import time

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    model = YOLO(args.weights)
    started = time.monotonic()
    count = 0
    while time.monotonic() - started < args.duration:
        model.predict(args.image, imgsz=640, conf=0.5, verbose=False)
        count += 1
    elapsed = time.monotonic() - started
    print("inferences={} elapsed_s={:.3f} rate_hz={:.3f}".format(
        count, elapsed, count / elapsed if elapsed else 0.0
    ))


if __name__ == "__main__":
    main()
