"""Capture positive litter boxes or unlabeled negative webcam images.

Install: python3 -m pip install opencv-python
Run:     python3 collect_dirt_dataset.py --camera 0 --kind positive --split train

For positive capture, press S and draw ONE box around litter, then press
Enter/Space. Canceling the ROI does not save. For negative capture, press S to
save an empty-label image. Never mark the Roomba or a battery as litter.
"""

import argparse
import platform
import time
from pathlib import Path
from datetime import datetime
import cv2


def yolo_label(box, image_size):
    x, y, width, height = box
    image_width, image_height = image_size
    if width <= 0 or height <= 0:
        raise ValueError("positive samples require a non-empty litter box")
    cx, cy = (x + width / 2) / image_width, (y + height / 2) / image_height
    return (
        f"0 {cx:.6f} {cy:.6f} "
        f"{width / image_width:.6f} {height / image_height:.6f}\n"
    )


def open_camera(camera):
    if platform.system() == "Darwin":
        return cv2.VideoCapture(camera, cv2.CAP_AVFOUNDATION)
    return cv2.VideoCapture(camera)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera index (try 0, 1, 2)")
    parser.add_argument("--output", type=Path, default=Path("dirt_dataset"))
    parser.add_argument(
        "--kind", choices=("positive", "negative"), required=True,
        help="positive=litter box, negative=empty label (Roomba/battery/background)",
    )
    parser.add_argument(
        "--split", choices=("train", "val", "test"), required=True,
        help="dataset split; keep each capture scene in only one split",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    args = parser.parse_args()

    root = args.output.resolve()
    for split in ("train", "val", "test"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    (root / "data.yaml").write_text(
        f'path: "{root.as_posix()}"\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: litter\n',
        encoding="utf-8",
    )

    cap = open_camera(args.camera)
    if not cap.isOpened():
        raise SystemExit("Camera not found. Close other camera apps and try --camera 0, 1, or 2.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    saved = 0
    last_frame_at = time.monotonic()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                if time.monotonic() - last_frame_at >= 10.0:
                    raise SystemExit(
                        "Camera frame could not be read for 10 seconds. "
                        "Close vision.py, calibrate_floor.py, Zoom, and other camera apps, "
                        "then try --camera 0 or --camera 1."
                    )
                time.sleep(0.1)
                continue
            last_frame_at = time.monotonic()
            preview = frame.copy()
            instruction = (
                "S: capture + draw litter box"
                if args.kind == "positive"
                else "S: save unlabeled negative"
            )
            cv2.putText(preview, f"{args.split}/{args.kind}  {instruction}  Q: quit", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("C270 capture", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key != ord("s"):
                continue

            box = None
            if args.kind == "positive":
                roi_window = "Draw ONE litter box; Enter to save; C to cancel"
                box = tuple(map(int, cv2.selectROI(
                    roi_window, frame, showCrosshair=True, fromCenter=False
                )))
                cv2.destroyWindow(roi_window)
                if box[2] <= 0 or box[3] <= 0:
                    print("Canceled: positive image was not saved")
                    continue
            saved += 1
            name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_path = root / "images" / args.split / f"{name}.jpg"
            label_path = root / "labels" / args.split / f"{name}.txt"
            if box is not None:
                height, width = frame.shape[:2]
                label = yolo_label(box, (width, height))
            else:
                label = ""
            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError(f"Could not write {image_path}")
            label_path.write_text(label, encoding="utf-8")
            print(
                f"Saved {args.split}: {image_path.name} "
                f"({'litter' if label else 'negative'})"
            )
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"Dataset: {root / 'data.yaml'}")


if __name__ == "__main__":
    main()
