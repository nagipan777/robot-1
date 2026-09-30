"""Capture and label dirt images from a webcam for YOLO detection training.

Install: python3 -m pip install opencv-python
Run:     python3 collect_dirt_dataset.py --camera 1

In the live view press S to freeze a frame. Draw ONE box around a dirt patch,
then press Enter/Space in the ROI window to save. Press C in the ROI window to
cancel the selection; a frame without a box is saved as a negative example.
Press Q in the live view to quit. Keep the camera stationary and vary the dirt,
lighting, position, and background. Do not save near-identical consecutive frames.
"""

import argparse
from pathlib import Path
from datetime import datetime
import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="Camera index (try 0, 1, 2)")
    parser.add_argument("--output", type=Path, default=Path("dirt_dataset"))
    args = parser.parse_args()

    root = args.output.resolve()
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    (root / "data.yaml").write_text(
        f'path: "{root.as_posix()}"\ntrain: images/train\nval: images/val\nnames:\n  0: dirt\n',
        encoding="utf-8",
    )

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit("Camera not found. Close other camera apps and try --camera 0, 1, or 2.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    saved = len(list((root / "images" / "train").glob("*.jpg"))) + len(
        list((root / "images" / "val").glob("*.jpg"))
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame could not be read.")
                break
            preview = frame.copy()
            cv2.putText(preview, "S: capture/label   Q: quit", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("C270 capture", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key != ord("s"):
                continue

            x, y, w, h = map(int, cv2.selectROI("Draw ONE dirt box; Enter to save; C for no dirt", frame,
                                               showCrosshair=True, fromCenter=False))
            cv2.destroyWindow("Draw ONE dirt box; Enter to save; C for no dirt")
            saved += 1
            split = "val" if saved % 5 == 0 else "train"
            name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_path = root / "images" / split / f"{name}.jpg"
            label_path = root / "labels" / split / f"{name}.txt"
            if w > 0 and h > 0:
                height, width = frame.shape[:2]
                cx, cy = (x + w / 2) / width, (y + h / 2) / height
                label = f"0 {cx:.6f} {cy:.6f} {w / width:.6f} {h / height:.6f}\n"
            else:
                label = ""  # Empty label file: clean floor / negative image.
            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError(f"Could not write {image_path}")
            label_path.write_text(label, encoding="utf-8")
            print(f"Saved {split}: {image_path.name} ({'dirt' if label else 'clean'})")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"Dataset: {root / 'data.yaml'}")


if __name__ == "__main__":
    main()
