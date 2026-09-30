"""Download pictograph/taco and convert its COCO boxes to YOLO litter labels."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import random
import shutil
import time
from urllib.parse import quote
from urllib.request import urlopen

from PIL import Image, ImageOps

BASE = "https://huggingface.co/datasets/pictograph/taco/resolve/main/train/"


def download(name, target):
    if target.exists():
        return
    temporary = target.with_suffix(target.suffix + ".part")
    for attempt in range(4):
        try:
            with urlopen(BASE + quote(name), timeout=120) as response, temporary.open("wb") as out:
                shutil.copyfileobj(response, out)
            temporary.replace(target)
            return
        except Exception:
            temporary.unlink(missing_ok=True)
            if attempt == 3:
                raise
            time.sleep(2 ** attempt)


def yolo_box(box, width, height):
    x, y, w, h = map(float, box)
    if w <= 0 or h <= 0:
        raise ValueError(f"Invalid box: {box}")
    x1, y1 = max(0, x), max(0, y)
    x2, y2 = min(width, x + w), min(height, y + h)
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"Box outside image: {box}")
    return f"0 {(x1+x2)/2/width:.8f} {(y1+y2)/2/height:.8f} {(x2-x1)/width:.8f} {(y2-y1)/height:.8f}\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("taco_dataset"))
    args = parser.parse_args()
    root = args.output.resolve()
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            (root / kind / split).mkdir(parents=True, exist_ok=True)
    download("metadata.jsonl", root / "metadata.jsonl")
    rows = [json.loads(line) for line in (root / "metadata.jsonl").read_text().splitlines() if line.strip()]
    rows.sort(key=lambda row: row["file_name"])
    if len(rows) < 2 or len({row['file_name'] for row in rows}) != len(rows):
        raise ValueError("Dataset must have distinct images and at least two rows")
    random.Random(42).shuffle(rows)
    nval = max(1, round(len(rows) * 0.2))

    def convert(item):
        index, row = item
        name = row["file_name"]
        if Path(name).name != name:
            raise ValueError(f"Unexpected filename: {name}")
        split = "val" if index < nval else "train"
        target = root / "images" / split / name
        download(name, target)
        with Image.open(target) as im:
            im.verify()
        with Image.open(target) as im:
            upright = ImageOps.exif_transpose(im)
            width, height = upright.size
            if im.getexif().get(274, 1) != 1:
                temporary = target.with_suffix(".upright.jpg")
                upright.convert("RGB").save(temporary, quality=95)
                temporary.replace(target)
        labels = ''.join(yolo_box(box, width, height) for box in row["objects"]["bbox"])
        (root / "labels" / split / (Path(name).stem + ".txt")).write_text(labels)

    with ThreadPoolExecutor(max_workers=24) as pool:
        for count, _ in enumerate(pool.map(convert, enumerate(rows)), 1):
            if count % 50 == 0 or count == len(rows):
                print(f"Converted {count}/{len(rows)}", flush=True)
    (root / "data.yaml").write_text(
        f"path: {json.dumps(str(root))}\ntrain: images/train\nval: images/val\nnames:\n  0: litter\n"
    )
    (root / "SOURCE.txt").write_text(
        "Source: https://huggingface.co/datasets/pictograph/taco\n"
        "Original dataset: TACO, Pedro F. Proenca and Pedro Simoes\n"
        "https://github.com/pedropro/TACO\n"
        "Dataset card license: CC BY 4.0 https://creativecommons.org/licenses/by/4.0/\n"
        "Changes: all categories mapped to litter (0); boxes converted from pixel xywh\n"
        "to normalized YOLO xywh; image split 80/20 with seed 42.\n"
        "EXIF-rotated images saved upright as JPEG; boxes clipped to image bounds.\n"
    )
    print(f"Ready: {root / 'data.yaml'} (train={len(rows)-nval}, val={nval})")


if __name__ == "__main__":
    main()
