"""Validate and package real-webcam litter data for mixed Colab fine-tuning."""

import argparse
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

from build_mixed_colab_export import validate_label


ROOT = Path(__file__).resolve().parent
SPLITS = ("train", "val", "test")


def excluded_stems(dataset):
    path = dataset / "exclude.txt"
    if not path.exists():
        return set()
    return {
        line.strip()
        for line in path.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }


def inspect_real_dataset(dataset):
    excluded = excluded_stems(dataset)
    records = []
    hashes = {}
    counts = {}
    for split in SPLITS:
        images = sorted((dataset / "images" / split).glob("*.jpg"))
        summary = {"images": 0, "positive": 0, "negative": 0, "excluded": 0}
        for image in images:
            label = dataset / "labels" / split / f"{image.stem}.txt"
            if not label.exists():
                raise FileNotFoundError(f"missing label: {label}")
            if image.stem in excluded:
                summary["excluded"] += 1
                continue
            result = validate_label(label.read_bytes(), label)
            summary["images"] += 1
            summary["negative" if result["empty"] else "positive"] += 1
            digest = hashlib.sha256(image.read_bytes()).hexdigest()
            hashes.setdefault(digest, set()).add(split)
            records.append((split, image, label))
        counts[split] = summary
    leaked = [sorted(splits) for splits in hashes.values() if len(splits) > 1]
    if leaked:
        raise ValueError(f"exact image duplicates cross splits: {leaked}")
    for split in SPLITS:
        if counts[split]["positive"] == 0 or counts[split]["negative"] == 0:
            raise ValueError(
                f"{split} requires both positive and negative images: {counts[split]}"
            )
    return records, counts, excluded


def package(dataset, output):
    records, counts, excluded = inspect_real_dataset(dataset)
    output.mkdir(parents=True, exist_ok=True)
    real_yaml = (
        "path: /content/real_webcam\ntrain: images/train\nval: images/val\n"
        "test: images/test\nnames:\n  0: litter\n"
    )
    with zipfile.ZipFile(output / "real_webcam.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("data.yaml", real_yaml)
        for split, image, label in records:
            archive.write(image, f"images/{split}/{image.name}")
            archive.write(label, f"labels/{split}/{label.name}")
    mixed_yaml = (
        "path: /content\n"
        "train:\n  - taco/images/train\n  - gazebo/images/train\n  - real_webcam/images/train\n"
        "val:\n  - taco/images/val\n  - gazebo/images/val\n  - real_webcam/images/val\n"
        "test:\n  - taco/images/test\n  - gazebo/images/test\n  - real_webcam/images/test\n"
        "names:\n  0: litter\n"
    )
    (output / "mixed_real_data.yaml").write_text(mixed_yaml)
    manifest = {
        "dataset": str(dataset.resolve()),
        "counts": counts,
        "excluded_stems": sorted(excluded),
        "class_mapping": {"0": "litter"},
        "split_policy": "capture scenes must be assigned to exactly one split",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    shutil.copy2(
        ROOT / "notebooks/real_webcam_finetune_colab.ipynb",
        output / "real_webcam_finetune_colab.ipynb",
    )
    (output / "README.md").write_text(
        """# Real-webcam Colab fine-tuning

Upload these existing files to `MyDrive/robot-1-colab/`:

- `colab_export/taco.zip`
- `colab_export/gazebo.zip`
- `colab_export/taco_best.pt`
- `colab_export_real/real_webcam.zip`
- `colab_export_real/mixed_real_data.yaml`

Then open `real_webcam_finetune_colab.ipynb` in Colab, select a GPU runtime,
and run the cells in order. The notebook trains on TACO + Gazebo + real webcam,
then reports both mixed-test and real-webcam-test metrics. Do not move test
images into train, and do not enable robot control based on training metrics alone.
"""
    )
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, default=ROOT / "datasets/real_webcam")
    parser.add_argument("--output", type=Path, default=ROOT / "colab_export_real")
    args = parser.parse_args()
    manifest = package(args.dataset, args.output)
    print(json.dumps(manifest, indent=2))
    print(f"Colab files: {args.output}")


if __name__ == "__main__":
    main()
