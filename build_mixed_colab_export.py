"""Package scene-separated TACO and Gazebo YOLO datasets for Colab."""

import argparse
import hashlib
import json
import re
import shutil
import zipfile
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SEED = 42
TACO_SPLIT_PLAN = {
    "train": {"source": "train", "groups": set(range(1, 8)), "count": 210},
    "val": {"source": "val", "groups": set(range(8, 12)), "count": 45},
    "test": {"source": "val", "groups": set(range(12, 16)), "count": 45},
}


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def taco_group(path):
    match = re.match(r"batch_(\d+)_", path.name)
    if not match:
        raise ValueError(f"cannot determine TACO batch group: {path.name}")
    return int(match.group(1))


def deterministic_select(paths, count, namespace):
    ranked = sorted(
        paths,
        key=lambda path: hashlib.sha256(
            f"{SEED}:{namespace}:{path.name}".encode()
        ).hexdigest(),
    )
    if len(ranked) < count:
        raise ValueError(f"not enough candidates for {namespace}: {len(ranked)}/{count}")
    return ranked[:count]


def validate_label(data, source):
    text = data.decode("utf-8").strip()
    boxes = 0
    classes = Counter()
    if not text:
        return {"boxes": 0, "empty": True, "classes": classes}
    for line_number, line in enumerate(text.splitlines(), 1):
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{source}:{line_number}: expected five fields")
        class_id = int(fields[0])
        values = [float(value) for value in fields[1:]]
        if class_id != 0:
            raise ValueError(f"{source}:{line_number}: unexpected class {class_id}")
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{source}:{line_number}: bbox outside [0,1]")
        if values[2] <= 0 or values[3] <= 0:
            raise ValueError(f"{source}:{line_number}: non-positive bbox")
        boxes += 1
        classes[class_id] += 1
    return {"boxes": boxes, "empty": False, "classes": classes}


def taco_selection(taco_root):
    selected = {}
    for split, plan in TACO_SPLIT_PLAN.items():
        candidates = [
            path
            for path in (taco_root / "images" / plan["source"]).iterdir()
            if path.is_file() and taco_group(path) in plan["groups"]
        ]
        selected[split] = deterministic_select(candidates, plan["count"], split)
    return selected


def add_dataset_to_zip(archive, records, data_yaml):
    for record in records:
        archive.write(record["image_source"], record["image_archive"])
        archive.write(record["label_source"], record["label_archive"])
    archive.writestr("data.yaml", data_yaml)


def inspect_records(records):
    image_hashes = {}
    split_groups = {}
    counts = {}
    for record in records:
        image_data = record["image_source"].read_bytes()
        label_data = record["label_source"].read_bytes()
        label = validate_label(label_data, record["label_source"])
        record["image_sha256"] = sha256_bytes(image_data)
        record["label_sha256"] = sha256_bytes(label_data)
        record["box_count"] = label["boxes"]
        record["empty_label"] = label["empty"]
        image_hashes.setdefault(record["image_sha256"], []).append(record)
        split_groups.setdefault(record["group"], set()).add(record["split"])
        summary = counts.setdefault(
            record["split"], {"images": 0, "boxes": 0, "empty_labels": 0}
        )
        summary["images"] += 1
        summary["boxes"] += label["boxes"]
        summary["empty_labels"] += int(label["empty"])
    cross_split_duplicates = [
        items
        for items in image_hashes.values()
        if len({item["split"] for item in items}) > 1
    ]
    duplicate_groups = [items for items in image_hashes.values() if len(items) > 1]
    leaked_groups = {
        group: sorted(splits)
        for group, splits in split_groups.items()
        if len(splits) > 1
    }
    return {
        "counts": counts,
        "duplicate_hash_groups_total": len(duplicate_groups),
        "duplicate_hashes_across_splits": len(cross_split_duplicates),
        "leaked_groups": leaked_groups,
    }


def make_taco_records(taco_root):
    records = []
    for split, images in taco_selection(taco_root).items():
        source_split = TACO_SPLIT_PLAN[split]["source"]
        for image in images:
            label = taco_root / "labels" / source_split / f"{image.stem}.txt"
            if not label.exists():
                raise FileNotFoundError(f"label missing for {image}")
            output_stem = f"taco_{image.stem}"
            records.append(
                {
                    "domain": "taco",
                    "split": split,
                    "original_split": source_split,
                    "group": f"taco_batch_{taco_group(image)}",
                    "image_source": image,
                    "label_source": label,
                    "image_archive": f"images/{split}/{output_stem}{image.suffix}",
                    "label_archive": f"labels/{split}/{output_stem}.txt",
                }
            )
    return records


def make_gazebo_records(gazebo_root):
    manifest = json.loads((gazebo_root / "manifest.json").read_text())
    records = []
    for item in manifest["records"]:
        image = Path(item["image"])
        label = Path(item["label"])
        if not image.is_absolute():
            image = ROOT / image
        if not label.is_absolute():
            label = ROOT / label
        records.append(
            {
                "domain": "gazebo",
                "split": item["split"],
                "original_split": item["split"],
                "group": f"gazebo_{item['scene']}",
                "kind": item["kind"],
                "image_source": image,
                "label_source": label,
                "image_archive": f"images/{item['split']}/{image.name}",
                "label_archive": f"labels/{item['split']}/{label.name}",
            }
        )
    return records


def serializable_record(record):
    return {
        key: str(value) if isinstance(value, Path) else value
        for key, value in record.items()
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--taco", type=Path, default=ROOT / "taco_dataset")
    parser.add_argument(
        "--gazebo", type=Path, default=ROOT / "datasets/gazebo_litter_pilot"
    )
    parser.add_argument("--output", type=Path, default=ROOT / "colab_export")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing export: {args.output}")
    args.output.mkdir(parents=True)

    taco_records = make_taco_records(args.taco)
    gazebo_records = make_gazebo_records(args.gazebo)
    taco_quality = inspect_records(taco_records)
    gazebo_quality = inspect_records(gazebo_records)
    if taco_quality["duplicate_hashes_across_splits"] or taco_quality["leaked_groups"]:
        raise RuntimeError(f"TACO leakage detected: {taco_quality}")
    if gazebo_quality["duplicate_hashes_across_splits"] or gazebo_quality["leaked_groups"]:
        raise RuntimeError(f"Gazebo leakage detected: {gazebo_quality}")

    taco_yaml = "path: /content/taco\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: litter\n"
    gazebo_yaml = "path: /content/gazebo\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: litter\n"
    with zipfile.ZipFile(args.output / "taco.zip", "w", zipfile.ZIP_STORED) as archive:
        add_dataset_to_zip(archive, taco_records, taco_yaml)
    with zipfile.ZipFile(args.output / "gazebo.zip", "w", zipfile.ZIP_STORED) as archive:
        add_dataset_to_zip(archive, gazebo_records, gazebo_yaml)

    source_weights = ROOT / "runs/detect/taco/weights/best.pt"
    shutil.copy2(source_weights, args.output / "taco_best.pt")
    mixed_yaml = "path: /content\ntrain:\n  - taco/images/train\n  - gazebo/images/train\nval:\n  - taco/images/val\n  - gazebo/images/val\ntest:\n  - taco/images/test\n  - gazebo/images/test\nnames:\n  0: litter\n"
    (args.output / "mixed_data.yaml").write_text(mixed_yaml)

    manifest = {
        "seed": SEED,
        "class_mapping": {
            "taco_source": {"0": "litter"},
            "gazebo_source": {"0": "litter"},
            "export": {"0": "litter"},
            "conversion": "identity; no class IDs rewritten",
        },
        "taco_split_policy": {
            split: {
                "source_split": plan["source"],
                "batch_groups": sorted(plan["groups"]),
                "images": plan["count"],
            }
            for split, plan in TACO_SPLIT_PLAN.items()
        },
        "gazebo_split_policy": "preserved from scene-separated pilot manifest",
        "taco_quality": taco_quality,
        "gazebo_quality": gazebo_quality,
        "combined_counts": {
            split: taco_quality["counts"][split]["images"]
            + gazebo_quality["counts"][split]["images"]
            for split in ("train", "val", "test")
        },
        "taco_best_sha256": sha256_file(source_weights),
        "records": {
            "taco": [serializable_record(item) for item in taco_records],
            "gazebo": [serializable_record(item) for item in gazebo_records],
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    report = [
        "# Mixed dataset quality report",
        "",
        "- Export class: `0: litter` (identity mapping for both domains)",
        "- TACO selection: seed 42, batch-group-separated",
        "- Gazebo selection: existing scene-separated split preserved",
        f"- TACO cross-split duplicate hashes: {taco_quality['duplicate_hashes_across_splits']}",
        f"- Gazebo cross-split duplicate hashes: {gazebo_quality['duplicate_hashes_across_splits']}",
        f"- TACO exact duplicate hash groups (all splits): {taco_quality['duplicate_hash_groups_total']}",
        f"- Gazebo exact duplicate hash groups (all splits): {gazebo_quality['duplicate_hash_groups_total']}",
        "- Label/image correspondence: passed",
        "- Normalized bbox range and positive dimensions: passed",
        "- Class ID check (0 only): passed",
        "",
        "| domain | split | images | boxes | empty labels |",
        "|---|---|---:|---:|---:|",
    ]
    for domain, quality in (("TACO", taco_quality), ("Gazebo", gazebo_quality)):
        for split in ("train", "val", "test"):
            values = quality["counts"][split]
            report.append(
                f"| {domain} | {split} | {values['images']} | {values['boxes']} | {values['empty_labels']} |"
            )
    (args.output / "QUALITY_REPORT.md").write_text("\n".join(report) + "\n")

    readme = """# TACO + Gazebo Colab export

This directory is input-only material for mixed fine-tuning. The original datasets and weights were not modified.

## Colab setup

Upload all files in this directory to Colab or Google Drive, then run:

```python
!unzip -q taco.zip -d /content/taco
!unzip -q gazebo.zip -d /content/gazebo
!cp mixed_data.yaml /content/mixed_data.yaml

from ultralytics import YOLO
model = YOLO("taco_best.pt")
model.train(
    data="/content/mixed_data.yaml",
    imgsz=640,
    batch=8,
    epochs=50,
    patience=10,
    seed=42,
    device=0,
    project="/content/drive/MyDrive/robot-1-colab/mixed_runs",
    name="taco_gazebo_mixed",
    save_period=1,
)
```

Do not train on `images/val` or `images/test`. Evaluate TACO and Gazebo test sets separately after training; combined metrics alone can hide domain regressions.
"""
    (args.output / "README.md").write_text(readme)

    for filename in ("taco.zip", "gazebo.zip", "taco_best.pt"):
        path = args.output / filename
        print(f"{filename}: {path.stat().st_size} bytes sha256={sha256_file(path)}")
    print(json.dumps(manifest["combined_counts"], indent=2))


if __name__ == "__main__":
    main()
