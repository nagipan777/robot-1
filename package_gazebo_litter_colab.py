"""Create a reproducible Colab bundle without changing the original weights."""

import argparse
import hashlib
import json
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, default=ROOT / "datasets/gazebo_litter_pilot")
    parser.add_argument("--weights", type=Path, default=ROOT / "runs/detect/taco/weights/best.pt")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/gazebo_litter_pilot_colab.zip")
    args = parser.parse_args()
    if not (args.dataset / "manifest.json").exists():
        raise FileNotFoundError("dataset manifest is missing")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    provenance = {
        "original_weights_name": "original_best.pt",
        "original_weights_sha256": digest(args.weights),
        "dataset_manifest_sha256": digest(args.dataset / "manifest.json"),
    }
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(args.dataset.rglob("*")):
            if path.is_file():
                archive.write(path, Path("gazebo_litter_pilot") / path.relative_to(args.dataset))
        archive.write(args.weights, "weights/original_best.pt")
        archive.write(ROOT / "evaluate_yolo_holdout.py", "tools/evaluate_yolo_holdout.py")
        archive.writestr("provenance.json", json.dumps(provenance, indent=2) + "\n")
    print(json.dumps({**provenance, "zip": str(args.output), "bytes": args.output.stat().st_size}, indent=2))


if __name__ == "__main__":
    main()
