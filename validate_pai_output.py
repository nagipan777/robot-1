"""Evaluate original, Gazebo-only, and mixed weights by domain without training."""

import argparse
import hashlib
import json
import tempfile
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
MODELS = {
    "taco_original": ROOT / "runs/detect/taco/weights/best.pt",
    "gazebo_only": ROOT / "datasets/gazebo_litter_pilot_result/weights/best.pt",
    "taco_gazebo_mixed": ROOT / "datasets/PAI_output/best.pt",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_yaml(path, dataset_root):
    path.write_text(
        f"path: {dataset_root}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "names:\n  0: litter\n"
    )


def evaluate(model_path, data_yaml, output_root, name):
    from ultralytics import YOLO

    model = YOLO(model_path)
    if model.task != "detect" or model.names != {0: "litter"}:
        raise ValueError(f"unexpected model task/classes: {model.task} {model.names}")
    metrics = model.val(
        data=str(data_yaml),
        split="test",
        imgsz=640,
        batch=2,
        device="cpu",
        workers=0,
        plots=False,
        verbose=False,
        project=str(output_root),
        name=name,
        exist_ok=True,
    )
    precision = float(metrics.box.mp)
    recall = float(metrics.box.mr)
    return {
        "precision": precision,
        "recall": recall,
        "f1": 2 * precision * recall / (precision + recall)
        if precision + recall
        else 0.0,
        "map50": float(metrics.box.map50),
        "map50_95": float(metrics.box.map),
        "preprocess_ms": float(metrics.speed["preprocess"]),
        "inference_ms": float(metrics.speed["inference"]),
        "postprocess_ms": float(metrics.speed["postprocess"]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/pai_output_validation",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    archives = {
        "taco": ROOT / "colab_export/taco.zip",
        "gazebo": ROOT / "colab_export/gazebo.zip",
    }
    results = []
    with tempfile.TemporaryDirectory(prefix="pai-output-validation-") as temporary:
        temporary = Path(temporary)
        datasets = {}
        for domain, archive_path in archives.items():
            dataset_root = temporary / domain
            with zipfile.ZipFile(archive_path) as archive:
                archive.extractall(dataset_root)
            data_yaml = dataset_root / "data.local.yaml"
            write_yaml(data_yaml, dataset_root)
            datasets[domain] = data_yaml
        for model_name, model_path in MODELS.items():
            for domain, data_yaml in datasets.items():
                print(f"validating {model_name} on {domain} test", flush=True)
                result = evaluate(
                    model_path,
                    data_yaml,
                    args.output / "ultralytics",
                    f"{model_name}_{domain}",
                )
                results.append(
                    {
                        "model": model_name,
                        "model_path": str(model_path),
                        "model_sha256": sha256(model_path),
                        "domain": domain,
                        "split": "test",
                        **result,
                    }
                )
    (args.output / "domain_comparison.json").write_text(
        json.dumps(results, indent=2) + "\n"
    )
    lines = [
        "# PAI mixed-model domain validation",
        "",
        "All rows use the same exported test splits and `imgsz=640`.",
        "",
        "| model | domain | precision | recall | F1 | mAP50 | mAP50-95 | inference ms/image |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            f"| {result['model']} | {result['domain']} | "
            f"{result['precision']:.3f} | {result['recall']:.3f} | "
            f"{result['f1']:.3f} | {result['map50']:.3f} | "
            f"{result['map50_95']:.3f} | {result['inference_ms']:.1f} |"
        )
    (args.output / "REPORT.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
