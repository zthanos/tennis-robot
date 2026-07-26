#!/usr/bin/env python3
"""Train and export the required neural court-scene detector.

The input is an Ultralytics detection dataset with exactly:

    0: net
    1: fence

Example:
    uv run --with ultralytics python scripts/train_court_scene_yolo.py \
      --data datasets/court_scene/court_scene.yaml

The script validates that both classes occur in the training labels, fine-tunes
a nano YOLO model, and exports the best checkpoint to the ONNX path consumed by
``tennis_robot.perception_node``.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import shutil
import sys
import tempfile

import yaml


EXPECTED_NAMES = {0: "net", 1: "fence"}


def _normalise_names(value) -> dict[int, str]:
    if isinstance(value, list):
        return {index: str(name) for index, name in enumerate(value)}
    if isinstance(value, dict):
        return {int(index): str(name) for index, name in value.items()}
    return {}


def _dataset_root(config_path: Path, config: dict) -> Path:
    root = Path(config.get("path") or config_path.parent)
    if not root.is_absolute():
        root = (config_path.parent / root).resolve()
    return root


def _label_counts(config_path: Path, config: dict) -> Counter:
    root = _dataset_root(config_path, config)
    train_value = config.get("train")
    if not train_value:
        raise ValueError("dataset YAML must define 'train'")
    image_dir = Path(train_value)
    if not image_dir.is_absolute():
        image_dir = root / image_dir
    label_dir = Path(str(image_dir).replace("/images/", "/labels/"))
    if label_dir == image_dir:
        label_dir = root / "labels" / "train"
    counts: Counter = Counter()
    for label_path in label_dir.rglob("*.txt"):
        for line in label_path.read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if fields:
                counts[int(fields[0])] += 1
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, help="Ultralytics dataset YAML")
    parser.add_argument("--weights", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default=None)
    parser.add_argument("--project", default="runs/court_scene")
    parser.add_argument("--name", default="yolov8n_net_fence")
    parser.add_argument("--out", default="models/court_scene_yolov8n.onnx")
    parser.add_argument("--opset", type=int, default=12)
    args = parser.parse_args()

    config_path = Path(args.data).resolve()
    if not config_path.is_file():
        print(f"dataset YAML not found: {config_path}", file=sys.stderr)
        return 2
    config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    names = _normalise_names(config.get("names"))
    if names != EXPECTED_NAMES:
        print(
            f"dataset names must be exactly {EXPECTED_NAMES}; got {names}",
            file=sys.stderr,
        )
        return 2
    try:
        counts = _label_counts(config_path, config)
    except (OSError, ValueError) as exc:
        print(f"could not validate labels: {exc}", file=sys.stderr)
        return 2
    missing = sorted(set(EXPECTED_NAMES) - set(counts))
    if missing:
        print(
            f"training labels are missing class ids: {missing}; counts={dict(counts)}",
            file=sys.stderr,
        )
        return 2

    try:
        from ultralytics import YOLO
    except ImportError:
        print(
            "ultralytics is not installed; run with `uv run --with ultralytics`",
            file=sys.stderr,
        )
        return 2

    print(f"Validated dataset: counts={dict(counts)}")
    model = YOLO(args.weights)
    # Ultralytics resolves an existing relative `path:` against the process
    # working directory, while our dataset YAML is intentionally portable and
    # relative to its own location. Feed it a temporary absolute-path config so
    # training behaves identically regardless of the caller's current folder.
    resolved_config = dict(config)
    resolved_config["path"] = str(_dataset_root(config_path, config))
    with tempfile.TemporaryDirectory(prefix="court-scene-yolo-") as temp_dir:
        resolved_config_path = Path(temp_dir) / config_path.name
        resolved_config_path.write_text(
            yaml.safe_dump(resolved_config, sort_keys=False), encoding="utf-8"
        )
        train_kwargs = {
            "data": str(resolved_config_path),
            "epochs": args.epochs,
            "imgsz": args.imgsz,
            "batch": args.batch,
            "project": args.project,
            "name": args.name,
        }
        if args.device:
            train_kwargs["device"] = args.device
        result = model.train(**train_kwargs)
    save_dir = Path(result.save_dir)
    best = save_dir / "weights" / "best.pt"
    if not best.is_file():
        print(f"training completed without expected checkpoint: {best}", file=sys.stderr)
        return 1

    exported = Path(
        YOLO(str(best)).export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=True,
        )
    )
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != out.resolve():
        shutil.move(str(exported), str(out))
    print(f"Wrote required runtime model: {out}")
    print(f"export COURT_SCENE_MODEL_PATH={out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
