#!/usr/bin/env python3
"""Export a YOLOv8/YOLOv11-nano detector to ONNX for the perception node.

The perception node (`tennis_robot.perception_node`) runs its neural ball
detector through ONNX Runtime so the robot does not need PyTorch at runtime.
This one-off helper produces the `.onnx` file it loads.

Usage
-----
    # default: yolov8n (COCO) -> models/yolov8n.onnx, 640x640
    uv run python scripts/export_yolo_onnx.py

    # a custom-trained single-class tennis-ball model
    uv run python scripts/export_yolo_onnx.py --weights runs/train/best.pt \
        --out models/tennis_ball.onnx

This script needs `ultralytics` (and torch) installed in the *export*
environment only — they are NOT required on the robot. Install on demand:

    uv run --with ultralytics python scripts/export_yolo_onnx.py

Stock yolov8n detects COCO class 32 ("sports ball"), which covers tennis balls,
so the pipeline runs end-to-end with no training. Set BALL_CLASS_IDS=32 (the
default) for that model, or BALL_CLASS_IDS=0 for a single-class custom model.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="yolov8n.pt",
                        help="Ultralytics weights (.pt) or model name (default: yolov8n.pt)")
    parser.add_argument("--out", default="models/yolov8n.onnx",
                        help="Output .onnx path (default: models/yolov8n.onnx)")
    parser.add_argument("--imgsz", type=int, default=640, help="Square input size")
    parser.add_argument("--opset", type=int, default=12, help="ONNX opset version")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print(
            "ultralytics is not installed in this environment.\n"
            "Run:  uv run --with ultralytics python scripts/export_yolo_onnx.py",
            file=sys.stderr,
        )
        return 2

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    exported = model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=True)

    exported_path = Path(exported)
    if exported_path.resolve() != out_path.resolve():
        shutil.move(str(exported_path), str(out_path))

    print(f"Wrote {out_path} (imgsz={args.imgsz}, opset={args.opset})")
    print("Point the perception node at it with BALL_MODEL_PATH, e.g.:")
    print(f"  export BALL_MODEL_PATH={out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
