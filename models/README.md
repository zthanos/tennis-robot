# Perception models

The simulated OAK-D AI pipeline (`tennis_robot.perception_node`) loads its
neural ball detector from here by default:

    models/yolov8n.onnx

## Get the model

```bash
# Stock YOLOv8n (COCO) — detects class 32 "sports ball", covers tennis balls.
uv run --with ultralytics python scripts/export_yolo_onnx.py
```

This needs `ultralytics`/`torch` only at export time; the robot and the ROS
image run the `.onnx` through ONNX Runtime (CPU), no PyTorch required.

## Point the node at a different file

```bash
export BALL_MODEL_PATH=/abs/path/to/your_model.onnx
export BALL_CLASS_IDS=0        # e.g. a single-class custom tennis-ball model
export BALL_CONF_THRESHOLD=0.35
```

If the model file is missing or invalid, perception fails startup. There is no
classical detector fallback.

The `.onnx` weights are intentionally **git-ignored** (they are large and
reproducible); commit only this README.
