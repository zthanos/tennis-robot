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

## Court-scene model (`net` / `fence`)

Survey semantics use a second required model:

```text
models/court_scene_yolov8n.onnx
class 0 = net
class 1 = fence
```

Train and export it from an Ultralytics-format dataset:

```bash
# Terminal 1: simulation only, without runtime perception.
TENNIS_LAUNCH_SIM=true TENNIS_LAUNCH_BRAIN=false \
  TENNIS_PERCEPTION_ON_PC=false SIM_SKIP_CONTROL_PANEL=true \
  ros2 launch tennis_robot sim.launch.py headless:=true

# Terminal 2: capture ground-truth labels.
python3 scripts/capture_court_scene_dataset.py \
  --output datasets/court_scene --max-images 1200

# Terminal 3: deterministic balanced viewpoints.
python3 scripts/sweep_court_scene_capture.py --count 1500

# After capture completes:
uv run --with ultralytics python scripts/train_court_scene_yolo.py \
  --data datasets/court_scene/court_scene.yaml
```

The simulation-only capture tool combines Gazebo's ground-truth robot pose with
the static OAK-D mount transform and projects the known court geometry,
producing drift-free YOLO boxes while the robot is driven through varied
viewpoints. The sweep utility teleports the simulated robot through net,
end-fence, side-fence, oblique, and random-heading views. Ground truth is used
only to create labels, never by runtime
perception. Add real OAK-D frames and review labels before hardware deployment;
simulation-only training is a bootstrap, not final domain validation.

Runtime configuration:

```bash
export COURT_SCENE_MODEL_PATH=/abs/path/to/court_scene_yolov8n.onnx
export COURT_SCENE_CLASS_MAP=0:net,1:fence
export COURT_SCENE_CONF_THRESHOLD=0.45
export COURT_SCENE_CONFIRM_FRAMES=3
```

The dataset must include difficult overlap cases where the foreground net and
the perimeter fence are visible in the same frame. A missing/invalid model
fails perception startup; the old grid heuristic is not a runtime fallback.
The tracked `court_scene_yolov8n.metadata.json` records the checksum, training
provenance, and validation metrics for the locally generated ONNX artifact.
