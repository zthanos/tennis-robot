# Simulated OAK-D AI Perception Pipeline

This document describes how the perception layer emulates the OAK-D's on-device
AI in simulation, and why the contract it publishes is identical to the one the
real OAK-D will expose — so the Collector, Nav2 and Behaviour Tree never depend
on whether a detection came from Gazebo or from hardware.

## Goal

The real OAK-D runs a neural network *on the camera* (DepthAI) and streams ball
detections with spatial coordinates. In simulation there is no OAK-D — Gazebo
only gives a plain RGB image and an aligned depth image. The perception node
closes that gap by running the *same kind* of pipeline a real OAK-D runs
internally, treating Gazebo purely as the image source.

The pipeline does **not** use HSV colour thresholding. The neural model is
mandatory; a missing or invalid model fails perception startup.

## Pipeline

```text
Gazebo RGB  /camera/image_raw ─┐
                               ├─▶ neural detector (YOLOv8/v11n, ONNX Runtime)
                               │        │  2D boxes + class score
Gazebo depth /camera/depth ────┴─▶ depth fusion (per-box ROI percentile)
                                        │  bearing + distance + spatial XYZ
                                        ▼
                     ┌──────────────────┴───────────────────────┐
                     ▼                                            ▼
                         /perception/ball_detections
                         (BallDetectionArray — canonical contract)
                         /perception/diagnostics
                         (operator-only JSON; never a target source)
```

Stages (`tennis_robot/perception_node.py`, `tennis_robot/ball_detector.py`):

1. **Synchronize** RGB and depth by acquisition timestamp (bounded approximate
   synchronization), then decode the RGB frame (bgra8 / rgba8 / rgb8 / bgr8
   → BGR). Frames are never paired with an arbitrary "latest" depth image.
2. **Detect** tennis balls with `YoloOnnxBallDetector`: letterbox to the model
   input, run the ONNX model, decode the YOLOv8 head, keep the target class
   (COCO 32 "sports ball" by default), Non-Max-Suppress, map boxes back to
   image pixels. Output: `BallDetection` objects (pixel bbox + confidence).
3. **Fuse with depth** (`estimate_depth_ball_observation`): sample the aligned
   depth image inside each box (20th-percentile to favour the near ball
   surface over background), derive `distance_m`; derive `bearing_rad` from the
   horizontal pixel offset and camera FOV; derive camera-frame `position_x/y/z`.
4. **Publish** the camera-relative canonical contract. The controller combines
   it with its authoritative SLAM/odom pose when world coordinates are needed.

## Published contract (identical sim ↔ real)

| Topic | Type | Purpose |
| --- | --- | --- |
| `/perception/ball_detections` | `tennis_robot_msgs/BallDetectionArray` | Structured multi-ball detections mirroring DepthAI `SpatialDetectionArray`. |
| `/perception/diagnostics` | `std_msgs/String` (JSON) | Per-frame counts, measured depth range and explicit spatial-rejection reasons for operators/tests. It is not consumed as ball data. |
| `/survey/vision` | `std_msgs/String` (JSON) | Court-line / junction survey vision (unchanged). |

`BallDetectionArray` carries a `std_msgs/Header` (stamp +
`camera_link_optical_frame`) and a list of `BallDetection` (confidence, 2D bbox,
and — when depth was valid — `bearing_rad`, `distance_m`, optical-frame
`position_x/y/z`). Optical XYZ follows REP-103 and DepthAI: right, down,
forward. `bearing_rad` follows robot/navigation convention: positive left/CCW.
This is the entry the real OAK-D adapter will populate, so swapping the
simulated detector for the on-device network requires no downstream change.

### Τρία διαφορετικά όρια εμβέλειας

Δεν πρέπει να συγχέονται:

1. το depth range που μπορεί να μετρήσει ο αισθητήρας,
2. το range στο οποίο το neural detector αναγνωρίζει αξιόπιστα μπάλα στην
   επιλεγμένη RGB ανάλυση,
3. το range για το οποίο υπάρχει accepted covariance calibration evidence.

Ένα έγκυρο depth pixel δεν αρκεί για `has_spatial=true`: απαιτούνται neural
detection και calibrated covariance για το συγκεκριμένο range/depth-quality.
Το ενεργό Gazebo v3 artifact καλύπτει `1.0218–6.7653 m`, βάσει 540 accepted
target samples. Το pilot στα `8.263 m` δεν έδωσε αξιόπιστη target detection με
το stock YOLO, παρότι το depth sensor ήταν εντός του ονομαστικού ορίου. Άρα το
operational simulated perception range είναι περίπου `1.02–6.77 m`, όχι 9 m.
Το physical OAK-D απαιτεί ανεξάρτητο hardware artifact και validation του
on-device neural model.

## Sim ↔ real parity

The only thing that differs between sim and the physical robot is *who detects*:

```text
SIM:   Gazebo RGB+depth ─▶ perception_node (YOLO ONNX on CPU) ─▶ contract topics
REAL:  OAK-D on-device YOLO (DepthAI) ──────────────────────▶ contract topics
```

On the real robot the on-device network publishes the same
`BallDetectionArray` (via a thin DepthAI adapter), so `controller_node`,
`collector`, Nav2 and the Behaviour Tree are untouched. This mirrors the sensor
parity already established in `docs/hardware/sensor-topic-contract-el.md`, where the
driver only has to match topic name, `frame_id` and message type.

## Configuration

Environment variables (read by `ball_detector.load_ball_detector` and the node):

| Var | Default | Meaning |
| --- | --- | --- |
| `BALL_DETECTOR_BACKEND` | `yolo_onnx` | Must be `yolo_onnx`; other values are rejected |
| `BALL_MODEL_PATH` | `models/yolov8n.onnx` | ONNX model file |
| `BALL_CONF_THRESHOLD` | `0.35` | Min class score to keep |
| `BALL_IOU_THRESHOLD` | `0.45` | NMS IoU threshold |
| `BALL_CLASS_IDS` | `32` | Kept COCO class ids (`0` for a single-class custom model) |
| `BALL_CENTER_ZOOM_FACTOR` | `3.0` (sim) | Neural crop scale for small/far balls |
| `BALL_CENTER_ZOOM_TILES` | `0.30:0.333,0.50:0.333,0.70:0.333` (sim) | Normalized neural tile centres covering the scan sector |
| `CAMERA_FOV_RAD` | `1.204` (sim) | Horizontal FOV, matches `oak_d.urdf.xacro` |
| `CAMERA_FRAME_ID` | `camera_link_optical_frame` | REP-103 optical frame for the detection array |
| `RGB_DEPTH_SYNC_SLOP_S` | `0.05` | Maximum timestamp difference for an RGB/depth pair |
| `RGB_DEPTH_SYNC_QUEUE_SIZE` | `10` | Buffered messages available to the synchronizer |

Every synchronized acquisition publishes an observation and detection array,
including empty results. `controller_node` expires perception observations
after `PERCEPTION_OBSERVATION_TIMEOUT_S` (default `1.0`) if publishing stops.

The model is **not** committed (large, reproducible). Export it once:

```bash
uv run --with ultralytics python scripts/export_yolo_onnx.py   # -> models/yolov8n.onnx
```

If the model file is absent or invalid, the node fails startup. Runtime
inference uses ONNX Runtime only (no PyTorch on the robot).

## Why ONNX Runtime (not PyTorch/ultralytics)

ONNX Runtime is a small CPU-friendly wheel that installs cleanly into the ROS 2
Humble image (`Dockerfile.ros2`, `Dockerfile.gazebo`) and onto the robot's SBC.
A stock `yolov8n.onnx` already detects "sports ball", so the pipeline stands up
with no custom training; a purpose-trained single-class model drops in by
setting `BALL_MODEL_PATH` + `BALL_CLASS_IDS=0`.

## Tests

`tests/test_ball_detector.py` runs offline (no ROS, no model file, no GPU) by
injecting a fake ONNX session. It covers NMS, the YOLOv8 decode (both output
layouts), class/confidence filtering, letterbox box-remapping, factory
degradation, and the depth-fusion geometry (bearing sign, distance, camera-frame
axis conventions).
```bash
uv run pytest tests/test_ball_detector.py
```
