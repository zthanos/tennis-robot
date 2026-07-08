#!/usr/bin/env python3
"""Diagnose the simulated OAK-D detector on a single frame.

Answers one question: *does the YOLO model actually see anything in the image,
and as what class?* — so we can tell a model problem (sim balls not recognised)
apart from a pipeline problem (perception not wired up).

It runs the model with **no class filter** and a low confidence floor, then prints
every detection with its COCO class name + score + bbox. The tennis ball maps to
COCO class 32 ("sports ball"); if the model only fires as some other class (or not
at all) on the Gazebo balls, you'll see it here.

Usage
-----
    # Live: grab one frame from the camera topic (ROS must be sourced + sim up)
    python3 scripts/check_perception_frame.py

    # Offline: test a saved image instead of the camera
    python3 scripts/check_perception_frame.py --image /path/to/frame.png

    # Tuning
    python3 scripts/check_perception_frame.py --conf 0.05 --topk 15 \
        --model models/yolov8n.onnx --topic /camera/image_raw
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

# COCO-80 class names (index == class id). 32 == "sports ball".
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]
SPORTS_BALL_ID = 32


def decode_image(msg) -> np.ndarray | None:
    import cv2

    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding
    if enc == "bgra8":
        return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_RGBA2BGR)
    if enc == "rgb8":
        return cv2.cvtColor(arr.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
    if enc == "bgr8":
        return arr.reshape((msg.height, msg.width, 3))
    print(f"  ! unsupported encoding: {enc}")
    return None


def grab_frame_from_topic(topic: str, timeout_s: float) -> np.ndarray | None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("check_perception_frame")
    holder: dict[str, object] = {}

    def _cb(msg: Image) -> None:
        holder["msg"] = msg

    node.create_subscription(Image, topic, _cb, 1)
    print(f"Waiting up to {timeout_s:.0f}s for a frame on {topic} ...")
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while "msg" not in holder and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "msg" not in holder:
        print(f"  ! no frame received on {topic} (is the sim/bridge running?)")
        return None
    return decode_image(holder["msg"])


def decode_all_classes(det, frame, conf):
    """Run the model and return [(class_id, score, (x1,y1,x2,y2)), ...] for ALL classes."""
    blob, scale, pad = det._preprocess(frame)
    raw = det._session.run(None, {det._input_name: blob})[0]
    pred = np.asarray(raw)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T  # -> (N, 4+num_classes)
    boxes = pred[:, :4]
    scores_all = pred[:, 4:]
    class_ids = np.argmax(scores_all, axis=1)
    scores = scores_all[np.arange(scores_all.shape[0]), class_ids]
    mask = scores >= conf
    h0, w0 = frame.shape[:2]
    out = []
    for cx, cy, w, h, cid, sc in zip(
        boxes[mask, 0], boxes[mask, 1], boxes[mask, 2], boxes[mask, 3],
        class_ids[mask], scores[mask],
    ):
        x1 = (cx - w / 2 - pad[0]) / scale
        y1 = (cy - h / 2 - pad[1]) / scale
        x2 = (cx + w / 2 - pad[0]) / scale
        y2 = (cy + h / 2 - pad[1]) / scale
        x1 = max(0, min(w0 - 1, x1)); x2 = max(0, min(w0 - 1, x2))
        y1 = max(0, min(h0 - 1, y1)); y2 = max(0, min(h0 - 1, y2))
        out.append((int(cid), float(sc), (x1, y1, x2, y2)))
    out.sort(key=lambda r: r[1], reverse=True)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", help="Test a saved image file instead of the camera topic")
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--model", default=os.getenv("BALL_MODEL_PATH", "models/yolov8n.onnx"))
    ap.add_argument("--conf", type=float, default=0.05, help="Confidence floor for the diagnostic dump")
    ap.add_argument("--topk", type=int, default=15)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--save", default="runtime/perception_frame_annotated.png")
    args = ap.parse_args()

    import cv2

    if not os.path.isfile(args.model):
        print(f"  ! model not found: {args.model}\n    export it: uv run --with ultralytics python scripts/export_yolo_onnx.py")
        return 2

    # --- get a frame ---
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"  ! could not read image: {args.image}")
            return 2
        print(f"Loaded image {args.image} ({frame.shape[1]}x{frame.shape[0]})")
    else:
        frame = grab_frame_from_topic(args.topic, args.timeout)
        if frame is None:
            return 1
        print(f"Got frame {frame.shape[1]}x{frame.shape[0]}")

    # --- load detector + run all-class decode ---
    from tennis_robot.ball_detector import YoloOnnxBallDetector

    det = YoloOnnxBallDetector(args.model, conf_threshold=args.conf, class_ids=())
    dets = decode_all_classes(det, frame, args.conf)

    print(f"\n=== Top {min(args.topk, len(dets))} detections (conf >= {args.conf}, ALL classes) ===")
    if not dets:
        print("  (nothing detected — the model fired on no class above the floor)")
    for cid, sc, (x1, y1, x2, y2) in dets[:args.topk]:
        name = COCO_NAMES[cid] if 0 <= cid < len(COCO_NAMES) else f"id{cid}"
        flag = "  <-- SPORTS BALL" if cid == SPORTS_BALL_ID else ""
        print(f"  {name:>14s} ({cid:2d})  conf={sc:.3f}  box=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}){flag}")

    balls = [d for d in dets if d[0] == SPORTS_BALL_ID]
    print("\n=== Verdict ===")
    if balls:
        print(f"  ✅ model detects {len(balls)} sports-ball(s); best conf={balls[0][1]:.3f}")
        print(f"     Set BALL_CONF_THRESHOLD below {balls[0][1]:.2f} so perception keeps them.")
    elif dets:
        top = dets[0]
        nm = COCO_NAMES[top[0]] if top[0] < len(COCO_NAMES) else f"id{top[0]}"
        print(f"  ⚠️  model sees objects but NOT as 'sports ball' (top: {nm} {top[1]:.3f}).")
        print("     The sim balls likely need a fine-tuned single-class model"
              " (then BALL_CLASS_IDS=0).")
    else:
        print("  ❌ model detects nothing. Lower --conf, check the frame has a ball,"
              " or fine-tune on Gazebo frames.")

    # --- annotate + save ---
    vis = frame.copy()
    for cid, sc, (x1, y1, x2, y2) in dets[:args.topk]:
        color = (0, 0, 255) if cid == SPORTS_BALL_ID else (0, 200, 255)
        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        nm = COCO_NAMES[cid] if cid < len(COCO_NAMES) else str(cid)
        cv2.putText(vis, f"{nm} {sc:.2f}", (int(x1), max(12, int(y1) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
    cv2.imwrite(args.save, vis)
    print(f"\nAnnotated frame saved to {args.save}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
