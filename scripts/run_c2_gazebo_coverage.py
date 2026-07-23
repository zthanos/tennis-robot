#!/usr/bin/env python3
"""Run C2 controlled trials against an already-running headless Gazebo stack.

It only writes raw evidence and coverage inputs: no artifact builder, producer
configuration, or spatial-target activation is invoked here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ["docker", "compose", "-f", "docker-compose.yml", "-f", "docker-compose.ubuntu.yml"]


def _exec(command: str, *, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        COMPOSE + ["exec", "-T", "gazebo", "bash", "-lc", command],
        cwd=ROOT, check=True, text=True, capture_output=capture,
    )


def _set_pose_request(pose: dict) -> str:
    """Build the Gazebo pose request, including the manifest's declared yaw."""
    yaw = float(pose["yaw_rad"])
    half = yaw / 2.0
    return (
        'name: "tennis_robot", position: {'
        f'x: {pose["x_m"]}, y: {pose["y_m"]}, z: 0.09}}, orientation: {{'
        f'z: {math.sin(half)}, w: {math.cos(half)}}}'
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=ROOT / "config/gazebo_covariance_c2_trials.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "runtime/c2_controlled_coverage")
    parser.add_argument("--trial-id", help="Run exactly one manifest trial.")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    output_dir = args.output_dir.resolve()
    try:
        container_output_dir = Path("/workspace") / output_dir.relative_to(ROOT)
    except ValueError as exc:
        raise SystemExit("--output-dir must resolve inside the repository") from exc
    output_dir.mkdir(parents=True, exist_ok=True)
    trials = manifest["trials"]
    if args.trial_id:
        trials = [trial for trial in trials if trial["id"] == args.trial_id]
        if not trials:
            raise SystemExit(f"unknown trial ID: {args.trial_id}")
    for trial in trials:
        pose = trial["robot_pose"]
        request = _set_pose_request(pose)
        result = _exec(
            "gz service -s /world/tennis_court/set_pose --reqtype gz.msgs.Pose "
            f"--reptype gz.msgs.Boolean --timeout 3000 --req '{request}'",
            capture=True,
        )
        if "data: true" not in result.stdout:
            raise RuntimeError(f"set-pose did not complete for {trial['id']}: {result.stdout}")
        time.sleep(5.0)
        evidence = str(container_output_dir / f"{trial['id']}.jsonl")
        requested_pose_json = json.dumps(pose, separators=(",", ":"))
        _exec(
            "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install_docker/setup.bash && "
            "export BALL_MODEL_PATH=/workspace/models/yolov8n.onnx "
            "BALL_CENTER_ZOOM_FACTOR=${BALL_CENTER_ZOOM_FACTOR:-3.0} "
            "BALL_CENTER_ZOOM_TILES=${BALL_CENTER_ZOOM_TILES:-0.30:0.333,0.50:0.333,0.70:0.333} "
            f"GAZEBO_COVARIANCE_EVIDENCE_PATH={evidence} "
            f"GAZEBO_COVARIANCE_TRIAL_ID={trial['id']} "
            f"GAZEBO_COVARIANCE_TARGET_BALL_ID={trial['ball_id']} "
            f"GAZEBO_COVARIANCE_MASK_MISSING_RATIO={trial['missing_pixel_ratio']} "
            f"GAZEBO_COVARIANCE_MASK_SEED={trial['mask_seed']} "
            f"GAZEBO_COVARIANCE_MAX_ACCEPTED={manifest['minimum_accepted_samples_per_bin']} && "
            f"export GAZEBO_COVARIANCE_ASSOCIATION_GATE_M={manifest['association_gate_m']} "
            f"GAZEBO_COVARIANCE_ASSOCIATION_AMBIGUITY_MARGIN_M={manifest['association_ambiguity_margin_m']} "
            f"GAZEBO_COVARIANCE_TARGET_RESIDUAL_OUTLIER_THRESHOLD_M={manifest['target_residual_outlier_threshold_m']} && "
            f"export GAZEBO_COVARIANCE_REQUESTED_POSE_JSON='{requested_pose_json}' "
            f"GAZEBO_COVARIANCE_NOMINAL_RANGE_BIN={trial['range_bin']} && "
            "timeout 180s ros2 run tennis_robot gazebo_covariance_recorder"
        )
        summary_path = output_dir / f"{trial['id']}.jsonl.summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        if summary.get("rejected", {}).get("target_range_not_in_bin"):
            raise RuntimeError(
                f"target_range_not_in_bin for {trial['id']}: "
                f"{summary['readiness'].get('target_range')}"
            )


if __name__ == "__main__":
    main()
