#!/usr/bin/env python3
"""Teleport the Gazebo robot through varied net/fence training viewpoints.

Run this while ``capture_court_scene_dataset.py`` is subscribed to the camera.
The sweep is deterministic for a given seed and is only intended for offline
simulation dataset generation.
"""

from __future__ import annotations

import argparse
import math
import random
import subprocess
import time


def _look_at(
    x: float,
    y: float,
    target_x: float,
    target_y: float,
    jitter: float,
    rng: random.Random,
) -> tuple[float, float, float]:
    yaw = math.atan2(target_y - y, target_x - x)
    return x, y, yaw + rng.uniform(-jitter, jitter)


def build_poses(count: int, seed: int) -> list[tuple[float, float, float]]:
    """Build a balanced mixture of net, end-fence, side-fence and background views."""

    rng = random.Random(seed)
    poses: list[tuple[float, float, float]] = []
    for index in range(count):
        group = index % 10
        if group < 4:
            # Foreground net with the opposite end fence often visible behind it.
            side = -1.0 if index % 2 == 0 else 1.0
            x = side * rng.uniform(1.2, 14.0)
            y = rng.uniform(-6.8, 6.8)
            poses.append(
                _look_at(x, y, 0.0, rng.uniform(-3.5, 3.5), 0.24, rng)
            )
        elif group < 6:
            # East/west end fences, including oblique viewpoints.
            target_x = -16.5 if index % 2 == 0 else 16.5
            x = rng.uniform(-13.5, 13.5)
            y = rng.uniform(-7.0, 7.0)
            poses.append(
                _look_at(x, y, target_x, rng.uniform(-6.5, 6.5), 0.28, rng)
            )
        elif group < 8:
            # North/south side fences and gate panels.
            target_y = -8.5 if index % 2 == 0 else 8.5
            x = rng.uniform(-14.5, 14.5)
            y = rng.uniform(-6.5, 6.5)
            poses.append(
                _look_at(x, y, rng.uniform(-14.5, 14.5), target_y, 0.28, rng)
            )
        else:
            # Random headings supply capped negative/background examples.
            poses.append(
                (
                    rng.uniform(-14.5, 14.5),
                    rng.uniform(-7.0, 7.0),
                    rng.uniform(-math.pi, math.pi),
                )
            )
    rng.shuffle(poses)
    return poses


def set_pose(
    world: str,
    pose: tuple[float, float, float],
    *,
    attempts: int = 3,
) -> None:
    x, y, yaw = pose
    request = (
        'name: "tennis_robot", '
        f"position: {{x: {x:.6f}, y: {y:.6f}, z: 0.09}}, "
        f"orientation: {{z: {math.sin(yaw / 2.0):.9f}, "
        f"w: {math.cos(yaw / 2.0):.9f}}}"
    )
    command = [
        "gz",
        "service",
        "-s",
        f"/world/{world}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "3000",
        "--req",
        request,
    ]
    for attempt in range(1, attempts + 1):
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode == 0 and "data: true" in result.stdout:
            return
        if attempt < attempts:
            time.sleep(0.5 * attempt)
    raise RuntimeError(
        "set_pose failed after "
        f"{attempts} attempts: {result.stdout.strip()} {result.stderr.strip()}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=900)
    parser.add_argument("--delay-s", type=float, default=0.28)
    parser.add_argument("--seed", type=int, default=20260726)
    parser.add_argument("--world", default="tennis_court")
    args = parser.parse_args()
    if args.count < 1 or args.delay_s <= 0.0:
        parser.error("--count and --delay-s must be positive")

    poses = build_poses(args.count, args.seed)
    for index, pose in enumerate(poses, start=1):
        set_pose(args.world, pose)
        time.sleep(args.delay_s)
        if index % 25 == 0 or index == len(poses):
            print(f"swept {index}/{len(poses)} poses", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
