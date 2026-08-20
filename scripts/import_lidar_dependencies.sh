#!/usr/bin/env bash
# Import the pinned LiDAR driver source without vendoring it in this repository.
# Safe to re-run: an existing clean checkout at the pin is left untouched.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_DIR="$ROOT_DIR/ros2_ws/src"
DRIVER_DIR="$SOURCE_DIR/sllidar_ros2"
PIN="34300099fadfc772965962dec837bf436706188f"

if [ -d "$DRIVER_DIR/.git" ]; then
    if ! git -C "$DRIVER_DIR" diff --quiet || \
       ! git -C "$DRIVER_DIR" diff --cached --quiet; then
        echo "[lidar-deps] ERROR: $DRIVER_DIR has local changes; refusing to overwrite." >&2
        exit 1
    fi
    CURRENT="$(git -C "$DRIVER_DIR" rev-parse HEAD)"
    if [ "$CURRENT" != "$PIN" ]; then
        echo "[lidar-deps] updating clean checkout to pinned commit $PIN"
        git -C "$DRIVER_DIR" fetch origin "$PIN"
        git -C "$DRIVER_DIR" checkout --detach "$PIN"
    fi
else
    command -v vcs >/dev/null 2>&1 || {
        echo "[lidar-deps] ERROR: vcs is required (Ubuntu package: python3-vcstool)." >&2
        exit 1
    }
    echo "[lidar-deps] importing pinned sllidar_ros2 source"
    vcs import "$SOURCE_DIR" < "$ROOT_DIR/ros2_ws/lidar.repos"
fi

ACTUAL="$(git -C "$DRIVER_DIR" rev-parse HEAD)"
if [ "$ACTUAL" != "$PIN" ]; then
    echo "[lidar-deps] ERROR: expected $PIN, found $ACTUAL" >&2
    exit 1
fi
echo "[lidar-deps] sllidar_ros2 pinned at $ACTUAL"
