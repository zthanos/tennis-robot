#!/usr/bin/env bash
# Import the pinned LiDAR driver source without vendoring it in this repository.
# Safe to re-run: the only accepted local change is the verified repository
# shutdown patch applied below.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_DIR="$ROOT_DIR/ros2_ws/src"
DRIVER_DIR="$SOURCE_DIR/sllidar_ros2"
PIN="34300099fadfc772965962dec837bf436706188f"
PATCH_FILE="$ROOT_DIR/ros2_ws/patches/sllidar_ros2-clean-shutdown.patch"
PATCHED_SOURCE="$DRIVER_DIR/src/sllidar_node.cpp"
PATCHED_SOURCE_SHA256="133df72431aec2bec0f450a7fbf43780f47a1aa1539b60c1b694c4a461d82dd6"

if [ -d "$DRIVER_DIR/.git" ]; then
    CURRENT="$(git -C "$DRIVER_DIR" rev-parse HEAD)"
    if [ "$CURRENT" != "$PIN" ]; then
        if ! git -C "$DRIVER_DIR" diff --quiet || \
           ! git -C "$DRIVER_DIR" diff --cached --quiet; then
            echo "[lidar-deps] ERROR: $DRIVER_DIR has local changes; refusing to overwrite." >&2
            exit 1
        fi
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

if ! git -C "$DRIVER_DIR" diff --cached --quiet; then
    echo "[lidar-deps] ERROR: $DRIVER_DIR has staged local changes; refusing to continue." >&2
    exit 1
fi

if git -C "$DRIVER_DIR" diff --quiet; then
    echo "[lidar-deps] applying repository-managed clean-shutdown patch"
    git -C "$DRIVER_DIR" apply "$PATCH_FILE"
else
    CHANGED="$(git -C "$DRIVER_DIR" diff --name-only)"
    if [ "$CHANGED" != "src/sllidar_node.cpp" ]; then
        echo "[lidar-deps] ERROR: $DRIVER_DIR has unexpected local changes; refusing to continue." >&2
        exit 1
    fi
fi

echo "$PATCHED_SOURCE_SHA256  $PATCHED_SOURCE" | sha256sum --check --status || {
    echo "[lidar-deps] ERROR: patched sllidar_node.cpp does not match the repository-managed source hash." >&2
    exit 1
}

echo "[lidar-deps] sllidar_ros2 pinned at $ACTUAL with verified clean-shutdown patch"
