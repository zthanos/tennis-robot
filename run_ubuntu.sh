#!/usr/bin/env bash
# Native Ubuntu Docker bring-up. The existing run.sh remains the WSL 2 path.
#
# Gazebo GUI + RViz with software rendering (safe default):
#   ./run_ubuntu.sh
#
# Native Intel/AMD DRI acceleration:
#   UBUNTU_GPU=true ./run_ubuntu.sh foxglove
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-false}"
export SLAM_MODE="${SLAM_MODE:-localization}"
export NAV2_START_DELAY_S="${NAV2_START_DELAY_S:-25}"

if [ "$GAZEBO_HEADLESS" != "true" ]; then
    if [ -z "${DISPLAY:-}" ]; then
        echo "ERROR: DISPLAY is not set. Start this script from the Ubuntu desktop session."
        exit 1
    fi

    if [ -z "${XAUTHORITY:-}" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
        for candidate in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
            if [ -f "$candidate" ]; then
                export XAUTHORITY="$candidate"
                break
            fi
        done
    fi

    if [ ! -r "${XAUTHORITY:-}" ]; then
        echo "ERROR: Active X11 authorization cookie is not readable."
        echo "Set XAUTHORITY to the current desktop session cookie and retry."
        exit 1
    fi
fi

MODEL_PATH="${BALL_MODEL_PATH:-$SCRIPT_DIR/models/yolov8n.onnx}"
if [ ! -s "$MODEL_PATH" ]; then
    echo "ERROR: Neural perception model is missing or empty: $MODEL_PATH"
    echo "Create it with:"
    echo "  uv run --with ultralytics python scripts/export_yolo_onnx.py"
    exit 1
fi

COMPOSE=(
    docker compose
    -f docker-compose.yml
    -f docker-compose.ubuntu.yml
)

if [ "${UBUNTU_GPU:-false}" = "true" ]; then
    if [ ! -d /dev/dri ]; then
        echo "ERROR: UBUNTU_GPU=true requires /dev/dri on the host."
        exit 1
    fi
    COMPOSE+=(-f docker-compose.ubuntu-gpu.yml)
fi

SERVICES=(gazebo)
if [ "$GAZEBO_HEADLESS" != "true" ]; then
    SERVICES+=(rviz)
fi

"${COMPOSE[@]}" --profile gazebo down
"${COMPOSE[@]}" --profile gazebo up "${SERVICES[@]}" "$@"
