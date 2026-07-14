#!/usr/bin/env bash
# Native Ubuntu Docker bring-up. The existing run.sh remains the WSL 2 path.
#
# Gazebo GUI + RViz with native Intel/AMD DRI acceleration:
#   ./run_ubuntu.sh
#
# Software rendering fallback:
#   UBUNTU_GPU=false ./run_ubuntu.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-false}"
export UBUNTU_GPU="${UBUNTU_GPU:-true}"
export SLAM_MODE="${SLAM_MODE:-localization}"
export NAV2_START_DELAY_S="${NAV2_START_DELAY_S:-25}"
# Dual-wheel intake (docs/dual-wheel-intake-design-el.md). Fallbacks are the
# Phase 1-4 bench-proven geometry (debug-log #41-#42) and must match the
# defaults in scripts/generate_robot_urdf.py / generate_curved_scoop_mesh.py.
# All sweep-able. Validation-phase gates: INTAKE_ENABLE_FUNNEL / _RAMP.
export INTAKE_WHEEL_RADIUS_M="${INTAKE_WHEEL_RADIUS_M:-0.060}"
export INTAKE_WHEEL_GAP_M="${INTAKE_WHEEL_GAP_M:-0.056}"
export INTAKE_NIP_X_M="${INTAKE_NIP_X_M:-0.540}"
export INTAKE_WHEEL_TILT_DEG="${INTAKE_WHEEL_TILT_DEG:-35.0}"
export INTAKE_WHEEL_SPRING_K="${INTAKE_WHEEL_SPRING_K:-1000}"
export INTAKE_LIP_RAISE_M="${INTAKE_LIP_RAISE_M:-0.0}"
export INTAKE_ENABLE_FUNNEL="${INTAKE_ENABLE_FUNNEL:-true}"
export INTAKE_ENABLE_RAMP="${INTAKE_ENABLE_RAMP:-true}"
export INTAKE_RAMP_PROFILE="${INTAKE_RAMP_PROFILE:-launch}"
if [ "$INTAKE_RAMP_PROFILE" = "launch" ]; then
    export INTAKE_RAMP_ENTRY_X_M="${INTAKE_RAMP_ENTRY_X_M:-$INTAKE_NIP_X_M}"
else
    export INTAKE_RAMP_ENTRY_X_M="${INTAKE_RAMP_ENTRY_X_M:-0.500}"
fi
export INTAKE_LAUNCH_EXIT_X_M="${INTAKE_LAUNCH_EXIT_X_M:-0.465}"
export INTAKE_LAUNCH_EXIT_Z_M="${INTAKE_LAUNCH_EXIT_Z_M:-0.032}"
export INTAKE_LAUNCH_EXIT_ANGLE_DEG="${INTAKE_LAUNCH_EXIT_ANGLE_DEG:-35.0}"
export BASKET_CENTER_LIP_HEIGHT_M="${BASKET_CENTER_LIP_HEIGHT_M:-0.010}"

python3 "$SCRIPT_DIR/scripts/generate_curved_scoop_mesh.py"

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

if [ "$UBUNTU_GPU" = "true" ]; then
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
