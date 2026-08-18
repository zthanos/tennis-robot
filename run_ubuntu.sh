#!/usr/bin/env bash
# OBSOLETE — Docker/ROS 2 Humble bring-up.  The supported runtime is native
# Ubuntu 24.04 + ROS 2 Jazzy: use ./run_native.sh.
#
# This path is kept for historical reference only and is known broken for
# anything that navigates: config/nav2_params.yaml declares the Jazzy plugin
# name "nav2_smac_planner::SmacPlanner2D", which Humble's pluginlib does not
# export, so planner_server fails to configure, Nav2 bringup aborts, and every
# collect_route hangs in navigating_to_scan_pose while the robot never receives
# a velocity command (debug log #65).  That failure looks like a slow simulator
# and cost a full validation session, which is why this now refuses to start.
set -euo pipefail

if [ "${ALLOW_OBSOLETE_HUMBLE_DOCKER:-false}" != "true" ]; then
    cat >&2 <<'OBSOLETE'
run_ubuntu.sh is OBSOLETE (Docker + ROS 2 Humble).

  Supported runtime:  native Ubuntu 24.04 + ROS 2 Jazzy
  Use instead:        ./run_native.sh

Nav2 cannot start here: the Nav2 parameters use Jazzy plugin names, so
planner_server fails to configure and nothing ever drives the robot.  Do not
"fix" that by adding Humble compatibility -- the container is legacy.

To run it anyway (CAD/OpenSCAD profiles, archaeology):
  ALLOW_OBSOLETE_HUMBLE_DOCKER=true ./run_ubuntu.sh
OBSOLETE
    exit 2
fi

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
COURT_SCENE_PATH="${COURT_SCENE_MODEL_PATH:-$SCRIPT_DIR/models/court_scene_yolov8n.onnx}"
if [ ! -s "$COURT_SCENE_PATH" ]; then
    echo "ERROR: Neural court-scene model is missing or empty: $COURT_SCENE_PATH"
    echo "Train/export it with scripts/train_court_scene_yolo.py, or set COURT_SCENE_MODEL_PATH."
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
