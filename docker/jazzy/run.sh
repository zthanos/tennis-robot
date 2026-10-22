#!/usr/bin/env bash
# Build/run the Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic container.
#
#   docker/jazzy/run.sh build            # build the image (once, ~3-5 GB)
#   docker/jazzy/run.sh gpu-check        # glxinfo renderer + gz version
#   docker/jazzy/run.sh shell            # interactive shell, ROS sourced
#   docker/jazzy/run.sh sim [env...]     # ./run_native.sh inside the container
#       e.g. GAZEBO_HEADLESS=true docker/jazzy/run.sh sim
#   docker/jazzy/run.sh exec <cmd...>    # run one command (ROS sourced)
#
# The repo is mounted at the SAME absolute path, so colcon install paths and
# run_native.sh work unchanged. Runs as the host user (no root-owned files).
# GPU: the host /dev/dri (iGPU) is passed through; Gazebo uses Mesa.
set -eo pipefail

IMAGE="${TENNIS_JAZZY_IMAGE:-tennis-robot-jazzy:latest}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"

cmd="${1:-shell}"; shift || true

if [ "$cmd" = "build" ]; then
    exec docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
fi

# --- display (XWayland or X11) -------------------------------------------
display_args=()
if [ -n "${DISPLAY:-}" ]; then
    display_args+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
    if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
        display_args+=(-e "XAUTHORITY=/tmp/.xauth" -v "$XAUTHORITY:/tmp/.xauth:ro")
    fi
fi

# --- GPU -----------------------------------------------------------------
gpu_args=()
if [ -d /dev/dri ]; then
    gpu_args+=(--device /dev/dri)
    for dev in /dev/dri/card* /dev/dri/renderD*; do
        [ -e "$dev" ] && gpu_args+=(--group-add "$(stat -c %g "$dev")")
    done
fi

# --- pass selected env through to run_native.sh -----------------------------
env_args=()
for v in GAZEBO_HEADLESS BUILD SLAM_MODE ROBOT_PACKAGING_VARIANT ROS_DOMAIN_ID \
         TENNIS_LAUNCH_BRAIN TENNIS_PERCEPTION_ON_PC SIM_START_DELAY_S NAV2_START_DELAY_S \
         BALL_MODEL_PATH COURT_SCENE_MODEL_PATH; do
    [ -n "${!v:-}" ] && env_args+=(-e "$v=${!v}")
done

tty_args=(-i)
[ -t 0 ] && tty_args+=(-t)

CONTAINER="tennis-jazzy-$$"
# Always stop the container when this script ends (Ctrl-C, timeout, kill):
# signals to the docker client alone do not reliably stop gz/ROS inside.
cleanup() { docker stop -t 5 "$CONTAINER" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

run() {
    # Non-interactive: run the client in the background and `wait`, so a
    # TERM/INT to this script fires the trap immediately (bash defers traps
    # while a foreground child runs). Interactive shells stay in the foreground.
    if [ ! -t 0 ]; then
        _docker_run "$@" &
        wait $!
    else
        _docker_run "$@"
    fi
}

_docker_run() {
    docker run --rm --init "${tty_args[@]}" \
        --name "$CONTAINER" \
        --network host --ipc host \
        --user "$(id -u):$(id -g)" \
        -v "$REPO:$REPO" -w "$REPO" \
        "${display_args[@]}" "${gpu_args[@]}" "${env_args[@]}" \
        "$IMAGE" bash -lc "source /opt/ros/jazzy/setup.bash; $*"
}

case "$cmd" in
    shell)     run "exec bash" ;;
    gpu-check) run "glxinfo -B 2>&1 | grep -E 'renderer|OpenGL version|Accelerated' ; gz sim --versions" ;;
    sim)       run "./run_native.sh" ;;
    exec)      run "$*" ;;
    *) echo "usage: $0 {build|gpu-check|shell|sim|exec <cmd>}" >&2; exit 2 ;;
esac
