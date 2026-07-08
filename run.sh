#!/usr/bin/env bash
# Fast bring-up: NO image rebuild. Code changes are picked up by the dev
# overlay build inside the container (scripts/docker_dev_entry.sh), so this is
# the everyday command. Use ./build_run.sh only when Dockerfile/deps change.
#
#   GPU + fast sim, no GUI:   GAZEBO_HEADLESS=true  ./run.sh rviz foxglove
#   GUI (software, slower):   GAZEBO_HEADLESS=false ./run.sh
set -euo pipefail

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-true}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-890M}"

# Rendering is decided INSIDE the container (scripts/docker_dev_entry.sh):
# software/llvmpipe by default (stable on WSLg), GPU only with GAZEBO_GPU=true
# (experimental — d3d12 EGL init has been seen to fail in headless mode).
export GAZEBO_GPU="${GAZEBO_GPU:-false}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-}"

export SLAM_MODE="${SLAM_MODE:-localization}"
export NAV2_START_DELAY_S="${NAV2_START_DELAY_S:-25}"

docker compose --profile gazebo down
docker compose --profile gazebo up gazebo "$@"
