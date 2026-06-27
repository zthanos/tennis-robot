#!/usr/bin/env bash
set -euo pipefail

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-false}"
# Only the integrated 890M is enumerated inside the container via WSL D3D12 —
# the discrete 7900 XTX is NOT reachable, and naming it makes Mesa's d3d12 driver
# find no adapter and fall back to llvmpipe (software). Use 890M for HW accel.
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-890M}"

# GPU acceleration on WSL works only for OFFSCREEN/EGL rendering = HEADLESS.
# With the GUI (GLX via WSLg/Xwayland) the d3d12 driver crashes
# (drisw/GLXCreateNewContext fails), so we enable it ONLY when headless and
# leave the GUI on software (llvmpipe) where the window renders fine.
#   GPU + fast sim, no GUI:   GAZEBO_HEADLESS=true  ./build_run.sh
#   GUI (software, slower):   ./build_run.sh        (default)
if [ "$GAZEBO_HEADLESS" = "true" ]; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
else
  export GALLIUM_DRIVER=""
fi

# SLAM mode for the bring-up:
# - mapping: use this for surveying / Map Court.
# - localization: use this for collection / Nav Test with the saved survey map.
export SLAM_MODE="${SLAM_MODE:-localization}"
# export SLAM_MODE="${SLAM_MODE:-mapping}"

# Delay between starting SLAM and starting Nav2. Localization can be heavy while
# it loads the saved posegraph, so Nav2 gets a short grace period before lifecycle
# bring-up. Increase this if Nav2 starts inactive/unconfigured.
export NAV2_START_DELAY_S="${NAV2_START_DELAY_S:-25}"
# export NAV2_START_DELAY_S="${NAV2_START_DELAY_S:-10}"

docker compose --profile gazebo down
docker compose --profile gazebo build gazebo
docker compose --profile gazebo up gazebo
