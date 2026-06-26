#!/usr/bin/env bash
set -euo pipefail

export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-true}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-7900 XTX}"

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
