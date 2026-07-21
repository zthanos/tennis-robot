#!/usr/bin/env bash
# Gazebo container entrypoint with a dev overlay build.
#
# Instead of baking ros2_ws into the image (rebuild = minutes), colcon-build the
# mounted /workspace/ros2_ws on every container start into build_docker/
# install_docker (persisted on the host via the volume mount, so the build is
# incremental: unchanged packages are skipped, a Python-only edit rebuilds in
# seconds). The overlay is sourced on top of the image install, so overlay
# packages win. Disable with DEV_OVERLAY=false to run exactly what the image
# was built with.
#
# NOTE: no --symlink-install on purpose — /workspace is a drvfs (/mnt/c) mount
# under WSL where symlinks are unreliable. Freshness comes from rebuilding on
# every start instead.
set -e

. /opt/ros/humble/setup.sh
. /ros2_ws/install/setup.sh

# ── Rendering strategy ────────────────────────────────────────────────────────
# WSLg d3d12 GPU accel is unreliable for the gz server: GLX crashes with the
# GUI and eglInitialize fails in headless PBuffer mode (verified 2026-07-04 —
# "OpenGL 3.3 is not supported" x11 attempts, sensors dead, no odom). Default
# to SOFTWARE rendering (llvmpipe: slower but always works). Opt in to the GPU
# experiment with GAZEBO_GPU=true.
if [ "${GAZEBO_GPU:-false}" != "true" ]; then
  export GALLIUM_DRIVER=llvmpipe
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export LIBGL_ALWAYS_SOFTWARE=1
  echo "[dev_entry] rendering: software (llvmpipe). Set GAZEBO_GPU=true to try d3d12."
else
  echo "[dev_entry] rendering: GPU (d3d12) — experimental on WSLg."
fi

if [ "${DEV_OVERLAY:-true}" = "true" ]; then
  echo "[dev_entry] building workspace overlay (tennis_robot_msgs, tennis_robot_collection_controller, tennis_robot)…"
  cd /workspace/ros2_ws
  colcon build \
    --build-base build_docker \
    --install-base install_docker \
    --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot
  . install_docker/local_setup.sh
  echo "[dev_entry] overlay active: /workspace/ros2_ws/install_docker"
else
  echo "[dev_entry] DEV_OVERLAY=false — running image-baked install only"
fi

ros2 launch tennis_robot sim.launch.py "headless:=${GAZEBO_HEADLESS:-true}" &
sleep 12
ros2 launch "tennis_robot" "slam_${SLAM_MODE:-mapping}.launch.py" &
sleep "${NAV2_START_DELAY_S:-25}"
exec ros2 launch tennis_robot navigation.launch.py
