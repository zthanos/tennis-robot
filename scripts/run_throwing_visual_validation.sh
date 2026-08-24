#!/usr/bin/env bash
# PC side of the Throwing Mode VISUAL validation in the distributed topology.
#
# This is a thin, reproducible wrapper around ./run_native.sh — it does not add
# a second launch path. All it does is fix the environment the visual check
# needs, so nobody has to reconstruct four env vars by hand and then discover
# afterwards that the intake variant was spawned:
#
#   ROBOT_PACKAGING_VARIANT=option-a-launch   launcher present, intake absent
#   TENNIS_LAUNCH_BRAIN=false                 brain runs on the Pi (run_pi.sh)
#   GAZEBO_HEADLESS=false                     the point of this run is to LOOK
#
# The Pi side is unchanged: `./run_pi.sh` there, same ROS_DOMAIN_ID, after this
# one is up (SLAM/Nav2 need /clock, /scan and odom->base_footprint flowing).
#
#   ./scripts/run_throwing_visual_validation.sh
#   ROS_DOMAIN_ID=42 ./scripts/run_throwing_visual_validation.sh
#   BUILD=true ./scripts/run_throwing_visual_validation.sh
#
# NOTE: no `set -u` — ROS setup.bash references unbound env vars.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROBOT_PACKAGING_VARIANT=option-a-launch
export TENNIS_LAUNCH_BRAIN=false
export GAZEBO_HEADLESS=false

# ── Validation-environment hygiene, NOT robot startup ────────────────────────
# Repeated SIGKILLed Gazebo/Nav2 runs leave FastDDS segments behind; the next
# stack then fails discovery with "open_and_lock_file failed" and Nav2 aborts
# every goal. This belongs to a validation harness that deliberately kills
# stacks — never to a normal robot bring-up, which must not delete shared
# memory other processes may legitimately own. Guarded accordingly: refuse to
# clean while any ROS/Gazebo process is still alive.
if pgrep -x "gz|ruby" >/dev/null 2>&1 || pgrep -f "ros2 launch" >/dev/null 2>&1; then
    echo "ERROR: ROS/Gazebo processes are still running on this machine."
    echo "Stop them before starting a clean validation run (stale FastDDS"
    echo "shared memory is only safe to remove once nothing is using it)."
    pgrep -af "gz sim|ros2 launch" | head
    exit 1
fi
shm_removed=$(find /dev/shm -maxdepth 1 -name 'fastrtps_*' -o -maxdepth 1 -name 'fastdds_*' 2>/dev/null | wc -l)
find /dev/shm -maxdepth 1 \( -name 'fastrtps_*' -o -name 'fastdds_*' \) -delete 2>/dev/null || true
echo "[visual-validation] cleared $shm_removed stale FastDDS shm segment(s)"

echo "[visual-validation] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[visual-validation] variant=$ROBOT_PACKAGING_VARIANT  GUI=on  brain=Pi"
echo "[visual-validation] start ./run_pi.sh on the Pi once Gazebo is up."

exec ./run_native.sh
