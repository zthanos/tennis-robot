#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 {15|20} {0.6|0.9} {12|16|20|24} [REPEAT]" >&2
  exit 2
fi

angle="$1"
mu="$2"
surface="$3"
repeat="${4:-1}"
case "$angle" in 15|20) ;; *) echo "invalid angle" >&2; exit 2;; esac
case "$mu" in 0.6|0.9) ;; *) echo "invalid mu" >&2; exit 2;; esac
case "$surface" in 12|16|20|24) ;; *) echo "invalid surface speed" >&2; exit 2;; esac
case "$repeat" in 1|2|3) ;; *) echo "invalid repeat" >&2; exit 2;; esac

bench=engineering/shared-flywheel-isolated-bench
tag="${mu/./p}"
replay="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/cases/angle_${angle}/tm_${tag}/surface_${surface}/repeat_${repeat}/replay"
[[ -f "$replay/state.tlog" ]] || {
  echo "recording not found: $replay/state.tlog" >&2
  exit 1
}

# Playback intentionally starts paused: do not add -r.  Reuse the known-good
# side-view GUI configuration; this script never rewrites it.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}" GAZEBO_HEADLESS=false \
  docker/jazzy/run.sh exec \
  "gz sim -v 2 --gui-config $bench/5c3_side_gui.config --playback $replay"
