#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then echo "usage: $0 {8|11|14} {0.6|0.9|1.2} {0|2|4}" >&2; exit 2; fi
compression="$1"; mu="$2"; feed="$3"
case "$compression" in 8|11|14) ;; *) exit 2;; esac
case "$mu" in 0.6|0.9|1.2) ;; *) exit 2;; esac
case "$feed" in 0|2|4) ;; *) exit 2;; esac
tag="${mu/./p}"
replay="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p260/cases/compression_${compression}/tm_${tag}/feed_${feed}/replay"
[[ -f "$replay/state.tlog" ]] || { echo "recording not found: $replay/state.tlog" >&2; exit 1; }
bench=engineering/shared-flywheel-isolated-bench
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}" GAZEBO_HEADLESS=false docker/jazzy/run.sh exec \
  "gz sim -v 2 --gui-config $bench/5c3_side_gui.config --playback $replay"
