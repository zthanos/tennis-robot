#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 3 ]]||{ echo "usage: $0 {0.6|0.9} {12|16|20|24} {0|2}";exit 2;}
mu="$1";surface="$2";feed="$3";tag="${mu/./p}";b=engineering/shared-flywheel-isolated-bench
replay="$(pwd)/engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p261/cases/tm_$tag/surface_$surface/feed_$feed/replay"
[[ -f "$replay/state.tlog" ]]||{ echo "missing replay: $replay/state.tlog";exit 1;}
ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=false docker/jazzy/run.sh exec "gz sim -v 2 --gui-config $b/5c3_side_gui.config --playback $replay"
