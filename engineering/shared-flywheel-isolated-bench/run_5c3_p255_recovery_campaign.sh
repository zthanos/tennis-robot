#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
for profile in stop_1s reverse_75mm; do
  for tread_mu in 0.6 0.9; do
    echo "5c-3 jam recovery diagnostic: profile=$profile tread_mu=$tread_mu"
    ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 env BENCH_GUI=false BENCH_RECORD=true P2_55_DRIVE_PROFILE=$profile $bench/run_5c3_p255_case.sh side_by_side_y100 $tread_mu"
    if [[ -n "$(docker ps -q)" ]]; then echo "container remained active" >&2; docker ps >&2; exit 1; fi
  done
done
python3 "$bench/aggregate_5c3_p255.py"
