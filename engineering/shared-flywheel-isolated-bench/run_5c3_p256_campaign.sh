#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
cases=(p256_reverse_both_y50 p256_reverse_both_y100 p256_asym_left_y50 p256_asym_left_y100 p256_reverse150_yaw12_y50 p256_reverse150_yaw12_y100)
for tread_mu in 0.6 0.9; do
  for case_name in "${cases[@]}"; do
    echo "P2-56 headless recorded case=$case_name tread_mu=$tread_mu"
    ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec \
      "timeout 240 $bench/run_5c3_p256_case.sh $case_name $tread_mu"
    if [[ -n "$(docker ps -q)" ]]; then
      echo "container remained active" >&2; docker ps >&2; exit 1
    fi
  done
done
python3 "$bench/aggregate_5c3_p256.py"

