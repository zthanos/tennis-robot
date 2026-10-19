#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
count=0
for speed in 6.0 10.0 14.0; do
  for tread_mu in 0.6 0.9; do
    for ramp_mu in 0.15 0.25 0.4; do
      for lateral in -0.1 -0.05 0.0 0.05 0.1; do
        count=$((count+1))
        echo "P2-51 delivery matrix $count/90: v=$speed tread_mu=$tread_mu ramp_mu=$ramp_mu y=$lateral"
        ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 $bench/run_5c_p251_delivery_matrix_case.sh 42 $speed $tread_mu $ramp_mu $lateral"
        if [[ -n "$(docker ps -q)" ]]; then
          echo "container remained active after case $count" >&2
          docker ps >&2
          exit 1
        fi
      done
    done
  done
done
