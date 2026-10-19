#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
export P2_47_RESULTS_TAG=gazebo_5c2_p247_r6
export P2_47_CRITERIA="$bench/criteria_5c2_p247_r6.json"
export P2_47_SURFACE_SPEED=2.7
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/$P2_47_RESULTS_TAG
for preload in 0 25 40 44; do
  for repeat in 1 2 3; do
    summary="$root/tall_open_drop/delivery/preload_${preload}/r${repeat}/summary.json"
    [[ -f "$summary" ]] || "$bench/run_5c2_p247_case.sh" tall_open_drop delivery "$preload" "$repeat"
  done
done
for repeat in 1 2 3; do
  summary="$root/tall_open_drop/motion/preload_45/r${repeat}/summary.json"
  [[ -f "$summary" ]] || "$bench/run_5c2_p247_case.sh" tall_open_drop motion 45 "$repeat"
done
python3 "$bench/aggregate_5c2_p247_r6_results.py"
