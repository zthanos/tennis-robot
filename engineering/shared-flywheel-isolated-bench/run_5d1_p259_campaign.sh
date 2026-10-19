#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
for angle in 15 20; do
  for mu in 0.6 0.9; do
    for surface in 12 16 20 24; do
      for repeat in 1 2 3; do
        tag="${mu/./p}"
        summary="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/cases/angle_${angle}/tm_${tag}/surface_${surface}/repeat_${repeat}/summary.json"
        [[ -f "$summary" ]] && continue
        BENCH_RECORD=false "$bench/run_5d1_p259_case.sh" "$angle" "$mu" "$surface" "$repeat"
      done
    done
  done
done
python3 "$bench/aggregate_5d1_p259.py" --write-record-manifest
while read -r angle mu surface repeat; do
  [[ -n "$angle" ]] || continue
  BENCH_RECORD=true "$bench/run_5d1_p259_case.sh" "$angle" "$mu" "$surface" "$repeat"
done < engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p259/record_manifest.txt
python3 "$bench/aggregate_5d1_p259.py"
