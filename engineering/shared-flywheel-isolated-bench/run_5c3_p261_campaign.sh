#!/usr/bin/env bash
set -euo pipefail
b=engineering/shared-flywheel-isolated-bench
export P2_61_COLLECTION_SPEED=5.0 P2_61_MAX_STEP=.00025
export P2_55_RESULT_ROOT=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p261_p255
export P2_58_RESULT_ROOT=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p261_p258
for mu in 0.6 0.9; do
  for c in series_gap_0 series_gap_50 series_gap_150 series_gap_300 side_by_side_y50; do
    "$b/run_5c3_p255_case.sh" "$c" "$mu"
  done
done
for mu in 0.6 0.9; do
  for orient in lead_plus lead_minus; do
    for pair in '0 70' '0 100' '35 70' '35 100'; do
      read -r dx dy <<< "$pair"
      "$b/run_5c3_p258_case.sh" "$dx" "$dy" "$orient" "$mu"
    done
  done
done
python3 "$b/aggregate_5c3_p261.py"
