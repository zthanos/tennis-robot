#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5d1_p260
for compression in 8 11 14; do
  for mu in 0.6 0.9 1.2; do
    for feed in 0 2 4; do
      tag="${mu/./p}"; summary="$root/cases/compression_${compression}/tm_${tag}/feed_${feed}/summary.json"
      [[ -f "$summary" ]] && continue
      BENCH_RECORD=false "$bench/run_5d1_p260_case.sh" "$compression" "$mu" "$feed"
    done
  done
done
python3 "$bench/aggregate_5d1_p260.py" --write-record-manifest
while read -r compression mu feed; do
  [[ -n "$compression" ]] || continue
  BENCH_RECORD=true "$bench/run_5d1_p260_case.sh" "$compression" "$mu" "$feed"
done < "$root/record_manifest.txt"
python3 "$bench/aggregate_5d1_p260.py"
