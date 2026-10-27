#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
jobs="${P2_69_JOBS:-6}"
for speed in 5 6 8; do
  for tread_mu in 0.6 0.9; do
    for lateral in -100 -50 0 50 100; do
      printf '%s %s %s %s\n' "$speed" "$tread_mu" "$lateral" 0.3
    done
  done
done | xargs -P "$jobs" -n 4 "$bench/run_p269_case.sh"
python3 "$bench/aggregate_p269.py"
