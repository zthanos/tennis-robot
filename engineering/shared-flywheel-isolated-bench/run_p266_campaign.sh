#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
jobs="${P2_66_JOBS:-6}"
for angle in 70 75 80; do
  for radius in 75 110; do
    for speed in 5 6 8; do
      for tread_mu in 0.6 0.9; do
        for lateral in -100 -50 0 50 100; do
          for restitution in 0.1 0.3 0.5; do
            printf '%s %s %s %s %s %s\n' "$angle" "$radius" "$speed" "$tread_mu" "$lateral" "$restitution"
          done
        done
      done
    done
  done
done | xargs -P "$jobs" -n 6 "$bench/run_p266_case.sh"
python3 "$bench/aggregate_p266.py"
