#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p258
dxs=(0 35 70 100 150); dys=(0 20 35 50 70 100)
[[ -f "$bench/criteria_5c3_p258.json" ]] || { echo "P2-58 criteria missing" >&2; exit 1; }
for mu in 0.6 0.9; do
  for dx in "${dxs[@]}"; do
    for dy in "${dys[@]}"; do
      python3 - "$dx" "$dy" <<'PY' || continue
import math,sys
raise SystemExit(0 if math.hypot(float(sys.argv[1]),float(sys.argv[2])) >= 66 else 1)
PY
      if [[ "$dy" == 0 ]]; then
        orientations=(center)
      elif [[ "$mu" == 0.6 ]]; then
        orientations=(lead_plus lead_minus)
      else
        orientations=(lead_plus)
      fi
      for orientation in "${orientations[@]}"; do
        echo "P2-58 screen dx=$dx dy=$dy orientation=$orientation mu=$mu"
        BENCH_RECORD=false "$bench/run_5c3_p258_case.sh" "$dx" "$dy" "$orientation" "$mu"
      done
    done
  done
done
python3 "$bench/aggregate_5c3_p258.py" --write-record-manifest
while read -r dx dy orientation mu; do
  echo "P2-58 recorded boundary/failure dx=$dx dy=$dy orientation=$orientation mu=$mu"
  BENCH_RECORD=true "$bench/run_5c3_p258_case.sh" "$dx" "$dy" "$orientation" "$mu"
done < "$root/record_manifest.txt"
python3 "$bench/aggregate_5c3_p258.py"
