#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
variants=(p257_shield_m90 p257_shield_m100 p257_shield_m110 p257_stagger_open p257_stagger_shield)
cases=(side_y35 side_y50 side_y100 series_gap_0 series_gap_50 series_gap_150 series_gap_300 single_y0 single_ym20 single_yp20 single_ym40 single_yp40 single_ym60 single_yp60 single_ym80 single_yp80 single_ym100 single_yp100)
[[ -f "$bench/criteria_5c3_p257.json" ]] || { echo "P2-57 criteria must exist before the campaign" >&2; exit 1; }
for variant in "${variants[@]}"; do
  for tread_mu in 0.6 0.9; do
    for case_name in "${cases[@]}"; do
      echo "P2-57 headless recorded variant=$variant case=$case_name tread_mu=$tread_mu"
      BENCH_GUI=false BENCH_RECORD=true "$bench/run_5c3_p257_case.sh" "$variant" "$case_name" "$tread_mu"
    done
  done
done
python3 "$bench/aggregate_5c3_p257.py"
