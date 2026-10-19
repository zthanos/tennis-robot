#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
cases=(series_gap_0 series_gap_50 series_gap_150 series_gap_300 side_by_side_y50 side_by_side_y100)
for tread_mu in 0.6 0.9; do
  for case_name in "${cases[@]}"; do
    record=false
    case "$case_name" in series_gap_0|series_gap_50) record=true;; esac
    echo "corrected 5c-3 headless acceptance: $case_name tread_mu=$tread_mu record=$record"
    ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 env BENCH_GUI=false BENCH_RECORD=$record $bench/run_5c3_p255_case.sh $case_name $tread_mu"
    tag="${tread_mu/./p}"
    summary="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p255/matrix/tm_${tag}/${case_name}/summary.json"
    if ! python3 - "$summary" <<'PY'
import json,sys
raise SystemExit(0 if json.load(open(sys.argv[1]))['pass'] else 1)
PY
    then
      replay="${summary%/summary.json}/replay/state.tlog"
      if [[ ! -f "$replay" ]]; then
        echo "rerunning failed case with mandatory recording: $case_name tread_mu=$tread_mu"
        ROS_DOMAIN_ID=88 GAZEBO_HEADLESS=true docker/jazzy/run.sh exec "timeout 240 env BENCH_GUI=false BENCH_RECORD=true $bench/run_5c3_p255_case.sh $case_name $tread_mu"
      fi
    fi
    if [[ -n "$(docker ps -q)" ]]; then echo "container remained active" >&2; docker ps >&2; exit 1; fi
  done
done
python3 "$bench/aggregate_5c3_p255.py"
