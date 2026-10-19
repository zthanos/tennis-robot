#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a
validation=engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation
for case_name in empty full45 ballast; do
  for mu_token in 0p5 0p7 0p9; do
    python3 "$bench/analyze_5a_run.py" "$root/$case_name/mu_$mu_token/telemetry.csv" \
      --criteria "$bench/criteria_5a.json" --validation "$validation/$case_name.validation.json" \
      --output "$root/$case_name/mu_$mu_token/summary.json"
  done
done
python3 "$bench/aggregate_5a_results.py"
