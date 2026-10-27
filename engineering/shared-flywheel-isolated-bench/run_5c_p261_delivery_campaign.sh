#!/usr/bin/env bash
set -euo pipefail
b=engineering/shared-flywheel-isolated-bench;r=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p261_delivery
for mu in 0.6 0.9;do for rm in 0.15 0.25 0.4;do for y in -0.1 -0.05 0.0 0.05 0.1;do P2_51_RESULT_ROOT="$r" P2_51_CRITERIA="$b/criteria_5c_p251_delivery_matrix.json" P2_61_MAX_STEP=.00025 "$b/run_5c_p251_case.sh" 42 6.0 "$mu" "$rm" "$y";done;done;done
P2_51_AGG_ROOT="$r" P2_51_EXPECTED=30 python3 "$b/aggregate_5c_p251_delivery.py"
