#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
export P2_51_CRITERIA="$bench/criteria_5c_p251_delivery_matrix.json"
export P2_51_RESULT_ROOT=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_delivery_matrix
export P2_51_ANGLES="42"
exec "$bench/validate_5c_p251.sh"
