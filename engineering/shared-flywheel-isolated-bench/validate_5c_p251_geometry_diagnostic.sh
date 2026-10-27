#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
export P2_51_CRITERIA="$bench/criteria_5c_p251_geometry_diagnostic.json"
export P2_51_RESULT_ROOT=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_geometry_diagnostic
exec "$bench/validate_5c_p251.sh"
