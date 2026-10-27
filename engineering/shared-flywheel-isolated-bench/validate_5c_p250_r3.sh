#!/usr/bin/env bash
set -euo pipefail
export P2_50_VARIANT=p249_r125_rails
export P2_50_CRITERIA=engineering/shared-flywheel-isolated-bench/criteria_5c_p250_r3.json
export P2_50_RESULT_ROOT=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p250_r3
exec engineering/shared-flywheel-isolated-bench/validate_5c_p250_r2.sh
