#!/usr/bin/env bash
set -euo pipefail

bench=engineering/shared-flywheel-isolated-bench
results=engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation
mkdir -p "$results"

xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" drive_test_ballast:=false basket_full_45:=false > "$results/empty.urdf"
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" drive_test_ballast:=false basket_full_45:=true > "$results/full45.urdf"
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" drive_test_ballast:=true basket_full_45:=false > "$results/ballast.urdf"

for case_name in empty full45 ballast; do
    check_urdf "$results/$case_name.urdf" > "$results/$case_name.check_urdf.txt"
    python3 "$bench/validate_generated_urdf.py" "$results/$case_name.urdf" \
        --case "$case_name" --contract "$bench/v3_bench_contract.json" \
        --output "$results/$case_name.validation.json"
    sha256sum "$results/$case_name.urdf" > "$results/$case_name.urdf.sha256"
done
