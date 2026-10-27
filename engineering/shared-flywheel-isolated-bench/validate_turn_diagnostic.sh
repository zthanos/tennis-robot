#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/xacro_validation_turn_diagnostic
mkdir -p "$root"
args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270
 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756
 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
for case_name in empty full45 ballast; do
  ballast=false; full=false
  [[ "$case_name" == ballast ]] && ballast=true
  [[ "$case_name" == full45 ]] && full=true
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false drive_test_ballast:="$ballast" basket_full_45:="$full" "${args[@]}" > "$root/$case_name.urdf"
  check_urdf "$root/$case_name.urdf" > "$root/$case_name.check_urdf.txt"
  python3 "$bench/validate_generated_urdf.py" "$root/$case_name.urdf" --case "$case_name" --contract "$bench/v3_bench_contract.json" --output "$root/$case_name.validation.json"
  sha256sum "$root/$case_name.urdf" > "$root/$case_name.urdf.sha256"
done
