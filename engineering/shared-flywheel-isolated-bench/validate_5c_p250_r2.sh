#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
variant="${P2_50_VARIANT:-p249_r125}"
criteria="${P2_50_CRITERIA:-$bench/criteria_5c_p250_r2.json}"
result_root="${P2_50_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p250_r2}"
root="$result_root/preflight"
mkdir -p "$root"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:="$variant" tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$root/model.urdf"
check_urdf "$root/model.urdf" > "$root/check_urdf.txt"
python3 "$bench/preflight_5c_p250.py" --urdf "$root/model.urdf" --criteria "$criteria" --ball gazebo/models/tennis_ball_compliant/model.sdf --image-digest "${P2_50_IMAGE_DIGEST:?set P2_50_IMAGE_DIGEST}" --output "$root/preflight.json"
