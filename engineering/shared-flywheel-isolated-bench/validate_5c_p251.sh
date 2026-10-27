#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
criteria="${P2_51_CRITERIA:-$bench/criteria_5c_p251_calibration.json}"
result_root="${P2_51_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_calibration}"
root="$result_root/preflight"
mkdir -p "$root"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
angles="${P2_51_ANGLES:-35 45 55}"
urdfs=()
for a in $angles; do
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:="p251_ski_${a}" tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$root/model_${a}.urdf"
  check_urdf "$root/model_${a}.urdf" > "$root/check_${a}.txt"
  urdfs+=("$root/model_${a}.urdf")
done
csv_angles="${angles// /,}"
python3 "$bench/preflight_5c_p251.py" --criteria "$criteria" --ball gazebo/models/tennis_ball_compliant/model.sdf --image-digest "${P2_51_IMAGE_DIGEST:?set P2_51_IMAGE_DIGEST}" --output "$root/preflight.json" --angles "$csv_angles" "${urdfs[@]}"
