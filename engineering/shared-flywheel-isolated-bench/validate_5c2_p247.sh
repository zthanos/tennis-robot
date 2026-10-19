#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p247_r3/preflight
mkdir -p "$root"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
common=(sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow tilt_angle_rad:=-0.052359877560)
for variant in high_hood one_way_flap lip_cover; do
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" five_c_basket_variant:="$variant" > "$root/${variant}_delivery.urdf"
  xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" five_c_basket_variant:="$variant" five_a_enabled:=true five_a_case:=p247_full45_retention five_a_output_csv:=/tmp/p247_drive.csv retention_only:=true yaw_feedback_enabled:=true tilt_hold_enabled:=true > "$root/${variant}_motion.urdf"
  check_urdf "$root/${variant}_delivery.urdf" > "$root/${variant}_delivery.check_urdf.txt"
  check_urdf "$root/${variant}_motion.urdf" > "$root/${variant}_motion.check_urdf.txt"
done
cmake -S "$bench" -B "$bench/build_p247" -DCMAKE_BUILD_TYPE=Release > "$root/cmake_configure.log"
cmake --build "$bench/build_p247" -j2 > "$root/cmake_build.log"
python3 "$bench/preflight_5c2_p247.py" --root "$root" --criteria "$bench/criteria_5c2_p247_r3.json" --ball gazebo/models/tennis_ball_compliant/model.sdf --output "$root/preflight.json"
