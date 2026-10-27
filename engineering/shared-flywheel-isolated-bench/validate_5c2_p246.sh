#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c2_p246/preflight
mkdir -p "$root"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
common=(sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_cheek_variant:=exposed_shallow tilt_angle_rad:=-0.052359877560)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" > "$root/delivery.urdf"
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" "${common[@]}" "${mass_args[@]}" five_a_enabled:=true five_a_case:=p246_full45_retention five_a_output_csv:=/tmp/p246_drive.csv retention_only:=true yaw_feedback_enabled:=true tilt_hold_enabled:=true > "$root/motion.urdf"
check_urdf "$root/delivery.urdf" > "$root/delivery.check_urdf.txt"
check_urdf "$root/motion.urdf" > "$root/motion.check_urdf.txt"
cmake -S "$bench" -B "$bench/build_p246" -DCMAKE_BUILD_TYPE=Release > "$root/cmake_configure.log"
cmake --build "$bench/build_p246" -j2 > "$root/cmake_build.log"
python3 "$bench/preflight_5c2_p246.py" --delivery "$root/delivery.urdf" --motion "$root/motion.urdf" --criteria "$bench/criteria_5c2_p246.json" --ball gazebo/models/tennis_ball_compliant/model.sdf --output "$root/preflight.json"
