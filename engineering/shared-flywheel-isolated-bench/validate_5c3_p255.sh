#!/usr/bin/env bash
set -euo pipefail
bench=engineering/shared-flywheel-isolated-bench
root=engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c3_p255
preflight="$root/preflight"
mkdir -p "$preflight"
cmake -S "$bench" -B "$bench/build_p255"
cmake --build "$bench/build_p255" -j2
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=p251_ski_42 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$preflight/model.urdf"
check_urdf "$preflight/model.urdf" > "$preflight/check_urdf.txt"
gz sdf -p "$preflight/model.urdf" > "$preflight/model.sdf"
python3 "$bench/make_5c3_p255_world.py" --model-sdf "$preflight/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$preflight/world.sdf" --telemetry "$(pwd)/$preflight/telemetry.csv" --contacts "$(pwd)/$preflight/native_contacts.csv" --observer "$(pwd)/$preflight/observer.csv" --ball-contact "$(pwd)/$preflight/ball_00_contacts.csv" --ball-contact "$(pwd)/$preflight/ball_01_contacts.csv" --longitudinal 0 --longitudinal 0 --lateral -0.1 --lateral 0.1 > "$preflight/world_build.stdout"
python3 "$bench/preflight_5c3_p255.py" --criteria "$bench/criteria_5c3_p255.json" --ball gazebo/models/tennis_ball_compliant/model.sdf --urdf "$preflight/model.urdf" --world "$preflight/world.sdf" --output "$preflight/preflight.json"
