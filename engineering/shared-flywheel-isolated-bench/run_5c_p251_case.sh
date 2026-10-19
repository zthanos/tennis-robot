#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 5 ]]; then echo "usage: $0 ANGLE SURFACE_SPEED TREAD_MU RAMP_MU LATERAL" >&2; exit 2; fi
angle="$1"; speed="$2"; tread_mu="$3"; ramp_mu="$4"; lateral="$5"
case "$angle" in 35|42|45|50|55|60) ;; *) exit 2;; esac
case "$tread_mu" in 0.6|0.9) ;; *) exit 2;; esac
case "$ramp_mu" in 0.15|0.25|0.4) ;; *) exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench
criteria="${P2_51_CRITERIA:-$bench/criteria_5c_p251_calibration.json}"
root="${P2_51_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5c_p251_calibration}"
tag(){ local x="$1"; x="${x/-/m}"; echo "${x/./p}"; }
out="$root/a_${angle}/v_$(tag "$speed")/tm_$(tag "$tread_mu")/rm_$(tag "$ramp_mu")/y_$(tag "$lateral")"
mkdir -p "$out"
telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; ball_contacts="$(pwd)/$out/ball_contacts.csv"
rm -f "$telemetry" "$telemetry.done" "$native" "$ball_contacts"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:="p251_ski_${angle}" tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/model.urdf"
gz sdf -p "$out/model.urdf" > "$out/model.sdf"
python3 "$bench/make_5c_p244_world.py" --model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --ball-contacts "$ball_contacts" --approach 0.6 --lateral "$lateral" --surface-speed "$speed" --tread-mu "$tread_mu" --cheek-mu 0.15 --ramp-mu "$ramp_mu" --duration 7.0 --max-step-size "${P2_61_MAX_STEP:-0.001}" > "$out/world_build.stdout"
export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_5b:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="p251_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
pid=""
cleanup(){ if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 120 /usr/bin/gz sim -r -s -v 1 --physics-engine "$engine" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 900); do [[ -f "$telemetry.done" ]] && break; kill -0 "$pid" 2>/dev/null || break; sleep .1; done
[[ -f "$telemetry.done" ]] || { tail -80 "$out/gazebo.log" >&2; exit 1; }
cleanup; pid=""
python3 "$bench/analyze_5c_p251_run.py" "$telemetry" "$ball_contacts" "$native" --criteria "$criteria" --angle "$angle" --surface-speed "$speed" --tread-mu "$tread_mu" --ramp-mu "$ramp_mu" --lateral "$lateral" --output "$out/summary.json" > "$out/analysis.stdout"
sha256sum "$out/model.urdf" > "$out/model.urdf.sha256"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); print(('PASS' if p['pass'] else 'FAIL')+f" exit={p['metrics']['takeoff_speed_m_s']} delivery={p['delivery_success']}")
PY
