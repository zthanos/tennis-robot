#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then echo "usage: $0 empty|full45|ballast" >&2; exit 2; fi
case_name="$1"; case "$case_name" in empty|full45|ballast) ;; *) echo invalid-case >&2; exit 2;; esac
bench=engineering/shared-flywheel-isolated-bench
out_dir="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5b/$case_name"
mkdir -p "$out_dir"
csv_abs="$(pwd)/$out_dir/telemetry.csv"; contacts_abs="$(pwd)/$out_dir/contacts.csv"
rm -f "$csv_abs" "$csv_abs.done" "$contacts_abs"
ballast=false; full=false; dynamic=true
[[ "$case_name" == ballast ]] && ballast=true && dynamic=false
[[ "$case_name" == full45 ]] && full=true
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270
 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756
 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false drive_test_ballast:="$ballast" basket_full_45:="$full" \
 five_b_enabled:=true five_b_case:="$case_name" five_b_dynamic_tilt:="$dynamic" five_b_output_csv:="$csv_abs" five_b_contacts_csv:="$contacts_abs" \
 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out_dir/model.urdf"
python3 "$bench/validate_generated_urdf.py" "$out_dir/model.urdf" --case "$case_name" --contract "$bench/v3_bench_contract.json" --output "$out_dir/urdf_validation.json" > "$out_dir/urdf_validation.stdout"
python3 "$bench/preflight_5b.py" --urdf "$out_dir/model.urdf" --case "$case_name" --mapping "$bench/p2_40_collision_fixture_mapping.json" --criteria "$bench/criteria_5b.json" --output "$out_dir/preflight.json" > "$out_dir/preflight.stdout"
gz sdf -p "$out_dir/model.urdf" > "$out_dir/model.sdf"
python3 "$bench/make_5b_world.py" --model-sdf "$out_dir/model.sdf" --output "$out_dir/world.sdf" > "$out_dir/world_build.stdout"
sha256sum "$out_dir/model.urdf" > "$out_dir/model.urdf.sha256"
export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out_dir/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_5b:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="v3_5b_${case_name}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
server_pid=""
cleanup(){ if [[ -n "$server_pid" ]] && kill -0 "$server_pid" 2>/dev/null; then kill "$server_pid" 2>/dev/null || true; for _ in $(seq 1 30); do kill -0 "$server_pid" 2>/dev/null || break; sleep .1; done; kill -KILL "$server_pid" 2>/dev/null || true; wait "$server_pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=5 150 /usr/bin/gz sim -r -s -v 2 --physics-engine "$engine" "$out_dir/world.sdf" > "$out_dir/gazebo.log" 2>&1 & server_pid=$!
for _ in $(seq 1 1400); do [[ -f "$csv_abs.done" ]] && break; kill -0 "$server_pid" 2>/dev/null || break; sleep .1; done
if [[ ! -f "$csv_abs.done" ]]; then echo "ERROR: 5b run incomplete" >&2; tail -100 "$out_dir/gazebo.log" >&2; exit 1; fi
cleanup; server_pid=""
if grep -Eq 'Failed to find plugin|Could not find shared library|No joint named|Asked to create a joint|Unable to create joint' "$out_dir/gazebo.log"; then grep -E 'Failed to find plugin|Could not find shared library|No joint named|Asked to create a joint|Unable to create joint' "$out_dir/gazebo.log" >&2; exit 1; fi
python3 "$bench/analyze_5b_run.py" "$csv_abs" "$contacts_abs" --criteria "$bench/criteria_5b.json" --case "$case_name" --output "$out_dir/summary.json"
echo "5b complete: $case_name"
