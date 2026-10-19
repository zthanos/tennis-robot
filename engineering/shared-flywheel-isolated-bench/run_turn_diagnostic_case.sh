#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then echo "usage: $0 CASE MU open_yaw|gyro_yaw" >&2; exit 2; fi
case_name="$1"; mu="$2"; variant="$3"
case "$case_name" in empty|full45|ballast) ;; *) echo invalid-case >&2; exit 2;; esac
case "$mu" in 0.5|0.7|0.9) ;; *) echo invalid-mu >&2; exit 2;; esac
case "$variant" in open_yaw|gyro_yaw) ;; *) echo invalid-variant >&2; exit 2;; esac

bench=engineering/shared-flywheel-isolated-bench
out_dir="engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_5a_turn_diagnostic/$variant/$case_name/mu_${mu/./p}"
mkdir -p "$out_dir"
csv_abs="$(pwd)/$out_dir/telemetry.csv"; rm -f "$csv_abs" "$csv_abs.done"
ballast=false; full=false; gyro=false
[[ "$case_name" == ballast ]] && ballast=true
[[ "$case_name" == full45 ]] && full=true
[[ "$variant" == gyro_yaw ]] && gyro=true
args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270
 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756
 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false drive_test_ballast:="$ballast" basket_full_45:="$full" ground_mu:="$mu" \
 five_a_enabled:=true five_a_case:="$case_name" five_a_output_csv:="$csv_abs" turns_only:=true yaw_feedback_enabled:="$gyro" tilt_hold_enabled:=true "${args[@]}" > "$out_dir/model.urdf"
python3 "$bench/validate_generated_urdf.py" "$out_dir/model.urdf" --case "$case_name" --contract "$bench/v3_bench_contract.json" --output "$out_dir/validation.json" > "$out_dir/validation.stdout"
gz sdf -p "$out_dir/model.urdf" > "$out_dir/model.sdf"
python3 "$bench/make_5a_world.py" --model-sdf "$out_dir/model.sdf" --mu "$mu" --output "$out_dir/world.sdf"
export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out_dir/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_5a:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="v3_turn_${variant}_${case_name}_${mu/./p}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
server_pid=""
cleanup(){ if [[ -n "$server_pid" ]] && kill -0 "$server_pid" 2>/dev/null; then kill "$server_pid" 2>/dev/null || true; for _ in $(seq 1 30); do kill -0 "$server_pid" 2>/dev/null || break; sleep .1; done; kill -KILL "$server_pid" 2>/dev/null || true; wait "$server_pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=5 120 /usr/bin/gz sim -r -s -v 2 --physics-engine "$engine" "$out_dir/world.sdf" > "$out_dir/gazebo.log" 2>&1 & server_pid=$!
for _ in $(seq 1 1100); do [[ -f "$csv_abs.done" ]] && break; kill -0 "$server_pid" 2>/dev/null || break; sleep .1; done
if [[ ! -f "$csv_abs.done" ]]; then echo "ERROR: turn diagnostic incomplete" >&2; tail -80 "$out_dir/gazebo.log" >&2; exit 1; fi
cleanup; server_pid=""
python3 "$bench/analyze_turn_diagnostic.py" "$csv_abs" --criteria "$bench/criteria_turn_diagnostic.json" --validation "$out_dir/validation.json" --variant "$variant" --output "$out_dir/summary.json"
sha256sum "$out_dir/model.urdf" > "$out_dir/model.urdf.sha256"
echo "turn diagnostic complete: $variant $case_name mu=$mu"
