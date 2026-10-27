#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 6 ]]; then
  echo "usage: $0 {70|75|80} {75|110} {5|6|8} {0.6|0.9} {-100|-50|0|50|100} {0.1|0.3|0.5}" >&2
  exit 2
fi
angle="$1"; radius="$2"; speed="$3"; tread_mu="$4"; lateral_mm="$5"; restitution="$6"
case "$angle" in 70|75|80) ;; *) exit 2;; esac
case "$radius" in 75|110) ;; *) exit 2;; esac
case "$speed" in 5|6|8) ;; *) exit 2;; esac
case "$tread_mu" in 0.6|0.9) ;; *) exit 2;; esac
case "$lateral_mm" in -100|-50|0|50|100) ;; *) exit 2;; esac
case "$restitution" in 0.1|0.3|0.5) ;; *) exit 2;; esac

bench=engineering/shared-flywheel-isolated-bench
root="${P2_66_RESULT_ROOT:-engineering/chassis-strength-bench/results/shared_flywheel_phase2/gazebo_p266}"
tag(){ local value="$1"; value="${value/-/m}"; echo "${value/./p}"; }
out="$root/matrix/a_${angle}/r_${radius}/v_$(tag "$speed")/tm_$(tag "$tread_mu")/y_$(tag "$lateral_mm")/e_$(tag "$restitution")"
mkdir -p "$out"
if [[ -f "$out/summary.json" && "${P2_66_FORCE:-false}" != true ]]; then
  echo "SKIP existing $out/summary.json"
  exit 0
fi
telemetry="$(pwd)/$out/telemetry.csv"; native="$(pwd)/$out/native_contacts.csv"; ball_contacts="$(pwd)/$out/ball_contacts.csv"
rm -f "$telemetry" "$telemetry.done" "$native" "$ball_contacts"
mass_args=(base_mass:=7.404 base_com_x:=0.09386412749865 base_com_z:=0.13227174500270 base_ixx:=0.38652546 base_iyy:=0.35877559 base_izz:=0.54328377 base_ixz:=-0.05214756 drive_wheel_mass:=0.65 drive_wheel_ixx:=0.00154721666667 drive_wheel_iyy:=0.002348125 drive_wheel_izz:=0.00154721666667)
xacro "$bench/v3_chassis_flywheel_bench.urdf.xacro" sim_mode:=false validation_fixtures_enabled:=true five_c_enabled:=true five_c_design:=p244 five_c_p244_ramp_enabled:=true five_c_ramp_tip_clearance:=0.008 five_c_cheek_variant:=exposed_shallow five_c_basket_variant:=p251_ski_60 tilt_angle_rad:=-0.052359877560 "${mass_args[@]}" > "$out/baseline.urdf"
gz sdf -p "$out/baseline.urdf" > "$out/baseline.sdf"
python3 "$bench/prepare_p266_model.py" "$out/baseline.sdf" "$out/model.sdf" --angle-deg "$angle" --radius-mm "$radius" --front-wall-top-mm "${P2_66_WALL_TOP_MM:-240}" > "$out/model_build.stdout"
python3 "$bench/make_p266_world.py" --model-sdf "$out/model.sdf" --ball-model gazebo/models/tennis_ball_compliant/model.sdf --output "$out/world.sdf" --telemetry "$telemetry" --contacts "$native" --ball-contacts "$ball_contacts" --lateral "$(python3 -c "print($lateral_mm/1000)")" --surface-speed "$speed" --tread-mu "$tread_mu" --ramp-mu 0.15 --deflector-restitution "$restitution" --duration "${P2_66_DURATION:-3.5}" --max-step-size 0.00025 > "$out/world_build.stdout"

export GZ_CONFIG_PATH=/usr/share/gz
physics_dir="$out/physics_plugins"; mkdir -p "$physics_dir"
ln -sfn /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 "$physics_dir/libgz-physics7-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$(pwd)/$physics_dir"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/$bench/build_p261:$(pwd)/$bench/build_contact_5c:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$(pwd)/gazebo/models:$(pwd)/ros2_ws/src:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="p266_${angle}_${radius}_${speed}_${tread_mu}_${lateral_mm}_${restitution}_${$}_${RANDOM}"
engine=/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7
gz_args=(-r -s -v 1 --physics-engine "$engine")
if [[ "${BENCH_RECORD:-false}" == true ]]; then
  mkdir -p "$out/replay"; gz_args+=(--record-path "$(pwd)/$out/replay" --log-overwrite)
fi
pid=""
cleanup(){ if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
timeout --signal=TERM --kill-after=3 180 /usr/bin/gz sim "${gz_args[@]}" "$out/world.sdf" > "$out/gazebo.log" 2>&1 & pid=$!
for _ in $(seq 1 1200); do [[ -f "$telemetry.done" ]] && break; kill -0 "$pid" 2>/dev/null || break; sleep .1; done
[[ -f "$telemetry.done" ]] || { tail -100 "$out/gazebo.log" >&2; exit 1; }
cleanup; pid=""
python3 "$bench/analyze_p266_run.py" "$telemetry" "$native" --angle-deg "$angle" --radius-mm "$radius" --surface-speed "$speed" --tread-mu "$tread_mu" --lateral-mm "$lateral_mm" --restitution "$restitution" --output "$out/summary.json" > "$out/analysis.stdout"
sha256sum "$out/baseline.urdf" "$out/model.sdf" > "$out/model.sha256"
python3 - "$out/summary.json" <<'PY'
import json,sys
p=json.load(open(sys.argv[1])); m=p['metrics']
print(('PASS' if p['pass'] else 'FAIL')+f" angle={m['takeoff_angle_deg']} speed={m['takeoff_speed_m_s']} impact={m['deflector_impact_xyz_m']}")
PY
