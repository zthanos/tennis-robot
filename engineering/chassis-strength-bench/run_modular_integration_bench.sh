#!/usr/bin/env bash
set -euo pipefail

bench_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
payload="${CHASSIS_PAYLOAD_MASS_KG:-0}"
engine="${CHASSIS_PHYSICS_ENGINE:-dartsim}"
case "$engine" in
  dartsim|bullet|bullet-featherstone) ;;
  *) echo "ERROR: unsupported physics engine: $engine" >&2; exit 1 ;;
esac
engine_path="/usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-$engine-plugin.so.7"
case_name="payload_${payload//./p}kg"
out_dir="${CHASSIS_INTEGRATION_OUT_DIR:-$bench_dir/results/rigid_chassis_motion/$case_name}"
world="$out_dir/modular_chassis_integration_bench.sdf"
mkdir -p "$out_dir"

python3 "$bench_dir/generate_modular_integration_sdf.py" \
  --payload-mass-kg "$payload" --output "$world"

export GZ_CONFIG_PATH=/usr/share/gz
physics_plugin_dir="$out_dir/physics_plugins"
mkdir -p "$physics_plugin_dir"
ln -sfn \
  /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 \
  "$physics_plugin_dir/libgz-physics7-dartsim-plugin.so"
ln -sfn \
  /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 \
  "$physics_plugin_dir/libgz-physics-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$physics_plugin_dir"
export GZ_PARTITION="modular_chassis_${RANDOM}_${RANDOM}"

server_pid=""
capture_pids=()
cleanup() {
  for pid in "${capture_pids[@]}"; do
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  if [[ -n "$server_pid" ]] && kill -0 "$server_pid" 2>/dev/null; then
    kill "$server_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$server_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$server_pid" 2>/dev/null; then
      kill -KILL "$server_pid" 2>/dev/null || true
    fi
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

/usr/bin/gz sim -r -s -v 2 --physics-engine "$engine_path" "$world" >"$out_dir/gazebo.log" 2>&1 &
server_pid=$!

for _ in $(seq 1 60); do
  if /usr/bin/gz topic -l | grep -q '/modular_chassis_integration/cmd_vel'; then
    break
  fi
  sleep 0.2
done

if ! /usr/bin/gz topic -l | grep -q '/modular_chassis_integration/cmd_vel'; then
  echo "ERROR: integration bench did not expose cmd_vel" >&2
  exit 1
fi

/usr/bin/gz topic -l >"$out_dir/topics.txt"
mapfile -t wrench_topics < <(grep '/sensor/connection_wrench/forcetorque$' "$out_dir/topics.txt")
if [[ "${#wrench_topics[@]}" -ne 4 ]]; then
  echo "ERROR: expected all 4 wheel wrench topics, found ${#wrench_topics[@]}" >&2
  exit 1
fi

# Let fixed constraints and tyre contacts settle before opening the capture
# streams. Otherwise the one-step spawn impulse dominates `maximum` despite
# having no relation to the commanded turn.
sleep 2

# XML validity does not imply all constraints were created by the engine.
if grep -Eq 'Asked to create a joint|No joint named|Failed to find plugin|Unable to create joint|multiple parent joints' "$out_dir/gazebo.log"; then
  echo "ERROR: physics engine rejected a joint/plugin; structural loads are invalid" >&2
  exit 1
fi

for topic in "${wrench_topics[@]}"; do
  joint="$(sed -E 's#^.*/joint/([^/]+)/sensor/.*#\1#' <<<"$topic")"
  timeout 9 /usr/bin/gz topic -e --json-output -t "$topic" \
    >"$out_dir/${joint}.jsonl" 2>"$out_dir/${joint}.capture.log" &
  capture_pids+=("$!")
done

timeout 9 /usr/bin/gz topic -e --json-output \
  -t /model/modular_test_chassis_v2/odometry \
  >"$out_dir/odometry.jsonl" 2>"$out_dir/odometry.capture.log" &
capture_pids+=("$!")

sleep 0.5
/usr/bin/gz topic -t /modular_chassis_integration/cmd_vel -m gz.msgs.Twist \
  -p 'angular: {z: 2.0}'
sleep 4
/usr/bin/gz topic -t /modular_chassis_integration/cmd_vel -m gz.msgs.Twist \
  -p 'angular: {z: 0.0}'
sleep 1

for pid in "${capture_pids[@]}"; do
  wait "$pid" 2>/dev/null || true
done
capture_pids=()

python3 "$bench_dir/summarize_wrenches.py" \
  "$out_dir"/*_joint.jsonl --output "$out_dir/wrench_summary.json"
python3 "$bench_dir/summarize_integration_motion.py" \
  "$out_dir/odometry.jsonl" --output "$out_dir/motion_summary.json"

python3 - "$out_dir" <<'PY'
import json
from pathlib import Path
import sys
directory = Path(sys.argv[1])
motion = json.loads((directory / "motion_summary.json").read_text())
loads = json.loads((directory / "wrench_summary.json").read_text())
if not motion["turn_executed"] or loads["global"]["maximum_force_norm_n"] <= 0:
    raise SystemExit("ERROR: no usable turn/load evidence; captured summaries are diagnostic only")
PY

if grep -Eq 'Asked to create a joint|No joint named|Failed to find plugin|Unable to create joint|multiple parent joints' "$out_dir/gazebo.log"; then
  echo "ERROR: Gazebo did not create the complete physical model" >&2
  exit 1
fi

echo "Modular integration bench completed: $out_dir"
