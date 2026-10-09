#!/usr/bin/env bash
set -euo pipefail

bench_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
payload="${CHASSIS_PAYLOAD_MASS_KG:-0}"
case_name="payload_${payload//./p}kg"
out_dir="${CHASSIS_INTEGRATION_OUT_DIR:-$bench_dir/results/modular_integration/$case_name}"
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
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

/usr/bin/gz sim -r -s -v 2 "$world" >"$out_dir/gazebo.log" 2>&1 &
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
if [[ "${#wrench_topics[@]}" -lt 12 ]]; then
  echo "ERROR: expected at least 12 connection/wheel wrench topics, found ${#wrench_topics[@]}" >&2
  exit 1
fi

# Let fixed constraints and tyre contacts settle before opening the capture
# streams. Otherwise the one-step spawn impulse dominates `maximum` despite
# having no relation to the commanded turn.
sleep 2

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

if grep -q 'Failed to find plugin' "$out_dir/gazebo.log"; then
  echo "ERROR: Gazebo produced topics without a physics engine" >&2
  exit 1
fi

echo "Modular integration bench completed: $out_dir"
