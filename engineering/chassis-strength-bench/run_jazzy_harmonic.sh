#!/usr/bin/env bash
set -euo pipefail

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${CHASSIS_BENCH_OUT_DIR:-$BENCH_DIR/results/jazzy_harmonic}"
WORLD="$BENCH_DIR/gazebo/chassis_turn_bench.sdf"
mkdir -p "$OUT_DIR"

# The ROS vendor setup prepends a gz wrapper without the system sim command.
# GZ_CONFIG_PATH restores the Harmonic sim8 command registered by Ubuntu.
export GZ_CONFIG_PATH=/usr/share/gz
# The host runtime package exposes only ABI-versioned plugin symlinks, while
# gz-sim requests the unversioned filename. Provide that name inside the bench
# output instead of modifying /usr.
physics_plugin_dir="$OUT_DIR/physics_plugins"
mkdir -p "$physics_plugin_dir"
ln -sfn \
  /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 \
  "$physics_plugin_dir/libgz-physics7-dartsim-plugin.so"
ln -sfn \
  /usr/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7 \
  "$physics_plugin_dir/libgz-physics-dartsim-plugin.so"
export GZ_SIM_PHYSICS_ENGINE_PATH="$physics_plugin_dir"
export GZ_PARTITION="chassis_strength_${RANDOM}_${RANDOM}"

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

/usr/bin/gz sim -r -s -v 2 "$WORLD" >"$OUT_DIR/gazebo.log" 2>&1 &
server_pid=$!

for _ in $(seq 1 50); do
  if /usr/bin/gz topic -l | grep -q '/chassis_strength_bench/cmd_vel'; then
    break
  fi
  sleep 0.2
done

if ! /usr/bin/gz topic -l | grep -q '/chassis_strength_bench/cmd_vel'; then
  echo "ERROR: Gazebo benchmark did not expose cmd_vel" >&2
  exit 1
fi

/usr/bin/gz topic -l >"$OUT_DIR/topics.txt"

mapfile -t wrench_topics < <(grep '/sensor/wrench/forcetorque$' "$OUT_DIR/topics.txt")
if [[ "${#wrench_topics[@]}" -ne 4 ]]; then
  echo "ERROR: expected four force/torque topics, found ${#wrench_topics[@]}" >&2
  exit 1
fi
for topic in "${wrench_topics[@]}"; do
  joint="$(sed -E 's#^.*/joint/([^/]+)/sensor/.*#\1#' <<<"$topic")"
  timeout 8 /usr/bin/gz topic -e --json-output -t "$topic" \
    >"$OUT_DIR/${joint}.jsonl" 2>"$OUT_DIR/${joint}.capture.log" &
  capture_pids+=("$!")
done

# Settle, then command a deliberately aggressive turn-in-place for 4 seconds.
sleep 1
/usr/bin/gz topic -t /chassis_strength_bench/cmd_vel -m gz.msgs.Twist \
  -p 'angular: {z: 2.0}'
sleep 4
/usr/bin/gz topic -t /chassis_strength_bench/cmd_vel -m gz.msgs.Twist \
  -p 'angular: {z: 0.0}'
sleep 1

for pid in "${capture_pids[@]}"; do
  wait "$pid" 2>/dev/null || true
done
capture_pids=()

python3 "$BENCH_DIR/summarize_wrenches.py" \
  "$OUT_DIR"/*_joint.jsonl --output "$OUT_DIR/wrench_summary.json"

if grep -q 'Failed to find plugin' "$OUT_DIR/gazebo.log"; then
  echo "ERROR: Gazebo produced wrench topics without a physics engine" >&2
  exit 1
fi

echo "Gazebo Jazzy/Harmonic bench completed: $OUT_DIR"
echo "Four joint wrench streams captured as JSONL."
