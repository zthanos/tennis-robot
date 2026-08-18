#!/usr/bin/env bash
# Phase 10: run one deterministic collection scenario against the live sim and
# collect every artifact needed to evaluate it.
#
# Read-only with respect to production behaviour: it places balls, asks the
# controller for a collect_route, waits, and copies artifacts out.  Nothing here
# tunes the planner, controller or collector.
#
#   scripts/sim_debug/run_phase10_scenario.sh <scenario> [repetition]
#
# Bring the stack up FIRST, detached, and leave it up for the whole campaign:
#
#   UBUNTU_GPU=false GAZEBO_HEADLESS=true \
#     COLLECTION_ROUTE_AUDIT_DIR=/workspace/runtime/route_audit/phase10 \
#     COLLECTION_EXECUTION_TRACE_DIR=/workspace/runtime/route_audit/phase10 \
#     ./run_ubuntu.sh
#
# UBUNTU_GPU=false is required on this machine: with GPU headless rendering
# `gz sim` dies with SIGSEGV ("eglInitialize failed ... /dev/dri/card2"), which
# leaves slam_toolbox waiting for a /scan that never arrives.  Never wrap
# run_ubuntu.sh in `timeout` -- killing it tears the stack down mid-route.
#
# Scenarios match tests/collection_execution_fixtures.py:
#   straight_sweep two_passes_with_connector connector_collects near_miss
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."

SCENARIO="${1:?scenario name required}"
REPETITION="${2:-1}"
OUT_ROOT="runtime/route_audit/phase10"
RUN_DIR="$OUT_ROOT/${SCENARIO}-r${REPETITION}"
WORLD="tennis_court"
TIMEOUT_S="${PHASE10_TIMEOUT_S:-420}"

COMPOSE=(docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml --profile gazebo)

# Ball layouts in map coordinates, chosen to match the offline scenarios.  The
# court model already contains ball_00..ball_NN; a scenario parks the ones it
# does not use far off court rather than deleting them, so every run starts from
# the same world.
case "$SCENARIO" in
  straight_sweep)              LAYOUT="3.2,0.0 3.9,0.0 4.6,0.0" ;;
  two_passes_with_connector)   LAYOUT="3.0,1.2 3.7,1.2 3.0,-1.2 3.7,-1.2" ;;
  connector_collects)          LAYOUT="3.0,0.0 3.7,0.0 4.6,0.0 5.5,0.0 6.2,0.0" ;;
  near_miss)                   LAYOUT="3.2,0.0 3.9,0.0 4.6,0.55" ;;
  real_scan)                   LAYOUT="" ;;   # uses whatever the world holds
  *) echo "unknown scenario: $SCENARIO" >&2; exit 2 ;;
esac

# Scenario coordinates are in the MAP frame the planner uses; Gazebo's world
# frame has the court centred on its origin, so the two differ by the net centre
# recorded in court_boundary.json (~8.08 m in x).  Placing scenario balls in map
# coordinates without this shift puts them behind the net, where the robot never
# sees them and every scan comes back empty -- which looks exactly like a broken
# perception stack.
read -r MAP_TO_WORLD_X MAP_TO_WORLD_Y <<<"$(python3 -c "
import json
net = json.load(open('runtime/court_boundary.json'))['net']['center']
print(net['x_m'], net['y_m'])
")"

gz_set_pose() {  # name map_x map_y
  local world_x world_y
  world_x=$(python3 -c "print($2 - $MAP_TO_WORLD_X)")
  world_y=$(python3 -c "print($3 - $MAP_TO_WORLD_Y)")
  "${COMPOSE[@]}" exec -T gazebo bash -lc \
    "gz service -s /world/$WORLD/set_pose --reqtype gz.msgs.Pose \
       --reptype gz.msgs.Boolean --timeout 2000 \
       --req 'name: \"$1\", position: {x: $world_x, y: $world_y, z: 0.033}'" >/dev/null 2>&1
}

echo "== $SCENARIO repetition $REPETITION =="
mkdir -p "$RUN_DIR"

if [ -n "$LAYOUT" ]; then
  index=0
  for pair in $LAYOUT; do
    name=$(printf "ball_%02d" "$index")
    gz_set_pose "$name" "${pair%,*}" "${pair#*,}"
    index=$((index + 1))
  done
  # Park every other ball well outside the court so the scenario is exactly
  # the balls it declares.
  for spare in $(seq "$index" 19); do
    gz_set_pose "$(printf "ball_%02d" "$spare")" "$((40 + spare))" "40"
  done
  sleep 2
fi

BEFORE=$(ls "$OUT_ROOT" 2>/dev/null | wc -l)

python3 - "$SCENARIO" <<'PY'
import json, sys, time
path = "runtime/robot_command.json"
try:
    command = json.load(open(path))
except Exception:
    command = {}
command.update({
    "mode": "collect_route",
    "sequence": int(command.get("sequence", 0)) + 1,
    "source": f"phase10-{sys.argv[1]}",
    "updated_at": time.time(),
})
json.dump(command, open(path, "w"), indent=2)
print("requested collect_route")
PY

echo "waiting up to ${TIMEOUT_S}s for the route to finish..."
deadline=$(( $(date +%s) + TIMEOUT_S ))
outcome="timeout"
while [ "$(date +%s)" -lt "$deadline" ]; do
  mode=$(python3 -c "
import json
try:
    print(json.load(open('runtime/robot_status.json')).get('actual_mode',''))
except Exception:
    print('')
" 2>/dev/null)
  if [ "$mode" = "idle" ] && [ -n "$(ls "$OUT_ROOT"/*.trace.json 2>/dev/null)" ]; then
    outcome="finished"; break
  fi
  sleep 5
done
echo "outcome: $outcome"

# Collect everything that identifies this run.
cp -f "$OUT_ROOT"/*.json "$RUN_DIR"/ 2>/dev/null || true
cp -f runtime/robot_status.json "$RUN_DIR"/robot_status.json 2>/dev/null || true
tail -400 runtime/collection_events.jsonl > "$RUN_DIR/collection_events.tail.jsonl" 2>/dev/null || true
"${COMPOSE[@]}" logs --no-color --tail 600 gazebo > "$RUN_DIR/gazebo.log" 2>/dev/null || true

AUDIT=$(ls "$RUN_DIR"/collection-scan-*.json 2>/dev/null | head -1)
TRACE=$(ls "$RUN_DIR"/*.trace.json 2>/dev/null | head -1)
if [ -n "$AUDIT" ] && [ -n "$TRACE" ]; then
  echo "== evaluator =="
  uv run python scripts/sim_debug/collection_execution_report.py \
    --audit-artifact "$AUDIT" --trace "$TRACE" | tee "$RUN_DIR/report.txt"
  uv run python scripts/sim_debug/collection_execution_report.py \
    --audit-artifact "$AUDIT" --trace "$TRACE" --json > "$RUN_DIR/report.json"
else
  echo "MISSING ARTIFACTS: audit='$AUDIT' trace='$TRACE'" | tee "$RUN_DIR/report.txt"
  exit 1
fi
