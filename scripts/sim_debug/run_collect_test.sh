#!/usr/bin/env bash
# Orchestrates the instrumented collect_one test:
# recorders on -> collect_one -> wait for capture attempt -> recorders off -> analyze.
set -u
cd /home/thanosz/projects/diy/tennis-robot

COMPOSE=(docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml -f docker-compose.ubuntu-gpu.yml --profile gazebo)
TEST="${TEST_NAME:-collect_test1}"
Z_OFF="${TEST_Z_OFF:--0.005}"
SEQ="${TEST_SEQ:-62}"
BAGDIR=/workspace/runtime/bags/$TEST

# Guard against the restart race: the container must be up for a while
# (fresh `docker compose logs` match can come from a container that is about
# to be torn down by a concurrent ./run_ubuntu.sh restart).
UPTIME_OK=false
for _ in $(seq 1 60); do
  STATUS=$("${COMPOSE[@]}" ps --format '{{.Status}}' gazebo 2>/dev/null | head -1)
  case "$STATUS" in
    Up\ About\ a\ minute*|Up\ [1-9]\ minutes*|Up\ [0-9][0-9]*\ minutes*|Up\ *hour*) UPTIME_OK=true; break ;;
  esac
  sleep 5
done
if [ "$UPTIME_OK" != "true" ]; then
  echo "FAILED: gazebo container not stably up"; exit 1
fi

"${COMPOSE[@]}" exec -T gazebo bash -lc "rm -rf $BAGDIR ${BAGDIR}_frames ${BAGDIR}_analysis ${BAGDIR}_poses.jsonl; mkdir -p /workspace/runtime/bags"

# 1. recorders (self-terminate at 240s as safety)
"${COMPOSE[@]}" exec -T gazebo bash -lc '
. /opt/ros/humble/setup.sh
timeout 240 ros2 bag record -o '"$BAGDIR"' \
  /gz/roller_contact_0 /gz/roller_contact_1 /gz/roller_contact_2 /gz/roller_contact_3 /gz/roller_contact_4 /gz/roller_contact_5 /gz/roller_contact_6 /gz/roller_contact_7 /joint_states /collector/intake_beam_broken \
  /odom /cmd_vel /sim/roller_contact > /tmp/bagrec.log 2>&1 &
timeout 240 python3 /workspace/scripts/sim_debug/log_gz_poses.py '"${BAGDIR}_poses.jsonl"' > /tmp/poselog.log 2>&1 &
timeout 240 python3 /workspace/scripts/sim_debug/dump_intake_frames.py '"${BAGDIR}_frames"' > /tmp/framedump.log 2>&1 &
sleep 3
echo recorders-started
' || { echo "FAILED to start recorders"; exit 1; }

# 2. trigger collect_one
# Written from inside the container: the controller (root) owns the file on
# the bind mount, so a host-side write hits EACCES.
"${COMPOSE[@]}" exec -T gazebo python3 -c "
import json, time
json.dump({'mode':'collect_one','sequence':$SEQ,'source':'claude-rosbag-test','updated_at':time.time()},
          open('/workspace/runtime/robot_command.json','w'), indent=2)
" || { echo "FAILED to send collect_one"; exit 1; }
echo "collect_one sent (seq $SEQ)"

# 3. wait for a capture attempt to happen and end
python3 - <<'EOF'
import json, time
start = time.time()
saw_capture = False
last = ""
while time.time() - start < 210:
    try:
        d = json.load(open("runtime/robot_status.json"))
    except Exception:
        time.sleep(1); continue
    state = d.get("collector_state", "?")
    mode = d.get("mode", "?")
    got = d.get("balls_collected", 0)
    line = f"{mode}/{state}/collected={got}"
    if line != last:
        print(f"t={time.time()-start:5.1f}s {line}", flush=True)
        last = line
    if state == "capture":
        saw_capture = True
    if got > 0:
        print("SUCCESS: ball collected!", flush=True); break
    if saw_capture and state != "capture":
        # give it a moment to settle, then one more check
        time.sleep(3)
        d = json.load(open("runtime/robot_status.json"))
        if d.get("balls_collected", 0) > 0:
            print("SUCCESS: ball collected (late)!", flush=True)
        else:
            print("CAPTURE ATTEMPT ENDED without collection", flush=True)
        break
    time.sleep(1)
else:
    print("TIMEOUT waiting for capture attempt", flush=True)
EOF

# 4. stop recorders
"${COMPOSE[@]}" exec -T gazebo bash -lc 'pkill -INT -f "ros2 bag record" ; pkill -f log_gz_poses ; pkill -f dump_intake_frames; sleep 2; echo recorders-stopped'

# 5. analyze
"${COMPOSE[@]}" exec -T gazebo bash -lc '
. /opt/ros/humble/setup.sh
export INTAKE_ROLLER_X_OFFSET_M=0.015 INTAKE_ROLLER_Z_OFFSET_M='"$Z_OFF"'
python3 /workspace/scripts/sim_debug/analyze_collect_bag.py '"$BAGDIR $BAGDIR"'_poses.jsonl '"$BAGDIR"'_frames '"$BAGDIR"'_analysis
'
echo "=== TEST DONE ==="
