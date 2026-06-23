#!/usr/bin/env bash
# Motion-chain diagnosis for the tennis robot sim.
#
# Usage (sim must be running):
#   docker compose --profile gazebo exec gazebo bash /workspace/scripts/diagnose_motion.sh
#
# What it does:
#   1. Snapshots controller / hardware-interface / parameter state.
#   2. Records a rosbag of the whole cmd_vel chain into runtime/.
#   3. TEST A: publishes Twist directly to the diff_drive input (bypasses twist_mux).
#   4. TEST B: publishes Twist to /cmd_vel_teleop (through twist_mux).
#      After each test it samples wheel velocities from /joint_states.
#
# Interpretation:
#   A moves, B doesn't  -> twist_mux (config/locks/timeouts)
#   neither moves       -> diff_drive_controller or gz_ros2_control
#   both move           -> upstream (teleop/web panel/controller_node)
#
# Everything is written to runtime/motion_diag_<timestamp>/ (summary.log + bag/).
# NOTE: processes are stopped via their PIDs, never `pkill -f` (a -f pattern
# matches this script's own command line and kills the diagnostic itself).

# Source ROS before `set -u`: the setup scripts reference unbound variables
# (e.g. AMENT_TRACE_SETUP_FILES) and abort under nounset.
source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
set -u

CMD_TOPIC=/diff_drive_controller/cmd_vel_unstamped
TS=$(date +%Y%m%d_%H%M%S)
OUT=/workspace/runtime/motion_diag_${TS}
mkdir -p "$OUT"
SUMMARY="$OUT/summary.log"

log()  { echo -e "\n=== $* ===" | tee -a "$SUMMARY"; }
run()  { local title="$1"; shift; log "$title"; timeout 15 "$@" 2>&1 | tee -a "$SUMMARY"; }

sample_wheels() {
  log "wheel velocities ($1)"
  timeout 8 python3 - <<'PY' 2>&1 | tee -a "$SUMMARY"
import time
import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("diag_js_sample")
got = {}
node.create_subscription(JointState, "/joint_states", lambda m: got.update(m=m), 10)
deadline = time.time() + 6
while rclpy.ok() and "m" not in got and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
m = got.get("m")
if m is None:
    print("!! NO /joint_states message received")
else:
    vels = list(m.velocity) or [float("nan")] * len(m.name)
    for n, v in zip(m.name, vels):
        if "wheel" in n and "caster" not in n:
            print(f"  {n}: velocity={v:+.3f}")
node.destroy_node()
rclpy.shutdown()
PY
}

# ── 1. Static state ──────────────────────────────────────────────────────────
run "ros2 node list"                      ros2 node list
run "controllers"                         ros2 control list_controllers
run "hardware interfaces (claimed?)"      ros2 control list_hardware_interfaces
run "diff_drive cmd_vel_timeout (expect 5.0)" \
                                          ros2 param get /diff_drive_controller cmd_vel_timeout
run "diff_drive base_frame_id (expect base_footprint)" \
                                          ros2 param get /diff_drive_controller base_frame_id
run "controller_manager use_sim_time"     ros2 param get /controller_manager use_sim_time
run "sim clock running? (hz of /clock)"   ros2 topic hz /clock --window 50
run "topic info $CMD_TOPIC"               ros2 topic info -v "$CMD_TOPIC"
run "topic info /cmd_vel_teleop"          ros2 topic info -v /cmd_vel_teleop

# ── 2. rosbag of the whole chain ─────────────────────────────────────────────
log "starting rosbag record -> $OUT/bag"
ros2 bag record -o "$OUT/bag" \
  /cmd_vel_teleop /cmd_vel_nav /cmd_vel_collection \
  "$CMD_TOPIC" /joint_states /diff_drive_controller/odom /clock \
  >"$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
sleep 3

sample_wheels "baseline, no command"

# ── 3. TEST A: direct to controller (bypasses twist_mux) ────────────────────
log "TEST A: 0.3 m/s directly to $CMD_TOPIC (6 s)"
timeout 6 ros2 topic pub -r 10 "$CMD_TOPIC" geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' >/dev/null 2>&1 &
PUB_PID=$!
sleep 4
sample_wheels "during TEST A — nonzero => controller+gz OK"
wait "$PUB_PID" 2>/dev/null
sleep 3
sample_wheels "after TEST A — should return to ~0"

# ── 4. TEST B: through twist_mux ─────────────────────────────────────────────
log "TEST B: 0.3 m/s to /cmd_vel_teleop via twist_mux (6 s)"
timeout 6 ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' >/dev/null 2>&1 &
PUB_PID=$!
sleep 2
run "is twist_mux forwarding? (echo $CMD_TOPIC --once)" \
  ros2 topic echo "$CMD_TOPIC" --once
sample_wheels "during TEST B — nonzero => full chain OK"
wait "$PUB_PID" 2>/dev/null

# ── 4b. TEST C: lift wheel via ForwardCommandController ─────────────────────
# Exercises the same gz_ros2_control write path with a different controller.
# If the diff_drive wheels move but this does not, the issue is isolated to the
# lift controller rather than the shared gz_ros2_control write path.
log "TEST C: lift wheel 10 rad/s via /lift_wheel_velocity_controller/commands (5 s)"
timeout 5 ros2 topic pub -r 10 /lift_wheel_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray '{data: [10.0]}' >/dev/null 2>&1 &
PUB_PID=$!
sleep 3
sample_wheels "during TEST C — lift nonzero => gz write path OK"
wait "$PUB_PID" 2>/dev/null

# ── 5. wrap up ───────────────────────────────────────────────────────────────
kill -INT "$BAG_PID" 2>/dev/null
wait "$BAG_PID" 2>/dev/null

log "DONE — results in $OUT"
cat <<EOF | tee -a "$SUMMARY"
Interpretation:
  TEST A moves, TEST B doesn't -> twist_mux drops it (priorities/timeout/lock topic)
  Neither moves                -> diff_drive_controller or gz_ros2_control
                                  (check claimed interfaces + cmd_vel_timeout above)
  Both move                    -> problem is upstream (web panel / controller_node /
                                  teleop remappings), not the control stack
Replay the bag:   ros2 bag play $OUT/bag
Container logs:   docker compose logs gazebo | tail -200
EOF
