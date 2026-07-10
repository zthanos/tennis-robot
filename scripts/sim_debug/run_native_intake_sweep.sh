#!/usr/bin/env bash
# Native Ubuntu/Jazzy intake geometry sweep.
#
# Runs one headless Gazebo simulation per geometry config and writes per-run
# JSONL plus a combined summary CSV. The default "bench" driver bypasses
# perception / collect_one and drives a fixed robot pose directly into ball_02.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$SCRIPT_DIR"

ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-jazzy}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_TARGET/setup.bash"
WORKSPACE_SETUP="$SCRIPT_DIR/ros2_ws/install/setup.bash"

if [ ! -r "$ROS_SETUP" ]; then
    echo "ERROR: ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi
if [ ! -r "$WORKSPACE_SETUP" ]; then
    echo "ERROR: Workspace is not built. Run colcon build in ros2_ws first." >&2
    exit 1
fi

set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

export WORKSPACE="$SCRIPT_DIR"
export ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/gazebo/models:$SCRIPT_DIR/ros2_ws/src"
export ROBOT_COMMAND_FILE="$SCRIPT_DIR/runtime/robot_command.json"
export ROBOT_STATUS_FILE="$SCRIPT_DIR/runtime/robot_status.json"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$SCRIPT_DIR/runtime/ros_logs}"
export ROS_DOMAIN_ID="${INTAKE_SWEEP_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-$((100 + RANDOM % 100))}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/$ROS_DISTRO_TARGET/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
if [ -d "$ROS2_INSTALL/gz_ros2_control/lib" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROS2_INSTALL/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
fi

SITE_PKG=$(ls -d "$ROS2_INSTALL/tennis_robot/lib/python"*/site-packages 2>/dev/null | head -1 || true)
if [ -n "$SITE_PKG" ]; then
    export PYTHONPATH="$SITE_PKG:${PYTHONPATH:-}"
fi
VENV_SITE=$(ls -d "$SCRIPT_DIR/.venv/lib/python"*/site-packages 2>/dev/null | head -1 || true)
if [ -n "$VENV_SITE" ]; then
    export PYTHONPATH="$VENV_SITE:${PYTHONPATH:-}"
fi
export PYTHONPATH="$SCRIPT_DIR/ros2_ws/src/tennis_robot:$SCRIPT_DIR/ros2_ws/src/tennis_robot_msgs:${PYTHONPATH:-}"

mkdir -p "$SCRIPT_DIR/runtime" "$ROS_LOG_DIR"
export HOME="${INTAKE_SWEEP_HOME:-$SCRIPT_DIR/runtime/sweep_home}"
export ROS_HOME="${ROS_HOME:-$SCRIPT_DIR/runtime/ros_home}"
mkdir -p "$HOME" "$ROS_HOME" "$ROS_HOME/locks"

IFS=' ' read -r -a LIP_X_OFFSETS <<< "${INTAKE_SWEEP_LIP_X_OFFSETS:--0.003 -0.006 -0.009}"
IFS=' ' read -r -a LIP_HEIGHTS <<< "${INTAKE_SWEEP_LIP_HEIGHTS:-0.001 0.002 0.003}"
IFS=' ' read -r -a ROLLER_Z_OFFSETS <<< "${INTAKE_SWEEP_ROLLER_Z_OFFSETS:--0.003}"
IFS=' ' read -r -a RAMP_CLEAR_RUNS <<< "${INTAKE_SWEEP_RAMP_CLEAR_RUNS:-0.030}"
IFS=' ' read -r -a RAMP_CLEAR_ZS <<< "${INTAKE_SWEEP_RAMP_CLEAR_ZS:-0.004}"
IFS=' ' read -r -a BENCH_DRIVE_SPEEDS <<< "${INTAKE_SWEEP_DRIVE_SPEEDS:-${INTAKE_BENCH_DRIVE_SPEED:-0.12}}"
IFS=' ' read -r -a BENCH_ROLLER_SPEEDS <<< "${INTAKE_SWEEP_ROLLER_SPEEDS:-${INTAKE_BENCH_ROLLER_SPEED:-30.0}}"

ROLLER_X_OFFSET="${INTAKE_SWEEP_ROLLER_X_OFFSET:-0.015}"
ROLLER_BASE_X="0.600"
PROBE_DURATION="${INTAKE_SWEEP_PROBE_DURATION:-25}"
PROBE_PERIOD="${INTAKE_SWEEP_PROBE_PERIOD:-0.2}"
START_DISTANCE_M="${INTAKE_SWEEP_START_DISTANCE_M:-0.45}"
READY_TIMEOUT_S="${INTAKE_SWEEP_READY_TIMEOUT_S:-90}"
BALL_VISIBLE_TIMEOUT_S="${INTAKE_SWEEP_BALL_VISIBLE_TIMEOUT_S:-90}"
APPROACH_TIMEOUT_S="${INTAKE_SWEEP_APPROACH_TIMEOUT_S:-160}"
DRIVER="${INTAKE_SWEEP_DRIVER:-bench}"
BENCH_BALL_X="${INTAKE_BENCH_BALL_X:--6.4}"
BENCH_BALL_Y="${INTAKE_BENCH_BALL_Y:-0.0}"
BENCH_START_GAP_M="${INTAKE_BENCH_START_GAP_M:-0.78}"
BENCH_ROBOT_X="${INTAKE_BENCH_ROBOT_X:-$(python3 -c "print(float('$BENCH_BALL_X') - float('$BENCH_START_GAP_M'))")}"
BENCH_ROBOT_Y="${INTAKE_BENCH_ROBOT_Y:-$BENCH_BALL_Y}"
BENCH_ROBOT_Z="${INTAKE_BENCH_ROBOT_Z:-0.09}"
BENCH_ROBOT_YAW="${INTAKE_BENCH_ROBOT_YAW:-0.0}"
BENCH_SETTLE_S="${INTAKE_BENCH_SETTLE_S:-10}"
BENCH_ROLLER_LEAD_S="${INTAKE_BENCH_ROLLER_LEAD_S:-2}"
BENCH_ROLLER_READY_TIMEOUT_S="${INTAKE_BENCH_ROLLER_READY_TIMEOUT_S:-20}"
BENCH_DRIVE_RESPONSE_S="${INTAKE_BENCH_DRIVE_RESPONSE_S:-3}"
OUT_ROOT="${INTAKE_SWEEP_OUT_DIR:-$SCRIPT_DIR/runtime/intake_sweeps/$(date +%Y%m%d_%H%M%S)}"
SUMMARY_CSV="$OUT_ROOT/summary.csv"

mkdir -p "$OUT_ROOT"

launch_pid=""
drive_pub_pid=""
roller_pub_pid=""
pose_logger_pid=""

cleanup_publishers() {
    if [ -n "$drive_pub_pid" ] && kill -0 "$drive_pub_pid" >/dev/null 2>&1; then
        kill -- "-$drive_pub_pid" >/dev/null 2>&1 || kill "$drive_pub_pid" >/dev/null 2>&1 || true
        wait "$drive_pub_pid" >/dev/null 2>&1 || true
    fi
    drive_pub_pid=""
    if [ -n "$roller_pub_pid" ] && kill -0 "$roller_pub_pid" >/dev/null 2>&1; then
        kill -- "-$roller_pub_pid" >/dev/null 2>&1 || kill "$roller_pub_pid" >/dev/null 2>&1 || true
        wait "$roller_pub_pid" >/dev/null 2>&1 || true
    fi
    roller_pub_pid=""
    if [ -n "$pose_logger_pid" ] && kill -0 "$pose_logger_pid" >/dev/null 2>&1; then
        kill -- "-$pose_logger_pid" >/dev/null 2>&1 || kill "$pose_logger_pid" >/dev/null 2>&1 || true
        wait "$pose_logger_pid" >/dev/null 2>&1 || true
    fi
    pose_logger_pid=""
}

cleanup_launch() {
    cleanup_publishers
    if [ -n "$launch_pid" ] && kill -0 "$launch_pid" >/dev/null 2>&1; then
        kill -- "-$launch_pid" >/dev/null 2>&1 || kill "$launch_pid" >/dev/null 2>&1 || true
        wait "$launch_pid" >/dev/null 2>&1 || true
    fi
    launch_pid=""
}
trap cleanup_launch EXIT

wait_for_controllers() {
    local log_file="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if [ -r "$log_file" ] \
            && grep -q "Configured and activated diff_drive_controller" "$log_file" \
            && grep -q "Configured and activated lift_wheel_velocity_controller" "$log_file"; then
            return 0
        fi
        if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" \
            && ros2 control list_controllers 2>/dev/null | grep -q "lift_wheel_velocity_controller.*active"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

publish_stop_commands() {
    timeout 3 ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /lift_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0]}" >/dev/null 2>&1 || true
}

wait_for_roller_speed() {
    local min_abs_speed="$1"
    local timeout_s="$2"
    python3 - "$min_abs_speed" "$timeout_s" <<'PY'
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

target = abs(float(sys.argv[1]))
timeout_s = float(sys.argv[2])
last = None

rclpy.init()
node = rclpy.create_node("wait_for_intake_roller_speed")

def on_joint_states(msg):
    global last
    try:
        index = list(msg.name).index("lift_wheel_joint")
    except ValueError:
        return
    if index < len(msg.velocity):
        last = float(msg.velocity[index])

sub = node.create_subscription(JointState, "/joint_states", on_joint_states, 10)
deadline = time.time() + timeout_s
try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if last is not None and abs(last) >= target:
            print(f"roller_ready velocity={last:.3f}")
            raise SystemExit(0)
    print(f"roller_not_ready last={last}", file=sys.stderr)
    raise SystemExit(1)
finally:
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
PY
}

log_drive_response() {
    local out_path="$1"
    local duration_s="$2"
    python3 - "$out_path" "$duration_s" <<'PY'
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

out_path = sys.argv[1]
duration_s = float(sys.argv[2])
wheel_names = {
    "rear_left_wheel_joint",
    "front_left_wheel_joint",
    "rear_right_wheel_joint",
    "front_right_wheel_joint",
}
latest = {
    "cmd_vel_out": None,
    "odom": None,
    "wheel_velocity": {},
}
samples = {
    "cmd_vel_out": 0,
    "odom": 0,
    "joint_states": 0,
}

rclpy.init()
node = rclpy.create_node("intake_bench_drive_response")

def on_cmd(msg):
    samples["cmd_vel_out"] += 1
    latest["cmd_vel_out"] = {
        "linear_x": float(msg.linear.x),
        "angular_z": float(msg.angular.z),
    }

def on_odom(msg):
    samples["odom"] += 1
    latest["odom"] = {
        "x": float(msg.pose.pose.position.x),
        "y": float(msg.pose.pose.position.y),
        "vx": float(msg.twist.twist.linear.x),
        "wz": float(msg.twist.twist.angular.z),
    }

def on_joints(msg):
    samples["joint_states"] += 1
    velocities = {}
    for name, velocity in zip(msg.name, msg.velocity):
        if name in wheel_names:
            velocities[name] = float(velocity)
    if velocities:
        latest["wheel_velocity"] = velocities

subs = [
    node.create_subscription(Twist, "/diff_drive_controller/cmd_vel_out", on_cmd, 10),
    node.create_subscription(Odometry, "/diff_drive_controller/odom", on_odom, 10),
    node.create_subscription(JointState, "/joint_states", on_joints, 10),
]
deadline = time.time() + duration_s
try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    for sub in subs:
        node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()

report = {"samples": samples, "latest": latest}
with open(out_path, "w", encoding="utf-8") as handle:
    json.dump(report, handle, indent=2, sort_keys=True)
    handle.write("\n")
print(json.dumps(report, sort_keys=True))
PY
}

write_command() {
    local mode="$1"
    python3 - "$mode" <<'PY'
import json
import os
import time
import sys
from pathlib import Path

path = Path(os.environ["ROBOT_COMMAND_FILE"])
try:
    current = json.loads(path.read_text(encoding="utf-8"))
    seq = int(current.get("sequence", 0)) + 1
except Exception:
    seq = 1
payload = {
    "mode": sys.argv[1],
    "sequence": seq,
    "source": "native-intake-sweep",
    "updated_at": time.time(),
}
path.parent.mkdir(parents=True, exist_ok=True)
tmp = path.with_name(f"{path.name}.{os.getpid()}.{time.time_ns()}.tmp")
tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
tmp.replace(path)
print(f"{payload['mode']} seq={seq}")
PY
}

wait_for_status() {
    local since="$1"
    local timeout_s="$2"
    python3 - "$ROBOT_STATUS_FILE" "$since" "$timeout_s" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
since = float(sys.argv[2])
timeout_s = float(sys.argv[3])
deadline = time.time() + timeout_s
while time.time() < deadline:
    try:
        if path.stat().st_mtime >= since:
            data = json.loads(path.read_text(encoding="utf-8"))
            if data.get("updated_at", 0) >= since:
                print(json.dumps({
                    "mode": data.get("mode"),
                    "state": data.get("collector_state"),
                    "distance": data.get("ball_distance_m"),
                }))
                raise SystemExit(0)
    except Exception:
        pass
    time.sleep(0.5)
raise SystemExit(1)
PY
}

wait_until_ball_visible() {
    local timeout_s="$1"
    python3 - "$ROBOT_STATUS_FILE" "$timeout_s" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
timeout_s = float(sys.argv[2])
deadline = time.time() + timeout_s
last = None
while time.time() < deadline:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        time.sleep(0.5)
        continue
    visible = bool(data.get("ball_visible"))
    dist = data.get("ball_distance_m")
    state = data.get("collector_state")
    line = f"state={state} visible={int(visible)} distance={dist}"
    if line != last:
        print(line, flush=True)
        last = line
    if visible and isinstance(dist, (int, float)):
        raise SystemExit(0)
    time.sleep(0.5)
raise SystemExit(1)
PY
}

wait_until_close() {
    local timeout_s="$1"
    python3 - "$ROBOT_STATUS_FILE" "$START_DISTANCE_M" "$timeout_s" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
target = float(sys.argv[2])
timeout_s = float(sys.argv[3])
deadline = time.time() + timeout_s
last = None
seen_ball = False
while time.time() < deadline:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        time.sleep(0.5)
        continue
    state = data.get("collector_state")
    dist = data.get("ball_distance_m")
    visible = bool(data.get("ball_visible"))
    line = f"state={state} visible={int(visible)} distance={dist}"
    if line != last:
        print(line, flush=True)
        last = line
    if visible and isinstance(dist, (int, float)):
        seen_ball = True
    if visible and isinstance(dist, (int, float)) and dist <= target:
        raise SystemExit(0)
    if seen_ball and state in {"capture", "reverse_clear"}:
        raise SystemExit(0)
    time.sleep(0.5)
raise SystemExit(1)
PY
}

run_probe_and_summarize() {
    local case_dir="$1"
    python3 -m tennis_robot.sim_physics_probe \
        --duration "$PROBE_DURATION" \
        --period "$PROBE_PERIOD" \
        --jsonl "$case_dir/contact_physics.jsonl" \
        > "$case_dir/probe.log" 2>&1

    cp "$ROBOT_STATUS_FILE" "$case_dir/robot_status.json" 2>/dev/null || true
    python3 "$SCRIPT_DIR/scripts/sim_debug/summarize_contact_physics.py" \
        "$case_dir/contact_physics.jsonl" \
        --status "$case_dir/robot_status.json" \
        --json-out "$case_dir/summary.json" \
        --csv-append "$SUMMARY_CSV" \
        > "$case_dir/summary.pretty.json"

    if [ -s "$case_dir/gz_poses.jsonl" ]; then
        python3 "$SCRIPT_DIR/scripts/sim_debug/analyze_intake_bench_poses.py" \
            "$case_dir/gz_poses.jsonl" \
            --ball-name "${INTAKE_BENCH_BALL_NAME:-ball_02}" \
            --roller-x-offset-m "$INTAKE_ROLLER_X_OFFSET_M" \
            --roller-z-offset-m "$INTAKE_ROLLER_Z_OFFSET_M" \
            --base-link-height-m "${INTAKE_BENCH_BASE_LINK_Z:-0.045}" \
            --json-out "$case_dir/pose_summary.json" \
            > "$case_dir/pose_summary.pretty.json"
        local ramp_entry_x
        ramp_entry_x="$(python3 -c "print(float('$ROLLER_BASE_X') + float('$INTAKE_ROLLER_X_OFFSET_M') + float('$INTAKE_LIP_X_OFFSET_M'))")"
        local force_threshold_args=()
        if [ -n "${INTAKE_BENCH_FORCE_P95_THRESHOLD_N:-}" ]; then
            force_threshold_args=(--force-p95-threshold-n "$INTAKE_BENCH_FORCE_P95_THRESHOLD_N")
        fi
        python3 "$SCRIPT_DIR/scripts/sim_debug/analyze_intake_release_criteria.py" \
            "$case_dir/contact_physics.jsonl" \
            "$case_dir/gz_poses.jsonl" \
            --ball-name "${INTAKE_BENCH_BALL_NAME:-ball_02}" \
            --ramp-entry-x-m "$ramp_entry_x" \
            --ramp-crest-z-m "${INTAKE_BENCH_RAMP_CREST_Z_M:-0.138}" \
            --preferred-contact-duration-s "${INTAKE_BENCH_PREFERRED_CONTACT_DURATION_S:-0.50}" \
            --preferred-speed-m-s "${INTAKE_BENCH_PREFERRED_RELEASE_SPEED_M_S:-0.40}" \
            --front-lip-zone-m "${INTAKE_BENCH_FRONT_LIP_ZONE_M:-0.008}" \
            "${force_threshold_args[@]}" \
            --json-out "$case_dir/release_criteria.json" \
            > "$case_dir/release_criteria.pretty.json"
    fi
}

run_bench_driver() {
    local case_dir="$1"

    export SIM_ROBOT_SPAWN_X="$BENCH_ROBOT_X"
    export SIM_ROBOT_SPAWN_Y="$BENCH_ROBOT_Y"
    export SIM_ROBOT_SPAWN_Z="$BENCH_ROBOT_Z"
    export SIM_ROBOT_SPAWN_YAW="$BENCH_ROBOT_YAW"
    export SIM_BENCH_MINIMAL=true
    export SIM_SKIP_CONTROL_PANEL=true
    export INTAKE_BENCH_BALL_X="$BENCH_BALL_X"
    export INTAKE_BENCH_BALL_Y="$BENCH_BALL_Y"
    export INTAKE_BENCH_BALL_Z="${INTAKE_BENCH_BALL_Z:-0.033}"
    export INTAKE_BENCH_ROBOT_X="$BENCH_ROBOT_X"
    export INTAKE_BENCH_ROBOT_Y="$BENCH_ROBOT_Y"
    export INTAKE_BENCH_ROBOT_Z="$BENCH_ROBOT_Z"
    export INTAKE_BENCH_BASE_LINK_Z="${INTAKE_BENCH_BASE_LINK_Z:-0.045}"
    export INTAKE_BENCH_ROBOT_YAW="$BENCH_ROBOT_YAW"
    export INTAKE_BENCH_DRIVE_SPEED="$BENCH_DRIVE_SPEED"

    {
        echo "driver=bench"
        echo "ball_x=$BENCH_BALL_X"
        echo "ball_y=$BENCH_BALL_Y"
        echo "robot_x=$SIM_ROBOT_SPAWN_X"
        echo "robot_y=$SIM_ROBOT_SPAWN_Y"
        echo "robot_z=$SIM_ROBOT_SPAWN_Z"
        echo "robot_yaw=$SIM_ROBOT_SPAWN_YAW"
        echo "drive_speed=$BENCH_DRIVE_SPEED"
        echo "roller_speed=$BENCH_ROLLER_SPEED"
        echo "settle_s=$BENCH_SETTLE_S"
        echo "roller_lead_s=$BENCH_ROLLER_LEAD_S"
        echo "roller_ready_timeout_s=$BENCH_ROLLER_READY_TIMEOUT_S"
        echo "drive_response_s=$BENCH_DRIVE_RESPONSE_S"
    } > "$case_dir/bench_config.txt"

    setsid ros2 launch tennis_robot sim.launch.py headless:=true > "$case_dir/launch.log" 2>&1 &
    launch_pid="$!"

    if ! wait_for_controllers "$case_dir/launch.log" "$READY_TIMEOUT_S" > "$case_dir/controllers_ready.log" 2>&1; then
        echo "FAILED: controllers did not become active; see $case_dir/launch.log" >&2
        cleanup_launch
        return 1
    fi
    ros2 topic list | sort > "$case_dir/ros_topics.txt" 2>&1 || true
    ros2 topic info /diff_drive_controller/cmd_vel_unstamped \
        > "$case_dir/cmd_vel_unstamped.info" 2>&1 || true
    ros2 topic info /diff_drive_controller/cmd_vel \
        > "$case_dir/cmd_vel_stamped.info" 2>&1 || true

    setsid python3 "$SCRIPT_DIR/scripts/sim_debug/log_gz_poses.py" "$case_dir/gz_poses.jsonl" \
        > "$case_dir/gz_poses.log" 2>&1 &
    pose_logger_pid="$!"

    sleep "$BENCH_SETTLE_S"

    setsid ros2 topic pub --rate 20 /lift_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [$BENCH_ROLLER_SPEED]}" \
        > "$case_dir/roller_pub.log" 2>&1 &
    roller_pub_pid="$!"
    sleep "$BENCH_ROLLER_LEAD_S"
    if ! wait_for_roller_speed 1.0 "$BENCH_ROLLER_READY_TIMEOUT_S" > "$case_dir/roller_ready.log" 2>&1; then
        echo "FAILED: roller did not spin before drive; see $case_dir/roller_ready.log" >&2
        cleanup_launch
        return 1
    fi

    setsid ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: base_footprint}, twist: {linear: {x: $BENCH_DRIVE_SPEED}, angular: {z: 0.0}}}" \
        > "$case_dir/drive_pub.log" 2>&1 &
    drive_pub_pid="$!"

    sleep 1
    ros2 topic info /diff_drive_controller/cmd_vel_unstamped \
        > "$case_dir/cmd_vel_unstamped_after_pub.info" 2>&1 || true
    ros2 topic info /diff_drive_controller/cmd_vel \
        > "$case_dir/cmd_vel_stamped_after_pub.info" 2>&1 || true
    log_drive_response "$case_dir/drive_response.json" "$BENCH_DRIVE_RESPONSE_S" > "$case_dir/drive_response.log" 2>&1 || true

    run_probe_and_summarize "$case_dir"
    cleanup_publishers
    publish_stop_commands
    cleanup_launch
}

run_collect_one_driver() {
    local case_dir="$1"

    unset SIM_ROBOT_SPAWN_X SIM_ROBOT_SPAWN_Y SIM_ROBOT_SPAWN_Z SIM_ROBOT_SPAWN_YAW
    unset SIM_BENCH_MINIMAL SIM_SKIP_CONTROL_PANEL
    local start_time
    start_time="$(python3 -c 'import time; print(time.time())')"
    setsid ros2 launch tennis_robot sim.launch.py headless:=true > "$case_dir/launch.log" 2>&1 &
    launch_pid="$!"

    if ! wait_for_status "$start_time" "$READY_TIMEOUT_S" > "$case_dir/ready.json"; then
        echo "FAILED: status did not become fresh; see $case_dir/launch.log" >&2
        cleanup_launch
        return 1
    fi

    write_command idle > "$case_dir/idle_command.txt"
    sleep 1
    if ! wait_until_ball_visible "$BALL_VISIBLE_TIMEOUT_S" > "$case_dir/ball_visible.log"; then
        echo "FAILED: no visible target ball before collect_one; see $case_dir/ball_visible.log" >&2
        cleanup_launch
        return 1
    fi
    write_command collect_one > "$case_dir/collect_command.txt"

    if ! wait_until_close "$APPROACH_TIMEOUT_S" > "$case_dir/approach.log"; then
        echo "FAILED: ball did not reach probe start window; see $case_dir/approach.log" >&2
        cleanup_launch
        return 1
    fi

    run_probe_and_summarize "$case_dir"
    write_command idle > "$case_dir/final_idle_command.txt" || true
    cleanup_launch
}

run_case() {
    local lip_x="$1"
    local lip_h="$2"
    local roller_z="$3"
    local ramp_clear_run="$4"
    local ramp_clear_z="$5"
    local drive_speed="$6"
    local roller_speed="$7"
    local case_name="lipx_${lip_x}_liph_${lip_h}_rollerz_${roller_z}_clearrun_${ramp_clear_run}_clearz_${ramp_clear_z}_drive_${drive_speed}_rollerspeed_${roller_speed}"
    case_name="${case_name//- /}"
    case_name="${case_name//./p}"
    case_name="${case_name//-/m}"
    local case_dir="$OUT_ROOT/$case_name"
    mkdir -p "$case_dir"

    export INTAKE_LIP_X_OFFSET_M="$lip_x"
    export INTAKE_LIP_RAISE_M="$lip_h"
    export INTAKE_ROLLER_X_OFFSET_M="$ROLLER_X_OFFSET"
    export INTAKE_ROLLER_Z_OFFSET_M="$roller_z"
    export INTAKE_RAMP_CLEAR_RUN_M="$ramp_clear_run"
    export INTAKE_RAMP_CLEAR_Z_M="$ramp_clear_z"
    export BENCH_DRIVE_SPEED="$drive_speed"
    export BENCH_ROLLER_SPEED="$roller_speed"

    echo
    echo "=== $case_name ==="
    echo "lip_x=$lip_x lip_h=$lip_h roller_z=$roller_z ramp_clear_run=$ramp_clear_run ramp_clear_z=$ramp_clear_z drive_speed=$drive_speed roller_speed=$roller_speed"

    python3 "$SCRIPT_DIR/scripts/generate_curved_scoop_mesh.py" > "$case_dir/generate_scoop.log" 2>&1

    case "$DRIVER" in
        bench)
            run_bench_driver "$case_dir"
            ;;
        collect_one)
            run_collect_one_driver "$case_dir"
            ;;
        *)
            echo "ERROR: unknown INTAKE_SWEEP_DRIVER=$DRIVER (use bench or collect_one)" >&2
            return 1
            ;;
    esac
}

echo "Output: $OUT_ROOT"
echo "Summary CSV: $SUMMARY_CSV"

for roller_z in "${ROLLER_Z_OFFSETS[@]}"; do
    for ramp_clear_z in "${RAMP_CLEAR_ZS[@]}"; do
        for ramp_clear_run in "${RAMP_CLEAR_RUNS[@]}"; do
            for lip_h in "${LIP_HEIGHTS[@]}"; do
                for lip_x in "${LIP_X_OFFSETS[@]}"; do
                    for drive_speed in "${BENCH_DRIVE_SPEEDS[@]}"; do
                        for roller_speed in "${BENCH_ROLLER_SPEEDS[@]}"; do
                            run_case "$lip_x" "$lip_h" "$roller_z" "$ramp_clear_run" "$ramp_clear_z" "$drive_speed" "$roller_speed"
                        done
                    done
                done
            done
        done
    done
done

echo
echo "Sweep complete: $SUMMARY_CSV"
