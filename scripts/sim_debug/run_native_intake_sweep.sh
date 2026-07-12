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

# Dual-wheel intake sweep axes (docs/dual-wheel-intake-design-el.md).
IFS=' ' read -r -a WHEEL_GAPS <<< "${INTAKE_SWEEP_WHEEL_GAPS:-0.056}"
IFS=' ' read -r -a WHEEL_RADII <<< "${INTAKE_SWEEP_WHEEL_RADII:-0.060}"
IFS=' ' read -r -a NIP_XS <<< "${INTAKE_SWEEP_NIP_XS:-0.540}"
IFS=' ' read -r -a WHEEL_TILTS_DEG <<< "${INTAKE_SWEEP_WHEEL_TILTS_DEG:-${INTAKE_WHEEL_TILT_DEG:-35.0}}"
WHEEL_MAX_VEL="${INTAKE_WHEEL_MAX_VEL_RAD_S:-26.3}"
WHEEL_EFFORT="${INTAKE_WHEEL_EFFORT_NM:-1.77}"
IFS=' ' read -r -a SPRING_KS <<< "${INTAKE_SWEEP_SPRING_KS:-1000}"
IFS=' ' read -r -a BENCH_DRIVE_SPEEDS <<< "${INTAKE_SWEEP_DRIVE_SPEEDS:-${INTAKE_BENCH_DRIVE_SPEED:-0.12}}"
IFS=' ' read -r -a BENCH_WHEEL_SPEEDS <<< "${INTAKE_SWEEP_WHEEL_SPEEDS:-${INTAKE_BENCH_WHEEL_SPEED:-25.0}}"
IFS=' ' read -r -a BALL_LATERAL_OFFSETS <<< "${INTAKE_SWEEP_BALL_LATERAL_OFFSETS:-0.0}"
ENABLE_ASSIST="${INTAKE_ENABLE_ASSIST:-false}"
ASSIST_SPEED="${INTAKE_ASSIST_SPEED:-25.0}"
ASSIST_X="${INTAKE_ASSIST_X_M:-0.545}"
ASSIST_Z="${INTAKE_ASSIST_Z_M:-0.050}"
ASSIST_RADIUS="${INTAKE_ASSIST_RADIUS_M:-0.030}"
ASSIST_LENGTH="${INTAKE_ASSIST_LENGTH_M:-0.200}"
ENABLE_CONVEYOR="${INTAKE_ENABLE_CONVEYOR:-false}"
CONVEYOR_SPEED="${INTAKE_CONVEYOR_SPEED:-25.0}"
CONVEYOR_X_BIAS="${INTAKE_CONVEYOR_X_BIAS_M:-0.000}"
CONVEYOR_Z_BIAS="${INTAKE_CONVEYOR_Z_BIAS_M:-0.000}"

# Repeatability: run each case N times (4/5-per-condition acceptance).
SWEEP_REPEATS="${INTAKE_SWEEP_REPEATS:-1}"

# Concept Validation Plan gates (throat|funnel|ramp|full).
INTAKE_PHASE="${INTAKE_SWEEP_PHASE:-full}"
case "$INTAKE_PHASE" in
    throat) ENABLE_FUNNEL=false; ENABLE_RAMP=false ;;
    funnel) ENABLE_FUNNEL=true;  ENABLE_RAMP=false ;;
    ramp)   ENABLE_FUNNEL=false; ENABLE_RAMP=true ;;
    full)   ENABLE_FUNNEL=true;  ENABLE_RAMP=true ;;
    *) echo "ERROR: unknown INTAKE_SWEEP_PHASE=$INTAKE_PHASE" >&2; exit 1 ;;
esac
PROBE_DURATION="${INTAKE_SWEEP_PROBE_DURATION:-25}"
PROBE_PERIOD="${INTAKE_SWEEP_PROBE_PERIOD:-0.2}"
START_DISTANCE_M="${INTAKE_SWEEP_START_DISTANCE_M:-0.45}"
READY_TIMEOUT_S="${INTAKE_SWEEP_READY_TIMEOUT_S:-90}"
BALL_VISIBLE_TIMEOUT_S="${INTAKE_SWEEP_BALL_VISIBLE_TIMEOUT_S:-90}"
APPROACH_TIMEOUT_S="${INTAKE_SWEEP_APPROACH_TIMEOUT_S:-160}"
DRIVER="${INTAKE_SWEEP_DRIVER:-bench}"
BENCH_BALL_X="${INTAKE_BENCH_BALL_X:--6.4}"
BENCH_BALL_Y_BASE="${INTAKE_BENCH_BALL_Y:-0.0}"
BENCH_BALL_Y="$BENCH_BALL_Y_BASE"
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
assist_pub_pid=""
assist_spawner_pid=""
conveyor_pub_pid=""
conveyor_spawner_pid=""
pose_logger_pid=""
probe_pid=""

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
    if [ -n "$assist_pub_pid" ] && kill -0 "$assist_pub_pid" >/dev/null 2>&1; then
        kill -- "-$assist_pub_pid" >/dev/null 2>&1 || kill "$assist_pub_pid" >/dev/null 2>&1 || true
        wait "$assist_pub_pid" >/dev/null 2>&1 || true
    fi
    assist_pub_pid=""
    if [ -n "$assist_spawner_pid" ] && kill -0 "$assist_spawner_pid" >/dev/null 2>&1; then
        kill -- "-$assist_spawner_pid" >/dev/null 2>&1 || kill "$assist_spawner_pid" >/dev/null 2>&1 || true
        wait "$assist_spawner_pid" >/dev/null 2>&1 || true
    fi
    assist_spawner_pid=""
    if [ -n "$conveyor_pub_pid" ] && kill -0 "$conveyor_pub_pid" >/dev/null 2>&1; then
        kill -- "-$conveyor_pub_pid" >/dev/null 2>&1 || kill "$conveyor_pub_pid" >/dev/null 2>&1 || true
        wait "$conveyor_pub_pid" >/dev/null 2>&1 || true
    fi
    conveyor_pub_pid=""
    if [ -n "$conveyor_spawner_pid" ] && kill -0 "$conveyor_spawner_pid" >/dev/null 2>&1; then
        kill -- "-$conveyor_spawner_pid" >/dev/null 2>&1 || kill "$conveyor_spawner_pid" >/dev/null 2>&1 || true
        wait "$conveyor_spawner_pid" >/dev/null 2>&1 || true
    fi
    conveyor_spawner_pid=""
    if [ -n "$pose_logger_pid" ] && kill -0 "$pose_logger_pid" >/dev/null 2>&1; then
        kill -- "-$pose_logger_pid" >/dev/null 2>&1 || kill "$pose_logger_pid" >/dev/null 2>&1 || true
        wait "$pose_logger_pid" >/dev/null 2>&1 || true
    fi
    pose_logger_pid=""
    if [ -n "$probe_pid" ] && kill -0 "$probe_pid" >/dev/null 2>&1; then
        kill -- "-$probe_pid" >/dev/null 2>&1 || kill "$probe_pid" >/dev/null 2>&1 || true
        wait "$probe_pid" >/dev/null 2>&1 || true
    fi
    probe_pid=""
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
    local case_dir
    case_dir="$(dirname "$log_file")"
    local deadline=$((SECONDS + timeout_s))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if [ -r "$log_file" ] \
            && grep -q "Configured and activated diff_drive_controller" "$log_file" \
            && grep -q "Configured and activated intake_wheel_velocity_controller" "$log_file"; then
            if [ "$ENABLE_ASSIST" != "true" ] \
                || grep -q "Configured and activated assist_wheel_velocity_controller" "$log_file" \
                || grep -q "Configured and activated assist_wheel_velocity_controller" "$case_dir/assist_spawner.log"; then
                if [ "$ENABLE_CONVEYOR" != "true" ] \
                    || grep -q "Configured and activated conveyor_velocity_controller" "$log_file" \
                    || grep -q "Configured and activated conveyor_velocity_controller" "$case_dir/conveyor_spawner.log"; then
                    return 0
                fi
            fi
        fi
        if ros2 control list_controllers 2>/dev/null | grep -q "diff_drive_controller.*active" \
            && ros2 control list_controllers 2>/dev/null | grep -q "intake_wheel_velocity_controller.*active"; then
            if [ "$ENABLE_ASSIST" != "true" ] \
                || ros2 control list_controllers 2>/dev/null | grep -q "assist_wheel_velocity_controller.*active"; then
                if [ "$ENABLE_CONVEYOR" != "true" ] \
                    || ros2 control list_controllers 2>/dev/null | grep -q "conveyor_velocity_controller.*active"; then
                    return 0
                fi
            fi
        fi
        sleep 1
    done
    return 1
}

publish_stop_commands() {
    timeout 3 ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /intake_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0]}" >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /assist_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0]}" >/dev/null 2>&1 || true
    timeout 3 ros2 topic pub --once /conveyor_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [0.0, 0.0, 0.0]}" >/dev/null 2>&1 || true
}

wait_for_wheel_speed() {
    # Both intake wheels must reach speed AND counter-rotate (left negative,
    # right positive) — this is also the live verification that the two-motor
    # wiring drives both inner faces rearward.
    local min_abs_speed="$1"
    local timeout_s="$2"
    python3 - "$min_abs_speed" "$timeout_s" <<'PY'
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

target = abs(float(sys.argv[1]))
timeout_s = float(sys.argv[2])
last = {}

rclpy.init()
node = rclpy.create_node("wait_for_intake_wheel_speed")

def on_joint_states(msg):
    names = list(msg.name)
    for joint in ("intake_wheel_left_joint", "intake_wheel_right_joint"):
        try:
            index = names.index(joint)
        except ValueError:
            continue
        if index < len(msg.velocity):
            last[joint] = float(msg.velocity[index])

sub = node.create_subscription(JointState, "/joint_states", on_joint_states, 10)
deadline = time.time() + timeout_s
try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        left = last.get("intake_wheel_left_joint")
        right = last.get("intake_wheel_right_joint")
        if left is not None and right is not None:
            if abs(left) >= target and abs(right) >= target:
                if left < 0.0 < right:
                    print(f"wheels_ready counter-rotating left={left:.3f} right={right:.3f}")
                    raise SystemExit(0)
                print(
                    f"wheels_wrong_direction left={left:.3f} right={right:.3f} "
                    "(expected left<0<right)",
                    file=sys.stderr,
                )
                raise SystemExit(2)
    print(f"wheels_not_ready last={last}", file=sys.stderr)
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

start_probe() {
    local case_dir="$1"
    setsid python3 -m tennis_robot.sim_physics_probe \
        --duration "$PROBE_DURATION" \
        --period "$PROBE_PERIOD" \
        --jsonl "$case_dir/contact_physics.jsonl" \
        > "$case_dir/probe.log" 2>&1 &
    probe_pid="$!"
}

wait_for_probe() {
    if [ -n "$probe_pid" ]; then
        wait "$probe_pid"
        probe_pid=""
    fi
}

summarize_probe() {
    local case_dir="$1"

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
            --nip-x-m "$INTAKE_NIP_X_M" \
            --wheel-radius-m "$INTAKE_WHEEL_RADIUS_M" \
            --wheel-gap-m "$INTAKE_WHEEL_GAP_M" \
            --base-link-height-m "${INTAKE_BENCH_BASE_LINK_Z:-0.045}" \
            --json-out "$case_dir/pose_summary.json" \
            > "$case_dir/pose_summary.pretty.json"
        local force_threshold_args=()
        if [ -n "${INTAKE_BENCH_FORCE_P95_THRESHOLD_N:-}" ]; then
            force_threshold_args=(--force-p95-threshold-n "$INTAKE_BENCH_FORCE_P95_THRESHOLD_N")
        fi
        python3 "$SCRIPT_DIR/scripts/sim_debug/analyze_intake_release_criteria.py" \
            "$case_dir/contact_physics.jsonl" \
            "$case_dir/gz_poses.jsonl" \
            --ball-name "${INTAKE_BENCH_BALL_NAME:-ball_02}" \
            --phase "$INTAKE_PHASE" \
            --nip-x-m "$INTAKE_NIP_X_M" \
            --wheel-radius-m "$INTAKE_WHEEL_RADIUS_M" \
            --wheel-gap-m "$INTAKE_WHEEL_GAP_M" \
            --ramp-crest-z-m "${INTAKE_BENCH_RAMP_CREST_Z_M:-0.077}" \
            --preferred-contact-duration-s "${INTAKE_BENCH_PREFERRED_CONTACT_DURATION_S:-0.50}" \
            --transport-target-m-s "${INTAKE_BENCH_TRANSPORT_TARGET_M_S:-0.40}" \
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
        echo "wheel_speed=$BENCH_WHEEL_SPEED"
        echo "assist_enabled=$ENABLE_ASSIST"
        echo "assist_speed=$ASSIST_SPEED"
        echo "assist_x=$ASSIST_X"
        echo "assist_z=$ASSIST_Z"
        echo "assist_radius=$ASSIST_RADIUS"
        echo "assist_length=$ASSIST_LENGTH"
        echo "conveyor_enabled=$ENABLE_CONVEYOR"
        echo "conveyor_speed=$CONVEYOR_SPEED"
        echo "conveyor_x_bias=$CONVEYOR_X_BIAS"
        echo "conveyor_z_bias=$CONVEYOR_Z_BIAS"
        echo "phase=$INTAKE_PHASE"
        echo "wheel_gap=$INTAKE_WHEEL_GAP_M"
        echo "wheel_radius=$INTAKE_WHEEL_RADIUS_M"
        echo "nip_x=$INTAKE_NIP_X_M"
        echo "wheel_tilt_deg=$INTAKE_WHEEL_TILT_DEG"
        echo "wheel_max_vel=$WHEEL_MAX_VEL"
        echo "wheel_effort=$WHEEL_EFFORT"
        echo "spring_k=$INTAKE_WHEEL_SPRING_K"
        echo "ball_lateral_offset=$BENCH_BALL_LATERAL"
        echo "ramp_entry_x=${INTAKE_RAMP_ENTRY_X_M:-default}"
        echo "ramp_knee_x=${INTAKE_RAMP_KNEE_X_M:-default}"
        echo "ramp_knee_z=${INTAKE_RAMP_KNEE_Z_M:-default}"
        echo "ramp_end_x=${INTAKE_RAMP_END_X_M:-default}"
        echo "ramp_end_z=${INTAKE_RAMP_END_Z_M:-default}"
        echo "basket_floor_front_x=${INTAKE_BASKET_FLOOR_FRONT_X_M:-0.50}"
        echo "basket_floor_top_z=${INTAKE_BASKET_FLOOR_TOP_Z_M:-0.128}"
        echo "settle_s=$BENCH_SETTLE_S"
        echo "roller_lead_s=$BENCH_ROLLER_LEAD_S"
        echo "roller_ready_timeout_s=$BENCH_ROLLER_READY_TIMEOUT_S"
        echo "drive_response_s=$BENCH_DRIVE_RESPONSE_S"
    } > "$case_dir/bench_config.txt"

    setsid ros2 launch tennis_robot sim.launch.py headless:=true > "$case_dir/launch.log" 2>&1 &
    launch_pid="$!"

    if [ "$ENABLE_ASSIST" = "true" ]; then
        setsid ros2 run controller_manager spawner assist_wheel_velocity_controller \
            --controller-manager /controller_manager \
            --controller-manager-timeout 90 \
            > "$case_dir/assist_spawner.log" 2>&1 &
        assist_spawner_pid="$!"
    fi
    if [ "$ENABLE_CONVEYOR" = "true" ]; then
        setsid ros2 run controller_manager spawner conveyor_velocity_controller \
            --controller-manager /controller_manager \
            --controller-manager-timeout 90 \
            > "$case_dir/conveyor_spawner.log" 2>&1 &
        conveyor_spawner_pid="$!"
    fi

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

    # Dual-wheel intake: [left, right] = [-v, +v] so both inner faces drive
    # rearward (left CW / right CCW seen from above).
    setsid ros2 topic pub --rate 20 /intake_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{data: [-$BENCH_WHEEL_SPEED, $BENCH_WHEEL_SPEED]}" \
        > "$case_dir/roller_pub.log" 2>&1 &
    roller_pub_pid="$!"
    if [ "$ENABLE_ASSIST" = "true" ]; then
        setsid ros2 topic pub --rate 20 /assist_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray \
            "{data: [$ASSIST_SPEED]}" \
            > "$case_dir/assist_pub.log" 2>&1 &
        assist_pub_pid="$!"
    fi
    if [ "$ENABLE_CONVEYOR" = "true" ]; then
        setsid ros2 topic pub --rate 20 /conveyor_velocity_controller/commands std_msgs/msg/Float64MultiArray \
            "{data: [$CONVEYOR_SPEED, $CONVEYOR_SPEED, $CONVEYOR_SPEED]}" \
            > "$case_dir/conveyor_pub.log" 2>&1 &
        conveyor_pub_pid="$!"
    fi
    sleep "$BENCH_ROLLER_LEAD_S"
    if ! wait_for_wheel_speed 1.0 "$BENCH_ROLLER_READY_TIMEOUT_S" > "$case_dir/wheels_ready.log" 2>&1; then
        echo "FAILED: intake wheels not counter-rotating before drive; see $case_dir/wheels_ready.log" >&2
        cleanup_launch
        return 1
    fi

    start_probe "$case_dir"
    sleep 0.5

    setsid ros2 topic pub --rate 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: auto, twist: {linear: {x: $BENCH_DRIVE_SPEED}, angular: {z: 0.0}}}" \
        > "$case_dir/drive_pub.log" 2>&1 &
    drive_pub_pid="$!"

    sleep 1
    ros2 topic info /diff_drive_controller/cmd_vel_unstamped \
        > "$case_dir/cmd_vel_unstamped_after_pub.info" 2>&1 || true
    ros2 topic info /diff_drive_controller/cmd_vel \
        > "$case_dir/cmd_vel_stamped_after_pub.info" 2>&1 || true
    log_drive_response "$case_dir/drive_response.json" "$BENCH_DRIVE_RESPONSE_S" > "$case_dir/drive_response.log" 2>&1 || true

    wait_for_probe
    summarize_probe "$case_dir"
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

    setsid python3 "$SCRIPT_DIR/scripts/sim_debug/log_gz_poses.py" "$case_dir/gz_poses.jsonl" \
        > "$case_dir/gz_poses.log" 2>&1 &
    pose_logger_pid="$!"

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

    start_probe "$case_dir"
    wait_for_probe
    summarize_probe "$case_dir"
    write_command idle > "$case_dir/final_idle_command.txt" || true
    cleanup_launch
}

run_case() {
    local wheel_gap="$1"
    local wheel_radius="$2"
    local nip_x="$3"
    local wheel_tilt="$4"
    local spring_k="$5"
    local drive_speed="$6"
    local wheel_speed="$7"
    local ball_lateral="$8"
    local repeat="$9"
    local case_name="gap_${wheel_gap}_rw_${wheel_radius}_nipx_${nip_x}_tilt_${wheel_tilt}_k_${spring_k}_drive_${drive_speed}_wspeed_${wheel_speed}_assist_${ENABLE_ASSIST}_${ASSIST_SPEED}_ax_${ASSIST_X}_az_${ASSIST_Z}_conv_${ENABLE_CONVEYOR}_${CONVEYOR_SPEED}_cx_${CONVEYOR_X_BIAS}_cz_${CONVEYOR_Z_BIAS}_lat_${ball_lateral}_r${repeat}"
    case_name="${case_name//- /}"
    case_name="${case_name//./p}"
    case_name="${case_name//-/m}"
    local case_dir="$OUT_ROOT/$case_name"
    mkdir -p "$case_dir"

    export INTAKE_WHEEL_GAP_M="$wheel_gap"
    export INTAKE_WHEEL_RADIUS_M="$wheel_radius"
    export INTAKE_NIP_X_M="$nip_x"
    export INTAKE_WHEEL_TILT_DEG="$wheel_tilt"
    export INTAKE_WHEEL_MAX_VEL_RAD_S="$WHEEL_MAX_VEL"
    export INTAKE_WHEEL_EFFORT_NM="$WHEEL_EFFORT"
    export INTAKE_WHEEL_SPRING_K="$spring_k"
    export INTAKE_ENABLE_FUNNEL="$ENABLE_FUNNEL"
    export INTAKE_ENABLE_RAMP="$ENABLE_RAMP"
    export INTAKE_ENABLE_ASSIST="$ENABLE_ASSIST"
    export INTAKE_ENABLE_CONVEYOR="$ENABLE_CONVEYOR"
    export INTAKE_CONVEYOR_X_BIAS_M="$CONVEYOR_X_BIAS"
    export INTAKE_CONVEYOR_Z_BIAS_M="$CONVEYOR_Z_BIAS"
    export BENCH_DRIVE_SPEED="$drive_speed"
    export BENCH_WHEEL_SPEED="$wheel_speed"
    export BENCH_BALL_LATERAL="$ball_lateral"
    BENCH_BALL_Y="$(python3 -c "print(float('$BENCH_BALL_Y_BASE') + float('$ball_lateral'))")"

    echo
    echo "=== $case_name ==="
    echo "phase=$INTAKE_PHASE gap=$wheel_gap rw=$wheel_radius nip_x=$nip_x tilt=$wheel_tilt k=$spring_k drive=$drive_speed wheel_speed=$wheel_speed lateral=$ball_lateral"

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

for wheel_gap in "${WHEEL_GAPS[@]}"; do
    for wheel_radius in "${WHEEL_RADII[@]}"; do
        for nip_x in "${NIP_XS[@]}"; do
            for wheel_tilt in "${WHEEL_TILTS_DEG[@]}"; do
                for spring_k in "${SPRING_KS[@]}"; do
                    for drive_speed in "${BENCH_DRIVE_SPEEDS[@]}"; do
                        for wheel_speed in "${BENCH_WHEEL_SPEEDS[@]}"; do
                            for ball_lateral in "${BALL_LATERAL_OFFSETS[@]}"; do
                                for repeat in $(seq 1 "$SWEEP_REPEATS"); do
                                    run_case "$wheel_gap" "$wheel_radius" "$nip_x" "$wheel_tilt" "$spring_k" "$drive_speed" "$wheel_speed" "$ball_lateral" "$repeat"
                                done
                            done
                        done
                    done
                done
            done
        done
    done
done

echo
echo "Sweep complete: $SUMMARY_CSV"
