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
if [ -r "$SCRIPT_DIR/ros2_ws/install_jazzy/setup.bash" ]; then
    ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install_jazzy"
else
    ROS2_INSTALL="$SCRIPT_DIR/ros2_ws/install"
fi
WORKSPACE_SETUP="$ROS2_INSTALL/setup.bash"

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
export ROS2_INSTALL
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
PROBE_DURATION="${INTAKE_SWEEP_PROBE_DURATION:-35}"
PROBE_PERIOD="${INTAKE_SWEEP_PROBE_PERIOD:-0.2}"
BASKET_LOAD_COUNT="${INTAKE_BASKET_LOAD_COUNT:-0}"
BASKET_LOAD_SETTLE_S="${INTAKE_BASKET_LOAD_SETTLE_S:-3}"
RAMP_PROFILE="${INTAKE_RAMP_PROFILE:-launch}"
if [ -n "${INTAKE_SWEEP_START_DISTANCE_M:-}" ]; then
    START_DISTANCE_M="$INTAKE_SWEEP_START_DISTANCE_M"
elif [ "$RAMP_PROFILE" = "launch" ]; then
    # The launch handoff can complete between two perception samples once the
    # ball is inside 0.57 m. Start evidence capture before that transition.
    START_DISTANCE_M="0.70"
else
    START_DISTANCE_M="0.45"
fi
READY_TIMEOUT_S="${INTAKE_SWEEP_READY_TIMEOUT_S:-90}"
CLOCK_TIMEOUT_S="${INTAKE_SWEEP_CLOCK_TIMEOUT_S:-15}"
STARTUP_RETRIES="${INTAKE_SWEEP_STARTUP_RETRIES:-1}"
STARTUP_COOLDOWN_S="${INTAKE_SWEEP_STARTUP_COOLDOWN_S:-3}"
BALL_VISIBLE_TIMEOUT_S="${INTAKE_SWEEP_BALL_VISIBLE_TIMEOUT_S:-90}"
APPROACH_TIMEOUT_S="${INTAKE_SWEEP_APPROACH_TIMEOUT_S:-160}"
DRIVER="${INTAKE_SWEEP_DRIVER:-bench}"
COLLECT_ONE_TARGET_X="${INTAKE_COLLECT_ONE_TARGET_X:--6.4}"
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
    local had_launch=false
    if [ -n "$launch_pid" ] && kill -0 "$launch_pid" >/dev/null 2>&1; then
        had_launch=true
        kill -- "-$launch_pid" >/dev/null 2>&1 || kill "$launch_pid" >/dev/null 2>&1 || true
        wait "$launch_pid" >/dev/null 2>&1 || true
    fi
    launch_pid=""
    if [ "$had_launch" = "true" ]; then
        sleep "$STARTUP_COOLDOWN_S"
    fi
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
        sleep 1
    done
    return 1
}

verify_bench_ready() {
    # A bench is not ready because its processes started. This asserts that the
    # simulation clock advances, that controller_manager is actually on it, and
    # that a commanded joint's MEASURED state moves — the three properties that
    # "Configured and activated ..." in the launch log does not imply.
    local log_file="$1"
    local budget="$CLOCK_TIMEOUT_S"
    timeout "$(( budget * 3 + 30 ))" python3 \
        "$SCRIPT_DIR/scripts/sim_debug/verify_sim_bench.py" --timeout-s "$budget" \
        > "$log_file" 2>&1
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
    local case_dir="${1:-}"
    if [ -n "$probe_pid" ]; then
        local status=0
        wait "$probe_pid" || status="$?"
        probe_pid=""
        if [ "$status" -ne 0 ]; then
            if [ -n "$case_dir" ] \
                && grep -q '"type": "summary"' "$case_dir/contact_physics.jsonl" 2>/dev/null; then
                echo "probe exited status=$status after writing final summary; accepting completed evidence" \
                    >> "$case_dir/probe.log"
            else
                echo "FAILED: probe exited status=$status without final summary" >&2
                return "$status"
            fi
        fi
    fi
}

set_gz_model_pose() {
    local name="$1"
    local x="$2"
    local y="$3"
    local z="$4"
    gz service -s "/world/${GZ_WORLD_NAME:-tennis_court}/set_pose" \
        --reqtype gz.msgs.Pose \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "name: '$name' position: {x: $x, y: $y, z: $z} orientation: {w: 1.0}"
}

spawn_stored_ball() {
    local name="$1"
    local x="$2"
    local y="$3"
    local z="$4"
    local response sdf
    sdf="<sdf version='1.9'><model name='$name'><link name='ball'><inertial><mass>0.058</mass><inertia><ixx>2.5e-5</ixx><iyy>2.5e-5</iyy><izz>2.5e-5</izz></inertia></inertial><collision name='col'><geometry><sphere><radius>0.033</radius></sphere></geometry><surface><friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction></surface></collision><visual name='vis'><geometry><sphere><radius>0.033</radius></sphere></geometry><material><ambient>0.8 0.9 0.1 1</ambient><diffuse>0.8 0.9 0.1 1</diffuse></material></visual></link></model></sdf>"
    if ! response="$(gz service -s "/world/${GZ_WORLD_NAME:-tennis_court}/create" \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf: \"$sdf\" name: \"$name\" pose: {position: {x: $x, y: $y, z: $z} orientation: {w: 1.0}}")"; then
        return 1
    fi
    printf '%s\n' "$response"
    printf '%s\n' "$response" | grep -Eq 'data: (true|1)'
}

prepare_basket_load() {
    local log_file="$1"
    local count="$2"
    : > "$log_file"
    if ! [[ "$count" =~ ^[0-9]+$ ]] || [ "$count" -gt 45 ]; then
        echo "FAILED: INTAKE_BASKET_LOAD_COUNT must be an integer from 0 to 45" >> "$log_file"
        return 1
    fi
    [ "$count" -eq 0 ] && return 0

    # The basket interior is local x=0.02..0.42, y=+-0.14, z=0.045..0.25.
    # Three 5x3 layers represent up to 45 balls without initial overlap.
    local robot_x="${SIM_ROBOT_SPAWN_X:--8.0}"
    local robot_y="${SIM_ROBOT_SPAWN_Y:-0.0}"
    local robot_yaw="${SIM_ROBOT_SPAWN_YAW:-0.0}"
    local floor_front_x="${INTAKE_BASKET_FLOOR_FRONT_X_M:-0.42}"
    local floor_top_z="${INTAKE_BASKET_FLOOR_TOP_Z_M:-0.025}"
    local management_run="${BASKET_MANAGEMENT_RUN_M:-0.14}"
    local management_rise="${BASKET_MANAGEMENT_RISE_M:-0.010}"
    local index layer layer_index row column name local_x local_y local_z world_x world_y
    for index in $(seq 0 $((count - 1))); do
        layer=$((index / 15))
        layer_index=$((index % 15))
        row=$((layer_index / 5))
        column=$((layer_index % 5))
        name="$(printf 'stored_ball_%02d' "$index")"
        local_x="$(python3 -c "print(0.07 + int('$column') * 0.07)")"
        local_y="$(python3 -c "print(-0.075 + int('$row') * 0.075)")"
        local_z="$(python3 -c "x=float('$local_x'); front=float('$floor_front_x'); run=float('$management_run'); rise=float('$management_rise'); floor=float('$floor_top_z') + rise * max(0.0, min(1.0, (x - (front - run)) / run)); print(floor + 0.033 + int('$layer') * 0.066)")"
        world_x="$(python3 -c "import math; print(float('$robot_x') + math.cos(float('$robot_yaw')) * float('$local_x') - math.sin(float('$robot_yaw')) * float('$local_y'))")"
        world_y="$(python3 -c "import math; print(float('$robot_y') + math.sin(float('$robot_yaw')) * float('$local_x') + math.cos(float('$robot_yaw')) * float('$local_y'))")"
        if ! spawn_stored_ball "$name" "$world_x" "$world_y" "$local_z" \
            >> "$log_file" 2>&1; then
            echo "FAILED: could not spawn $name" >> "$log_file"
            return 1
        fi
        echo "spawned=$name local_x=$local_x local_y=$local_y local_z=$local_z world_x=$world_x world_y=$world_y" \
            >> "$log_file"
    done
}

wait_for_stored_balls() {
    local pose_file="$1"
    local expected="$2"
    local timeout_s="$3"
    python3 - "$pose_file" "$expected" "$timeout_s" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
expected = int(sys.argv[2])
deadline = time.monotonic() + float(sys.argv[3])
expected_names = {f"stored_ball_{index:02d}" for index in range(expected)}
while time.monotonic() < deadline:
    observed = set()
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            observed.update(
                pose.get("n", "")
                for pose in row.get("poses", [])
                if pose.get("n", "").startswith("stored_ball_")
            )
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    if observed == expected_names:
        print(f"stored_balls_ready count={len(observed)}")
        raise SystemExit(0)
    time.sleep(0.2)
print(
    f"stored_balls_not_ready expected={sorted(expected_names)} observed={sorted(observed)}",
    file=sys.stderr,
)
raise SystemExit(1)
PY
}

prepare_collect_one_target() {
    local log_file="$1"
    local target_y="$2"
    : > "$log_file"
    local index name far_x far_y
    for index in $(seq 0 17); do
        [ "$index" -eq 2 ] && continue
        name="$(printf 'ball_%02d' "$index")"
        far_x="$(python3 -c "print(20.0 + int('$index'))")"
        far_y="$(python3 -c "print(12.0 + (int('$index') % 3))")"
        if ! set_gz_model_pose "$name" "$far_x" "$far_y" 0.033 >> "$log_file" 2>&1; then
            echo "FAILED: could not isolate $name" >> "$log_file"
            return 1
        fi
    done
    if ! set_gz_model_pose ball_02 "$COLLECT_ONE_TARGET_X" "$target_y" 0.033 \
        >> "$log_file" 2>&1; then
        echo "FAILED: could not position ball_02" >> "$log_file"
        return 1
    fi
    echo "target=ball_02 x=$COLLECT_ONE_TARGET_X y=$target_y z=0.033" >> "$log_file"
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
        # This evidence gate is intentionally non-fatal here: current runs that
        # remove the target at the entry checkpoint should produce an honest
        # basket_evidence FAIL while preserving the transport diagnostics.
        python3 "$SCRIPT_DIR/scripts/sim_debug/analyze_basket_evidence.py" \
            "$case_dir/gz_poses.jsonl" \
            --target-name "${INTAKE_BENCH_BALL_NAME:-ball_02}" \
            --expected-stored-count "$BASKET_LOAD_COUNT" \
            --json-out "$case_dir/basket_evidence.json" \
            > "$case_dir/basket_evidence.pretty.json" || true
        # Ballistic fit is diagnostic while the mechanism is being tuned: a
        # failing trajectory must remain visible alongside transport evidence.
        python3 "$SCRIPT_DIR/scripts/sim_debug/analyze_launch_ballistics.py" \
            "$case_dir/gz_poses.jsonl" \
            --ball-name "${INTAKE_BENCH_BALL_NAME:-ball_02}" \
            --json-out "$case_dir/launch_ballistics.json" \
            > "$case_dir/launch_ballistics.pretty.json" || true
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
        echo "ramp_profile=${INTAKE_RAMP_PROFILE:-launch}"
        echo "probe_start_distance_m=$START_DISTANCE_M"
        echo "ramp_knee_x=${INTAKE_RAMP_KNEE_X_M:-default}"
        echo "ramp_knee_z=${INTAKE_RAMP_KNEE_Z_M:-default}"
        echo "ramp_end_x=${INTAKE_RAMP_END_X_M:-default}"
        echo "ramp_end_z=${INTAKE_RAMP_END_Z_M:-default}"
        echo "launch_exit_x=${INTAKE_LAUNCH_EXIT_X_M:-0.465}"
        echo "launch_exit_z=${INTAKE_LAUNCH_EXIT_Z_M:-0.032}"
        echo "launch_exit_angle_deg=${INTAKE_LAUNCH_EXIT_ANGLE_DEG:-35.0}"
        echo "basket_floor_front_x=${INTAKE_BASKET_FLOOR_FRONT_X_M:-0.42}"
        echo "basket_floor_top_z=${INTAKE_BASKET_FLOOR_TOP_Z_M:-0.025}"
        echo "basket_management_run=${BASKET_MANAGEMENT_RUN_M:-0.14}"
        echo "basket_management_rise=${BASKET_MANAGEMENT_RISE_M:-0.010}"
        echo "basket_receiver_run=${BASKET_RECEIVER_RUN_M:-0.050}"
        echo "basket_receiver_rise=${BASKET_RECEIVER_RISE_M:-0.005}"
        echo "basket_hood_rear_overhang=${BASKET_HOOD_REAR_OVERHANG_M:-0.040}"
        echo "basket_hood_rear_clearance_z=${BASKET_HOOD_REAR_CLEARANCE_Z_M:-0.120}"
        echo "basket_hood_front_clearance_z=${BASKET_HOOD_FRONT_CLEARANCE_Z_M:-0.135}"
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
    if ! verify_bench_ready "$case_dir/bench_ready.log"; then
        echo "FAILED: simulation bench is not usable; see $case_dir/bench_ready.log" >&2
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

    if ! prepare_basket_load "$case_dir/basket_load_setup.log" "$BASKET_LOAD_COUNT"; then
        echo "FAILED: basket preload setup; see $case_dir/basket_load_setup.log" >&2
        cleanup_launch
        return 1
    fi
    if [ "$BASKET_LOAD_COUNT" -gt 0 ]; then
        sleep "$BASKET_LOAD_SETTLE_S"
        if ! wait_for_stored_balls "$case_dir/gz_poses.jsonl" "$BASKET_LOAD_COUNT" 10 \
            > "$case_dir/basket_load_ready.log" 2>&1; then
            echo "FAILED: basket preload not visible in Gazebo; see $case_dir/basket_load_ready.log" >&2
            cleanup_launch
            return 1
        fi
    fi

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

    wait_for_probe "$case_dir"
    summarize_probe "$case_dir"
    cleanup_publishers
    publish_stop_commands
    cleanup_launch
}

run_collect_one_driver() {
    local case_dir="$1"

    unset SIM_ROBOT_SPAWN_X SIM_ROBOT_SPAWN_Y SIM_ROBOT_SPAWN_Z SIM_ROBOT_SPAWN_YAW
    unset SIM_BENCH_MINIMAL
    export SIM_SKIP_CONTROL_PANEL=true
    # The file-backed command survives process restarts. Reset it before launch
    # so a previous collect_one cannot start against half-spawned controllers.
    write_command idle > "$case_dir/prelaunch_idle_command.txt"
    local start_time attempt startup_ready=false startup_failure=""
    for attempt in $(seq 0 "$STARTUP_RETRIES"); do
        start_time="$(python3 -c 'import time; print(time.time())')"
        setsid ros2 launch tennis_robot sim.launch.py headless:=true > "$case_dir/launch.log" 2>&1 &
        launch_pid="$!"

        if ! wait_for_controllers "$case_dir/launch.log" "$READY_TIMEOUT_S" \
            > "$case_dir/controllers_ready.log" 2>&1; then
            startup_failure="controllers did not become active"
        elif ! verify_bench_ready "$case_dir/clock_ready.log"; then
            startup_failure="simulation bench is not usable (clock/controller/joint)"
        elif ! wait_for_status "$start_time" "$READY_TIMEOUT_S" \
            > "$case_dir/ready.json"; then
            startup_failure="status did not become fresh"
        else
            startup_ready=true
            break
        fi

        cleanup_launch
        if [ "$attempt" -lt "$STARTUP_RETRIES" ]; then
            local attempt_number=$((attempt + 1))
            mv "$case_dir/launch.log" "$case_dir/launch_startup_attempt_${attempt_number}.log"
            [ ! -e "$case_dir/controllers_ready.log" ] || mv \
                "$case_dir/controllers_ready.log" \
                "$case_dir/controllers_ready_attempt_${attempt_number}.log"
            [ ! -e "$case_dir/clock_ready.log" ] || mv \
                "$case_dir/clock_ready.log" \
                "$case_dir/clock_ready_attempt_${attempt_number}.log"
            [ ! -e "$case_dir/ready.json" ] || mv \
                "$case_dir/ready.json" "$case_dir/ready_attempt_${attempt_number}.json"
            echo "RETRY: $startup_failure (attempt $attempt_number)" >&2
        fi
    done
    if [ "$startup_ready" != "true" ]; then
        echo "FAILED: $startup_failure after $((STARTUP_RETRIES + 1)) startup attempt(s); see $case_dir/launch.log" >&2
        cleanup_launch
        return 1
    fi

    setsid python3 "$SCRIPT_DIR/scripts/sim_debug/log_gz_poses.py" "$case_dir/gz_poses.jsonl" \
        > "$case_dir/gz_poses.log" 2>&1 &
    pose_logger_pid="$!"

    if ! prepare_basket_load "$case_dir/basket_load_setup.log" "$BASKET_LOAD_COUNT"; then
        echo "FAILED: basket preload setup; see $case_dir/basket_load_setup.log" >&2
        cleanup_launch
        return 1
    fi
    if [ "$BASKET_LOAD_COUNT" -gt 0 ]; then
        sleep "$BASKET_LOAD_SETTLE_S"
        if ! wait_for_stored_balls "$case_dir/gz_poses.jsonl" "$BASKET_LOAD_COUNT" 10 \
            > "$case_dir/basket_load_ready.log" 2>&1; then
            echo "FAILED: basket preload not visible in Gazebo; see $case_dir/basket_load_ready.log" >&2
            cleanup_launch
            return 1
        fi
    fi

    if ! prepare_collect_one_target "$case_dir/collect_one_target_setup.log" "$BENCH_BALL_Y"; then
        echo "FAILED: deterministic target setup; see $case_dir/collect_one_target_setup.log" >&2
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

    start_probe "$case_dir"
    wait_for_probe "$case_dir"
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
    # The bench driver publishes BENCH_WHEEL_SPEED directly, while collect_one
    # obtains its command from ConceptACollectorConfig. Keep one sweep axis
    # authoritative for both paths.
    export COLLECTOR_INTAKE_WHEEL_SPEED="$wheel_speed"
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
