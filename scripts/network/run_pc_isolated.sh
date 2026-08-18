#!/usr/bin/env bash
# Run the PC simulation behind a local ROS domain and a minimal LAN endpoint.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PC_LOCAL_DOMAIN_ID="${PC_LOCAL_DOMAIN_ID:-41}"
LAN_ROS_DOMAIN_ID="${LAN_ROS_DOMAIN_ID:-42}"
BRIDGE_CONFIG="${TENNIS_PC_DOMAIN_BRIDGE_CONFIG:-$ROOT_DIR/config/network/pc41_lan42_domain_bridge.yaml}"

if [ "$PC_LOCAL_DOMAIN_ID" != "41" ] || [ "$LAN_ROS_DOMAIN_ID" != "42" ]; then
    echo "ERROR: checked-in PC bridge config is pinned to local=41 and LAN=42"
    exit 1
fi
if [ ! -r "$BRIDGE_CONFIG" ]; then
    echo "ERROR: PC domain bridge config not readable: $BRIDGE_CONFIG"
    exit 1
fi

required_udp_rmem=4194304
actual_udp_rmem="$(sysctl -n net.core.rmem_default)"
if [ "$actual_udp_rmem" -lt "$required_udp_rmem" ]; then
    echo "ERROR: isolated PC stack requires net.core.rmem_default >= $required_udp_rmem"
    echo "Install it with: sudo ./scripts/network/install_udp_buffer_profile.sh"
    exit 1
fi

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
export ROS2CLI_DISABLE_DAEMON=1
if [ ! -f "$ROOT_DIR/ros2_ws/install_jazzy/setup.bash" ]; then
    echo "ERROR: PC workspace is not built: $ROOT_DIR/ros2_ws/install_jazzy/setup.bash"
    exit 1
fi
# shellcheck disable=SC1091
. "$ROOT_DIR/ros2_ws/install_jazzy/setup.bash"
if ! ros2 pkg prefix domain_bridge >/dev/null 2>&1; then
    echo "ERROR: ros-jazzy-domain-bridge is not installed on the PC"
    echo "Install it with: sudo apt-get install ros-jazzy-domain-bridge"
    exit 1
fi

RUN_LOCK_FILE="/tmp/tennis_robot_pc_isolated_${PC_LOCAL_DOMAIN_ID}_${LAN_ROS_DOMAIN_ID}.lock"
exec 8>"$RUN_LOCK_FILE"
if ! flock -n 8; then
    echo "ERROR: another isolated PC stack is already active"
    exit 1
fi

pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${pids[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM HUP

echo "[run_pc_isolated] local domain=$PC_LOCAL_DOMAIN_ID -> LAN domain=$LAN_ROS_DOMAIN_ID"
echo "[run_pc_isolated] Gazebo Transport loopback only (GZ_IP=127.0.0.1)"
setsid ros2 run domain_bridge domain_bridge \
    --wait-for-publisher false \
    --wait-for-subscription false \
    "$BRIDGE_CONFIG" &
pids+=("$!")

sleep "${DOMAIN_BRIDGE_START_DELAY_S:-2}"
setsid env \
    GZ_IP=127.0.0.1 \
    TENNIS_LAUNCH_BRAIN=false \
    ROS_DOMAIN_ID="$PC_LOCAL_DOMAIN_ID" \
    "$ROOT_DIR/run_native.sh" &
sim_pid="$!"
pids+=("$sim_pid")
wait "$sim_pid"
