#!/usr/bin/env bash
# Run the Pi brain behind an explicit ROS-domain boundary.
#
# PC: domain 42, simulation/perception only.
# Pi: domain 43, brain/Nav2/SLAM/UI.
# This process runs one allowlisted domain bridge between them, preventing every
# Pi subscriber from becoming a separate remote Fast DDS reader on the LAN.
# NOTE: no `set -u` because ROS setup files reference optional variables.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PC_ROS_DOMAIN_ID="${PC_ROS_DOMAIN_ID:-42}"
PI_ROS_DOMAIN_ID="${PI_ROS_DOMAIN_ID:-43}"
BRIDGE_CONFIG="${TENNIS_DOMAIN_BRIDGE_CONFIG:-$ROOT_DIR/config/network/pc42_pi43_domain_bridge.yaml}"

if [ "$PC_ROS_DOMAIN_ID" = "$PI_ROS_DOMAIN_ID" ]; then
    echo "ERROR: PC_ROS_DOMAIN_ID and PI_ROS_DOMAIN_ID must differ"
    exit 1
fi
if [ "$PC_ROS_DOMAIN_ID" != "42" ] || [ "$PI_ROS_DOMAIN_ID" != "43" ]; then
    echo "ERROR: the checked-in bridge config is pinned to PC=42 and Pi=43"
    echo "Use the default IDs or provide a separately reviewed config."
    exit 1
fi
if [ ! -r "$BRIDGE_CONFIG" ]; then
    echo "ERROR: domain bridge config not readable: $BRIDGE_CONFIG"
    exit 1
fi

required_udp_rmem=4194304
actual_udp_rmem="$(sysctl -n net.core.rmem_default)"
if [ "$actual_udp_rmem" -lt "$required_udp_rmem" ]; then
    echo "ERROR: isolated Pi stack requires net.core.rmem_default >= $required_udp_rmem"
    echo "Install it with: sudo ./scripts/network/install_udp_buffer_profile.sh"
    exit 1
fi

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# The CLI daemon is not part of the runtime data path. Leaving one daemon per
# domain alive adds hidden Fast DDS participants with stale, small UDP sockets.
export ROS2CLI_DISABLE_DAEMON=1
if [ ! -f "$ROOT_DIR/ros2_ws/install_jazzy/setup.bash" ]; then
    echo "ERROR: Pi workspace is not built: $ROOT_DIR/ros2_ws/install_jazzy/setup.bash"
    echo "Run BUILD=true ./run_pi.sh once before using the isolated profile."
    exit 1
fi
# Custom tennis_robot_msgs types are required by the bridge.
# shellcheck disable=SC1091
. "$ROOT_DIR/ros2_ws/install_jazzy/setup.bash"
if ! ros2 pkg prefix domain_bridge >/dev/null 2>&1; then
    echo "ERROR: ros-jazzy-domain-bridge is not installed"
    echo "Install it with: sudo apt-get install ros-jazzy-domain-bridge"
    exit 1
fi

RUN_LOCK_FILE="/tmp/tennis_robot_pi_isolated_${PC_ROS_DOMAIN_ID}_${PI_ROS_DOMAIN_ID}.lock"
exec 8>"$RUN_LOCK_FILE"
if ! flock -n 8; then
    echo "ERROR: another isolated Pi stack is already active"
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

echo "[run_pi_isolated] PC domain=$PC_ROS_DOMAIN_ID -> Pi domain=$PI_ROS_DOMAIN_ID"
echo "[run_pi_isolated] allowlist=$BRIDGE_CONFIG"
setsid ros2 run domain_bridge domain_bridge \
    --wait-for-publisher false \
    --wait-for-subscription false \
    "$BRIDGE_CONFIG" &
pids+=("$!")

# Give both bridge contexts time to join before the local brain starts.
sleep "${DOMAIN_BRIDGE_START_DELAY_S:-2}"
setsid env ROS_DOMAIN_ID="$PI_ROS_DOMAIN_ID" "$ROOT_DIR/run_pi.sh" &
brain_pid="$!"
pids+=("$brain_pid")
wait "$brain_pid"
