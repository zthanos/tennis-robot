#!/usr/bin/env bash
# WS2 — Raspberry Pi bring-up (Ubuntu 24.04 aarch64 / ROS 2 Jazzy).
#
# The Pi runs the CONTROL side only (controller_node + perception + Nav2 +
# slam_toolbox + web console). The Gazebo simulation stays on the PC and reaches
# the Pi over the network (WS3), so this installs NO Gazebo / ros_gz / ros2_control
# packages — only what the Pi-side nodes and the C++ CollectionFollowPath plugin
# need to build and run.
#
# Idempotent: safe to re-run. Steps already satisfied are skipped.
#
#   ./scripts/setup_pi.sh            # full: apt deps + pip deps + colcon build
#   INSTALL_ROS=true ./scripts/setup_pi.sh   # also add the ROS 2 apt repo + ros-base
#   BUILD_ONLY=true ./scripts/setup_pi.sh    # skip apt/pip, just colcon build
#
# NOTE: no `set -u` — ROS setup.bash references unbound env vars.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SCRIPT_DIR"
WS="ros2_ws"

# System Python must own the ROS/build toolchain; a uv-managed python3.12 under
# ~/.local/bin shadows it and lacks empy/lark/rclpy (same trap as run_native.sh).
export PATH="/usr/bin:$PATH"

echo "[setup_pi] arch=$(uname -m)  $(. /etc/os-release 2>/dev/null; echo "$PRETTY_NAME")"
[ "$(uname -m)" = "aarch64" ] || echo "[setup_pi] WARNING: not aarch64 — this script targets the Pi."

# ── 0. (optional) ROS 2 Jazzy apt repo + ros-base ────────────────────────────
if [ "${INSTALL_ROS:-false}" = "true" ] && [ ! -d /opt/ros/jazzy ]; then
    echo "[setup_pi] adding ROS 2 Jazzy apt repo + ros-base…"
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-ros-base
fi

if [ ! -d /opt/ros/jazzy ]; then
    echo "[setup_pi] ERROR: /opt/ros/jazzy not found. Re-run with INSTALL_ROS=true." >&2
    exit 1
fi

if [ "${BUILD_ONLY:-false}" != "true" ]; then
    # ── 1. Pi-side ROS packages (NO gazebo / ros_gz / ros2_control) ──────────
    echo "[setup_pi] apt: Pi control-side ROS packages…"
    sudo apt-get install -y \
        ros-dev-tools python3-colcon-common-extensions python3-rosdep \
        build-essential cmake libssl-dev \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-twist-mux \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
        ros-jazzy-xacro \
        ros-jazzy-rmw-fastrtps-cpp

    # ── 2. rosdep for anything the manifests still need (skip the sim keys) ──
    echo "[setup_pi] rosdep…"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    # ros_gz_* / gz_ros2_control / the ros2_control spawners run in Gazebo on the
    # PC — never on the Pi — so they are intentionally skipped here.
    rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy \
        --skip-keys "ros_gz_sim ros_gz_interfaces gz_ros2_control controller_manager \
diff_drive_controller joint_state_broadcaster forward_command_controller \
ros2_controllers teleop_twist_keyboard explore_lite" || \
        echo "[setup_pi] WARNING: rosdep reported unresolved keys (expected for skipped sim deps)."

    # ── 3. Python runtime deps (match pyproject.toml; system Python, PEP 668) ─
    echo "[setup_pi] pip: numpy opencv-python-headless duckdb onnxruntime…"
    # onnxruntime is the ball detector's neural backend (Risk #1 on ARM64). The
    # aarch64 wheel exists on PyPI for onnxruntime>=1.20; if pip cannot find it,
    # see docs/process/pi-setup-el.md for the piwheels / source-build fallback.
    python3 -m pip install --user --break-system-packages \
        "numpy>=1.26" "opencv-python-headless>=4.9" "duckdb>=1.5.3" "onnxruntime>=1.20"
fi

# ── 4. Build the three workspace packages (control side only) ────────────────
echo "[setup_pi] colcon build (tennis_robot_msgs, _collection_controller, tennis_robot)…"
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon --log-base log_jazzy build \
    --build-base build_jazzy --install-base install_jazzy \
    --packages-select tennis_robot_msgs tennis_robot_collection_controller tennis_robot )

echo
echo "[setup_pi] ✅ done. Verify:"
echo "    source /opt/ros/jazzy/setup.bash && source $WS/install_jazzy/setup.bash"
echo "    python3 -c 'import onnxruntime, duckdb, cv2; print(\"py deps OK\")'"
echo "    ros2 pkg prefix tennis_robot_collection_controller   # plugin package present"
echo
echo "  Next (WS3): set a shared ROS_DOMAIN_ID on PC + Pi, start the PC sim, and"
echo "  bring up the Pi control nodes with use_sim_time:=true. See"
echo "  docs/process/pi-setup-el.md."
