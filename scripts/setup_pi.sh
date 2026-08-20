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

# Headless Pi over SSH: apt must never open an interactive prompt (needrestart,
# service-restart, config diffs) or the run hangs forever.
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

# Ubuntu's unattended-upgrades runs on boot and holds the dpkg lock; a big apt
# run started while it is active dies with "Could not get lock". Stop the
# periodic units and wait for any in-flight dpkg to finish before we install.
_apt_prepare() {
    sudo systemctl stop unattended-upgrades apt-daily.service apt-daily.timer \
        apt-daily-upgrade.service apt-daily-upgrade.timer 2>/dev/null || true
    for _ in $(seq 1 120); do
        sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || return 0
        echo "[setup_pi] waiting for the dpkg lock (unattended-upgrades)…"; sleep 10
    done
}

# Some Ubuntu 24.04 SD-card images ship WITHOUT the noble-updates pocket. The
# installed runtime libs then sit at the security-patched version (e.g.
# liblz4-1 1build1.1) while only the base -dev (1build1) is offered, so build
# deps hit "held broken packages". Enable noble-updates so the matching -dev
# packages are available. arm64 uses ports.ubuntu.com.
_ensure_noble_updates() {
    . /etc/os-release
    if ! apt-cache policy 2>/dev/null | grep -q "${UBUNTU_CODENAME}-updates"; then
        echo "[setup_pi] enabling ${UBUNTU_CODENAME}-updates pocket…"
        local uri="http://ports.ubuntu.com/ubuntu-ports/"
        [ "$(dpkg --print-architecture)" = "amd64" ] && uri="http://archive.ubuntu.com/ubuntu/"
        sudo tee /etc/apt/sources.list.d/${UBUNTU_CODENAME}-updates.sources > /dev/null <<SRC
Types: deb
URIs: ${uri}
Suites: ${UBUNTU_CODENAME}-updates
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
SRC
    fi
}

# ── 0. (optional) ROS 2 Jazzy apt repo + ros-base ────────────────────────────
if [ "${INSTALL_ROS:-false}" = "true" ] && [ ! -d /opt/ros/jazzy ]; then
    echo "[setup_pi] adding ROS 2 Jazzy apt repo + ros-base…"
    _apt_prepare
    _ensure_noble_updates
    sudo -E apt-get update
    sudo -E apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo -E apt-get update
    sudo -E apt-get install -y ros-jazzy-ros-base
fi

if [ ! -d /opt/ros/jazzy ]; then
    echo "[setup_pi] ERROR: /opt/ros/jazzy not found. Re-run with INSTALL_ROS=true." >&2
    exit 1
fi

if [ "${BUILD_ONLY:-false}" != "true" ]; then
    # ── 1. Pi-side ROS packages (NO gazebo / ros_gz / ros2_control) ──────────
    echo "[setup_pi] apt: Pi control-side ROS packages…"
    _apt_prepare
    _ensure_noble_updates
    sudo -E apt-get update
    sudo -E apt-get install -y \
        ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool \
        build-essential cmake libssl-dev \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-twist-mux \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
        ros-jazzy-xacro \
        ros-jazzy-rmw-fastrtps-cpp

    # The official Slamtec driver is source-pinned in ros2_ws/lidar.repos.
    # Import it only after vcstool is available and before rosdep scans src/.
    "$SCRIPT_DIR/scripts/import_lidar_dependencies.sh"

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
    # numpy<2 and OpenCV<5: perception is tested against the 4.x/1.x line; a
    # fresh install otherwise pulls opencv 5.0 / numpy 2.5 and crashes in
    # detect_court_line (cv2.HoughLinesP shape changed). Match the PC versions.
    python3 -m pip install --user --break-system-packages \
        "numpy>=1.26,<2" "opencv-python-headless>=4.9,<5" "duckdb>=1.5.3" "onnxruntime>=1.20"
else
    # BUILD_ONLY still restores the external source checkout from the manifest;
    # vcstool must already be installed by an earlier full setup.
    "$SCRIPT_DIR/scripts/import_lidar_dependencies.sh"
fi

# ── 4. Build the workspace packages plus the pinned LiDAR driver ────────────
echo "[setup_pi] colcon build (robot packages + pinned sllidar_ros2)…"
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
( cd "$WS" && colcon --log-base log_jazzy build \
    --build-base build_jazzy --install-base install_jazzy \
    --packages-select sllidar_ros2 tennis_robot_msgs tennis_robot_collection_controller tennis_robot )

echo
echo "[setup_pi] ✅ done. Verify:"
echo "    source /opt/ros/jazzy/setup.bash && source $WS/install_jazzy/setup.bash"
echo "    python3 -c 'import onnxruntime, duckdb, cv2; print(\"py deps OK\")'"
echo "    ros2 pkg prefix tennis_robot_collection_controller   # plugin package present"
echo "    ros2 pkg prefix sllidar_ros2                          # pinned LiDAR driver"
echo
echo "  Next (WS3): set a shared ROS_DOMAIN_ID on PC + Pi, start the PC sim, and"
echo "  bring up the Pi control nodes with use_sim_time:=true. See"
echo "  docs/process/pi-setup-el.md."
