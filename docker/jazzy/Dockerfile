# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic — the CANONICAL runtime
# (same as run_native.sh), packaged as a container so it runs on a host that
# is not Ubuntu 24.04 (e.g. the Ubuntu 26.04 dev PC, where the noble ROS
# packages cannot load under Python 3.14).
#
# This is NOT the obsolete Humble image (Dockerfile.gazebo / Dockerfile.ros2).
# Use docker/jazzy/run.sh to build and run it; it mounts the repo at the same
# absolute path and runs ./run_native.sh unchanged.

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Sim + control stack. Mirrors scripts/setup_pi.sh plus the PC-only sim pieces
# (ros_gz / gz_ros2_control / controllers) that the Pi intentionally skips.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-twist-mux \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
        ros-jazzy-xacro \
        ros-jazzy-cv-bridge \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool \
        python3-pip build-essential cmake libssl-dev \
        mesa-utils libgl1-mesa-dri libegl1 libglx-mesa0 vulkan-tools mesa-vulkan-drivers \
        sudo less nano \
    && rm -rf /var/lib/apt/lists/*

# System Gazebo Harmonic from the OSRF repo, as on the native 24.04 install:
# sim.launch.py reads /usr/share/gz/gz-sim8/gui/gui.config and the chassis
# bench scripts call /usr/bin/gz + /usr/lib/x86_64-linux-gnu/gz-physics-7.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Python runtime deps, pinned like scripts/setup_pi.sh (numpy<2 / OpenCV<5:
# perception is tested against the 4.x/1.x line). System-wide so any UID works.
RUN python3 -m pip install --break-system-packages --no-cache-dir \
        "numpy>=1.26,<2" "opencv-python-headless>=4.9,<5" "duckdb>=1.5.3" "onnxruntime>=1.20"

# rosdep database (keys are resolved later against the mounted workspace).
RUN rosdep update --rosdistro jazzy || true

# The host user is mapped in at run time (--user UID:GID); give it a writable
# HOME and passwordless sudo for one-off apt installs while experimenting.
RUN mkdir -p /home/dev && HOME=/home/dev rosdep update --rosdistro jazzy \
    && chmod -R a+rwX /home/dev \
    && echo "ALL ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev-nopasswd

ENV HOME=/home/dev
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
