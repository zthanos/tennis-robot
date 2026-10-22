# Jazzy container (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)

The canonical runtime (`run_native.sh`) packaged as a container, for hosts that
are not Ubuntu 24.04. On the Ubuntu 26.04 dev PC the noble ROS packages in
`/opt/ros/jazzy` cannot load (`rclpy` is built for Python 3.12, the host has
3.14), so ROS/Gazebo run here instead.

This is **not** the obsolete Humble image (`Dockerfile.gazebo`,
`Dockerfile.ros2`, `run_ubuntu.sh`). It runs the same `run_native.sh` with the
repo mounted at the same absolute path, as the host user.

```bash
docker/jazzy/run.sh build                      # once (~7.5 GB image)
docker/jazzy/run.sh gpu-check                  # expect "Accelerated: yes", gz 8.x
BUILD=true docker/jazzy/run.sh sim             # colcon build + full stack, GUI
GAZEBO_HEADLESS=true docker/jazzy/run.sh sim   # no GUI
docker/jazzy/run.sh shell                      # ROS-sourced shell
docker/jazzy/run.sh exec "ros2 topic list"     # one command
```

- GPU: host `/dev/dri` is passed through (Mesa, e.g. Radeon 890M iGPU).
- Display: `DISPLAY`/`XAUTHORITY` (X11 or XWayland) are forwarded.
- Network/IPC: host, so `ROS_DOMAIN_ID` discovery works as natively.
- The container is stopped whenever the script exits (Ctrl-C, `timeout`,
  kill); `docker ps` should be empty afterwards.
- Gazebo Harmonic is installed both as the ROS vendor packages and as the OSRF
  `gz-harmonic` system packages, because `sim.launch.py` and the chassis bench
  scripts use the system paths (`/usr/share/gz/...`, `/usr/lib/.../gz-physics-7`).

Verified 2026-10-11: workspace builds (3 packages); headless
`SLAM_MODE=mapping` run brings Gazebo, the robot, SLAM and Nav2 to
"Managed nodes are active" with no errors.
