# Native Ubuntu handoff

Status captured on 2026-07-05 from branch `feat/matrix-based-collection`.

## Current working baseline

The Windows/WSL 2 workflow starts the simulation with:

```bash
./run.sh rviz foxglove
```

`run.sh` defaults to headless Gazebo and uses the Docker development overlay,
so source and launch changes are rebuilt without rebuilding the image. A clean
startup can spend roughly 40-60 seconds in `colcon build` before Gazebo and the
controller appear. Once the controller starts, a persisted `collect` command
is applied immediately.

The current stack keeps Gazebo, ROS 2, perception, SLAM/Nav2, collector
behaviour and ros2_control in Linux. The Windows browser / Foxglove client is
only a UI. Preserve that split initially on native Ubuntu; do not introduce a
second Windows control path during the migration.

## First native-Ubuntu checks

1. Install Docker Engine with the Compose plugin and verify `docker compose
   version`.
2. Check out this branch and obtain the required neural model. Model weights
   are intentionally ignored by Git:

   ```bash
   ls -lh models/yolov8n.onnx
   ```

   If missing, recreate it using `scripts/export_yolo_onnx.py` as documented in
   `models/README.md`.
3. The base Compose file contains WSL-specific GPU/UI integration:
   `/dev/dxg`, `/mnt/wslg`, `/usr/lib/wsl`, and the Mesa D3D12 adapter setting.
   These paths do not exist on native Ubuntu. `docker-compose.ubuntu.yml`
   removes those mounts and provides a software-rendered GUI default, while
   `docker-compose.ubuntu-gpu.yml` optionally exposes `/dev/dri`:

   - keep the existing WSL settings available;
   - use `/dev/dri` for Intel/AMD native Linux rendering;
   - keep CPU/software rendering as the default fallback;
   - treat NVIDIA Container Toolkit support as a separate future override;
   - do not remove the stable ROS topic and perception contracts.
4. Start Gazebo and RViz together, then optionally add Foxglove:

   ```bash
   ./run_ubuntu.sh
   ./run_ubuntu.sh foxglove
   UBUNTU_GPU=true ./run_ubuntu.sh foxglove  # only with /dev/dri
   ```

   Run the launcher from the logged-in Ubuntu desktop session. It passes the
   active Xwayland authorization cookie into both GUI containers; running it
   from a shell without `DISPLAY`/`XAUTHORITY` fails early with an explanation.

5. Verify the neural detector started. Classical HSV detection is not an
   allowed runtime fallback:

   ```bash
   docker compose --profile gazebo logs gazebo | grep -E "loaded YOLO|perception_node"
   ```

## Collector changes in this checkpoint

- The intake is roller-first. The old push-ramp collision is replaced by a
  curved channel and deflector.
- The scoop is shifted 15 mm rearward.
- The scoop physics collision is a 2 mm curved sheet with 1 mm court
  clearance; the visual lip can still appear at court level.
- Approach speed is `0.16 m/s`; capture speed is `0.07 m/s`.
- Each roller paddle has a Gazebo contact sensor.
- RViz receives red roller-contact markers on
  `/sim/roller_contact_markers`.
- `/sim/roller_contact` is the short-held boolean contact heartbeat.

The mechanism has not yet completed an end-to-end simulated ball capture.
Validate roller contact, channel travel, basket entry and collection
confirmation in that order before changing the motor or redesigning geometry.

## Simulation-time contract

The WSL software-rendered run measured a very low Gazebo real-time factor.
Collector timeouts therefore use ROS simulation time:

- `controller_node` launches with `use_sim_time=true` in simulation;
- mission, perception-expiry and ball-map timing use the ROS node clock;
- wall time remains only for human-readable uptime and file-write throttling.

On physical hardware, without `use_sim_time`, the same node clock naturally
uses system time.

## Telemetry and evidence

`runtime/robot_status.json` is a current snapshot, not a durable history.
Semantic collection events are now also emitted and persisted:

- ROS/Foxglove: `/telemetry/collection_events`
- append-only file: `runtime/collection_events.jsonl`
- command history: `runtime/robot_command_history.jsonl`

Each collection event contains `run_id`, mission-relative `t_s`, ROS
`sim_time_s`, wall `recorded_at_s`, robot pose, phase, motion owner/path,
blocker and event-specific fields.

Useful live checks:

```bash
tail -f runtime/collection_events.jsonl
python3 scripts/watch_collect.py
docker compose --profile gazebo logs -f gazebo
```

Foxglove is currently a bridge, not an automatic recorder. It can record MCAP
manually, while the JSONL file remains the always-on semantic event history.

## First validation sequence on Ubuntu

1. Record GPU, renderer and Gazebo real-time factor.
2. Confirm `/clock`, `/scan`, camera RGB/depth and
   `/perception/ball_detections`.
3. Confirm active ros2_control controllers and wheel odometry.
4. Run a short straight-motion test and compare commanded versus measured
   velocity.
5. Start `collect` and watch `collection_events.jsonl`.
6. Confirm a roller/ball contact event visually in RViz/Foxglove.
7. Confirm the ball reaches the basket IR pair and increments
   `balls_collected`.

## Repeatable Ubuntu verification commands

Run these checks from the repository root in native Ubuntu or WSL 2. Do not use
native Windows/PowerShell for the ROS-dependent groups.

### 1. Environment and repository checks

```bash
git branch --show-current
git status --short
docker compose version
test -r /opt/ros/jazzy/setup.bash
test -s models/yolov8n.onnx
```

The branch should be `feat/matrix-based-collection`. A missing or empty ONNX
model is a failed prerequisite: the simulator must report perception as
unhealthy and must not switch automatically to HSV/color detection.

### 2. Fast checks before launching ROS

```bash
uv sync
UV_CACHE_DIR=/tmp/uv-cache uv lock --check
bash -n run.sh run_ubuntu.sh build_run.sh start_sim.sh scripts/docker_dev_entry.sh
python3 -m py_compile ros2_ws/src/tennis_robot/launch/sim.launch.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python -m pytest \
  tests --ignore=tests/test_console_app.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python -m pytest \
  tests/test_console_app.py
uv run python scripts/collect_pattern_smoke.py
uv run python scripts/search_behavior_smoke.py
```

The console test runs separately because the repository contains distinct
console and ROS Python packages that both use the name `tennis_robot`; collecting
them in one pytest process makes imports order-dependent.
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` also prevents ROS `launch_testing` plugins
from leaking into the isolated project environment. The
`tests/test_perception_contract_ros.py` test may still be skipped when `rclpy`
is not visible there. It must be exercised in the ROS environment by the next
group; a skip is not equivalent to a passing ROS test.

### 3. Native ROS 2 build and package tests

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
cd ..
mkdir -p runtime/ros_logs
ROS_LOG_DIR="$PWD/runtime/ros_logs" \
  python3 -m pytest tests/test_perception_contract_ros.py
```

The ROS packages currently do not register tests with `colcon test`, so the
perception-contract test is invoked explicitly in the sourced ROS environment.
The dedicated `ROS_LOG_DIR` keeps the check usable on restricted hosts and in
CI. Any failure must be investigated before a simulation change is accepted.
If ROS 2 Jazzy is deliberately installed elsewhere, set `ROS_DISTRO_TARGET`
for `start_sim.sh` and source the matching setup file in this group.

### 4. Simulation smoke test

For the native Ubuntu Compose workflow, start the Gazebo GUI and RViz in one
terminal:

```bash
./run_ubuntu.sh
```

Then run the checks below in a second terminal:

```bash
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo ps
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo logs gazebo |
  grep -E "loaded YOLO|perception_node|controller_manager|ERROR|FATAL"
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh; . /ros2_ws/install/setup.sh;
   . /workspace/ros2_ws/install_docker/local_setup.sh; ros2 node list'
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh; . /ros2_ws/install/setup.sh;
   . /workspace/ros2_ws/install_docker/local_setup.sh;
   ros2 control list_controllers'
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh; . /ros2_ws/install/setup.sh;
   . /workspace/ros2_ws/install_docker/local_setup.sh;
   timeout 10 ros2 topic hz /clock --window 20;
   code=$?; [ $code -eq 124 ]'
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo exec gazebo bash -lc \
  '. /opt/ros/humble/setup.sh; . /ros2_ws/install/setup.sh;
   . /workspace/ros2_ws/install_docker/local_setup.sh;
   timeout 10 ros2 topic echo /perception/ball_detections --once'
```

Passing means that the Gazebo service remains running, required controllers are
`active`, `/clock` advances, and `/perception/ball_detections` emits either
detections or an empty heartbeat. No message before the timeout is a failure,
because downstream control must be able to expire stale observations.

For a native, non-Docker Jazzy installation, use `./start_sim.sh true` and run
the same `ros2` commands directly after sourcing
`ros2_ws/install/setup.bash`.

Stop and clean up the Compose smoke test with:

```bash
docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml \
  --profile gazebo down
```

Do not interpret missing ROS/Linux dependencies in native Windows as product
failures. All ROS 2, Gazebo and simulation validation belongs to Ubuntu.
