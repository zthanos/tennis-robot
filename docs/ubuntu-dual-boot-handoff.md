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
3. The current Compose file contains WSL-specific GPU/UI integration:
   `/dev/dxg`, `/mnt/wslg`, `/usr/lib/wsl`, and the Mesa D3D12 adapter setting.
   These paths do not exist on native Ubuntu. Before the first full launch,
   add an Ubuntu Compose override (or refactor platform-specific device/mount
   settings into overrides):

   - keep the existing WSL settings available;
   - use `/dev/dri` for Intel/AMD native Linux rendering;
   - use NVIDIA Container Toolkit for an NVIDIA GPU;
   - keep a CPU/headless fallback;
   - do not remove the stable ROS topic and perception contracts.
4. Start Gazebo alone first, then add RViz and Foxglove:

   ```bash
   ./run.sh
   ./run.sh rviz foxglove
   ```

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

Do not interpret missing ROS/Linux dependencies in native Windows as product
failures. All ROS 2, Gazebo and simulation validation belongs to Ubuntu.
