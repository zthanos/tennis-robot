# Tennis Robot Simulation Starter

This is a first simulation-first experiment for a tennis-ball collecting and serving robot.

The initial goal is intentionally small:

- create a virtual tennis court;
- scatter tennis balls at random positions;
- place a differential-drive robot with a camera on the court;
- detect tennis balls from the camera stream using Python/OpenCV.

The project starts with **Webots + Python** because it is easier to get moving than a full ROS/Gazebo stack. ROS 2 can be added once perception and basic navigation behavior are proven.

## Project Layout

```text
controllers/
  ball_detector/
    ball_detector.py        # Webots Python controller
scripts/
  generate_balls.py         # Generate random Webots ball nodes
worlds/
  tennis_court.wbt          # Webots world
docker-compose.yml          # Optional Python dev container
Dockerfile                  # Optional Python/OpenCV environment
Dockerfile.webots           # Webots GUI container with noVNC
pyproject.toml              # Python deps managed with uv
```

## Quick Start

Install Webots on the host machine, then open:

```text
worlds/tennis_court.wbt
```

The robot controller is:

```text
controllers/ball_detector/ball_detector.py
```

When the simulation runs, the controller reads from the robot camera, detects tennis-ball-colored blobs, and draws detection rectangles on the Webots camera display.
The controller also estimates a rough monocular ball distance and bearing from the camera field of view and the known tennis ball diameter.

## Generate New Random Ball Positions

The world contains a marked block where tennis ball nodes are generated. To print a new deterministic set of balls:

With `uv` installed:

```powershell
uv run python scripts/generate_balls.py --count 18 --seed 7
```

Copy the output between the `# BALLS_START` and `# BALLS_END` markers in `worlds/tennis_court.wbt`.

## Optional Docker Dev Shell

Docker is useful for Python tooling and OpenCV experiments, but Webots GUI is usually simpler to run on the host or inside WSLg. The container uses `uv` too.

```powershell
docker compose run --rm sim-dev
uv run python scripts/generate_balls.py --count 18 --seed 7
uv run python -m py_compile controllers/ball_detector/ball_detector.py scripts/generate_balls.py
uv run python scripts/perception_smoke.py
```

## Webots In Docker

The `webots` compose service runs Webots in a virtual X display and exposes it through noVNC.

```powershell
docker compose up webots
```

Then open:

```text
http://localhost:6080/vnc.html
```

The service opens `worlds/tennis_court.wbt` automatically. This is convenient for a reproducible setup, but native Webots on Windows or WSLg may still be smoother for interactive editing.

## Telemetry

The Webots controller has optional OpenTelemetry instrumentation for simulation data:

- `robot.camera.frames`
- `robot.vision.ball.detections`
- `robot.vision.ball.area`
- `robot.vision.ball.distance`
- `robot.vision.ball.bearing`
- `robot.control.loop.duration`
- `simulation.step` traces

In Docker, telemetry is enabled for the `webots` service and sent to the local OpenTelemetry Collector:

```powershell
docker compose up webots otel-collector
docker compose logs -f otel-collector
```

For local debugging without a collector, run with:

```powershell
$env:OTEL_ENABLED="true"
$env:OTEL_EXPORTER="console"
```

## Next Milestones

1. Add depth estimation from camera geometry or a simulated depth camera.
2. Convert image detections into court/world coordinates.
3. Add a simple behavior loop: rotate, detect nearest ball, drive toward it.
4. Add ROS 2 nodes and topics around camera frames, detections, and velocity commands.
5. Test a simulated collector mechanism.
