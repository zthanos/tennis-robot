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
Detections are projected into robot-base and court/world XY coordinates using the front camera mount and the robot pose, so downstream navigation can reason in meters instead of image pixels.

The current world includes **Concept A: Funnel + Lift Wheel** as the front collector module:

- orange funnel plates and a low intake lip on the robot front;
- a rubber-like lift wheel with `lift_wheel_motor`;
- a transparent visual hopper;
- a simulated intake zone that removes a tennis ball once the collector state machine captures it.
- an aligned `front_depth` RangeFinder that mimics the OAK-D depth contract for distance estimates.

The Webots court is modeled after the reference clay court photos:

- red clay playing surface with lighter wear/brush marks;
- off-white tennis lines;
- tall dark perimeter fencing for survey/depth measurements;
- side stone patio with planter boxes, a chair/umbrella area, floodlight poles, and green visual mass outside the court.

The controller behavior is:

```text
idle -> scan -> align -> approach -> capture -> collected
```

The Webots camera display overlays the current collector state and collected-ball count. The controller starts in `idle` and waits for a command from the Python control panel before it begins collecting. It prefers depth-based ball distance when `front_depth` returns valid range pixels, then falls back to monocular ball-size distance.

## Python Control Panel

Run the web control panel:

```powershell
uv run python scripts/control_panel.py
```

Then open:

```text
http://127.0.0.1:8081
```

Use **Start collection** to write `mode=collect` to `runtime/robot_command.json`; the Webots controller polls that file and starts the collector state machine. Use **Stop** to return the robot to `idle`. You can override the command file for both processes with `ROBOT_COMMAND_FILE` if the UI and Webots are launched from different working directories.

Use **Survey court** to drive a boustrophedon measurement pattern across the court. The controller visits rows across the court, samples the front depth reading at each waypoint, and writes measurements to:

```text
runtime/court_survey.csv
```

The survey CSV includes robot position, heading, front range reading, distance to the four outer fences, and distances to the nearest singles sideline, doubles sideline, baseline, and service line.

The movement speed defaults are deliberately conservative in Docker:

```text
COLLECTOR_SCAN_ANGULAR_SPEED_RAD_S=0.35
COLLECTOR_MAX_ALIGN_ANGULAR_SPEED_RAD_S=0.45
COLLECTOR_ALIGN_ANGULAR_GAIN=1.4
COLLECTOR_APPROACH_SPEED_M_S=0.12
COLLECTOR_CAPTURE_SPEED_M_S=0.05
COLLECTOR_REVERSE_SPEED_M_S=0.08
ROBOT_MAX_WHEEL_SPEED_RAD_S=3.5
```

Increase `COLLECTOR_APPROACH_SPEED_M_S` only after the robot reliably keeps the ball in view during `align` and `approach`.

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
uv run python scripts/collector_behavior_smoke.py
uv run python scripts/survey_behavior_smoke.py
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

## OpenSCAD In Docker

### GUI in the browser (no local install)

The `openscad-gui` service runs OpenSCAD on a virtual display and exposes it through noVNC (same idea as the Webots container). Port `6081` avoids clashing with Webots on `6080`.

```powershell
docker compose --profile cad up --build openscad-gui
```

Open:

```text
http://localhost:6081/vnc.html
```

By default it loads `cad/3d-printable-base/full_robot_concept.scad`. Override with:

```powershell
$env:OPENSCAD_FILE="/workspace/cad/3d-printable-base/base_tile.scad"
docker compose --profile cad up openscad-gui
```

Files under `cad/` are the mounted repo workspace, so saves in the GUI write back to your project folder.

### Headless STL export

The `openscad` compose service uses the official `openscad/openscad:bookworm` image to export printable CAD files without a GUI.

```powershell
.\scripts\export_stl.ps1
```

To export a single model manually:

```powershell
docker compose --profile cad run --rm openscad openscad -o cad/3d-printable-base/stl/base_tile.stl cad/3d-printable-base/base_tile.scad
```

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

To watch the robot controller state directly:

```powershell
docker compose logs -f webots
```

Useful controller log lines look like:

```text
control mode changed to collect
mode=collect collector=align visible=True distance=5.03m bearing=-29.5deg balls=0 ball_world=(-3.64,-2.49)
```

For local debugging without a collector, run with:

```powershell
$env:OTEL_ENABLED="true"
$env:OTEL_EXPORTER="console"
```

## Browser Route Simulator

Run the lightweight route simulation panel:

```powershell
uv run python scripts/simulation_panel.py
```

Then open:

```text
http://127.0.0.1:8082
```

This panel is a fast planning sandbox before deeper Webots work. It models the tennis court bounds, half-court or full-court movement area, random 40-50 ball dispersion, fixed obstacles, moving people with safety zones, travel speed, pickup time, scan time, periodic re-scans, and obstacle-aware route planning. During movement it performs continuous lookahead collision checks, pauses for emergency clearance, and replans if a dynamic obstacle enters the active path. It records telemetry for the run, including detected/collectable/blocked balls, total distance, total time, planned and runtime replans, safety stops, near misses, per-leg travel time, and scan/pickup/re-scan events.

## Next Milestones

Done:

1. Add depth estimation from camera geometry or a simulated depth camera.
2. Convert image detections into robot-base and court/world coordinates.
3. Add a simple behavior loop: rotate, detect nearest ball, drive toward it.
4. Test a simulated collector mechanism.

Next:

1. Add ROS 2 nodes and topics around camera frames, detections, and velocity commands.
2. Publish the world-coordinate detections to a route planner instead of only using the nearest visible ball.
3. Add jam/blocked-intake sensing to the collector state machine.
