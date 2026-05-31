# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Simulation-first tennis ball collection robot. Stack: Webots (simulation) + Python (controller). ROS 2 planned for later. The active design is "Concept A": pitched chassis, orange funnel, wide intake roller, optional OAK-D depth camera.

## Current Architecture Direction

Treat the current Webots controller, control panel, and world as legacy reference
for the next implementation phase. New implementation work should start from
`docs/architecture-implementation-guide-el.md` and the active baseline docs in
`docs/`, especially validation, search strategy, collection state machine, and
mission dashboard plans. Reuse existing code only when it supports the new
contracts; do not let old controller states or UI labels define the new design.

## Commands

### Run the simulation
```powershell
# 1. Open worlds/tennis_court.wbt in Webots and press Play
# 2. Controller at controllers/ball_detector/ball_detector.py runs automatically
```

### Web control panel (sends commands to a running Webots controller)
```powershell
uv run python scripts/control_panel.py
# Open http://127.0.0.1:8081
```

### Route simulator (no Webots needed)
```powershell
uv run python scripts/simulation_panel.py
# Open http://127.0.0.1:8082
```

### Smoke tests (no Webots needed)
```powershell
uv run python scripts/perception_smoke.py
uv run python scripts/collector_behavior_smoke.py
uv run python scripts/survey_behavior_smoke.py
```

### Syntax check
```powershell
uv run python -m py_compile controllers/ball_detector/ball_detector.py
```

### Route benchmarking (Monte Carlo)
```powershell
uv run python scripts/route_benchmark.py --runs 100 --balls 40
```

### Docker workflows
```powershell
docker compose run --rm sim-dev              # Python dev shell
docker compose up webots                     # Webots GUI via noVNC on port 6080
docker compose --profile cad up openscad-gui # OpenSCAD GUI on port 6081
```

## Architecture

### Controller modules (`controllers/ball_detector/`)

| File | Role |
|---|---|
| `ball_detector.py` | Webots controller entry point; wires perception → state machine → telemetry each timestep |
| `perception.py` | OpenCV HSV blob detection; monocular and depth distance estimation; camera→world coordinate transforms |
| `collector.py` | `ConceptACollectorBehavior` — 8-state FSM (idle → scan → align → approach → capture → collected); drives wheel and lift-wheel motors |
| `survey.py` | `CourtSurveyBehavior` — boustrophedon waypoint navigation; writes measurements to `runtime/court_survey.csv` |
| `control_bus.py` | `RobotCommandStore` — file-backed IPC via `runtime/robot_command.json`; bridges web UI ↔ Webots |
| `telemetry.py` | Optional OpenTelemetry setup (metrics, spans); enabled via `OTEL_ENABLED=true` |

### Data flow (single timestep)
```
Webots camera frame
  → perception.detect_largest_ball()      # HSV mask → contour → pixel coords
  → estimate_ball_observation()            # monocular focal-length distance
  → (optional) estimate_depth_ball_observation()  # median depth pixels
  → observation_to_world()                # camera mount + robot pose → world XY
  → ConceptACollectorBehavior.update()    # FSM → motor velocity commands
```

### Scripts (`scripts/`)

| File | Role |
|---|---|
| `control_panel.py` | Minimal HTTP server + HTML UI; writes to `runtime/robot_command.json` |
| `simulation_panel.py` | Browser-based route simulator (fast planning sandbox) |
| `generate_balls.py` | Generates random/realistic-biased ball positions as Webots Solid nodes |
| `route_benchmark.py` | Monte Carlo scenario evaluator; generates training data for policy learning |
| `train_next_ball_policy.py` | Trains next-ball selection model from benchmark output |
| `evaluate_*.py` | Compare planning strategies: defer/edge-pass, skip-risky, scan-replan |

### IPC: web UI ↔ Webots
`runtime/robot_command.json` is a file written by the web panel and polled by the controller's `RobotCommandStore`. No message queues or ROS yet.

### Perception modes
Two modes controlled by `USE_RGB_VISION`:
- **Depth** (preferred when `USE_RGB_VISION=true`): OAK-D camera; `estimate_depth_ball_observation()` uses median of valid depth pixels.
- **Monocular** (fallback): single camera; `estimate_ball_observation()` uses known ball diameter and focal length.

### Configuration via environment variables
All behavioral parameters are tunable without code changes (speeds, PID gains, tolerances, telemetry). Key vars: `ROBOT_COMMAND_FILE`, `USE_RGB_VISION`, `OTEL_ENABLED`, `OTEL_EXPORTER`, `ROUTE_VISUALIZATION`, `ROUTE_VISUALIZATION_PRESET`, `COLLECTOR_*`, `SURVEY_*`.

### Webots world (`worlds/tennis_court.wbt`)
Red clay court (23.77 m × 10.97 m). Net at x=0, baselines at x=±11.885 m, service lines at x=±6.4 m. Perimeter fencing, survey depth targets (floodlight poles, chairs) around the court.

## Key Constraints
- Python 3.12+, managed with `uv` (not pip directly).
- No formal test framework — smoke tests in `scripts/` are the test suite.
- `runtime/` is gitignored; generated files (command JSON, survey CSV, benchmark results) live there.
- Concept A is the active hardware baseline; earlier concept docs in `docs/research/` are archived.
