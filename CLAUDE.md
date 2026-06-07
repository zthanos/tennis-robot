# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Simulation-first tennis ball collection robot. Stack: Gazebo Harmonic (simulation) + ROS 2 Humble + Python. The active design is "Concept A": pitched chassis, orange funnel, wide intake roller, optional OAK-D depth camera.

## Current Architecture Direction

New implementation work should start from `docs/architecture-implementation-guide-el.md` and the active baseline docs in `docs/`, especially validation, search strategy, collection state machine, and mission dashboard plans.

## Commands

### Run the simulation (Gazebo)

```bash
# From WSL shell:
docker compose --profile gazebo up gazebo
# Or with survey recorder:
docker compose --profile gazebo-rec up gazebo-rec
```

### Route simulator (no sim needed)

```powershell
uv run python scripts/simulation_panel.py
# Open http://127.0.0.1:8082
```

### Route benchmarking (Monte Carlo)

```powershell
uv run python scripts/route_benchmark.py --runs 100 --balls 40
```

### Survey replay

```powershell
uv run python scripts/replay_ros2_lidar_survey.py <fixture.jsonl>
uv run python scripts/replay_survey_cmd_vel.py <fixture.jsonl>
```

### Docker workflows

```powershell
docker compose run --rm sim-dev              # Python dev shell
docker compose --profile gazebo up gazebo    # Gazebo GUI via WSLg
docker compose --profile cad up openscad-gui # OpenSCAD GUI on port 6081
```

## Architecture

### ROS 2 nodes (`ros2_ws/src/tennis_robot/tennis_robot/`)

| File | Role |
| --- | --- |
| `controller_node.py` | Main ROS 2 node; integrates survey + motion; publishes cmd_vel and status |
| `lidar_survey.py` | `Ros2LidarCourtSurvey` — LiDAR-first perimeter survey FSM; writes `runtime/court_boundary.json` |
| `motion.py` | Turn tracking and motion primitives |
| `motion_controller.py` | Translates survey/collector commands to `cmd_vel` Twist messages |
| `navigation_node.py` | Navigation node (in progress) |

### Scripts (`scripts/`)

| File | Role |
| --- | --- |
| `simulation_panel.py` | Browser-based route simulator (fast planning sandbox) |
| `route_benchmark.py` | Monte Carlo scenario evaluator; generates training data for policy learning |
| `train_next_ball_policy.py` | Trains next-ball selection model from benchmark output |
| `evaluate_*.py` | Compare planning strategies: defer/edge-pass, skip-risky, scan-replan |
| `survey_replay_record.py` | Records live survey ticks to JSONL for deterministic replay |
| `replay_ros2_lidar_survey.py` | Deterministic replay of recorded survey ticks |
| `replay_survey_cmd_vel.py` | Replay recorded cmd_vel commands |
| `replay_navigation_fixtures.py` | Replay navigation fixtures |

### IPC: web UI ↔ robot

`runtime/robot_command.json` — written by external UI, polled by controller node.
`runtime/robot_status.json` — written by controller node, read by UI and replay scripts.

### Survey output

`runtime/court_boundary.json` — written by `Ros2LidarCourtSurvey` after survey completes. Contains net/baseline positions, fence distances, navigation waypoints, doubles flag.

### Gazebo world (`gazebo/models/tennis_court/`)

Red clay court (23.77 m × 10.97 m). Net at x=0, baselines at x=±11.885 m, service lines at x=±6.4 m. Perimeter fencing modeled as separate DAE meshes.

### Configuration via environment variables

All behavioral parameters are tunable without code changes (speeds, PID gains, tolerances, telemetry). Key vars: `ROBOT_COMMAND_FILE`, `ROBOT_STATUS_FILE`, `OTEL_ENABLED`, `OTEL_EXPORTER`, `COLLECTOR_*`, `SURVEY_*`, `ROS2_SURVEY_*`.

## Key Constraints

- Python 3.12+, managed with `uv` (not pip directly).
- No formal test framework — replay scripts in `scripts/` and `fixtures/` are the test suite.
- `runtime/` is gitignored; generated files (command JSON, survey JSON, benchmark results) live there.
- Concept A is the active hardware baseline; earlier concept docs in `docs/research/` are archived.
