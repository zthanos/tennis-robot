# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Simulation-first tennis ball collection robot. Stack: Gazebo Harmonic (simulation) + ROS 2 Humble + Python. The active design is "Concept A": level (un-pitched) 4WD skid-steer chassis with four driven 180 mm wheels (two per side, no casters), orange funnel, wide intake roller, optional OAK-D depth camera, and an MPU6050 IMU (gyro+accel) for fusing with wheel odometry.

## Current Architecture Direction

New implementation work should start from `docs/architecture-implementation-guide-el.md` and the active baseline docs in `docs/`, especially validation, search strategy, collection state machine, and mission dashboard plans.

## Commands

### Run the simulation (Gazebo)

```bash
# From WSL shell:
docker compose --profile gazebo up gazebo
# With RViz visualization (map, TF, /scan, robot model):
docker compose --profile gazebo up gazebo rviz
# Or with survey recorder:
docker compose --profile gazebo-rec up gazebo-rec
# Motion-chain diagnosis (sim must be running; writes runtime/motion_diag_*/):
docker compose --profile gazebo exec gazebo bash /workspace/scripts/diagnose_motion.sh
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
| `court_survey_v2_node.py` | **Active** survey coverage controller (`INIT→FIND_NET→COVERAGE→SAVING_MAP→DONE/FAILED`); writes `runtime/court_boundary.json` (schema `court_knowledge_model/v2`) |
| `court_extraction.py` | Pure extraction functions (net/posts, fence rectangle, court lines, obstacles, run-off distances, fail-loud checks) — offline-testable |
| `court_coverage.py` | Vantage points (8 + return pass) and recoverable-failure classifier |
| `lidar_survey.py` | Legacy dead-reckoning perimeter survey FSM (`Ros2LidarCourtSurvey`) — **superseded by the v2 nodes above** |
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

### Web console architecture (`scripts/control_panel.py` + `tennis_robot/console/`)

**Rule — keep the console layered; do not put business logic back in the HTTP handler.** The browser console follows a Controller → Application → Services structure with constructor dependency injection and no global/class-level mutable state:

| Layer | Where | Responsibility |
| --- | --- | --- |
| Entrypoint | `scripts/control_panel.py` | Parse args, build `ConsoleConfig` + stores + services + `ConsoleApp` + `ConsoleServer`, serve. The only wiring point. |
| Controller | `console/server.py` (`ControlPanelHandler`) | HTTP only: parse, validate, route via the `GET_JSON_ROUTES` table, format JSON. Reads the app from `self.server.app`. No business logic. |
| Application | `console/app.py` (`ConsoleApp`) | Use-case orchestration across services (e.g. `nav_test`, `build_diagnostics`, `set_command`). Returns transport-agnostic results (e.g. `NavTestOutcome.kind`); the controller maps kind→HTTP status. |
| Services | `console/*_service.py` | One capability each, owning their own I/O, unaware of each other: `RosService` (all ros2 CLI: survey launch + nav goal/cancel), `SurveyService` (`court_boundary.json`/`court_survey_live.json` + fence bounds), `PathService` (`robot_path.json`), `CameraService` (webcam + CV), `DatabaseService` (DuckDB façade). |
| Config | `console/config.py` (`ConsoleConfig`) | Injected paths + ROS prelude; no service derives paths from `__file__` or globals. |

When adding an endpoint: add a use-case method on `ConsoleApp`, add the route to the controller (a `GET_JSON_ROUTES` entry for simple reads), and keep any cross-service logic in the app, not the handler. The public HTTP API and JSON shapes are a contract consumed by `scripts/control_panel/*.js`, replay scripts and the controller node — keep them stable. Tests: `tests/test_console_app.py` (run without ROS/webcam/DuckDB via fakes). Note: ROS-dependent runs happen in WSL 2.

### IPC: web UI ↔ robot

`runtime/robot_command.json` — written by external UI, polled by controller node.
`runtime/robot_status.json` — written by controller node, read by UI and replay scripts.

### Survey output

`runtime/court_boundary.json` — written by `court_survey_v2_node.py` after survey completes (schema `court_knowledge_model/v2`). Contains net center/posts, court line geometry (court frame), fence corners + extents, run-off distances to fence, obstacles, occupancy point count, and a best-effort `map_artifact` (serialized SLAM map for Nav2 reuse). See `docs/court-survey-v2-spec-el.md`.

### Gazebo world (`gazebo/models/tennis_court/`)

Red clay court (23.77 m × 10.97 m). Ne