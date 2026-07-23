# C2 Gazebo covariance evidence report — corrected coverage passed

## Controlled repeat run identity

- Environment: Linux `run_ubuntu.sh`, ROS 2 Humble, headless software renderer.
- World: `gazebo/worlds/tennis_court.sdf`.
- Raw evidence: `runtime/c2_controlled_coverage/*.jsonl`.
- Recorder: `gazebo_covariance_recorder`, which uses the neural YOLO model,
  canonical RGB/depth streams, `/sim/balls` GT and TF at each RGB timestamp.
- Public spatial producer state during the run: unchanged C1 unhealthy; no
  C2 artifact was loaded and no `has_spatial=true` target was published.

## Controlled coverage result

| Metric | Value |
| --- | ---: |
| Trial | Target samples | measured target range (m) | mask ratio | non-target detections | target outlier rate |
| --- | ---: | ---: | ---: | ---: |
| r0_q90 | 30 | 1.02166 | 10% | 29 | 0.0% |
| r0_q98 | 30 | 1.02157 | 2% | 29 | 0.0% |
| r1_q90 | 30 | 1.59126 | 10% | 29 | 0.0% |
| r1_q98 | 30 | 1.59120 | 2% | 29 | 0.0% |
| r2_q90 | 30 | 2.97984 | 10% | 65 | 0.0% |
| r2_q98 | 30 | 2.97982 | 2% | 71 | 0.0% |

## Gate decision

**Coverage PASS — candidate artifact fitting is authorized, but producer
activation remains outside this gate.**

The corrected association rule records `ball_09` and other non-target
detections separately. They do not contribute to target residual or outlier
metrics. The r1 pose was corrected from measured camera geometry, and every
trial validates its measured camera-to-target GT range before recording. The
report-only coverage gate passed with 180 target samples. The separate fitter
gate generated a candidate artifact at
`calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v1.json`
and its per-bin conservation report is
`gazebo-perception-covariance-c2-artifact-report-el.md`. The candidate has not
been loaded; the Gazebo producer remains `spatial_targets_healthy=false`.

## Required C2 follow-up

The next decision is an independent producer-activation review. It must not
load this candidate automatically.
