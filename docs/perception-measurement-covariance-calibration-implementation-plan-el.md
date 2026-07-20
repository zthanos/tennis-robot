# Implementation Plan: perception measurement covariance calibration

> Η φάση αυτή προηγείται του `ScanSnapshotBuilder` και είναι ανεξάρτητη από
> Gazebo/physical calibration evidence.

## Phase C1 — Schema, artifact model και health contract

**Implementation**

- Update `BallDetection.msg` και `BallDetectionArray.msg` με το documented
  metadata/health schema.
- Add `perception_covariance_calibration.py` with artifact parser/validator,
  `range_depth_quality_diagonal_v1` model interface and typed rejection.
- Update perception producer wiring to load artifact explicitly and publish
  only non-spatial detections/heartbeat while unhealthy.
- Add health fields to every `BallDetectionArray` heartbeat.

**Tests**

- `tests/test_perception_covariance_calibration.py`: schema parsing, finite,
  symmetric PSD covariance, monotonic range/quality behavior, invalid artifact
  and out-of-domain rejection.
- Update `tests/test_perception_contract_ros.py`: message metadata, health
  status and no-spatial behavior while artifact is unavailable.

**Gate**

- Synthetic valid artifact proves the model/contract behavior.
- Missing, invalid and out-of-domain artifact/input never publish
  `has_spatial=true`.
- No calibrated artifact is shipped or enabled by this phase.

## Phase C2 — Gazebo calibration evidence

Create and validate a Gazebo artifact against the documented
[`gazebo calibration scenario`](gazebo-perception-covariance-calibration-scenario-el.md)
and its acceptance metrics. The evidence capture must compare raw fusion XYZ
with timestamp-aligned `/sim/balls` ground truth, record every rejection, fit
only from measured errors, and emit a versioned report. The artifact gate
requires a conservative per-axis check over every accepted sample in its
declared domain. Only after this gate may Gazebo publish spatial targets and
unblock collection-route Phase 2.

## Phase C3 — Physical OAK-D calibration evidence

Create and validate the physical artifact. This gate is required only for
physical runs and does not block Gazebo Phase 2.
