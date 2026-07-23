# C2 Gazebo producer activation report — blocked

## Executed checks

- Linux `run_ubuntu.sh` headless Gazebo environment rebuilt both
  `tennis_robot_msgs` and `tennis_robot` successfully.
- The Gazebo `perception_node` loaded
  `calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v1.json`
  through the strict parser, with platform `gazebo`, calibration ID
  `gazebo-range-depth-quality-diagonal-v1-20260719` and version `gazebo-v1`.
- Startup telemetry reported:
  `spatial_targets_healthy=True`, `spatial_targets_health_reason=healthy`, and
  the artifact ID/version above.
- A live `/perception/ball_detections` message confirmed the healthy heartbeat,
  but every detection correctly had `has_spatial=false`.

## Blocking evidence

The C2 artifact's depth-quality domain is
`[0.8979591836734694, 0.9797979797979798]`. Native Gazebo depth has ROI quality
`1.0`; evaluating the approved model at range `1.5 m`, quality `1.0` returns
`calibration_out_of_domain`. Therefore the producer correctly emits no spatial
target, no matched-depth timestamp and no covariance for those detections.

Extending the domain to `1.0`, applying an activation-only quality degradation,
or supplying a default covariance would be extrapolation/fallback and is not
permitted by the calibration contract.

## Gate decision

**FAILED / blocked.** Strict loading and unhealthy/out-of-domain behavior pass,
but the required live in-domain Gazebo spatial-target evidence cannot be
obtained from the approved artifact. The artifact has not been changed and no
additional activation behavior was introduced.

## Required next decision

Run an additional controlled C2 capture for the native `quality=1.0` bin and
refit/review a new artifact, or explicitly authorize a separately documented
runtime quality mechanism. Until then, this activation gate must not be marked
passed.
