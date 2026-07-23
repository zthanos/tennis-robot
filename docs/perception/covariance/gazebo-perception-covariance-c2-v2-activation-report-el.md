# C2 Gazebo v2 producer activation report

## Selected artifact and startup telemetry

Το Gazebo launch επιλέγει ρητά και μόνο:

`calibration_artifacts/gazebo/range_depth_quality_diagonal_v1-gazebo-v2.json`

Το v1 artifact δεν τροποποιήθηκε ούτε φορτώθηκε. Στο Linux `run_ubuntu.sh`
startup, μετά από rebuild των `tennis_robot_msgs` και `tennis_robot`, το
`perception_node` εξέπεμψε:

- `spatial_targets_healthy=True`
- `spatial_targets_health_reason=healthy`
- `calibration_id=gazebo-range-depth-quality-diagonal-v1-20260719-v2`
- `calibration_version=gazebo-v2`

## Native live verification (q100)

Σε native unmasked Gazebo depth, live `BallDetectionArray` είχε
`has_spatial=true`, RGB/matched-depth stamp `7.712 s`, και covariance:

`[2.494997007200257e-06, 0, 0, 0, 0.0005688899341584057, 0, 0, 0, 0.002199572393755609]`.

Για το published optical XYZ, το v2 producer-model adapter με `quality=1.0`
υπολόγισε:

`[2.4949972498137068e-06, 0, 0, 0, 0.0005688899394959016, 0, 0, 0, 0.0021995724023683868]`.

Η διαφορά είναι μόνο float32 ROS message serialization.

## Domain behavior

- q90/q98: supported calibrated domains, covered από passed controlled
  evidence και pure adapter tests· δεν είναι native live Gazebo input modes.
- Φυσική μετακίνηση του robot στο `x=-10.5 m` έβαλε το central `ball_02`
  target εκτός calibrated range. Η live array παρέμεινε healthy, αλλά η
  central target detection είχε `has_spatial=false`, χωρίς matched-depth
  timestamp ή covariance. Ένα άλλο, nearer in-domain ball παρέμεινε spatial,
  όπως απαιτεί το per-detection out-of-domain contract.
- Το physical OAK-D platform test απορρίπτει το Gazebo v2 artifact με
  `calibration_platform_mismatch`.

## Gate decision

**PASS.** Δεν προστέθηκε fallback covariance, runtime quality mutation ή
ScanSnapshotBuilder change.
