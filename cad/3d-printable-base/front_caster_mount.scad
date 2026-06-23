include <common.scad>

// DEPRECATED — the robot is now 4WD (four driven 180 mm wheels, no casters).
// The front corners use the same direct-drive motor pod as the rear, so there
// is no longer a passive caster bracket to print.
//
// Use instead:
//   motor_pod.scad             — the motor pod, printed x4 (one per wheel)
//   drive_wheel_direct_hub.scad — the wheel hub that bolts to the motor shaft
//
// This file is kept only so old references resolve; it renders the motor pod.

use <motor_pod.scad>

motor_pod();
