# Compact ramp-to-receiving-chute handoff repair

Date: 2026-08-25

## Diagnosis

The failed ball centre `(403.5, 0, 33.0) mm` is not touching either ramp.
Exact STL triangle distance gives the same closest point for the relieved-bin
collision and the authoritative receiving chute:

- contact point `(370.597, 0, 34.030) mm`;
- surface-to-ball normal `(0.9995, 0, -0.0313)`;
- distance `32.919 mm`, or `0.081 mm` penetration for a 33 mm radius ball;
- relieved and original ramp distance `53.424 mm` (`20.424 mm` clearance).

For motion from larger to smaller X, the original chute first becomes reachable
at ball-centre `X=403.597 mm`; the protected wheel-contact construction is
`X=381.200 mm`. The receiving edge therefore leads the wheels by `22.397 mm`
and presents an almost purely longitudinal blocking normal.

Controlled runs held the ball, robot, wheel speed, physics, PARKED datum and
flywheel state fixed:

- historical pre-relief reference `runtime/intake_sweeps/20260824_055401`:
  wheel contacts `43/43`, release `6/6`, basket evidence `8/8`;
- all PARKED reliefs: final X `403.6 mm`, wheel contacts `0/0`, release `4/6`,
  basket evidence `4/8`;
- identical model with the original unrelieved ramp: final X `403.5 mm`, wheel
  contacts `0/0`, release `4/6`, basket evidence `4/8`.

The ramp subtraction is therefore not causal. Restoring the old ramp alone is
both ineffective and unnecessary.

The longitudinal X/Z construction used ground `Z=0`, ball radius `33 mm`,
wheel capture `X=381.200 mm`, chute edge `X=370.597 mm`, ramp envelope
`X=319.650..360.350 mm`, and the hood above the same 180 mm entry corridor.
The original and prerepair-relieved ramp solids first diverge at
`X=344.277 mm` when travelling inward (removed-solid extent
`X=319.650..344.277`, `Z=17.000..49.780 mm`). This is `59.220 mm` after the
ball has already met the blocking chute edge, so it cannot cause the first
stop. The ramp can first support a ground-resting ball at approximately centre
`X=370.186 mm`, chronologically after wheel capture once the obstruction is
removed.

## Minimum repair

`compact_repaired_receiving_chute()` opens a fabrication-realistic centre lane
through the 50 mm-long receiving tile. Its width is 70 mm: one 66 mm tennis-ball
diameter plus the approved 2 mm clearance on each side. The nominal X=470 mm
entry plane remains represented by the fixed hood's lateral cheeks and roof;
no launcher, wheel, basket PARKED, bridge or chassis datum moves.

Cutting only the front transverse rod was tested first and rejected: the square
ends of the longitudinal wires became a new stop at ball-centre X `400.0 mm`
with `0/0` wheel contact. The accepted lane removes those successive end-face
steps. `compact_parked_ramp_relief()` now subtracts the actual repaired PARKED
bin and fixed hood, restoring the existing ramp skin only inside the opened
lane, then independently subtracts both approved tyre pockets. No replacement
ramp profile was invented.

## Static and alignment gates

Exact Boolean results for the accepted geometry are zero for basket/intake,
each basket/wheel pair, basket/launcher, unintended basket/chassis,
hood/launcher, every hood-support blocker, and each ramp/wheel pair. The
intentional flange/chassis engagement remains `360 mm3`. CAD-to-URDF mesh
deviation is zero for the regenerated bin, hood and ramp; the unchanged intake
wheel tessellation remains the overall maximum at `0.139 mm`.

## Dynamic and contact order

One isolated, no-retry acceptance run produced:

- left/right wheel contacts `48/48`;
- peak left/right carriage travel `5.021/5.021 mm`;
- release criteria `8/8`, including the new contact-order guard;
- basket entry, settling, dwell and retention `8/8`;
- final ball position `(265.5, 2.5, 56.3) mm` in robot coordinates.

The final visual-validation run recorded wheel contact at `12.9249 s`, compact
ramp contact at `13.1521 s`, and no receiving-chute contact before either.
It measured `41/41` wheel samples, 19 ramp samples, force P95 `8.014 N` and
maximum `9.525 N`. The camera sequence shows continuous approach, bilateral
capture, travel through the visible centre lane and lift onto the ramp, with
clear tyre pockets and no floating or clipping geometry.

The required analyzer criterion
`wheel_capture_before_blocking_chute_contact` now fails when either wheel has
no contact, when a chute contact precedes bilateral capture, or when the ball
does not cross the wheel throat. Reanalysis gives `false` for the original
X=403.5 mm regression and `true` for the repair.

## Mass properties

Exact homogeneous volume/centroid comparison for the moving bin is:

- before: `612072.677 mm3`, centroid `(211.174, 1.744, 91.093) mm`;
- after: `607811.743 mm3`, centroid `(209.460, 1.757, 91.481) mm`;
- removed material: `4260.934 mm3`;
- mass: `1.019 -> 1.012 kg` (delta `-0.007 kg` using the established proxy);
- moving-bin COM delta `(-1.714, +0.012, +0.389) mm`;
- bin plus fixed hood: `1.106 -> 1.099 kg`.

The ramp volume and mass proxy remain `122079.815 mm3` and `0.074 kg`; the
repair changes which contact skin survives the Boolean without a material mass
change at reported precision.
