# C2 r1 diagnostic capture — root cause

## Scope

Μόνο το `r1_q98` trial εκτελέστηκε. Δεν δημιουργήθηκε artifact, δεν άλλαξε
producer activation και δεν εκτελέστηκε νέο full capture.

## Accepted-target evidence

- Requested Gazebo robot pose: `(-8.98, 0.0, 0.0 rad)`.
- Observed true robot base pose: `(-8.98, -0.0, -0.001, -0.0 rad)`.
- Readiness: pass after 3 required stable frames (41 observed), with a fresh
  recorder buffer and RGB timestamps after the settle timestamp.
- Target world pose: `ball_02 = (-6.4, 0.0, 0.033) m`.
- Measured camera-to-target GT range at RGB timestamp: `2.09483–2.09501 m`.
- Nominal manifest r1 range: `1.25–2.00 m`.
- RGB/depth timestamp delta: `0.0 s`; simulation-clock sample age: `0.032 s`.
- Shared ROI quality: `0.97959`.
- First accepted raw camera XYZ: `[0.00442, -0.10391, 2.05723] m`.
- Its timestamp-aligned GT XYZ: approximately `[0.00005, -0.11380, 2.09178] m`;
  3D residual `0.0362 m`.

Raw evidence is retained under
`runtime/c2_controlled_coverage/r1_q98.jsonl`; rejected-candidate diagnostics
are in the adjacent `.rejections.jsonl` file.

## Explicit diagnosis

| Candidate cause | Finding |
| --- | --- |
| 1. Async set-pose / insufficient settle | **Ruled out for accepted target samples.** Requested and Gazebo true base poses agree within tolerance; readiness and fresh-frame gates passed. |
| 2. Base-vs-camera extrinsic mismatch | **Not supported by this trial.** The timestamp-aligned camera-from-base TF is stable, and the accepted target estimate is only 3.6 cm from GT. |
| 3. Stale frame / TF timestamp mismatch | **Ruled out for accepted target samples.** RGB/depth delta is zero, age is 32 ms in simulation time, and both base/camera TF lookups use the RGB timestamp. |
| 4. Incorrect GT association / trial leakage | **Confirmed for the reported outlier rate.** All 29 `ground_truth_match_gate_failed` candidates are nearest to `ball_09` (0.020–0.077 m), not target `ball_02` (0.874–0.951 m). Trial ID and target GT reference remain `r1_q98` / `ball_02`; there is no trial-ID leakage. |
| 5. Nominal vs measured range | **Confirmed.** The manifest’s nominal r1 bin is wrong for this camera geometry: measured GT range is about 2.095 m, outside `[1.25,2.00]`. |

## Decision

The current C2 outlier accounting is invalid: it counts visible, correctly
localized non-target ball detections as target measurement outliers. The next
change must make target association explicit before counting an outlier. Also,
the trial manifest must be derived from measured camera-to-ball GT range, not
from approximate base/world placement. C2 remains failed; no artifact may be
generated until those two issues are resolved and coverage is repeated.
