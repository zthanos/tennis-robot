# Survey fix plan: event-processing deadlock + turn-completion logic

Scope: `ros2_ws/src/tennis_robot/tennis_robot/lidar_survey.py` and
`ros2_ws/src/tennis_robot/tennis_robot/motion.py`.

Two independent defects, planned together because both touch state-transition
control flow and share the same replay-based validation path.

---

## Problem 1 — guard-rejected waypoint proposals halt the robot

### Root cause

Every drive/approach state calls `_propose_waypoint(...)` and then
**unconditionally** `return BaseCommand(0.0, 0.0)`, ignoring the boolean the
proposer returns (`lidar_survey.py`: BASELINE_APPROACH ~439–445,
DRIVE_SIDELINE ~481–487, DRIVE_LONG_SIDE ~526–531, DRIVE_FAR_SHORT ~567–573,
DRIVE_RETURN ~599–607). When the parallelism guard rejects the proposal
(heading error > `waypoint_boundary_tolerance_rad`, lines 640–643), the robot
still stops, `_active_wp` stays `None`, the state does not advance, and the next
tick reproduces the identical rejected condition. Result: a permanent stall at
the fence until the section timeout fires `PARTIAL` (and the TURN states have no
timeout at all — see Problem 2). The guard meant to *reject an unreliable
detection* instead *freezes the robot with no recovery*.

### Fix strategy

Make a rejected proposal a **"keep driving / re-align"** signal, not a stop.

1. **Branch on the proposer's return value.** In each state, only `return
   BaseCommand(0.0, 0.0)` when `_propose_waypoint(...)` returns `True`. On
   `False`, fall through to the existing `WaypointGenerator` drive path so the
   robot continues and the fence-following lateral correction can re-establish
   a parallel approach.

2. **Add heading authority to the recovery path.** The current
   `WaypointGenerator.compute` only corrects lateral fence distance; it does not
   pull heading back toward the section heading. Add an optional
   `align_heading_rad` term to `_drive_to_waypoint` (or blend the section
   heading into the lookahead target) so a rejected, non-parallel approach
   actively steers back toward `expected_heading_rad` instead of creeping
   straight into the fence.

3. **Add a re-alignment backoff guard.** If the robot is inside
   `safety_slow_range_m` *and* the proposal is being rejected, prefer rotation
   over forward motion (command low/zero linear, non-zero angular toward the
   section heading) so `_apply_safety` cannot zero out the angular correction
   and trap it (currently `front <= safety_stop_range_m` returns `(0,0)`,
   killing both axes — lines 1040–1041).

4. **Bound the recovery.** Keep the existing section timeout as the final
   backstop, but add an explicit `guard_reject_count` so that if the guard
   rejects N consecutive times without the heading improving, finalize with a
   distinct `failure_reason` (e.g. `..._unrecoverable_heading`) instead of
   waiting out the full 300 s timeout.

### Files / functions touched

- `lidar_survey._step` — five `if front <= stop:` blocks: branch on proposer
  result.
- `lidar_survey._drive_to_waypoint` — add optional heading-alignment term.
- `lidar_survey._apply_safety` — allow angular-only commands through when
  linear is already ~0 (don't zero a pure rotation).
- `lidar_survey.__init__` — add `_guard_reject_count` state.

---

## Problem 2 — turn completion logic, worst on the 180°

### Root cause

`_turn_complete` (lines 770–780) locks a single rotation `direction` from the
sign of the initial `angle_delta`, and `TurnTracker.complete` (motion.py 56–69)
requires **both** an accumulated-progress threshold **and** a heading-error
threshold. Three compounding issues:

- **Antipodal singularity (the 180).** `target = yaw + π` makes the initial
  `err` exactly `±π`. `angle_delta` returns `-π`, locking `direction = -1`, but
  ±π is an unstable point for the proportional controller: any odometry
  perturbation flips the commanded sign to `+π`, so the robot rotates opposite
  to the pinned `direction`. Then `progress_rad += step*direction` goes negative
  and clamps to 0 (motion.py 46), so the turn can rotate the wrong way and/or
  never satisfy `enough_progress`. The 90° turns sit at `err = +π/2`, far from
  the singularity, so they are unaffected — hence "especially the 180."

- **Hard-coded `target_delta_rad`.** Turns pass a nominal `π` / `π/2`, but the
  real angle to cover is `angle_delta(target_heading, actual_entry_yaw)`, which
  drifts because the next heading is built from the *ideal* previous heading
  (`+π/2`, lines 671–678), not the measured one. When the true angle is less
  than `target_delta − tol`, `enough_progress` is unsatisfiable at the target
  heading → overshoot/hunting.

- **No TURN-state timeout.** Every drive state has `sideline_drive_timeout_s`,
  but none of the five TURN states do, so any non-completion spins forever.

- (Minor) `TurnTracker.update` drops steps `> 35°` (motion.py 44); the 180
  needs the most cumulative progress and is most exposed to under-counting.

### Fix strategy

Replace progress-integral completion with **heading-error + settle**, and make
direction deterministic and singularity-safe.

1. **Complete on heading error, not accumulated progress.** A turn is done when
   `abs(angle_delta(target_heading, yaw)) <= heading_tolerance_rad` for K
   consecutive ticks (settle counter, e.g. K=3) to reject momentary crossings.
   Drop the `target_delta`/progress requirement for the completion test, or keep
   `progress_rad` purely as telemetry.

2. **Derive the turn magnitude from the actual entry heading.** Compute
   `required = angle_delta(target_heading, entry_yaw)` once on entry; use its
   sign as the commanded direction and its magnitude only for telemetry / a
   sanity bound — never as a completion gate.

3. **Make the 180 direction deterministic.** Because `±π` is ambiguous, pick the
   rotation sense explicitly when `abs(required) > π − ε` (e.g. always CCW, or
   inherit the court-traversal handedness) and **hold it** for the whole turn so
   a perturbation at the antipode cannot flip the controller mid-turn. Drive the
   shortest-path `err` continuously via the existing `_proportional_turn`.

4. **Add a timeout to every TURN state.** Introduce `turn_timeout_s` (config,
   env-overridable) and, in each TURN state, finalize with a
   `..._turn_timeout` reason if exceeded — symmetric with the drive states.

5. **Guard the 35° step filter.** Either raise the per-tick cap relative to
   `turn_speed_rad_s * dt`, or — since completion no longer depends on the
   progress integral — demote the filter to telemetry-only so it cannot stall a
   turn.

### Files / functions touched

- `motion.TurnTracker` — add settle-counter completion; keep `progress_rad` as
  telemetry only; make `complete()` heading-error based.
- `lidar_survey._turn_complete` — compute required angle/direction from entry
  yaw; deterministic 180 direction; pass settle params.
- `lidar_survey.LidarSurveyConfig` — add `turn_timeout_s`,
  `turn_settle_ticks`; wire into `from_env`.
- `lidar_survey._step` — add timeout check to the five TURN states.
- `lidar_survey._enter` — record turn entry yaw / reset settle counter.

---

## Shared design notes

- The `or`-based float fallbacks for headings (lines 670–678,
  `(self._sideline_heading or 0.0)`, `(self._net_approach_yaw or yaw_rad)`)
  treat a legitimate `0.0 rad` heading as missing. Switch to `... if x is not
  None else ...` while editing this region.
- Keep all new thresholds env-overridable per the project's
  "tunable-without-code-changes" constraint (CLAUDE.md).

## Validation

No formal test framework — use the replay suite as the regression gate:

1. `tests/test_lidar_survey_long_side.py` must still pass.
2. Add a unit test for `TurnTracker`: a 180 starting at the antipode with
   injected ±noise on yaw must complete and end within `heading_tolerance`, and
   must not rotate net > ~π + tol.
3. Add a unit test: a 90° turn whose entry yaw is off-ideal by ±15° still
   completes at the correct absolute heading without overshoot.
4. Add a `Ros2LidarCourtSurvey` test where the parallelism guard is forced to
   reject once: the robot must re-align and still commit the corner (not stall).
5. Replay the recorded fixtures
   (`scripts/replay_ros2_lidar_survey.py runtime/survey_replay_latest.jsonl`)
   and confirm `navigation_pattern.geometry_valid is True` and
   `canonical_fence_model.status == "VALID"` end-to-end.

## Sequencing

1. Land Problem 2 first (turn logic + timeouts) — it is the higher-severity
   infinite-spin and is a prerequisite for Problem 1's re-alignment recovery to
   actually exit a turn.
2. Land Problem 1 (guard-rejection recovery + safety angular passthrough).
3. Add the `TurnTracker` and survey unit tests, then run the full replay gate.
