# Collection subsystem — simulation freeze and hardware handoff baseline

**Status:** `COLLECTION_SIMULATION_FROZEN_FOR_HARDWARE_HANDOFF`
**Frozen at:** Phase 21 validation, 2026-08-18
**Evidence:** debug log entries #57–#86 in
[`docs/collection-route/collection-route-debug-log-el.md`](../collection-route/collection-route-debug-log-el.md)

This document is the answer to five questions a future session will ask:
what was proven in simulation, what is still unknown, which values are production
contracts, which measurements must be repeated on hardware, and what evidence is
required before reopening a frozen decision.

A guiding principle for the whole document:

> A different physical calibration does **not** reopen the architecture.
> A violated architectural invariant does.

---

## 1. What is frozen

| Subsystem | Classification | Note |
| --- | --- | --- |
| Collection scan (360°, coverage fraction) | `FROZEN_ARCHITECTURE` | `scan.required_coverage_fraction 0.6` is `SIMULATION_CALIBRATION_ONLY` (distributed RGB/depth slop, #34) |
| Ball snapshot / planning input | `FROZEN_ARCHITECTURE` | Immutable frozen snapshot per run; positions never rewritten mid-route |
| Route planner (bounded anytime pass router) | `FROZEN_ARCHITECTURE` | Coverage-first lexicographic objective, exact dominance, budget-monotonic |
| Passes and connectors | `FROZEN_ARCHITECTURE` | Pass/corridor is the search primitive; clustering only orders exploration |
| Execution plan (segments, crossings, progress) | `FROZEN_ARCHITECTURE` | Frozen at execution start; never mutated by the executor |
| Map-frame execution | `FROZEN_ARCHITECTURE` | See §4 — the single most important correction of this campaign |
| Tracking core mathematics | `FROZEN_ARCHITECTURE` | ROS-free, frame-agnostic, self-consistency verified to 7×10⁻¹⁸ m |
| Frame contract at the ROS boundary | `FROZEN_ARCHITECTURE` | Mismatched/unnamed frames throw, never compare |
| Collection crossing contract | `FROZEN_ARCHITECTURE` | Capture-grade geometry required at the ball |
| Capture heading requirement | `FROZEN_PRODUCTION_DEFAULT` | `0.15 rad`, unchanged all campaign |
| Trajectory tube (inner controller gate) | `FROZEN_PRODUCTION_DEFAULT` | `0.20 m`, recalibrated in Phase 14 against the true map-frame error |
| Progress projection | `FROZEN_ARCHITECTURE` | Forward-only accepted progress; kinematically bounded regression evidence |
| Re-anchoring handling | `FROZEN_ARCHITECTURE` | One-update tube deferral, bounded by construction |
| Confirmation attribution | `FROZEN_ARCHITECTURE` | Authoritative runtime event, persisted verbatim; no offline heuristic |
| Follow-up scan/replan | `FROZEN_ARCHITECTURE` | Phase 20B eligibility, §6 |
| Run budget | `FROZEN_PRODUCTION_DEFAULT` | `follow_up.max_total_runs = 2` |
| Audit / trace / evaluator pipeline | `FROZEN_ARCHITECTURE` | One audit + one trace per executed route, paired by `plan_id` |
| Wheel-odometry translation scale | `OPEN_FOR_HARDWARE_CALIBRATION` | No Gazebo coefficient is transferable (#82) |
| Localization correction magnitude | `KNOWN_LIMITATION` | 0.24–0.37 m erroneous jumps remain (#79) |
| Collector ingestion rate | `KNOWN_LIMITATION` | ~77% given presentation (#71); a measurement, not a specification |

Nothing in this table is called *solved* merely because simulation tolerated it.

---

## 2. Canonical runtime

**Native Ubuntu 24.04 + ROS 2 Jazzy + native Gazebo/GPU, launched by
`./run_native.sh`.** Docker/Humble is obsolete: its Nav2 parameters carry Jazzy
plugin names, so `planner_server` never configures and every route hangs before
moving (#65). Do not add Humble compatibility or distro-aware Nav2 configuration.

### Operational preflight (learned the hard way, #69 and #86)

Before every measured run:

1. terminate the full process set — `install_jazzy`, `twist_mux`, `nav2_*`,
   `slam_toolbox`, `gz sim`, and the project nodes;
2. verify no stale process survives;
3. verify **exactly one** `/clock` publisher (four publishers caused
   "Detected jump back in time", which broke TF and every route, #69);
4. verify no backward-time events in the stack log;
5. verify Nav2 lifecycle nodes `active [3]`;
6. verify a valid `map → base_footprint` transform;
7. remove **only project-owned** stale `/dev/shm/fastrtps_*` resources — 355 had
   accumulated by the end of Phase 20B and `planner_server` stopped starting
   (#86); after clearing them, six of six missions came up first time;
8. never teleport the robot after localization starts.

If Nav2 does not become healthy, the run is invalid. Do not debug collection
behaviour in a degraded environment.

---

## 3. Frozen production parameters

Verified against `ros2_ws/src/tennis_robot/config/collection_route.yaml` at
freeze time.

| Parameter | Value | What it protects |
| --- | --- | --- |
| `planning.default_execution_profile.max_lateral_error_m` | **0.20 m** | **Inner controller gate.** Enforced per sample by the C++ tracking core; exceeding it aborts the route. |
| `safety.trajectory_tube_radius_m` | **0.20 m** | **Outer executor contract.** Feeds `trajectory_tube_ok`, consumed only as a *resume precondition after a safety pause*. |
| `planning.default_execution_profile.max_heading_error_rad` | **0.15 rad** | Capture-grade alignment at the ball. Unchanged all campaign. |
| `planning.default_execution_profile.required_entry_m` | **0.8 m** | Heading-alignment allowance at the entry of a capture segment. Moves *when* the capture gate starts applying, not how strict it is. |
| `planning.default_execution_profile.required_run_in_m` | **1.0 m** | Straight run-in the planner guarantees before the first crossing; keeps the entry allowance ending before any ball. |
| `planning.connector_max_heading_error_rad` | **0.5 rad** | Transit connectors are not capture motion; pure pursuit legitimately leads the tangent on curves. |
| `planning.default_execution_profile.max_curvature_per_m` | **2.5 1/m** | Commanded-curvature ceiling; crossings are straight so capture accuracy is unaffected. |
| `follow_up.max_total_runs` | **2** | The only guard against `abort → rescan → abort` cycling. |

### The two 0.20 m values are not duplicates

They are different invariants at different layers and were verified independently
in Phase 14 §1:

* `max_lateral_error_m` is the **inner gate**: the tracking core compares its
  pose against the path every sample and *aborts* on violation. This is what
  caused the Phase 12/13 aborts.
* `trajectory_tube_radius_m` is the **outer contract**: the Python executor port
  computes `trajectory_tube_ok` and uses it **only** in
  `_can_resume_after_pause`. It has never aborted a route.

They agree numerically today by deliberate choice, not by shared implementation.
Changing one does not change the other, and a future change must state which
layer it means.

### Why 0.20 m and 0.8 m

Both were derived from measurement, not tuned to make a run pass (#74):

* the true map-frame envelope is **0.009 m median / 0.043 m max on straight-pass
  cores** and **0.063 m median / 0.134 p90 / 0.214 max on connectors**; at
  0.10 m the gate rejected 24.6% of samples including straight passes, at 0.20 m
  only 0.1%, all on connectors;
* passes entered from a connector start at **0.30–0.32 rad** of heading error and
  are still at 0.19–0.24 rad after 0.2 m, but reach 0.02–0.04 rad by 0.5 m — so
  0.8 m is sufficient with margin and still ends before the first crossing.

---

## 4. Frozen frame architecture

| Object | Frame |
| --- | --- |
| Planner, snapshot, ball geometry | `map` |
| Execution path | `map` |
| Tracker reference point | `map` |
| Tracker pose | transformed **into `map` at every update** via TF, at the pose's own stamp |
| Odometry | stays `odom`; it is an **estimator input**, never the frame the corridor is frozen in |

### The rejected architecture

```
map plan  →  one-time map→odom transform at route start  →  frozen odom corridor
```

**Obsolete and must not return.** Measured consequence (#72): the corridor was
baked into `odom` once and never re-anchored, so every subsequent SLAM correction
slid the physical corridor away from the map-anchored balls — 0.13 to 0.44 m of
displacement against a 0.205 m funnel half-width. The tracker reported 0.012 m of
cross-track while the corridor was a third of a metre off the ball.

### Invariant

> A tracking comparison between a pose and a path carrying different or unnamed
> frame ids is invalid and must fail explicitly.

Enforced in `CollectionNav2Controller::pose_in_plan_frame()`; an unnamed frame is
not agreement. Locked by three gtests.

---

## 5. Frozen execution robustness contracts

### Re-anchoring (Phase 16)

A map-pose step larger than the robot could physically have travelled
(`plan_max_speed_mps × elapsed_s + 0.10 m`) defers the trajectory-tube verdict
**for one update only**.

It must not: widen the tube, smooth the pose, alter the path, freeze `map→odom`,
suppress any other gate, or grant a second consecutive deferral. A deferral
cannot follow a deferral, so repeated discontinuities can never compound into an
open-ended grace period. Sustained error still aborts on the next update — pinned
by test.

The 0.10 m margin is measured, not chosen: across 7597 updates the pose step
never exceeded 0.111 m, and the smallest observed discontinuity was 0.209 m.

### Progress projection (Phase 17A)

Accepted execution progress is **forward-only by construction** — the bounded
projection is clamped to `[last_progress, last_progress + window]` and never
moved backward, including during the original defect.

The defect was in the *evidence* the regression check consulted: a global
nearest-point search admitted a candidate 9.81 m behind on a route that returned
near itself. A raw candidate now counts as evidence of backward motion only if it
lies inside the physically reachable neighbourhood of the accepted progress.

> The known self-near false-positive mechanism was corrected. A later
> `non_monotonic_progress` termination remains intentionally terminal and
> unclassified unless new evidence justifies reopening it.

One such event was observed in Phase 21 (real_scan r3) and correctly terminated
the mission.

---

## 6. Frozen follow-up / replan semantics (Phase 20B)

A **classified-skippable** tracking abort causes:

```
abort current route → navigate to scan pose → 360° scan → fresh plan → execute remaining balls
```

It must **not**: continue on the old displaced corridor, skip a segment index and
assume the next connector is safe, invent a connector inside the executor,
replan already-confirmed balls, or exceed `max_total_runs`.

### Eligibility, read from the implementation

`collection_route_executor.py::_is_skippable_tracking_abort` requires **all** of:

* `outcome is ExecutorState.ABORTED_TRACKING`;
* `reason is ExecutorReasonCode.PATH_FAILED`;
* the detail contains one of `_SKIPPABLE_TRACKING_DETAILS =
  ("trajectory_tube_exceeded", "heading_error_exceeded")`.

**Follow-up eligible:** `trajectory_tube_exceeded`, `heading_error_exceeded`
(both only with reason `PATH_FAILED`).

**Terminal — never eligible:** `SAFETY_RESUME_INVALID` (it also ends in
`ABORTED_TRACKING`, which is exactly why the reason code is checked as well as
the state), `ABORTED_SAFETY` / `SAFETY_TIMEOUT`, `ABORTED_COLLECTOR` and every
collector fault, `ABORTED_SCAN` / `SCAN_FAILED` / `NAVIGATION_FAILED` /
`NAVIGATION_UNAVAILABLE`, `ABORTED_PLANNING` / `PLANNING_FAILED`,
`PROFILE_UNENFORCEABLE`, and any unrecognised detail — including
`non_monotonic_progress` and `curvature_exceeded`.

Confirmed balls are preserved physically (they are gone from the court) and were
verified never re-planned: 0 run-2 targets within 0.25 m of a run-1 confirmation
across all multi-run missions.

---

## 7. Simulation acceptance baseline (Phase 21)

**6 / 6 valid missions, first attempt each.** Environment health identical across
all six: 0 residual processes, 1 `/clock` publisher, 0 time jumps.

| mission | planned | confirmed | retained | runs | final state |
| --- | ---: | ---: | ---: | ---: | --- |
| two_passes r1 | 4 | 2 | 2 | 1 + rescan | `completed_no_targets` |
| two_passes r2 | 4 | 2 | 2 | 1 + rescan | `completed_no_targets` |
| two_passes r3 | 4 | 1 | 1 | 1 + rescan | `completed_no_targets` |
| real_scan r1 | 16 | 3 | 4 | 2 | `aborted_tracking` |
| real_scan r2 | 11 | 0 | 0 | 2 | `completed` |
| real_scan r3 | 11 | 6 | 6 | 2 | `aborted_tracking` |
| **total** | **50** | **14** | **15** | | |

### Behavioural evidence that matters more than the totals

* every eligible first-run abort triggered a follow-up (5 of 5);
* confirmed balls were never re-planned (0 collisions in 3 multi-run missions);
* the run budget held — all three real_scan missions stopped at 2 runs;
* an unclassified abort (`non_monotonic_progress`) stayed terminal;
* every run kept an independent `plan_id`, audit artifact and trace.

The clearest single result: **real_scan r1** aborted run 1 with 1 of 9 confirmed,
then run 2 planned the 7 unresolved balls and added 2 more — a mission that would
previously have ended at 1 confirmed finished at 3.

### 14/50 is not a mechanism effectiveness figure

That denominator mixes four different subsystem responsibilities, and they must
stay separate (#71):

| stage | meaning | responsible subsystem |
| --- | --- | --- |
| `planned` | crossings the planner scheduled | planner |
| `reached` | crossings the route actually got to | execution / tracking |
| `presented to mouth` | ball inside the 0.205 m intake envelope | tracking + localization + perception |
| `confirmed ingestion` | collector reported an attributed collection | collector mechanism |
| `basket retained` | ball physically in the basket | collector mechanism |

Phase 13 measured **6 of 6 balls reached were presented inside the mouth**, and
Phase 10 measured **77% ingestion given presentation**. Quoting 28% as a
mechanism figure would conflate all five stages.

---

## 8. Known open issues (recorded, not reopened)

| Issue | Evidence | Status |
| --- | --- | --- |
| **Localization discontinuities** 0.24–0.37 m, 5 of 6 late ones moving the estimate *away* from truth | #76, #79 | Downstream effect is bounded by the one-update deferral, **not eliminated**. Not a solved localization problem. |
| **Wheel odometry** translation scale/slip not session-stable (straight ratio moved 0.9271 → 0.9005 between identical sessions) | #81, #82 | **No Gazebo-derived effective radius or velocity scale transfers to hardware.** |
| **Collector ingestion** ~77% given physical presentation | #71 | An effectiveness *measurement*, not a frozen mechanical specification. |
| **`non_monotonic_progress`** terminal event observed after the Phase 17A fix | #86 | Known, intentionally terminal, unclassified. |
| **Confirmation `measured_speed_mps`** reads 0.0 in 27/28 events while progress advances at 0.35 m/s | #71 | Telemetry field unreliable; the value is available from crossing telemetry. |
| **Beam edges never recorded**, so the evaluator can never conclude `EXECUTED_CROSSING_NOT_COLLECTED` | #71 | Instrumentation gap; swept-but-unconfirmed balls fall to `OBSERVATION_UNCERTAIN`. |
| **Terminal `trajectory_tube_exceeded`** 2–5 cm before completion | #74, #76 | Part of its mystery *was* the failure-telemetry defect (now fixed); whether a distinct bug remains is unproven. |
| **Environment**: FastDDS `/dev/shm` accumulation over long sessions | #86 | Covered by the §2 preflight. |

---

## 9. Simulation-only assumptions — must not transfer silently

| Quantity | Hardware action | Why |
| --- | --- | --- |
| Gazebo wheel–ground friction / slip | `MEASURE` | Clay differs entirely; slip will be larger and surface-dependent |
| Effective rolling scale (~0.90–0.93 in sim) | `CALIBRATE` | Not session-stable even in simulation (#82) |
| Simulated motor response / velocity tracking | `REVALIDATE` | Real drivetrain has inertia, deadband, saturation |
| Gazebo truth poses (`/sim/robot_true_pose`) | *not available* | Every truth-based metric needs a physical substitute (tape measure, external reference) |
| Simulated IR beam timing | `REVALIDATE` | Real IR has different rise/fall and noise |
| Simulated camera calibration and noise | `CALIBRATE` | Real intrinsics/extrinsics and lighting |
| Simulated LiDAR noise and range | `REVALIDATE` | RPLIDAR C1 real returns, reflectivity, sunlight |
| Collector contact/friction behaviour | `REVALIDATE` | Sim contact model is not the real funnel/intake |
| Absolute ingestion percentages (77%) | `MEASURE` | Bench measurement on the real mechanism (H5) |
| SLAM correction distribution (median 0.030 m, 2.7% >0.15 m) | `MEASURE` | Real scan geometry, real drift |
| `scan.required_coverage_fraction 0.6` | `REVALIDATE` | Was lowered for distributed RGB/depth timing slop |

---

## 10. Hardware bring-up order (H1–H8) — defined, not executed

Each stage isolates physical variables before the next depends on them.

| Stage | Purpose | Proves |
| --- | --- | --- |
| **H1 — Base motion and safety** | No collection route | Motor direction, encoder direction, wheel velocity, emergency stop, `/cmd_vel` behaviour, commanded stop, forward/reverse, left/right steering |
| **H2 — Odometry + IMU on the real surface** | Physical characterization | Straight distance error, left/right rotation error, gyro behaviour, wheel slip, repeatability, speed and turn dependence. Hardware calibration is derived **only** from these |
| **H3 — LiDAR / localization** | Frames before behaviour | Scan frame, TF chain, map localization, `map→odom` correction magnitude and rate, stationary/straight/turning stability. **Do not compensate localization problems in the collection controller** |
| **H4 — Perception geometry** | Ball localization | `ball truth → camera/depth estimate → map position`; longitudinal and lateral error with uncertainty |
| **H5 — Collector bench** | No navigation | Present balls at controlled offsets and speeds; measure presentation offset, entry, confirmation, retention, failure modes. Establishes the real "ingestion given presentation" |
| **H6 — Robot-driven presentation** | Straight controlled passes over known balls | Separates *tracking/presentation* failure from *ingestion* failure |
| **H7 — Minimal autonomous collection** | Only after H1–H6 pass | 1 ball → 3 balls on one pass → two passes + connector → small real court scan/replan |
| **H8 — Hardware collection regression** | The hardware equivalent of the reduced simulation regression | Full mission behaviour |

Do not jump from electrical bring-up to a full `real_scan` mission.

---

## 11. Hardware acceptance gates — defined before testing

Marked `BASELINE_REQUIRED` wherever no defensible number exists yet; inventing
one would be tuning against an anecdote.

| Metric | Type | Gate |
| --- | --- | --- |
| Emergency stop halts motion | `SAFETY GATE` | Must stop within one control period, from any state (H1) |
| Commanded stop → zero motion | `SAFETY GATE` | No creep (H1) |
| Motor/encoder direction agreement | `SAFETY GATE` | Sign-correct on all four wheels (H1) |
| Pose/path frame agreement | `ARCHITECTURAL GATE` | Mismatched or unnamed frames must fail explicitly — never compare |
| Accepted progress monotonic | `ARCHITECTURAL GATE` | Never decreases |
| Confirmed ball never re-planned | `ARCHITECTURAL GATE` | Zero occurrences |
| Run budget respected | `ARCHITECTURAL GATE` | Mission stops at `max_total_runs` |
| Skippable vs terminal abort classification | `ARCHITECTURAL GATE` | Matches §6 exactly |
| Independent plan/audit/trace per run | `ARCHITECTURAL GATE` | One of each, paired by `plan_id` |
| Commanded vs actual straight distance | `CALIBRATION TARGET` | `BASELINE_REQUIRED` — H2 straight runs on clay |
| Odometry drift per metre | `CALIBRATION TARGET` | `BASELINE_REQUIRED` — H2 |
| Heading error after a commanded turn | `CALIBRATION TARGET` | `BASELINE_REQUIRED` — H2 |
| Localization correction magnitude | `PERFORMANCE TARGET` | `BASELINE_REQUIRED` — H3; sim reference: median 0.030 m, 2.7% above 0.15 m |
| Localization discontinuity rate | `PERFORMANCE TARGET` | `BASELINE_REQUIRED` — H3 |
| Ball localization lateral error | `PERFORMANCE TARGET` | `BASELINE_REQUIRED` — H4; must be well inside the 0.205 m intake half-width |
| Straight-pass cross-track | `PERFORMANCE TARGET` | Sim achieved 0.009 m median / 0.043 m max; hardware `BASELINE_REQUIRED` |
| Heading error at ball crossing | `ARCHITECTURAL GATE` | Must satisfy the unchanged 0.15 rad capture gate |
| Presentation rate (ball inside the mouth when reached) | `PERFORMANCE TARGET` | Sim reference 6/6 in Phase 13; hardware `BASELINE_REQUIRED` |
| Ingestion given presentation | `PERFORMANCE TARGET` | Sim reference ~77%; hardware `BASELINE_REQUIRED` from H5 |
| Confirmation attribution correctness | `ARCHITECTURAL GATE` | Every confirmation attributable, `unassigned` rate reported explicitly |
| Retention (ball stays in the basket) | `PERFORMANCE TARGET` | `BASELINE_REQUIRED` — H5 |
| Autonomous mission completion | `PERFORMANCE TARGET` | `BASELINE_REQUIRED` — H7/H8 |

---

## 12. Preserved regression suite

The smallest set that must keep passing after any hardware-driven change. It
exists to stop a calibration change silently breaking a contract.

| Area | Tests |
| --- | --- |
| Frame semantics | `tests/test_collection_execution_anchoring.py`, `tests/test_frame_diagnosis_analysis.py`, gtest `test_collection_nav2_controller_runtime` (frame-contract cases) |
| Tracking core / gates / progress | gtest `test_collection_tracking_core` (48 tests: gates, re-anchoring, projection continuity, truthful failure telemetry) |
| Follow-up behaviour | `tests/test_collection_route_executor.py` (classification), `tests/test_collection_execution_trace_integration.py` (session-level continuation, budget) |
| Confirmation attribution | `tests/test_collection_execution_trace_integration.py`, `tests/test_collection_execution_recorder.py` |
| Planner/executor identity | `tests/test_collection_executor_node_factory.py`, `tests/test_collection_execution_context_builder.py`, gtest `test_collection_path_canonicalization` |
| Capture-gate semantics | gtest `test_collection_tracking_core` (entry allowance, capture heading, run-in invariant) |
| Route quality | `tests/test_collection_route_real_scan_quality.py` |

Run the full Python suite plus the five gtest binaries. Do **not** rerun the
historical campaign for every change; rerun the six-mission Phase 21 protocol
only when execution semantics change.

---

## 13. Hardware handoff matrix

| Capability | Simulation evidence | Frozen? | Hardware dependency | First hardware test | Reopen software when |
| --- | --- | --- | --- | --- | --- |
| Route planning (coverage-first) | 10/10 coverage, 45.03 m vs 55.92 m baseline, deterministic | `FROZEN_ARCHITECTURE` | None | H7 | Planner returns zero-route on a solvable layout |
| Pass/connector decomposition | 65/67 crossings on passes, 2 on connectors | `FROZEN_ARCHITECTURE` | None | H7 | A physically executable route cannot be expressed |
| Map-frame execution | Presentation error 0.22–0.40 m → 0.01–0.15 m (#74) | `FROZEN_ARCHITECTURE` | TF chain correctness | H3 | Pose and path frames disagree anywhere |
| Frame contract enforcement | 3 gtests, live `(map, map)` at 100% of updates | `FROZEN_ARCHITECTURE` | None | H3 | An unnamed/mismatched frame is ever compared |
| Trajectory tube 0.20 m | 0.1% of samples exceed it, all on connectors | `FROZEN_PRODUCTION_DEFAULT` | Real tracking envelope | H6 | Straight-pass core error approaches the gate |
| Capture heading 0.15 rad | Held at 0.0001–0.0508 rad at every crossing | `FROZEN_PRODUCTION_DEFAULT` | Real tracking | H6 | Capture geometry proves unachievable on hardware |
| Entry allowance 0.8 m | Entry 0.30 rad → 0.005 rad at the ball | `FROZEN_PRODUCTION_DEFAULT` | Real convergence rate | H6 | Convergence measurably slower on clay |
| Re-anchoring deferral | 0 masked failures; sustained error still aborts | `FROZEN_ARCHITECTURE` | Real correction distribution | H3 | Correction sizes make one update structurally insufficient |
| Progress projection | 0 false regressions in 7 runs | `FROZEN_ARCHITECTURE` | None | H7 | Accepted progress ever moves backward |
| Follow-up replan | 5/5 eligible aborts continued; budget held | `FROZEN_ARCHITECTURE` | None | H7 | A confirmed ball is re-planned, or the budget is exceeded |
| Confirmation attribution | 28 runtime = 28 persisted, 15/15 runs | `FROZEN_ARCHITECTURE` | Real IR timing | H5 | Attribution becomes unreliable or frequently `unassigned` |
| Audit/trace pipeline | One audit + one trace per route, paired by `plan_id` | `FROZEN_ARCHITECTURE` | None | H7 | A route executes without its own artifacts |
| Wheel odometry scale | Not session-stable (#82) | `OPEN_FOR_HARDWARE_CALIBRATION` | Surface, tyres, load | H2 | — (calibration, not architecture) |
| Localization quality | 0.24–0.37 m erroneous jumps | `KNOWN_LIMITATION` | Real SLAM behaviour | H3 | Corrections routinely exceed what the deferral bounds |
| Collector ingestion | ~77% given presentation | `KNOWN_LIMITATION` | Real mechanism | H5 | — (measurement, not architecture) |
| Perception ball localization | Planner corridors centred to 0.0000 m on snapshot positions | `FROZEN_ARCHITECTURE` (contract) | Real camera/depth | H4 | Ball position error approaches the intake half-width |

---

## 14. Repository state at freeze

**Production configuration diff — two values only, both Phase 14:**

```
planning.default_execution_profile.required_entry_m      0.2 → 0.8
planning.default_execution_profile.max_lateral_error_m   0.1 → 0.2
```

(The same file also carries planner-search settings from earlier approved phases:
`max_search_expansions`, `maximum_planning_time_s`, `successor_batch_*`,
`cluster_*`/`maximum_macro_*`.)

**Verified absent from production:**

* no `COLLECTION_DIAGNOSTIC_TUBE_M` override or any remnant of it;
* no Gazebo-derived coefficient (`0.9276`, `0.0788`, effective radius, odometry
  scale) anywhere in `ros2_ws/`;
* `controllers.yaml` and the URDF wheel geometry are **unmodified**
  (`wheel_radius 0.085` in both).

**Production source changes** (behaviour): map-anchored execution and the frame
contract, one-update re-anchoring detection, progress-projection continuity,
truthful failure telemetry, follow-up eligibility, plus the diagnostic fields on
`CollectionControllerState`.

**Diagnostics and tooling** (no runtime effect):
`scripts/sim_debug/record_frame_diagnosis.py`,
`analyze_frame_diagnosis.py`, `characterize_wheel_odometry.py`,
`collection_execution_report.py`.

### Test results at freeze

* **Python: 622 passed, 3 skipped**
* **C++: 48 + 8 + 5 + 7 = 68 passed**, 2 skipped (parity fixtures)

---

## 15. Reopening a frozen decision

| Change | Reopens the architecture? |
| --- | --- |
| A different wheel calibration on clay | **No** — expected; that is what H2 is for |
| Different localization correction magnitudes | **No**, unless they exceed what the one-update deferral bounds |
| Lower ingestion rate on the real collector | **No** — a measurement, addressed mechanically |
| Different perception noise | **No**, unless ball error approaches the 0.205 m intake half-width |
| Pose and path compared in different frames | **Yes** — architectural invariant violated |
| Accepted progress moving backward | **Yes** |
| A confirmed ball re-planned in a follow-up | **Yes** |
| Follow-up exceeding `max_total_runs` | **Yes** |
| A ball crossed while outside capture-grade geometry | **Yes** |
| A route executing without its own audit/trace | **Yes** |
