# Court Knowledge Model Specification

> **As-built, LiDAR-first.** This document defines *what* the Court Knowledge
> Model is and *when* it is considered done. The *how* (coverage controller,
> extraction algorithms, output schema) is specified in
> [`court-survey-v2-spec-el.md`](court-survey-v2-spec-el.md), which is the
> authoritative as-built survey spec. The two are kept consistent: this file
> must never re-introduce the obsolete camera-driven perimeter-traversal FSM.

## Purpose

The purpose of the Court Knowledge Model process is to enable the robot to
autonomously discover, understand, and map a tennis court environment before
any operational activities begin.

The Court Knowledge Model process is responsible only for environment
understanding and knowledge acquisition.

It does not perform ball collection, route optimization, coverage planning, or
task execution.

The output of the Court Knowledge Model process is a complete environmental
representation (`court_boundary.json`, schema `court_knowledge_model/v2`) that
can later be consumed by other subsystems.

## Objectives

The Court Knowledge Model process must determine:

- Court orientation and reference frame (the **court frame**, anchored to the
  measured net centre).
- Court dimensions and line geometry (from the net anchor + ITF standard
  dimensions).
- Fence locations (the outer rectangle around the court).
- Run-off clearance between each court line and the surrounding fence.
- Obstacles inside the fences (position + size).

The Court Knowledge Model process creates environmental knowledge only.

## Inputs

### Sensors

#### LiDAR (primary, Phase 1)

The 360° LiDAR is the primary sensor for the Court Knowledge Model. All
**geometry** — anything vertical (fences, net position, posts, obstacles) and
every distance — is measured from the LiDAR. It is used for:

- Net **position/distance** (front LiDAR range → net lock → court frame).
- Fence detection and the fence rectangle fit.
- Obstacle detection inside the fences.
- Distance (run-off) measurement.
- SLAM and localization (slam_toolbox, with conservative loop closure).

#### OAK-D Camera (net confirmation only)

The OAK-D camera is used in Phase 1 for **one purpose only: confirming the net.**
The net lock is triggered when the camera classifies a "net" ahead (via
`/survey/vision`), while the **distance and position of the net come from the
front LiDAR range**, not from the camera. The camera contributes **no geometry**:
painted court lines (invisible to LiDAR) are derived from the net anchor plus ITF
standard dimensions, and fences, obstacles, and all distances come from the LiDAR
alone. Camera-based singles-line refinement is a future extension and is out of
scope for this spec.

## Preconditions

Before the Court Knowledge Model process begins:

- Robot is placed somewhere inside the tennis court.
- LiDAR is operational and calibrated.
- SLAM (slam_toolbox) is running and publishing the `map` frame and TF.
- Battery level above minimum operational threshold.

No prior map is assumed. No knowledge of the environment is available.

## Court Knowledge Strategy

The strategy is **LiDAR occupancy → Court Knowledge Model**. Motion is the
*means*, not the goal: the deliverable is the MEASUREMENT, taken from the
accumulated map rather than from any instantaneous pose. The same code runs in
Gazebo and on the physical robot; differences are expressed only through env
vars / topics.

> **No-fallbacks principle.** There are no silent estimates or defaults. If a
> step lacks data or a structural check fails, the survey **fails loudly** with
> a named reason and does **not** write an invented boundary. The only allowed
> constants are the ITF regulation dimensions.

### Step 1 - Accumulate occupancy

Accumulate `/scan` returns into a map-frame voxel grid as the robot moves. Live
points are streamed to `court_survey_live.json` for the panel.

### Step 2 - Find the net and establish the court frame

Drive forward until the net is confirmed ahead. The **net lock** is triggered by
the OAK-D net classification and takes the net distance from the front LiDAR
range. The lock defines the court frame: origin at net centre, `+x'` = robot→net
direction (length axis), `+y'` = along the net (width axis). Posts (±5.65 m,
doubles) are placed by standard geometry anchored to the measured net centre.

### Step 3 - Deterministic coverage drive

Visit 8 vantage points expressed in the court frame (**not** Nav2 — closed-loop
deterministic drive-to-waypoint on the SLAM pose, since Nav2 proved unstable
run-to-run): drive deep into each half toward the fences, cross the net through
the post→fence gaps, then a **return pass** re-crosses the net and revisits the
near half so slam_toolbox loop closure can align the two halves and complete the
map. A fence-approach stop halts the robot ~1.5 m short of each fence (footprint
aware) for dense mapping without collision.

Measurement **decouples** from driving: it locks on the first successful
extraction, but the robot continues the full path and re-extracts on the
fuller, loop-closed map. The process reaches `DONE` only at the end.

### Step 4 - Extraction (pure functions)

From the accumulated map points and the locked net, compute (offline-testable,
no ROS):

- **Net + posts** → court frame.
- **Fence rectangle** via court-frame histograms (the two extreme dense peaks
  per axis), gated on sufficient coverage beyond both baselines.
- **Court lines** from the net anchor + ITF standard dimensions.
- **Obstacles** inside the fences via clustering, with a smart fence-artifact
  filter (clusters hugging and parallel to a fence are rejected).
- **Distances (run-off)** as position differences on the map.

### Step 5 - Fail-loud validation

Distinguish **structural** failures (fail-loud, abort) from **recoverable**
ones (keep visiting vantage points):

- Structural: net not observed; positive run-off > 12 m (nonstandard/bad fit);
  all vantage points exhausted with no measurement.
- Recoverable: a fence side missing or thin coverage; **negative** run-off (the
  far fence is simply not mapped yet) → continue coverage.

> A fence cannot lie **inside** the baseline; negative run-off means "the real
> fence is not mapped yet" = a coverage problem, not a nonstandard court. This
> classification prevents the survey from aborting before it reaches the far
> fence.

### Step 6 - SLAM map serialization (best-effort)

On completion (`SAVING_MAP` before `DONE`) the node best-effort serializes the
slam_toolbox map (`.posegraph` + `.data` for localization mode, `.pgm` + `.yaml`
occupancy grid) under `runtime/maps/court_<ts>.*`, and records the paths plus
the court frame in `court_boundary.json` as `map_artifact`. This never blocks:
on missing services or timeout the survey still completes with a valid
measurement and `map_artifact.status` = `error`/`pending`.

## Outputs

The Court Knowledge Model process produces `court_boundary.json` (schema
`court_knowledge_model/v2`) containing:

### Net Geometry

- Net centre, length/width axes, the two posts, and post span.

### Court Geometry

- Length, width, doubles flag, and the line geometry in the court frame
  (baselines, service lines, sidelines, centre line).

### Fence Geometry

- Fence corners (map frame) and extents in the court frame.

### Clearance (run-off)

- Distance from each baseline and each sideline to the surrounding fence.

### Obstacles

- For each obstacle inside the fences: id, class, centre, size, point count.

### Map Artifact (best-effort)

- Serialized SLAM map paths + shared court frame, for Nav2 reuse in the
  collection phase.

> Free-movement corridors, entry/exit points, and an explicit accessibility map
> are **not** produced by this process. Downstream navigation derives reachable
> space from the fence rectangle, obstacles, and the serialized occupancy grid.

## Constraints

The Court Knowledge Model process:

- Must not collect balls.
- Must not perform coverage planning for collection.
- Must not assume a navigation matrix exists.
- Must not require a predefined map.
- Must not depend on Collection logic.
- Must not use OAK-D camera evidence for geometry (court lines, fences,
  obstacles, distances). In Phase 1 the camera is used only to confirm the net.
- Must not use simulator/world-specific waypoints: all waypoints are derived
  from the measured net frame, so the same behaviour runs in sim and on the
  physical robot.

The Court Knowledge Model process is responsible only for environmental
knowledge acquisition.

## Error Conditions

Court Knowledge Model creation fails (structural fail-loud) if:

- The net cannot be observed with sufficient points (`net_not_observed`).
- All vantage points are visited without a valid measurement
  (`coverage_incomplete: all vantage points visited`).
- A fitted run-off is nonstandard/implausible (`nonstandard_or_bad_fit`).
- Fewer than the minimum required occupancy points are available.

A failed Court Knowledge Model attempt returns `status = FAILED` with a
structured `failure_reason` and writes no invented boundary. No operational
phase may start from a failed model.

## Definition Of Done

The Court Knowledge Model process is considered complete only when all of the
following are satisfied.

### Court Geometry

- Court frame established from the locked net.
- Court length, width, and line geometry recorded.

### Fence Mapping

- All four fence sides detected and the fence rectangle fitted.
- Run-off distance between each baseline/sideline and the fence measured
  (all positive and plausible).

### Obstacle Mapping

- All detected obstacles inside the fences recorded (position + size), with
  fence artifacts filtered out.

### Map Completeness

- Both halves mapped and loop-closed into a single connected map (net appears
  as a single line, not a drift double-line).

### Validation

- Coverage gate passed (fences densely in view beyond both baselines).
- No structural fail-loud condition triggered.

### Deliverables

- `court_boundary.json` (schema `court_knowledge_model/v2`): net, court, fence,
  distances, obstacles, occupancy, and (best-effort) `map_artifact`.

### Completion Result

```text
status = OK
failure_reason = null
```

If any required condition is not met:

```text
status = FAILED
failure_reason = <named cause>
```

A failed Court Knowledge Model cannot be used by the Collection subsystem.
