# Implementation Plan — Collection Nav2 Controller

> Κατάσταση: **draft for review**. Κανένα βήμα δεν αλλάζει survey/RPP behavior
> ή collection runtime πριν περάσει το προηγούμενο gate.

## C0 — Interface and context contracts

**Scope**

- Δημιουργία νέου independent C++ `ament_cmake` package
  `tennis_robot_collection_controller`, χωρίς αλλαγή στο Python package type.
- ROS messages/services στο `tennis_robot_msgs`: context load/reset/safety hold/
  post-action finalize και typed controller telemetry.
- Ορισμός canonical `CollectionExecutionContext`, exact
  `CollectionPathCanonicalizationV1` path byte stream/SHA-256 algorithm και
  typed service/telemetry schemas.
- Pure C++ contract tests: exact hash matching, timestamp exclusion, non-finite
  rejection, immutable context, activation timeout, reset/consumed lifecycle
  και plan/hash-bound safety-hold rejection.

**Gate**

- Generated schemas carry every required context/profile/crossing/configuration
  field; `CollectionPathCanonicalizationV1` is byte-exact and tested; the pure
  lifecycle contract rejects invalid identity/hash/schema, preserves loaded
  context on mismatch, clears only on timeout/reset and consumes terminal
  contexts. Full semantic validation of the generated context occurs in C1
  before motion exists.

## C1 — Dedicated plugin skeleton

**Scope**

- Νέο `nav2_core::Controller` plugin με distinct collection controller ID.
- Load-context service lifecycle, FollowPath hash binding and explicit typed
  failures before motion.
- Finalize service consumes only matching executing context after terminal
  FollowPath result; internal plugin failure consumes context directly.
- Δεν αλλάζονται RPP plugin, survey controller ID or survey BT.

**Tests**

- Plugin unit tests for idle-only load, exact hash, mismatch/no-context failure
  and single-use context.

**Gate**

- No collection motion command is produced without exact loaded context.

## C2 — Profile enforcement and telemetry

**Scope**

- Local schema completion: immutable controller-wide `CollectionControllerTuning`
  (`lookahead_distance_m`, `max_angular_velocity_rad_s`,
  `progress_projection_window_m`, `crossing_speed_window_m`,
  `terminal_progress_tolerance_m`), all required and without defaults; immutable
  `terminal_pose` completes terminal semantic validation.
- C2A ROS-free C++ tracking core: monotonic bounded projection, forward-only
  lookahead pursuit, curvature/command generation, tube/progress/run-in/run-out
  checks, crossing-window speed verdict and terminal-progress detection.
- Monotonic progress, tube, curvature, no-reverse/no-rotate checks.
- Crossing min/max speed hard enforcement, nominal telemetry warning,
  run-in/run-out validation and typed telemetry publication.
- Dynamic safety resume validation; insufficient run-in fails without backtrack.
- C2B plugin bridge, semantic context validation, typed state topic and
  terminal-gated Finalize success.

**Tests**

- C2A deterministic C++ unit tests for nominal forward tracking, curvature,
  crossing hard speed bounds, nominal-only deviation, tube/progress/run-in,
  hold/resume, terminal completion and no reverse/rotate command.
- Subsequent runtime tests for every hard failure reason and immutable telemetry
  identity, semantic context rejection, early terminal rejection and valid
  terminal finalization.

**Gate**

- Test evidence proves all required profile fields are enforced or explicitly
  failed; no implicit speed/profile fallback exists.

## C3 — Isolated Nav2 integration

**Scope**

- Add collection-only controller plugin registration, configuration and an
  isolated direct FollowPath integration harness.
- Prove it has no collection BT, BackUp, Spin, recovery or automatic replan.
- Do not wire `CollectionPathFollower`, executor or scan navigation yet.
  Scan navigation remains existing separate concern.

**Tests**

- ROS integration: context load, one FollowPath, profile telemetry, hash
  mismatch failure, no reverse/rotate/recovery commands.

**Gate**

- Survey RPP behavior is regression-tested unchanged and collection execution
  is isolated.

## C4 — Gazebo enforcement evidence

**Scope**

- Controlled Gazebo runs for min/max/nominal speed, run-in/run-out, tube,
  pause/resume and terminal telemetry evidence.

**Gate**

- Measured telemetry proves the approved execution contract before collection
  runtime cutover.

## C3.5 / C4A — Python Nav2 adapter and runtime wiring

**Scope:** deterministic executable `CollectionRoutePlan` to final Path and
immutable context; direct Load→FollowPath→hold/resume→Finalize→Reset; adapter
owns cancel and typed telemetry forwarding. No BT, RPP fallback, per-ball goals,
controller_node, legacy mission or Gazebo acceptance. Timeout/reset failure is
terminal and leaves adapter unavailable until explicit upper-level recovery.

**Tests:** pure Python conversion/lifecycle, real controller-server ROS
integration, and isolation assertions.

**Gate:** every terminal path either resets successfully or exposes typed
unavailable state; C4 remains exclusively Gazebo enforcement evidence.
