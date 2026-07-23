# Πλάνο υλοποίησης: continuous collection route rewrite

> Κατάσταση: **ενεργό implementation plan**. Βασίζεται στο
> [specification](collection-route-rules-el.md) και στο
> [technical design](collection-route-design-el.md). Δεν επιτρέπει fallback
> στην παλιά `collect_route` συμπεριφορά.

## Σκοπός και κανόνας cutover

Υλοποιούμε νέο `collect_route` ως πλήρες rewrite. Η παλιά υλοποίηση μπορεί να
παραμένει προσωρινά μόνο ως αχρησιμοποίητο source file όσο χτίζονται και
ελέγχονται τα pure νέα modules. Στο integration change γίνεται atomic cutover:
ο controller καλεί μόνο τον νέο executor και το παλιό
`collect_route_mission.py` αφαιρείται. Δεν εισάγεται environment flag,
compatibility adapter ή runtime fallback.

Κάθε φάση τελειώνει με συγκεκριμένα tests. Αποτυχία gate σταματά την επόμενη
φάση μέχρι να διορθωθεί η τρέχουσα.

## Επιφάνεια αλλαγής

| Περιοχή | Ενέργεια |
| --- | --- |
| `tennis_robot/collection_route_types.py` | Νέο: immutable domain models, enums και validators |
| `tennis_robot/collection_scan_snapshot.py` | Νέο: 360° scan aggregation, fusion και snapshot finalization |
| `tennis_robot/collection_route_planner.py` | Rewrite: deterministic feasibility, global candidates, scoring, immutable plan |
| `tennis_robot/collection_route_executor.py` | Νέο: lifecycle FSM και orchestration adapters |
| `tennis_robot/collection_path_follower.py` | Νέο: frozen `FollowPath`, segment-profile enforcement και progress |
| `tennis_robot/collection_route_telemetry.py` | Νέο: executed trajectory, crossing metrics και result serialization |
| `tennis_robot/controller_node.py` | Rewrite του `collect_route` wiring στο νέο executor· αφαίρεση per-stop flow |
| `tennis_robot/nav2_lane_navigator.py` | Μόνο adapter API που χρειάζεται ο path follower, όχι mission policy |
| `tennis_robot/ball_map.py` | Μόνο snapshot/telemetry boundary που χρειάζεται· όχι planner policy |
| `config/nav2_params.yaml` | Collection controller profile και verified speed/profile integration |
| `tests/test_*.py` | Αντικατάσταση legacy route tests και νέα unit/integration fixtures |
| `collect_route_mission.py` | Διαγραφή στο atomic cutover |

Πριν από το Phase 2 integration, το stable ROS contract
`tennis_robot_msgs/BallDetection.msg` επεκτείνεται με matched depth acquisition
stamp και row-major 3×3 optical-frame position covariance. Το
`BallDetectionArray.header.stamp` παραμένει RGB acquisition stamp. Το
`has_spatial` παραμένει validity flag: μόνο `has_spatial=true` detection είναι
spatial target και απαιτεί valid νέα metadata· `has_spatial=false` detection
αγνοείται από τον builder. Empty arrays παραμένουν heartbeat. Οι Gazebo και
physical OAK-D publishers συμπληρώνουν τα ίδια fields για spatial detections.
Consumers κάνουν map transform μόνο με TF/localization στο RGB detection
timestamp· δεν επιτρέπεται current-pose fallback.

## Φάση 0 — Baseline και fixtures

**Παραδοτέα**

- Καταγραφή του σημερινού legacy behavior μόνο ως regression evidence, χωρίς
  νέες διορθώσεις σε αυτό.
- Κοινά deterministic fixtures: empty scan, μία ελεύθερη μπάλα, δύο κοντινές,
  net/fence/corner, static obstacle, moving obstacle και all-unreachable.
- Test helper για court frame, poses, uncertainty και fake time.

**Αρχεία tests**

- Νέο `tests/collection_route_fixtures.py`.
- Τα υπάρχοντα `tests/test_collect_route_mission.py` και
  `tests/test_collection_route_planner.py` σημειώνονται προς αντικατάσταση·
  δεν αποτελούν acceptance evidence για το νέο contract.

**Gate**

- Τα fixtures είναι ανεξάρτητα από ROS, Nav2 και Gazebo.
- Κάθε case δηλώνει expected `planning_status` και πλήρη `BallResult` set.

## Φάση 1 — Domain contracts

**Υλοποίηση**

- Δημιουργία `collection_route_types.py` με frozen dataclasses ή ισοδύναμα
  immutable models: `ScanSnapshot`, `SnapshotBall` με canonical map XY και
  PSD map 2×2 covariance, `BallResult`,
  `ExecutionProfile`, `RouteSegment`, `CollectionRoutePlan`.
- Strict enums για `status`, `reason_code`, `planning_status`.
- Validators για ίδιο `map_frame`, monotonic segment progress, πλήρες coverage
  των snapshot ball IDs και valid terminal pose.
- Stable JSON/dict serialization για UI, logs και saved plan artifacts.
- Immutable `configuration_snapshot` με πλήρη perception validation,
  calibration-artifact και snapshot-association groups, χωρίς defaults.

**Tests**

- Νέο `tests/test_collection_route_types.py`.
- Invalid enum ή missing ball result απορρίπτεται.
- Mutation μετά την κατασκευή plan αποτυγχάνει.
- Empty plan και partial plan είναι έγκυρα μόνο με το σωστό status.

**Gate**

- Δεν υπάρχει ROS import στα domain types ή tests.
- Κάθε πλήρες plan περνά validation πριν γίνει executable input.

## Φάση 1.5 — Covariance calibration και perception contract

**Υλοποίηση**

- Επέκταση του canonical `BallDetection.msg` με `matched_depth_stamp` και
  `position_covariance` σύμφωνα με το documented schema.
- Shared producer covariance-model interface με inputs τουλάχιστον range και
  depth-quality metrics. Gazebo και physical OAK-D χρησιμοποιούν διαφορετικά
  versioned calibration artifacts μέσω του ίδιου interface.
- Versioned artifact/configuration με model/version ID, parameters, validity
  domain, calibration evidence reference, date και acceptance metrics.
- Εκτός calibrated domain ή με ανεπαρκές depth quality, ο producer δημοσιεύει
  `has_spatial=false`; δεν κάνει extrapolation ή fallback covariance.
- Προσθήκη calibration identity/parameters και runtime timestamp/covariance
  thresholds στο configuration snapshot.

**Tests**

- Νέο `tests/test_perception_covariance_model.py` για PSD, symmetry, finite
  values, monotonic degradation με range/depth quality και out-of-domain
  rejection.
- Ενημέρωση `tests/test_perception_contract_ros.py` για timestamps, covariance
  contract και TF projection στο RGB timestamp.

**Gate**

- Κάθε `has_spatial=true` detection έχει valid matched depth timestamp και
  calibrated covariance.
- Οι δύο producers publish το ίδιο canonical contract.
- Κανένα out-of-domain ή low-quality input δεν παίρνει spatial fallback.

## Φάση 2 — Snapshot builder

Προϋπόθεση: έχει περάσει η Φάση 1.5 covariance calibration και perception
contract. Ο builder δεν δημιουργεί covariance και δεν δέχεται uncalibrated
spatial fallback.

**Υλοποίηση**

- Δημιουργία `collection_scan_snapshot.py` με explicit states:
  collecting, coverage-check, finalized, failed.
- Δημιουργία reusable `PerceptionSpatialObservationAdapter` boundary. Ο
  controller και ο builder καταναλώνουν τα ίδια
  `AcceptedSpatialObservation`/`SpatialObservationRejection` semantics· δεν
  αντιγράφουν validation ή TF logic. Ο adapter κάνει full camera-optical →
  map XY/2×2 covariance propagation με timestamp-aligned `map_from_camera`
  και explicit Gazebo `localization_xy_covariance` configuration.
- Ο builder δεν κάνει RGB/depth fusion, current-pose read ή TF lookup. Δέχεται
  μόνο adapter outputs και configuration identity. Camera XYZ χρησιμοποιείται
  μόνο για timestamped map XY projection.
- Για το Gazebo MVP το injected `configuration_snapshot` περιέχει το explicit
  provisional planning-safety `localization_xy_covariance_m2 =
  [[0.01, 0.0], [0.0, 0.01]]`. Είναι nonzero conservative engineering budget,
  όχι calibrated localization claim, και δεν υπάρχει default/fallback όταν
  λείπει. Physical snapshot activation παραμένει blocked.
- Εφαρμογή duplicate fusion, confirmation count, scan timeout και scan
  coverage policy. Missing, stale ή inconsistent perception/TF metadata
  επιστρέφουν typed rejected observation και μπορούν να οδηγήσουν σε
  `aborted_scan` όταν δεν επιτευχθεί coverage/confirmation.
- Coverage contract: ο pure builder δέχεται immutable ordered
  `expected_scan_step_ids`, required coverage fraction και timeout. Coverage
  είναι το πλήθος distinct accepted expected step IDs προς όλα τα expected IDs.
  Unknown step είναι typed rejection και δεν προσμετράται. `finalize(now_s)`
  αποτυγχάνει typed σε timeout ή insufficient coverage· empty snapshot είναι
  valid μόνο μετά εντός-time, επαρκές coverage.
- Παράγει μία μόνο immutable snapshot ανά `scan_id`.
- Μετά το finalize, το observation stream μεταφέρεται σε telemetry-only
  monitor για `target_position_invalidated`.

**Tests**

- Νέο `tests/test_collection_scan_snapshot.py`.
- Empty successful scan → empty snapshot, όχι error.
- Timeout/incomplete coverage → scan failure.
- Stale depth, duplicate detections και cross-half targets δεν μπαίνουν στο
  snapshot.
- Matched depth timestamp και covariance propagation προς `map` είναι
  υποχρεωτικά· current-pose transform, missing metadata και TF timestamp
  mismatch απορρίπτονται.
- Tests καλύπτουν required explicit `localization_xy_covariance`, complete
  immutable configuration snapshot και calibration artifact/configuration
  identity. Physical snapshot activation παραμένει blocked χωρίς ξεχωριστό
  verified localization covariance contract.
- Phase 2B runtime-adapter tests επιβεβαιώνουν ότι το profile είναι immutable
  και serialized στο snapshot configuration και ότι ο thin session bridge
  προωθεί μόνο canonical frames και caller-supplied `scan_step_id` προς τον
  pure builder — χωρίς ROS subscription, planner, executor ή legacy mission
  wiring.
- Post-finalize detection δεν αλλάζει snapshot.

**Gate**

- Ο builder επιστρέφει μόνο snapshot ή typed scan failure.
- Δεν γνωρίζει planner, collector, Nav2 ή executor.

## Φάση 3 — Offline global planner

**Υλοποίηση**

- Δημιουργία προσωρινού pure `collection_route_planner_v2.py` που παίρνει μόνο
  `ScanSnapshot`, explicit polygon court/obstacle model και immutable
  configuration snapshot. Δεν καλείται από runtime πριν το atomic cutover και
  δεν είναι fallback.
- Phase 3A deterministic per-ball feasibility: inflated swept-disk keepouts,
  finite heading set, net/fence tangent constraints, positive effective
  capture corridor και entry/exit/run-in/run-out. Το `turn_radius` και κάθε
  connector evaluation ξεκινούν μόνο στο Phase 3B· `deferred` δεν αποφασίζεται
  στη 3A.
- Phase 3B1: directed connector graph από `start_pose` προς κάθε pass entry
  και από pass exit προς κάθε άλλου pass entry. Παράγει μόνο CSC Dubins
  `LSL/RSR/LSR/RSL`; CCC, loops, self-intersection, reverse και standalone
  rotate απορρίπτονται. Required immutable limits είναι minimum radius,
  maximum connector length, maximum individual arc angle και maximum total
  turn. Edge telemetry διαχωρίζει `collision`, `length` και
  `turning_constraint_rejected`; δεν εκδίδεται ακόμη BallResult.
- Phase 3B2: exact bounded DFS/branch-and-bound με required
  `GlobalRouteSearchConfiguration`, valid terminal straight extension και
  lexicographic score coverage → exact operational cost → pass count → stable
  route ID. Το `planning_search_status` διαχωρίζει complete/budget exhaustion/
  failure από το plan outcome και budget exhaustion χωρίς terminal route δίνει
  non-executable `planning_timeout`.
- Phase 3C: pure bounded shared-pass generation από valid 3A common-heading
  nodes, με longitudinal spacing, full entry→exit swept-disk check και
  immutable `SharedPassConfiguration`. Candidate cap exhaustion είναι telemetry
  budget only και δεν μετατρέπεται σε unreachable.
- Phase 3D: `collection_route_planner_v2.py` γίνεται μοναδικό pure composition
  entry point 3A→3C→merge/deduplicate→B1→B2 και επιστρέφει μόνο immutable
  `CollectionRoutePlan`; δεν αλλάζει runtime wiring.
- Candidate generation για κοινά passes και connectors, χωρίς greedy
  per-ball execution policy.
- Lexicographic scoring: maximum covered → minimum operational cost → minimum
  passes. Budget exhaustion δίνει `deferred`, όχι `unreachable`.
- Full `CollectionRoutePlan`, including terminal connector/pose και profile
  feasibility, πριν επιστραφεί.

**Tests**

- Νέο `tests/test_collection_route_planner_v2.py`.
- Phase 3A: free ball, keepout, no-entry, no-exit, net/fence tangent, corner,
  invalid/missing feasibility configuration και continuous swept-disk checking.
- Global ordering, partial/deferred, shared passes, connectors, cost tie και
- budget timeout είναι Phase 3B/3C tests. Το Phase 3B1 προσθέτει pure tests
  για CSC edge, exclusion CCC/loop, length/arc/total-turn rejection,
  obstacle και continuous swept-arc collision, graph directionality και
  immutable edge data.
- Phase 3B2 tests καλύπτουν score priority, directionality, terminal rejection,
  budget exhaustion, deferred-vs-unreachable, deterministic tie-break και
  multi-ball synthetic pass nodes.
- Phase 3C tests καλύπτουν pair/triplet, lateral/tangent/obstacle rejection,
  spacing, deterministic cap/order και B2 selection of a shared node.
- Phase 3D end-to-end pure tests καλύπτουν empty, free/multiple/shared,
  keepout, partial, budget exhaustion, terminal timeout και determinism.

**Gate**

- Planner έχει μόνο domain/geometry/cost dependencies.
- Κάθε snapshot ball έχει ακριβώς ένα independent status/reason result.
- Same fixture/configuration παράγει deterministically το ίδιο plan ID/input
  result (εκτός εάν timestamp είναι εσκεμμένα μέρος του ID).

## Φάση 4 — Executor με fakes

**Υλοποίηση**

- Δημιουργία `collection_route_executor.py` με την FSM του specification.
- Interfaces για scan navigation, snapshot builder, planner, collector,
  safety, follower και telemetry. Οι interfaces είναι injected dependencies.
- Immutable `FollowUpConfiguration(enabled, max_total_runs)` στο persisted
  configuration snapshot· ο αρχικός cycle μετρά στο όριο και δεν υπάρχει
  implicit default.
- Immutable typed dependency results για navigator, scan session, collector
  start/stop, path follower και safety. Τα failure reasons είναι structured
  reason codes.
- Collector start/stop timeout και force-disable semantics.
- Safety pause/resume μόνο με forward progress window· no backtrack όταν δεν
  απομένει valid run-in.
- Bounded follow-up cycle και `completed_no_targets` without collector start.

**Tests**

- Νέο `tests/test_collection_route_executor.py` με fake adapters και fake
  clock.
- Καλύπτει κάθε state transition, planning failure, collector timeout/jam/full,
  safety timeout, valid resume, invalid resume, tracking abort και follow-up
  limit.
- Επιβεβαιώνει ότι stop timeout μετά από terminal route outcome κάνει
  force-disable + `collector_stop_fault` και συνεχίζει σε evaluation/completed,
  ενώ μόνο start/active-route collector faults δίνουν `aborted_collector`.

**Gate**

- Executor δεν αλλάζει `CollectionRoutePlan`.
- Κανένα capture/miss/post-scan detection δεν προκαλεί planner invocation ή
  geometry mutation κατά την execution.

## Φάση 5 — Path follower και Nav2 contract

**Υλοποίηση**

- Δημιουργία `collection_path_follower.py` και adapter πάνω στο υπάρχον Nav2
  integration, μόνο με `FollowPath` για collection.
- Προσθήκη/επιλογή collection controller profile στο `nav2_params.yaml`:
  `use_rotate_to_heading=false`, `allow_reversing=false`.
- Υλοποίηση verified τρόπου επιβολής segment `ExecutionProfile`: dedicated
  controller/plugin ή controller-specific speed-limit contract. Η επιλογή
  καταγράφεται στο plan configuration snapshot.
- Progress projection, trajectory tube check και per-crossing measurements.

**Tests**

- Νέο `tests/test_collection_path_follower.py` με fake FollowPath client.
- Integration test σε ROS/Nav2 environment: ένα `FollowPath`, χωρίς
  per-ball `NavigateToPose`, reverse ή standalone rotate.
- Telemetry test: κάθε crossing έχει measured speed, lateral/heading error και
  profile compliance verdict.
- Το pure follower contract ξεχωρίζει hard min/max speed violation από
  nominal-speed telemetry deviation, με required immutable
  `nominal_speed_warning_tolerance_mps` στο `ExecutionProfile`.
- Κάθε funnel pass carries immutable ordered per-ball `planned_crossings`, ώστε
  shared-pass telemetry/profile verdict να αντιστοιχεί σε κάθε πραγματικό
  crossing και όχι σε κοινό representative point.

**Gate**

- Μετρημένη execution επιβεβαιώνει speed/run-in/run-out και trajectory tube.
- Αν το Nav2 configuration/controller δεν μπορεί να το εγγυηθεί, η φάση δεν
  περνά και δεν γίνεται Gazebo route run.

## Φάση 6 — ROS wiring και atomic cutover

**Υλοποίηση**

- Αντικατάσταση του `collect_route` wiring στο `controller_node.py` με τον νέο
  executor και adapters.
- Καθαρός διαχωρισμός: perception feeds snapshot builder/telemetry; Nav2 feeds
  follower; collector and safety feed executor events.
- Αναβάθμιση runtime status/UI serialization στο νέο
  `CollectionRoutePlan`, ball results και crossing telemetry.
- Διαγραφή `collect_route_mission.py`, αφαίρεση legacy imports, old per-stop
  telemetry fields και tests που περιγράφουν την παλιά συμπεριφορά.

**Tests**

- Ενημέρωση των υπάρχοντων `tests/test_collect_route_mission.py`,
  `tests/test_collection_route_planner.py` και
  `tests/test_ball_map_console_export.py` ή αντικατάστασή τους από v2 tests.
- Static import check: δεν υπάρχει import ή callable fallback προς
  `CollectRouteMission`.
- ROS node smoke test για healthy startup και explicit unhealthy Nav2/collector
  state.

**Gate**

- Μία και μόνη σημασία του `collect_route` στο runtime.
- Το παλιό module δεν υπάρχει και κανένα mode δεν μπορεί να το ενεργοποιήσει.
- Περνούν όλα τα pure tests πριν ROS/Gazebo execution.

## Φάση 7 — Gazebo acceptance

**Scenarios**

1. Empty scan και all-unreachable ως valid `completed_no_targets`.
2. Πολλαπλές ελεύθερες μπάλες: μία frozen route χωρίς per-ball rotation.
3. Κοντινές μπάλες: shared pass, connector ή documented deferred outcome.
4. Μπάλες σε net/fence/corner: tangent crossing ή deterministic unreachable.
5. Μissed capture: route συνεχίζει χωρίς replan.
6. Άνθρωπος/κινούμενο εμπόδιο: stop, valid forward resume ή abort κοντά σε
   crossing.
7. Collector fault/full hopper: `aborted_collector`, χωρίς αλλαγή route.
8. Follow-up enabled/disabled και maximum-run limit.

**Evidence per run**

- Saved `ScanSnapshot` και `CollectionRoutePlan`.
- `ExecutedTrajectory` με progress and crossing metrics.
- Ball result table και status/reason codes.
- Controller/profile evidence, Nav2 action log και collector/safety events.

**Gate**

- Περνούν όλα τα acceptance criteria του specification σε controlled Gazebo
  scenarios. Αποτυχία κατηγοριοποιείται ως planning, tracking, safety ή
  collector evidence πριν αλλάξει κώδικας.

## Πρώτη εκτελέσιμη εργασία

Ξεκινάμε από τη Φάση 1: δημιουργία των immutable domain contracts και των
`tests/test_collection_route_types.py`. Δεν αλλάζουμε `controller_node.py`,
Nav2 parameters ή runtime behavior πριν περάσει αυτό το gate.
