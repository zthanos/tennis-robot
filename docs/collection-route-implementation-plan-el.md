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
  immutable models: `ScanSnapshot`, `SnapshotBall`, `BallResult`,
  `ExecutionProfile`, `RouteSegment`, `CollectionRoutePlan`.
- Strict enums για `status`, `reason_code`, `planning_status`.
- Validators για ίδιο `map_frame`, monotonic segment progress, πλήρες coverage
  των snapshot ball IDs και valid terminal pose.
- Stable JSON/dict serialization για UI, logs και saved plan artifacts.

**Tests**

- Νέο `tests/test_collection_route_types.py`.
- Invalid enum ή missing ball result απορρίπτεται.
- Mutation μετά την κατασκευή plan αποτυγχάνει.
- Empty plan και partial plan είναι έγκυρα μόνο με το σωστό status.

**Gate**

- Δεν υπάρχει ROS import στα domain types ή tests.
- Κάθε πλήρες plan περνά validation πριν γίνει executable input.

## Φάση 2 — Snapshot builder

**Υλοποίηση**

- Δημιουργία `collection_scan_snapshot.py` με explicit states:
  collecting, coverage-check, finalized, failed.
- Εφαρμογή timestamp-aligned detection/depth, duplicate fusion, confirmation
  count, scan timeout και scan coverage policy.
- Παράγει μία μόνο immutable snapshot ανά `scan_id`.
- Μετά το finalize, το observation stream μεταφέρεται σε telemetry-only
  monitor για `target_position_invalidated`.

**Tests**

- Νέο `tests/test_collection_scan_snapshot.py`.
- Empty successful scan → empty snapshot, όχι error.
- Timeout/incomplete coverage → scan failure.
- Stale depth, duplicate detections και cross-half targets δεν μπαίνουν στο
  snapshot.
- Post-finalize detection δεν αλλάζει snapshot.

**Gate**

- Ο builder επιστρέφει μόνο snapshot ή typed scan failure.
- Δεν γνωρίζει planner, collector, Nav2 ή executor.

## Φάση 3 — Offline global planner

**Υλοποίηση**

- Rewrite του `collection_route_planner.py` ως pure planner που παίρνει
  `ScanSnapshot`, court model και immutable configuration snapshot.
- Υλοποίηση deterministic per-ball feasibility: inflated keepouts, tangent
  constraints, heading interval, run-in/run-out, curvature και swept funnel.
- Candidate generation για κοινά passes και connectors, χωρίς greedy
  per-ball execution policy.
- Lexicographic scoring: maximum covered → minimum operational cost → minimum
  passes. Budget exhaustion δίνει `deferred`, όχι `unreachable`.
- Full `CollectionRoutePlan`, including terminal connector/pose και profile
  feasibility, πριν επιστραφεί.

**Tests**

- Νέο `tests/test_collection_route_planner_v2.py`.
- Empty snapshot, all-unreachable, partial/deferred, close balls, shared pass,
  no valid connector, net/fence tangent, corner, cost tie και budget timeout.
- Every planned segment undergoes continuous swept-footprint checking in the
  test double or geometry implementation.

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
