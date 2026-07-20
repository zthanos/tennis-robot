# Technical Design — Collection Nav2 Controller

> Κατάσταση: **draft for review**. Δεν αποτελεί build/runtime implementation.

## Components and ownership

```text
CollectionRouteExecutor
  -> CollectionPathFollower (μελλοντική pure view + Nav2 adapter)
       -> LoadCollectionExecutionContext service
       -> SetCollectionSafetyHold / ResetCollectionExecutionContext services
       -> FinalizeCollectionExecutionContext service (post-action terminal only)
       -> FollowPath(controller_id=collection_controller)
            -> CollectionNav2Controller plugin
                 -> typed controller telemetry/state
```

`CollectionPathFollower` θα είναι owner της conversion από immutable
`CollectionRoutePlan` σε canonical FollowPath `Path` και
`CollectionExecutionContext`. Ο controller δεν ξανασχεδιάζει route, δεν
διαβάζει perception και δεν κάνει scan/planning/collector policy.

Το C++ plugin και η canonicalization library ανήκουν στο νέο independent
`ament_cmake` package `tennis_robot_collection_controller`. Το Python package
`tennis_robot` δεν αλλάζει build type. Τα wire messages/services ανήκουν στο
`tennis_robot_msgs`.

## Service contract

Προτείνεται plugin-specific service `LoadCollectionExecutionContext` με request:

```text
CollectionExecutionContext context
```

και response:

```text
bool accepted
CollectionContextRejectionCode rejection_code
string detail  # diagnostic only; ποτέ FSM input
```

`CollectionContextRejectionCode` περιλαμβάνει τουλάχιστον:

```text
controller_not_idle
invalid_context
unsupported_schema
path_hash_invalid
context_already_consumed
```

Το C0 wire schema στο `tennis_robot_msgs` ορίζει
`CollectionExecutionContext` (`context_schema_version`, `plan_id`,
`path_sha256`, `context_activation_timeout_s`, ordered
segments/crossings/profiles, immutable `controller_tuning`, `terminal_progress_s`,
`terminal_pose`, `configuration_snapshot_json`), μαζί με typed profile verdict/controller state.
`configuration_snapshot_json` είναι UTF-8 canonical JSON από το immutable
domain `CollectionRouteConfiguration.to_dict()`, με lexicographically sorted
object keys και separators `,`/`:`· missing, invalid UTF-8 ή non-canonical
serialization απορρίπτεται από το context validator.
Η canonical domain representation παραμένει immutable και ROS-free μέχρι το
adapter boundary.

Το C0 προσθέτει επίσης:

```text
ResetCollectionExecutionContext() -> typed acceptance/rejection
SetCollectionSafetyHold(plan_id, path_sha256, hold) -> typed acceptance/rejection
FinalizeCollectionExecutionContext(plan_id, path_sha256, action_outcome)
  -> typed acceptance/rejection
```

Safety hold είναι plan/hash-bound: `hold=true` εκδίδει zero command και παγώνει
progress. `hold=false` περνά μόνο από monotonic/tube/run-in validation.
Finalize γίνεται μόνο μετά από terminal action result και μόνο για matching
executing context. Internal controller failure καταναλώνει context αμέσως.

## Hash binding and lifecycle

1. Ο follower canonicalizes το flattened path με exact
   `CollectionPathCanonicalizationV1` και υπολογίζει SHA-256.
2. Φτιάχνει context με το ίδιο hash και κάνει successful service load.
3. Στέλνει `FollowPath` με το dedicated collection controller ID.
4. Ο plugin canonicalizes το received path με την ίδια versioned algorithm:
   frame ID UTF-8 with big-endian uint32 length prefix, big-endian uint32 pose
   count, έπειτα x/y/z/qx/qy/qz/qw raw IEEE-754 float64 bits in big-endian
   order. Header timestamp δεν συμμετέχει· non-finite values απορρίπτονται.
5. Αν hash/context δεν ταιριάζουν, το action αποτυγχάνει πριν εκδοθεί motion.
6. Σε terminal success ή failure, εκδίδεται zero velocity, το context γίνεται
   consumed και δεν μπορεί να ξαναχρησιμοποιηθεί μέχρι explicit reset.

Loaded context δέχεται μόνο το επόμενο matching FollowPath μέσα στο required
`context_activation_timeout`. Hash mismatch αποτυγχάνει το action χωρίς να
διαγράφει context· timeout/reset το καθαρίζει.

Δεν υπάρχει implicit context lookup, profile inference από geometry ή fallback
στο survey controller.

## Controller state machine

```text
idle -> context_loaded -> validating_follow_path -> executing
executing -> safety_paused -> executing        (μόνο valid forward resume)
executing -> succeeded | failed
safety_paused -> failed                        (timeout / invalid run-in)
succeeded | failed -> consumed -> idle          (μόνο explicit reset)
```

Το plugin δεν ξεκινά `NavigateToPose`, `ComputePathToPose`, replan, BackUp ή
Spin. Safety pause είναι zero-velocity hold· resume validation χρησιμοποιεί
monotonic progress, tube και remaining run-in του loaded context.

## C2B semantic validation και terminal gate

Το Load service αποδέχεται μόνο πλήρες immutable context: valid canonical JSON,
non-empty/ordered segments και crossings, valid profiles/tuning, terminal
progress μέσα στο route, και finite normalized terminal pose. Στο matching
`setPlan`, η canonical path μετατρέπεται σε cumulative progress και ελέγχονται
segment continuity, crossing membership, terminal progress και XY terminal pose
consistency με το explicit `terminal_progress_tolerance_m`. Path/profile
failure καταναλώνει το context και εκπέμπει typed failure πριν από motion.

Το `Finalize(...succeeded)` απαιτεί last `terminal_ready`, terminal progress
within the same explicit tolerance και valid tube/profile state. Αποτυχία του
gate απαντά `terminal_not_reached` χωρίς να μετατρέψει action success σε route
completion.

## Enforcement and measurements

### Pure C2A tracking core

Πριν από Nav2 runtime wiring, ένα ROS-free C++ core δέχεται canonical flattened
polyline, immutable segment profiles/crossings και immutable
`CollectionControllerTuning`. Υπολογίζει monotonic bounded projection,
lookahead-pursuit curvature, forward-only command, tube/progress/run-in/run-out
measurements και terminal progress. Δεν κάνει TF lookup, service callback,
parameter lookup ή ROS publish.

`progress_projection_window_m` περιορίζει το forward projection search από το
τελευταίο accepted progress. `crossing_speed_window_m` είναι symmetric
path-distance window γύρω από crossing progress. Safety hold επιστρέφει μόνο
zero command και δεν παράγει crossing-speed verdict. Κατά resume ελέγχεται το
remaining run-in χωρίς backtrack.

Command generation χρησιμοποιεί nominal speed μόνο όταν η απαιτούμενη
curvature και `max_angular_velocity_rad_s` επιτρέπουν το ίδιο forward command.
Δεν γίνεται speed clipping κάτω από `min_speed_mps` ή rotate/reverse fallback:
το αποτέλεσμα είναι typed `profile_unenforceable`, `curvature_exceeded`,
`reverse_required` ή `standalone_rotate_required`.

Για κάθε controller cycle ο plugin:

1. προβάλλει την current pose σε monotonic route progress;
2. ελέγχει trajectory tube και active segment profile;
3. υπολογίζει curvature/speed command που δεν απαιτεί reverse ή rotate;
4. στο crossing εκδίδει keyed measurement και verdict;
5. αποτυγχάνει σε hard profile/safety constraint.

`nominal_speed_deviation` εντός hard min/max είναι telemetry event, όχι action
failure. Η κυκλική/kinematic implementation πρέπει να αποδείξει ότι min/max,
run-in/run-out και curvature μπορούν να τηρηθούν πριν από crossing· διαφορετικά
επιστρέφει typed failure.

## Typed telemetry

Προτείνεται `CollectionControllerState` με:

```text
plan_id, path_sha256, lifecycle_state, progress_s,
active_segment_id, active_ball_id, active_crossing_progress_s,
measured_speed_mps, lateral_error_m, heading_error_rad,
ProfileComplianceVerdict, failure_reason
```

`failure_reason` είναι typed enum, τουλάχιστον:

```text
missing_context
path_hash_mismatch
profile_unenforceable
speed_below_min
speed_above_max
run_in_insufficient
run_out_insufficient
curvature_exceeded
trajectory_tube_exceeded
non_monotonic_progress
heading_error_exceeded
reverse_required
standalone_rotate_required
safety_resume_invalid
```

## Isolation from survey Nav2

Στο C3, isolated ROS harness κάνει context load και direct FollowPath action
call με collection controller ID, σε controller-server-only test launch. Δεν
συνδέεται ακόμη Nav2 adapter του `CollectionPathFollower`. Δεν υπάρχει collection BT.
Το υπάρχον RPP parameter block, survey BT, recovery nodes και NavigateToPose
integration μένουν αμετάβλητα.

## C3.5/C4A runtime adapter

Νέος narrow Python ROS adapter υλοποιεί το executor PathFollower boundary,
ενώ ο pure follower παραμένει ROS-free. Flattening διατηρεί order/geometry και
αφαιρεί μόνο exact join poses. Ο adapter owns Load/FollowPath/hold/Finalize/
Reset και cancel; correlated typed telemetry `(plan_id,path_sha256)` προωθείται
read-only. Timeout/reset failure κάνει adapter unavailable μέχρι explicit
higher-level recovery. Δεν χρησιμοποιούνται BT, recovery, RPP fallback ή
per-ball goals.
