# Specification — Collection Nav2 Controller

> Κατάσταση: **draft for review**. Αυτό το feature ορίζει το runtime controller
> boundary που απαιτείται για την εκτέλεση του ήδη εγκεκριμένου immutable
> `CollectionRoutePlan`. Δεν ενεργοποιεί ακόμη Nav2 integration.

## Σκοπός και όρια

Το `collection-nav2-controller` είναι dedicated Nav2 `nav2_core::Controller`
plugin για collection execution. Υλοποιείται σε νέο ανεξάρτητο C++
`ament_cmake` package `tennis_robot_collection_controller`; δεν αλλάζει το
build type του υπάρχοντος Python `tennis_robot` package. Τα ROS
messages/services ανήκουν στο `tennis_robot_msgs`. Έχει νέο, αποκλειστικό
controller ID και δεν
τροποποιεί ούτε αντικαθιστά το υπάρχον Regulated Pure Pursuit survey
controller. Scan-pose navigation παραμένει ανεξάρτητο `NavigateToPose`
concern.

Ο controller εκτελεί μόνο frozen collection route. Δεν επιτρέπονται reverse,
standalone rotate, backup, spin, recovery maneuver ή automatic replan. Αν το
controller δεν μπορεί να τηρήσει profile ή safety contract, επιστρέφει explicit
failure.

## Immutable execution context

Το bare `nav2_msgs/action/FollowPath` μεταφέρει μόνο `nav_msgs/Path` και δεν
είναι επαρκές transport για collection metadata. Πριν από το `FollowPath`, ο
`CollectionPathFollower` φορτώνει στον collection controller immutable
`CollectionExecutionContext` μέσω explicit plugin-specific service.

Το context περιέχει υποχρεωτικά:

- `plan_id`;
- `context_schema_version`;
- `path_sha256`: SHA-256 του canonical serialized FollowPath path;
- ordered route segments με progress intervals;
- ordered `PlannedCrossing` values με stable identity `ball_id + progress_s`;
- immutable `ExecutionProfile` ανά segment;
- `terminal_progress_s`;
- immutable `controller_tuning`;
- `terminal_pose` στο ίδιο frame με το canonical FollowPath;
- πλήρες immutable `configuration_snapshot`.

Στο C0 wire schema το τελευταίο μεταφέρεται ως
`configuration_snapshot_json`: UTF-8 canonical JSON του domain
`CollectionRouteConfiguration.to_dict()`, με lexicographically sorted object
keys και separators `,`/`:`. Missing, invalid UTF-8 ή non-canonical form
απορρίπτεται.

Δεν υπάρχουν defaults, profile fallback, mutable context ή partial context.
Το service αποδέχεται context μόνο όταν ο controller είναι idle και επιστρέφει
typed success ή typed rejection. Το loaded context πρέπει να ενεργοποιηθεί από
το επόμενο matching FollowPath μέσα σε required
`context_activation_timeout`.

Ο controller δέχεται collection commands μόνο όταν το επόμενο FollowPath path
hash ταιριάζει ακριβώς με το loaded context. Mismatch, missing context, άλλο
`plan_id` ή reuse context μετά από terminal/failure δίνουν explicit failure.
Mismatch δεν διαγράφει loaded context. Timeout ή explicit reset το καθαρίζει.
Terminal/failure context είναι consumed και απαιτεί explicit reset πριν από νέο
load. Κάθε terminal/failure εκδίδει zero velocity πριν αλλάξει lifecycle state.

### Immutable controller tuning

Το `CollectionExecutionContext` περιέχει ακριβώς ένα immutable
`CollectionControllerTuning`, με required fields χωρίς runtime defaults:

- `lookahead_distance_m`: finite `> 0`;
- `max_angular_velocity_rad_s`: finite `> 0`;
- `progress_projection_window_m`: finite `> 0`;
- `crossing_speed_window_m`: finite `> 0`.
- `terminal_progress_tolerance_m`: finite `> 0`.

Το τελευταίο ορίζει symmetric path-distance window γύρω από κάθε
`PlannedCrossing.progress_s`: μόνο μέσα σε αυτό εφαρμόζονται και μετρώνται τα
hard `min_speed_mps`/`max_speed_mps`. Το tuning είναι controller-wide
configuration και δεν αντιγράφεται ανά segment `ExecutionProfile`.
Το `terminal_progress_tolerance_m` χρησιμοποιείται για semantic consistency
μεταξύ terminal progress και terminal pose/path point, καθώς και για terminal
success. Δεν υπάρχει implicit heading tolerance.

Το `ResetCollectionExecutionContext` service καθαρίζει explicit lifecycle
state. Το plugin lifecycle είναι:
`idle -> context_loaded -> validating_follow_path -> executing ->
succeeded|failed -> consumed`.

Το `FinalizeCollectionExecutionContext` service δέχεται `plan_id`,
`path_sha256` και typed `action_outcome` (`succeeded`, `canceled`, `failed`).
Χρησιμοποιείται μόνο από τον `CollectionPathFollower` αφού λάβει terminal
FollowPath action result. Matching executing context γίνεται consumed.
Mismatch ή invalid lifecycle είναι typed rejection. Plugin-internal path/profile
failure καταναλώνει context απευθείας, χωρίς να περιμένει finalize. Το C2 θα
δέσει terminal success με configured terminal progress.

Το `FinalizeCollectionExecutionContext(succeeded)` γίνεται accepted μόνο όταν
το τελευταίο core result είναι `terminal_ready`, το progress βρίσκεται μέσα στο
`terminal_progress_tolerance_m`, και το τελευταίο tube/profile state είναι
valid. Διαφορετικά επιστρέφει typed `terminal_not_reached`: action-level
FollowPath success δεν ισοδυναμεί με collection completion.

### CollectionPathCanonicalizationV1

Το `path_sha256` είναι SHA-256 του exact `CollectionPathCanonicalizationV1`
byte stream, όχι approximate geometry hash. Το stream περιέχει με fixed order:

1. UTF-8 length-prefixed `Path.header.frame_id`, με length ως unsigned
   big-endian uint32;
2. pose count ως unsigned big-endian uint32;
3. για κάθε pose, με σειρά `position.x`, `position.y`, `position.z`,
   `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`, κάθε
   τιμή ως raw IEEE-754 float64 bit representation σε big-endian byte order
   (το signed zero διατηρείται).

Strings είναι UTF-8 με fixed-width length prefix. Non-finite float και invalid
UTF-8/frame input απορρίπτονται. Το `Path.header.stamp` αγνοείται. Δεν υπάρχει
"same geometry approximately" matching.

## Profile and safety enforcement

Ο controller επιβάλλει ανά segment και crossing:

- `min_speed_mps` και `max_speed_mps` ως hard bounds στο crossing;
- `nominal_speed_mps` ως target και
  `nominal_speed_warning_tolerance_mps` ως telemetry-only deviation threshold;
- required entry/run-in/run-out;
- maximum curvature;
- trajectory tube και monotonic progress;
- `allow_reversing=false` και `allow_standalone_rotate=false`.

Speed εκτός min/max, curvature/tube/progress violation, reverse/rotate demand,
ή αδυναμία profile enforcement είναι explicit failure. Nominal deviation εντός
min/max δεν κάνει abort ή replan.

Το core δεν μειώνει κρυφά nominal speed για να χωρέσει σε angular/curvature
constraint, ούτε εκδίδει zero-angular standalone rotate για να ξαναπιάσει
route. Σε τέτοια περίπτωση αποτυγχάνει `profile_unenforceable` ή το ακριβές
typed geometry reason.

Ο controller δεν μαντεύει safety state. Το plugin-specific, plan/hash-bound
`SetCollectionSafetyHold` service δέχεται `hold=true|false`: `true` κάνει
zero-velocity hold χωρίς progress, ενώ `false` αξιολογεί monotonic/tube/run-in
resume contract. Wrong plan/hash ή invalid lifecycle είναι typed rejection.

Μετά από dynamic safety stop, ο controller επιτρέπεται να συνεχίσει μόνο με
forward progress και όταν παραμένει valid run-in πριν από το επόμενο crossing.
Αν δεν παραμένει, αποτυγχάνει ρητά. Δεν κάνει backtrack για να ανακτήσει
run-in.

## Telemetry and completion

Ο controller δημοσιεύει typed state/telemetry με:

- `plan_id` και `path_sha256`;
- monotonic progress `s`;
- active segment ID και optional active crossing identity;
- measured speed, lateral error και heading error;
- typed `ProfileComplianceVerdict`;
- explicit failure reason ή terminal completion.

Το terminal success επιτρέπεται μόνο στο configured terminal progress/pose.
Telemetry δεν αλλάζει το immutable context ή RoutePath.

## Nav2 execution boundary

Collection execution δεν χρησιμοποιεί BT. Στο C3 η direct `FollowPath` ροή
ασκείται μόνο από isolated ROS integration harness: load context και action
goal με explicit collection controller ID. Ο `CollectionPathFollower` δεν
συνδέεται ακόμη. Δεν υπάρχει collection recovery tree ή δυνατότητα `BackUp`,
`Spin` ή replan. Το survey NavigateToPose behavior tree και ο υπάρχων RPP
controller δεν αλλάζουν.

## C3.5/C4A — Python Nav2 adapter

Μετά το C3 και πριν από το C4 υπάρχει νέα phase runtime wiring. Ο adapter είναι
ο μόνος ROS owner της deterministic conversion frozen executable plan σε ένα
direct `FollowPath(controller_id=CollectionFollowPath)` και immutable context:
ordered segments/profiles/crossings, terminal progress/pose, canonical
configuration JSON και hash του τελικού Path μόνο με
`CollectionPathCanonicalizationV1`. Lifecycle: Load → FollowPath → hold/resume
→ Finalize → Reset. Ο adapter είναι owner cancel: success→SUCCEEDED,
safety/external cancel→CANCELED, controller failure→FAILED μόνο αν δεν είναι
already consumed. Telemetry είναι read-only plan/hash-bound input και δεν
μεταλλάσσει plan. Timeout, rejection, hash mismatch ή reset failure είναι typed
terminal failure και αφήνουν adapter unavailable μέχρι explicit upper-level
recovery/reset.
