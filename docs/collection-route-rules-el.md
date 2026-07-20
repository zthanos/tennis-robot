# Ενεργός οδηγός: συνεχής διαδρομή συλλογής

> Κατάσταση: **ενεργό specification** για `collect_route`. Τα έγγραφα στο
> `docs/archive/` είναι ιστορικά και δεν αποτελούν οδηγία υλοποίησης.

## Σκοπός και όρια run

Κάθε run συλλέγει μπάλες μόνο από τη μισή πλευρά του γηπέδου όπου βρίσκεται το
ρομπότ. Δεν διασχίζει το φιλέ. Η συλλογή είναι αποτέλεσμα της διέλευσης του
funnel και δεν αλλάζει τη διαδρομή που ήδη εκτελείται.

Η ενεργή route δεν είναι σειρά από per-ball goals. Είναι μία frozen,
collision-free **RoutePath** η οποία έχει σχεδιαστεί πριν αρχίσει η συλλογή.

## Βασικοί ορισμοί

- **Scan snapshot**: το αμετάβλητο σύνολο επιβεβαιωμένων detections από ένα
  ολοκληρωμένο 360° scan.
- **RoutePath**: γεωμετρική, διατεταγμένη ακολουθία poses και segments στο
  `map`/court frame. Δεν είναι χρονικά παραμετροποιημένη trajectory.
- **ExecutionProfile**: target speed, acceleration/deceleration limits,
  curvature limits και tracking tolerances ανά segment.
- **ExecutedTrajectory**: η πραγματική, χρονικά διατεταγμένη πορεία που
  καταγράφεται από localization/odometry.
- **Connector**: τμήμα RoutePath από το exit ενός pass στο entry του επόμενου.
  Δεν επιτρέπεται να απαιτεί reverse, standalone rotate ή επιστροφή μέσα σε
  ήδη εκτελεσμένο funnel pass.
- **Funnel pass**: συνεχές forward segment με έγκυρο entry, crossing και exit.
- **Effective capture corridor**: το μηχανικό μισό πλάτος capture, μειωμένο
  κατά uncertainty θέσης μπάλας, localization, tracking και safety margin.
- **Covered**: η μπάλα περιλαμβάνεται στην επιλεγμένη RoutePath και το κέντρο
  της περνά μέσα στο effective capture corridor ενός συγκεκριμένου pass.
- **Deferred**: υπάρχει individual feasible crossing ή candidate route, αλλά η
  μπάλα δεν επιλέχθηκε στην τελική κοινή RoutePath.
- **Unreachable**: απορρίπτεται από deterministic feasibility test, όχι απλώς
  επειδή δεν βρέθηκε candidate μέσα στο planning budget.

## State machine

```text
idle
  -> navigating_to_scan_pose
  -> scanning
  -> planning
  -> collector_starting
  -> executing_route
       -> waiting_path_clear -> executing_route
       -> aborted_safety | aborted_tracking | aborted_collector
  -> route_completed
  -> collector_stopping
  -> evaluating_results
       -> navigating_to_scan_pose  (μόνο αν επιτρέπεται follow-up run)
       -> completed

scanning -> aborted_scan
planning -> aborted_planning
planning -> completed_no_targets
```

### Pure executor contracts (Phase 4A)

Το pure executor δέχεται μόνο injected dependencies και immutable typed
results. Δεν εισάγει ROS, actions, clocks ή raw-string FSM events.

`FollowUpConfiguration` είναι μέρος του immutable configuration snapshot και
έχει τα υποχρεωτικά πεδία `enabled: bool` και `max_total_runs: positive int`.
Το `max_total_runs` περιλαμβάνει τον αρχικό cycle: όταν `enabled=false` πρέπει
να είναι ακριβώς `1`, όταν `enabled=true, max_total_runs=1` δεν εκτελείται
follow-up, και όταν είναι `N` επιτρέπονται έως `N` πλήρεις
scan→plan→execute cycles. Ο run counter αυξάνεται όταν αρχίζει νέο navigation
προς scan pose.

Τα injected pure dependency results είναι immutable typed values:

- `NavigatorResult`: `running`, `succeeded`, `failed(reason)`,
  `unavailable(reason)`.
- `ScanSessionResult`: `running`, `snapshot_ready(snapshot)`, `failed(reason)`.
- `CollectorStartResult`: `starting`, `ready`, `failed(reason)`.
- `CollectorStopResult`: `stopping`, `stopped`, `failed(reason)`.
- `PathFollowerResult`: `running(progress_s, trajectory_tube_ok,
  remaining_run_in_m, requires_reverse, requires_standalone_rotate)`,
  `completed`, `failed(reason)`.
- `SafetyResult`: `clear`, `blocked`, `timeout`.

Τα `reason` είναι structured reason codes, όχι raw strings.

Στο safety pause ο executor αποθηκεύει immutable `s_before_pause`. Με
`SafetyResult.clear`, resume επιτρέπεται μόνο όταν το τελευταίο
`PathFollowerResult` έχει `progress_s >= s_before_pause`,
`trajectory_tube_ok=true`, `remaining_run_in_m` τουλάχιστον ίσο με το
απαιτούμενο run-in του επόμενου crossing και δεν απαιτεί reverse ή standalone
rotate. Κάθε άλλη περίπτωση είναι `aborted_tracking`. `SafetyResult.timeout`
είναι `aborted_safety`.

Μετά από ολοκληρωμένη ή ήδη aborted route, failed/timeout collector stop κάνει
`force_disable`, καταγράφει telemetry `collector_stop_fault` και δεν αλλάζει
το ήδη καταγεγραμμένο route outcome: συνεχίζει
`evaluating_results -> completed`. Το `aborted_collector` αφορά μόνο start
failure ή jam/full/health failure κατά την ενεργή route.

## Scan και frozen snapshot

1. Το ρομπότ πηγαίνει στο κέντρο της service line της τρέχουσας πλευράς.
2. Η μετάβαση σε αυτό το scan pose γίνεται με `NavigateToPose` και απαιτεί
   διαθέσιμο court model, localization confidence και collision-free path.
3. Στο scan pose εκτελείται 360° scan. Δεν υπάρχει fallback scan pose.
4. Αν το scan pose δεν είναι reachable, το localization δεν είναι έγκυρο, το
   scan timeout λήξει ή δεν επιτευχθεί η απαιτούμενη coverage/confirmation,
   το run λήγει ως `aborted_scan`.
5. Με επιτυχία δημιουργείται frozen snapshot. Μόνο οι μπάλες αυτού του
   snapshot συμμετέχουν στο τρέχον planning.
6. Empty snapshot είναι κανονική έκβαση: παράγεται empty plan με
   `planning_status=empty_no_balls` και το run τελειώνει ως
   `completed_no_targets`, χωρίς να ξεκινήσει collector.

Κάθε snapshot περιλαμβάνει τουλάχιστον:

```text
scan_id, scan_timestamp, map_frame, robot_pose_at_scan,
ball_id, ball_position, confidence, position_covariance
```

Το snapshot απαιτεί timestamp-aligned detection/depth, duplicate-ball fusion
και ελάχιστο ορισμένο αριθμό επιβεβαιωτικών observations.

### Snapshot geometry και planner view

Για το Gazebo MVP το canonical `SnapshotBall` διατηρεί:

```text
position_map_xy: Point2D
position_covariance_map_xy: symmetric PSD 2×2 covariance (m²)
```

Camera XYZ χρησιμοποιείται μόνο για timestamped TF projection προς map XY.
Το Z δεν είναι μέρος του Gazebo MVP snapshot/planner contract.

### Stable perception metadata boundary

Το `/perception/ball_detections` (`BallDetectionArray`) παραμένει το μοναδικό
downstream contract για ball perception. Το `BallDetectionArray.header.stamp`
είναι το RGB acquisition timestamp. Το υπάρχον `has_spatial` παραμένει
validity flag: μόνο detection με `has_spatial=true` είναι spatial target. Κάθε
τέτοιο detection φέρει το timestamp του depth frame που έγινε match
(`matched_depth_stamp`) και 3×3 position covariance της θέσης στο
`camera_link_optical_frame` (REP-103 optical, row-major, m²). Detection με
`has_spatial=false` ή missing/invalid spatial metadata αγνοείται από τον
`ScanSnapshotBuilder` και δεν μπαίνει σε snapshot. Empty arrays παραμένουν
perception heartbeat.

Ο downstream transform προς `map` γίνεται με TF/localization pose στο RGB
detection timestamp, ποτέ με την τρέχουσα pose του robot. Στο Gazebo MVP ο
adapter λαμβάνει timestamp-aligned `map_from_camera` transform και required
explicit `localization_xy_covariance` configuration. Η snapshot XY covariance
είναι rotated camera XY covariance συν αυτή τη configured conservative
localization covariance· δεν υπάρχει implicit default ή zero-covariance
assumption.

Για το Gazebo MVP η μοναδική επιτρεπτή provisional planning-safety profile
configuration είναι:

```text
localization_xy_covariance_m2:
  [[0.01, 0.0],
   [0.0, 0.01]]
```

Είναι declared conservative engineering budget για Gazebo planning, όχι
calibrated claim για physical localization. Αποθηκεύεται αυτούσια και
immutable στο `configuration_snapshot`, δεν έχει default/fallback και δεν
ενεργοποιεί physical snapshot. Μελλοντικό timestamped Gazebo GT +
`map`-alignment evidence μπορεί να την αντικαταστήσει χωρίς να μπλοκάρει το
τρέχον Phase 2 MVP.

Ο `ScanSnapshotBuilder` δεν επαναλαμβάνει RGB/depth fusion ούτε TF/metadata
validation: καταναλώνει μόνο accepted output του reusable adapter.

Metadata που λείπουν, είναι stale ή ασυνεπή δημιουργούν typed rejected
observation στο snapshot boundary. Δεν υπάρχει fallback σε current-pose
transform, unstamped depth ή estimated covariance. Αν τα rejected observations
εμποδίσουν coverage ή confirmation, το scan λήγει ως `aborted_scan`.

### Immutable snapshot configuration και adapter contract

Το `ScanSnapshot` και το `CollectionRoutePlan` αποθηκεύουν immutable copy:

```text
perception_spatial_validation:
  max_rgb_depth_timestamp_delta_s
  max_detection_to_tf_age_s
  covariance_psd_relative_tolerance
  min_position_covariance_trace_m2
  max_position_covariance_trace_m2
  localization_xy_covariance
calibration_artifact:
  calibration_id, model_id, model_version, platform, artifact_sha256,
  parameters, range_domain_m, depth_quality_domain, evidence_reference,
  calibration_date, acceptance_metrics
snapshot_association:
  association_mahalanobis_gate_chi2, min_confirmations,
  scan_step_id, min_distinct_scan_steps
```

Δεν υπάρχουν implicit defaults: missing/invalid configuration δεν δημιουργεί
εκτελέσιμο snapshot ή plan.

`PerceptionSpatialObservationAdapter` είναι το κοινό boundary για
`controller_node.py` και builder. Input: canonical `BallDetectionArray`,
detection index, timestamped `map_from_camera` transform, timestamped
Gazebo `localization_xy_covariance` configuration και immutable validation + calibration
configuration. Output είναι ακριβώς ένα από:

```text
AcceptedSpatialObservation {
  scan_id, rgb_timestamp, matched_depth_timestamp, map_xy, map_covariance_2x2,
  confidence, scan_step_id, calibration/configuration identity
}
SpatialObservationRejection { code, detection_index, rgb_timestamp, detail }
```

Codes περιλαμβάνουν τουλάχιστον `spatial_targets_unhealthy`,
`non_spatial_detection`, `perception_metadata_rejected`,
`perception_tf_rejected` και `frame_mismatch`. Physical mode δεν ενεργοποιεί
collection snapshot χωρίς ξεχωριστό verified localization covariance contract.

### Calibrated covariance production

Ο producer υπολογίζει την 3×3 camera-optical covariance από calibrated model
με inputs τουλάχιστον range και depth-quality metrics. Diagonal covariance
επιτρέπεται μόνο όταν οι axis variances τεκμηριώνονται από calibration evidence.
Gazebo και physical OAK-D χρησιμοποιούν το ίδιο model interface και το ίδιο ROS
contract, αλλά διαφορετικά calibrated parameter sets επιτρέπονται.

Κάθε calibration είναι versioned artifact/configuration με model/version ID,
parameters, range/depth-quality validity domain, dataset ή Gazebo scenario που
τη δικαιολογεί, ημερομηνία και acceptance metrics. Το model/version και τα
parameters είναι μέρος του `configuration_snapshot` του `ScanSnapshot` και του
`CollectionRoutePlan`. Εκτός validity domain ή με ανεπαρκές depth quality ο
producer θέτει `has_spatial=false`: δεν επιτρέπεται extrapolation, arbitrary
covariance ή fallback covariance.

Το perception monitor παραμένει ενεργό μετά το scan, αλλά post-scan detections
χρησιμοποιούνται αποκλειστικά για telemetry/validation. Δεν εισάγονται στον
planner ούτε μεταβάλλουν τη frozen RoutePath.

## Τι σημαίνει «βέλτιστη διαδρομή»

Η «βέλτιστη» ισχύει μόνο μέσα στο πεπερασμένο σύνολο collision-free candidates
που μπορεί να παραγάγει ο planner μέσα στο planning budget. Η επιλογή είναι
ιεραρχική:

1. Απορρίπτονται candidates που παραβιάζουν swept collision checking,
   costmap/keepout constraints, curvature, speed ή funnel constraints.
2. Επιλέγεται candidate με μέγιστο αριθμό `covered` μπαλών.
3. Σε ισοπαλία επιλέγεται ο μικρότερος **operational cost**.
4. Σε νέα ισοπαλία προτιμώνται λιγότερα funnel passes.

Operational cost συνδυάζει συνολικό μήκος, αναμενόμενο χρόνο, curvature,
ενεργειακή επιβάρυνση και πλήθος passes. Τα βάρη και τα όρια του είναι μέρος
του configuration snapshot του plan.

Η σειρά των κέντρων μπαλών είναι input στον planner, όχι η ίδια η διαδρομή.

Το `planning_status` είναι ένα από:

```text
feasible                    # όλα τα snapshot targets είναι covered
partial                     # υπάρχει τουλάχιστον ένα covered και ένα non-covered
empty_no_balls              # το snapshot δεν είχε targets
empty_no_feasible_targets   # κανένα target δεν μπορεί να καλυφθεί
planning_timeout
planning_failed
```

## Feasibility και ball results

Κάθε ball result έχει δύο ανεξάρτητα enums:

```text
status: covered | deferred | unreachable

reason_code:
selected | route_conflict | planning_budget | no_candidate_found |
keepout | turn_radius | no_entry | no_exit | mechanical_spacing
```

Παράδειγμα: `{ball_id: "ball_12", status: "deferred",
reason_code: "route_conflict"}`.

`unreachable` χρησιμοποιείται μόνο όταν deterministic test αποδείξει ότι δεν
υπάρχει επιτρεπτό crossing: π.χ. η μπάλα είναι εντός inflated keepout, δεν
υπάρχει επιτρεπτό heading interval, δεν χωρούν entry/exit ή κάθε crossing
τέμνει εμπόδιο. Αν ο planner απλώς δεν εξέτασε ή δεν βρήκε candidate εντός
budget, το αποτέλεσμα είναι `deferred`, όχι `unreachable`.

Αν το snapshot έχει targets αλλά όλα είναι `unreachable`, παράγεται empty plan
με `planning_status=empty_no_feasible_targets` και το run τελειώνει ως
`completed_no_targets`.

Αν η observation μιας frozen target αλλάξει σημαντικά μετά το scan, δεν γίνεται
replan. Καταγράφεται `target_position_invalidated` και η τελική έκβαση της
μπάλας διαχωρίζεται από το απλό `missed_collection`.

## Κοντινές μπάλες

Ο planner δημιουργεί global candidates, δεν αποφασίζει greedy τοπικά.

- **Κοινό pass**: επιτρέπεται μόνο αν υπάρχει συγκεκριμένο pass segment με
  κοινό heading, valid entry/exit, collision-free swept funnel και διαμήκη
  απόσταση των κέντρων τουλάχιστον ίση με τη μηχανική ελάχιστη απόσταση εισόδου.
- **Δύο passes**: επιτρέπονται μόνο με connector που αρχίζει στο προηγούμενο
  exit, φτάνει στο επόμενο entry χωρίς reverse/standalone rotate και τηρεί
  curvature, speed και swept-footprint constraints.
- Διαφορετικά η βέλτιστη κοινή route καλύπτει ό,τι είναι εφικτό και η άλλη
  μπάλα γίνεται `deferred` ή `unreachable` με τον αντίστοιχο reason code.

Δεν επιτρέπεται runtime σμίκρυνση run-in για να «χωρέσει» δεύτερη μπάλα.

## Φιλέ, φράχτης και στατικά εμπόδια

Φιλέ, φράχτης, στύλος, πάγκος και κάθε στατικό εμπόδιο έχουν keepout που
περιλαμβάνει robot footprint, funnel footprint και safety margin.

- Το crossing κοντά σε εμπόδιο ακολουθεί local obstacle tangent:
  `abs(wrap(crossing_heading - tangent_heading)) <= max_parallel_heading_error`.
- Σε γωνία απαιτεί clearance από όλα τα γειτονικά obstacle segments. Αν δεν
  υπάρχει μοναδικός ασφαλής tangent corridor, η μπάλα είναι `unreachable`.
- Κάθε connector, entry, crossing και exit περνά continuous swept-footprint
  collision checking. Έλεγχος μόνο σε αραιά sampled poses δεν αρκεί.
- Μετωπικό fallback προς εμπόδιο απαγορεύεται.

### Deterministic Phase 3A geometry contract

Το pure Phase 3A δέχεται explicit `CourtModel` με closed
`navigable_polygon` και `obstacles[{id, kind, polygon}]`, όπου το `kind` είναι
`net | fence | post | bench | other`. Κάθε obstacle και το exterior του
navigable polygon ελέγχεται inflated με required
`footprint_clearance_radius_m`, τον conservative circumscribed radius του
robot + funnel footprint + safety margin. Ένα straight segment είναι valid
μόνο όταν ολόκληρο το swept disk του κέντρου δεν τέμνει inflated obstacle ή
exterior court area.

Για heading `h` με lateral normal `n`, ο effective capture corridor είναι:

```text
capture_half_width_m - ball_radius_m
- confidence_multiplier * sqrt(nᵀ Σ_xy n)
- tracking_lateral_error_bound_m
- capture_safety_margin_m
```

Η snapshot covariance περιλαμβάνει ήδη Gazebo localization covariance και δεν
προστίθεται δεύτερη φορά. Το width πρέπει να είναι αυστηρά θετικό.

Ο planner παράγει μόνο το finite deterministic set από `N` equally spaced
headings στο `[0, 2π)` και τις δύο ακριβείς tangent directions κάθε relevant
net/fence segment. Αν η ball απέχει από uninflated net/fence boundary το πολύ
`tangent_activation_distance_m`, κάθε heading οφείλει να ικανοποιεί
`abs(wrap(h - tangent_heading)) <= max_parallel_heading_error`. Σε corner
ισχύουν ταυτόχρονα όλα τα active constraints.

Για crossing `p` και direction `u(h)`, `entry = p - minimum_run_in_m*u` και
`exit = p + minimum_run_out_m*u`. Failure στο entry ή στο full entry→crossing
segment είναι `no_entry`; αντίστοιχο failure στο exit/crossing→exit είναι
`no_exit`. Ball μέσα σε inflated keepout είναι `keepout`. Το `turn_radius`
δεν εκδίδεται στο Phase 3A: isolated straight pass έχει zero curvature και
connector feasibility αξιολογείται στο Phase 3B.

### Deterministic Phase 3B1 connector contract

Το Phase 3B1 παράγει μόνο forward, simple CSC Dubins connectors των families
`LSL`, `RSR`, `LSR`, `RSL`. CCC (`RLR`, `LRL`), loops, self-intersecting
connectors, reverse και standalone rotate απαγορεύονται. Κάθε edge λαμβάνει
μόνο required immutable configuration: `minimum_turning_radius_m`,
`max_connector_length_m`, `max_connector_arc_angle_rad` και
`max_connector_total_turn_rad`; δεν υπάρχουν defaults.

Connector που δεν έχει CSC candidate το οποίο ταυτόχρονα τηρεί radius, length,
individual-arc, total-turn και continuous swept-disk collision check έχει
typed edge rejection `turning_constraint_rejected`. Τα `collision`, `length`
και `turning-limit` rejections παραμένουν διακριτά edge telemetry. Το
`unreachable_turn_radius` δεν εκδίδεται ακόμη για BallResult: απαιτεί
μελλοντική global graph evaluation που αποδεικνύει ότι κάθε connector προς/από
κάθε candidate pass της μπάλας απορρίφθηκε για αυτόν τον λόγο.

### Deterministic Phase 3B2 global search contract

Το immutable `GlobalRouteSearchConfiguration` είναι required planner input και
αντιγράφεται αυτούσιο στο `CollectionRoutePlan.configuration_snapshot`:
`max_search_expansions`, `terminal_run_out_m`, connector/crossing nominal
speed, turn-energy equivalent και τα πέντε non-negative weights length/time/
curvature/energy/pass-count, με τουλάχιστον ένα θετικό weight. Δεν υπάρχουν
defaults.

Για selected route, `L` είναι το άθροισμα connector, funnel-pass και terminal
lengths. `T` είναι connector/terminal length divided by connector nominal
speed συν pass length divided by crossing nominal speed. `K` είναι το sum των
absolute connector arc angles, `E = L + turn_energy_equivalent_m_per_rad*K`
και `C = wL*L + wT*T + wK*K + wE*E + wP*pass_count`. Είναι planner surrogate:
δεν περιλαμβάνει acceleration/deceleration estimate.

Ο solver κάνει deterministic bounded DFS/branch-and-bound σε simple directed
paths. Η σειρά είναι maximum unique covered IDs, minimum `C`, minimum pass
count, stable route-ID tie-break. Κάθε final pass απαιτεί straight forward
terminal extension `terminal_run_out_m` με continuous swept-disk check.

Το ανεξάρτητο `planning_search_status` είναι `complete | budget_exhausted |
failed`. Με budget exhaustion και valid terminal route το plan είναι
executable `partial` και unexamined targets είναι `deferred/planning_budget`.
Χωρίς valid terminal route επιστρέφεται non-executable `planning_timeout`.
Όταν η search ολοκληρωθεί αλλά κανένα candidate δεν έχει valid terminal,
`planning_search_status=complete` και το αποτέλεσμα παραμένει
`planning_timeout`; το status περιγράφει terminal-validity outcome, όχι μόνο
budget exhaustion.

### Deterministic Phase 3C shared-pass contract

Το required immutable `SharedPassConfiguration` έχει χωρίς defaults
`max_shared_pass_balls >= 2`, positive `max_shared_pass_candidates` και
positive `minimum_mechanical_ball_spacing_m`. Ο generator χρησιμοποιεί μόνο
ήδη valid 3A single-ball candidates με ακριβώς κοινό heading. Κάθε member
διατηρεί έτσι effective corridor, tangent και individual collision validity.

Τα members ταξινομούνται longitudinally στο heading. Κάθε adjacent pair έχει
separation τουλάχιστον `minimum_mechanical_ball_spacing_m`. Το shared entry
είναι πριν από το πρώτο crossing κατά `minimum_run_in_m`, το exit μετά από το
τελευταίο κατά `minimum_run_out_m`, και όλο το entry→exit segment περνά
continuous swept-disk/boundary check. Δεν επιτρέπεται runtime shrink, stop ή
διαφορετικό heading μεταξύ members.

Παράγονται deterministic groups μεγέθους 2 έως cap και σε deterministic order
μέχρι `max_shared_pass_candidates`. Cap exhaustion είναι μόνο candidate-
generation budget telemetry, ποτέ deterministic `unreachable`. Κάθε shared
candidate φέρει όλα τα `covered_ball_ids` και εισέρχεται στο B1/B2 API ως
ordinary pass node.

### Deterministic Phase 3D planner composition

Το `collection_route_planner_v2.py` είναι το μοναδικό pure orchestration entry
point. Εκτελεί με σταθερή σειρά `ScanSnapshot → 3A individual candidates →
3C shared candidates → deterministic merge/deduplicate → B1 graph → B2
solver → immutable CollectionRoutePlan`. Το final plan έχει ordered connector,
funnel-pass και terminal segments, contiguous progress, execution-profile
references, terminal pose, planning/search statuses, πλήρη BallResult set και
το πλήρες immutable configuration snapshot. Δεν εισάγεται ROS/runtime access
σε καμία από αυτές τις φάσεις.

## Collector και speed profile

Η RoutePath δεν ξεκινά πριν ο collector δηλώσει `collector_ready` μέσα σε
`collector_start_timeout`. Timeout ή health failure στο start δίνουν
`aborted_collector`. Ο collector μένει ενεργός σε connectors και passes έως το
terminal pose.

Το ExecutionProfile ορίζει ανά segment:

- nominal/minimum/maximum crossing speed,
- required `nominal_speed_warning_tolerance_mps` χωρίς default,
- acceleration πριν από entry και deceleration μετά από exit,
- minimum run-in και run-out,
- maximum curvature και tracking tolerance.

Ένα pass είναι μηχανικά άκυρο αν δεν μπορεί να φτάσει στο crossing με stable
heading και speed μέσα στο έγκυρο εύρος. Jam, collector health failure ή full
hopper τερματίζουν το run ως `aborted_collector`; δεν προκαλούν route replan.

Στο crossing, `speed < min_speed_mps` ή `speed > max_speed_mps` είναι hard
`profile_violation`. Αν η speed είναι εντός αυτού του κλειστού interval αλλά
`abs(speed - nominal_speed_mps) > nominal_speed_warning_tolerance_mps`, η
route παραμένει profile-compliant και καταγράφεται μόνο telemetry
`nominal_speed_deviation`. Το typed `ProfileComplianceVerdict` περιέχει
`hard_compliant`, optional `hard_violation_reason`, `nominal_tracking`,
`measured_speed_mps` και `nominal_speed_error_mps`. Nominal deviation δεν
κάνει abort ή replan χωρίς μελλοντικό, ρητό sustained-speed contract.

Κάθε `FUNNEL_PASS` περιέχει immutable ordered `planned_crossings[]`. Κάθε
`PlannedCrossing` έχει `ball_id`, `position_xy`, `progress_s`, `heading_rad`
και `predicted_lateral_error`. Τα ball IDs εμφανίζονται ακριβώς μία φορά, το
`progress_s` αυξάνει αυστηρά και τα IDs είναι ακριβώς ίδια, με την ίδια σειρά,
με το summary `covered_ball_ids`. Single-ball pass έχει ένα crossing· shared
pass έχει ένα ανά μπάλα στις πραγματικές διαμήκεις θέσεις του pass. Per-crossing
telemetry και profile verdict συνδέονται με `ball_id + progress_s`, ποτέ με
ένα κοινό representative crossing point.

Το `collector_stopping` έχει timeout. Σε timeout γίνεται force-disable και
καταγράφεται collector fault, χωρίς να αλλάζει το ήδη καταγεγραμμένο αποτέλεσμα
της RoutePath.

## Εκτέλεση και safety pause

Η frozen RoutePath εκτελείται με Nav2 `FollowPath` και ειδικό collection
controller profile: `use_rotate_to_heading=false`, `allow_reversing=false`.
`NavigateThroughPoses`, per-ball `NavigateToPose` goals και automatic recovery
replans δεν χρησιμοποιούνται για τη συλλογή.

Η route έχει monotonic scalar progress `s ∈ [0, route_length]`.

- Άνθρωπος ή κινούμενο εμπόδιο θέτει το run σε `waiting_path_clear` και το
  safety layer σταματά το ρομπότ χωρίς αλλαγή geometry.
- Το resume προβάλλει την πραγματική pose μόνο σε forward progress window της
  ίδιας path: `s_resume >= s_before_pause`.
- Resume επιτρέπεται μόνο αν η pose είναι μέσα στο trajectory tube, υπάρχει
  επαρκές remaining run-in πριν από επόμενο crossing και δεν απαιτείται reverse
  ή standalone rotate.
- Αν το pause έγινε πολύ κοντά σε crossing και δεν απομένει έγκυρο run-in, το
  run κάνει abort. Δεν επιτρέπεται backtrack για να ξαναδημιουργηθεί run-in.
- Διαφορετικά, ή αν λήξει safety timeout, το run γίνεται `aborted_safety` ή
  `aborted_tracking`.

## Επιβολή ExecutionProfile

Η απλή αποστολή γεωμετρικού `nav_msgs/Path` στο `FollowPath` δεν αρκεί. Η
υλοποίηση πρέπει να αποδεικνύει ότι ο collection controller καταναλώνει ή
επιβάλλει το `ExecutionProfile` ανά segment, με controller/plugin,
segment-specific speed-limit updates ή ισοδύναμο controller-specific contract.

Χωρίς αυτή την επιβολή, speed, run-in, run-out και trajectory tube είναι μόνο
metadata του plan και όχι λειτουργικές απαιτήσεις.

## Terminal pose και follow-up runs

Κάθε plan περιέχει collision-free `terminal_connector` και `terminal_pose`
μετά το τελευταίο exit. Εκεί ολοκληρώνεται η route, ο collector σταματά και
αξιολογούνται τα results.

Το UI παρέχει checkbox **Follow-up runs** και μέγιστο πλήθος runs.

- Αν είναι κλειστό, το run τελειώνει και εμφανίζει `deferred`, `unreachable`,
  missed και νέες-after-scan μπάλες στη λίστα επόμενου run.
- Αν είναι ανοικτό, ξεκινά νέο ανεξάρτητο cycle από το terminal pose:
  `NavigateToPose(scan_pose) -> νέο 360° scan -> νέο frozen plan`.
- Κάθε follow-up χρησιμοποιεί μόνο το δικό του snapshot· δεν μεταλλάσσει την
  προηγούμενη RoutePath.

## Αποτέλεσμα planner και telemetry

Ο planner επιστρέφει `CollectionRoutePlan`:

```text
plan_id, scan_id, map_frame, start_pose, terminal_pose,
total_length, expected_duration, planning_status,
segments[{id, type, path, execution_profile, progress_start, progress_end,
          covered_ball_ids, obstacle_constraint}],
ball_results[{ball_id, status, reason_code, pass_id, predicted_lateral_error}],
configuration_snapshot
```

Το telemetry καταγράφει την `ExecutedTrajectory`, collection metrics,
`target_position_invalidated`, safety/collector states και την πραγματική
route progress. Console και logs δείχνουν την RoutePath και όχι μόνο κέντρα
μπαλών ή ενδιάμεσα Nav2 goals.

## Υποχρεωτικές παράμετροι πριν την υλοποίηση

- mechanical capture half-width και ball radius,
- ball-position, localization και tracking uncertainty,
- robot/funnel footprint και safety margins,
- minimum turning radius και maximum curvature,
- minimum entry/run-in/run-out/mechanical spacing,
- crossing speed/acceleration/deceleration limits,
- `max_parallel_heading_error`, trajectory tube και progress window,
- scan coverage/timeout/minimum confirmation,
- maximum RGB-depth timestamp delta, maximum detection-to-localization/TF age
  και covariance validity thresholds,
- planning budget και operational-cost weights,
- costmap inflation και safety timeout.

Αν κάποια λείπει, το αποτέλεσμα είναι μόνο heuristic preview, όχι εκτελέσιμο
collection plan.

## Κριτήρια αποδοχής

- Πριν από κίνηση υπάρχει immutable scan snapshot και πλήρες
  `CollectionRoutePlan`.
- Κάθε target έχει `covered`, `deferred` ή `unreachable` με reason code.
- Το `status` και το `reason_code` κάθε target είναι ανεξάρτητα enums.
- Empty snapshot και snapshot χωρίς feasible targets ολοκληρώνονται ως valid
  `completed_no_targets`, όχι ως scan failure.
- Κάθε pass έχει valid speed/heading/run-in/run-out και effective capture
  corridor που περιλαμβάνει τις αβεβαιότητες.
- Για κάθε crossing το telemetry συνδέει την ExecutedTrajectory με measured
  crossing speed, lateral error και heading error.
- Το telemetry αποδεικνύει ότι ο controller τήρησε το ExecutionProfile στα
  crossings.
- Κάθε segment περνά swept-footprint collision checking.
- Η RoutePath δεν μεταβάλλεται από αποτέλεσμα συλλογής ή detections μετά το
  scan.
- Safety pause συνεχίζει μόνο monotonic forward πάνω στην ίδια path ή aborts.
- Follow-up run δημιουργεί νέο scan snapshot και νέο plan.
