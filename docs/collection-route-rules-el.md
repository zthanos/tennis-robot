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

## Collector και speed profile

Η RoutePath δεν ξεκινά πριν ο collector δηλώσει `collector_ready` μέσα σε
`collector_start_timeout`. Timeout ή health failure στο start δίνουν
`aborted_collector`. Ο collector μένει ενεργός σε connectors και passes έως το
terminal pose.

Το ExecutionProfile ορίζει ανά segment:

- nominal/minimum/maximum crossing speed,
- acceleration πριν από entry και deceleration μετά από exit,
- minimum run-in και run-out,
- maximum curvature και tracking tolerance.

Ένα pass είναι μηχανικά άκυρο αν δεν μπορεί να φτάσει στο crossing με stable
heading και speed μέσα στο έγκυρο εύρος. Jam, collector health failure ή full
hopper τερματίζουν το run ως `aborted_collector`; δεν προκαλούν route replan.

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
