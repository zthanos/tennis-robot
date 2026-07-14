# Collect Route — γρήγορο μάζεμα με 360° scan, route planning και Nav2

Σχεδίαση και as-built τεκμηρίωση του mode `collect_route` (issue #10). Υλοποιεί
το «Collection Planning» layer του
[architecture-implementation-guide-el.md](architecture-implementation-guide-el.md)
πάνω στον επικυρωμένο μηχανισμό dual-wheel intake (PR #9) και συνδέεται με το
per-ball FSM του
[collection-state-machine-plan-el.md](collection-state-machine-plan-el.md).

## 1. Στόχος

Μάζεμα **όλων** των ορατών μπαλών του μισού γηπέδου στον ελάχιστο χρόνο:

1. **360° scan**: επιτόπια περιστροφή ώστε η OAK-D (sim) να καταγράψει όσο το
   δυνατόν περισσότερες μπάλες (εμβέλεια ~9 m).
2. **Route plan**: ταξινόμηση των μπαλών σε σειρά επίσκεψης ελάχιστης
   διαδρομής και υπολογισμός approach pose ανά μπάλα.
3. **Εκτέλεση**: μετακίνηση από μπάλα σε μπάλα με **Nav2 NavigateToPose**
   goals (costmap avoidance)· το τελικό ζύγωμα/μάζεμα με τον υπάρχοντα direct
   P-controller (`ConceptACollectorBehavior`).
4. **Dynamic insertion**: μπάλα που εμφανίζεται στην κάμερα mid-route μπαίνει
   στη διαδρομή με **cheapest insertion** (η σειρά των υπολοίπων δεν αλλάζει).
5. **Οπτικός έλεγχος**: η υπολογισμένη διαδρομή και η σειρά επίσκεψης
   εμφανίζονται στο Collection Map του web console.
6. **Πλάγια προσέγγιση**: μπάλα κοντά σε φράχτη/φιλέ προσεγγίζεται με heading
   ≈ παράλληλο στο όριο, ώστε το funnel να οδηγήσει την μπάλα στους τροχούς
   αντί το ρομπότ να καρφωθεί στο εμπόδιο.

## 2. Αρχιτεκτονική

| Κομμάτι | Αρχείο | Ρόλος |
| --- | --- | --- |
| Planner library | `tennis_robot/collection_route_planner.py` | Καθαρή γεωμετρία/ταξινόμηση — χωρίς rclpy. Port των primitives του `scripts/route_benchmark.py`. |
| Mission FSM | `tennis_robot/collect_route_mission.py` | `CollectRouteMission`: scan → plan → nav → approach → settle → done. Χωρίς rclpy. |
| Controller glue | `tennis_robot/controller_node.py` | Dispatch του mode, Nav2 wiring, τροφοδοσία ball map, status/console export. |
| Console | `scripts/control_panel/collection_map.js` | Ήδη υπήρχε render για `map.route` + `order` badges· απλώς γεμίζουν server-side πλέον. |

Motion arbitration (αμετάβλητο, ίδιο με τα lawnmower lanes):
- Nav2 → `/cmd_vel_nav` (twist_mux priority 50). Στα legs το mission
  επιστρέφει μηδενικό SURVEY command ώστε ο motor adapter να σιγήσει και οι
  τροχοί να περάσουν στον Nav2.
- Fine approach/capture → κανάλι priority 100 (κερδίζει πάντα).
- Στη μετάβαση nav → approach ο controller ακυρώνει το Nav2 goal.

**Πολιτική fail-loud**: το `collect_route` οδηγεί τα legs ΜΟΝΟ μέσω Nav2. Αν
τα nav2_msgs δεν κάνουν import ή ο action server δεν είναι up, το ρομπότ
σταματά και γράφεται event `nav2_unavailable` — δεν υπάρχει σιωπηλό fallback
στον P-controller.

## 3. Καταστάσεις mission

```
IDLE → SCAN_ROTATE → PLAN → [NAV_TO_BALL ⇄ FINE_APPROACH → SETTLE]* → DONE
```

- **SCAN_ROTATE** — step-rotation 12 × 30° με settle 0.20 s/βήμα (pattern του
  `CollectOneMission._scan_step`). Όσο τρέχει, ο controller περνά **όλα** τα
  detections κάθε frame στο `BallMap` (όχι μόνο το κοντινότερο) και το create
  gate του χάρτη ανοίγει από 3 m σε `COLLECT_ROUTE_SCAN_RANGE_M` (9 m) μέσω
  του `BallMap.max_create_distance_override_m`. Στο τέλος `prune_phantoms`.
- **PLAN** — υποψήφιες: confirmed (`seen_count ≥ min_seen_count`), φρέσκες,
  ίδια πλευρά, όχι collected/failed (ίδιο φίλτρο με `nearest_target_id`).
  Ταξινόμηση `order_route` = greedy nearest-neighbor + 2-opt polish (τρέχει
  μία φορά, όχι στο 31 ms tick). Approach pose ανά stop, αλυσιδωτά από το
  προηγούμενο. 0 μπάλες → DONE.
- **NAV_TO_BALL** — το mission εκθέτει `nav_goal=(x, y, yaw)`· ο controller
  καλεί `Nav2LaneNavigator.request()`. `reached` → FINE_APPROACH. `failed` ή
  leg timeout → cancel + επανέκδοση goal, μέχρι `COLLECT_ROUTE_NAV_RETRIES`·
  μετά το stop γίνεται `skipped` και η μπάλα `collection_failed`.
- **FINE_APPROACH** — `behavior.reset()` + lock στη θέση της χαρτογραφημένης
  μπάλας. Blind zone: το lock ανανεώνεται από live παρατηρήσεις εντός 0.6 m
  (`_RELOCK_GATE_M`, pattern του collect_one/debug-log #44) και dead-reckons
  όταν η κάμερα τυφλώνει < ~0.9 m. Επειδή το Nav2 goal έχει ήδη το σωστό
  (πιθανώς πλάγιο) yaw και η μπάλα είναι `standoff` ευθεία μπροστά, το ALIGN
  κάνει μικρή διόρθωση και το capture leg μένει παράλληλο στο όριο.
  Αποτυχία (gave_up/timeout 35 s) → retry από NAV_TO_BALL μία φορά, μετά
  skip. Μπάλα άφαντη → scan spin έως `COLLECT_ROUTE_MISSING_SCAN_S` → status
  `missing`, συνέχεια.
- **SETTLE** — 2.0 s ακινησία (intake τα πρώτα 0.25 s), μετά επόμενο stop.
- **Dynamic insertion** — σε κάθε tick των nav/approach/settle: νέα confirmed
  μπάλα εκτός πλάνου → `cheapest_insertion` στα pending stops **μετά** το
  τρέχον leg. Αν το detour ≤ `COLLECT_ROUTE_INSERTION_MAX_DETOUR_M` μπαίνει
  εκεί, αλλιώς στο τέλος. Αναριθμούνται μόνο τα εκκρεμή stops· ανανεώνεται
  και το approach pose του επόμενου stop (άλλαξε η διεύθυνση εισόδου του).

## 4. Πλάγια προσέγγιση (φράχτης/φιλέ)

`approach_pose_for_ball()` στο planner library:

- **Ρίσκο**: `CourtModel.ball_risk` — `obstacle` αν clearance από εμπόδιο ≤
  robot_radius + margin, `net_wall` αν απόσταση από φιλέ ή φράχτη ≤
  `COLLECT_ROUTE_BOUNDARY_MARGIN_M` (0.9 m), αλλιώς `normal`.
- **normal** → direct: standoff 1.3 m πίσω από την μπάλα πάνω στη διεύθυνση
  εισόδου, yaw προς την μπάλα.
- **net_wall/obstacle** → lateral: 16 υποψήφια headings σε δακτύλιο. Ένα
  heading είναι βιώσιμο αν (α) το standoff point είναι οδηγήσιμο
  (`pose_is_free`: εντός φράχτη, ≥ robot_radius από φράχτη/φιλέ, εκτός
  εμποδίων) και (β) ο **funnel corridor** — λωρίδα μισού πλάτους 0.17 m
  (στόμιο 260–340 mm / 2) από το standoff έως 0.30 m πίσω από την μπάλα —
  μένει εντός φράχτη και μακριά από φιλέ/εμπόδια. Score: μέγιστη
  παραλληλία `|dot(heading, boundary_tangent)|`, tie-break η απόσταση
  ταξιδιού. Αν τίποτα δεν περνά και τους δύο ελέγχους: το heading με το
  μέγιστο clearance (fallback), το ρίσκο μένει ώστε να ισχύει το όριο
  προσπαθειών.

Το γεωμετρικό όριο που δικαιολογεί το corridor: το funnel συγχωρεί πλευρικό
σφάλμα ±80 mm ([concept-a-funnel-lift-wheel-plan.md](concept-a-funnel-lift-wheel-plan.md)).

## 5. Court knowledge model

`CourtModel.from_boundary_file(runtime/court_boundary.json)` (schema
`court_knowledge_model/v2`, βλ.
[court-survey-v2-spec-el.md](court-survey-v2-spec-el.md)):

- fence: τα 4 corners (map frame) ως πολύγωνο· ερωτήματα απόστασης ανά ακμή.
- net: το segment των δύο posts (fallback: center ± span/2 × axis_width).
- obstacles: κύκλοι ακτίνας `hypot(w, h)/2` γύρω από το map-frame center.

Αν το αρχείο λείπει/είναι άκυρο → `None` → όλα τα approach poses γίνονται
direct (χωρίς lateral) και γράφεται warning. Τρέξε **Map Court** πρώτα.

## 6. Ρυθμίσεις (env)

| Env | Default | Ρόλος |
| --- | --- | --- |
| `COLLECT_ROUTE_SCAN_RANGE_M` | 9.0 | create-distance χάρτη στο 360° scan |
| `COLLECT_ROUTE_STANDOFF_M` | 1.3 | απόσταση approach pose από μπάλα |
| `COLLECT_ROUTE_BOUNDARY_MARGIN_M` | 0.9 | όριο ενεργοποίησης lateral |
| `COLLECT_ROUTE_INSERTION_MAX_DETOUR_M` | 3.0 | cap cheapest insertion (αλλιώς append) |
| `COLLECT_ROUTE_NAV_TIMEOUT_S` | 60 | timeout ανά Nav2 leg |
| `COLLECT_ROUTE_NAV_RETRIES` | 2 | Nav2 αποτυχίες πριν skip |
| `COLLECT_ROUTE_MAX_BALL_ATTEMPTS` | 2 | κύκλοι nav+capture ανά μπάλα |
| `COLLECT_ROUTE_MISSING_SCAN_S` | 6.0 | scan budget όταν η μπάλα λείπει |
| `COLLECT_ROUTE_TWO_OPT` | true | 2-opt polish μετά το greedy |

Capture timeout: `COLLECT_PATTERN_COLLECTION_TIMEOUT_S` (35 s).

## 7. JSON contracts (additive)

- `robot_status.json → map.route`: `[{x_m, y_m}, ...]` — ρομπότ, μετά
  (approach, μπάλα) ανά εκκρεμές stop. Το `collection_map.js` το ζωγραφίζει
  ως μπλε polyline (χωρίς αλλαγές στο JS).
- `map.balls[].planned/order`: `true` + αριθμός σειράς για μπάλες στο πλάνο
  (μέσω `BallMap.to_console_balls(planned_order=...)`).
- `map.metrics.total_distance_m` (υπόλοιπο μήκος route) και
  `map.metrics.planned_replans` (πλήθος insertions) — τα διαβάζει το summary
  του χάρτη.
- `robot_status.json → collect_route`: telemetry του mission (phase, τρέχον
  ball id, μετρητές stops ανά status, insertions, blocker, route table).
- Collection events (`runtime/collection_events.jsonl`): `route_scan_start`,
  `route_planned{stops, lateral_stops, route_length_m}`,
  `route_leg_start{ball_id, order, approach_mode, risk, goal_*}`,
  `route_fine_approach`, `route_ball_collected`, `route_leg_nav_retry`,
  `route_leg_retry`, `route_leg_skip`, `route_ball_missing`,
  `route_insertion{ball_id, detour_m, position}`, `route_complete{...}`,
  `nav2_unavailable`.

## 8. Εκκίνηση & επαλήθευση

1. Gazebo stack με `COLLECTION_USE_NAV2=true` ώστε να είναι up Nav2 +
   twist_mux + SLAM· φρέσκο `runtime/court_boundary.json` (Map Court).
2. Panel → Collection → **Collect Route**. Μετά την περιστροφή: μπλε route +
   αριθμημένες μπάλες στο Collection Map.
3. `runtime/collection_events.jsonl`: `route_scan_start → route_planned →
   route_leg_start...`· το `motion_owner` εναλλάσσεται nav2 → collector_fsm.
4. Fence/net: μπάλα ~0.3 m από φράχτη/φιλέ → `approach_mode=lateral`, τελικό
   heading ≈ παράλληλο στο όριο.
5. Insertion: μπάλα που εμφανίζεται mid-route → `route_insertion`,
   αναρίθμηση χωρίς αναδιάταξη των προηγούμενων.
6. Τέλος: `route_complete`, `collection_count` == μπάλες, mode → idle.

Unit tests (χωρίς ROS): `tests/test_collection_route_planner.py`,
`tests/test_collect_route_mission.py`, `tests/test_ball_map_console_export.py`.

## 9. Εκκρεμότητες / επόμενα

Κάθε αλλαγή/run του collect_route καταγράφεται στο
[collection-route-debug-log-el.md](collection-route-debug-log-el.md)
(υπόθεση/αποτέλεσμα/status, ίδια πειθαρχία με το intake log).

- Sim επαλήθευση end-to-end (βήματα §8) και καταγραφή στο παραπάνω log.
- Deferred gaps του issue #10 σε loaded runs: live loaded collect, loaded
  lateral envelope, σταδιακό γέμισμα (beam counting σε διαδοχικά μαζέματα).
- Keepout layer στο Nav2 costmap από τα obstacles του survey (σημειωμένο ως
  επόμενο βήμα στο court-survey-v2 spec) — μέχρι τότε το avoidance βασίζεται
  στο SLAM map + live LiDAR.
- Πιθανή αντικατάσταση του greedy+2-opt από το learned next-ball policy
  (`scripts/train_next_ball_policy.py`) όταν ωριμάσει.
