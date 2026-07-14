# Collection route (collect_route) — log

Σκοπός: αρχείο-καταγραφή για κάθε ενέργεια/διόρθωση γύρω από τον αλγόριθμο
μαζέματος (`collect_route`: 360° scan, route planning, Nav2 legs, lateral
approach, dynamic insertion, console overlay) ώστε να μην ξαναγυρνάμε στον
ίδιο κύκλο — ίδια πειθαρχία με το [intake-debug-log-el.md](intake-debug-log-el.md).
Νέα εγγραφή ανά αλλαγή/run: υπόθεση, τι δοκιμάστηκε, αποτέλεσμα, status,
επόμενο βήμα. Design doc αναφοράς:
[collection-route-plan-el.md](collection-route-plan-el.md) (issue #10).

## 2026-07-14

### 1. Αρχική υλοποίηση collect_route (κώδικας + unit tests, χωρίς sim run)

- **Πλαίσιο**: ο μηχανισμός intake κηρύχθηκε working (intake log #57, PR #9)·
  αποφάσεις χρήστη για τον αλγόριθμο: Nav2 goals για τα legs (όχι direct
  A*), cheapest insertion για νέες μπάλες (όχι full replan μετά από κάθε
  μάζεμα), πλάγια προσέγγιση κοντά σε φράχτη/φιλέ.
- **Υπόθεση**: 360° scan (create gate BallMap 3→9 m) + greedy NN/2-opt
  ordering + Nav2 legs + ο υπάρχων P-controller για το τελικό capture
  αρκούν· για μπάλες < 0.9 m από όριο, approach heading παράλληλο στο όριο
  με καθαρό funnel corridor (±0.17 m ≈ στόμιο/2) ώστε το funnel να οδηγεί
  την μπάλα στους τροχούς.
- **Υλοποίηση** (commits 5ab3369, 78f0cc1, cd15c42 στο feat/collection-pattern):
  - `tennis_robot/collection_route_planner.py`: CourtModel (parse
    court_boundary.json v2 — fence corners πολύγωνο, net segment από posts,
    obstacles ως κύκλοι), `order_route` (greedy NN + 2-opt), 
    `approach_pose_for_ball` (direct/lateral, 16 candidate headings,
    corridor check), `cheapest_insertion`, `route_polyline`.
  - `tennis_robot/collect_route_mission.py`: FSM scan→plan→nav→approach→
    settle→done· nav retries/timeout→skip (`collection_failed`), blind-zone
    lock από το collect_one (relock gate 0.6 m), missing-ball scan budget
    6 s, insertion μόνο μετά το τρέχον leg.
  - Controller: mode `collect_route` (dispatch, SUPPORTED_MODES),
    multi-ball frame feed στο ball map (scoped στο mode — αλλιώς το scan
    χάνει μπάλες πίσω από την κοντινότερη), Nav2LaneNavigator κατασκευάζεται
    πάντα όταν τα deps υπάρχουν, fail-loud `nav2_unavailable` χωρίς
    P-controller fallback στα legs, route/planned/order/metrics στο
    Collection Map payload (το collection_map.js τα render ήδη — μηδέν
    αλλαγές JS στον χάρτη), κουμπιά «Collect Route».
  - Νέα collection events: `route_scan_start/route_planned/route_leg_start/
    route_fine_approach/route_ball_collected/route_leg_nav_retry/
    route_leg_retry/route_leg_skip/route_ball_missing/route_insertion/
    route_complete`.
- **Αποτέλεσμα (offline)**: 35 νέα unit tests πράσινα
  (`test_collection_route_planner.py` 16, `test_collect_route_mission.py`
  14, `test_ball_map_console_export.py` +5)· πλήρης σουίτα 84 passed.
  Σημείωση: το collection error του `test_console_app.py` όταν τρέχει όλη η
  σουίτα μαζί ΠΡΟΫΠΑΡΧΕΙ στο main (import-order conflict)· μόνο του περνάει.
- **Ευρήματα κατά την υλοποίηση**:
  1. Το `collection_map.js` είχε ήδη ΕΤΟΙΜΟ, νεκρό hook για route overlay
     (`map.route` polyline γρ. 200, `planned`/`order` badges γρ. 316-340,
     `metrics.total_distance_m`/`planned_replans` γρ. 495) — κανείς δεν τα
     γέμιζε server-side. Η δουλειά στο console ήταν καθαρά Python-side.
  2. `MAPPED_BALL_MAX_CREATE_DISTANCE_M=3.0`: χωρίς override το 360° scan
     δεν θα κατέγραφε τίποτα πέρα από 3 m — μπήκε
     `BallMap.max_create_distance_override_m`, ενεργό ΜΟΝΟ στο scan phase.
  3. Ο `_on_ball_detections` κρατούσε μόνο το κοντινότερο detection ανά
     frame· για το scan χρειάζεται όλη η λίστα (`_latest_observations`).
  4. Πλάγια προσέγγιση σε μπάλα < robot_radius (0.36 m) από το όριο: το
     ακριβώς-παράλληλο standoff είναι μη οδηγήσιμο· τα 16 headings ανά
     22.5° δίνουν το πρώτο βιώσιμο ελαφρώς κεκλιμένο (|dot|=0.924 > cos30°).
  5. Insertion bug που έπιασαν τα tests: το `_insert_new_balls` έτρεχε και
     στο tick του PLAN και μετρούσε τις αρχικές μπάλες ως insertions —
     περιορίστηκε στα nav/approach/settle phases.
- **Status**: ✅ κώδικας + unit tests. ⚠️ ΕΚΚΡΕΜΕΙ sim επαλήθευση
  end-to-end — βήματα στο §8 του design doc: (α) stack με
  `COLLECTION_USE_NAV2=true` + φρέσκο Map Court, (β) route overlay +
  numbered balls στο Collection Map μετά το scan, (γ) `motion_owner`
  εναλλαγή nav2 ⇄ collector_fsm ανά leg, (δ) fence/net case με
  `approach_mode=lateral` και heading ≈ παράλληλο στο όριο, (ε) insertion
  mid-route χωρίς αναδιάταξη προηγούμενων, (στ) `route_complete` +
  `collection_count` == μπάλες.
- **Επόμενο βήμα**: sim run κατά τα παραπάνω· μετά τα deferred gaps του
  issue #10 (loaded collect, incremental fill/beam counting, camera blind
  zone σε loaded προσέγγιση).
