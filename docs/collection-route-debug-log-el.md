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
  end-to-end — βήματα στο §8 του design doc: (α) `./run_ubuntu.sh` (ο Nav2
  σηκώνεται πάντα· ΔΕΝ χρειάζεται COLLECTION_USE_NAV2 — μόνο τα lawnmower
  lanes του `collect` το τιμούν) + υπάρχον map/court_boundary.json ή Map
  Court, (β) route overlay +
  numbered balls στο Collection Map μετά το scan, (γ) `motion_owner`
  εναλλαγή nav2 ⇄ collector_fsm ανά leg, (δ) fence/net case με
  `approach_mode=lateral` και heading ≈ παράλληλο στο όριο, (ε) insertion
  mid-route χωρίς αναδιάταξη προηγούμενων, (στ) `route_complete` +
  `collection_count` == μπάλες.
- **Επόμενο βήμα**: sim run κατά τα παραπάνω· μετά τα deferred gaps του
  issue #10 (loaded collect, incremental fill/beam counting, camera blind
  zone σε loaded προσέγγιση).

### 2. Πρώτο sim run: 4/11 ✅, stall στην 5η μπάλα — «έπιασε αλλά δεν το κατάλαβε»

- **Run** (2026-07-14, `./run_ubuntu.sh`, χειροκίνητο stop από χρήστη):
  - Scan + plan δούλεψαν: 360° σε ~21 s, `route_planned{stops=11,
    route_length_m=30.78, lateral_stops=0}`, route overlay ΟΚ.
  - Nav2 legs + handoff ΟΚ: `route_leg_start → nav2 pending → reached →
    route_fine_approach` για κάθε μπάλα.
  - Μπάλες 1, 2, 4, 3 μαζεύτηκαν και επιβεβαιώθηκαν (`route_ball_collected`,
    collection_count=4). Η #4 χρειάστηκε 1 retry (`approach_timeout` στα
    35 s) και πέτυχε στη 2η προσπάθεια.
  - **Μπάλα #11 (5η στη σειρά): STALL.** Fine approach t=155 → 35 s χωρίς
    confirm → `route_leg_retry(approach_timeout)` → Nav2 ξανά στο standoff →
    2ο fine approach 26 s → ο χρήστης σταμάτησε (t=220). ΣΗΜΕΙΩΣΗ: το 2ο
    timeout θα έριχνε skip στα ~229 s — η ασφάλεια δούλευε, απλώς αργά.
  - **Παρατήρηση χρήστη**: το ρομπότ ΕΠΙΑΣΕ την μπάλα φυσικά, αλλά δεν το
    κατάλαβε (κανένα confirm) και έκανε συνεχώς διορθώσεις για να «μαζέψει»
    το φάντασμα του dead-reckoned lock.
  - Ενδείξεις από τα δεδομένα: η mapped θέση της #11 μετατοπίστηκε ~0.4 m
    κατά την προσέγγιση (plan-time ~(4.10,-3.60) → τέλος (3.71,-3.73)) με
    seen_count 287 — η κάμερα έβλεπε μπάλα για πολλή ώρα ΚΑΤΑ τη διάρκεια
    των «διορθώσεων», άρα η μπάλα (ή κάποια μπάλα) ήταν στο έδαφος μπροστά
    του για μεγάλο μέρος του stall.
- **Υποθέσεις (ανοιχτές, χρειάζονται ground truth)**:
  - H1 — η μπάλα πιάστηκε από τους τροχούς αλλά ΔΕΝ εκτοξεύθηκε στο καλάθι
    (κόλλησε σε λαιμό/ράμπα): το basket-volume κριτήριο σωστά δεν την
    μετράει, αλλά το mission δεν έχει τρόπο να το μάθει και κυνηγάει το
    lock.
  - H2 — μπήκε στο καλάθι σε στιγμή που το intake ήταν off
    (`_check_collection` επιστρέφει False όταν `intake_enabled=False`) και
    κάτι εμπόδισε το count στα επόμενα CAPTURE (π.χ. z<0.055 ή εκτός x/y
    ζώνης ακουμπώντας σε άλλες μπάλες — 4 ήδη μέσα = πρώτο άγγιγμα του
    deferred «incremental fill» gap του #10).
  - H3 — bulldoze/αναπήδηση: η μπάλα σπρώχτηκε και μετακινήθηκε (συμβατό με
    το 0.4 m drift), το capture πέτυχε μόνο μερικώς. Το 2/5 approach
    timeout rate (και η #4) δείχνει ότι το capture στο collect_route είναι
    πιο εύθραυστο από το validated collect_one — πιθανή διαφορά: standoff
    1.3 m σημαίνει ότι το ALIGN→CAPTURE ξεκινά πιο κοντά απ' ό,τι στο
    collect_one (που σκανάρει από απόσταση), άλλο προφίλ πρόσκρουσης.
- **Αλλαγή (instrumentation, όχι behavior)**: νέο event
  `route_capture_probe` κάθε ~2 s όσο τρέχει fine approach (sim only):
  nearest ground-truth μπάλα σε robot frame (`local_x/y`, `z`,
  `already_counted` — ίδιος μετασχηματισμός με το `_check_sim_collection`,
  άμεσα συγκρίσιμο με τις πύλες του καλαθιού x∈(0.02,0.42), |y|≤0.14,
  z≥0.055), plus behavior state, approach elapsed, intake beam/latch,
  locked_world. Στο επόμενο run το stall θα δείχνει ΜΟΝΟ ΤΟΥ πού είναι η
  φυσική μπάλα (καλάθι/λαιμός/δίπλα) και αν έσπασε ποτέ το beam.
- **Status**: ⏳ αναμονή rerun με το probe· scan/plan/Nav2/overlay/4
  captures επιβεβαιωμένα ✅.
- **Επόμενο βήμα**: rerun → ανάγνωση των `route_capture_probe` στο stall →
  απόφαση fix βάσει H1/H2/H3.
