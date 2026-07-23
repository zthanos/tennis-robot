# ΑΡΧΕΙΟ — ιστορικό log του παλαιού collect_route

> Δεν χρησιμοποιείται ως specification ή οδηγός υλοποίησης. Περιέχει ιστορικές
> δοκιμές και διορθώσεις του προηγούμενου per-ball μοντέλου. Το ενεργό
> specification είναι το [Ενεργός οδηγός συνεχούς διαδρομής](../collection-route-rules-el.md).

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

## 2026-07-15

### 3. Run 2 με probes: root cause ΒΡΕΘΗΚΕ — λοξά launches παρκάρουν μπάλες στο deck, εκτός scoring volume

- **Run** (2026-07-15, χειροκίνητο stop από χρήστη στο 6ο/7ο stop λόγω
  «πολλών failures»): 6 μπάλες μαζεύτηκαν φυσικά, 2 live insertions
  (`route_insertion` #11 detour 0.17 m θέση 6, #15 detour 0.91 m θέση 8 —
  το dynamic insertion δουλεύει live ✅).
- **Το μοτίβο, με απόδειξη από τα `route_capture_probe`** (κάθε γραμμή
  local x/y/z της πλησιέστερης φυσικής μπάλας):
  - Κεντραρισμένες προσεγγίσεις = γρήγορα, καθαρά captures: μπάλες 1, 2, 4
    μπήκαν με |ly| ≤ 0.11 και επιβεβαιώθηκαν σε ~4-18 s χωρίς κανένα retry.
  - Μη κεντραρισμένες (ly 0.15-0.25 στο capture — 2-3× το validated
    envelope ±0.08 του funnel): η μπάλα πιάνεται μεν, αλλά το launch βγαίνει
    λοξό και η μπάλα προσγειώνεται **πάνω στο ρομπότ, ΕΞΩ από το bin**:
    ball_06 → (0.06, **0.24**, z 0.058)· ball_00 → (−0.22, **0.32-0.47**,
    z 0.058) στο ΠΙΣΩ deck· ball_05 → (0.0, **0.21-0.25**, z 0.058).
    Το z=0.058 = μπάλα ακουμπισμένη σε επιφάνεια ~25 mm (deck), όχι έδαφος
    (0.033).
  - Το scoring μετρούσε ΜΟΝΟ το εσωτερικό του bin (x 0.02-0.42, |y|≤0.14,
    z≥0.055 — σωστό ως προς το spec του bin v2), οπότε deck-parked μπάλα =
    κανένα confirm → το mission κυνηγά το dead-reckoned φάντασμα 20-30 s,
    μέχρι κάποιο τράνταγμα να κυλήσει μια μπάλα μέσα στο παράθυρο. Στο stop
    της #10 το confirm ήρθε από ΛΑΘΟΣ μπάλα (η παρκαρισμένη ball_05 πέρασε
    το gate, ενώ ο στόχος ball_12 ήταν ακόμα στο έδαφος).
  - Beam: έσπασε μόνο 1 transient tick σε όλο το run — άχρηστο ως
    confirmation signal με την τρέχουσα γεωμετρία του.
- **Fix 1 (scoring — υλοποιήθηκε)**: νέο module
  `tennis_robot/collection_scoring.py` με `onboard_ball_zone()`: κρατά το
  bin gate ως «bin» και προσθέτει «deck» ζώνη (lx −0.30…0.45, |ly|≤0.35,
  z≥0.050) = μπάλα πάνω στο σώμα του ρομπότ. Το
  `_check_sim_collection` πιστώνει και τις δύο (η μπάλα έχει φύγει από το
  γήπεδο) και γράφει νέο event `sim_collection_credit{ball_def, zone,
  local_x/y, z}` ώστε να μετράμε πόσα launches καταλήγουν deck αντί bin.
  Unit tests: `tests/test_collection_scoring.py` (gates πάνω στις
  παρατηρημένες θέσεις του run 2). Πλήρης σουίτα 88 passed.
- **Fix 2 (instrumentation)**: το probe απέκτησε `lock_error_m` = απόσταση
  locked_world → πλησιέστερη φυσική μπάλα, για να ποσοτικοποιηθεί στο
  επόμενο run η πηγή του πλευρικού σφάλματος (καμερική προβολή vs pose/SLAM
  drift vs stale lock).
- **ΑΝΟΙΧΤΟ (μηχανισμός/έλεγχος, ΔΕΝ λύθηκε)**: γιατί η fine approach
  φτάνει στο nip με ly έως 0.25; Ύποπτοι: (α) σφάλμα προβολής κάμερας →
  world στο lock, (β) yaw/pose σφάλμα SLAM μετά τα Nav2 legs, (γ) αδύναμη
  γωνιακή διόρθωση στο CAPTURE (gain 1.2, commit-straight <0.45 m —
  bench-validated τιμές, ΜΗΝ αλλαχθούν χωρίς δεδομένα). Θα κριθεί από τα
  `lock_error_m` του επόμενου run. Σχετίζεται και με το «loaded lateral
  envelope» deferred gap του issue #10.
- **Status**: ✅ scoring fix + tests offline· ⏳ rerun για επιβεβαίωση ότι
  τα stalls εξαφανίζονται και για μέτρηση lock_error_m.
- **Επόμενο βήμα**: restart stack (DEV_OVERLAY rebuild) → rerun Collect
  Route → αν τα deck credits είναι συχνά, ανοίγει mechanism entry για το
  πλευρικό σφάλμα (πιθανά: ψηλότερα guards/χαμηλότερο tray στο bin ή
  διόρθωση του lock πριν το capture commit).

### 4. «Δεύτερο 360» μετά την πρώτη μπάλα: τυφλό scan-spin όταν το Nav2 reached αφήνει τη μπάλα πίσω

- **Αναφορά χρήστη (run 2/3)**: μετά την πρώτη μπάλα το ρομπότ «ξαναέκανε
  360». ΔΕΝ ήταν το mission scan (τρέχει μόνο μία φορά) — probes της
  μπάλας 2 (run 2): στο fine-approach entry το behavior έμεινε σε `scan`
  8+ s με την πλησιέστερη φυσική μπάλα σε local (−1.0, 1.4) = ΠΙΣΩ από το
  ρομπότ. Ο Nav2 έδωσε «reached» με χαλαρό τελικό yaw (το goal ήταν ~0.35 m
  από τη θέση μετά το προηγούμενο capture — έγινε δεκτό σχεδόν επιτόπου),
  το `_world_to_robot_obs` επιστρέφει None για μπάλα πίσω (local_x ≤ −0.1),
  και το behavior ξεκίνησε την τυφλή περιστροφή αναζήτησης (1.1 rad/s) —
  αυτό φάνηκε ως δεύτερο 360.
- **Δεύτερο bug στο ίδιο σημείο**: όσο η μπάλα-στόχος ήταν πίσω, το
  tracking έπεφτε στο ωμό camera observation (`tracking_obs = locked_obs
  or observation`) — δηλαδή μπορούσε να κλειδώσει σε ΑΛΛΗ ορατή μπάλα και
  να «κλέψει» το approach (πιθανή συνεισφορά στο juggling του run 2).
- **Fix (υλοποιήθηκε)**: στο `_approach_phase`, όταν το locked ball είναι
  πίσω από το ρομπότ, το mission στρίβει ΚΑΤΕΥΘΕΙΑΝ προς το bearing του
  (shortest turn, P-gain 1.8, cap 0.65 rad/s) αντί να αφήσει το behavior
  να ψάχνει στα τυφλά· και το tracking τρέφεται ΜΟΝΟ από το locked obs —
  το ωμό observation επηρεάζει μόνο το lock refresh (gated 0.6 m), ποτέ
  την ταυτότητα του στόχου. Tests:
  `test_ball_behind_turns_toward_it_instead_of_blind_scan`,
  `test_other_visible_ball_does_not_steal_tracking`. Σουίτα 90 passed.
- **Status**: ✅ offline· ⏳ επιβεβαίωση στο ίδιο rerun με το #3.

### 5. Αξιολόγηση: επιπλέον tilt κάμερας για μικρότερο τυφλό σημείο — ΑΠΟΡΡΙΦΘΗΚΕ (καμία αλλαγή)

- **Ερώτημα χρήστη**: μικρό επιπλέον tilt προς τα κάτω για να μικρύνει το
  τυφλό σημείο, χωρίς να χαθεί εμβέλεια/αναγνώριση φιλέ.
- **Δεδομένα** (mount: x 0.535, ύψος 0.488, pitch 15.6° κάτω· RGB hFOV
  1.204 rad @640x480 → vFOV 54.5°· τυφλό σημείο σήμερα ≈0.99 m από
  base_footprint για κορυφή μπάλας):
  | Δtilt | τυφλό σημείο | κέρδος | πλήρες φιλέ ορατό από |
  |---|---|---|---|
  | +3° | 0.94 m | 6 cm | 3.8 m |
  | +5° | 0.92 m | 8 cm | 5.0 m |
  | +10° | 0.85 m | 15 cm | 20.6 m |
- **Γιατί όχι**: (1) κέρδος 6-15 cm μόνο — η κάτω ακτίνα κοιτάει ήδη 42.8°
  κάτω και τα τελευταία ~30 cm ως το funnel μένουν τυφλά ό,τι κι αν
  κάνουμε (occlusion + depth near-clip 0.20 m)· (2) το πλήρες φιλέ σήμερα
  χωράει στο κάδρο από 2.8 m — με +5° χάνεται μέσα στα 5 m, με +10°
  πρακτικά πάντα· (3) το πλευρικό σφάλμα του #3 εμφανίζεται στα 0.9-1.2 m
  όπου η κάμερα ΒΛΕΠΕΙ την μπάλα και το lock φρεσκάρεται — το τυφλό σημείο
  δεν είναι ο πιθανός ένοχος. Αναμένουμε τα `lock_error_m` πριν
  οποιαδήποτε αλλαγή οπτικής.
- **Status**: ❌ δεν εφαρμόστηκε αλλαγή (τεκμηριωμένη απόρριψη).

### 6. Run 3: το ρομπότ έπεσε στο ΦΙΛΕ — phantom στόχος + 4 ρίζες, 4 διορθώσεις

- **Run 3** (με τα fixes των #3/#4): ξεκίνησε καθαρά — 3 πρώτες μπάλες
  χωρίς stalls και χωρίς «δεύτερα 360», 2 insertions. Μετά: το stop της
  μπάλας #14 (net_wall/lateral) κατέληξε με το ρομπότ στο (7.42, 5.70) —
  πάνω στο φιλέ (net x≈8.07, posts y≈±5.66) — να σπρώχνει σε capture creep
  επί **78+ s**. Ο χρήστης το σταμάτησε.
- **Τι έδειξαν τα probes** (τεκμήρια):
  1. `lock_error_m` 3.3→4.0 (αυξανόμενο): ΚΑΜΙΑ φυσική μπάλα κοντά στο
     lock — η εγγραφή #14 του χάρτη ήταν phantom· το approach
     dead-reckon-άριζε προς το πουθενά, με το φιλέ στη διαδρομή.
  2. Το approach timeout (35 s) ΔΕΝ πυροδότησε ποτέ: ο έλεγχος ήταν ΜΕΤΑ
     τα early returns του `_approach_phase` — το turn-toward-target branch
     του #4 επέστρεφε πριν φτάσει εκεί (regression δικό μας).
  3. Στο fine approach οδηγεί ο P-controller (twist_mux priority 100) —
     κανένα costmap/obstacle awareness· και το φιλέ ΔΕΝ υπάρχει στο Nav2
     costmap ούτως ή άλλως (το keepout από το survey είναι ανυλοποίητο
     «επόμενο βήμα» του court-survey-v2).
  4. Η ταξινόμηση «ίδια πλευρά» (across_net) υποθέτει net_x=0 — στο map
     frame αυτού του κόσμου το φιλέ είναι στο x≈8.08. Μπάλες πέρα από το
     πραγματικό φιλέ περνιούνται για ίδια πλευρά (και μπάλες της αριστερής
     near-half κόβονται ως «across» όταν το ρομπότ είναι σε x>0.25).
- **Διορθώσεις (υλοποιήθηκαν, 95 tests πράσινα)**:
  - Timeout ΠΡΩΤΑ: gave_up/timeout έλεγχος στην αρχή του `_approach_phase`,
    πριν από κάθε early return (`_approach_failed` helper).
  - Phantom gate: αν από την έναρξη του approach δεν υπάρξει ΚΑΝΕΝΑ live
    sighting εντός του relock gate μέχρι `MISSING_SCAN_S` (6 s) → η μπάλα
    κηρύσσεται `missing` με reason `no_live_sighting_at_standoff` (η κάμερα
    ΠΡΕΠΕΙ να βλέπει πραγματική μπάλα από το 1.3 m standoff — το blind zone
    αρχίζει στα ~0.9 m).
  - LiDAR forward guard στο approach: εντολή με linear>0 μπλοκάρεται όταν
    front range < `COLLECT_ROUTE_FRONT_BLOCK_M` (default 1.45 m ΑΠΟ ΤΟ
    LIDAR στο x=−0.42 → στοπ ~15 cm πριν το funnel tip αγγίξει)· event
    `route_approach_blocked`. Το LiDAR δεν βλέπει μπάλες — ό,τι στέκεται
    μπροστά είναι φιλέ/φράχτης/έπιπλο.
  - Πλευρά από το ΠΡΑΓΜΑΤΙΚΟ φιλέ: `CourtModel.same_side()` (signed
    distance από τη surveyed net line) + `contains()` (fence polygon) —
    χρησιμοποιούνται στο πλάνο/insertions ΚΑΙ στο observation filter του
    collect_route (`_collect_route_observation`)· fallback στο legacy
    across_net μόνο χωρίς court model.
- **ΑΝΟΙΧΤΟ (D2)**: το φιλέ εξακολουθεί να ΜΗΝ υπάρχει στο Nav2 costmap —
  τα legs του Nav2 κοντά στη ζώνη του φιλέ βασίζονται μόνο στο SLAM map +
  live LiDAR. Σωστή λύση: keepout filter ή virtual obstacle από το court
  knowledge model (το «επόμενο βήμα (4)» του court-survey-v2 spec).
- **ΑΝΟΙΧΤΟ (γιατί phantom;)**: πώς γράφτηκε εγγραφή με seen_count ≥
  threshold χωρίς φυσική μπάλα κοντά — ύποπτο το 360° scan με το 9 m
  create gate (προβολικά σφάλματα σε μεγάλες αποστάσεις + merge). Τα
  probes του επόμενου run (lock_error_m στο approach start) θα δείξουν.
- **Status**: ✅ fixes offline· ⏳ rerun.

### 7. Run-3 follow-ups: η «μισή περιστροφή» είναι φυσιολογική· αρχικό adoption gate 1.0 m

- **Αναφορά χρήστη**: ανάμεσα στην 1η και 2η μπάλα «ξεκίνησε περιστροφή
  αλλά σταμάτησε πριν τις 90°». Από τα δεδομένα: ΔΥΟ κανονικές φάσεις —
  (α) t=32.2 έναρξη Nav2 leg: το rotate-to-heading του RPP σταματά μόλις
  το heading error πέσει κάτω από το κατώφλι του και συνεχίζει οδηγώντας
  σε καμπύλη (αυτό φάνηκε ως κομμένη στροφή)· (β) t=41.9 στο standoff, η
  μπάλα πίσω-αριστερά (bearing ~122°): turn-branch (#4) + ALIGN την
  έφεραν μπροστά σε ~4 s. Μπάλα 2 μαζεύτηκε 20 s μετά την 1η. ΟΧΙ bug.
- **Εύρημα από lock_error_m (approach start)**: ball1 0.05, ball2 **0.49**,
  ball4 0.08, ball3 0.14 — οι εγγραφές του 360°/9 m scan κουβαλούν έως
  ~0.5 m σφάλμα θέσης. Με το αυστηρό relock gate 0.6 m, εγγραφή με σφάλμα
  >0.6 m θα απορριπτόταν το live sighting → ψευδο-phantom (missing) ενώ η
  μπάλα υπάρχει δίπλα. **Αλλαγή**: `_INITIAL_ADOPT_GATE_M=1.0` — ΜΟΝΟ το
  πρώτο sighting του approach υιοθετείται με gate 1.0 m (από το standoff η
  κοντινότερη μπάλα εντός 1 m του πλάνου είναι ο στόχος)· κάθε επόμενο
  refresh κρατά το 0.6 m anti-steal gate. Επιπλέον όφελος: μειώνει και το
  αρχικό πλευρικό σφάλμα του capture (το lock κεντράρεται στο πραγματικό
  σημείο νωρίς). Tests 96 passed.
- **Σημείωση telemetry**: τα `sim_collection_credit zone=deck` του run 3
  πυροδοτούνται συχνά ΚΑΤΑ τη διέλευση από τη ράμπα (x≈0.44, z 0.06-0.08),
  όχι σε παρκαρισμένη μπάλα — το count είναι σωστό (και νωρίτερο = καλύτερη
  απόδοση στόχου), αλλά η αναλογία bin/deck ΔΕΝ δείχνει το τελικό σημείο
  ηρεμίας· για την αξιολόγηση του funnel #59 θα κοιτάμε το τελικό
  /sim/balls state στο τέλος του run.
- **Status**: ✅ κώδικας· ⏳ run 4.

### 8. Run 4: ΠΡΩΤΟ route_complete (8/12) — τα 4 «missing» ήταν ΠΡΑΓΜΑΤΙΚΕΣ μπάλες: cluster confusion + stale standoff

- **Run 4** (όλα τα fixes #3-#7 + νέο funnel #59): **route_complete στα
  364 s** — 12 stops, 8 collected, 4 missing, 1 insertion, 0 skipped,
  κανένα άγγιγμα στο φιλέ, κανένα stall >35 s, τα retries/misses έκλεισαν
  γρήγορα (6 s phantom gate). Ο χρήστης ανέφερε «σφάλματα» = τα 4 missing.
- **Εύρημα — τα missing (ids 5, 6, 9, 12) ΔΕΝ ήταν phantoms**: τελικό
  map state με seen_count 174-757 και καταστάσεις collection_failed. Η
  κάμερα τις έβλεπε συνέχεια. Δύο ρίζες:
  1. **Cluster confusion**: το mission τρέφεται με το detection που είναι
     κοντινότερο ΣΤΟ ΡΟΜΠΟΤ· όταν άλλη μπάλα είναι πιο κοντά από τον
     στόχο, ο στόχος «δεν έχει live sighting» → ψευδο-missing στα 6 s
     (π.χ. stop 9 στο (−0.49,−2.73) με άλλες μπάλες τριγύρω).
  2. **Stale standoff στο retry**: μπάλα που σπρώχτηκε στο 1ο attempt
     (π.χ. stop 12: 35 s juggling, το φυσικό ball_12 μαζεύτηκε τελικά ΚΑΤΩ
     από το stop 10) — το retry πήγε στο standoff της ΑΡΧΙΚΗΣ θέσης, όπου
     πλέον δεν υπήρχε τίποτα εντός adoption gate.
- **Διορθώσεις (υλοποιήθηκαν, 98 tests)**:
  - `_collect_route_target_observation` στον controller: επιλέγει από ΟΛΟ
    το frame το detection που είναι κοντινότερο στο `current_target_xy`
    του mission (lock ή μπάλα του stop, gate 1.5 m, με freshness check)·
    fallback στο κοντινότερο-στο-ρομπότ.
  - Goal refresh στο `_nav_phase`: αν η χαρτογραφημένη μπάλα έχει
    μετατοπιστεί > `_GOAL_REFRESH_DRIFT_M` (0.3 m) από τη θέση του stop,
    ενημερώνονται θέση + approach pose + nav goal (event
    `route_goal_updated`)· το Nav2 replan-άρει (replan_tolerance 0.5).
- **Παρατήρηση funnel (#59 live)**: τα 3-4 πρώτα καθαρά captures πέρασαν
  τη ράμπα με |ly| ≤ 0.085 (υγιή)· τα credits σε περίεργα σημεία του deck
  (π.χ. ball_12 πίσω γωνία, ball_00 ly −0.35) προέκυψαν από captures ΚΑΤΑ
  τη διάρκεια juggling — αναμένεται να μειωθούν όσο πέφτουν τα juggling
  με τα παραπάνω.
- **Status**: ✅ κώδικας· ⏳ run 5 — κριτήριο επιτυχίας: missing ΜΟΝΟ αν η
  μπάλα όντως δεν υπάρχει· διαφορετικά 100% των υπαρκτών same-side μπαλών
  collected.
- **Addendum (παρατήρηση χρήστη: «την 5 τη μάζεψε αλλά δεν τη μέτρησε»)**:
  τα probes του stop 5 δείχνουν την ball_03 να μπαίνει στο στόμιο (0.65 m
  μπροστά, capture) και να ΕΚΤΡΕΠΕΤΑΙ πλάγια (ly 0.40→1.10 σε 4 s) — μπήκε
  και βγήκε, δεν μετρήθηκε σωστά (δεν μπήκε ποτέ onboard). Και στο stop 6
  αποκαλύφθηκε χειρότερο μοτίβο: **η εγγραφή του χάρτη «σερνόταν» μέσω
  chain-merges σε γειτονικές μπάλες** (lock_error 0.1→4.0, το approach την
  ακολούθησε 4.7 m βόρεια μέχρι το timeout — η merge_distance μεγαλώνει
  έως 1.6 m σε απόσταση, οπότε γειτονικές μπάλες συγχωνεύονται στην ίδια
  εγγραφή όσο το ρομπότ κινείται). **Fix**: `_GOAL_DRIFT_ABANDON_M=1.5` —
  εγγραφή που έχει απομακρυνθεί >1.5 m από την plan-time θέση της δεν
  είναι πια η ίδια μπάλα: το stop εγκαταλείπεται (event `route_ball_lost/
  map_entry_drifted`) αντί να κυνηγιέται. Το RouteStop απέκτησε
  planned_x/y. Tests 99 passed. Το target-aware observation του κυρίως #8
  αντιμετωπίζει και το lock-drift στο approach.

### 9. Run 5: καταρράκτης ακαριαίων nav απορρίψεων — start pose μέσα στο inflation

- **Run 5** (fixes #8 + abandon cap): 3 πρώτες μπάλες καθαρές (goal
  refreshes δούλεψαν — 2×`route_goal_updated` στο stop 3 ακολούθησαν τη
  μετατοπισμένη μπάλα), 1 lateral insertion. Το stop 3 έληξε missing μετά
  από retry (drift <1.5 m, δεν κόπηκε από το cap). ΜΕΤΑ: **κάθε επόμενο
  Nav2 goal απορρίφθηκε ακαριαία** (3 «αποτυχίες» σε ~0.1 s ανά stop) και
  το mission διέτρεξε τα 8 υπόλοιπα stops σε 1.3 s → route_complete 3/12
  με 8 skipped.
- **Διάγνωση**: το ρομπότ κατέληξε (κυνηγώντας το stop 3 προς τη ζώνη του
  φιλέ, τελευταία θέση ~(6.5, 3)) με το footprint **μέσα στο inflation
  του costmap** — ο Smac planner απορρίπτει ακαριαία ΚΑΘΕ goal όταν το
  start είναι σε lethal/inflated cost, ακόμα και προς ελεύθερο χώρο.
  Επιπλέον το lateral goal του stop 14 (standoff 0.36 m από το φιλέ —
  ακριβώς το robot_radius) πέφτει ΜΟΝΙΜΑ μέσα στο inflation band του Nav2
  → εγγενώς άκυρο goal.
- **Διορθώσεις (100 tests)**:
  - **Recovery αντί για cascade**: nav αποτυχία με leg elapsed <
    `_NAV_INSTANT_FAIL_S` (2 s) = απόρριψη planner, ΟΧΙ γνήσια αποτυχία
    πλοήγησης → νέο phase `recover`: όπισθεν ευθεία 2.5 s @ 0.15 m/s
    (έξοδος από το inflation) και επανέκδοση του goal· έως 2 recoveries
    ανά stop (event `route_nav_recovery`), μετά ο κανονικός
    retry/skip δρόμος. Το retry budget δεν καίγεται από απορρίψεις.
  - **Goal clearance margin**: τα lateral standoffs απαιτούν πλέον
    `robot_radius + COLLECT_ROUTE_GOAL_CLEARANCE_M` (0.15 → σύνολο 0.51 m)
    από φιλέ/φράχτη/εμπόδια ώστε να μην γεννιούνται goals μέσα στο
    inflation band.
- **Σημείωση**: το reverse του recovery είναι «τυφλό» (το LiDAR είναι
  πίσω-τοποθετημένο αλλά δεν ελέγχεται εδώ)· 0.375 m σε χώρο που μόλις
  διέσχισε το ρομπότ — αποδεκτό στο sim, να επανεξεταστεί για hardware.
- **Status**: ✅ κώδικας· ⏳ run 6.

### 10. Run 6: ύποπτη ΑΠΟΚΛΙΣΗ LOCALIZATION — pose watchdog + cascade abort

- **Run 6** (fixes #9): 4 πρώτες μπάλες άψογες — και το πρώτο
  **`zone=bin` credit** (μπάλα #3, launch κατευθείαν στο καλάθι — το
  funnel #59 + goal-following δούλεψαν). ΜΕΤΑ: το Nav2 leg προς το stop 12
  απέτυχε μετά από 17 s («Failed to make progress» στα docker logs), και
  ακολούθησε καταρράκτης ακαριαίων απορρίψεων. Το recovery του #9
  ενεργοποιήθηκε (2 ανά stop) αλλά ΔΕΝ ξεμπλόκαρε — 8 τυφλές όπισθεν
  συνολικά περπάτησαν το ρομπότ ~3 m ΒΑ, με το status να το δείχνει στο
  (5.4, 7.8) = σχεδόν στον βόρειο φράχτη.
- **Κρίσιμη μαρτυρία χρήστη**: το reverse ξεκίνησε ΧΩΡΙΣ το ρομπότ να
  είναι κοντά σε φιλέ/φράχτη — δηλαδή η ΠΕΠΟΙΘΗΣΗ θέσης και η
  πραγματικότητα πιθανόν αποκλίνουν (SLAM localization jump), Ή το ρομπότ
  σκάλωσε φυσικά σε κάτι αόρατο για το LiDAR (το mesh του φιλέ;) και τα
  «Failed to make progress» + οι όπισθεν το μετατόπισαν. ΔΕΝ κρίνεται από
  τα υπάρχοντα δεδομένα — τα /sim/balls είναι re-projected στο believed
  frame, οπότε το lock_error_m ΔΕΝ πιάνει pose drift (ακυρώνεται).
- **Instrumentation (υλοποιήθηκε)**: το gazebo_extras δημοσιεύει
  `/sim/robot_true_pose` (καθαρό world-frame ground truth από gz
  pose/info)· ο controller υπολογίζει `pose_error_m` (believed vs truth,
  έγκυρο γιατί map frame ≈ world frame από το survey start) σε κάθε
  probe + στο status, και εκπέμπει **`pose_divergence`** event όταν
  ξεπεράσει το 1.0 m (throttled 5 s).
- **Cascade abort (υλοποιήθηκε)**: 2 συνεχόμενα stops που χάνονται από
  nav failures → **`route_aborted{nav_rejected_cascade}`** και το mission
  σταματά loud, αφήνοντας τα υπόλοιπα stops pending — όχι burning-through
  (και όχι ατέρμονες τυφλές όπισθεν· το run-6 έδειξε ότι το reversing
  ΧΩΡΙΣ αξιόπιστο pose κάνει ζημιά).
- **Status**: ✅ κώδικας (101 tests)· ⏳ run 7 → το pose_error_m θα
  ξεχωρίσει οριστικά localization drift vs φυσικό σκάλωμα, και ανάλογα
  ανοίγει είτε SLAM/odom entry είτε net-visibility-στο-LiDAR entry.

### 11. Run 7: LOCALIZATION ΥΓΙΕΣ (pose_error σταθερό) — ο ένοχος είναι φυσικό μπλοκάρισμα στα Nav2 legs

- **Run 7**: 4/4 πρώτες μπάλες (2 από αυτές **zone=bin** — το funnel
  πετάει πλέον κατευθείαν στο καλάθι), και το **cascade abort δούλεψε**:
  σταμάτησε καθαρά (`route_aborted` στα 2 συνεχόμενα nav skips) αντί να
  κάψει το route.
- **ΟΡΙΣΤΙΚΟ (pose watchdog)**: το believed↔truth offset έμεινε
  **8.00 ± 0.12 σε όλο το run** (το 8.0 είναι το σταθερό map↔world frame
  offset — το world είναι court-centred, το map ξεκινά στο spawn).
  **ΚΑΝΕΝΑ SLAM drift** — η υπόθεση του pose jump (#10) καταρρίπτεται.
  (Το calibration του offset μπήκε στον κώδικα ώστε το pose_error_m να
  δείχνει καθαρό drift στα επόμενα runs.)
- **Το πραγματικό μοτίβο του μπλοκαρίσματος** (leg προς stop 5/ball 11):
  ο controller_server έστελνε paths επί 15 s με το ρομπότ ΑΚΙΝΗΤΟ και στο
  ground truth (5.68,3.07 believed / −2.33,3.05 true, creeping ~0.2 m),
  «Failed to make progress» → BT clear local costmap → retry → abort.
  Adapter/twist_mux σωστά (zero-once → σιωπή → Nav2 ο νικητής — γι' αυτό
  τα legs 1-4 δούλευαν).
- **Κύρια υπόθεση πλέον: μπάλα σφηνωμένη κάτω από το σασί.** Ο Nav2 ΔΕΝ
  βλέπει μπάλες (LiDAR plane z≈0.55, μπάλα 0.066) — τα legs περνούν ΜΕΣΑ
  από θέσεις μπαλών· σε πυκνή περιοχή (εκεί που καταρρέουν όλα τα runs,
  μετά την 4η-5η) κάποια μπάλα μαγκώνει κάτω από το πλαίσιο/τροχό.
  Συμβατό και με το drift της εγγραφής 11 (goal_updated ×3 — η μπάλα
  σπρώχνεται/μετακινείται από το ίδιο το ρομπότ).
- **Instrumentation**: τα `route_capture_probe` τρέχουν πλέον ΚΑΙ στη
  φάση nav (με `mission_phase` field) — στο επόμενο πάγωμα θα δείξουν την
  πλησιέστερη φυσική μπάλα σε robot frame (αναμένεται lx≈0-0.3, z=0.033
  αν σφηνώνει από κάτω).
- **Πιθανό διαρθρωτικό fix (προς συζήτηση)**: opportunistic capture στα
  legs — μπάλα ορατή μπροστά σε <1.2 m κατά το nav → cancel goal, capture
  (γίνεται το τρέχον stop μέσω insertion), συνέχεια route. Μετατρέπει τις
  συγκρούσεις σε συλλογές και είναι και ταχύτερο. Εναλλακτικά: κράτημα
  απόστασης από χαρτογραφημένες μπάλες στα legs (χρειάζεται keepout/
  costmap injection — βαρύτερο).
- **Status**: ✅ instrumentation· ⏳ run 8 για την απόδειξη του «από
  κάτω», και απόφαση χρήστη για το opportunistic capture.

### 12. Opportunistic capture στα Nav2 legs (fix για τα reverses/χαμένο plan)

- **Αναφορά χρήστη (run 7)**: «πάλι έβαλε reverse χωρίς λόγο και έχασε
  το plan» — τα τυφλά recovery reverses + το cascade abort είναι κακή
  απάντηση όταν η ρίζα είναι μπάλες στην πορεία των legs που ο Nav2 δεν
  βλέπει.
- **Υλοποίηση**: νέο mission phase `opportunistic` — όταν στη διάρκεια
  Nav2 leg η κάμερα δει μπάλα μπροστά (≤1.2 m, |bearing| ≤ 40°):
  1. Ακυρώνεται το goal, ο collector την μαζεύει επιτόπου (lock/refresh
     όπως στο approach, timeout 15 s).
  2. Στο confirm: πιστώνεται το pending stop της οποίας η planned μπάλα
     είναι εντός 0.8 m (αν υπάρχει) — αν ήταν του ΤΡΕΧΟΝΤΟΣ stop → settle
     και κανονική συνέχεια· αλλιώς το leg επανεκδίδεται και το route
     συνεχίζει με το stop της κομμένο από τη λίστα.
  3. Σε αποτυχία/timeout: `route_opportunistic_abort` και το leg
     συνεχίζει (η μπάλα μένει στον χάρτη).
  - Events: `route_opportunistic_start/collected/abort`. Στα nav legs το
    observation είναι πλέον το πλησιέστερο-στο-ρομπότ (το target-aware
    ισχύει στο approach/opportunistic).
- **Αναμενόμενο αποτέλεσμα**: οι «αόρατες» μπάλες των legs γίνονται
  συλλογές αντί για σφηνώματα → λιγότερα «failed to make progress»,
  λιγότερα recovery reverses, και ταχύτερο συνολικό μάζεμα.
- **Status**: ✅ κώδικας (104 tests)· ⏳ run 8 (restart stack για rebuild).

### 13. Πολιτική πλάνου (απόφαση χρήστη): ledger, plan-only, συνέχιση χωρίς abort

- **Απόφαση χρήστη**: (α) το mission κρατά ΣΥΝΟΛΟ πλάνου = μπάλες του 360°
  scan + insertions· αφαιρείται ό,τι μαζεύεται· (β) μπάλα μαζεύεται ΜΟΝΟ
  αν ανήκει στο πλάνο (αρχικό ή insertion) — καμία παρέκκλιση για εκτός
  πλάνου· (γ) σε προβλήματα navigation, η προβληματική μπάλα καταχωρείται
  στις ΑΠΟΤΥΧΙΕΣ και το ΙΔΙΟ πλάνο συνεχίζει από την επόμενη μπάλα· (δ)
  ολοκλήρωση = όταν κάθε μπάλα του πλάνου έχει λογαριαστεί
  (collected/failed) → event completed.
- **Υλοποίηση**:
  - Το `route_aborted`/cascade abort του #10 ΚΑΤΑΡΓΗΘΗΚΕ — nav skip →
    καταχώρηση αποτυχίας → επόμενο stop του ίδιου πλάνου.
  - Blind reverse recoveries: ΣΥΝΟΛΙΚΟ budget 4/run (πέρα από το 2/stop) —
    μετά, οι αποτυχίες απλά προσπερνιούνται (όχι fence-walking του run 6).
  - Opportunistic capture: **plan-only** — ενεργοποιείται μόνο όταν η
    μπάλα μπροστά ταιριάζει (≤0.8 m) με pending/active stop· τα
    confirmed νέα μπαίνουν στο πλάνο μέσω insertion ούτως ή άλλως, τα
    αχαρτογράφητα στιγμιαία sightings αγνοούνται.
  - Ledger στο telemetry: `planned_total`, `remaining`, `failed_ball_ids`·
    το `route_complete` αναφέρει planned_total/collected/skipped/missing/
    failed_ball_ids/insertions.
- **Status**: ✅ κώδικας (105 tests)· ⏳ run 8.

### 14. Code review του frozen-plan diff: 4 στοχευμένα fixes πριν το verification run

- **Πλαίσιο**: review του working-tree diff (freeze initial plan, retention-based
  credit). 10 επιβεβαιωμένα ευρήματα (8 correctness, 2 cleanup) — εφαρμόστηκαν
  ΜΟΝΟ όσα επηρεάζουν την αξιολόγηση της εκτέλεσης του παγωμένου πλάνου στο
  sim. Το nav-reject cascade (χωρίς reverse recovery + NAV_RETRIES=0)
  αφέθηκε συνειδητά: το reverse recovery είχε false triggers και αφαιρέθηκε
  με απόφαση χρήστη· αποδεκτό ρίσκο για το run.
- **Fixes**:
  1. `ball_map.update`: το absorb από terminal entries (collected/
     collection_failed) γίνεται μόνο όταν το terminal είναι το ΠΛΗΣΙΕΣΤΕΡΟ
     match. Πριν, ghost εντός merge_dist (0.65–1.6 m) «έκλεβε» τα
     observations πλησιέστερης ενεργής μπάλας (χαλούσε relock/refresh σε
     πυκνά clusters) και μπλόκαρε τη δημιουργία νέων entries σε ΟΛΑ τα modes.
  2. Delayed attribution (approach + opportunistic): καθυστερημένο retention
     παλιότερης μπάλας πιστώνεται ΧΩΡΙΣ να διακόπτει το τρέχον capture —
     πριν, γινόταν behavior.reset, έπεφτε το `_opp_locked` και επανεκδιδόταν
     το Nav2 leg ενώ η ζωντανή μπάλα ήταν μισο-πιασμένη στο funnel.
  3. `capture_ball_id`: owner binding μόνο σε approach/settle/opportunistic —
     όχι κατά το nav leg, ώστε μπάλα που «κλωτσιέται» onboard στη διαδρομή να
     μην πιστωθεί στο μη-προσεγγισμένο ακόμα stop.
  4. `_assign_sim_ball_route_owners`: η ζώνη deck ΔΕΝ ανανεώνει πλέον το
     capture-pending grace (deck ≠ δρόμος προς bin, |y|>0.14) — το missing
     decision μένει στο 6 s phantom gate αντί να κολλάει ως το 35 s budget.
- **Καταγεγραμμένα, εκτός scope (σκόπιμα, για μετά το run)**: hardware paths
  (IR latch χωρίς dwell πίσω από το χείλος· capture_pending μόνο από sim
  ground truth· διπλό credit mark_nearest_collected+set_state)· deck-credit
  regression των collect/collect_one· efficiency (telemetry rebuild ανά
  event, route_stops σε κάθε event του deque, 3x sim-ball transforms/tick)·
  dead code στο control panel (renderCollectionTruth, ορφανό nav_test).
- **Status**: ✅ κώδικας (116 tests, το delayed-attribution test ενημερώθηκε
  να απαιτεί συνέχιση του capture)· ⏳ sim verification run του frozen πλάνου.

### 15. Run 8: μπάλες στο καλάθι αμέτρητες — frame mismatch odom↔map στο zone scoring

- **Παρατήρηση (run 1784265681-454, 2026-07-17)**: counter 4, αλλά 7 μπάλες
  φυσικά στο καλάθι (gz ground truth: ball_02/09/13/06 μετρημένες +
  ball_00/12/05 αμέτρητες). Τα stops 12 (ball_00) και 8 (ball_05) βγήκαν
  «missing» ενώ οι μπάλες τους ήταν ήδη μέσα → το πλάνο κυνηγούσε ghosts.
- **Root cause**: το `gazebo_extras._gz_point_to_odom` αγκυρώνει τις
  ground-truth θέσεις μπαλών στο **odom** pose, ενώ ο controller υπολογίζει
  τις onboard ζώνες με το **map/slam_tf** pose. Σφάλμα στα robot-local =
  απόκλιση odom↔map (skid-steer drift). Αρχή run ≈0 → μπάλες 1-4 σωστές·
  από t≈110 s (drift 0.3-1.0 m) μπάλα στο καλάθι (true local ~0.25 m)
  υπολογιζόταν local έως (-0.8, 0) → εκτός ζωνών → ποτέ bin/retained.
  Απόδειξη: probe ball_05 @235.2 s local (-0.816) με z=0.058 (ύψος πάτου
  καλαθιού — court rest είναι 0.033). Τα «deck» owner assignments των
  ball_00/12/05 ήταν μπάλες στο σκούπισμα, μετατοπισμένες από το drift.
- **Δευτερεύον**: το `_check_collection` έκοβε ΟΛΟ το sim tracking όταν το
  intake ήταν off → μπάλα που κάθεται στο bin αμέσως μετά από
  opportunistic abort (roller off) δεν παρακολουθιόταν καν.
- **Fixes**:
  1. `gazebo_extras`: το /sim/balls δημοσιεύει πλέον και **true robot-local**
     συντεταγμένες (`local_x/local_y/local_z`, ball+robot pose από το ίδιο gz
     snapshot — ανεξάρτητες από κάθε drift). Το x/y μένει odom-anchored για
     τους παλιούς καταναλωτές.
  2. `controller`: νέος κοινός helper `_sim_ball_local()` (προτιμά τα
     ground-truth locals, fallback ο παλιός map-frame μετασχηματισμός για
     παλιά fixtures) — αντικατέστησε και τα 4 αντιγραμμένα transform loops
     (bonus: το reuse εύρημα του review).
  3. `_check_collection`: το sim ground-truth path τρέχει ΑΝΕΞΑΡΤΗΤΑ από το
     intake_enabled — το retention μετά από abort πιστώνεται.
  4. `_nav_phase`: δέχεται delayed confirmation (μόνο με ρητό
     confirmed_ball_id): πιστώνει το stop, και αν είναι το ΤΡΕΧΟΝ stop
     παρακάμπτει το leg προς το ghost standoff (route_advance με collected).
- **Σημείωση εγκυρότητας δεδομένων**: το pose/info του Gazebo Harmonic
  εκπέμπει ΟΛΕΣ τις οντότητες (και κοιμισμένες) — το σχόλιο «only moved
  entities» δεν ισχύει εδώ· το merge cache παραμένει ως άμυνα.
- **Status**: ✅ κώδικας (118 tests, +2 nav-phase delayed credit)· ⏳ run 9
  (θέλει rebuild/restart του stack — gazebo_extras + controller άλλαξαν).

### 16. Run 9 ΕΠΙΤΥΧΕΣ (σωστή λογιστική) → beam-primary confirmation + truth referee

- **Run 9 (1784270673-460, 2026-07-17)**: route_complete — planned 12,
  collected 7, missing 5 (3, 9, 10, 6, 5), skipped 0, insertions 0. Το frame
  fix του #15 επιβεβαιώθηκε: **counter = μπάλες στο καλάθι σε όλο το run**
  (7=7), πλήρεις αλυσίδες entry→bin→retained→credit με σωστά locals βαθιά στο
  run (t=305 s), pose error 0.02. Πρώτο τεκμηριωμένα σωστό run του frozen
  360° πλάνου: ολοκλήρωση by exhaustion με ειλικρινές ledger.
- **Ανοιχτό (φυσικό, όχι λογιστικό)**: και τα 5 misses ίδιο μοτίβο — το
  opportunistic chase ΣΠΡΩΧΝΕΙ τη μπάλα με το χωνί αντί να τη συλλάβει
  (π.χ. ball 3/ball_06: 15 s κυνήγι, κλωτσήθηκε 4+ m εκτός playable area,
  μετά το frozen standoff ήταν άδειο → missing). Επόμενο behavioral θέμα:
  ευθυγράμμιση/ταχύτητα του chase — ΔΕΝ αφορά scoring.
- **Beam-primary (απόφαση χρήστη — sim και hardware να κρίνονται από το
  ΙΔΙΟ σήμα)**:
  - `SIM_COLLECTION_CONFIRM_SOURCE=beam` (default): το collection_confirmed
    στο sim είναι πλέον το basket IR latch (τα sim beams τροφοδοτούν ήδη το
    /ir/readings) — ίδιος κώδικας με hardware. `=truth` επαναφέρει το
    ground-truth dwell (debug fallback).
  - Ground truth → διαιτητής: `_sim_retention_step(credit=False)` συνεχίζει
    zones/retention events, και ο νέος `CreditReconciler`
    (collection_scoring) συγκρίνει beam vs truth counts· επίμονη απόκλιση
    >5 s → `beam_false_credit` / `beam_missed_credit` (critical). Το
    beam-μπλοκαρισμένο-από-γεμάτο-καλάθι θα φανεί ως beam_missed_credit.
  - Hardware parity: το capture-pending deferral απενεργό σε beam mode
    (κανένα ground-truth σήμα στον έλεγχο)· νέο event
    `beam_collection_credit` στο latch.
  - Fix του review #3 (διπλό credit): σε collect_route χωρίς ground-truth
    id, ο controller ΔΕΝ κάνει πια mark_nearest_collected — το mission
    αποδίδει το credit στο δικό του stop (μοναδική απόδοση ανά capture).
- **Status**: ✅ κώδικας (121 tests, +3 CreditReconciler)· ⏳ run 10
  (beam-primary πιστοποίηση του hardware confirmation pipeline).

### 17. Run 10 (beam-primary): ο διαιτητής έπιασε τα ψέματα του beam — debounce + επόμενο βήμα route

- **Run 10 (1784276141-459)**: 15 beam credits, 2 truth retained (delta +13,
  όλα flagged ως beam_false_credit από τον CreditReconciler — ο διαιτητής
  του #16 δούλεψε από το πρώτο run).
- **Ανάλυση τιμών ray**: (α) πραγματικές διελεύσεις = συμμετρικές τιμές
  (652/646 → επιφάνεια μπάλας 77 mm από κάθε sensor, κεντραρισμένη στο
  tray) αλλά ΔΙΠΛΟμετρημένες — το latch καθάριζε στιγμιαία στο αναπήδημα·
  (β) ψεύτικα credits = μονόπλευρες τιμές (899/352) ΤΗ ΣΤΙΓΜΗ opportunistic
  chase: η μπάλα διέσχισε το επίπεδο του beam και ΒΓΗΚΕ ξανά (launch πάνω σε
  κινούμενο/στρίβον ρομπότ → αναπήδηση έξω). Το beam είναι ανιχνευτής
  ΔΙΕΛΕΥΣΗΣ, όχι παραμονής — το ίδιο ψέμα θα έλεγε και στο hardware. Το
  mission πίστωνε το stop και έφευγε αφήνοντας τη μπάλα.
- **Fixes τώρα**: (1) beam debounce/re-arm (BEAM_REARM_QUIET_S=0.6 s μετά το
  καθάρισμα πριν νέο count — μία διέλευση=ένα credit)· (2) το panel
  Collection Run δείχνει «Beam / truth (sim)» ώστε η απόκλιση να είναι
  ορατή live.
- **Επόμενα (προτάσεις, εκκρεμεί απόφαση)**: (α) route που βοηθά τη συλλογή
  χωρίς διορθώσεις — κατάργηση του επιθετικού opportunistic chase (πηγή και
  των 5 misses του run 9 ΚΑΙ των 5 bounce-outs του run 10): μπάλα στην
  πορεία του leg γίνεται κανονικό επόμενο stop με ευθύ fine approach·
  approach yaw ευθυγραμμισμένο με το leg (ελάχιστη επιτόπια περιστροφή
  κοντά σε pending μπάλες)· legs που αποφεύγουν διαδρόμους πάνω από άλλες
  planned μπάλες. (β) staggered δεύτερο beam (entry x≈0.40 + retention
  x≈0.28): κατεύθυνση διέλευσης → «μπήκε ΚΑΙ έμεινε» — ίδια λύση sim και
  hardware (λύνει και το full-basket blocking ως ανιχνεύσιμο σήμα).
- **Status**: ✅ debounce + panel (tests πράσινα)· ⏳ απόφαση χρήστη για
  route redesign + staggered beams.

### 18. Κανόνες βέλτιστης διαδρομής (R1-R5) + R2: promotion αντί για chase

- **Απόφαση χρήστη**: το route να συμπεριλαμβάνει και τον ΤΡΟΠΟ συλλογής —
  κοντά σε εμπόδιο πάντα πλαϊνή/παράλληλη προσέγγιση· κανόνες βέλτιστης
  διαδρομής. Πλήρης λίστα: docs/collection-route/collection-route-rules-el.md (R1 παράλληλη
  ήδη υπήρχε στον planner· R3 drive-through, R4 rotation clearance,
  R5 leg-corridor penalty εκκρεμούν).
- **R2 υλοποιήθηκε**: το opportunistic chase ΚΑΤΑΡΓΗΘΗΚΕ ως trigger — planned
  μπάλα στην πορεία του leg γίνεται το ΕΠΟΜΕΝΟ stop (τοπική αναδιάταξη,
  frozen ledger ανέπαφο) και συλλέγεται με κανονικό standoff + ευθύ fine
  approach. Event `route_on_path_promoted`, το προηγούμενο stop σε pending.
  Ίδια μπάλα ορατή ξανά → κανένα churn (δεν ξανα-προωθείται). Το
  `_opportunistic_phase` μένει προσωρινά ως νεκρός κώδικας (τα delayed
  credits πάνε πλέον μέσω approach/nav phase).
- **Αναμενόμενο**: εξαφάνιση των punts (run 9) και των bounce-outs/ψεύτικων
  beam credits (run 10) — όλα τα captures πλέον στάσιμα/ευθεία.
- **Status**: ✅ κώδικας (121 tests· 3 opportunistic tests ξαναγράφτηκαν για
  promotion)· ⏳ run 11 (beam-primary + R2)· ⏳ R3/R4/R5.

### 19. Run 11: το intake αθώο — το beam έβλεπε μπάλες γηπέδου μέσα από το πλέγμα

- **Run 11 (1784279285-461)**: route 11 collected / 1 missing, truth 7 —
  4 «έξω» (παρατήρηση χρήστη). R2 promotion: κανένα chase/punt ✓.
- **Ανάλυση**: οι 7 πραγματικές συλλήψεις = πλήρης αλυσίδα entry→bin→
  ΣΥΜΜΕΤΡΙΚΟ beam credit (≈640/630)→retained — **7/7 launches πέτυχαν, ο
  μηχανισμός intake ΔΕΝ φταίει**. Τα 4 ψεύτικα credits (59.8/118.9/149.6/
  194.1 s): ΚΑΝΕΝΑ entry/bin candidate + ΜΟΝΟΠΛΕΥΡΟ ir (901/233, 178/514,
  497/682, 857/414 → αντικείμενο 2-11 cm από το ένα sensor) = **μπάλα
  γηπέδου δίπλα στο ρομπότ ορατή μέσα από το συρμάτινο πλέγμα**: beam z
  0.063, κορυφή μπάλας εδάφους 0.066. Το stop πιστωνόταν καθώς το ρομπότ
  περνούσε δίπλα από μπάλα — γι' αυτό έμειναν 4 έξω με stops "collected".
- **Fix**: basket_ir_z +0.033→+0.045 (beam 0.075 από το court: πάνω από
  κορυφή μπάλας εδάφους 0.066, κάτω από κορυφή μπάλας στο tray 0.096)·
  ίδια αλλαγή στο visual confirmation_beam_z (basket.urdf.xacro).
- **Παρατήρηση για hardware**: σε πραγματικό συρμάτινο καλάθι ισχύει το
  ίδιο — το beam πρέπει να είναι πάνω από ύψος μπάλας εδάφους ή με σκίαση
  (solid bracket) προς τα έξω.
- **Status**: ✅ κώδικας/urdf (tests πράσινα)· ⏳ run 12 (στόχος: beam =
  truth = καλάθι, 0 false credits).

### 20. Run 12: ψεύτικο credit από αναπηδώσα μπάλα → beam symmetry gate

- **Run 12 (1784290711-462)**: μπάλες 1-2 καθαρές· t=61.0 ψεύτικο credit
  (stop 4 «collected» ενώ το ρομπότ απλά πέρασε δίπλα — παρατήρηση χρήστη):
  ir 869/576, ΚΑΝΕΝΑ entry/bin candidate = μπάλα που αναπηδά δίπλα στο
  ρομπότ, ορατή μέσα από το πλέγμα. Το ύψος 0.075 (#19) κάλυψε στατικές
  μπάλες εδάφους, όχι αναπηδήσεις.
- **Διαχωριστικό από τα δεδομένα**: πραγματικές διελεύσεις = ΣΥΜΜΕΤΡΙΚΑ ir
  (631/625, 638/633, 635/645 — κεντραρισμένη μπάλα στο tray)· όλες οι
  ψεύτικες μονόπλευρες/ασύμμετρες (869/576, 901/233, 178/514, 497/682).
- **Fix**: το latch απαιτεί ΚΑΙ τα δύο >500 ΚΑΙ |L-R| ≤ 200
  (BEAM_SYMMETRY_MAX_DELTA). Όλα τα πραγματικά credits των runs 11-12
  περνούν (diff ≤13), όλα τα ψεύτικα κόβονται.
- **Εκκρεμεί (σημείωση χρήστη)**: περιστροφή/καθυστέρηση στην εκκίνηση κάθε
  leg (Nav2 rotate-to-path) → R3 drive-through ordering.
- **Status**: ✅ κώδικας (tests πράσινα)· ⏳ run 13.

### 21. ΑΠΟΦΑΣΗ ΧΡΗΣΤΗ: διαχωρισμός συλλογής από διαδρομή (sweep route)

- Η διαδρομή σχεδιάζεται ΜΙΑ φορά από τις θέσεις του 360 (+costmap για τον
  χώρο) ως συνεχές πέρασμα: κάθε μπάλα πρέπει να βρίσκεται ΣΤΟ ΧΩΝΙ (funnel
  corridor) καθώς το ρομπότ περνάει — through-poses, όχι στάσεις.
- ΚΑΜΙΑ σύνδεση συλλογής-διαδρομής: χωρίς stop/fine approach/settle/retry
  ανά μπάλα· intake συνεχώς ενεργό· η διαδρομή ολοκληρώνεται ανεξάρτητα από
  το αν μαζεύτηκε κάθε μπάλα. Beam(+referee) απλώς μετράνε· ό,τι μείνει →
  αναφορά στο τέλος (μελλοντικά δεύτερο πέρασμα).
- Λύνει: καθυστερήσεις μπάλα-σε-μπάλα (καμία στάση/στροφή), ledger απλό
  (μέτρημα, όχι πίστωση stop), τα ψεύτικα credits δεν εκτρέπουν πλέον
  τίποτα. Εκκρεμεί: υλοποίηση sweep planner + εκτέλεση (NavigateThroughPoses
  ή διαδοχικά goals χωρίς παύση).

### 22. Sweep mode υλοποιημένο: μία συνεχής διαδρομή, συλλογή αποσυνδεδεμένη

- **Planner** (log #21, task 1): `sweep_route()` — αλυσιδωτά drive-through
  legs: ευθύ run-in 1.0 m πριν από κάθε μπάλα (κεντραρισμένη στο χωνί),
  exit 0.35 m μετά, το επόμενο leg ξεκινά από την έξοδο του προηγούμενου
  (μηδενικές επιτόπιες στροφές)· heading από approach_pose_for_ball
  (incoming ή obstacle-parallel R1).
- **Mission** (task 2): `COLLECT_ROUTE_SWEEP` (default true)· τα RouteStops
  επαναχρησιμοποιούνται με approach = EXIT pose (goal ΠΕΡΑ από τη μπάλα).
  `_sweep_drive`: reached ή απόσταση ≤0.45 m → αμέσως επόμενο goal — καμία
  στάση/approach/settle/retry/promotion· nav failure/timeout → skip και
  συνέχεια· intake ενεργό σε όλο το drive (SURVEY + lift_wheel_speed).
  Beam credit (rising edge) → πίστωση στην πλησιέστερη un-collected planned
  μπάλα ≤1.5 m (μόνο για αναφορά — η ροή δεν επηρεάζεται ποτέ). Πέρασμα
  χωρίς credit → status "swept" → μετρά στα failed του route_complete
  (νέο πεδίο swept_uncollected). Νέα events: route_ball_swept.
- **Status**: ✅ κώδικας (126 tests, 5 νέα sweep)· ⏳ run 14 (πρώτο sweep
  run — στόχος: συνεχής ροή χωρίς παύσεις, Beam=truth, αναφορά τίμια)·
  ⏳ task 3: costmap validation των poses + console panel.

### 23. Sweep pass: το πέρασμα το οδηγεί το mission, όχι ο Nav2

- **Παρατήρηση (χρήστης, run 15)**: οι συνεχείς διορθώσεις πορείας του Nav2
  μέχρι το exit pose χτυπούσαν τη μπάλα με τα μάγουλα του χωνιού και την
  έδιωχναν.
- **Αλλαγή**: το Nav2 goal κάθε sweep leg είναι πλέον το ENTRY (1.0 m πριν
  τη μπάλα, yaw προς αυτήν). Στο reached/≤0.45 m: το goal ακυρώνεται και το
  mission οδηγεί το πέρασμα (`_sweep_pass_tick`): ευθεία με
  COLLECT_ROUTE_SWEEP_PASS_SPEED_M_S=0.35· μικρή διόρθωση heading ΜΟΝΟ όσο
  η μπάλα απέχει >0.6 m (gain 1.2, cap 0.5 rad/s)· μέσα στο τελευταίο 0.6 m
  το heading ΠΑΓΩΝΕΙ — το κεντράρισμα το κάνει το χωνί (#60). Ολοκλήρωση
  pass: πρόοδος ≥0.35 m μετά τη μπάλα (ή timeout 10 s) → αμέσως επόμενο
  entry goal. Νέο event: route_pass_start.
- **Status**: ✅ κώδικας (126 tests)· ⏳ run 16 (sweep + στενό funnel #60 +
  ευθύ pass — στόχος: καθαρές διελεύσεις χωρίς εκτροπές).
