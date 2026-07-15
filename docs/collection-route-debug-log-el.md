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
