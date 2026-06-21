# Court Survey v2 — LiDAR Occupancy → Court Knowledge Model (as-built)

> Η **έγκυρη, τελική** προδιαγραφή του survey. Αντικατέστησε το dead-reckoning
> perimeter FSM (εύθραυστο) και είναι πλέον **υλοποιημένη και επικυρωμένη στο
> Gazebo**: Map Court → πλήρες `court_boundary.json` με σωστές αποστάσεις, καθαρό
> χάρτη και 0 ψεύτικα obstacles σε άδειο γήπεδο. Όλα τα προηγούμενα survey
> documents (perimeter / Nav2-explore / FSM-fix) είναι ξεπερασμένα.

## 1. Σκοπός & αρχές

**Σκοπός:** Στο τέλος της χαρτογράφησης το robot γνωρίζει:

1. Πού είναι οι **φράχτες** (εξωτερικό όριο).
2. Πόσο **απέχουν οι φράχτες από τις γραμμές** (run-off ανά πλευρά).
3. Πού είναι το **δίχτυ** και οι **δύο στύλοι** του.
4. Κάθε **άλλο εμπόδιο εντός των φραχτών** (πάγκοι, στύλοι) — θέση + μέγεθος.

Η κίνηση είναι το **μέσο**, όχι ο σκοπός: το ζητούμενο είναι η ΜΕΤΡΗΣΗ.

**Αρχές σχεδίασης (μη διαπραγματεύσιμες):**

- **LiDAR-first.** Ό,τι είναι κάθετο (φράχτες, δίχτυ, στύλοι, εμπόδια) και κάθε
  απόσταση **μετριέται από το 360° LiDAR**. Η κάμερα OAK-D χρησιμοποιείται στη Φάση 1
  **μόνο για την επιβεβαίωση του φιλέ** (net classification που σκανδαλίζει το net
  lock μέσω `/survey/vision`)· η θέση/απόσταση του φιλέ έρχεται από το LiDAR. Καμία
  γεωμετρία δεν προκύπτει από την κάμερα.
- **Μέτρηση από τον χάρτη, όχι από στιγμιαία pose.** Οι αποστάσεις είναι διαφορές
  παγκόσμιων θέσεων στον συσσωρευμένο occupancy χάρτη.
- **Standard διαστάσεις για τις γραμμές.** Οι γραμμές (αόρατες στο LiDAR) προκύπτουν
  από το δίχτυ ως άγκυρα + κανονιστικές διαστάσεις ITF.
- **Ίδιος κώδικας σε Gazebo & πραγματικό ρομπότ.** Διαφορές μόνο μέσω env vars/topics.
- **ΚΑΘΑΡΗ ΥΛΟΠΟΙΗΣΗ — ΧΩΡΙΣ FALLBACKS.** Καμία σιωπηλή εκτίμηση/default. Αν ένα βήμα
  δεν έχει αρκετά δεδομένα ή αποτυγχάνει δομικός έλεγχος, το survey **αποτυγχάνει
  ρητά** (fail-loud) με σαφή αιτία και ΔΕΝ γράφει επινοημένο boundary. Η μόνη
  «σταθερά» που επιτρέπεται είναι οι **κανονιστικές διαστάσεις γηπέδου**.

## 2. Κανονιστικές διαστάσεις (ITF) — η μόνη επιτρεπτή σταθερά

| Στοιχείο | Τιμή |
| --- | --- |
| Μήκος γηπέδου (baseline→baseline) | 23.77 m (half = 11.885 m) |
| Πλάτος doubles | 10.97 m (sideline ±5.485 m) |
| Πλάτος singles | 8.23 m |
| Service line από δίχτυ | 6.40 m |
| Net post span (doubles) | 11.3 m (στύλοι ±5.65 m) |

Tunable μέσω `CourtSpec` / env, με κανονιστικά defaults.

## 3. Αρχιτεκτονική / data flow

```
/scan (360° LiDAR) ─┐
                    ├─► [1] Occupancy accumulation (map frame, voxel grid)
TF map→<scan frame>─┘            │  → court_survey_live.json (live points)
                                ▼
                       [2] Coverage controller (deterministic drive)
                       8 vantage points στο court frame + return pass
                                │  (μέτρηση κλειδώνει νωρίς, η οδήγηση
                                ▼   συνεχίζει για πλήρη/loop-closed χάρτη)
                       [3] Extraction (pure functions, offline-testable)
                         3a Net+posts → court frame   3d Obstacles (+smart filter)
                         3b Fence rectangle           3e Distances (run-off)
                         3c Court lines (standard)     3g Fail-loud checks
                                │
                                ▼
                       [4] court_boundary.json (Court Knowledge Model v2)
```

Τα βήματα **[3a–3g] είναι καθαρές συναρτήσεις** (`court_extraction.py`) → unit-testable
χωρίς ROS, με synthetic point clouds.

## 4. Coverage controller (`court_survey_v2_node.py` + `court_coverage.py`)

States: `INIT → FIND_NET → COVERAGE → DONE/FAILED`.

- **FIND_NET:** οδήγηση μπροστά μέχρι το δίχτυ· το **net lock** σκανδαλίζεται από την
  OAK-D net classification και παίρνει την απόσταση/θέση του φιλέ από το μπροστινό
  LiDAR range → ορίζει το court frame (origin = net center, +x' = robot→net, +y' =
  κατά μήκος του διχτιού).
- **Deterministic drive-to-waypoint** (closed-loop στο SLAM pose) — **όχι Nav2**, που
  αποδείχθηκε ασταθές run-to-run. 8 waypoints στο court frame:
  1. `(-deep, 0)` βαθιά κοντινό μισό → πυκνός κοντινός φράχτης
  2. `(-2, +gap)` έξω στο gap (κοντινή πλευρά)
  3. `(+2, +gap)` πέρασμα x'=0 από το gap (καθαρά από το δίχτυ)
  4. `(+quarter, 0)` κέντρο μακρινού μισού
  5. `(+deep, 0)` βαθιά μακρινό μισό → πυκνός μακρινός φράχτης
  6–8. **Return pass:** `(+2, -gap) → (-2, -gap) → (-quarter, 0)` ξαναπερνά το δίχτυ
     από το άλλο gap και επιστρέφει στο κοντινό μισό.
  όπου `deep = half_length + 4`, `gap = post_half_span_doubles + 1.05 ≈ 6.7 m`.
- **Fence-approach stop:** όταν το μπροστινό LiDAR δει φράχτη < ~1.4 m ενώ κοιτάζει το
  waypoint, σταματάει. Λαμβάνει υπόψη το **footprint** (το σώμα φτάνει ~0.54 m
  μπροστά από το LiDAR) → πυκνή χαρτογράφηση φράχτη χωρίς τρακάρισμα.
- **Decouple μέτρησης / χάρτη:** η μέτρηση **κλειδώνει στην πρώτη επιτυχή εξαγωγή**
  (ξέρουμε ότι δεν είναι αποτυχία), αλλά το ρομπότ **συνεχίζει** όλη τη διαδρομή και
  ξανα-εξάγει στον πληρέστερο/loop-closed χάρτη· `DONE` μόνο στο τέλος. Fail-loud
  μόνο αν δομική αποτυχία **πριν** κλειδώσει η μέτρηση.

> Η κίνηση δεν χρειάζεται ακρίβεια — μόνο να φέρει τις πλευρές εντός εμβέλειας. Η
> μέτρηση γίνεται από τον χάρτη.

## 5. Extraction — αλγόριθμοι (`court_extraction.py`)

### 3a. Δίχτυ + στύλοι → court frame
- Από `locked_net`: net center `C` (map frame) και κατεύθυνση robot→net = **άξονας
  μήκους** `û`· άξονας πλάτους `v̂ = perp(û)`. Court frame: origin `C`, άξονες `(û, v̂)`.
- **Στύλοι:** το φυσικό γήπεδο είναι **doubles-width**· οι στύλοι (±5.65 m) προκύπτουν
  από **standard geometry** αγκυρωμένη στο μετρημένο net center (όχι από εύθραυστο
  post-span fit). Το singles είναι εσωτερικό subset βαμμένων γραμμών (μελλοντική
  κάμερα).

### 3b. Fence rectangle
- Μετασχηματισμός όλων των map_points σε court frame.
- Προβολή σε `x'`/`y'` → histogram (bins 0.15 m) → οι **δύο ακραίες πυκνές κορυφές** =
  near/far φράχτες (`x_near, x_far`) και side φράχτες (`y_left, y_right`).
- **Coverage gate:** απαιτείται έκταση `≥ half_length + 2` ώστε ο **φράχτης** (πέρα από
  τη baseline) — όχι μόνο η baseline — να είναι μέσα στο πεδίο πριν το fit. Κάθε
  πλευρά απαιτεί `≥ fence_side_min_points`.

### 3c. Court lines (net-anchor, standard — χωρίς κάμερα)
Στο court frame: baselines `x'=±11.885`, service `x'=±6.40`, center line `y'=0`,
sidelines `y'=±5.485` (doubles).

### 3d. Obstacles εντός φραχτών (+ smart fence-artifact filter)
- Υποψήφια = σημεία εντός fence rectangle (margin **0.9 m**) που δεν ανήκουν στη ζώνη
  διχτιού (band **0.8 m** γύρω από x'=0).
- **Clustering** (grid connected cells) → bounding box (θέση + μέγεθος).
- **Smart fence-artifact rejection:** cluster **κοντά** σε φράχτη (< 1.8 m) **και**
  επιμηκυμένο **παράλληλα** σε αυτόν = scatter του φράχτη → απορρίπτεται. Πραγματικό
  εμπόδιο που **προεξέχει προς τα μέσα** (επιμηκυμένο κάθετα) ή είναι μακριά από
  φράχτη → κρατιέται. Αποτέλεσμα: **0 ψεύτικα obstacles** σε άδειο γήπεδο.

### 3e. Distances (run-off) — διαφορές θέσεων στον χάρτη
`near = |x_near| − 11.885`, `far = x_far − 11.885`,
`left = |y_left| − 5.485`, `right = y_right − 5.485`.

### 3f. Singles/doubles
**Bypassed:** `is_doubles = True` πάντα (φυσικό γήπεδο doubles-width). Το run-off
εξακολουθεί να **μετριέται** από τους φράχτες — αυτό είναι που διαφέρει ανά γήπεδο.

### 3g. Fail-loud vs recoverable (η «καρδιά» του no-fallbacks)
Κρίσιμη διάκριση που έλυσε το βασικό bug ολοκλήρωσης:

| Κατάσταση | Ταξινόμηση | Αιτία |
| --- | --- | --- |
| Δίχτυ χωρίς αρκετά σημεία | structural fail-loud | `net_not_observed` |
| Λείπει πλευρά φράχτη / λίγη κάλυψη | **recoverable** → συνέχισε coverage | `coverage_incomplete`, `fence_side_missing` |
| **Αρνητικό** run-off (fit κόλλησε στο δίχτυ πριν χαρτογραφηθεί ο μακρινός φράχτης) | **recoverable** (ΟΧΙ structural) | `coverage_incomplete: <side> fence not yet mapped` |
| **Θετικό** run-off > 12 m | structural fail-loud | `nonstandard_or_bad_fit` |
| Όλα τα vantage points εξαντλήθηκαν χωρίς μέτρηση | structural fail-loud | `coverage_incomplete: all vantage points visited` |

> Ένας φράχτης δεν μπορεί να βρίσκεται **μέσα** στη baseline· αρνητικό run-off σημαίνει
> «δεν χαρτογραφήθηκε ακόμα ο πραγματικός φράχτης» = πρόβλημα κάλυψης, όχι μη-στάνταρ
> γήπεδο. Αυτή η ταξινόμηση εμποδίζει το survey να αυτοκτονήσει πριν φτάσει στον
> μακρινό φράχτη.

## 6. Σχήμα εξόδου — `court_boundary.json` (Court Knowledge Model v2)

```json
{
  "schema": "court_knowledge_model/v2",
  "status": "OK",
  "failure_reason": null,
  "frame": "map",
  "net": {
    "center": {"x_m": 8.04, "y_m": 0.0},
    "axis_length": {"x_m": 1.0, "y_m": 0.0},
    "axis_width":  {"x_m": 0.0, "y_m": 1.0},
    "posts": [{"x_m": 8.04, "y_m": 5.65}, {"x_m": 8.04, "y_m": -5.65}],
    "span_m": 11.3
  },
  "court": {
    "is_doubles": true, "length_m": 23.77, "width_m": 10.97,
    "lines_court_frame": {
      "baselines_x": [-11.885, 11.885], "service_x": [-6.4, 6.4],
      "sidelines_y": [-5.485, 5.485], "center_line_y": 0.0
    }
  },
  "fence": {
    "corners": [{"x_m": -8.59, "y_m": -8.46}, "...x4"],
    "extents_court_frame": {"x_near": -16.63, "x_far": 16.37, "y_left": -8.46, "y_right": 8.79}
  },
  "distances_to_fence_m": {
    "near_baseline": 4.75, "far_baseline": 4.48,
    "left_sideline": 3.28, "right_sideline": 3.31
  },
  "obstacles": [
    {"id": 1, "class": "obstacle", "center": {"x_m": 5.0, "y_m": 1.0},
     "size_m": {"w": 0.4, "h": 1.2}, "point_count": 42}
  ],
  "map_artifact": {
    "status": "saved",
    "basename": "runtime/maps/court_1781845000",
    "files": {"posegraph": "...", "data": "...", "yaml": "...", "pgm": "..."},
    "court_frame": {"center": {"x_m": 8.04, "y_m": 0.0},
                    "axis_length": {"x_m": 1.0, "y_m": 0.0},
                    "axis_width": {"x_m": 0.0, "y_m": 1.0}}
  },
  "occupancy": {"point_count": 1500},
  "surveyed_at": 0
}
```

## 7. SLAM & μοντέλο — διορθώσεις που έκαναν τον χάρτη καθαρό/πλήρη

- **Loop closure (slam_toolbox) ενεργό αλλά συντηρητικό:** search μόνο < 3 m (ποτέ δεν
  ταιριάζει τον απέναντι φράχτη), αυστηρά response thresholds, + το δίχτυ/γωνίες
  σπάνε την κατά-μήκος ασάφεια. Χωρίς αυτό, το odometry drift μεταξύ των δύο μισών
  «δίπλωνε» το δίχτυ σε διπλή γραμμή. Δουλεύει μαζί με το **return pass** (§4) που
  δίνει την επικάλυψη για να κλείσει ο βρόχος.
- **Net visual strand στο ύψος του LiDAR:** το `gpu_lidar` βλέπει **visuals** (όχι το
  collision box)· το πλέγμα κλωστών άφηνε κενό στα 0.713 m. Προστέθηκε συνεχής
  οριζόντια λωρίδα στο `model.sdf` → σταθερό net lock ανά scan.

## 7.1 SLAM map serialization για Nav2 (collection phase)

Όταν το survey ολοκληρώνεται (state `SAVING_MAP` πριν το `DONE`), ο node **σειριοποιεί
best-effort** τον slam_toolbox χάρτη ώστε να τον ξαναφορτώνει το Nav2 χωρίς νέο mapping:

- `serialize_map` → `runtime/maps/court_<ts>.posegraph` + `.data` (για slam_toolbox
  **localization mode** — ταιριάζει με το SLAM-based `navigation.launch.py`).
- `save_map` → `.pgm` + `.yaml` (occupancy grid, για επιθεώρηση / map_server+AMCL).
- Τα paths + το **court frame** (center + άξονες) γράφονται στο `court_boundary.json`
  ως `map_artifact`. Έτσι μετρήσεις (Knowledge Model) και grid μοιράζονται **ένα
  κοινό frame**.

Best-effort & **fail-safe**: αν τα services λείπουν ή λήξει το timeout, το survey
ολοκληρώνεται κανονικά με την (έγκυρη) μέτρηση — απλώς `map_artifact.status` =
`error`/`pending`. Δεν μπλοκάρει ποτέ.

Η βάση κρατά το `map_artifact` μέσα στο raw_json κάθε survey· το
`TennisRobotDB.current_survey(court_id)` το εκθέτει στη φάση συλλογής για να φορτωθεί
ο σωστός χάρτης ανά court. (Επόμενο βήμα (4): inject των CKM obstacles ως Nav2
keepout/obstacle layer.)

## 8. Panel — Sensor Views (`survey_map_v2.js`)

Αυτόνομο module που, όταν `schema === "court_knowledge_model/v2"`, ζωγραφίζει
κατευθείαν από το v2: live LiDAR points, περίγραμμα φράχτη (`fence.corners`), δίχτυ +
posts, εσωτερικές γραμμές μέσω του net frame, run-off βελάκια + πίνακας αποστάσεων,
obstacles, θέση ρομπότ, status (OK/FAILED/live-error, fail-loud). Το `app.js` κάνει
delegate στο call site· η παλιά v1 λογική μένει ως fallback.

## 9. Χάρτης αρχείων (as-built)

| Αρχείο | Ρόλος |
| --- | --- |
| `tennis_robot/court_extraction.py` | Pure μετρητική (3a–3g), offline-testable |
| `tennis_robot/court_coverage.py` | Vantage points (8 + return pass), recoverable classifier |
| `tennis_robot/court_survey_v2_node.py` | Coverage controller (FIND_NET→COVERAGE→DONE), decouple |
| `tennis_robot/court_survey_mission_node.py` | Delegation stub → v2 node |
| `config/slam_toolbox.yaml` | SLAM + conservative loop closure |
| `gazebo/models/tennis_court/model.sdf` | net strand στο ύψος LiDAR |
| `runtime/maps/court_<ts>.*` | Σειριοποιημένος SLAM χάρτης per survey (Nav2) |
| `scripts/control_panel/survey_map_v2.js` | Panel v2 renderer |
| `tests/test_court_extraction.py`, `tests/test_court_coverage.py` | Offline tests |

## 10. Επικύρωση

End-to-end στο Gazebo: Map Court → `survey OK (map complete)`, αποστάσεις φράχτη
συμμετρικές (~4.7 / 4.5 / 3.3 / 3.3 m), **0 obstacles** σε άδειο γήπεδο, δίχτυ ως μονή
γραμμή μετά το loop closure. Offline: όλα τα `test_court_*` περνούν (synthetic courts,
rotated/translated invariance, missing-side & nonstandard fail-loud, artifact filter).
