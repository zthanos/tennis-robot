# Court Survey v2 — LiDAR Occupancy → Court Knowledge Model

> Καθαρή επανασχεδίαση του survey γύρω από τον **πραγματικό σκοπό**: μέτρηση, όχι
> τέλεια περιφορά. Αντικαθιστά το dead-reckoning perimeter FSM
> (`court_survey_mission_node.py`) που αποδείχθηκε εύθραυστο.

## 1. Σκοπός & αρχές

**Σκοπός:** Στο τέλος της χαρτογράφησης το robot πρέπει να γνωρίζει:

1. Πού είναι οι **φράχτες** (εξωτερικό όριο).
2. Πόσο **απέχουν οι φράχτες από τις γραμμές** του γηπέδου (run-off ανά πλευρά).
3. Πού είναι το **δίχτυ** και οι **δύο στύλοι** του.
4. Κάθε **άλλο εμπόδιο εντός των φραχτών** (πάγκοι, στύλοι φωτισμού, …) — θέση + μέγεθος.

**Αρχές σχεδίασης (μη διαπραγματεύσιμες):**

- **LiDAR-first.** Ό,τι είναι κάθετο (φράχτες, δίχτυ, στύλοι, εμπόδια) μετριέται από το
  360° LiDAR. Η κάμερα ΔΕΝ χρησιμοποιείται στη Φάση 1.
- **Μέτρηση από τον χάρτη, όχι από στιγμιαία pose.** Οι αποστάσεις είναι διαφορές
  παγκόσμιων θέσεων στον συσσωρευμένο occupancy χάρτη.
- **Standard διαστάσεις για τις γραμμές.** Οι γραμμές του γηπέδου (αόρατες στο LiDAR)
  προκύπτουν από το δίχτυ ως άγκυρα + κανονιστικές διαστάσεις ITF. Καμία κάμερα.
- **Ίδιος κώδικας σε Gazebo & πραγματικό ρομπότ.** Διαφορές μόνο μέσω env vars/topics.
- **ΚΑΘΑΡΗ ΥΛΟΠΟΙΗΣΗ — ΧΩΡΙΣ FALLBACKS.** Καμία σιωπηλή εκτίμηση/default. Αν ένα βήμα
  δεν έχει αρκετά δεδομένα ή αποτυγχάνει έλεγχος, το survey **αποτυγχάνει ρητά**
  (fail-loud) με σαφή αιτία και ΔΕΝ γράφει επινοημένο boundary. Η μόνη «σταθερά» που
  επιτρέπεται είναι οι **κανονιστικές διαστάσεις γηπέδου** — συνειδητή σχεδιαστική
  επιλογή, όχι fallback.

## 2. Κανονιστικές διαστάσεις (ITF) — η μόνη επιτρεπτή σταθερά

| Στοιχείο | Τιμή |
| --- | --- |
| Μήκος γηπέδου (baseline→baseline) | 23.77 m |
| Πλάτος doubles | 10.97 m |
| Πλάτος singles | 8.23 m |
| Service line από δίχτυ | 6.40 m |
| Doubles alley | 1.37 m |
| Net post span (doubles) | ~11.3 m (στύλοι ±5.65 m) |

Όλα tunable μέσω env (`COURT_SURVEY_COURT_LENGTH_M` κ.λπ.) αλλά με κανονιστικά defaults.

## 3. Αρχιτεκτονική / data flow

```
/scan (360° LiDAR) ─┐
                    ├─► [1] Occupancy accumulation (map frame, voxel grid)
TF map→<scan frame>─┘            │  (υπάρχει ήδη: court_survey_live.json)
                                ▼
                       [2] Coverage controller
                       (κίνηση μέχρι πλήρη παρατήρηση των 4 πλευρών)
                                │
                                ▼
                       [3] Extraction (pure functions, testable offline)
                         3a. Net + posts        → court frame (origin, axes)
                         3b. Fence rectangle    → 4 πλευρές, γωνίες
                         3c. Court lines        → net-anchor + standard dims
                         3d. Obstacles          → clustering εντός φραχτών
                         3e. Distances          → φράχτης − γραμμή
                         3f. Singles/doubles    → από post span
                         3g. Consistency checks → fail-loud
                                │
                                ▼
                       [4] court_boundary.json (Court Knowledge Model)
```

Τα βήματα **[3a–3g] είναι καθαρές συναρτήσεις** πάνω σε λίστα σημείων → unit-testable
χωρίς ROS/Gazebo (deterministic, με synthetic point clouds).

## 4. Coverage controller (κίνηση = μέσο, όχι σκοπός)

Στόχος: το 360° LiDAR (εμβέλεια 12 m) να δει **και τις 4 πλευρές του φράχτη** + το
εσωτερικό, χωρίς occlusions.

- Το δίχτυ εντοπίζεται νωρίς (όπως σήμερα, `locked_net`) → ορίζει το court frame.
- Οδήγηση σε **λίγα vantage points εκφρασμένα στο court frame** (π.χ. κέντρο κάθε
  μισού, ώστε η μακρινή baseline να μπει εντός 12 m). Όχι cardinal/dead-reckoning
  περίμετρος.
- **Coverage completeness check** = το ίδιο το 3g: συνεχίζει η κίνηση μέχρι κάθε πλευρά
  φράχτη να έχει ≥ N σημεία. Όταν ολοκληρωθεί → extraction. 
- **Fail-loud:** αν μετά από max vantage points/χρόνο μια πλευρά παραμένει
  ακάλυπτη (μόνιμο occlusion) → `coverage_incomplete: side=<X>` και ΔΕΝ εκδίδεται
  boundary.

> Σημείωση: η κίνηση δεν χρειάζεται ακρίβεια — μόνο να φέρει τις πλευρές εντός
> εμβέλειας. Η μέτρηση γίνεται από τον χάρτη, όχι από τη διαδρομή.

## 5. Extraction — αλγόριθμοι

### 3a. Δίχτυ + στύλοι → court frame
- Από το `locked_net` (LiDAR): net center `C` (map frame) και κατεύθυνση προσέγγισης
  robot→net = **άξονας μήκους** `û`. Άξονας πλάτους `v̂ = perp(û)`.
- **Στύλοι:** σημεία κοντά στη γραμμή του δικτύου (|προβολή σε û| < ε), τα **δύο
  ακραία** κατά μήκος `v̂` → θέσεις στύλων → **span** (doubles/singles).
- Court frame: origin = `C`, άξονες `(û, v̂)`. Όλα τα επόμενα σε court-frame `(x', y')`.

### 3b. Fence rectangle
- Μετασχηματισμός όλων των map_points σε court frame.
- Προβολή σε `x'` → histogram → οι **δύο ακραίες πυκνές κορυφές** = near/far baseline
  φράχτες (`x' = x_near`, `x_far`). Ομοίως `y'` → side φράχτες (`y_left`, `y_right`).
- Κάθε πλευρά απαιτεί ≥ N σημεία και γραμμικότητα (RANSAC line, residual < τ) →
  αλλιώς fail-loud (3g).
- Αποτέλεσμα: ορθογώνιο φράχτη (4 γραμμές/γωνίες) στο court frame.

### 3c. Court lines (net-anchor, standard — χωρίς κάμερα)
Στο court frame (δίχτυ στο x'=0):
- Baselines: `x' = ±11.885`
- Service lines: `x' = ±6.40`, center line `y'=0`
- Singles sidelines: `y' = ±4.115`, doubles sidelines: `y' = ±5.485`
(όλα από τις σταθερές §2, ανάλογα singles/doubles από 3f).

### 3d. Obstacles εντός φραχτών
- Υποψήφια = σημεία **εντός** του fence rectangle, με margin, που **δεν** ανήκουν στη
  γραμμή δικτύου/στύλων ούτε στις πλευρές φράχτη.
- **Clustering** (grid/DBSCAN). Κάθε cluster → bounding box (θέση + μέγεθος).
- Κατηγοριοποίηση κατά μέγεθος: `obstacle` (≥ threshold, π.χ. στύλος/πάγκος) vs
  πιθανή `ball` (μικρό) — αλλά αναφέρονται όλα· η collection αποφασίζει.

### 3e. Distances (run-off)
Διαφορές θέσεων στον χάρτη:
- `near_baseline_to_fence = |x_near| − 11.885`
- `far_baseline_to_fence  = x_far − 11.885`
- `left_sideline_to_fence = |y_left| − sideline_y'`
- `right_sideline_to_fence = y_right − sideline_y'`

### 3f. Singles/doubles
- Από **post span**: ~11.3 m → doubles· στενότερο → singles (με tolerance).
- Διασταύρωση με το πλάτος φράχτη (πρέπει να χωράει το αντίστοιχο πλάτος + run-off).

### 3g. Consistency checks (fail-loud — η «καρδιά» του no-fallbacks)
Το survey ΕΠΙΤΥΓΧΑΝΕΙ μόνο αν **όλα** ισχύουν· αλλιώς γράφει `status:"FAILED"` με
ρητή αιτία και ΟΧΙ boundary:

| Check | Fail reason |
| --- | --- |
| Δίχτυ με αρκετά σημεία/confidence | `net_not_observed` |
| 4 πλευρές φράχτη με ≥N σημεία & γραμμικότητα | `fence_side_missing:<X>` |
| Run-off distances ≥ 0 & εντός [0, max] | `nonstandard_or_bad_fit` |
| Post span ↔ doubles ή singles εντός tol | `ambiguous_court_width` |
| Συμμετρία/ορθογωνιότητα φράχτη εντός tol | `fence_not_rectangular` |
| Coverage όλων των πλευρών | `coverage_incomplete:<X>` |

## 6. Σχήμα εξόδου — `court_boundary.json` (Court Knowledge Model)

```json
{
  "schema": "court_knowledge_model/v2",
  "status": "OK | FAILED",
  "failure_reason": null,
  "frame": "map",
  "confidence": 0.0,
  "net": {
    "center":  {"x_m": 0.0, "y_m": 0.0},
    "axis_length":  {"x_m": 1.0, "y_m": 0.0},
    "axis_width":   {"x_m": 0.0, "y_m": 1.0},
    "posts": [{"x_m": 0.0, "y_m": 5.65}, {"x_m": 0.0, "y_m": -5.65}],
    "span_m": 11.3
  },
  "court": {
    "is_doubles": true,
    "length_m": 23.77,
    "width_m": 10.97,
    "lines": {
      "baselines_x":  [-11.885, 11.885],
      "service_x":    [-6.40, 6.40],
      "sidelines_y":  [-5.485, 5.485],
      "center_line_y": 0.0
    }
  },
  "fence": {
    "corners": [ {"x_m": 0,"y_m": 0}, "...x4" ],
    "extents_court_frame": {"x_near": -15.0, "x_far": 15.0, "y_left": -8.0, "y_right": 8.0}
  },
  "distances_to_fence_m": {
    "near_baseline": 3.1, "far_baseline": 3.1,
    "left_sideline": 2.5, "right_sideline": 2.5
  },
  "obstacles": [
    {"id": 1, "class": "obstacle", "center": {"x_m": 0,"y_m": 0},
     "size_m": {"w": 0.4, "h": 1.2}, "point_count": 42}
  ],
  "occupancy": {"point_count": 5800, "voxel_m": 0.10},
  "surveyed_at": 0
}
```

## 7. Τι αντικαθιστά / τι κρατάμε

- **Κρατάμε:** το live occupancy accumulation (`_accumulate_and_write_live`,
  `court_survey_live.json`), το `locked_net`, το panel/Sensor Views (ήδη κλειδωμένα).
- **Αντικαθιστούμε:** ολόκληρο το dead-reckoning FSM (states
  `follow_*`, `cross_net`, `second_half_*`, `_snap_cardinal_yaw`, locked-heading legs).
- **Νέο:** module `court_extraction.py` (καθαρές συναρτήσεις 3a–3g) +
  λεπτό coverage controller στο mission node.

## 8. Πλάνο υλοποίησης (βήματα με verification)

1. **`court_extraction.py`** — pure functions (net/posts, fence fit, lines, obstacles,
   distances, doubles, checks). **Unit tests με synthetic point clouds** (standard
   court, με offset/rotation, με obstacle, με missing side → fail). _Χωρίς ROS._
2. **Coverage controller** — vantage points στο court frame + completeness gate +
   fail-loud occlusion. Αντικαθιστά τα follow/cross/second-half states.
3. **Wiring** — το mission node καλεί extraction στο τέλος, γράφει το v2 schema.
4. **Panel** — ενημέρωση Sensor Views να δείχνει: fence rectangle (μετρημένο),
   net+posts, lines, obstacles, distances — από το νέο schema.
5. **End-to-end στο sim** — Map Court → πλήρες Court Knowledge Model, με τα
   consistency checks να περνούν· verify distances/doubles.

Κάθε βήμα: deterministic verification πριν προχωρήσουμε (όπως κάναμε με το live map).
