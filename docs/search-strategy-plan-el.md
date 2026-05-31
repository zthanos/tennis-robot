# Tennis Robot Search Strategy Plan — V2 Coarse-to-Fine

Αυτό το έγγραφο ορίζει τη στρατηγική αναζήτησης V2 για το tennis robot.
Αντικαθιστά πλήρως την V1 (Boundary-First + Lane Sweep).
Συμπληρώνει το validation plan και το collection state machine plan.

## Objective

Να σχεδιαστεί και να επικυρωθεί ένας αλγόριθμος αναζήτησης που:

- χτίζει γρήγορα global understanding του court πριν ξεκινήσει collection,
- κατευθύνεται πρώτα στις περιοχές με τη μεγαλύτερη πυκνότητα μπαλών,
- εντοπίζει μπάλες με την OAK-D,
- αποφεύγει ανθρώπους και εμπόδια με το LiDAR,
- συνεχίζει την αναζήτηση μετά από κάθε συλλογή.

## 1. Αρχιτεκτονική: Coarse-to-Fine

Αντί για πλήρες lawnmower sweep, το robot λειτουργεί σε δύο επίπεδα:

```text
Coarse Layer (Survey)
  ↓
Κάθε viewpoint: 360° camera sweep → zone heatmap

Fine Layer (Prioritized Local Scan)
  ↓
Πήγαινε πρώτα στη zone με περισσότερες εκτιμώμενες μπάλες
  ↓
Τοπικό mini-sweep + collection
```

### Sensor Responsibilities

| Sensor | Role | Not responsible for |
| --- | --- | --- |
| LiDAR | obstacle detection, human avoidance, safe path validation | ball recognition |
| OAK-D | ball detection, zone heatmap accumulation, target localization | primary obstacle map |
| Front collection sensors | collection confirmation | long-range search |

## 2. Search State Machine

```text
SURVEY_VIEWPOINT
  ↓  (όλα τα viewpoints ολοκληρώθηκαν)
TRANSIT_TO_ZONE
  ↓  (φτάσαμε στο κέντρο της zone)
LOCAL_SCAN
  ↓  (μπάλα εντοπίστηκε)
BALL_DETECTED
  ↓  (target_hold_s ολοκληρώθηκε)
  → resume LOCAL_SCAN ή SURVEY_VIEWPOINT
  ↓  (όλες οι lanes της zone τελείωσαν)
  → TRANSIT_TO_ZONE (επόμενη zone) ή COMPLETE
```

Το `BALL_DETECTED` μπορεί να ενεργοποιηθεί από οποιοδήποτε state.
Το resume γυρίζει πάντα στο state που ήταν πριν το interrupt.

## 3. Phase 1 — Survey Viewpoints

### 3.1 Στόχος

Να χτιστεί Zone Heatmap πριν ξεκινήσει η collection.

### 3.2 Τι κάνει το robot

Το robot σταματά σε N strategic viewpoints (default: `zone_cols` viewpoints κατά μήκος του x-άξονα).

Σε κάθε viewpoint περιστρέφεται αργά για `survey_viewpoint_dwell_s` seconds. Κάθε ball observation με γνωστές world coordinates χρεώνεται στην αντίστοιχη zone:

```text
zone.estimated_count += max(0.1, observation.confidence)
```

### 3.3 Zone Heatmap

Παράδειγμα μετά το survey:

```text
Zone A → 0.0   Zone C → 6.2   Zone E → 0.0
Zone B → 0.8   Zone D → 3.1   Zone F → 0.0
```

### 3.4 DoD

1. Το robot επισκέπτεται όλα τα survey viewpoints.
2. Κάθε ball observation με world coordinates ενημερώνει το σωστό zone.
3. Το zone heatmap εκτίθεται στο telemetry/snapshot.
4. Το robot μπορεί να διακόψει το survey αν εντοπίσει μπάλα κοντά.

## 4. Phase 2 — Zone Prioritization

### 4.1 Στόχος

Να επιλεχθεί η επόμενη zone για collection με βάση density και proximity.

### 4.2 Scoring Formula

```text
score(zone) = estimated_count / (1 + distance_to_zone / zone_proximity_weight)
            + 0.1 αν η zone δεν έχει επισκεφτεί ποτέ
```

Επιλέγεται η zone με το υψηλότερο score.

### 4.3 Σειρά επίσκεψης

Παράδειγμα με heatmap:

```text
Zone C → 6.2 (score: 1ο)
Zone D → 3.1 (score: 2ο)
Zone B → 0.8 (score: 3ο)
Zone A → 0.0 (αποφεύγεται αν δεν υπάρχουν άλλες)
```

### 4.4 DoD

1. Το robot επιλέγει zone με βάση τον παραπάνω τύπο.
2. Οι unvisited zones προτιμώνται έναντι ήδη επισκεφθέντων.
3. Zones με `estimated_count == 0` επισκέπτονται μόνο αν δεν υπάρχουν άλλες επιλογές.

## 5. Phase 3 — Transit to Zone

### 5.1 Στόχος

Να φτάσει το robot στο κέντρο της επιλεγμένης zone.

### 5.2 Behavior

- Χρησιμοποιεί `_goto_command` (heading gain + proportional speed).
- Σταματά αν το LiDAR ανιχνεύσει εμπόδιο μπροστά.
- Μεταβαίνει σε LOCAL_SCAN μόλις φτάσει εντός `waypoint_tolerance_m`.

### 5.3 DoD

1. Το robot φτάνει στο zone center χωρίς να αγνοεί εμπόδια.
2. Αν το path είναι blocked, σταματά και αναφέρει `path_status = "blocked"`.

## 6. Phase 4 — Local Scan (Boustrophedon εντός Zone)

### 6.1 Στόχος

Να σαρωθεί συστηματικά η επιλεγμένη zone για ball detection.

### 6.2 Search Pattern

Mini boustrophedon εντός των ορίων της zone:

```text
zone.min_x ─────────────────→ zone.max_x
zone.min_x ←─────────────────  zone.max_x
zone.min_x ─────────────────→ zone.max_x
```

Πλάτος λωρίδας: `lane_width_m` (default: 1.5m).

### 6.3 Interrupt

Αν εντοπιστεί μπάλα κατά τη διάρκεια LOCAL_SCAN:

```text
→ BALL_DETECTED (hold target_hold_s)
→ resume LOCAL_SCAN από το ίδιο σημείο
```

### 6.4 Ολοκλήρωση zone

Όταν τελειώσουν όλες οι lanes:

```text
zone.visit_count += 1
→ επιλογή επόμενης zone ή COMPLETE
```

### 6.5 DoD

1. Το robot καλύπτει ολόκληρη τη zone με overlapping lanes.
2. Γνωρίζει σε ποια lane βρισκόταν μετά από interrupt.
3. Το `zone.visit_count` αυξάνεται μόλις τελειώσει η zone.
4. Το coverage metric αντικατοπτρίζει zones_visited / total_zones.

## 7. Phase 5 — Ball Detected (Interrupt)

### 7.1 Στόχος

Να ειδοποιηθεί το collection layer ότι βρέθηκε στόχος.

### 7.2 Behavior

```text
Ball Confidence >= threshold
AND distance <= max_interrupt_distance_m
  ↓
BALL_DETECTED state (robot σταματά)
  ↓
Hold για target_hold_s
  ↓
Resume search state
  ↓
Cooldown: detected_target_cooldown_s (αποτρέπει re-detection ίδιου στόχου)
```

### 7.3 Resume Marker

Αποθηκεύεται πριν την interrupt:

```text
"survey_viewpoint:2"        ← αν ήταν σε survey
"transit_to_zone:C"         ← αν ήταν σε transit
"local_scan:C:4"            ← zone + waypoint index
```

### 7.4 DoD

1. Το robot σταματά στο BALL_DETECTED.
2. Γυρίζει στο σωστό state μετά από target_hold_s.
3. Δεν re-detects τον ίδιο στόχο κατά τη διάρκεια του cooldown.
4. Αποθηκεύει world coordinates στο zone heatmap.

## 8. Phase 6 — Safety and Obstacle Handling

Ίδιες αρχές με V1:

| Event | Action |
| --- | --- |
| Human detected (LiDAR) | STOP → WAIT → REPLAN |
| Obstacle εμπρός | Σταμάτα, `path_status = "blocked"` |
| Path not available | Προχώρα στον επόμενο waypoint |

### 8.1 DoD

1. Το robot σταματά όταν ανιχνεύεται άνθρωπος.
2. Δεν σπρώχνει εμπόδια.
3. Μπορεί να συνεχίσει μετά την απομάκρυνση εμποδίου.

## 9. Coverage Metric

```text
coverage_pct = (survey_fraction × 0.25 + zone_fraction × 0.75) × 100
```

Όπου `survey_fraction = viewpoints_done / total_viewpoints` (0.0 → 1.0) και `zone_fraction = zones_visited / total_zones` (0.0 → 1.0).

Παράδειγμα:

- Μετά το survey: 25%
- Μετά 2/6 zones: 25% + 75% × (2/6) = 50%
- Μετά όλες τις zones: 100%

## 10. Zone Grid

Το court half χωρίζεται σε `zone_cols × zone_rows` grid.

Default (3 × 2 = 6 zones):

```text
┌──────┬──────┬──────┐
│  A   │  C   │  E   │  ← top row (y > 0)
├──────┼──────┼──────┤
│  B   │  D   │  F   │  ← bottom row (y < 0)
└──────┴──────┴──────┘
baseline      net
```

Zone IDs: `chr(ord("A") + col * rows + row)`

## 11. Telemetry Fields

```text
search_state         SURVEY_VIEWPOINT | TRANSIT_TO_ZONE | LOCAL_SCAN | BALL_DETECTED | COMPLETE
phase                ίδιο με search_state
zone_id              τρέχουσα zone
coverage_pct         0.0 → 100.0
waypoint_index       τρέχον waypoint εντός local scan ή survey index
waypoint_count       συνολικά waypoints της τρέχουσας φάσης
target_id            ID του στόχου (αν υπάρχει)
target_status        none | detected | queued
path_status          clear | blocked | waiting | pending_validation
resume_marker        survey_viewpoint:N | transit_to_zone:Z | local_scan:Z:N
target_cooldown_s    remaining cooldown μετά από detect
zone_heatmap[]       zone_id, estimated_count, visit_count
```

## 12. Configuration (Environment Variables)

| Variable | Default | Description |
| --- | --- | --- |
| `SEARCH_SIDE` | `left` | Ποιο μισό του court |
| `SEARCH_ZONE_COLS` | `3` | Στήλες grid |
| `SEARCH_ZONE_ROWS` | `2` | Γραμμές grid |
| `SEARCH_SURVEY_VIEWPOINT_DWELL_S` | `6.0` | Χρόνος αναμονής σε κάθε viewpoint |
| `SEARCH_SURVEY_ROTATE_SPEED_RAD_S` | `0.45` | Ταχύτητα περιστροφής κατά το survey |
| `SEARCH_LANE_WIDTH_M` | `1.5` | Πλάτος local scan lane |
| `SEARCH_ZONE_PROXIMITY_WEIGHT` | `3.0` | Σταθερά proximity στο zone scoring |
| `SEARCH_DETECTION_CONFIDENCE_THRESHOLD` | `0.03` | Minimum confidence για interrupt |
| `SEARCH_DRIVE_SPEED_M_S` | `0.24` | Ταχύτητα κίνησης |
| `SEARCH_TARGET_HOLD_S` | `1.25` | Χρόνος αναμονής σε BALL_DETECTED |
| `SEARCH_DETECTED_TARGET_COOLDOWN_S` | `8.0` | Cooldown μετά από detection |

## 13. Comparison V1 vs V2

| | V1 | V2 |
| --- | --- | --- |
| Αρχική φάση | Boundary perimeter | Survey viewpoints |
| Routing | Fixed sequence | Zone heatmap priority |
| Coverage pattern | Full lawnmower | Per-zone mini-sweep |
| Global awareness | Καμία | Zone heatmap πριν collection |
| Ball cluster handling | Βρίσκει τυχαία | Πάει πρώτα στη high-density zone |
| States | BOUNDARY_FIRST, LANE_SWEEP | SURVEY_VIEWPOINT, TRANSIT_TO_ZONE, LOCAL_SCAN |

## 14. First Webots Implementation Target

1. Survey pass σε 3 viewpoints του half-court.
2. Zone heatmap με 6 zones (3×2).
3. Zone selection με density + proximity scoring.
4. Local boustrophedon sweep εντός κάθε zone.
5. Ball interrupt + resume από σωστό marker.
6. Telemetry για όλα τα fields της ενότητας 11.
7. Smoke test χωρίς Webots (survey → interrupt → resume → heatmap → coverage).
8. Dashboard fields συμβατά με `docs/mission-dashboard-plan-el.md`.
9. Collection handoff συμβατό με `docs/collection-state-machine-plan-el.md`.
