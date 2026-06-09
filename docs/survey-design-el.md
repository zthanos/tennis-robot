# Survey Design — Waypoint-Guided Perimeter Navigation

## Σκοπός

Το survey χαρτογραφεί το γήπεδο κινούμενο στην περίμετρο του. Αποτέλεσμα: αρχείο
`runtime/court_boundary.json` με διαστάσεις γηπέδου, θέση φιλέ, και ορόσημα για
την επόμενη φάση (collection).

Η ίδια λογική πρέπει να τρέχει σε Gazebo simulation και σε πραγματικό ρομπότ χωρίς
αλλαγές κώδικα. Οι διαφορές καλύπτονται αποκλειστικά από env vars και ROS topics.

---

## Navigation Pattern

Το survey ακολουθεί σταθερό γεωμετρικό pattern:

```text
[START: net standoff]
  │
  ▼ 180° στροφή
[baseline fence]
  │
  ▼ 90° αριστερά
[side fence (short side)]
  │
  ▼ 90° αριστερά
[long side traverse →→→→→→→→→→]
  │                             │
  │   (περνά φιλέ)              ▼
  │                    [far baseline fence]
  │
  ▼ 90° αριστερά
[far short side]
  │
  ▼ 90° αριστερά
[return long side ←←←←←←←←←←]
  │
  ▼
[DONE: baseline fence]
```

### Expected waypoint output (top-down, court από ψηλά)

Το παρακάτω σχήμα **δεν είναι το σχέδιο του route** — είναι το **αναμενόμενο
αποτέλεσμα** της συσσώρευσης waypoints κατά τη διάρκεια του survey. Το σύστημα
επαληθεύει ότι τα waypoints που παρήγαγαν οι sensors σχηματίζουν αυτό ακριβώς το
ορθογώνιο. Αν το σχήμα αποκλίνει, το ρομπότ έχει χάσει την πορεία του.

```text
NEAR FENCE                     NET                      FAR FENCE
(baseline)                      │                      (baseline)
    │                           │                           │
    ┼───────────────────────────────────────────────────────┼  ← TOP FENCE
    │                           │                           │
    ⑥ ←←←←←←←←←←←← return long side ←←←←←←←←←←←←←←← ⑤
    ↑  ┌────────────────────────┴───────────────────────┐   ↑
    │  │                        │                       │   │
    ②  │          ①(P)          │                       │   ⑤
    ↓  │       net standoff     │                       │   ↑
    │  │         (start)        │                       │   │
    │  └────────────────────────┬───────────────────────┘   │
    ③ →→→→→→→→→→→→→→ long side traverse →→→→→→→→→→→→→→→→ ④
    │                           │                           │
    ┼───────────────────────────────────────────────────────┼  ← BOT FENCE
    │                           │                           │
```

```text
Ακολουθία waypoints που πρέπει να προκύψουν από τους sensors:

  ①(start) ──180°──► ② ──90°L──► ③ ──90°L──► ④ ──90°L──► ⑤ ──90°L──► ⑥ ── DONE
  net standoff    near fence   bot-left     bot-right    top-right    top-left
                  (baseline)   corner       corner       corner       corner
```

Κάθε waypoint (①–⑥) καταγράφεται τη στιγμή που ο sensor trigger εκπυρσοκροτεί
(front LiDAR threshold ή net standoff distance). Η γεωμετρία των 6 σημείων
συγκρίνεται με το expected ορθογώνιο για να επαληθευτεί η σωστή πορεία.

---

## Κύρια Αρχή: Waypoint-First Navigation

Αντί για heading-lock με corrections, κάθε DRIVE leg παράγει ζωντανά waypoints από
τους sensors:

```text
κάθε control tick (DRIVE_* state):
  next_wp = WaypointGenerator.compute(pose, lidar, camera)
  cmd     = drive_to_waypoint(pose, next_wp)

corner detection:
  front_range < threshold → record corner waypoint → enter TURN_* state
```

Το ρομπότ δεν "ξέρει" πού πρέπει να πάει από μνήμη. Το υπολογίζει κάθε φορά από
αυτό που βλέπουν οι sensors.

---

## States

```text
NET_STANDOFF       Κινείται προς το φιλέ, σταματά στο standoff. Camera επιβεβαιώνει.
TURN_180           180° στροφή προς baseline.
BASELINE_APPROACH  Κινείται προς baseline fence, σταματά.
TURN_TO_SIDELINE   90° αριστερά.
DRIVE_SIDELINE     Waypoint-guided. LiDAR: left fence. Camera: court lines.
TURN_TO_LONG_SIDE  90° αριστερά.
DRIVE_LONG_SIDE    Waypoint-guided. Περνά φιλέ με 80th-pct front range.
TURN_TO_FAR_SHORT  90° αριστερά.
DRIVE_FAR_SHORT    Waypoint-guided. LiDAR: left fence. Camera: court lines.
TURN_TO_RETURN     90° αριστερά.
DRIVE_RETURN       Waypoint-guided. Περνά φιλέ με 80th-pct front range.
DONE               Γράφει court_boundary.json.
```

Κάθε TURN_* state: ο `TurnTracker` μετράει γωνία από gyro/odometry. Μόλις φτάσει
στο target → αμέσως DRIVE_*.

---

## WaypointGenerator

Εκτελείται κάθε tick σε κάθε DRIVE_* state.

```python
inputs:
  pose      = (x, y, yaw)          # από odometry/localization
  lidar     = ranges[]              # LiDAR scan
  camera    = SurveyVision          # line_detected, line_offset_m, obstacle_class

υπολογισμός:
  # 1. Προβολή LOOKAHEAD_M μπροστά
  fwd_x = x + cos(yaw) * LOOKAHEAD_M
  fwd_y = y + sin(yaw) * LOOKAHEAD_M

  # 2. Lateral correction (κάθετα στο heading)
  lateral = 0.0

  # LiDAR: side fence distance correction
  side_range = sector_median(lidar, LEFT_90°, ±25°)
  if side_range is finite:
      lateral += (side_range - TARGET_FENCE_M) * K_LIDAR

  # Camera: court line offset correction
  if camera.line_detected:
      lateral += camera.line_offset_m * K_CAMERA

  # 3. Εφαρμογή lateral (αριστερά/δεξιά ως προς heading)
  left_x = -sin(yaw)
  left_y =  cos(yaw)
  wp = (fwd_x + left_x * lateral,
        fwd_y + left_y * lateral)

output:
  next waypoint (x, y)
```

Το ρομπότ οδηγείται με `drive_to_target(pose, wp)` — υπάρχει ήδη στον κώδικα.

### Sensor roles ανά leg

| Leg             | LiDAR side         | Camera               |
| --------------- | ------------------ | -------------------- |
| DRIVE_SIDELINE  | left fence         | sideline offset      |
| DRIVE_LONG_SIDE | left fence (outer) | — (net pass-through) |
| DRIVE_FAR_SHORT | left fence         | far sideline offset  |
| DRIVE_RETURN    | left fence (outer) | — (net pass-through) |

### Lookahead

`LOOKAHEAD_M = 1.5 m` (responsive + stable). Env var: `SURVEY_WAYPOINT_LOOKAHEAD_M`.

### Target fence distance

`TARGET_FENCE_M = 0.70 m` (clearance από τον φράχτη). Env var: `SURVEY_TARGET_FENCE_M`.

---

## Corner Detection

Σε κάθε DRIVE_* tick:

```python
front_range = sector_80th_pct(lidar, 0°, ±20°)

if front_range < STOP_THRESHOLD:
    if obstacle is net (camera.obstacle_class ∈ {net, post}):
        pass  # drive through
    else:
        record_corner_waypoint(label, x, y)
        enter TURN_* state
```

- Long sides: `STOP_THRESHOLD = 2.50 m` (αργή προσέγγιση fence)
- Short sides: `STOP_THRESHOLD = 1.20 m`
- Net pass-through: 80th-pct αγνοεί sparse returns από το δίχτυ

---

## Pattern Validation

Τα corner waypoints συγκεντρώνονται ως η "υπογραφή" του route:

```text
expected pattern (σε σειρά):
  near_net_standoff
  near_baseline_fence_standoff
  left_side_fence_corner
  far_baseline_fence_corner
  right_side_fence_corner
  return_baseline_fence_corner
```

Validation ελέγχει:

1. **Σειρά labels**: εμφανίστηκαν με τη σωστή σειρά;
2. **Γεωμετρία**: οι αποστάσεις μεταξύ corners συμφωνούν με expected court dims;
3. **Γωνίες**: οι συνεχόμενοι τομείς έχουν ≈ 90° μεταξύ τους;

Αν η γεωμετρία αποκλίνει → `survey_pattern.valid = false` στο telemetry.

### Geometric checks

```text
near_baseline → left_side_fence    ≈ court_width / 2 + alley
left_side → far_baseline            ≈ court_length
far_baseline → right_side           ≈ court_width / 2 + alley
right_side → return_baseline        ≈ court_length
```

Tolerance: ±3 m (αρκετό για ανομοιομορφίες τοποθέτησης).

---

## Sensors

### LiDAR

- Κύρια χρήση: μέτρηση απόστασης από fence για lateral correction
- Corner detection: front range trigger
- Net pass-through: 80th-pct sector για sparse obstacles
- Obstacle detection: front-left / front-right sectors

### Camera (SurveyVision)

- `line_detected`, `line_offset_m`, `line_heading_error_rad`: lateral correction
- `obstacle_class`: net confirmation (pass-through vs stop)

Η κάμερα δεν είναι απαραίτητη για την ολοκλήρωση του survey — το LiDAR αρκεί για
navigation. Η κάμερα βελτιώνει την ακρίβεια στις straight legs και επιτρέπει net
pass-through.

---

## Αποτέλεσμα: court_boundary.json

```json
{
  "status": "SUCCESS",
  "survey_complete": true,
  "survey_type": "full_perimeter",
  "surveyed_at": 1780450000.0,
  "elapsed_s": 142.3,
  "is_doubles": true,
  "boundary_distances": {
    "near_baseline_to_fence_m": 1.85,
    "far_baseline_to_fence_m": 1.82,
    "left_sideline_to_fence_m": 1.41,
    "right_sideline_to_fence_m": 1.39
  },
  "navigation_points": [
    {"label": "near_net_standoff",          "x_m": 1.6,   "y_m": 0.0},
    {"label": "near_baseline_fence_standoff","x_m": -10.1, "y_m": 0.0},
    {"label": "left_side_fence_corner",      "x_m": -10.1, "y_m": -6.4},
    {"label": "far_baseline_fence_corner",   "x_m": 10.1,  "y_m": -6.4},
    {"label": "right_side_fence_corner",     "x_m": 10.1,  "y_m":  6.4},
    {"label": "return_baseline_fence_corner","x_m": -10.1, "y_m":  6.4}
  ],
  "navigation_pattern": {
    "complete": true,
    "valid": true,
    "matched": ["near_net_standoff", "near_baseline_fence_standoff", "..."]
  }
}
```

---

## Sim / Real World Compat

Το survey δεν έχει κώδικα που να ξέρει αν τρέχει σε Gazebo ή σε real robot.
Διαφορές καλύπτονται από:

| Παράμετρος                    | Gazebo default          | Real robot               |
| ----------------------------- | ----------------------- | ------------------------ |
| `ROS2_SURVEY_DRIVE_SPEED_M_S` | 0.60                    | χαμηλότερο               |
| `SURVEY_TARGET_FENCE_M`       | 0.70                    | ρυθμίζεται               |
| `SURVEY_WAYPOINT_LOOKAHEAD_M` | 1.50                    | ρυθμίζεται               |
| LiDAR topic                   | `/scan` (Gazebo plugin) | `/scan` (RPLidar)        |
| Camera topic                  | `/camera/image_raw`     | `/oak/rgb/image_raw`     |
| Odometry                      | `/odom` (Gazebo)        | `/odom` (wheel encoders) |

Κανένας hardcoded τιμή μέσα στον survey κώδικα — όλα μέσω `LidarSurveyConfig.from_env()`.

---

## Failure Handling

Κάθε state έχει timeout. Αν λήξει → `_finalize_full_survey(failure_reason)`.

Failure καταγράφεται στο `court_boundary.json`:

```json
{
  "status": "PARTIAL",
  "failure_reason": "long_side_drive_timeout",
  "survey_complete": false
}
```

Το ρομπότ σταματά και επιστρέφει `DONE` state — δεν κολλάει.

---

## Τι ΔΕΝ κάνει το survey

- Δεν χαρτογραφεί εσωτερικές γραμμές (service lines, center line)
- Δεν εντοπίζει μπαλάκια
- Δεν κάνει point cloud accumulation / SLAM
- Δεν έχει fallback / legacy mode — αυτή η υλοποίηση είναι η μοναδική
