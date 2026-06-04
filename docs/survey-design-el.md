# Survey Design Document

## Purpose

Το `map_court` survey πρέπει να μετατραπεί από απλό line-following behavior σε
δομημένο pipeline αναγνώρισης γηπέδου.

Ο στόχος δεν είναι μόνο να δει το robot μία λευκή γραμμή. Ο στόχος είναι να
κατασκευάσει ένα αξιόπιστο court model με:

- φιλέ και στήλες φιλέ,
- εξωτερικές γραμμές,
- εσωτερικές γραμμές,
- συντεταγμένες γραμμών,
- φράχτες,
- σταθερά εμπόδια,
- προσωρινά εμπόδια,
- ελεύθερους διαδρόμους κίνησης,
- confidence και failure evidence για κάθε βήμα.

Η νέα σχεδίαση πρέπει να αντικαταστήσει τη σημερινή λογική που ακολουθεί την
πιο δυνατή/καθαρή λευκή γραμμή της κάμερας. Αυτή η παλιά λογική αποτυγχάνει
όταν το robot πέφτει πάνω σε service lines ή σε T/intersections, επειδή δεν
γνωρίζει ποια γραμμή βλέπει.

## Core Principle

Το survey πρέπει πρώτα να βρει σταθερό reference frame.

Το φιλέ είναι το πρωτεύον anchor:

```text
baseline -> service line -> net -> service line -> baseline
```

Μόλις το φιλέ εντοπιστεί, κάθε γραμμή μπορεί να ταξινομηθεί με βάση τη θέση της
σε σχέση με το φιλέ, τη γωνία της, την απόστασή της, και το γνωστό tennis court
geometry.

Χωρίς net anchor, η κάμερα βλέπει απλώς λευκές γραμμές. Με net anchor, το robot
μπορεί να καταλάβει αν η γραμμή είναι outer sideline, baseline, service line,
center service line, ή άγνωστη εσωτερική γραμμή.

## Survey Pipeline

Το survey πρέπει να χωριστεί σε βήματα. Κάθε βήμα είναι checkpoint και δεν
επιτρέπεται να προχωράει στο επόμενο χωρίς επιτυχημένο αποτέλεσμα και επαρκές
confidence.

```text
LOCATE_NET
  -> CONFIRM_NET_VISUAL
  -> BUILD_COURT_REFERENCE_FRAME
  -> DETECT_AND_LABEL_LINES
  -> MOVE_TO_EXTERNAL_LINE
  -> FOLLOW_EXTERNAL_LINE_AND_MAP_OBSTACLES
  -> CLOSE_PERIMETER_LOOP
  -> RETURN_TO_INITIAL_POSITION
  -> SURVEY_COMPLETE
```

Αν οποιοδήποτε βήμα αποτύχει, το robot σταματάει, γράφει structured failure
reason, και δεν συνεχίζει στο επόμενο βήμα.

## Step 1 - Locate Net

### Objective

Να εντοπιστούν οι στήλες στήριξης του φιλέ με LiDAR και να παραχθεί υποψήφια
`net_line`.

### Sensor Priority

- Primary: LiDAR.
- Secondary: odometry/pose.
- Camera is not required yet.

### Expected Evidence

Το robot ψάχνει LiDAR clusters που μοιάζουν με λεπτούς σταθερούς στύλους.

Ένα πιθανό ζευγάρι net posts πρέπει να έχει:

- δύο λεπτά object clusters,
- απόσταση συμβατή με πλάτος φιλέ/court,
- σταθερότητα σε παραπάνω από ένα scan,
- λογική συμμετρία ως προς την πιθανή net line,
- ελεύθερο χώρο ανάμεσα ή γύρω από τους στύλους.

### Success Output

```json
{
  "step": "locate_net",
  "status": "complete",
  "confidence": 0.82,
  "outputs": {
    "net_post_candidates": [
      {"x_m": 0.1, "y_m": -5.45, "confidence": 0.84},
      {"x_m": 0.0, "y_m": 5.48, "confidence": 0.81}
    ],
    "estimated_net_line": {
      "p1": [0.1, -5.45],
      "p2": [0.0, 5.48]
    }
  }
}
```

### Failure Conditions

- `NET_POSTS_NOT_FOUND`
- `NET_POST_PAIR_GEOMETRY_INVALID`
- `NET_POST_CONFIDENCE_TOO_LOW`
- `LIDAR_SCAN_UNAVAILABLE`

## Step 2 - Confirm Net Visual

### Objective

Να κινηθεί το robot σε καλύτερη θέση παρατήρησης και να επιβεβαιώσει οπτικά ότι
ανάμεσα στους υποψήφιους στύλους υπάρχει φιλέ.

### Sensor Priority

- Primary: camera / OAK-D RGB.
- Secondary: depth.
- Supporting: LiDAR post candidates.

### Behavior

Το robot κινείται προς ασφαλές observation pose κοντά στην υποψήφια `net_line`,
κρατώντας safe standoff.

Η camera/depth επιβεβαιώνει:

- οριζόντια/κατακόρυφη net-like δομή,
- οπτική συνέχεια ανάμεσα στους δύο posts,
- βάθος συμβατό με LiDAR estimate,
- όχι απλό fence ή bench misclassification.

### Success Output

```json
{
  "step": "confirm_net_visual",
  "status": "complete",
  "confidence": 0.78,
  "outputs": {
    "net_confirmed": true,
    "net_line": {
      "p1": [0.1, -5.45],
      "p2": [0.0, 5.48]
    },
    "visual_features": {
      "net_pattern_confidence": 0.72,
      "depth_consistency": 0.85
    }
  }
}
```

### Failure Conditions

- `NET_VISUAL_CONFIRMATION_FAILED`
- `NET_DEPTH_INCONSISTENT`
- `NET_OBSERVATION_POSE_BLOCKED`
- `CAMERA_FRAME_UNAVAILABLE`

## Step 3 - Build Court Reference Frame

### Objective

Να δημιουργηθεί court coordinate frame με βάση το confirmed net.

### Outputs

Το court reference frame πρέπει να περιλαμβάνει:

- `net_line`,
- `net_center`,
- `court_long_axis`,
- `court_width_axis`,
- robot side relative to net,
- expected line positions,
- expected outer boundary rectangle hypotheses.

### Known Tennis Geometry

Οι αποστάσεις πρέπει να χρησιμοποιούνται ως priors, όχι ως τυφλές απόλυτες
αλήθειες:

- service lines: περίπου 6.40 m από το φιλέ,
- baselines: περίπου 11.885 m από το φιλέ,
- doubles sidelines: περίπου 5.485 m από τον κεντρικό άξονα,
- singles sidelines: περίπου 4.115 m από τον κεντρικό άξονα.

### Failure Conditions

- `COURT_FRAME_CONFIDENCE_TOO_LOW`
- `NET_LINE_GEOMETRY_INVALID`
- `ROBOT_POSE_UNAVAILABLE`

## Step 4 - Detect And Label Lines

### Objective

Κάθε detected visual line πρέπει να ταξινομείται. Δεν αρκεί το
`line_detected=true`.

### Required Labels

```text
net
outer_baseline_a
outer_baseline_b
outer_sideline_left
outer_sideline_right
service_line_a
service_line_b
center_service_line
inner_unknown
unknown
```

### Classification Inputs

- camera line detection,
- line angle in image,
- projected world/court coordinates,
- distance from net,
- parallel/perpendicular relation to net,
- robot pose,
- previous line identity,
- known court geometry priors,
- confidence history.

### Runtime Rule

For perimeter traversal, only these labels may be used as follow targets:

```text
outer_baseline_a
outer_baseline_b
outer_sideline_left
outer_sideline_right
```

Service lines and center service line must be ignored for perimeter following.

### Failure Conditions

- `LINE_LABEL_AMBIGUOUS`
- `EXTERNAL_LINE_NOT_FOUND`
- `LINE_PROJECTION_FAILED`
- `COURT_MODEL_INCONSISTENT`

## Step 5 - Move To External Line

### Objective

Να κινηθεί το robot προς την κοντινότερη ασφαλή εξωτερική γραμμή και να
τοποθετηθεί στην εξωτερική πλευρά της, ώστε να ξεκινήσει perimeter traversal.

### Behavior

Το robot πρέπει:

- να επιλέξει εξωτερική γραμμή από labeled court model,
- να κινηθεί σε target pose με line offset,
- να κρατήσει το line στην κατάλληλη πλευρά,
- να αποφύγει εμπόδια με LiDAR,
- να μη χρησιμοποιήσει internal/service lines ως navigation target.

### Failure Conditions

- `EXTERNAL_LINE_APPROACH_BLOCKED`
- `SAFE_OFFSET_POSE_NOT_FOUND`
- `LOCALIZATION_CONFIDENCE_TOO_LOW`

## Step 6 - Follow External Line And Map Obstacles

### Objective

Να γίνει πλήρης περιμετρική κίνηση μόνο πάνω στις εξωτερικές γραμμές, ενώ
χαρτογραφούνται φράχτες, εμπόδια, πάγκοι, στύλοι και free space.

### Behavior

Σε κάθε control tick:

```text
read sensors
detect visible lines
label visible lines using court model
select current outer boundary line
ignore service/internal lines
compute line-follow command
apply LiDAR safety
record obstacle/fence/free-space evidence
write checkpoint update
send motion command
```

### Obstacle Mapping

LiDAR points πρέπει να ταξινομούνται σε:

- fence/wall,
- net post,
- bench,
- fixed obstacle,
- temporary obstacle,
- unknown obstacle,
- free-space sample.

Για κάθε obstacle:

```json
{
  "id": "obs_012",
  "type": "bench",
  "x_m": 4.2,
  "y_m": -6.8,
  "distance_to_nearest_court_line_m": 1.4,
  "confidence": 0.66,
  "first_seen_at": 1780450000.0,
  "last_seen_at": 1780450021.5
}
```

### Failure Conditions

- `CURRENT_EXTERNAL_LINE_LOST`
- `INTERNAL_LINE_CONFUSED_WITH_EXTERNAL`
- `OBSTACLE_TOO_CLOSE`
- `PATH_BLOCKED`
- `LIDAR_MAPPING_INCONSISTENT`

## Step 7 - Close Perimeter Loop

### Objective

Να ολοκληρωθεί το perimeter loop μόνο όταν το robot επιστρέψει στο starting
perimeter reference με επαρκή distance traveled, corner evidence και map
coverage.

### Completion Conditions

Το loop closure απαιτεί:

- επιστροφή κοντά στο starting spot,
- πλήρη κάλυψη των outer boundary segments,
- αρκετές εξωτερικές γωνίες,
- dimension consistency,
- όχι σημαντικά unexplored perimeter gaps,
- localization confidence πάνω από threshold.

### Failure Conditions

- `LOOP_CLOSURE_FAILED`
- `PERIMETER_SEGMENT_MISSING`
- `COURT_DIMENSIONS_INVALID`
- `INSUFFICIENT_CORNER_EVIDENCE`

## Step 8 - Return To Initial Position

### Objective

Μετά το successful survey, το robot επιστρέφει στο αρχικό σημείο ή σε
προκαθορισμένο home pose.

### Behavior

Το return path πρέπει να χρησιμοποιεί το newly built court model και obstacle
map. Δεν πρέπει να περνάει μέσα από blocked ή unknown unsafe regions.

### Failure Conditions

- `RETURN_TO_START_FAILED`
- `RETURN_PATH_BLOCKED`
- `HOME_POSE_UNREACHABLE`

Αν το survey model έχει ολοκληρωθεί αλλά το return αποτύχει, το survey result
μπορεί να μείνει `SUCCESS_WITH_RETURN_FAILURE`, αλλά το robot πρέπει να
σταματήσει και να αναφέρει καθαρά το failure.

## Checkpoint Contract

Κάθε step πρέπει να γράφει checkpoint.

Προτεινόμενο αρχείο:

```text
runtime/survey_checkpoint.json
```

Schema:

```json
{
  "survey_id": "survey_2026_06_03_001",
  "status": "running",
  "current_step": "locate_net",
  "started_at": 1780450000.0,
  "updated_at": 1780450012.5,
  "initial_pose": {"x_m": 1.0, "y_m": -2.0, "yaw_rad": 0.0},
  "robot_pose": {"x_m": 1.2, "y_m": -2.4, "yaw_rad": 0.1},
  "steps": [
    {
      "name": "locate_net",
      "status": "complete",
      "started_at": 1780450000.0,
      "completed_at": 1780450011.0,
      "confidence": 0.82,
      "failure_code": null,
      "failure_reason": null,
      "outputs": {}
    }
  ]
}
```

## Failure Handling

Το survey πρέπει να είναι fail-fast.

Αν ένα βήμα αποτύχει:

1. στέλνει stop command,
2. γράφει checkpoint `failed`,
3. γράφει `failure_code`,
4. γράφει ανθρώπινα κατανοητό `failure_reason`,
5. γράφει telemetry event,
6. ενημερώνει dashboard/status,
7. δεν συνεχίζει στο επόμενο step,
8. επιστρέφει το command mode σε `idle` ή `survey_failed`.

Παράδειγμα:

```json
{
  "survey_id": "survey_2026_06_03_001",
  "current_step": "confirm_net_visual",
  "status": "failed",
  "failure_code": "NET_VISUAL_CONFIRMATION_FAILED",
  "failure_reason": "LiDAR found candidate net posts, but camera/depth could not confirm a net-like structure between them.",
  "robot_pose": {"x_m": 2.14, "y_m": -1.82, "yaw_rad": 0.42},
  "confidence": 0.31,
  "safe_to_continue": false
}
```

## Court Model Output

Προτεινόμενο αρχείο:

```text
runtime/court_model.json
```

Το τελικό court model πρέπει να περιλαμβάνει:

```json
{
  "status": "success",
  "court_frame": {
    "net_center": [0.0, 0.0],
    "long_axis": [1.0, 0.0],
    "width_axis": [0.0, 1.0]
  },
  "court_lines": [
    {
      "id": "outer_baseline_a",
      "type": "outer_boundary",
      "label": "baseline",
      "p1": [-11.885, -5.485],
      "p2": [-11.885, 5.485],
      "confidence": 0.88
    },
    {
      "id": "service_line_a",
      "type": "internal_line",
      "label": "service_line",
      "p1": [-6.40, -4.115],
      "p2": [-6.40, 4.115],
      "confidence": 0.81
    }
  ],
  "net": {
    "p1": [0.0, -5.5],
    "p2": [0.0, 5.5],
    "posts": [
      {"x_m": 0.0, "y_m": -5.5},
      {"x_m": 0.0, "y_m": 5.5}
    ],
    "confidence": 0.84
  },
  "fences": [],
  "obstacles": [],
  "free_space": [],
  "accessibility": {
    "reachable_regions": [],
    "blocked_regions": []
  }
}
```

## Runtime Interfaces

Η νέα σχεδίαση πρέπει να χωρίζει το survey logic από το runtime backend.

```text
Survey Pipeline
  -> RobotRuntime interface
      -> WebotsRuntime adapter
      -> RosRuntime adapter
```

Το survey δεν πρέπει να ξέρει αν τρέχει σε Webots ή σε φυσικό robot.

### SensorInput

```python
@dataclass
class SensorInput:
    timestamp_s: float
    robot_pose: Pose2D
    camera_frame: object | None
    depth_frame_m: object | None
    lidar_scan: object | None
    odometry: object | None
```

### MotionCommand

```python
@dataclass
class MotionCommand:
    linear_speed_m_s: float
    angular_speed_rad_s: float
    stop_reason: str | None = None
```

### RobotRuntime

```python
class RobotRuntime(Protocol):
    def read_sensors(self) -> SensorInput: ...
    def send_motion(self, command: MotionCommand) -> None: ...
    def stop(self, reason: str) -> None: ...
    def publish_status(self, status: dict) -> None: ...
    def write_checkpoint(self, checkpoint: dict) -> None: ...
```

### SurveyStep

```python
class SurveyStep(Protocol):
    name: str
    def update(self, sensors: SensorInput, context: SurveyContext) -> StepResult: ...
```

### StepResult

```python
@dataclass
class StepResult:
    status: Literal["running", "complete", "failed"]
    motion_command: MotionCommand
    checkpoint: dict
    outputs: dict
    confidence: float
    failure_code: str | None = None
    failure_reason: str | None = None
```

## Webots Adapter

Το Webots adapter είναι υπεύθυνο για:

- `camera.getImage()` -> `SensorInput.camera_frame`,
- `range_finder.getRangeImage()` -> `SensorInput.depth_frame_m`,
- `lidar.getRangeImage()` -> `SensorInput.lidar_scan`,
- `Supervisor.getSelf().getPosition()` -> `SensorInput.robot_pose`,
- `MotionCommand` -> `Motor.setVelocity(...)`,
- status/checkpoints -> runtime JSON files.

Η μετατροπή `linear_speed_m_s` και `angular_speed_rad_s` σε left/right wheel
velocity μένει στο adapter layer.

## ROS Adapter

Το ROS adapter είναι υπεύθυνο για αντίστοιχο mapping:

- camera topic -> `SensorInput.camera_frame`,
- depth topic -> `SensorInput.depth_frame_m`,
- `/scan` -> `SensorInput.lidar_scan`,
- odometry/localization topic -> `SensorInput.robot_pose`,
- `MotionCommand` -> `/cmd_vel`,
- status -> diagnostics/status topic,
- checkpoints -> file, service, ή mission storage.

Η πρώτη υλοποίηση μπορεί να έχει μόνο Webots adapter, αλλά τα contracts πρέπει
να σχεδιαστούν ώστε το ROS adapter να μπει χωρίς αλλαγή στο survey pipeline.

## Telemetry And Dashboard

Το dashboard πρέπει να δείχνει:

- current survey step,
- checkpoint status,
- failure reason,
- confidence,
- detected net posts,
- confirmed net line,
- labeled visible line,
- current followed outer line,
- ignored internal line count,
- obstacle/fence samples,
- loop closure progress,
- return-to-start status.

Minimum telemetry fields:

```text
survey.status
survey.current_step
survey.failure_code
survey.failure_reason
survey.step_confidence
survey.net.confirmed
survey.net.confidence
survey.line.current_label
survey.line.current_type
survey.line.ignored_internal_count
survey.obstacles.count
survey.fences.count
survey.loop.progress_pct
survey.return.status
```

## Definition Of Done

Το νέο survey design θεωρείται υλοποιημένο όταν:

- το survey έχει explicit step pipeline,
- κάθε step γράφει checkpoint,
- κάθε failure σταματάει το robot με structured reason,
- το net εντοπίζεται πρώτα από LiDAR post candidates,
- το net επιβεβαιώνεται οπτικά από camera/depth,
- παράγεται court reference frame,
- οι γραμμές ταξινομούνται με label,
- service/internal lines δεν χρησιμοποιούνται για perimeter following,
- το robot κινείται στην εξωτερική πλευρά της εξωτερικής γραμμής,
- το obstacle/fence map ενημερώνεται κατά το perimeter traversal,
- το loop closure ελέγχεται με coverage και geometry consistency,
- το robot επιστρέφει στο initial pose ή αναφέρει structured return failure,
- η ίδια survey logic μπορεί να τρέξει μέσω Webots adapter και ROS adapter.

## Migration From Current Code

Η τωρινή υλοποίηση σε `controllers/ball_detector/survey.py` μπορεί να
χρησιμοποιηθεί μόνο ως legacy reference για:

- basic camera line detection,
- line offset / heading error control,
- LiDAR safety,
- telemetry ideas,
- output JSON patterns.

Δεν πρέπει να παραμείνει ως source of truth για το νέο map court behavior,
επειδή δεν ταξινομεί γραμμές και μπορεί να μπερδέψει service lines με external
perimeter lines.

