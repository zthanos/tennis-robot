# Tennis Robot Architecture Implementation Guide

Αυτό είναι το βασικό architecture guide για την επόμενη φάση υλοποίησης.
Από εδώ και πέρα, η υλοποίηση πρέπει να ξεκινά από τα active baseline
documents και όχι από την τρέχουσα μορφή του Webots controller.

## 1. Architecture Reset Decision

Ο υπάρχων κώδικας θεωρείται **legacy reference**.

Αυτό σημαίνει:

- δεν είναι πλέον η πηγή αλήθειας για τη νέα αρχιτεκτονική,
- μπορεί να χρησιμοποιηθεί για χρήσιμα κομμάτια, patterns, smoke tests και telemetry ideas,
- δεν πρέπει να περιορίζει τον νέο σχεδιασμό,
- μπορεί να αντικατασταθεί σταδιακά ή πλήρως όταν συγκρούεται με τα νέα baseline docs.

Η νέα πηγή αλήθειας είναι τα active baseline documents:

- `docs/process/validation-plan-el.md`
- `docs/survey/court-knowledge-model-specification.md`
- `docs/archive/collection-state-machine-plan-el.md` (ιστορικό, μη ενεργό)
- `docs/collection-route/mission-dashboard-plan-el.md`
- `docs/mechanism/concept-a-funnel-lift-wheel-plan.md`
- `docs/hardware/prototype-purchase-list-el.md`
- `docs/hardware/plywood-cut-list.md`

## 2. Implementation Principle

Κάθε νέα υλοποίηση πρέπει να απαντά πρώτα:

```text
Ποιο active baseline requirement υλοποιεί;
Ποιο sensor contract χρησιμοποιεί;
Ποιο state machine state επηρεάζει;
Ποιο telemetry/dashboard field ενημερώνει;
Ποιο validation DoD βοηθάει να αποδειχθεί;
```

Αν μια αλλαγή δεν μπορεί να συνδεθεί με κάποιο από αυτά, πρέπει να θεωρείται
πειραματική και όχι baseline implementation.

## 3. Webots Rebuild Direction

Η επόμενη Webots φάση πρέπει να ανακατασκευάσει το robot ώστε να καλύπτει τη
γενική σχεδίαση αντί να συντηρεί το παλιό minimal demo.

### Required Webots Robot Shape

Το simulated robot πρέπει να περιλαμβάνει:

- differential-drive mobile base,
- front collector placeholder με wide intake/funnel geometry,
- simulated intake zone,
- low 360-degree LiDAR στα 25-35 cm,
- OAK-D approximation στα 40-60 cm με ελαφριά κλίση προς τα κάτω,
- front low collection sensors ή simulated collection trigger,
- clear coordinate frames για base, camera, LiDAR και collector center,
- telemetry output που μπορεί να τροφοδοτήσει το Mission Dashboard.

### Required Webots Environment

Το court πρέπει να υποστηρίζει:

- full ή half-court Court Knowledge Model creation και selected-side scan,
- Court Knowledge Model-gated court matrix,
- selectable collection side,
- tennis balls σε realistic και edge-case θέσεις,
- static obstacles,
- moving human/person obstacles,
- blocked paths,
- balls κοντά σε γραμμές, δίχτυ, γωνίες και σκιά όπου είναι εφικτό.

## 4. Target Runtime Architecture

Η νέα υλοποίηση πρέπει να χωρίζεται σε layers:

```text
Sensors
  ↓
Perception
  ↓
World / Court Model
  ↓
Court Knowledge Model / Side Scan
  ↓
Collection Planning
  ↓
Target Selection
  ↓
Collection State Machine
  ↓
Safety / Recovery
  ↓
Actuation
  ↓
Telemetry / Mission Dashboard
```

### Layer Responsibilities

| Layer | Responsibility |
| --- | --- |
| Sensors | Raw camera, depth, LiDAR, odometry, front collection sensors. |
| Perception | Ball/person observations, confidence, bearing, distance. |
| World / Court Model | Court Knowledge Model, matrix cells, coverage, obstacle map, ball map, target states. |
| Court Knowledge Model / Side Scan | Require a valid Court Knowledge Model, scan the selected collection side, and update matrix cells. |
| Collection Planning | Build an ordered collection plan from matrix data, current pose, obstacles and target confidence. |
| Target Selection | Choose the next planned target using distance, confidence, stale data and obstacle cost. |
| Collection State Machine | Approach, fine alignment, attempt, verification, retry. |
| Safety / Recovery | Human stop, blocked path, stuck robot, lost ball, full bucket, jam. |
| Actuation | Drive commands and collector commands. |
| Telemetry / Dashboard | Mission progress, timeline, safety, performance and report fields. |

## 5. Sensor Contracts

### LiDAR

```text
LiDAR = navigation, safety, obstacle map
```

LiDAR outputs:

```text
obstacle_sectors
blocked_paths
safe_heading_deg
occupancy_or_cost_map
human_or_obstacle_events
```

Δεν χρησιμοποιείται ως κύριο sensor για ball recognition.

### OAK-D / Camera

```text
OAK-D = ball/person recognition
```

OAK-D outputs:

```text
balls_visible
people_visible
ball_observations[]τ
target_confidence
ball_bearing_rad
ball_distance_m
ball_world_xy
```

### Front Collection Sensors

```text
Low front sensors = final collection confirmation
```

Collection sensor outputs:

```text
ball_present
ball_crossed_entry
stored_ball_count
collection_confirmed
jam_or_partial_capture
```

## 6. Core State Machines

### Court Knowledge Model-Gated Collection Planning

Το collection planning behavior ακολουθεί:

```text
NO_COURT_KNOWLEDGE_MODEL
↓
BUILDING_COURT_KNOWLEDGE_MODEL
↓
COURT_KNOWLEDGE_MODEL_READY
↓
SIDE_SCAN
↓
PLAN_COLLECTION
↓
EXECUTE_COLLECTION_PLAN
```

Source: `docs/survey/court-knowledge-model-specification.md`

### Collection

Το collection behavior ακολουθεί:

```text
BALL_DETECTED
↓
TARGET_SELECTED
↓
APPROACH_TARGET
↓
FINE_ALIGNMENT
↓
COLLECTION_ATTEMPT
↓
VERIFY_COLLECTION
↓
SUCCESS
↓
RESUME_SEARCH
```

Historical source: `docs/archive/collection-state-machine-plan-el.md`.
For the active `collect_route` behavior use `docs/collection-route/collection-route-rules-el.md`.

### Mission Dashboard

Κάθε state transition πρέπει να ενημερώνει:

```text
mission progress
current zone
current action
current target
timeline events
performance counters
safety counters
```

Source: `docs/collection-route/mission-dashboard-plan-el.md`

## 7. First Implementation Scope

Η πρώτη νέα υλοποίηση πρέπει να είναι μικρή αλλά πλήρης σαν vertical slice:

1. Rebuild/clean Webots robot model with target sensor layout.
2. Implement or adapt perception outputs into stable observation contracts.
3. Add Court Knowledge Model storage and require a valid model before collection.
4. Add matrix cell model for the selected court.
5. Add selected-side scan that updates matrix cells with ball observations and blocked cells.
6. Add ordered collection plan generation from matrix data.
7. Add target selection from planned targets.
8. Add collection state machine with approach, fine alignment, attempt and verification.
9. Add retry with `max_retries = 3`.
10. Add mission dashboard fields and timeline events.
11. Add smoke/regression scenario for a Court Knowledge Model -> side scan -> planned collection mission.

## 8. Migration Rule

When touching existing code:

- keep useful code only if it supports the new contracts,
- prefer extracting reusable pieces over extending legacy state tangles,
- do not preserve old names or states just because they exist,
- document any temporary compatibility layer,
- remove compatibility once the new state machine is stable.

Examples of legacy concepts that may be reused but should not define the new design:

- old `idle -> scan -> align -> approach -> capture -> collected` collector flow,
- old dashboard KPIs centered on requested/actual mode,
- route visualization presets that do not know Court Knowledge Model status, matrix cells or mission progress,
- simulated collection without verification semantics.

## 9. Telemetry Contract

Every implementation step should update at least one telemetry group:

```text
mission.*
scan.*
court_knowledge.*
matrix.*
target.*
collection.*
perception.*
navigation.*
safety.*
performance.*
```

Minimum fields for the first dashboard:

```text
mission_name
mission_elapsed_s
current_action
current_cell
court_knowledge_status
court_knowledge_model_id
collection_side
matrix_coverage_pct
current_target_id
coverage_pct
balls_detected
balls_collected
collection_attempts
collection_retries
human_encounters
obstacle_avoidance_events
timeline_events[]
```

## 10. Validation Rule

No implementation phase is considered complete unless it produces evidence for
the validation plan:

- repeatable Webots scenario,
- telemetry or dashboard output,
- pass/fail behavior for the relevant DoD,
- documented limitation if Webots cannot prove the behavior.

Source: `docs/process/validation-plan-el.md`

## 11. Recovery Plan Placeholder

Μετά το πρώτο vertical slice, χρειάζεται ξεχωριστό Recovery Plan για:

- lost ball,
- stuck robot,
- blocked path,
- full bucket,
- jammed collector,
- stale target,
- unsafe human proximity.

Μέχρι να γραφτεί αυτό το plan, οι υλοποιήσεις πρέπει να κρατούν recovery hooks
και telemetry events, αλλά να μην προσπαθούν να λύσουν όλες τις περιπτώσεις μαζί.

## 12. Definition Of Ready For New Work

Πριν ξεκινήσει νέα υλοποίηση, πρέπει να είναι σαφές:

1. Ποιο baseline document αφορά.
2. Ποιο state machine αλλάζει.
3. Ποια telemetry fields θα γράφει.
4. Ποιο Webots scenario θα το αποδεικνύει.
5. Ποιο legacy behavior μπορεί να αντικατασταθεί.

Αυτός είναι ο κανόνας για να μη γυρίσει το project σε ad-hoc simulation demo.
