# Tennis Robot Mission Dashboard Plan

Αυτό το έγγραφο ορίζει την εξέλιξη του σημερινού Webots controller console σε
πραγματικό Mission Control Center. Στόχος είναι το ίδιο UI να μπορεί αργότερα να
δουλέψει σχεδόν αμετάβλητο είτε το robot τρέχει στο Webots είτε σε πραγματικό
γήπεδο.

## Objective

Ο operator πρέπει να μπορεί να καταλάβει άμεσα:

- τι κάνει τώρα το robot,
- πόση πρόοδο έχει η αποστολή,
- ποιες περιοχές έχουν σαρωθεί,
- τι βλέπουν οι sensors,
- πόσο αποδοτικά συλλέγει μπάλες,
- αν λειτουργεί με ασφάλεια,
- τι συνέβη τα τελευταία λεπτά χωρίς να ανοίξει raw logs.

## 1. Phase 1 - Mission Dashboard

### Current State

Το panel που υπάρχει ήδη δείχνει:

- Requested Mode,
- Actual Mode,
- Controller State,
- Sequence,
- Updated.

### Target State

Να εξελιχθεί σε mission-first summary:

```text
Mission:
Collect All Balls

Progress:
12 / 25 balls

Current Zone:
Zone B

Current Action:
Approaching Ball

Current Target:
Ball #14
```

### Required Fields

```text
mission_name
balls_collected
balls_total_estimated
current_zone
current_action
current_target_id
mission_elapsed_s
coverage_pct
```

### DoD

Ο operator μπορεί να καταλάβει σε 5 δευτερόλεπτα τι κάνει τώρα το robot.

## 2. Phase 2 - Court Progress Map

Το υπάρχον Collection Map είναι η βάση για mission progress visualization.

### Court Zones

Να εμφανίζονται οι βασικές ζώνες:

```text
Zone A - Left Baseline
Zone B - Right Baseline
Zone C - Net Area
Zone D - Center Court
Zone E - Corners
Zone F - Outside Court / buffer
```

### Zone Colors

| Color | Meaning |
| --- | --- |
| Grey | Not scanned |
| Blue | Scanning |
| Green | Completed |
| Orange | Ball detected |
| Red | Blocked |

### Coverage

```text
Court Coverage: 67%
```

### Ball Statistics

```text
Detected: 22
Confirmed: 19
Collected: 17
Missed: 2
```

### Required Fields

```text
zones[].id
zones[].label
zones[].status
zones[].coverage_pct
zones[].detected_count
zones[].collected_count
court_coverage_pct
balls_detected
balls_confirmed
balls_collected
balls_missed
```

### DoD

Ο operator μπορεί να δει ποιες περιοχές έχουν ήδη σαρωθεί και πού υπάρχουν στόχοι.

## 3. Phase 3 - Robot Timeline

Νέο tab:

```text
Mission Timeline
```

### Example

```text
10:22:01 Mission Started
10:22:14 Ball Detected
10:22:17 Route Planned
10:22:24 Ball Collected
10:22:31 Human Detected
10:22:45 Resume Search
```

### Required Event Fields

```text
timestamp
event_type
message
state
zone_id
target_id
severity
```

### DoD

Να μπορείς να εξηγήσεις τι συνέβη τα τελευταία 5 λεπτά χωρίς να ανοίξεις logs.

## 4. Phase 4 - Live Sensor Awareness

Νέο panel:

```text
Perception
```

### OAK-D

```text
Balls Visible: 3
People Visible: 1
Confidence: 92%
```

### LiDAR

```text
Obstacles: 4
Blocked Paths: 1
Safe Heading: 145°
```

### Required Fields

```text
oak_balls_visible
oak_people_visible
oak_best_confidence
lidar_obstacles
lidar_blocked_paths
lidar_safe_heading_deg
```

### DoD

Να βλέπεις τι "βλέπει" το robot.

## 5. Phase 5 - Collection Performance

Νέο dashboard:

```text
Collection Efficiency
```

### Metrics

- Balls Detected,
- Balls Collected,
- Collection Success Rate,
- Collection Attempts,
- Retries,
- Average Collection Time,
- Average Search Time.

### Example

```text
Detected: 38
Collected: 34
Success Rate: 89%
Average Time: 22 sec/ball
```

### Required Fields

```text
balls_detected
balls_collected
collection_attempts
collection_retries
collection_success_rate
avg_collection_time_s
avg_search_time_s
```

### DoD

Να μπορείς να απαντήσεις πόσο αποδοτικό είναι το robot.

## 6. Phase 6 - Safety Dashboard

Αυτό είναι κρίσιμο για να αποδεικνύεται ότι το robot λειτουργεί με ασφάλεια.

### Counters

- Human Encounters,
- Emergency Stops,
- Obstacle Avoidance Events,
- Blocked Targets.

### Example

```text
Human Encounters: 12
Avoidance Events: 17
Emergency Stops: 0
Blocked Targets: 3
```

### Required Fields

```text
human_encounters
emergency_stops
obstacle_avoidance_events
blocked_targets
near_collision_events
```

### DoD

Να μπορείς να αποδεικνύεις ότι το robot λειτουργεί με ασφάλεια.

## 7. Phase 7 - Mission Completion Report

Όταν τελειώνει η αποστολή, το UI πρέπει να εμφανίζει report.

### Example

```text
Mission:
Collect All Balls

Duration:
14m 12s

Coverage:
100%

Balls Detected:
31

Balls Collected:
29

Missed:
2

Human Encounters:
5

Replans:
12
```

### Future Exports

- PDF Report,
- JSON Report,
- CSV Metrics.

### Required Fields

```text
mission_name
mission_started_at
mission_completed_at
duration_s
coverage_pct
balls_detected
balls_collected
balls_missed
human_encounters
replans
```

### DoD

Μετά από κάθε mission υπάρχει συνοπτικό report που εξηγεί αποτέλεσμα, απόδοση και safety behavior.

## 8. Final Dashboard Vision

Το τελικό UI πρέπει να έχει:

```text
Dashboard
├── Mission Status
├── Court Map
├── Sensor Views
├── Telemetry
├── Timeline
├── Performance
├── Safety
├── History
└── Reports
```

Το σημαντικότερο KPI μπαίνει στην κορυφή:

```text
Mission Progress

Collected 17 / 25 balls
Coverage 82%
Elapsed Time 08:42
Current State: Approaching Ball
```

Αυτό μετατρέπει το σημερινό UI από Webots Controller Console σε πραγματικό
Mission Control Center.

## 9. Implementation Order

Η προτεινόμενη σειρά υλοποίησης είναι:

1. Add mission summary fields to controller status JSON.
2. Rename/reframe the top dashboard KPIs around mission progress.
3. Add zone state model and render it in the Collection Map.
4. Add in-memory mission timeline events.
5. Add perception summary counters from OAK-D and LiDAR data.
6. Add performance and safety counters.
7. Add mission completion report JSON.
8. Add CSV/JSON export, then PDF export later.

## 10. First Webots UI Target

Η πρώτη μικρή υλοποίηση στο υπάρχον control panel πρέπει να δείχνει:

1. `Collected X / Y balls`.
2. `Coverage %`.
3. `Current Zone`.
4. `Current Action`.
5. `Current Target`.
6. Zone colors στο Collection Map.
7. Timeline με τα τελευταία mission events.
