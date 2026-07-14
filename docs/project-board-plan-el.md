# GitHub Project Board Plan

Προτεινόμενο όνομα board:

```text
Tennis Robot Build
```

## Current Operating Model

Το board πρέπει να είναι project-oriented και evidence-oriented, ώστε να
λειτουργεί αργότερα ως καθαρό public proof-of-work για crowdfunding, όχι μόνο
ως εσωτερική task list.

Το κύριο grouping γίνεται με το custom field `Project Area`.

Top-level project areas:

| Project Area | Purpose |
|---|---|
| Robot Design / Features / Expectations | Τι robot χτίζουμε, ποια features περιμένουμε, ποιοι φυσικοί/λειτουργικοί περιορισμοί ισχύουν. |
| Simulation Environment | Πώς αποδεικνύουμε συμπεριφορές σε Webots/Gazebo/ROS πριν πάμε σε hardware. |
| Court Survey & Knowledge Model | Πώς το robot καταλαβαίνει το γήπεδο, αποθηκεύει survey identity και χτίζει court model. |
| Intake Research | Πώς επιλέγουμε και επικυρώνουμε τον φυσικό μηχανισμό συλλογής μπάλας. |
| Collection Research Patterns | Πώς αποφασίζει το robot τι να μαζέψει, με ποια σειρά, και πώς κάνει rescan/replan/recovery. |

Τα παλιά workstreams μένουν χρήσιμα ως secondary classification, αλλά δεν
πρέπει να οδηγούν την ιεραρχία του board. Για παράδειγμα, `Collector Intake
Prototype` είναι feature/workstream κάτω από `Intake Research`, όχι ανεξάρτητο
top-level epic.

## Future Update Rules

Κάθε νέα ενημέρωση στο GitHub Project πρέπει να ακολουθεί αυτούς τους κανόνες:

1. Κάθε νέο item παίρνει πάντα `Project Area`.
2. Τα top-level items χρησιμοποιούν prefix `Epic:` και πρέπει να αντιστοιχούν
   σε ένα από τα 5 project areas.
3. Κάτω από τα epics, χρησιμοποιούμε `Feature:`, `[Research]`, `[Validation]`
   ή απλό task title. Δεν δημιουργούμε νέο top-level `Epic:` αν χωράει σε
   υπάρχον project area.
4. Research items κρατούν decision trail: branch, PR, docs, evidence, outcome
   και stop/continue gate.
5. Rejected concepts δεν διαγράφονται και δεν κρύβονται. Κλείνουν ως `Done`
   με σαφές evidence, ώστε να φαίνεται ότι η απόφαση ήταν engineering result.
6. Active build views πρέπει να φιλτράρουν με `-status:Done`, αλλά τα Done
   evidence items μένουν διαθέσιμα σε research/history views.
7. Για hardware-related work, δεν θεωρείται `Done` χωρίς τουλάχιστον ένα από:
   bench result, simulation result, physical test result, wiring/CAD doc, ή PR.
8. Για software-related work, δεν θεωρείται `Done` χωρίς PR, test/validation
   note, ή runtime evidence.
9. Αν ένα item αφορά και software και hardware, κρατάμε `Track=Mixed` και το
   βάζουμε στο project area που απαντάει στο κύριο project question.
10. Κάθε σημαντικό pivot πρέπει να έχει child research/validation item που
    εξηγεί γιατί κλείνει η παλιά κατεύθυνση και ποια είναι η νέα.

### Recommended Intake Views

Για το collector intake work, το βασικό board view είναι:

```text
project area:"Intake Research"
```

Για active build-only view:

```text
project area:"Intake Research" -status:Done
```

Για prototype/workbench view:

```text
project area:"Intake Research" workstream:"Collector Intake Prototype"
```

Αυτό πρέπει να δείχνει μαζί research history, active concept validation,
breadboard/perfboard tasks και physical tests, χωρίς να χάνεται το γιατί
απορρίφθηκε ή προτιμήθηκε κάθε μηχανική λύση.

Στόχος του board είναι να κρατάει καθαρά το hardware, software, simulation και
validation work για το tennis robot, χωρίς να μπερδεύονται τα παλιά search docs
με τη νέα survey/matrix collection διαδικασία.

## Status Columns

| Status | Meaning |
|---|---|
| Backlog | Ιδέες και εργασίες που δεν έχουν μπει ακόμα σε άμεσο scope. |
| Ready | Καθαρά tasks που μπορούν να ξεκινήσουν. |
| In Progress | Κάτι που δουλεύεται τώρα. |
| Blocked | Περιμένει part, απόφαση, μέτρηση ή εξωτερικό dependency. |
| Done | Ολοκληρώθηκε και έχει τεκμηρίωση ή validation. |

## Workstreams

### 1. Court Survey & Knowledge Model

Καλύτερη ονομασία για το αρχικό `Court Mapping`.

Scope:

- survey για συγκεκριμένο γήπεδο πριν από οποιοδήποτε collection,
- court bounds, net line, safe corridors και blocked/static obstacle areas,
- αποθήκευση `court_id`, `survey_id` και matrix metadata,
- validation ότι το loaded survey ταιριάζει στο τρέχον γήπεδο.
- σύνδεση με το active `court-knowledge-model-specification.md`.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define court matrix schema | Software | High |
| Store and load survey by `court_id` | Software | High |
| Add collection gate when survey is missing | Software | High |
| Visualize surveyed court bounds | Dashboard | Medium |
| Mark static blocked cells from survey | Mapping | Medium |

### 2. Matrix-Based Collection Planning

Καλύτερη ονομασία για το αρχικό `Collection`.

Scope:

- scan της επιλεγμένης πλευράς με βάση το matrix,
- ball observations ανά cell,
- confidence/freshness/risk ανά cell,
- ordered collection plan πριν ξεκινήσει η συλλογή,
- rescan/replan όταν χαθεί target ή αλλάξει η διαδρομή.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Generate scan viewpoints for selected side | Software | High |
| Attach ball observations to matrix cells | Perception | High |
| Build first greedy ordered collection plan | Planning | High |
| Add rescan trigger after missed target | Planning | Medium |
| Track target states: planned/approaching/collected/missed/blocked | Software | Medium |

### 3. Collector Intake Prototype

Καλύτερη ονομασία για το αρχικό `Collector Prototype`.

Scope:

- front intake / funnel / roller mechanism,
- DC motor, motor driver, power and wiring,
- sensor feedback για collection confirmation,
- mechanical fit πάνω στη βάση.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Bench-test DFRobot DC motor with driver | Hardware | High |
| Build first intake roller mount | Mechanical | High |
| Add collector power and control wiring | Electronics | High |
| Add collection confirmation sensor plan | Sensors | Medium |
| Test ball pickup on flat court-like surface | Validation | High |

### 4. Flywheel Launcher Prototype

Καλύτερη ονομασία για το αρχικό `FlyWheel Prototype`.

Scope:

- dual flywheel bench prototype,
- motor selection,
- speed control,
- guarded launcher geometry,
- repeatable launch speed testing.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define flywheel safety constraints | Safety | High |
| Select launcher motors and ESC/driver approach | Hardware | Medium |
| Build guarded bench flywheel rig | Mechanical | Medium |
| Measure launch consistency at low speed | Validation | Medium |
| Define interface from hopper to launcher | Mechanical | Medium |

### 5. Ball Launching System

Καλύτερη ονομασία για το αρχικό `Ball throwing`.

Scope:

- πλήρες σύστημα εκτόξευσης μπάλας,
- hopper/feed gate,
- spin/speed control,
- aim/trajectory control,
- operator safety and interlocks.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define launch modes and safety interlocks | Safety | High |
| Design hopper-to-flywheel feed gate | Mechanical | Medium |
| Add throw command contract | Software | Medium |
| Add launch telemetry fields | Dashboard | Low |
| Validate repeatable throw distance | Validation | Medium |

## Missing Workstreams To Add

### 6. Perception & Sensors

Χρειάζεται ξεχωριστό workstream γιατί OAK-D, LiDAR, odometry και front sensors
είναι cross-cutting.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define OAK-D ball observation contract | Perception | High |
| Define LiDAR obstacle/costmap contract | Sensors | High |
| Add front collection sensor contract | Sensors | Medium |
| Calibrate camera-to-court projection | Perception | High |
| Track confidence and stale detections | Perception | Medium |

### 7. Simulation & Validation

Χρειάζεται για να αποδεικνύουμε κάθε βήμα πριν πάει στο hardware.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Add survey -> matrix -> side scan smoke test | Simulation | High |
| Add planned collection Webots scenario | Simulation | High |
| Add blocked-cell scenario | Simulation | Medium |
| Add missed-target rescan scenario | Simulation | Medium |
| Export validation reports | Validation | Low |

### 8. Mission Dashboard & Telemetry

Το UI πρέπει να δείχνει survey status, matrix coverage και active plan.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Show survey status and `survey_id` | Dashboard | High |
| Render matrix cells and coverage | Dashboard | High |
| Show active collection plan | Dashboard | Medium |
| Add mission timeline events | Dashboard | Medium |
| Export mission report JSON | Dashboard | Low |

### 9. Power, Electronics & Wiring

Χρειάζεται για να μην μπλεχτούν motor drivers, 12V power, logic power και
sensors στο ίδιο bucket με mechanics.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define 12V and logic power distribution | Electronics | High |
| Wire Arduino Nano controller prototype | Electronics | High |
| Test TB6612FNG motor driver | Electronics | High |
| Add emergency stop wiring concept | Safety | High |
| Document connector map | Electronics | Medium |

### 10. Mechanical Base & Integration

Χρειάζεται ξεχωριστά από collector/launcher γιατί αφορά βάση, mounting,
serviceability και CAD.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Validate plywood base cut list | Mechanical | High |
| Define sensor mounting points | Mechanical | High |
| Define collector mounting interface | Mechanical | High |
| Reserve launcher mounting envelope | Mechanical | Medium |
| Add service access for wiring and batteries | Mechanical | Medium |

### 11. Navigation & Safety

Χρειάζεται ξεχωριστά γιατί η ασφαλής κίνηση επηρεάζει survey, collection,
collector testing και αργότερα throwing.

Initial cards:

| Title | Type | Priority |
|---|---|---|
| Define safe speed limits per mode | Safety | High |
| Add obstacle stop and resume behavior | Navigation | High |
| Add human proximity stop behavior | Safety | High |
| Validate planned path before target approach | Navigation | High |
| Define blocked-path recovery flow | Navigation | Medium |

## Suggested Labels

```text
area:mapping
area:collection-planning
area:collector-intake
area:launcher
area:perception
area:simulation
area:dashboard
area:electronics
area:mechanical
area:safety
area:navigation
type:hardware
type:software
type:docs
type:validation
priority:high
priority:medium
priority:low
blocked:parts
blocked:decision
```

## First Board Setup

Recommended first cards to put in `Ready`:

1. Define court matrix schema.
2. Store and load survey by `court_id`.
3. Add collection gate when survey is missing.
4. Generate scan viewpoints for selected side.
5. Bench-test DFRobot DC motor with driver.
6. Test TB6612FNG motor driver.
7. Show survey status and `survey_id`.

Everything else can start in `Backlog`.
