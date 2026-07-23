# Φάση 7 — Gazebo acceptance plan (continuous collect_route)

> Κατάσταση: **ενεργό πλάνο εκτέλεσης** για τη Φάση 7 του continuous-route
> rewrite. Οι Φάσεις 6A–6D.5 (cutover) είναι ολοκληρωμένες και pushed στο
> `feat/collection-pattern` (HEAD `b7b0a11`): το `collect_route` τρέχει 100% τον
> νέο `CollectionRouteExecutor` → C++ `CollectionFollowPath`, κανένα legacy.
> Η Φάση 7 ΔΕΝ επαληθεύεται με unit tests — είναι **πραγματικά sim runs +
> παρατήρηση**. Reference: `docs/collection-route-{rules,design,implementation-plan}-el.md`.

Το dev γίνεται σε **native Linux** — entry point `./run_ubuntu.sh` (όχι WSL).

---

## 0. Pre-run setup (μία φορά, πριν το πρώτο scenario)

- [ ] **Overlay rebuild**: μετά τη 6D, ο `tennis_robot_collection_controller`
      χτίζεται από τον dev overlay (`docker_dev_entry.sh`) και τα `Collection*`
      msgs ξαναχτίζονται. Το **πρώτο** `./run_ubuntu.sh` θα ξαναχτίσει overlay
      (msgs + controller + tennis_robot) — λίγα λεπτά. Αν τρέχεις με
      `DEV_OVERLAY=false` (baked-only), χρειάζεσαι **image rebuild**
      (`docker compose -f docker-compose.yml -f docker-compose.ubuntu.yml build gazebo`)
      ώστε το `COPY` του Dockerfile.gazebo να πιάσει το plugin.
- [ ] **Επιβεβαίωσε ότι φορτώνει το plugin**: στα logs του controller_server →
      `Created controller : CollectionFollowPath of type
      tennis_robot_collection_controller::CollectionNav2Controller`.
- [ ] **Config wiring** (fail-loud αν λείπουν — ο executor δεν κατασκευάζεται):
  - [ ] `config/collection_route.yaml` παρόν + πλήρες (12 groups).
  - [ ] `COLLECTION_ROUTE_CALIBRATION_ARTIFACT` env → το ίδιο v2 artifact με τον
        perception producer (`calibration_artifacts/gazebo/...v2.json`).
  - [ ] `nav2_params.yaml`: υπάρχει το `collection_route_executor` tuning block
        (5 πεδία) **και** ο `CollectionFollowPath` controller.
- [ ] **Υπάρχει surveyed court**: `runtime/court_boundary.json`
      (schema `court_knowledge_model/v2`, `status: OK`, `completed: true`).
      Αν όχι, τρέξε πρώτα το court survey (map_court/court_survey) — ο
      CourtModel builder fail-loud χωρίς αυτό.

## 1. Πώς τρέχεις & πού παρατηρείς

- **Launch**: `./run_ubuntu.sh` (Gazebo GUI + RViz). Software fallback:
  `UBUNTU_GPU=false ./run_ubuntu.sh`. Headless: `GAZEBO_HEADLESS=true`.
- **Trigger `collect_route`**: web control panel **http://127.0.0.1:8081**
  (γράφει `/robot/command`). Επιβεβαίωσε mode → `collect_route`.
- **Παρατήρηση**:
  - Web console 8081: `collection_run` / `collect_route` status (executor-backed:
    `executor.state`, `planning_status`, `ball_results`, crossings), Collection Map.
  - RViz: robot path, `/scan`, TF, το FollowPath path.
  - `runtime/collection_events.jsonl`: typed events (scan/plan/safety/collector).
  - ROS topics: `/CollectionFollowPath/state` (CollectionControllerState —
    lifecycle/progress/failure/profile verdict), `/follow_path` action, `/cmd_vel_*`.
  - twist_mux: scan-rotation → `/cmd_vel_collection` (prio 70), FollowPath →
    `/cmd_vel_nav` (50), teleop 100.

## 2. Evidence να κρατάς ΑΝΑ run (spec §«Evidence per run»)

- [ ] Saved **ScanSnapshot** + **CollectionRoutePlan** (plan_id, planning_status,
      segments, ball_results).
- [ ] **ExecutedTrajectory** με progress + per-crossing metrics (measured speed,
      lateral/heading error, profile-compliance verdict).
- [ ] **Ball result table**: κάθε ball_id → `status` (covered/deferred/unreachable)
      + `reason_code`.
- [ ] Controller/profile evidence (`/CollectionFollowPath/state`), Nav2 action log,
      collector/safety events.

---

## 3. Τα 8 acceptance scenarios

Για κάθε ένα: **Setup → Expected → Pass criteria**. Σημείωσε PASS/FAIL + evidence.

### S1 — Empty scan & all-unreachable → `completed_no_targets`
- Setup: (α) άδεια πλευρά· (β) μόνο μπάλες σε keepout (net/fence/corner).
- Expected: scan ολοκληρώνεται, planner → `empty_no_balls` ή
  `empty_no_feasible_targets`, executor → `completed_no_targets`, **χωρίς** να
  ξεκινήσει collector.
- Pass: valid completion (όχι scan failure)· ball_results όλα unreachable στο (β).

### S2 — Πολλαπλές ελεύθερες μπάλες → μία frozen route
- Setup: 4–6 ελεύθερες μπάλες, ίδια πλευρά.
- Expected: **μία** frozen RoutePath, drive-through, **χωρίς** per-ball rotation/
  stop/reverse. Όλες covered (ή documented deferred).
- Pass: κανένα `NavigateToPose` per-ball, κανένα rotate/reverse στο action log·
  η route δεν αλλάζει μετά το scan.

### S3 — Κοντινές μπάλες → shared pass / connector / deferred
- Setup: 2–3 μπάλες πολύ κοντά (εντός shared-pass spacing).
- Expected: shared pass (κοινό heading) ή connector μεταξύ passes ή τεκμηριωμένο
  `deferred` με reason code — **όχι** greedy chase.
- Pass: το plan δείχνει shared/connector· ball_results συνεπή.

### S4 — Net/fence/corner → tangent ή deterministic unreachable
- Setup: μπάλες κοντά σε φιλέ/φράχτη + σε γωνία.
- Expected: tangent crossing (παράλληλα στο εμπόδιο) ή `unreachable/keepout`
  ντετερμινιστικά. Καμία μετωπική προσέγγιση εμποδίου.
- Pass: crossing heading ∥ tangent εντός `max_parallel_heading_error`· corner →
  unreachable αν δεν υπάρχει ασφαλής tangent corridor.

### S5 — Missed capture → route συνεχίζει χωρίς replan
- Setup: μπάλα που ο funnel αστοχεί (π.χ. οριακή lateral).
- Expected: η frozen route **συνεχίζει** αμετάβλητη· καταγράφεται
  `missed_collection` (ή `target_position_invalidated`)· **κανένα replan**.
- Pass: plan_id αμετάβλητο· καμία geometry mutation μετά το scan.

### S6 — Άνθρωπος/κινούμενο εμπόδιο → stop → valid forward resume ή abort
- Setup: βάλε moving obstacle/άνθρωπο στο μπροστινό sector κατά την εκτέλεση.
- Expected: `/scan` obstacle → SafetyMonitor BLOCKED → executor
  `waiting_path_clear` (stop, χωρίς geometry change). Καθαρίζει → resume **μόνο**
  forward (progress ≥ s_before_pause, tube ok, αρκετό run-in). Κοντά σε crossing
  χωρίς run-in → `aborted_safety/tracking` (όχι backtrack).
- Pass: stop χωρίς route change· resume μονότονο forward ή καθαρό abort.

### S7 — Collector fault / full hopper → `aborted_collector`
- Setup: προσομοίωσε collector start failure ή jam/full κατά την ενεργή route.
  ⚠️ **Carry-forward**: στο Gazebo MVP ο `GazeboCollectorAdapter.active_fault`
  επιστρέφει πάντα `None` (δεν υπάρχουν sim fault signals). Άρα το S7 μπορεί να
  ΜΗΝ είναι triggerable χωρίς πρώτα να wire-άρεις sim collector faults. Τεκμηρίωσε
  το αποτέλεσμα: είτε πραγματικό `aborted_collector`, είτε «not testable in sim —
  needs fault wiring».
- Pass (αν triggerable): `aborted_collector`, route outcome αμετάβλητο, όχι replan.

### S8 — Follow-up enabled/disabled & max-run limit
- Setup: UI follow-up OFF (1 run)· μετά ON με max_total_runs=N.
- Expected: OFF → ένα run τελειώνει, δείχνει deferred/unreachable/new-after-scan.
  ON → νέο ανεξάρτητο cycle από terminal pose (νέο scan + νέο frozen plan), έως N.
  **Μόνο** μετά από καθαρό `route_completed` (όχι μετά abort — 6D.4/F5 decision).
- Pass: run counter αυξάνει σωστά, σταματά στο N, κάθε follow-up = δικό του snapshot.

---

## 4. Carry-forwards να παρατηρήσεις ενεργά (πιθανά σημεία αστοχίας)

1. **Curve-tracking profile tuning** — τα `default_profile` limits
   (`max_lateral_error_m=0.1`, `max_heading_error_rad=0.1`) ίσως πολύ αυστηρά για
   πραγματικό curved tracking → ο controller μπορεί να αποτύχει με
   `FAILURE_TRAJECTORY_TUBE_EXCEEDED` / `HEADING_ERROR_EXCEEDED`. Αν συμβεί:
   χαλάρωσε τα profile limits στο `collection_route.yaml` (planning group) και
   ξανατρέξε — τεκμηρίωσε τις τιμές που δουλεύουν.
2. **Fail-safe stale-scan watchdog** — επιβεβαίωσε ότι απώλεια `/scan` → BLOCKED
   (όχι σιωπηλά CLEAR). Δοκίμασε σταματώντας το lidar publish στιγμιαία.
3. **hands-off `/navigation/cmd_vel` arbitration** — ο node στέλνει (0,0) στο
   `/navigation/cmd_vel` στο collect_route. Επιβεβαίωσε ότι ΔΕΝ μπλοκάρει το
   scan-rotation (`/cmd_vel_collection` 70) ή το FollowPath (`/cmd_vel_nav` 50).
   Αν το ρομπότ «κολλάει» στο scan/execution, εδώ κοίτα πρώτα (motion_controller
   → twist_mux routing του `/navigation/cmd_vel`).
4. **SIGINT shutdown hygiene** — benign traceback στο Ctrl-C· αν θες καθαρό
   shutdown, minor fix αργότερα.

## 5. Troubleshooting — κατηγοριοποίηση πριν αλλάξεις κώδικα (spec §Gate)

Ταξινόμησε κάθε αστοχία ΠΡΙΝ αγγίξεις κώδικα:
- **Planning** — λάθος/κενό plan, λάθος ball_results, unreachable που έπρεπε
  covered → κοίτα ScanSnapshot + CourtModel (survey quality) + feasibility config.
- **Tracking** — controller failure (tube/heading/curvature/speed) → profile
  tuning (carry-forward #1) ή path fidelity.
- **Safety** — stop/resume λάθος → SafetyMonitor thresholds / watchdog.
- **Collector** — start/stop/fault → adapter (carry-forward: sim faults).
Το frozen-route contract ΔΕΝ αλλάζει λόγω Gazebo αποτελέσματος — τα results είναι
evidence συμπεριφοράς.

## 6. Πίνακας αποτελεσμάτων (συμπλήρωσε ανά run)

| # | Scenario | Date | Result | plan_id | planning_status | Evidence / notes |
|---|----------|------|--------|---------|-----------------|------------------|
| S1 | empty/all-unreachable | | | | | |
| S2 | multi free balls | | | | | |
| S3 | close balls | | | | | |
| S4 | net/fence/corner | | | | | |
| S5 | missed capture | | | | | |
| S6 | moving obstacle | | | | | |
| S7 | collector fault | | | | | |
| S8 | follow-up limit | | | | | |

**Definition of done (Φάση 7):** όλα τα acceptance criteria του specification σε
controlled Gazebo scenarios· ένα saved `CollectionRoutePlan` συσχετίζεται
μονοσήμαντα με την `ExecutedTrajectory` και τα `BallResult` του.
