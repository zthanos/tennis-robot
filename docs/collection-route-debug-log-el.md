# Collect-route debug log (ενεργό)

> Το προηγούμενο log βρίσκεται στο `docs/archive/`. Εδώ καταγράφεται κάθε
> αλλαγή του collect_route rewrite με υπόθεση / αποτέλεσμα / status.

## #1 — Φάση 2R fixes από review 2026-07-20: σπασμένο runtime session + single source coverage

- **Υπόθεση:** Το `CollectionSnapshotRuntimeSession` έσπασε (TypeError) επειδή ο
  `ScanSnapshotBuilder` απέκτησε required `expected_scan_step_ids` /
  `required_coverage_fraction` / `scan_timeout_s` και `finalize(now_s)` χωρίς να
  ενημερωθεί το session. Παράλληλα, τα `required_coverage_fraction` και
  `scan_timeout_s` ως ctor args ήταν δεύτερη πηγή αλήθειας δίπλα στο
  `configuration_snapshot.scan`.
- **Αλλαγή:** Ο builder δέχεται πλέον ΜΟΝΟ scan-instance δεδομένα
  (`scan_id`, `scan_timestamp_s`, `robot_pose_at_scan`,
  `expected_scan_step_ids`)· τα `required_coverage_fraction`/`scan_timeout_s`
  διαβάζονται αποκλειστικά από `configuration_snapshot.scan` μέσω read-only
  properties. Το session δέχεται `expected_scan_step_ids` και εκθέτει
  `finalize(now_s)`.
- **Αποτέλεσμα:** `tests/test_collection_scan_snapshot.py` +
  `tests/test_collection_snapshot_runtime_adapter.py` πράσινα (10/10), μαζί με
  νέο test ότι coverage/timeout έχουν μοναδική πηγή το configuration snapshot.
- **Status:** ΟΚ.

## #2 — Πλήρες configuration snapshot + calibration identity + artifact_sha256

- **Υπόθεση:** Το `configuration_snapshot` δεν περιείχε τα spec groups
  `perception_spatial_validation` και `calibration_artifact`, το artifact δεν
  είχε content hash, και το runtime bridge έκανε
  `getattr(frame, "calibration_id", "runtime")` — πάντα ψεύτικη ταυτότητα αφού
  το `BallDetectionArray` δεν είχε τέτοια fields.
- **Αλλαγές:**
  - `CollectionRouteConfiguration`: νέα required groups
    `perception_spatial_validation` (ο ίδιος τύπος
    `PerceptionSpatialValidationConfig` — καμία διπλή δήλωση) και
    `calibration_artifact` (ο ίδιος τύπος `CalibrationArtifact`), με exact-field
    to_dict/from_dict και wrapping του `CalibrationError` σε
    `DomainValidationError`. Το `localization_xy_covariance` μένει μόνο στο
    `gazebo_snapshot`.
  - `CalibrationArtifact`: νέο required πεδίο `artifact_sha256` (sha256 του
    canonical JSON χωρίς το ίδιο το πεδίο, μέσω `compute_artifact_sha256`)· το
    `from_dict` επαληθεύει το hash, νέο `to_dict` για round-trip. Τα δύο
    committed gazebo artifacts (v1/v2) ξαναγράφτηκαν με πραγματικά υπολογισμένο
    hash· `fit_conservative_artifact` και `build_gazebo_covariance_artifact.py`
    υπολογίζουν το hash κατά την παραγωγή.
  - `BallDetectionArray.msg`: νέα `calibration_id` + `configuration_id`· ο
    `perception_node` τα γεμίζει από το `SpatialCalibrationRuntime`
    (artifact_id/artifact_version, κενά όταν unhealthy). Το runtime bridge τα
    απαιτεί και απορρίπτει typed (`calibration_identity_missing`) όταν
    λείπουν/είναι κενά — κανένα implicit default.
  - Το session αντλεί πλέον το validation config από το
    `configuration_snapshot.perception_spatial_validation` (μία πηγή αλήθειας).
- **Αποτέλεσμα:** 101 non-ROS tests πράσινα, συμπεριλαμβανομένων νέων tests για
  serialization των groups, sha-tampering rejection και typed rejection της
  ελλιπούς calibration ταυτότητας. `test_perception_contract_ros.py`
  ενημερωμένο για τα νέα fields (τρέχει μόνο με rclpy).
- **Status:** ΟΚ.

## #3 — Localization budget στη fusion, cross-half filter, TF age, builder fixes

- **Υπόθεση (fusion):** Ο adapter πρόσθετε το localization covariance ΣΕ ΚΑΘΕ
  observation και ο builder έκανε information fusion σαν να ήταν ανεξάρτητα·
  με k observations το κοινό localization σφάλμα διαιρούνταν με k και η fused
  covariance έπεφτε κάτω από το conservative budget 0.01 m².
- **Αλλαγές:**
  - `PerceptionSpatialObservationAdapter`: το `AcceptedSpatialObservation`
    κρατά ΜΟΝΟ τη rotated measurement covariance (το localization config
    παραμένει required input — η απουσία του είναι typed rejection). Ο builder
    προσθέτει το `localization_xy_covariance` ΜΙΑ φορά ανά μπάλα στο
    `finalize`. Test αποδεικνύει ότι με 4 σχεδόν τέλειες observations η τελική
    covariance μένει ≥ 0.01 m² (budget + measurement/k).
  - Cross-half filter: νέο injected `CourtHalfBoundary` (net line + πλευρά
    ρομπότ από το court model, όχι hardcoded) required στον builder και στο
    session· observation στην απέναντι πλευρά ή ακριβώς πάνω στο φιλέ γίνεται
    typed rejection με detail `opposite_court_half`. Test με μπάλα ακριβώς
    πίσω από το φιλέ.
  - `max_detection_to_tf_age_s`: επιβάλλεται πλέον στο runtime bridge — αν το
    transform του tf_provider απέχει από το RGB stamp πάνω από το όριο,
    typed `perception_tf_rejected` με detail `detection_to_tf_age_exceeded`.
    Ο pure adapter κρατά το exact-timestamp check.
  - Builder fixes: (α) το scan step μετρά σε coverage ΜΟΝΟ όταν η observation
    γίνει τελικά δεκτή (μετά και το ambiguous-association gate)· (β) singular
    covariance (det≈0) γίνεται typed rejection `singular_covariance` αντί για
    ZeroDivisionError· (γ) same-step observation σε υπάρχον track μένει
    σιωπηλή απόρριψη αλλά καταγράφεται στο νέο telemetry
    `duplicate_step_observations`· (δ) docstring τεκμηριώνει ότι min
    confirmations = distinct steps και confidence = μέσος όρος.
- **Αποτέλεσμα:** Πλήρες Φάση-2R gate: 107 passed, 1 skipped (ROS contract
  test χωρίς rclpy).
- **Status:** ΟΚ — η Φάση 2R έκλεισε με 3 commits.

## #4 — (Φάση 3-5R, F6) Αφαίρεση dead UncertaintyConfiguration

- **Υπόθεση:** Το `UncertaintyConfiguration` group στο
  `CollectionRouteConfiguration` δεν διαβάζεται πουθενά στη γεωμετρία 3A/3B/3C
  ούτε στον executor/follower — οι αβεβαιότητες κωδικοποιούνται ήδη στο snapshot
  covariance (ball-position + localization, μετά τη 2R fusion) και στο
  `feasibility.tracking_lateral_error_bound_m`. Ένα ξεχωριστό group ρίσκαρε
  μελλοντικό double-counting του localization.
- **Αλλαγή:** Πλήρης αφαίρεση της `UncertaintyConfiguration` και του πεδίου
  `uncertainty` από `CollectionRouteConfiguration` (dataclass, type check,
  to_dict, from_dict). Ενημερώθηκαν `default_configuration` και το config helper
  του `test_collection_route_types`. Τα `perception_spatial_validation`,
  `calibration_artifact`, `gazebo_snapshot` δεν αγγίχτηκαν.
- **Αποτέλεσμα:** Νέο test ότι το to_dict δεν έχει `uncertainty` και ότι
  extra `uncertainty` key απορρίπτεται· 13 types/fixtures + 72 λοιπά pure tests
  πράσινα.
- **Status:** ΟΚ.

## #5 — (Φάση 3-5R, F1) Status όταν best is None από τα ball results

- **Υπόθεση:** Στον `solve_global_route`, όταν `best is None` το status κρινόταν
  από το `outgoing.get("start")` (ύπαρξη valid start edge), όχι από τα
  πραγματικά ball outcomes. Έτσι μια 3A-feasible μπάλα χωρίς usable start edge
  (π.χ. όλα τα start edges collision-rejected) έβγαινε λανθασμένα
  `EMPTY_NO_FEASIBLE_TARGETS` ενώ το BallResult της ήταν DEFERRED/ROUTE_CONFLICT
  — αντιφατικό με το spec (EMPTY_NO_FEASIBLE = run completed_no_targets, μόνο
  όταν όλες οι μπάλες είναι deterministically unreachable).
- **Αλλαγή:** Υπολογίζονται πρώτα τα ball results· `EMPTY_NO_FEASIBLE_TARGETS`
  μόνο όταν ΟΛΑ είναι `UNREACHABLE`, αλλιώς (έστω μία DEFERRED) →
  `PLANNING_TIMEOUT` (non-executable, zero geometry, χωρίς segments). Το
  `search_status` (complete/budget_exhausted) δεν αλλάζει.
- **Αποτέλεσμα:** Το προϋπάρχον `test_directed_edges_only_define_valid_route_search`
  ενημερώθηκε (πλέον PLANNING_TIMEOUT + DEFERRED/ROUTE_CONFLICT, zero geometry).
  Νέα tests: όλα τα start edges collision-rejected → PLANNING_TIMEOUT· όλες
  keepout → EMPTY_NO_FEASIBLE_TARGETS. 18 solver+composition tests πράσινα.
- **Status:** ΟΚ.

## #6 — (Φάση 3-5R, F2) Corridor collapse ≠ no_entry· διακριτά reason codes

- **Υπόθεση:** Στο `_analyze_ball`, όταν `effective_width <= 0` (corridor
  collapse από uncertainty/margins) το heading χρεωνόταν σε `entry_failed`, και
  τα `entry_failed`/`exit_failed` ήταν κοινά boolean με το τελικό reason να
  προτιμά πάντα `NO_ENTRY`. Έτσι corridor collapse ή exit-only failures με
  παρεμβαλλόμενο collapse κατέληγαν λανθασμένα `no_entry`.
- **Αλλαγή:** Τρία ξεχωριστά flags (`corridor_collapsed`, `entry_failed`,
  `exit_failed`). Corridor collapse → δικό του flag. Τελικό reason κατά
  precedence της πραγματικής αιτίας: entry_failed → `NO_ENTRY`, αλλιώς
  exit_failed → `NO_EXIT`, αλλιώς (μόνο corridor collapse) →
  `NO_CANDIDATE_FOUND` (κατά την επιλογή του χρήστη). Καμία αλλαγή στη γεωμετρία
  ή στη σειρά ελέγχων.
- **Αποτέλεσμα:** Νέο test με μεγάλη isotropic covariance → όλες οι headings
  collapse → `NO_CANDIDATE_FOUND`. Τα προϋπάρχοντα no_entry/no_exit tests
  παραμένουν πράσινα (34 planner/shared/solver/composition tests).
- **Status:** ΟΚ.

## #7 — (Φάση 3-5R, F3) shared-pass candidate_budget_exhausted σε observable telemetry

- **Υπόθεση:** Το `SharedPassGenerationResult.candidate_budget_exhausted`
  παραγόταν στο Phase 3C αλλά το `plan_collection_route` το πετούσε — δεν
  έφτανε σε κανένα observable output.
- **Απόφαση χρήστη (F3):** Δεν αλλάζουμε το immutable `CollectionRoutePlan`
  contract. Το `plan_collection_route` επιστρέφει νέο frozen wrapper
  `PlannerResult(plan, shared_pass_candidate_budget_exhausted)`, επεκτάσιμο για
  μελλοντικό planner telemetry. Το plan μένει καθαρό frozen artifact· στη
  Φάση 6 το wiring κάνει `.plan` unwrap (το `PurePlanner.plan()` protocol
  παραμένει `-> CollectionRoutePlan`).
- **Αλλαγή:** Νέο `PlannerResult` στο `collection_route_planner_v2.py`·
  `plan_collection_route -> PlannerResult`. Ενημερώθηκαν οι callers: τα
  executor test helpers (`executable_plan`/`empty_plan`) κάνουν `.plan` unwrap,
  το composition test διαβάζει `.plan` και το flag από το wrapper.
- **Αποτέλεσμα:** Νέο composition test ότι με `max_shared_pass_candidates=1` +
  3 ευθυγραμμισμένες μπάλες το flag γίνεται True και δεν υπάρχει στο plan, ενώ
  single-ball snapshot δίνει False. 27 composition/executor/planner_v2 tests
  πράσινα.
- **Status:** ΟΚ.

## #8 — (Φάση 3-5R, F4) Speed-only verdict· lateral/heading σε ξεχωριστό telemetry

- **Υπόθεση:** Ο pure follower έβαζε lateral/heading exceedance στο
  `hard_violation_reason` (`LATERAL_ERROR_EXCEEDED`/`HEADING_ERROR_EXCEEDED`),
  κάνοντας `hard_compliant=False`. Το C++ `collection_tracking_core.cpp` όμως
  κρατά το crossing `ProfileComplianceVerdict.hard_violation_reason` speed-only
  (kSpeedBelowMin/kSpeedAboveMax) και βγάζει lateral/heading ως ξεχωριστά
  tracking failures — το Python verdict απέκλινε από το spec/C++.
- **Αλλαγή:** Το `ProfileViolationReason` κρατά μόνο `SPEED_BELOW_MIN`/
  `SPEED_ABOVE_MAX`. Το `_measure_crossing` υπολογίζει speed-only violation και
  ξεχωριστό `tracking_compliant` (lateral ≤ max_lateral ΚΑΙ heading ≤
  max_heading). Νέο πεδίο `CrossingMeasurement.tracking_compliant` και νέο
  `FollowerTelemetryCode.CROSSING_TRACKING_VIOLATION`, που εκπέμπεται ανεξάρτητα
  από το speed verdict και ΔΕΝ αλλάζει το `hard_compliant`.
- **Αποτέλεσμα:** Το `test_crossing_lateral_heading_metrics` ενημερώθηκε
  (verdict compliant + CROSSING_TRACKING_VIOLATION, όχι PROFILE_VIOLATION). Νέο
  test ότι το enum είναι speed-only (ίδιες τιμές με το C++ verdict) και ότι
  slow+off-tube crossing δίνει speed hard_violation ΚΑΙ ξεχωριστό tracking
  violation. 5 follower tests + 91 gate tests πράσινα.
- **Status:** ΟΚ.

## #9 — (Φάση 3-5R, F5) Follow-up μόνο μετά από καθαρό ROUTE_COMPLETED

- **Υπόθεση:** Το `_can_follow_up` έλεγχε μόνο `policy.enabled` και τον run
  counter, όχι το `route_outcome`. Έτσι, με follow-up enabled, ένα active-route
  abort (safety/tracking/collector) ξεκινούσε νέο scan cycle — auto-retry μετά
  από αποτυχία, εκπληκτικό. (Απόφαση χρήστη: follow-up = «μάζεψε κι άλλες
  μπάλες σε επιπλέον περάσματα», όχι retry.)
- **Αλλαγή:** Το `_can_follow_up` απαιτεί επιπλέον
  `route_outcome is ExecutorState.ROUTE_COMPLETED`. Καμία αλλαγή στα FSM
  transitions.
- **Αποτέλεσμα:** Νέο test: ABORTED_SAFETY + follow-up enabled → COMPLETED,
  run_count 1, navigator.starts 1 (κανένα νέο cycle)· καθαρό ROUTE_COMPLETED +
  follow-up enabled → run_count 2, navigator.starts 2. 10 executor tests
  πράσινα.
- **Status:** ΟΚ.

## #10 — (Φάση 6A) Pure CourtModel builder από court_boundary.json v2

- **Υπόθεση:** Ο planner (`collection_route_planner_v2`) χρειάζεται explicit
  immutable `CourtModel` (closed `navigable_polygon` + `obstacles[{id,kind,
  polygon}]`), αλλά το survey γράφει `court_boundary.json`
  (schema `court_knowledge_model/v2`, frame `map`). Λείπει καθαρό, offline
  μεταφραστικό στρώμα dict→CourtModel — πρώτο slice της Φάσης 6 (μόνο pure
  module + tests, καμία αλλαγή σε ROS wiring / controller / 3A geometry).
- **Αλλαγή:** Νέο pure module
  `ros2_ws/.../collection_court_model_builder.py` με `build_court_model(dict)
  -> CourtModel` (χωρίς ROS/file I/O import). Κρίσιμες σχεδιαστικές αποφάσεις:
  (α) `navigable_polygon` = τα 4 fence corners — ο planner ελέγχει το inflated
  exterior ως keepout, άρα ο φράχτης-ως-όριο καλύπτεται· ΔΕΝ μπαίνει ως filled
  polygon (θα έκανε κάθε εσωτερική μπάλα point-in-polygon → KEEPOUT). (β) Ο
  φράχτης ΕΠΙΣΗΣ ως ΤΕΣΣΕΡΑ λεπτά `fence`-kind wall obstacles (ένα ανά ακμή,
  inner long edge πάνω στην ακμή, body offset `FENCE_WALL_THICKNESS_M=0.05`
  ΕΞΩ), ώστε ο planner να παράγει fence-tangent headings
  (`_active_tangent_headings` παίρνει tangent μόνο από net/fence kinds) χωρίς
  false keepout σε εσωτερικές μπάλες. (γ) Το φιλέ ως ένα λεπτό `net`-kind wall
  post-to-post (`NET_WALL_THICKNESS_M=0.04`, centred)· τα posts είναι τα άκρα
  του net wall (όχι ξεχωριστά `post` obstacles) για αποφυγή spurious
  perpendicular tangents. (δ) Εσωτερικά obstacles → axis-aligned ορθογώνια από
  `center`+`size_m`· `class`→kind mapping `perimeter_fixture`→`bench`,
  unknown→`other` (κανένα από τα δύο δεν δίνει tangent). Απαιτεί
  `schema==court_knowledge_model/v2`, `frame==map`, `status==OK`,
  `completed==True`, παρουσία `fence.corners`/`net`· κάθε missing/invalid →
  typed `CourtModelBuildError`, όχι σιωπηλό default.
- **Αποτέλεσμα:** Νέο `tests/test_collection_court_model_builder.py` (15
  cases) — κάθε geometry test περνά το CourtModel σε πραγματικό
  `analyze_snapshot`: εσωτερική μπάλα ΔΕΝ βγαίνει KEEPOUT· μπάλα κοντά σε ακμή
  φράχτη → μόνο παράλληλα (fence-tangent) headings· μπάλα κοντά στο φιλέ →
  net-tangent· μπάλα εκτός φράχτη → KEEPOUT (exterior)· μπάλα πάνω σε εσωτερικό
  obstacle → KEEPOUT· 6 invalid-schema typed rejections· determinism. Gate
  (`test_collection_court_model_builder` + `..._planner_v2` +
  `..._planner_composition`) 33 passed· ο builder διαβάζει και το πραγματικό
  `runtime/court_boundary.json` καθαρά (4 fence walls + net + 4 fixtures).
  Καμία αλλαγή στο `collection_route_planner_v2.py`.
- **Status:** ΟΚ.
