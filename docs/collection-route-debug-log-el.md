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

## #11 — (Φάση 6B) Pure PathFollower serialization + cross-language sha256/JSON parity

- **Υπόθεση:** Ο C++ collection controller δέχεται ένα
  `CollectionExecutionContext` + `nav_msgs/Path` που πρέπει να παραχθούν από το
  immutable `CollectionRoutePlan`. Δύο cross-language συμβόλαια είναι εύθραυστα:
  (α) το `path_sha256` (ο controller ξανα-υπολογίζει sha256 πάνω στο received
  path με `collection_path_sha256_v1` — πρέπει byte-for-byte ίδιο), (β) το
  `configuration_snapshot_json` (ο C++ κάνει `nlohmann::json::parse(s).dump()`
  και ΑΠΟΡΡΙΠΤΕΙ αν `!= s`). Χρειάζεται pure serializer + απόδειξη ότι ο
  ΠΡΑΓΜΑΤΙΚΟΣ C++ το δέχεται. Μόνο pure modules + parity harness· καμία live
  action/service wiring ούτε αλλαγή controller_node (6C).
- **Αλλαγή:** Τρία pure modules (χωρίς ROS import):
  (1) `collection_path_canonicalization.py` — αναπαράγει ΑΚΡΙΒΩΣ το v1 wire
  format (BE u32 frame len + UTF-8 + BE u32 pose count + 7× BE float64
  x,y,z,qx,qy,qz,qw ανά pose) → lowercase-hex sha256· non-finite float → typed
  `CanonicalizationError`. (2) `collection_execution_context_builder.py` —
  `build_execution_context(plan, *, controller_tuning, context_schema_version,
  context_activation_timeout_s)` → immutable `CollectionExecutionContextValues`
  με ΟΛΑ τα field-values του msg (segments/type-codes 0/1/2, profiles 14-πεδία
  1-1, crossings, terminal_progress=total_length, terminal_pose yaw→quat) +
  `build_follow_path_poses` (ενώνει segment paths, αφαιρεί exact-duplicate join
  poses ώστε κάθε 2D step > 0) + `canonical_configuration_snapshot_json`
  (`json.dumps(sort_keys=True, separators=(",",":"), ensure_ascii=False,
  allow_nan=False)`). `ControllerTuning` (5 πεδία) validated θετικό όπως ο C++
  `valid_tuning`· είναι runtime input, ΔΕΝ ανήκει στο
  `CollectionRouteConfiguration`. (3) Parity: `scripts/emit_collection_parity_
  fixture.py` (pure, χτίζει real plan_collection_route plan μέσω 6A CourtModel
  builder + serializer → fixture JSON), νέο gtest
  `test/test_collection_execution_context_parity.cpp` (φορτώνει το ΠΡΑΓΜΑΤΙΚΟ
  plugin μέσω pluginlib, καλεί το πραγματικό Load service + `setPlan`),
  `scripts/run_collection_parity.sh` orchestration μέσα στο container. Ο parity
  fixture χρησιμοποιεί ΕΥΘΥΓΡΑΜΜΗ route (net wall @x=8, robot (0,0,0)→ball
  (3,0)) ώστε polyline length == total_length ΑΚΡΙΒΩΣ (curved connectors έχουν
  chord-vs-arc σφάλμα > tolerance → θα έσπαγε το `make_tracking_plan`).
- **Αποτέλεσμα:** Pure gate (`test_collection_path_canonicalization` +
  `test_collection_execution_context_builder` + `..._planner_composition`) 28
  passed. Container parity (`docker run … bash scripts/run_collection_parity.sh`)
  build 2 pkgs 29.6s → 2/2 gtests PASSED: (i) Python sha256 ==
  `collection_path_sha256_v1` του C++ για το ίδιο path, (ii) Load **ACCEPTED**
  (άρα το canonical JSON επέζησε `nlohmann parse→dump`, segments/tuning/terminal
  πέρασαν `valid_load_context`), (iii) `setPlan` δεκτό (path_sha256 match +
  `make_tracking_plan`). Το nlohmann canonical JSON ταίριαξε με την πρώτη — τα
  numbers του `default_configuration` επιβιώνουν parse→dump. Καμία αλλαγή σε C++
  implementation (μόνο νέο test target στο CMakeLists) ή controller_node.
- **Status:** ΟΚ.

## #12 — (Φάση 6B.1) Densify Dubins connector arc poses + chord-based terminal progress

- **Υπόθεση:** Το parity της 6B περνούσε ΜΟΝΟ με ευθύγραμμη route. Το
  `_materialize_path` (collection_route_connector_graph.py) αποθήκευε 2 poses ανά
  arc primitive (start,end), οπότε η flattened chord polyline ήταν πολύ κοντύτερη
  από το ARC-based `length_m` (μετρημένο LSL R=0.8: connector arc 2.376 vs single
  chord· γενικά err μέτρα). Ο C++ `make_tracking_plan` αθροίζει chord polyline και
  απαιτεί ≈ `terminal_progress_s` εντός `terminal_progress_tolerance_m`, οπότε
  ΚΑΘΕ curved route απορριπτόταν. Στόχος: densify ΜΟΝΟ το pose sampling, χωρίς
  αλλαγή cost/scoring/length.
- **Αλλαγή:** (1) `_materialize_path`: κάθε arc primitive υποδιαιρείται σε
  `max(1, ceil(arc_angle/_ARC_CHORD_ANGLE_RAD))` sub-arcs (ΙΔΙΟ granularity 15°
  με το `_path_is_collision_free`), advance ανά sub-arc, append κάθε ενδιάμεση
  pose. Straight (S) μένει 2 poses. **`primitives`, `length_m`, `arc_angle_rad`,
  `total_turn_rad` ΜΕΝΟΥΝ arc-based** — μόνο το `poses` tuple πυκνώνει (chord-sum
  τώρα εντός ~0.04% του arc length ανά connector· επαληθεύτηκε per-mode
  LSL/RSR/LSR/RSL). `_self_intersects` δεν βγάζει false positive (πιο πυκνά chords
  ακολουθούν στενότερα ένα simple CSC). (2) **Δύο επιπλέον fixes στον 6B serializer
  που αποκάλυψε το container parity με curved route** (ο C++ tracking-core
  constructor, ΟΧΙ μόνο το make_tracking_plan): (α) `build_follow_path_poses` —
  τα join poses μεταξύ segments διαφέρουν ~2e-15 (densified connector endpoint vs
  pass entry_pose), οπότε το exact-equality dedup τα άφηνε → step 2e-15 έσπαγε το
  strict-increasing progress του core· άλλαξε σε epsilon dedup
  (`_JOIN_DEDUP_EPSILON_M=1e-9`). (β) `terminal_progress_s` = **flattened chord
  polyline length** (η πρόοδος που ΜΕΤΡΑΕΙ/φτάνει ο controller), ΟΧΙ το arc-based
  `plan.total_length_m`: ο core κάνει HARD `terminal_progress_s <=
  path.back().progress_s` (chord-sum, χωρίς tolerance)· αφού arc > chord ΠΑΝΤΑ για
  curved, το arc-based terminal ΔΕΝ περνάει ποτέ. Τα segment progress spans μένουν
  arc-based (το valid_load_context τα δέχεται εντός tolerance). Καμία αλλαγή σε C++
  /controller_node/scoring/FSM.
- **Αποτέλεσμα:** Pure gate (connector_graph + global_solver + composition +
  execution_context_builder) 43 passed (νέα: 4 param chord-sum≈arc ανά CSC mode +
  endpoint/turn invariance + curved terminal-progress). Container parity ΤΩΡΑ με
  **CURVED** route (robot (0,0,0)→ball (0,3), start→entry χρειάζεται ~90° Dubins·
  sparse err 0.12m FAIL vs densified 0.004m PASS εναντίον tol 0.05): 2/2 gtests
  PASSED — sha256 match + Load ACCEPTED + `setPlan` δεκτό (make_tracking_plan
  length/terminal + tracking-core constructor). Ο fixture (0,3) επιλέχθηκε επειδή
  το pre-6B.1 sparse θα αποτύγχανε (0.12>0.05)· η densification το ΞΕΜΠΛΟΚΑΡΕΙ.
- **Status:** ΟΚ.

## #13 — (Φάση 6C.1) ROS adapters για sensor/actuator + scan executor ports

- **Υπόθεση:** Ο pure `CollectionRouteExecutor` (collection_route_executor.py)
  δέχεται injected ports (ScanPoseNavigator/ScanSession/Collector/SafetyMonitor/
  TelemetrySink/MonotonicClock) που επιστρέφουν typed results. Λείπουν οι ROS
  υλοποιήσεις. Πρώτο μισό της σύνθεσης — μόνο sensor/actuator adapters + scan
  driver, ΚΑΜΙΑ αλλαγή controller_node, ΚΑΝΕΝΑ FollowPath/C++ context (6C.2),
  ΚΑΜΙΑ διαγραφή legacy (6D).
- **Αλλαγή:** Νέο `collection_executor_ports.py` — **κανένα rclpy import**· κάθε
  ROS touch-point είναι injected duck-typed handle (node, publisher callable,
  "latest message" provider) που θα δώσει το 6C.2, ώστε ολόκληρο το module +
  η decision logic να είναι offline-testable. Ports: (1) `RosMonotonicClock`
  (node.get_clock().now().nanoseconds*1e-9). (2) `telemetry_event_to_dict`
  (pure) + `CallbackTelemetrySink` (emit→dict→callback). (3)
  `ScanPoseNavigatorAdapter` wrap Nav2LaneNavigator· pure `navigator_result_for_
  state(str)` map (idle/pending/active→RUNNING, reached→SUCCEEDED, failed→
  FAILED, unavailable→UNAVAILABLE) — δέχεται το plain state string, ΔΕΝ κάνει
  import το ROS-bound LaneNavState enum. (4) `GazeboCollectorAdapter` wrap
  CollectorInterface· MVP: start_result→READY άμεσα, active_fault→None,
  stop_result→STOPPED, force_disable→stop — **κανένα ψεύτικο fault** (το real
  hardware θα wire-άρει jam/full/health μελλοντικά, τεκμηριωμένο). (5)
  SafetyMonitor: pure `forward_sector_blocked(...)` (valid return εντός forward
  sector < stop_distance) + pure `ForwardSectorSafetyLogic` (blocked-duration
  timeout FSM: CLEAR/BLOCKED/TIMEOUT, thresholds required χωρίς defaults) + thin
  `LidarSafetyMonitor` (/scan provider callback· missing scan→CLEAR, documented
  future stale-watchdog). (6) ScanSession: pure `ScanRotationFsm` (360° discrete
  step targets start+k*step_angle, observe(yaw)→step_id όταν εντός tolerance,
  is_complete μετά από step_count captures — testable με fake yaw feed,
  ξεχωριστό από cmd_vel) + thin `ScanSessionDriver` (yaw/frame providers +
  cmd_vel callable + clock· ανά step forward_frame στο CollectionSnapshotRuntime
  Session, στο τέλος finalize→SNAPSHOT_READY(snapshot) ή FAILED(SCAN_FAILED)·
  wall-clock scan_timeout guard fail-loud). Όλα τα required thresholds/config
  validated (ExecutorPortError, κανένα default).
- **Αποτέλεσμα:** Νέο `tests/test_collection_executor_ports.py` (fake node/clock/
  navigator/collector/laserscan/frames) — typed result mapping ανά port·
  lidar return εντός sector→BLOCKED / εκτός→CLEAR / παρατεταμένο→TIMEOUT /
  invalid(inf/out-of-range)→ignored· scan rotation FSM ολοκληρώνει 360° και
  παράγει SNAPSHOT_READY· finalize-fail & rotation-stall→FAILED. Gate
  (`test_collection_executor_ports` + `..._route_executor` + `..._snapshot_
  runtime_adapter`) 42 passed. Καμία αλλαγή controller_node/collect_route_mission
  /C++/6B serializers.
- **Status:** ΟΚ.

## #14 — (Φάση 6C.2) Live PathFollower port + σύνθεση CollectionRouteExecutor

- **Υπόθεση:** Λείπει ο 8ος port (PathFollower) που οδηγεί τον ΠΡΑΓΜΑΤΙΚΟ C++
  `CollectionFollowPath` controller, + η σύνθεση που συναρμολογεί και τους 8
  ports σε έναν `CollectionRouteExecutor`. Δεύτερο μισό της σύνθεσης 6C. ΚΑΜΙΑ
  αλλαγή controller_node/legacy/C++ (6D).
- **Αλλαγή:** (1) Νέο `collection_path_follower_port.py` — **κανένα rclpy import**·
  `LiveCollectionPathFollower` υλοποιεί το PathFollower Protocol με injected
  duck-typed handles (load/follow_path/hold/finalize senders + load_outcome/
  goal_status/state providers + clock). Handshake non-blocking polling (σαν
  Nav2LaneNavigator): start(plan)→build 6B context/path/sha256 (ΧΡΗΣΙΜΟΠΟΙΕΙ τους
  6B serializers) + Load async· result()→ σε Load ACCEPTED στέλνει FollowPath
  (controller_id="CollectionFollowPath") εντός activation timeout, μετά map
  CollectionControllerState+goal status → PathFollowerResult (goal SUCCEEDED→
  completed· lifecycle FAILED ή failure_reason≠NONE→failed(reason)· EXECUTING/
  SAFETY_PAUSED→running με progress_s, tube_ok=lateral≤trajectory_tube_radius,
  remaining_run_in από reuse του pure CollectionPathFollower.remaining_run_in_m·
  Load reject/activation timeout→failed). pure `failure_reason_for_code` map
  (SAFETY_RESUME_INVALID→ίδιο, υπόλοιπα tube/curvature/speed/reverse/rotate/
  non-monotonic→PATH_FAILED). pause/resume→SetCollectionSafetyHold(hold true/
  false). **Απόφαση Finalize:** ο follower στέλνει FinalizeCollectionExecution
  Context(SUCCEEDED) ΑΥΤΟΜΑΤΑ ΜΙΑ φορά στο result()==completed transition (goal
  action terminal success = η μόνη στιγμή που ο C++ context είναι terminal_ready·
  αλλιώς TERMINAL_NOT_REACHED)· ο executor δεν χρειάζεται να ξέρει για Finalize.
  (2) Νέο `collection_executor_assembly.py` — `build_collection_route_executor
  (node, config, handles)` συναρμολογεί ΚΑΙ τους 8 ports (6C.1 adapters + αυτόν
  τον follower + PurePlanner=plan_collection_route wrapper με CourtModel από 6A
  builder + ScanSessionDriver)· `read_controller_tuning(node)` διαβάζει τα 5
  tuning params (duck-typed node.get_parameter, κανένα rclpy import), validated
  θετικά (ControllerTuning == C++ valid_tuning). (3) nav2_params.yaml: νέο
  `collection_route_executor` node block με τα 5 controller_tuning params
  (controller-runtime config, ΟΧΙ μέρος του frozen plan· ο 6D node τα διαβάζει).
- **Αποτέλεσμα:** Νέο `tests/test_collection_path_follower_port.py` (fake service/
  action/state handles) — κάθε mapping: Load accept→FollowPath sent· SUCCEEDED→
  completed+Finalize ΜΙΑ φορά· failure_reason X→failed reason· EXECUTING→running
  με σωστό remaining_run_in/tube_ok· tube violation· SAFETY_PAUSED→running·
  Load reject→failed· activation timeout→failed· Finalize ΜΟΝΟ σε terminal·
  pause/resume→Hold calls· read_controller_tuning· **assembly full fake cycle
  idle→…→COMPLETED**. Pure gate (`..._path_follower_port` + `..._executor_ports`
  + `..._route_executor`) 52 passed. **Container smoke** (`docker run … bash
  scripts/run_collection_follower_smoke.sh`): launch_test σηκώνει ΠΡΑΓΜΑΤΙΚΟ nav2
  controller_server + CollectionFollowPath plugin, ο ΠΥΘΩΝΙΚΟΣ
  LiveCollectionPathFollower οδηγεί ΠΡΑΓΜΑΤΙΚΟ curved plan (robot(0,0,0)→ball
  (0,3)) end-to-end: Load ACCEPTED → FollowPath (controller_id+sha match) →
  "Reached the goal!" → Finalize ACCEPTED. 1 test PASSED (robot pre-parked στο
  terminal pose, xy goal tolerance 0.10, ίδιο pattern με το 6B isolated launch
  test). Καμία αλλαγή controller_node/collect_route_mission/C++.
- **Status:** ΟΚ.

## #15 — (Φάση 6D.1) Build-config για το collection controller + fail-safe scan watchdog

- **Υπόθεση:** Το `tennis_robot_collection_controller` (C++ Nav2 plugin) χτιζόταν
  ΜΟΝΟ σε isolated overlays στα smokes — ΟΧΙ στο image bake ούτε στον dev overlay,
  άρα το πραγματικό σιμ ΔΕΝ φόρτωνε το plugin (το controller_id
  "CollectionFollowPath" δεν θα έβρισκε class). Prerequisite infra πριν το atomic
  node cutover (6D.2). Παράλληλα carry-forward από 6C.1: το missing/None /scan
  → CLEAR (όχι fail-safe).
- **Αλλαγή:** **Μέρος 1 (build config):** (α) `Dockerfile.gazebo` — νέο
  `COPY tennis_robot_collection_controller` + το bake `colcon build` άλλαξε σε
  `--packages-select tennis_robot_collection_controller tennis_robot` (msgs ήδη
  χτισμένο νωρίτερα + sourced, ο controller εξαρτάται από msgs — colcon λύνει τη
  σειρά). (β) `scripts/docker_dev_entry.sh` — dev overlay select έγινε
  `tennis_robot_msgs tennis_robot_collection_controller tennis_robot`. (γ)
  `package.xml` exec_depend υπήρχε ήδη. Καμία αλλαγή C++ source ή nav2_params.
  **Μέρος 2 (fail-safe scan watchdog):** στο `collection_executor_ports.py` νέο
  immutable `ScanSample(stamp_s, ranges, angle_min/increment, range_min/max)`· το
  `ForwardSectorSafetyLogic` πήρε required `max_scan_age_s` (no default) και το
  `_is_blocked` πλέον: **scan is None Ή age > max_scan_age_s → BLOCKED (fail-safe)**,
  αλλιώς forward_sector_blocked· το ίδιο blocked-duration timeout FSM κλιμακώνει
  σε TIMEOUT. `LidarSafetyMonitor.result()` εξάγει το stamp από το LaserScan
  header (`_stamp_seconds`) και χτίζει `ScanSample` (ή None). `CollectionExecutor
  Config.safety_max_scan_age_s` + wiring στο assembly (explicit, no default).
- **Αποτέλεσμα:** Pure gate (`test_collection_executor_ports` +
  `test_collection_path_follower_port`) 45 passed — νέα watchdog tests: missing→
  BLOCKED, stale→BLOCKED, fresh clear→CLEAR, fresh blocked→BLOCKED, sustained
  stale→TIMEOUT, monitor stamp-freshness· assembly smoke ενημερώθηκε με fresh
  clear scan (missing πλέον BLOCKED). **Container verify**: clean build 3 pkgs
  (msgs 6.4s + controller 24.3s + tennis_robot 0.55s = 31.3s) — το plugin πλέον
  χτίζεται στο packages-select. C++ tests (εξαιρώντας το harness-only parity
  gtest που θέλει COLLECTION_PARITY_FIXTURE env, τρέχει μέσω
  run_collection_parity.sh): plugin 1 + runtime 5 + path_canonicalization 7 +
  tracking_core 7 + isolated launch 5 = **25/0 πράσινα** — καμία regression από
  το 6D.1. debug log #15. Καμία αλλαγή controller_node/legacy/C++ source.
- **Status:** ΟΚ.
