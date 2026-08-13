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

## #16 — (Φάση 6D.2) Runtime `CollectionRouteConfiguration` builder (pure)

- **Υπόθεση:** Το immutable configuration των 12 groups υπήρχε μόνο σε test
  fixtures/smoke scripts. Το runtime cutover 6D.3 χρειάζεται έναν ROS-free
  builder που δέχεται ήδη parsed mapping και δεν εφευρίσκει καμία τιμή.
- **Αλλαγή:** Νέο `collection_route_config_builder.py` με
  `build_collection_route_configuration(source, *, calibration_artifact_path)`.
  Το source schema είναι το serialized `CollectionRouteConfiguration` χωρίς το
  `calibration_artifact`: κάθε root/group/nested πεδίο είναι υποχρεωτικό και
  exact, ενώ missing/extra/invalid τιμές γίνονται typed
  `CollectionRouteConfigurationBuildError` με group/field context. Όλα τα group
  values περνούν από τα υπάρχοντα `from_dict`/dataclass `__post_init__`. Το
  versioned artifact φορτώνεται αποκλειστικά από το ρητό path μέσω
  `load_artifact`, χωρίς ROS/env/path discovery ή fallback. Νέο packaged
  `config/collection_route.yaml` με τις Gazebo MVP fixture τιμές, explicit
  provisional localization covariance `[[0.01, 0], [0, 0.01]]`, association
  thresholds και τα πέντε perception spatial validation thresholds. Δεν υπάρχει
  `UncertaintyConfiguration`.
- **Αποτέλεσμα:** Νέο `tests/test_collection_route_config_builder.py`: structural
  equality με baseline + πραγματικό gazebo-v2 artifact, αυτούσιο configuration
  identity σε `ScanSnapshotBuilder` και `plan_collection_route`, typed failures
  για missing group/field, invalid group value και invalid artifact path, και
  `to_dict`/`from_dict` round-trip. Καμία αλλαγή σε `controller_node` και καμία
  διαγραφή legacy.
- **Status:** ΟΚ.

## #17 — (Φάση 6D.3) Node-side ROS handles + dormant executor assembly

- **Υπόθεση:** Το 6C assembly είχε όλα τα pure ports, αλλά τα πραγματικά ROS
  service/action/message handles υπήρχαν μόνο hand-built μέσα στο 6C.2 smoke.
  Το παρόν βήμα κατασκευάζει το πλήρες node-side transport χωρίς να το συνδέει
  στο `controller_node` control loop ή στο `collect_route` dispatch (cut-over
  παραμένει αποκλειστικά στο 6D.4).
- **Αλλαγή:** Νέο `collection_executor_node_factory.py`: explicit node cache για
  latest `/scan`, yaw και canonical `BallDetectionArray`; TF wrapper που ζητά
  camera→map transform ακριβώς στο RGB timestamp και επιστρέφει
  `TimestampedCameraToMapTransform`; service clients για Load/Hold/Finalize,
  FollowPath action client, controller-state subscription και angular-only
  `/navigation/cmd_vel` publisher. Ο `load_sender` είναι το μοναδικό ROS
  serialization boundary και αντιγράφει 1:1 όλα τα context/segment/profile/
  crossing/tuning/terminal/configuration fields (segment codes 0/1/2). Το
  immutable snapshot χτίζεται από explicit `collection_route.yaml` + calibration
  artifact, ενώ tuning και operational scan/safety/context values διαβάζονται
  ως required ROS params από `nav2_params.yaml` χωρίς code defaults. Τα explicit
  helpers παράγουν scan pose στο κέντρο της service line της τρέχουσας πλευράς
  και `CourtHalfBoundary` από τα surveyed court axes/net posts· invalid ή
  αμφίσημη γεωμετρία αποτυγχάνει άμεσα. Προστέθηκε δηλωμένη PyYAML dependency.
- **Αποτέλεσμα:** `tests/test_collection_executor_node_factory.py` με fake node/
  TF/clients/action/messages αποδεικνύει την κατασκευή όλων των assembly handles,
  live cache providers, cmd_vel shape, endpoints, fail-loud missing param και
  field-by-field ROS context serialization από πραγματικό 6B value-object.
  Offline gate: **49 passed in 0.49s** (`node_factory` + `path_follower_port` +
  `executor_ports`). Το container smoke πλέον αφαιρεί τον hand-built transport,
  κατασκευάζει handles και executor μέσω του factory και οδηγεί τον πραγματικό
  C++ `CollectionFollowPath`: overlay **3 packages finished [0.97s]**, controller
  `Reached the goal!`, launch test **Ran 1 test — OK**, Finalize ACCEPTED.
  Καμία αλλαγή σε `controller_node.py`, legacy mission, mode dispatch/control
  loop ή robot status.
- **Status:** ΟΚ.

## #18 — (Φάση 6D.4) Atomic flip του `collect_route` στον νέο executor

- **Υπόθεση:** Το live cut-over πρέπει να έχει έναν μόνο motion/collector owner:
  ο executor δημιουργείται μία φορά στην είσοδο του mode, κάνει `start()` και
  `tick()` στο υπάρχον 32 ms loop, ενώ το `ControllerNode` επιστρέφει hands-off
  command. Δεν υπάρχει legacy mission/config/court fallback.
- **Αλλαγή:** Το `collect_route` dispatch δεν καλεί πλέον
  `CollectRouteMission.start/update`, legacy confirmation, `mark_nearest` ή
  route credit. Στην είσοδο κατασκευάζει `CollectionExecutorNodeFactory` με το
  explicit `runtime/court_boundary.json`, το packaged `collection_route.yaml`,
  required calibration artifact και required tuning/runtime params που
  φορτώνονται από `nav2_params.yaml`, και αποτυγχάνει άμεσα αν λείπει κάτι. Τα
  raw canonical detections, `/scan`, pose/TF, Nav2 και publisher-backed
  collector interface συνδέονται στα factory handles. Το per-tick command έχει
  base `(0,0)` και collector idle, ενώ `_apply_command` δεν δημοσιεύει καθόλου
  `/collector/cmd` στο `collect_route`, ώστε μόνο το executor port να κάνει
  start/stop. Έγκυρο empty perception heartbeat μετρά πλέον ως scan coverage,
  ώστε empty scan να παράγει `EMPTY_NO_BALLS` και `completed_no_targets`.
- **Arbitration:** Η scan περιστροφή δημοσιεύει απευθείας στο
  `/cmd_vel_collection` (twist_mux priority 70). Το πραγματικό FollowPath
  controller δημοσιεύει στο `/cmd_vel_nav` (priority 50). Μετά το τελευταίο
  scan zero, το collection input λήγει με το configured mux timeout 0.5 s και
  το Nav2 αναλαμβάνει· ο hands-off zero του node μένει στο upstream
  `/navigation/cmd_vel` και δεν μπορεί να overwrite το scan publisher.
- **Status/console contract:** Τα `collect_route` και `collection_run` είναι
  πλέον executor serialization: `state`, `route_outcome`, `plan_id`,
  `planning_status`, `ball_results`, πλήρη `segments`, flattened planned
  `crossings`, bounded executed crossing telemetry από το controller-state
  stream και executor events. Το `collection_run.status` διατηρείται για το
  υπάρχον JS και ισούται με το executor state όσο τρέχει. Το `map.route`
  διατηρεί το υπάρχον `{x_m,y_m,yaw_rad}` point shape, τώρα από τα segment paths.
  Τα legacy aggregate counters (`planned`, `missing`, `route_collected` κ.λπ.)
  δεν παράγονται πλέον· τυχόν JS προσαρμογή τους παραμένει για 6D.5.
- **Έλεγχος:** Node-wiring fake tests καλύπτουν build/start-on-entry, per-tick
  tick, `completed_no_targets`, executor status και collector hands-off. Το
  container startup smoke σηκώνει πραγματικό `controller_node`, πραγματικό
  `controller_server`/CollectionFollowPath και minimal NavigateToPose/sensor/TF
  dependencies και απαιτεί δημοσιευμένο `completed_no_targets` χωρίς crash.
- **Status:** ΟΚ. Requested gate: **7 passed in 0.43s**. Ευρύτερο executor/
  ports/snapshot regression: **74 passed in 0.99s**. Container startup smoke:
  overlay **3 packages finished [1.17s]**, πραγματικός `controller_server`
  activated, `controller_node` έγραψε `collect_route executor terminal:
  completed_no_targets`, launch test **Ran 1 test — OK**.

## #19 — (Φάση 6D.5) Οριστική αφαίρεση legacy route + parity hygiene

- **Υπόθεση:** Μετά το 6D.4 οι `_collect_route_observation` και
  `_collect_route_target_observation` δεν έχουν call sites, επειδή ο executor
  καταναλώνει τα canonical detections απευθείας. Τα
  `_assign_sim_ball_route_owners` και `_pending_sim_capture_ball_id` είναι επίσης
  orphaned και εξαρτώνται αποκλειστικά από το legacy
  `capture_ball_id`· δεν μοιράζονται με `collect`, `collect_one` ή
  `collect_pattern`.
- **Αλλαγή:** Αφαιρέθηκαν οι τέσσερις orphaned helpers, τα
  route-owner/grace fields και τα unused `SIM_CAPTURE_PENDING_GRACE_S` /
  `COLLECT_ROUTE_FRONT_BLOCK_M`. Το κοινό sim retention, beam/truth reconciliation
  και `mark_nearest_collected` των άλλων modes παραμένουν. Το
  `DEFAULT_BOUNDARY_FILE` παραμένει επειδή το χρειάζεται ο νέος factory,
  και `_last_collect_route_summary` επειδή διατηρεί το terminal/stopped
  executor status μετά την έξοδο από το mode. Διαγράφηκαν το
  `collect_route_mission.py`, ο παλαιός `collection_route_planner.py` και τα
  δύο legacy test modules. Νέο AST-based static test αποτυγχάνει αν
  source/script/test εισάγει ξανά τα legacy modules ή `CollectRouteMission`.
  Το parity gtest κάνει `GTEST_SKIP()` στην κορυφή του `SetUp()` όταν
  λείπει το `COLLECTION_PARITY_FIXTURE`, με null-safe `TearDown()`.
- **Αποτέλεσμα:** Το πλήρες pytest gate στο ROS 2 Humble container
  πέρασε **292 tests** (ignore μόνο το γνωστό `test_console_app.py`),
  συμπεριλαμβανομένου του static import guard. Το clean isolated
  colcon build των msgs/controller/tennis_robot ολοκλήρωσε **3
  packages**. Το bare controller colcon test πέρασε **6/6 CTest
  targets**, με τα δύο parity cases ρητά **SKIPPED** και return code 0.
  Με fixture, το `run_collection_parity.sh` πέρασε **2/2**. Το
  node-startup smoke έχτισε καθαρά τα τρία packages και παρατήρησε
  `collect_route executor terminal: completed_no_targets` (**1/1 OK**).
- **Status:** ΟΚ.

## #20 — (Φάση 7, sim run 1) `aborted_scan`: collision στο `/cmd_vel_collection`
- **Υπόθεση (αρχική):** Το πρώτο Gazebo run του νέου `collect_route` (S1, άδεια
  πλευρά) έφτασε στο scan pose (Nav2 «Reached the goal!») αλλά κατέληξε
  `aborted_scan` μετά ~20s. Πρώτη υποψία: το ρομπότ δεν περιστράφηκε.
- **Παρατήρηση χρήστη:** Το ρομπότ **έκανε** περιστροφή (αργή/κομπιαστή).
- **Root cause (επιβεβαιωμένο στον κώδικα):** Δύο publishers στο **ίδιο**
  `/cmd_vel_collection`: (1) το scan rotation του executor
  (`collection_executor_node_factory` → `publish_scan_twist`, `angular.z`), και
  (2) το hands-off base twist `(0,0)` που ο `_apply_command` publishάρει στο
  `/navigation/cmd_vel` **πριν** τον collect_route guard, και ο `MotionController`
  το relay στο `/cmd_vel_collection`. Και τα δύο @ ~31 Hz → η περιστροφή γίνεται
  stuttering/μισή ταχύτητα → τα 8 steps (45°, 0.5 rad/s, ~12.6s καθαρά) δεν
  ολοκληρώνονται εντός `scan_timeout_s=20s` → `aborted_scan` (timeout).
- **Fix:** Στο `controller_node._apply_command` ο `collect_route` guard
  μετακινήθηκε **πάνω** από το `_pub_motion_cmd.publish(twist)` — σε collect_route
  ο node δεν publishάρει **κανένα** base twist· ο executor κατέχει αποκλειστικά
  τα κανάλια (scan → `/cmd_vel_collection` 70, FollowPath → `/cmd_vel_nav` 50) και
  τον collector (Collector port).
- **Status:** ΕΦΑΡΜΟΣΤΗΚΕ, ΕΚΚΡΕΜΕΙ sim επαλήθευση (rebuild + ξανα-S1). Uncommitted
  μέχρι να επιβεβαιωθεί στο Gazebo.

## #21 — (Φάση 7, sim run 2-3) `insufficient_coverage`: rejected detections δεν μετρούσαν coverage
- **Παρατήρηση:** Μετά τον cmd_vel fix (#20) η περιστροφή ολοκληρώνεται (όχι πια
  timeout), αλλά το scan κατέληξε `aborted_scan (scan_failure=insufficient_coverage:
  4/8 steps covered)`. Η σκηνή είχε 15 μπάλες στην **απέναντι** μισή (RViz SimBalls).
- **Root cause:** Το coverage καταγραφόταν μόνο για (α) empty heartbeat frames
  (`record_empty_step`, όταν `not frame.detections`) και (β) **accepted** detections
  (`builder.add`). Ένα scan step του οποίου το frame είχε detections που **όλα
  απορρίφθηκαν** (cross-half `opposite_court_half`, stale metadata κ.λπ.) δεν
  μετρούσε coverage — έπεφτε ανάμεσα στις δύο διαδρομές. Τα 4 steps που κοιτούσαν
  τις far-half μπάλες → uncovered → coverage 4/8 < required 1.0.
- **Fix:** Το coverage μετράει **παρατήρηση sector**, όχι ball acceptance.
  `record_empty_step` → μετονομάστηκε `record_visited_step`· ο
  `CollectionSnapshotRuntimeAdapter.forward` το καλεί για **κάθε** frame με έγκυρη
  ταυτότητα (healthy + calibration_id + configuration_id + scan_step_id),
  **ανεξάρτητα** αν οι detections γίνουν accept ή reject, και μετά επεξεργάζεται
  κανονικά τις detections. Απαραίτητο και για πραγματικά runs (S2+): rejected
  detection στη δική σου μισή δεν πρέπει να χάνει coverage.
- **Diagnostics που προστέθηκαν (#20/#21):** το `ScanSessionDriver.last_failure_detail`
  + το node terminal log δείχνουν πλέον `scan_failure=<code>: N/M steps`.
- **Tests:** test_collection_scan_snapshot + test_collection_snapshot_runtime_adapter
  ενημερωμένα (rename + νέο assert ότι non-empty frame μαρκάρει visited_step)· 51
  targeted + 281 regression πράσινα.
- **Status:** ΕΦΑΡΜΟΣΤΗΚΕ, ΕΚΚΡΕΜΕΙ sim επαλήθευση (rebuild + ξανα-S1 → αναμένεται
  `completed_no_targets`). Uncommitted μαζί με #20.

## #22 — (Φάση 7, sim run 4) `completed_no_targets` με υπαρκτό 2D ball detection

- **Παρατήρηση:** Μετά από επιτυχημένο NavigateToPose και πλήρες scan, ο executor
  κατέληξε `completed_no_targets`. Το serialized route status έδειξε
  `planning_status=empty_no_balls` και μηδέν snapshot targets.
- **Live evidence:** Το canonical perception heartbeat ήταν healthy και ο YOLO
  δημοσίευε sports-ball detection με confidence περίπου `0.707`. Το ακριβές ROI
  probe στο timestamp-matched `32FC1` depth είχε shape `5×5`, valid fraction
  `1.0` και p20 range `3.9837 m`. Παρ' όλα αυτά το published detection είχε
  `has_spatial=false`, μηδενικά XYZ/distance και κενό matched-depth stamp.
- **Root cause:** Το approved Gazebo v2 covariance artifact έχει calibrated
  range `[1.0215745, 2.9799471] m`. Η μέτρηση `~3.984 m` επέστρεψε
  `calibration_out_of_domain`. Ο producer εφάρμοσε σωστά το no-extrapolation
  contract, αλλά έχανε το συγκεκριμένο rejection reason και το route terminal
  έμοιαζε με πραγματικά άδεια σκηνή.
- **Diagnostics fix:** Προστέθηκε operator-only `/perception/diagnostics` JSON
  heartbeat με 2D/spatial/rejected counts, rejection histogram, observed depth
  range/quality και calibrated range. Το `BallDetectionArray` παραμένει ακριβώς
  το canonical downstream target contract. Το controller διατηρεί το τελευταίο
  diagnostic στο collect-route status και το `completed_no_targets` terminal log
  εμφανίζει πλέον τον dominant λόγο και τα observed/calibrated ranges.
- **Calibration decision:** Δεν αλλάζει ούτε γίνεται overwrite το v2 artifact.
  Η επέκταση σχεδιάζεται ως ξεχωριστό Gazebo v3 evidence/activation flow στο
  `docs/perception/covariance/gazebo-perception-covariance-c2-v3-plan-el.md`. Ο design στόχος `0.2–9 m`
  θα περιοριστεί στο effective range που περνά πραγματικά neural-detection και
  covariance gates. Physical OAK-D evidence παραμένει ανεξάρτητο.
- **Tests:** Νέα pure diagnostics tests καλύπτουν empty heartbeat έναντι rejected
  2D detection και το ακριβές `calibration_out_of_domain` terminal detail.
  Targeted host gate: **17 passed**. Στο ROS 2 Humble container πέρασαν τα ίδια
  **17 tests** και χωριστά τα **8 perception contract tests**.
- **Live verification:** Με rebuild/restart, το `/perception/diagnostics` έδειξε
  frame με `detections_2d=3`, `spatial_accepted=2`, `spatial_rejected=1`,
  `calibration_out_of_domain=1`, observed range `1.1169–6.8922 m` και calibrated
  range `1.0216–2.9799 m`. Πλήρες collect-route run έφτασε ξανά
  `completed_no_targets`, τώρα με terminal detail
  `detections_2d=1, spatial=0, primary_rejection=calibration_out_of_domain,
  observed_range_m=3.942..3.942, calibrated_range_m=1.022..2.980`. Το frozen
  `collect_route`/`collection_run` status διατήρησε το ίδιο structured payload
  μετά την επιστροφή σε idle.
- **Status:** DIAGNOSTICS ΕΦΑΡΜΟΣΤΗΚΑΝ ΚΑΙ LIVE-VERIFIED· V3 CALIBRATION ΕΚΚΡΕΜΕΙ.

## #23 — Perception geometry, v3 calibration και scan association

- **Root causes:** (1) RGB HFOV `1.204` και depth HFOV `1.274` έκαναν το ίδιο
  pixel ROI να δείχνει διαφορετική ακτίνα, (2) το depth optical-axis `Z`
  χρησιμοποιούνταν σαν slant range, και (3) ο snapshot adapter αγνοούσε το
  optical `forward Z` μετασχηματίζοντας μόνο `(right, down)` σαν map XY.
- **Detector/scan:** Προστέθηκαν neural-only zoom tiles (factor `3.0`, confidence
  `0.35`, χωρίς HSV fallback), containment-aware NMS και 18 scan headings. Το
  association χρησιμοποιεί μία φορά το shared localization budget, χωρίς να
  το διαιρεί κατά το information fusion.
- **Calibration:** Νέο immutable v3 artifact από 18/18 trials και 540 accepted
  samples. Κάθε trial είχε 0% target outliers. Domain `1.0218–6.7653 m`, quality
  `0.8889–1.0`, SHA-256
  `338adb895e764422e51ddde549726514815539a8ddcf3dbab82c17c2563b7027`.
- **9 m clarification:** Το pilot στα `8.263 m` απέτυχε στο stock-YOLO target
  detection gate. Τα 9 m είναι depth-sensor capability, όχι εγγυημένο neural
  ball-perception range. Δεν έγινε extrapolation.
- **Live result:** Η τελική σάρωση παρήγαγε 30 accepted observations, 13 tracks
  και **10 confirmed snapshot targets** (7×3-step, 3×2-step confirmations).
  Ο planner επέστρεψε `empty_no_feasible_targets` (1 `keepout`, 9
  `no_candidate_found`), άρα το επόμενο blocker είναι planner/court geometry,
  όχι perception.
- **Reports:** `gazebo-perception-covariance-c2-v3-{coverage,artifact,
  activation}-report-el.md` και structured `snapshot_diagnostics` στο status.
- **Status:** PERCEPTION FIX + V3 ACTIVATION LIVE-VERIFIED. PLANNER FEASIBILITY
  ΕΚΚΡΕΜΕΙ ΩΣ ΞΕΧΩΡΙΣΤΟ ΘΕΜΑ.

## #24 — Bounded planning και end-to-end collection route

- **Snapshot lifecycle:** Το timeout ξεκινά πλέον στην πραγματική έναρξη της
  σάρωσης και το snapshot κλειδώνει το post-scan robot pose. Το scan FSM
  αρχικοποιείται από το yaw της configured scan pose, οπότε ολοκληρώνει και τα
  18/18 headings χωρίς ψευδές timeout μετά το navigation.
- **Planner boundedness:** Το `maximum_candidate_count` εφαρμόζεται πριν από την
  κατασκευή του connector graph με deterministic set coverage και ισόρροπες
  εναλλακτικές. Το live cap είναι 48 candidates, αντί για graph ~244 nodes / ~238k
  directed Dubins edges που μπλόκαρε τον controller node.
- **Controller semantics:** Το heading hard gate συγκρίνει το planned path yaw
  με το robot yaw· το lookahead bearing χρησιμοποιείται μόνο για curvature.
  Το controller διαβάζει velocity από `/odometry/filtered`, όχι από το raw
  Gazebo `/odom`, και το nested `profile_verdict` γράφεται ως JSON-safe dict.
- **Tracking margin:** Planning radius `1.25 m`, hard curvature `1.25 1/m` και
  lookahead `0.6 m`. Έτσι η nominal planned curvature περιορίζεται σε `0.8 1/m`
  και μένει ~56% περιθώριο closed-loop correction χωρίς χαλάρωση του hard gate.
- **Live result:** 18/18 scan headings, 29 accepted observations, 11 tracks και
  **9 confirmed targets**. Η bounded αναζήτηση επέστρεψε `partial`: 2 selected,
  7 deferred. Η route των `12.13 m` ολοκληρώθηκε με
  `route_outcome=route_completed`, `failure_reason=0` και 48 crossing telemetry
  samples. Και οι δύο ball crossings ήταν `hard_compliant=true` στα `0.35 m/s`.
- **Status:** END-TO-END LIVE PASS. Το perception δεν είναι πλέον το blocker· η
  κάλυψη των 7 deferred balls είναι ξεχωριστή βελτίωση search budget/follow-up.

## #25 — Startup race, live 360° map και progress tolerance

- **Observed log:** Πρώτη εντολή `collect_route` δόθηκε πριν το delayed Nav2
  lifecycle bring-up και τερμάτισε `aborted_scan` με 0/18 steps. Σε επόμενη run
  το scan είχε 13 tracks / 9 confirmed, αλλά το Collection Workspace έδειχνε 0
  επειδή διάβαζε μόνο το legacy `BallMap`. Η route εμφάνισε επίσης typed
  `failure_reason=11` στα 0.005 m από μικρό localization projection jitter.
- **Nav2 readiness:** Το scan-pose adapter ξαναδοκιμάζει όταν το action endpoint
  λείπει ή απορρίπτει goal πριν γίνει lifecycle-active. Accepted goal που μετά
  αποτυγχάνει παραμένει κανονικό `navigation_failed`.
- **Live map:** Τα `snapshot_diagnostics.tracks` τροφοδοτούν πλέον το map payload
  κατά τη σάρωση. Track ενός distinct heading εμφανίζεται pending και γίνεται
  confirmed μόλις καλύψει το configured confirmation gate. Το source δηλώνεται
  `oak_depth` και διατηρούνται οι fused map συντεταγμένες.
- **Controller tolerance:** Η δηλωμένη ανοχή προόδου 0.05 m χρησιμοποιείται για
  μικρό projection regression και terminal completion. Πραγματική υποχώρηση
  μεγαλύτερη από την ανοχή εξακολουθεί να δίνει non-monotonic hard failure.
- **Live verification:** Εντολή στάλθηκε πριν το Nav2 activation και παρέμεινε
  `navigating_to_scan_pose` μέχρι να γίνει διαθέσιμο. Στο step 4/18 ο χάρτης
  έδειχνε ήδη 3 tracks (1 pending, 2 confirmed), στο 18/18 έδειχνε 12 tracks / 9
  confirmed. Η route ξεκίνησε με `failure_reason=0`, πέρασε hard-compliant
  crossing στα 0.35 m/s και έφτασε εντός 1.4 mm του terminal point.
- **Tests:** 213 collection Python tests PASS. Όλα τα C++ gtests/runtime tests
  PASS και το isolated Nav2 launch test PASS σε `ROS_DOMAIN_ID=43`.
## #26 — Collection route σταματά στο terminal και ο Nav2 κάνει progress abort

- Παρατήρηση live run: ο collection controller έφτασε στο terminal progress
  `5.270 / 5.271 m`, επέστρεψε μηδενική ταχύτητα και δεν είχε profile failure.
- Η τελική απόσταση της βάσης από το path goal ήταν περίπου `0.219 m`, ενώ ο
  κοινός `general_goal_checker` απαιτούσε `0.10 m`. Ο Nav2 επομένως δεν έκλεινε
  το FollowPath και μετά το progress timeout επέστρεφε abort.
- Προστέθηκε αποκλειστικός `collection_goal_checker` (`0.30 m`, yaw ελεύθερο)
  και το `goal_checker_id` περνά ρητά στο FollowPath goal. Ο γενικός checker
  μένει στα `0.10 m`, άρα survey και κανονική πλοήγηση δεν χαλαρώνουν.
- Η ανοχή καλύπτεται από το ανεξάρτητο terminal run-out `0.5 m` της collection
  route και είναι μεγαλύτερη από το μετρημένο Gazebo stopping envelope.
- Επειδή το Nav2 Humble δεν δέχεται κενό checker id όταν έχουν φορτωθεί δύο
  plugins, τα κανονικά NavigateToPose/court-survey behavior trees δηλώνουν
  πλέον ρητά `general_goal_checker`. Χωρίς αυτό, η μετάβαση προς το scan pose
  απορριπτόταν πριν κινηθεί το robot.
- **Live verification:** πλήρες νέο run με 18/18 headings, 14 tracks και 10
  confirmed targets. Η partial route είχε μήκος `12.975 m`, δύο crossings και
  τερμάτισε `completed / route_completed`, με `failure_reason=0`. Ο Nav2 έγραψε
  `Reached the goal!` χωρίς progress abort. Η βάση σταμάτησε `0.277 m` από το
  path endpoint και `0.523 m` μετά το τελευταίο ball crossing, άρα διατηρείται
  το required run-out `0.30 m`.
- Το Collection Log συγχωνεύει πλέον και τα `collect_route.executor_events`,
  ώστε οι καταστάσεις scanning/planning/executing/route_completed να φαίνονται
  στην κονσόλα αντί να μένει μόνο το αρχικό `Entered collect_route mode`.

## #27 — Route καλύπτει μόνο 2–4 μπάλες: bottleneck στο candidate/edge layer

- **Παρατήρηση:** Live Gazebo runs (Φ7) έδιναν route με 2 selected / 7 deferred
  ενώ η σάρωση έβρισκε 9–14 confirmed. Πρώτη υπόθεση «ρηχή αναζήτηση»
  (`max_search_expansions`).
- **Μέτρηση που ανέτρεψε την υπόθεση:** σε probe 10 σκορπισμένων μπαλών, το
  search budget `1.000 → 1.000.000` **δεν** άλλαξε τίποτα· η DFS τρέχει σε
  **6ms** και saturate-άρει στις 4/10. Το πραγματικό όριο ήταν το
  `maximum_candidate_count: 48`: cap 48→**4/10**, 100→8/10, **200→10/10
  (feasible)** με την **ίδια** DFS.
- **Root cause:** το `_build_edge` έκανε `_materialize_path` (subdivided
  geometry) **πριν** τον έλεγχο turning/length limits. Στο cap=200
  υπολογίζονταν **121.104 CSC edges, valid 1.781 (98,5% turning-rejected)**,
  graph-build **3,6s** — γι' αυτό το cap κρατιόταν στο 48.
- **Fix #1 (edge gate):** analytic turning/length gate πάνω στα closed-form
  `(t, p, q)` **πριν** το materialize. Μέτρηση: gate-only 90ms για όλα τα 121k,
  ίδια 1.781 valid → `_materialize_path` ~68× λιγότερα. Graph-build cap=200
  **3,6s → 0,45s**, cap=48 286ms → 52ms. Regression test επιβεβαιώνει
  byte-identical accept/reject set και rejection codes vs materialize-first.
- **Fix #2 (bounded search):** cost-guided DFS ordering (μέγιστη νέα κάλυψη →
  μικρότερο μήκος → stable id) + admissible branch-and-bound (static
  forward-reachable coverage bound + prefix cost lower-bound). Το full-coverage
  route βρίσκεται σε λίγα expansions ανά μπάλα· χωρίς αυτό, στο cap=200/14-ball
  η εξαντλητική αναζήτηση εκρήγνυτο σε **12s**.
- **Fix #3 (config):** `maximum_candidate_count 48 → 200`,
  `max_search_expansions 1000 → 3000`. Το graph-build είναι φραγμένο από το cap
  ανεξαρτήτως αριθμού μπαλών.
- **Αποτέλεσμα (full pipeline, cap=200):** 10-ball **10/10 feasible σε ~490ms**,
  14-ball **14/14 feasible σε ~733ms** — εντός του δηλωμένου
  `maximum_planning_time_s: 1.0`.
- **Tests:** 319 pure pytest PASS (73 collection-route). Contracts αμετάβλητα:
  scoring (max coverage → min cost → min pass count → stable id), BallReasonCode,
  PlanningStatus/SearchStatus, determinism. C++ αμετάβλητο.
- **Εκτός scope (ξεχωριστό):** το tracking-abort στο handoff μετά τη σάρωση (βλ.
  ανάλυση Φ7) παραμένει ανοιχτό και μπορεί ακόμη να εμφανιστεί live· δεν αφορά το
  candidate/edge layer.

## #28 — Abort reason στο Collection Log + διόρθωση failure-code mapping

- **Live επιβεβαίωση #27:** πρώτο Gazebo run μετά το coverage fix έδειξε
  **planned 9** (vs 2 πριν) — ο planner καλύπτει πλέον πολλές μπάλες. Όμως η route
  κόβει αμέσως μετά τη σάρωση με `aborted tracking | path failed`, χωρίς ο λόγος
  να φαίνεται στο log.
- **Διόρθωση mapping (σημαντικό):** το published `CollectionControllerState.
  failure_reason` χρησιμοποιεί **msg numbering**, ΟΧΙ τον 0-based δείκτη του C++
  `TrackingFailureCode`. Άρα το `failure_reason=10` που είχε καταγραφεί ήταν
  **`FAILURE_TRAJECTORY_TUBE_EXCEEDED`**, όχι `reverse_required` (που είναι 12).
  Η προηγούμενη ανάλυση Φ7 είχε μεταφράσει λάθος τον κωδικό — γι' αυτό ακριβώς
  χρειαζόταν ανθρωπινό label στο log.
- **Fix (observability):** ο live PathFollower port χτίζει τώρα detail string με
  το σωστό `FAILURE_LABELS[code]` + live geometry (`seg`, `progress`, `lat_err`,
  `head_err`, `speed`) από το controller state, και το περνά μέσω
  `PathFollowerResult.detail` → `TelemetryEvent.detail` →
  `telemetry_event_to_dict` → `executor_events` → Collection Log (app.js). Το
  executor reason contract (`path_failed`) μένει αμετάβλητο· το detail είναι
  παράλληλη διαγνωστική πληροφορία.
- **Αρχεία:** `collection_path_follower_port.py` (labels + `_failure_detail`),
  `collection_route_executor.py` (`PathFollowerResult.detail`,
  `TelemetryEvent.detail`, `_abort_active_route`/`_transition`),
  `collection_executor_ports.py` (serialization), `scripts/control_panel/app.js`
  (render). Node factory `state_provider` ήδη παρέχει όλα τα πεδία.
- **Tests:** 320 pure pytest PASS (+ νέα: detail flows through port +
  `telemetry_event_to_dict` περιλαμβάνει detail). C++ αμετάβλητο.
- **Επόμενο:** επόμενο live run θα δείξει στο log τον πραγματικό κωδικό+γεωμετρία
  (π.χ. `trajectory_tube_exceeded | seg connector-0 progress 0.120m lat_err
  0.31m ...`), ώστε να κριθεί αν είναι transient handoff ή γνήσιο tube violation.

## #29 — Root cause του tracking abort: pure-pursuit vs σφιχτό heading gate

- **Live log (χάρη στο #28):** ο planner τώρα σχεδιάζει πλήρη διαδρομή (planned 10,
  57.7m, replans 0) και το robot οδηγεί ~7m πριν κόψει με
  `aborted tracking | path failed | heading_error_exceeded | seg connector-2
  progress 7.171m lat_err 0.000m head_err -0.152rad speed 0.000m/s`.
- **Artifact:** το `speed 0.000m/s` ΔΕΝ σημαίνει stall. Το published
  `state.measured_speed_mps` γεμίζει **μόνο από crossing measurement**
  (`collection_nav2_controller.cpp:340`) → πάντα 0 σε connector. Το robot κινούνταν
  κανονικά· η πραγματική ταχύτητα υπάρχει μόνο στο RCLCPP_ERROR (`:262`).
- **Root cause:** ο controller οδηγεί με **pure-pursuit** (lookahead 0.6m,
  `collection_tracking_core.cpp:167`) αλλά το hard gate (`:158`) ελέγχει σφάλμα ως
  προς το **path tangent**. Στις μεταβάσεις ευθεία→τόξο (S→C) των CSC connectors το
  lookahead μπαίνει στο τόξο νωρίς → anticipatory turn → το heading **προηγείται**
  του tangent κατά ~`lookahead*curvature/2` = `0.6*0.8/2 ≈ 0.24 rad` στο planned cap
  0.8/m. Το gate ήταν **0.15 rad** → self-abort. `lat_err=0` (θέση τέλεια, καθαρά
  heading transient) το επιβεβαιώνει. Συστηματικό: κάθε connector με curvature
  ≳0.5/m αβορτάρει — γι' αυτό οδήγησε 7m (ήπια) και μετά κόπηκε.
- **Γιατί λάθος στο connector:** το heading gate προστατεύει **capture alignment**,
  που έχει σημασία μόνο στα crossings (funnel passes = **ευθεία**, κ≈0 → μηδέν lead
  → 0.15 ικανοποιείται φυσικά). Στους connectors (transit) το heading lead είναι
  εγγενές στο pure-pursuit και αβλαβές.
- **Fix:** νέο config `planning.connector_max_heading_error_rad: 0.5`. Το
  `_connector_segment` δίνει στους CONNECTOR segments profile με χαλαρό heading gate
  (`replace(default, max_heading_error_rad=0.5)`), ενώ τα funnel passes κρατούν το
  σφιχτό 0.15. Per-segment profile → serialize-άρεται στο C++ context
  (`node_factory:229`) → ο core το διαβάζει ανά segment. Όλα τα άλλα bounds (speed,
  curvature, lateral tube) αμετάβλητα. Value 0.5: άνετο πάνω από το lead
  ~0.24-0.38 rad, πιάνει ακόμα >28.6° γνήσιο mis-tracking.
- **Fix #28 artifact:** το `_failure_detail` δείχνει πλέον `speed` **μόνο** στα
  speed-limit failure codes (5/6), όχι το παραπλανητικό 0 σε heading/lateral aborts.
- **Tests:** 320 pure pytest PASS (+ νέο composition test: connectors χαλαρό gate,
  passes default). C++ αμετάβλητο (per-segment profile ήδη υποστηρίζεται).
- **Επόμενο:** live run — η διαδρομή πρέπει να ολοκληρώνεται χωρίς
  `heading_error_exceeded` στους connectors.

## #30 — non_monotonic_progress false-positive σε self-crossing loop routes

- **Live μετά το #29:** το heading fix δούλεψε (κανένα `heading_error_exceeded`).
  Το robot οδήγησε **35.8m / 54.9m** και **μάζεψε 2 μπάλες** (`ball_17`, `ball_12`
  πέρασαν στο καλάθι). Έκοψε με
  `aborted tracking | path failed | non_monotonic_progress | seg connector-10
  progress 35.806m lat_err 0.000m head_err 0.000rad`.
- **Artifact:** τα `lat_err=0, head_err=0` είναι **defaults** (δεν υπολογίζονται
  γι' αυτόν τον τύπο failure — early return στο `collection_tracking_core.cpp:120`).
  Μη ενδεικτικά.
- **Root cause:** ο non-monotonic guard (`:120`, `:268`) χρησιμοποιεί το **global
  nearest-point projection** (`raw`, χωρίς window — ψάχνει όλα τα segments). Οι νέες
  υψηλής κάλυψης διαδρομές είναι **loops που αυτο-διασταυρώνονται**· εκεί που ο
  βρόχος επιστρέφει κοντά σε προηγούμενο segment, το global nearest «κολλάει» πίσω
  (π.χ. 35.8m→~5m) → ψευδές «η πρόοδος πήγε πίσω» → abort. Το bounded (τοπικό)
  tracking ήταν μια χαρά.
- **Fix (C++ tracking core):** το raw projection για τον backward έλεγχο
  περιορίζεται σε window `progress_projection_window_m` (10m) **πίσω** από το
  tracked progress: ανιχνεύει γνήσιο on-path regression αλλά αγνοεί μακρινά
  self-intersections. Καμία αλλαγή σε msg/param/Python — reuse υπάρχοντος window.
  `collection_tracking_core.cpp` raw-update guard `within_regression_window`.
- **gtest:** νέο `SelfCrossingLoopDoesNotFalselyReportNonMonotonicProgress` —
  figure-8 (περνά από αρχή σε t=0 και t=π), mid-path crossing, lenient profile·
  επιβεβαιώνει κανένα false non_monotonic. Το υπάρχον regression test (γνήσιο
  backward εντός window) παραμένει (window 10m ≫ 1m regression). Core syntax OK
  τοπικά· τα gtests τρέχουν στο container.
- **Δευτερεύον (χωριστό):** `beam missed ball: 2 retained but beam saw 0` —
  reconciliation του basket-beam sensor· οι μπάλες όντως μαζεύτηκαν (basket-entry
  events πυροδότησαν). Δεν αφορά τη διαδρομή.
- **Επόμενο:** rebuild C++ plugin + live run· η διαδρομή πρέπει να προχωρά πέρα από
  τα self-crossings και να πλησιάζει ολοκλήρωση.

## #31 — Πρώτη πλήρης ολοκλήρωση route + UI speed/elapsed

- **Live:** πρώτο `route_outcome=route_completed` / `state=completed` end-to-end (ο
  μηχανισμός συλλογής μπαλών είναι εκτός scope προς το παρόν).
- **UI observability:** προστέθηκαν «Speed» και «Elapsed» στο Collection Run panel.
  - *Speed:* πραγματική ground speed από `/odom` twist. Το `_on_odom` την κρατά
    **πριν** το `slam_tf` short-circuit (το twist έρχεται πάντα από /odom), σε
    `_robot_speed_mps` → top-level status `measured_speed_mps`. Δεν χρησιμοποιείται
    το `cmd_linear_m_s` γιατί στο hands-off collect_route η Python base command
    είναι idle (0)· ούτε το `controller_state.measured_speed_mps` (crossing-only
    artifact, βλ. #29).
  - *Elapsed:* `_collect_route_elapsed_s(state)` — live `now - started` όσο τρέχει,
    frozen στο last-event `t_s` σε terminal state. Στο collect_route summary →
    `collection_run.elapsed_s`.
  - `app.js renderCollectionRun` δείχνει τα δύο πεδία (fallback `0.00 m/s` / `-`).
  - *Collection Log ordering:* το sort/filter/clear χρησιμοποιεί πλέον κοινό
    `eventRecency` = `sim_time_s ?? t_s`. Το `sim_time_s` είναι μονότονο ανά
    session (δεν μηδενίζεται ανά run όπως το `t_s`), οπότε το log μένει «πιο
    πρόσφατο πρώτα» ακόμη και σε επαναλαμβανόμενα runs (πριν, stale events παλιού
    run με υψηλό `t_s` επέπλεαν στην κορυφή).
- **Tests:** 321 pure pytest PASS· controller_node/app.js syntax OK. Ο container
  test `_build_collect_route_summary` προστατεύεται με `getattr` για το
  `_collection_event_started_at` (το SimpleNamespace helper δεν το ορίζει).
- **Επόμενο:** rebuild (C++ #30 + Python) + live· επανάληψη ολοκληρωμένης διαδρομής
  με ορατά speed/elapsed ⇒ κοντά στο κλείσιμο της υλοποίησης.

## #32 — Route map/odom mismatch: σωστό plan, μετατοπισμένη εκτέλεση

- **Σύμπτωμα:** η σάρωση και ο planner έδειχναν πολλές μπάλες/crossings, αλλά το
  robot περνούσε δίπλα τους και φυσικά κρατούσε μόνο μία ή δύο. Σε ολοκληρωμένο
  run ο planner ανέφερε 8 covered crossings, ενώ τα Gazebo basket-entry events
  επιβεβαίωσαν μόνο 2 retained balls.
- **Root cause:** το immutable route και το `FollowPath` είχαν frame `map`, ενώ ο
  Nav2 controller server δίνει στο custom `CollectionNav2Controller` το robot
  pose στο local-costmap frame `odom`. Ο C++ tracking core, σκόπιμα χωρίς TF
  dependency, συνέκρινε απευθείας τις odom συντεταγμένες του robot με map
  συντεταγμένες της διαδρομής. Η διαφορά `map→odom` μετατόπιζε ολόκληρη τη φυσική
  τροχιά από τα detected ball positions.
- **Fix:** αμέσως πριν χτιστούν execution context/path hash γίνεται ένα
  `lookup_transform(odom, map)` και παγώνει. Νέο pure module
  `collection_execution_frame.py` εφαρμόζει τον ίδιο rigid 2D transform σε start,
  terminal, όλα τα path points και όλα τα planned crossings. Planner/UI μένουν
  στο `map`, αλλά context και FollowPath είναι συνεπή στο `odom`.
- **Terminal hardening:** η C++ ολοκλήρωση απαιτεί πλέον και πραγματική ευκλείδεια
  απόσταση από το terminal point εντός tolerance, όχι μόνο projection progress.
  Η tolerance του core ευθυγραμμίστηκε με τον αποκλειστικό collection goal
  checker στα `0.30 m`. Αν ο controller απορρίψει το terminal Finalize, ο executor
  επιστρέφει `path_failed` αντί για ψευδές `completed`.
- **Nav2 Humble startup:** διορθώθηκαν τα pluginlib class names του Smac planner
  και των behavior plugins από `::` σε `/`. Πριν, ο lifecycle manager σταματούσε
  στο configure και δεν ενεργοποιούσε όλο το navigation stack.
- **Verification:** `220` collection pytest PASS, ROS Humble image build PASS και
  controller package `36 tests, 0 errors, 0 failures`. Νέο headless run:
  18/18 scan headings, 10 tracks, 8 planned crossings, συνεχής εκτέλεση με
  lateral error περίπου `0.005–0.028 m` και χωρίς άμεσο cancel/tracking abort.
  Το run διακόπηκε ελεγχόμενα στα `3.77 m` πριν από το πρώτο crossing όταν
  ξεκίνησε παράλληλο native Jazzy stack πάνω στο ίδιο status/command file· δεν
  καταγράφεται ως physical multi-ball acceptance.
- **Acceptance που απομένει:** ένα αποκλειστικό (single-stack) live run μέχρι
  τουλάχιστον το δεύτερο crossing, με δύο διαφορετικά basket-entry/retained ball
  IDs. Δεν πρέπει να τρέχουν ταυτόχρονα Docker Humble και native Jazzy.

## #33 — Native Jazzy: collect_route εκτελείται (δύο migration bugs)

**Context:** πρώτο collect_route σε native ROS 2 Jazzy (WS1 του Pi deployment). Το
Nav2 έφτανε `Managed nodes are active`, ο planner έβγαζε πλήρες route (10/10 balls,
57.7 m), αλλά το robot δεν προχωρούσε — και μετά το fix, crash-άριζε στο ~50%.

- **Bug 1 — Nav2 → twist_mux cmd_vel type mismatch (robot ακίνητο υπό Nav2).**
  Το `controller_server`/`behavior_server` δημοσίευαν σκέτο `geometry_msgs/Twist`
  στο `/cmd_vel_nav`, ενώ το twist_mux (stamped mode στο Jazzy, όπως teleop &
  collection) έκανε subscribe ως `TwistStamped`. Το `ros2 topic info /cmd_vel_nav`
  έδειχνε **δύο** τύπους — το DDS δεν συνέδεε ασύμβατα endpoints, οπότε **καμία
  εντολή ταχύτητας δεν έφτανε στο diff_drive**. Ο planner «Passing new path»
  συνεχώς, ο controller «Failed to make progress» μετά ~28 s. Το D-pad δούλευε
  γιατί περνά από restamp relay. **Fix:** `enable_stamped_cmd_vel: true` σε
  `controller_server` + `behavior_server` (`nav2_params.yaml`). Επαλήθευση: το
  `/cmd_vel_nav` έγινε **single** `TwistStamped` και το robot κινήθηκε προς το
  scan pose και εκτέλεσε το route. (Root cause = Humble→Jazzy: το Humble ήταν
  όλο Twist· στο Jazzy ο stack πήγε TwistStamped αλλά ο Nav2 έμεινε στο default
  `enable_stamped_cmd_vel: false`.)

- **Bug 2 — `RuntimeError: Executor is already spinning` στο finalize (crash ~50%).**
  Μόλις ο controller έφτανε `Reached the goal!`, ο executor έμπαινε στο finalize
  και ο `controller_node` πέθαινε με exit 1. Traceback: `finalize_sender`
  (`collection_executor_node_factory.py:399`) καλούσε
  `spin_until_future_complete` **μέσα από το timer callback** του controller_node,
  δηλαδή nested spin στον ίδιο single-threaded executor. Το Humble rclpy το
  ανεχόταν (γι' αυτό ολοκλήρωναν τα προηγούμενα runs)· το Jazzy rclpy πρόσθεσε
  guard (`_enter_spin`) και σκάει. **Fix:** fire-and-forget `call_async` (ίδιο
  pattern με το διπλανό `hold_sender`), return `True` — το request παραδίδεται
  στον collection controller, χωρίς αναμονή ack (ο Nav2 goal έχει ήδη πετύχει).
  Ήταν το μοναδικό nested-spin site (τα υπόλοιπα `rclpy.spin` είναι top-level
  `main()`). Επαλήθευση: `23` pytest (`test_collection_executor_node_factory.py`
  + `test_collection_path_follower_port.py`) PASS.

- **Επίσης σε αυτό το run:** spawners (joint_state_broadcaster, diff_drive,
  intake) «finished cleanly» μετά από καθαρό restart — το προηγούμενο
  `Failed to acquire lock` ήταν transient, όχι μόνιμο. Το D-pad web control
  ξαναδούλεψε αφού διορθώθηκαν 3 UI bugs (type=submit navigation, aborted-fetch
  NetworkError, stale-survey auto-route redirect).

- **Bug 3 — `invalid terminal progress` (arc-vs-chord), σταματούσε μετά το 360.**
  Μετά τα bug 1+2, το route σχεδιαζόταν αλλά μόλις ο executor έδινε το πρώτο
  segment στον C++ `CollectionFollowPath`, ο controller πετούσε
  `collection_controller_profile_unenforceable`. Το catch στο
  `setPlan` (`collection_nav2_controller.cpp`) κατάπινε τον πραγματικό λόγο —
  προστέθηκε `e.what()` logging και αποκαλύφθηκε: `make_tracking_plan rejected
  path: invalid terminal progress`. **Root cause:** το `progress` που υπολογίζει
  το `make_tracking_plan` είναι άθροισμα **chord** αποστάσεων των διακριτών
  FollowPath poses, ενώ το `context.terminal_progress_s` είναι το **arc length**
  της route (πάντα ≥ chord-sum σε καμπύλες). Ο έλεγχος στο `make_tracking_plan`
  δέχεται ισότητα εντός `terminal_progress_tolerance_m`, αλλά ο constructor του
  `CollectionTrackingCore` απαιτεί `terminal_progress_s <= path.back().progress_s`
  **αυστηρά** → overshoot λίγων mm έριχνε το guard. Επιφάνεια τώρα επειδή η νέα
  route planner (Phase 7, cap 200, B&B) βγάζει πιο καμπύλες διαδρομές. **Fix:**
  cap του `terminal_progress_s` στο πραγματικό μήκος path (`progress`) στο
  `make_tracking_plan`, αφήνοντας NaN ανέγγιχτο για τον constructor. Επαλήθευση:
  10 tracking-core + 5 runtime + plugin + canonicalization gtests PASS.
- **Χωριστό (μη-μπλοκάρον):** το `test_collection_controller_server_isolated.launch.py`
  αποτυγχάνει γιατί το harness δεν δημοσιεύει robot TF (base_link→odom) — ο Nav2
  controller_server δεν παίρνει pose, δεν καλεί το `computeVelocityCommands`,
  καμία εντολή. Test-infra θέμα, όχι το live path (live: TF υπάρχει).
- **Acceptance που απομένει:** πλήρης single-stack native Jazzy run που να
  **ολοκληρώνεται** (πέρα από το finalize + πρώτο segment), + ο μηχανισμός μαζέματος.

## #34 — Distributed (Pi) collect_route: scan coverage + connector curvature

**Context:** πρώτα distributed collect_route runs (Pi εγκέφαλος οδηγεί PC sim, GUI
ώστε το perception να βλέπει μπάλες). Δύο tuning fixes ξεκλείδωσαν το route.

- **`insufficient_coverage` abort παρόλο που βρέθηκαν μπάλες.** Το scan
  επιβεβαίωσε έως 6 μπάλες αλλά abort-άριζε στο `insufficient_coverage: 12/18
  steps covered` γιατί `scan.required_coverage_fraction` ήταν **1.0** (valid
  spatial observation σε ΟΛΑ τα 18 headings). Στο distributed (RGB/depth sync
  slop → `non_spatial_detection` rejections + rotation timing) σπάνια πιάνεις
  18/18. **Fix:** `required_coverage_fraction` 1.0→**0.6** (+ `scan_timeout_s`
  20→90 για τον πιο αργό distributed sweep). Οι confirmed tracks οδηγούν το
  planning ούτως ή άλλως.
- **`FAILURE_CURVATURE_EXCEEDED` (code 9) στο route execution.** Μετά το scan,
  το πρώτο route segment abort-άρισε στα `progress_m=0.359` με
  `lateral_error=0, heading_error=0, speed=0.441` — τέλειο tracking, αλλά η
  **pure-pursuit commanded curvature ξεπέρασε το `max_curvature_per_m` (1.25)**
  σε ένα connector→crossing transition (αδερφή περίπτωση του
  `connector_max_heading_error_rad`, #29). **Fix:** default profile
  `max_curvature_per_m` 1.25→**2.5** (crossings είναι ευθεία → capture accuracy
  ανεπηρέαστη· 2.5 1/m @0.35 m/s = 0.875 rad/s ≪ max_angular_velocity 3.0).
  Καθαρότερο μελλοντικό: connector-specific curvature override στο
  `collection_route_global_solver.py:276` (όπου μπαίνει το heading override).
- **Ανοιχτά (γνωστά):** (α) ο planner διαλέγει υποσύνολο των confirmed μπαλών
  (half-court filter ή feasibility — θέλει scan-diagnostic+planning-result για
  root cause)· (β) collected-count telemetry δεν γράφεται στα runtime files
  (beam/plan reconciliation)· (γ) scan coverage / clean-court για fuller
  collection. Αυτά είναι το collect_route/mechanism refinement layer.

## #35 — Regression audit: γιατί η αποστολή σταμάτησε πριν μαζέψει όλες τις μπάλες

### Freeze

- Σταμάτησαν καθαρά και τα δύο stacks (Pi brain και PC Gazebo). Δεν γίνεται άλλη
  live αλλαγή ή tuning πριν απομονωθεί η μεταβλητή που άλλαξε τη συμπεριφορά.
- Τελευταίο επιβεβαιωμένο καθαρό checkpoint: `1d3ba10`. Η καταγραφή WS5 αναφέρει
  `scan → plan → execute → route completes` και `10/10 balls planned`, με ρητά
  ανοιχτή ακόμη την ποιότητα φυσικής συλλογής.
- Τρέχον committed checkpoint: `d76fdc9`. Το worktree έχει επιπλέον μη committed
  αλλαγές και **δεν** αποτελεί νέο baseline.

### Τι άλλαξε μετά το `1d3ba10`

- Το `d76fdc9` δεν αλλάζει τον planner ή τον global solver. Οι μόνες
  route-lifecycle ρυθμίσεις του είναι:
  - `collector_stop_timeout_s: 2.0 → 6.0`,
  - `follow_up.enabled: false → true`,
  - `follow_up.max_total_runs: 1 → 2`.
- Οι τρέχουσες μη committed αλλαγές επίσης δεν αλλάζουν planner/global-solver
  source. Η μοναδική άμεση αλλαγή γεωμετρίας route είναι:
  - `minimum_run_in_m: 1.0 → 1.9`,
  - `default_execution_profile.required_run_in_m: 1.0 → 1.9`.
- Οι υπόλοιπες μη committed αλλαγές αφορούν intake attribution/reset,
  foreground-depth validation, terminal status/diagnostics και UI.

### Τι έδειξαν τα live runs

- Ένα προηγούμενο run με το τρέχον dirty worktree έδειξε στο UI
  `confirmed 11 / planned 11`, route μήκους `70.9 m`. Άρα το dirty code/config
  μπορεί να σχεδιάσει πολυ-στόχο route και δεν υπάρχει από μόνο του απόδειξη
  ότι το run-in 1.9 κατέρρευσε τον planner σε έναν στόχο.
- Το τελευταίο run **δεν** ξεκίνησε από pristine Gazebo state:
  - robot start `(-0.630, -5.056)` αντί του αρχικού pose,
  - είχαν προηγηθεί failed/completed routes χωρίς PC/Gazebo reset,
  - μπάλες είχαν ήδη συλλεχθεί, μετακινηθεί ή σπρωχτεί.
- Σε αυτό το contaminated state έγιναν δύο `partial` plans, από ένα executable
  target το καθένα. Συλλέχθηκαν και retained δύο μπάλες (`ball_12`, `ball_17`).
  Μετά το δεύτερο clean sub-route ο executor σταμάτησε σε
  `incomplete_targets`, με `unresolved_targets=5`.

### Offline A/B χωρίς κίνηση robot

Ίδιο frozen synthetic/pristine snapshot: οι 12 μπάλες του αριστερού μισού από
το Gazebo world, μετασχηματισμένες στο τρέχον survey map frame, ίδιο
`court_boundary.json`, ίδιο start pose `(2.07, 0.00)`, ίδια covariance και όλο
το υπόλοιπο configuration:

| Μεταβλητή | Status | Covered | Deferred | Route length |
|---|---:|---:|---:|---:|
| run-in `1.0 m` | partial | 9/12 | 3 route_conflict | 41.58 m |
| run-in `1.9 m` | partial | 9/12 | 3 route_conflict | 55.29 m |

Συμπέρασμα A/B: το `1.0 → 1.9` αυξάνει σημαντικά το route length, αλλά **δεν**
μείωσε την κάλυψη στο ίδιο καθαρό snapshot. Δεν τεκμηριώνεται ως η αλλαγή που
έκανε το τελευταίο plan μονο-στόχο.

### Pristine pre-execution capture και exact replay

Προστέθηκε opt-in instrumentation στο planner boundary με
`COLLECTION_ROUTE_AUDIT_DIR`. Είναι default-off και αποθηκεύει atomically το
πλήρες immutable `ScanSnapshot` και `CollectionRoutePlan` αμέσως μετά το pure
planning, πριν επιστραφεί το plan στον executor. Πέρασε `244` collection tests.

Controlled capture:

- καθαρό restart PC/Gazebo και Pi,
- αρχικό world/robot state,
- audit monitor οπλισμένο πριν το command,
- μόνο navigation προς scan pose + 360 scan,
- artifact γράφτηκε και `idle` στάλθηκε `0.13 s` αργότερα, πριν από FollowPath,
- κανένα collection route δεν εκτελέστηκε.

Frozen evidence:

- artifact:
  `runtime/route_audit/clean_current_20260728_1315/collection-scan-39264000000.json`,
- ακριβές Pi court artifact:
  `runtime/route_audit/clean_current_20260728_1315/court_boundary.json`,
- snapshot: `10` confirmed balls,
- current captured config (`run-in 1.9`): `partial`, search `complete`,
  `6/10 covered`, `4 route_conflict`, route `46.112815 m`.

Exact deterministic replay του **ίδιου snapshot και court artifact**:

| Planner/config | Covered | Deferred | Route length |
|---|---:|---:|---:|
| current captured, run-in `1.9 m` | 6/10 | 4 route_conflict | 46.112815 m |
| μόνο run-in `1.0 m` | 9/10 | 1 route_conflict | 63.471704 m |
| ολόκληρο exact config από `d76fdc9` | 9/10 | 1 route_conflict | 63.471704 m |

Τα `collection_route_planner_v2.py`, `collection_route_global_solver.py`,
`collection_route_connector_graph.py` και `collection_route_shared_pass.py`
είναι αμετάβλητα ως προς `d76fdc9`. Το exact-config replay αποκλείει τις
follow-up/status/UI αλλαγές από το planning αποτέλεσμα.

**Regression identified:** η μη committed αλλαγή
`minimum_run_in_m: 1.0 → 1.9` μαζί με
`default_execution_profile.required_run_in_m: 1.0 → 1.9` αυξάνει τις
route-conflict απορρίψεις στο πραγματικό pristine snapshot από `1` σε `4` και
ρίχνει την κάλυψη από `9/10` σε `6/10`. Αυτή είναι αποδεδειγμένα η αλλαγή που
έσπασε την πολυ-στόχο ολοκλήρωση της διαδρομής. Το προηγούμενο synthetic A/B
δεν την αποκάλυψε επειδή δεν αναπαρήγαγε την πραγματική scan geometry.

### Δεύτερο pristine capture μετά το minimal revert

Έγινε δεύτερο καθαρό PC/Gazebo + Pi start αφού επανήλθαν **μόνο** τα δύο
run-in πεδία στο committed `1.0 m`. Το opt-in planner capture έγραψε:

- artifact:
  `runtime/route_audit/clean_reverted_20260728_1324/collection-scan-28672000000.json`,
- snapshot: `10` confirmed balls,
- live plan με run-in `1.0 m`: `partial`, search `complete`, `8/10 covered`,
  `2 route_conflict`, route `40.795885 m`.

Ο guard έστειλε `idle` `0.026 s` μετά την ανίχνευση του artifact. Ο Nav2
πρόλαβε να δεχτεί το πρώτο goal, αλλά το status έμεινε σε `progress 0.0 m`,
`speed 0.0 m/s`, χωρίς executed crossings ή retained balls. Επομένως δεν
εκτελέστηκε ουσιαστική collection route και το capture παραμένει κατάλληλο για
planner A/B.

Exact replay του **ίδιου δεύτερου snapshot**, αλλάζοντας μόνο τα δύο run-in
πεδία:

| run-in | Covered | Deferred | Route length |
|---|---:|---:|---:|
| `1.0 m` | 8/10 | 2 route_conflict | 40.795885 m |
| `1.9 m` | 5/10 | 5 route_conflict | 40.514663 m |

Το δεύτερο ανεξάρτητο pristine scan επιβεβαιώνει το regression (`-3` επιλέξιμοι
στόχοι) χωρίς να βασίζεται στην ακριβή γεωμετρία του πρώτου capture. Ταυτόχρονα
δείχνει ότι το revert **δεν εγγυάται ακόμη 10/10**: επαναφέρει τη συμπεριφορά
προς το τελευταίο baseline, αλλά παραμένουν `1–2` προϋπάρχοντα route conflicts
ανάλογα με τη scan geometry. Άρα δεν χαρακτηρίζουμε ακόμη το end-to-end
collection ως διορθωμένο.

### Root-cause separation

1. **Γιατί σταμάτησε η αποστολή:** άμεση και αποδεδειγμένη αιτία είναι το
   bounded policy του `d76fdc9`, `max_total_runs=2`, σε συνδυασμό με δύο
   `partial` plans. Μετά το δεύτερο route δεν επιτρέπεται τρίτο scan/run.
2. **Γιατί παλιότερα έγραφε completed:** ο executor μετέτρεπε κάθε clean
   sub-route completion σε mission `completed`, ακόμη και όταν το τελικό plan
   ήταν partial. Το νέο `incomplete_targets` δεν έσπασε την κίνηση· αφαίρεσε το
   false-positive completion label.
3. **Γιατί μειώθηκε η κάλυψη σε partial plans:** στο pristine frozen snapshot
   αποδόθηκε πλέον στην αύξηση run-in `1.0 → 1.9` (`9/10 → 6/10`). Το ακραίο
   μονο-στόχο αποτέλεσμα του προηγούμενου contaminated run δεν αναπαράχθηκε
   (`6/10` στο clean capture), άρα το υπόλοιπο της πτώσης ήταν state
   contamination από προηγούμενες διαδρομές.
4. **Το προηγούμενο `heading_error_exceeded` στα `-0.151 rad`:** είναι
   πραγματικό tracking abort πάνω από το pass gate `0.150 rad`. Το μεγαλύτερο
   run-in αλλάζει τη route geometry και ίσως εξέθεσε το οριακό gate, αλλά δεν
   υπάρχει ακόμη controlled A/B που να αποδεικνύει αιτιότητα.

### Επόμενο ασφαλές isolation

1. Δεν αυξάνεται το `max_total_runs` και δεν χαλαρώνει κανένα tracking gate.
2. Γίνεται μόνο το ελάχιστο revert των δύο run-in τιμών σε `1.0`.
3. Επαναλαμβάνεται pre-execution-only capture σε pristine world. Acceptance:
   ίδιο snapshot cardinality και επαναφορά τουλάχιστον `9/10 covered`, χωρίς
   FollowPath.
4. Μόνο μετά από αυτό εξετάζεται ένα controlled route execution. Η διάρκεια
   λειτουργίας των intake τροχών παραμένει ανεξάρτητη collector concern και δεν
   πρέπει να επιβληθεί μέσω planner approach geometry.

## #36 — (Φάση 0/1 opt-plan) Shadow adaptive-approach model + generator + offline replay

- **Υπόθεση:** Πρώτη ασφαλής φάση του adaptive collection-route approach: να
  εξεταστεί **offline** αν εναλλακτικά approach gates + capture-safe lateral
  offsets δίνουν πληρέστερη/μικρότερη/ευκολότερη route, **χωρίς** να αλλάξει η
  live `collect_route` ή το production tuning. Shadow-only.
- **Αλλαγή (μόνο νέα αρχεία, κανένα υπάρχον tracked file δεν άλλαξε):**
  - `collection_capture_geometry.py` — pure, immutable `CaptureGeometry` στο
    `base_footprint` frame: πέντε capture planes (intake mouth `x≈0.876`, entry
    beam `x=0.720`, roller nip `x=0.540`, confirmed beam `x=0.350`, retention
    `x=0.420`). **Και τα πέντε είναι `CONFIGURED`** (δηλωμένα στο URDF/xacro/env,
    ΟΧΙ physical measurement)· το `MEASURED` κρατιέται μόνο για τιμές με πραγματικό
    measurement/trial artifact, που δεν υπάρχει ακόμη. Το
    `required_pre_contact_straight_m` (και άρα το minimum alignment corridor)
    είναι **explicit uncalibrated required input** (όχι κρυφό default) —
    χρειάζεται intake trials. Validation + deterministic round-trip. Το URDF
    comment `0.590` για το nip είναι documentation drift· η αληθινή injected τιμή
    είναι `0.540` (`INTAKE_NIP_X_M` σε run_ubuntu.sh/compose/generator, όλα συνεπή).
  - `collection_adaptive_approach.py` — pure bounded generator: για κάθε valid
    3A heading κρατά το baseline candidate (πάντα) + bounded {gates}×{lateral
    offsets}, με υποχρεωτικό ευθύ final alignment corridor (καμία καμπύλη μέσα
    στο calibrated corridor), lateral ≤ `effective_capture_half_width_m`, ίδιο
    run-out, και το ίδιο υπάρχον swept collision check. Deterministic Pareto
    filter (connector-lower-bound, pass length, approach length, clearance,
    capture margin) με per-heading/per-ball caps· baseline pinned· budget
    exhaustion ≠ unreachable. `shadow_global_solve` τρέχει τα **υπάρχοντα**
    graph/solver μόνο μέσω νέου API, με baseline-preserving bounding.
  - `scripts/sim_debug/collection_route_adaptive_replay.py` — offline CLI:
    audit artifact + court_boundary.json → machine-readable JSON (baseline vs
    adaptive candidate totals, Pareto-pruned totals, connector rejection
    histogram, coverage, deferred/unreachable reasons, length breakdown,
    duration, passes, balls/pass, wall time, budget flags). Read-only.
- **Frozen A/B (χωρίς κίνηση robot, χωρίς ωραιοποίηση):**
  - `clean_reverted` (run-in 1.0): baseline **8/10, 40.796 m**· shadow adaptive
    **8/10, 40.796 m** — **καμία βελτίωση**.
  - `clean_current` (run-in 1.9): baseline **6/10, 46.113 m**· shadow adaptive
    **6/10, 46.113 m** — **καμία βελτίωση**.
  - `baseline_shadow_solve` == `plan_collection_route` **byte-identical** και
    στα δύο· ο edge histogram ταιριάζει με το opt-plan (112.896 edges,
    100.981 turning / 9.741 length / 2.174 accepted).
  - Παρατήρηση: φλόμπασμα του adaptive pool μέσα στο υπάρχον
    `_bounded_candidates` στο cap 200 **χωρίς** baseline preservation ρίχνει
    coverage σε 1/10 (bounding artifact, όχι πραγματική adaptive συμπεριφορά) —
    γι' αυτό το shadow solve διατηρεί ρητά τα baselines.
- **Παρατήρηση (ΟΧΙ αιτιακό συμπέρασμα):** στα δύο frozen scans το shadow
  adaptive δεν έδειξε βελτίωση coverage/length. **ΔΕΝ** εξάγεται συμπέρασμα ότι
  τα approach gates αδυνατούν να λύσουν τα route_conflict ή ότι ο connector model
  είναι η αποδεδειγμένη μοναδική αιτία: και στα δύο η adaptive σύγκριση είναι
  **inconclusive** επειδή εξαντλήθηκαν candidate-generation/global-cap budgets ΚΑΙ
  ο production planner εκθέτει μόνο **combined** budget status (δεν διαχωρίζεται
  independent DFS/search exhaustion — βλ. Review fixes round 2). Οι raw A/B
  αριθμοί κρατιούνται ως observation, χωριστά από αιτιότητα.

### Review fixes (A–F, 2026-07-28)

- **A — Physical ball positions σε adaptive shared passes:** νέος shadow-only
  `generate_adaptive_shared_passes` στο `collection_adaptive_approach.py`.
  Κρατά ΠΑΝΤΑ τα φυσικά `crossing_positions[0]` κάθε member· τα shifted
  centrelines χρησιμοποιούνται μόνο ως **προτεινόμενα** centreline offsets· η
  lateral feasibility ελέγχεται στις **φυσικές** θέσεις με το ατομικό
  `effective_capture_half_width` κάθε member· ένα shared candidate ανά
  combination στο centreline που μεγιστοποιεί το min capture margin· υπάρχον
  swept collision check· μη-εκπροσωπήσιμα/ανέφικτα combinations απορρίπτονται με
  ρητό reason (κανένα fabricated geometry). Η production `generate_shared_passes`
  **δεν** άλλαξε· το `baseline_shadow_solve` την κρατά → baseline byte-identical.
- **B — Calibration gate fail-loud:** το `generate_adaptive_candidates` κάνει
  raise `AdaptiveApproachError` (με λίστα uncalibrated fields) σε uncalibrated
  geometry· το `shadow_global_solve` έχει belt-and-braces guard. Το CLI αφαίρεσε
  το σιωπηλό `required_pre_contact_m=0.0 + uncalibrated`: adaptive mode απαιτεί
  ρητό calibrated/configured input, αλλιώς exit **2** με machine-readable error·
  νέο `--baseline-only` mode εμφανίζει uncalibrated geometry χωρίς adaptive solve.
- **C — Budget evidence:** το report ξεχωρίζει τα **independently known**
  budgets (candidate-generation, shared-pass, global candidate cap) από το
  **combined** production planner status (βλ. round 2 / Fix A). Νέα πεδία
  `comparison_conclusive` / `comparison_limitations` / `comparison_conclusion`.
  Η σύγκριση είναι conclusive μόνο όταν **κανένα** relevant budget δεν εξαντλήθηκε.
  Η raw παρατήρηση είναι πλέον ανεξάρτητη structured ταξινόμηση
  (`coverage_delta`, `length_delta_m`, `observed_result`): coverage πρώτα και,
  μόνο σε ίσο coverage, length με deterministic epsilon `1e-6 m`. Το conclusion
  παράγεται από αυτή την ταξινόμηση και, όταν είναι inconclusive, παραπέμπει στα
  `comparison_limitations` χωρίς να εφευρίσκει αιτίες exhaustion.
- **D — Candidate counters:** αντικαταστάθηκαν τα ασαφή πεδία με
  `raw_baseline/raw_adaptive_extra/raw_candidate_count` και
  `pareto_kept_baseline/adaptive/total` + `pareto_pruned_count`, με ελεγμένες
  ακριβείς αριθμητικές ταυτότητες (`raw = baseline+extra`,
  `kept_total = kept_b+kept_a`, `pruned = raw - kept_total`).
- **E — Provenance:** entry beam & confirmed beam άλλαξαν `MEASURED→CONFIGURED`·
  όλες οι repo τιμές είναι CONFIGURED (δηλωμένες, όχι μετρημένες).
- **F — Docs:** αυτό το entry (καμία άλλη ιστορική εγγραφή δεν αγγίχτηκε).
- **Uncalibrated / χρειάζεται φυσική επιβεβαίωση:** intake `required
  pre-contact straight` (και άρα minimum alignment corridor) + επιλογή capture
  reference plane (mouth/entry-beam/nip) — από intake trials, όχι από τον χάρτη.

### Review fixes round 2 (A–D, 2026-07-28)

- **A — Ειλικρινές search-exhaustion reporting:** ο production solver υπολογίζει
  `planning_search_status = search_exhausted OR candidate_budget_exhausted`
  (`collection_route_global_solver.py`), άρα δεν διαχωρίζεται DFS-search από
  candidate-cap exhaustion. Το `ShadowSolveResult.search_status` **μετονομάστηκε**
  σε `combined_planner_budget_status`. Το report εκθέτει πλέον
  `search_exhaustion_independently_known=false`, `search_expansions=null` με note
  ότι ο solver εκθέτει μόνο combined status. Το CLI **δεν** εκπέμπει πλέον
  `adaptive_search_budget_exhausted`/`baseline_search_budget_exhausted`
  (παραπλανητικά)· αντ' αυτού non-complete combined status → limitation
  `*_combined_planner_budget_status_budget_exhausted`. Καμία αλλαγή/reimplementation
  του production solver.
- **B — Πραγματικός asymmetric-width regression test:** το παλιό test περνούσε
  ένα covariance → ίδιο width. Νέα tests χτίζουν single-ball `FunnelPassCandidate`
  με ρητά widths **0.02/0.05** (το default mechanical config κόβει το derived
  width ~0.047, άρα δεν βγαίνει 0.05 από covariance): separation 0.06 →
  **feasible/accepted** (physical positions preserved, κάθε ball εντός του δικού
  του width)· separation 0.08 → **rejected `lateral_exceeds_capture`**. Χρειάστηκε
  επίσης να προστεθεί στον generator η **width-aware feasibility-interval centre**
  (Chebyshev center) ως centreline option: για ασύμμετρα widths το feasible
  centreline είναι μετατοπισμένο από το physical midpoint, οπότε ο παλιός
  midpoint-only έχανε feasible shared passes.
- **C — Reproducible frozen replay:** το report περιλαμβάνει `analysis_inputs`
  με ΟΛΕΣ τις algorithm-affecting παραμέτρους (gates, offsets, caps, capture
  reference, pre-contact, provenance, shadow-solve flag, `maximum_candidate_count`,
  `minimum_run_in_m`) + `assumptions` που δηλώνει ρητά ότι
  `required_pre_contact_straight_m=0.0, provenance=configured` είναι **OFFLINE
  ANALYSIS ASSUMPTION** για αυτές τις frozen συγκρίσεις — **ΟΧΙ** intake
  measurement και **ΟΧΙ** production calibration· η πραγματική τιμή θέλει intake
  trials. Τα paths μπαίνουν στο report μόνο για provenance (εκτός deterministic
  comparison).

#### Ακριβείς εντολές frozen replay (τα documented counters ισούνται με το JSON)

```
# clean_reverted (run-in 1.0):
python3 scripts/sim_debug/collection_route_adaptive_replay.py \
  --audit-artifact runtime/route_audit/clean_reverted_20260728_1324/collection-scan-28672000000.json \
  --court-boundary runtime/route_audit/clean_reverted_20260728_1324/court_boundary.json \
  --additional-gates 1.4,1.9 --lateral-offsets 0.0,0.02,-0.02 \
  --max-per-heading 6 --max-per-ball 64 --capture-reference-plane intake_mouth_contact \
  --required-pre-contact-m 0.0 --pre-contact-provenance configured --shadow-solve

# clean_current (run-in 1.9):
python3 scripts/sim_debug/collection_route_adaptive_replay.py \
  --audit-artifact runtime/route_audit/clean_current_20260728_1315/collection-scan-39264000000.json \
  --court-boundary runtime/route_audit/clean_current_20260728_1315/court_boundary.json \
  --additional-gates 2.4,2.9 --lateral-offsets 0.0,0.02,-0.02 \
  --max-per-heading 6 --max-per-ball 64 --capture-reference-plane intake_mouth_contact \
  --required-pre-contact-m 0.0 --pre-contact-provenance configured --shadow-solve
```

| Artifact | baseline cov/len | adaptive cov/len | observed result | raw (b+e) | kept (b+a) | pruned | combined status | comparison |
|---|---|---|---|---|---|---|---|---|
| clean_reverted (1.0) | 8/10, 40.795885 m | 8/10, 40.795885 m | `unchanged` | 160+1235=1395 | 160+407=567 | 828 | baseline complete / adaptive budget_exhausted | inconclusive |
| clean_current (1.9) | 6/10, 46.112815 m | 6/10, 46.112815 m | `unchanged` | 157+1100=1257 | 157+337=494 | 763 | baseline complete / adaptive budget_exhausted | inconclusive |

`baseline_shadow_solve == plan_collection_route` byte-identical και στα δύο (edge
histogram: 112.896 edges, 100.981 turning / 9.741 length / 2.174 accepted).
Limitations (και στα δύο): `adaptive_candidate_generation_budget_exhausted`,
`adaptive_global_candidate_cap_exhausted`,
`adaptive_combined_planner_budget_status_budget_exhausted` (ΟΧΙ ανεξάρτητο
search-exhaustion). `search_exhaustion_independently_known=false`. Και στα δύο
`coverage_delta=0`, `length_delta_m=0.0`, `observed_result=unchanged`,
`comparison_conclusive=false`. Μήνυμα: «No improvement was observed, but the
comparison is inconclusive; see comparison_limitations. Raw observations are
reported separately and no causal conclusion may be drawn.»

- **Tests:** 58 νέα pure tests (τα απαιτούμενα reporting regressions + fail-loud uncalibrated,
  exact-count invariants, adaptive shared-pass incl. asymmetric-width
  feasible/infeasible, honest combined-status/no-independent-search-exhaustion,
  conclusive-when-clean, analysis_inputs, CLI baseline-only/exit-2). Relevant
  `tests/test_collection*.py` regression: **286 passed**. `git diff --check`
  καθαρό.
- **Status:** SHADOW/OFFLINE ΕΤΟΙΜΟ· καμία live αλλαγή· ΕΚΚΡΕΜΕΙ review.

## #37 — Checkpoint πριν το σταδιακό PC+Pi qualification run (2026-08-02)

**Context:** το platform/network στάδιο (ONNX thread pools, sensor QoS, 50 Hz
sim-clock relay, UI snapshot rate limiting, shutdown lifecycle, symmetric domain
isolation) τεκμηριώνεται πλήρως στο
`docs/network/network-stabilization-plan-el.md`. Εδώ καταγράφονται **μόνο** οι
αλλαγές που αγγίζουν τον collect_route μηχανισμό και μπαίνουν στο ίδιο
checkpoint. **Καμία από αυτές δεν έχει ακόμη live επικύρωση** — το σταδιακό
PC+Pi run (PC-only → +Pi → +UI → route → δεύτερο scan/route → clean shutdown)
είναι το επόμενο βήμα.

- **`context_already_consumed` στο 2ο run χωρίς restart.** Ο Nav2 controller
  κρατά το execution-context lifecycle και επιβιώνει του executor: μετά από
  terminal πηγαίνει `kConsumed` και μόνο `reset()` το επαναφέρει σε `kIdle`. Ένα
  νέο route φτιάχνει νέο transport, οπότε οι τοπικοί counters δεν μπορούν να
  ξέρουν ότι ο controller κρατά consumed context από προηγούμενο run. **Fix:**
  το `load_sender` (`collection_executor_node_factory.py`) καλεί πάντα
  `reset_collection_execution_context` **πριν** από κάθε load, και κάνει
  fail-loud αν το reset απορριφθεί (το C++ `reset()` απορρίπτει σε
  `kExecuting`/`kSafetyPaused`, άρα δεν μπορεί να σκοτώσει ενεργό run).
- **Scan FSM: επιστροφή στο start yaw.** Το `ScanRotationFsm` θεωρούνταν
  complete μόλις έπαιρνε το τελευταίο διακριτό sample, αφήνοντας το robot σε yaw
  διαφορετικό από αυτό που παγώνει στο planner snapshot. Τώρα απαιτείται
  επιστροφή στο `start_yaw` για να κλείσει η στροφή γεωμετρικά· η επιστροφή
  **δεν** εκπέμπει διπλό sample.
- **`EMPTY_NO_FEASIBLE_TARGETS` δεν είναι άδειο γήπεδο.** Ο executor το
  αντιστοιχούσε σε `COMPLETED_NO_TARGETS`, δηλαδή ίδιο outcome με «καμία μπάλα».
  Τώρα πηγαίνει σε `INCOMPLETE_TARGETS`: οι στόχοι παρατηρήθηκαν και
  ταξινομήθηκαν αλλά κανένας δεν έχει εκτελέσιμη διαδρομή. Το UI εμφανίζει
  ανάλυση planner blockers (`deferred`/`unreachable` ανά `reason_code`) αντί για
  σκέτο «0 unresolved».
- **Heading-error entry grace σε capture segments (C++ tracking core).** Ένα
  connector μπορεί να παραδώσει σε ευθύ capture pass μικρό, παροδικό heading
  error παρότι και οι δύο frozen διαδρομές είναι tangent-continuous. Το gate
  χαλαρώνει **μόνο** μέσα στο ήδη ρυθμισμένο `required_entry_m` του segment και
  **μόνο** πριν το πρώτο crossing (`min(progress_start + required_entry_m,
  next_crossing - ε)`). Lateral tube, curvature, reverse και standalone-rotate
  guards παραμένουν ενεργά σε όλο το διάστημα. Το `required_entry_m` προστέθηκε
  στο `TrackingExecutionProfile` + `valid_profile` (ήταν ήδη στο wire contract,
  απλώς δεν διαβαζόταν).
- **Execution-truth telemetry (για το ανοιχτό spatial offset, #32/#35).** Το
  route audit artifact και το `robot_status` κουβαλούν πλέον
  `execution_frame_diagnostics` + `execution_truth_snapshot` (sim ball poses,
  sim true pose, believed map pose, pose/yaw drift τη στιγμή της εκτέλεσης).
  Καθαρά διαγνωστικά — δεν αγγίζουν plan ή geometry.
- **Console BallMap: απόκρυψη φυσικά επιβεβαιωμένων μπαλών.** Operator-only
  προβολή· το immutable snapshot και τα planner results μένουν ανέπαφα. Η
  αντιστοίχιση απαιτεί κοντινή mapped μπάλα (`min(1.0,
  max_merge_distance_m)`), ώστε ένα ασθενές association να μη σβήνει άσχετο
  στόχο.
- **Stale Jazzy test.** Το `test_collection_controller_server_isolated.launch.py`
  άκουγε `Twist` στο `/cmd_vel` ενώ το `nav2_params.yaml` έχει
  `enable_stamped_cmd_vel: true` (Jazzy) → ο harness δεν έπαιρνε ποτέ εντολές
  και 3 tests έπεφταν. Fix: `TwistStamped` + unwrap σε `msg.twist`.
- **Tests:** 471 pure pytest (459 + 12 console, χωριστά process λόγω του διπλού
  `tennis_robot` package namespace) και 37 C++/launch tests με **2 failures**.
  Και τα δύο ανήκουν στο ίδιο **pre-existing** stale test
  (`test_collection_is_forward_only_and_survey_rpp_needs_no_context` + το
  cascade στο tearDown του): με το robot ακριβώς στο terminal του 2-pose path, η
  RPP κάνει `Resulting plan has 0 poses in it` πριν προλάβει ο goal checker.
  Είναι υπόλειμμα της Humble→Jazzy μετάβασης, **όχι** regression αυτού του
  checkpoint (το αρχείο δεν έχει αλλάξει από το `83f33af`).

### Live distributed επικύρωση του #37 (2026-08-03)

Σταδιακό PC+Pi run πάνω από **gigabit Ethernet** (και οι δύο μηχανές ενσύρματα·
το PC έτρεχε προηγουμένως σε WiFi και όλο το DDS traffic μοιραζόταν airtime με
το internet — αυτό εξηγεί γιατί ~3 Mbit/s application traffic έριχνε το internet
από 330 σε 7.5 Mbps). Νέο Map Court από το Pi πρώτα, ώστε όλα τα δεδομένα να
είναι στο τρέχον Pi slam frame (`OK`, 136.8 s, 1.500 occupancy points, 4
obstacles, robot επέστρεψε στο start pose).

- **Context lifecycle: ΕΠΙΚΥΡΩΘΗΚΕ.** Ένα `collect_route` mission έκανε scan →
  plan(3) → execute → **δεύτερο scan** → plan(run-2) → execute → terminal, με
  **δύο** διαδοχικά execution context loads. **0** `context_already_consumed`,
  **0** reset rejections στο Pi log. Το `aborted_scan` του δεύτερου scan δεν
  επανεμφανίστηκε.
- **Φυσική συλλογή: 4/4 retained.** `basket_retained=4`, `confirmed=4`,
  `beam_credits=4`, `crossed_unconfirmed=0`. Η επιβεβαίωση **δεν** είναι
  κυκλική: το `/ball/collected` εκπέμπεται μόνο όταν ο
  `_sim_retention_tracker` δει τη μπάλα στη ζώνη `bin` σε **robot-frame ground
  truth** συντεταγμένες για το απαιτούμενο dwell (`controller_node.py:2248`),
  ανεξάρτητα από το beam. Οι μπάλες `ball_04/05/06/13` έφυγαν από το `/sim/balls`
  ως αποτέλεσμα αυτού, όχι ως αιτία του.
- **Το spatial offset ΥΠΑΡΧΕΙ ακόμη και τώρα μετρήθηκε:** το νέο
  `execution_truth_snapshot` έδωσε **`pose_drift_m = 0.429`**,
  `yaw_drift_rad = 0.0245`, με `pose_frame_offset` x=-8.643 m. Τα per-crossing
  `lateral_error_m ≈ 0` δείχνουν ότι το robot ακολουθεί **τέλεια το δικό του
  μετατοπισμένο plan** — η ίδια υπογραφή «σωστό plan, μετατοπισμένη εκτέλεση»
  των #32/#35, τώρα με αριθμό αντί για εντύπωση. Με capture corridor ~0.15 m,
  drift 0.43 m σημαίνει ότι κάθε μπάλα εκτός των στοχευμένων προσπερνιέται.
- **Τερματισμός `incomplete_targets` — σωστός.** Στο run-2 τρεις στόχοι
  απορρίφθηκαν ως `unreachable / turn_radius` και ένας επιλέχθηκε. Δηλαδή στο
  γήπεδο έμειναν μπάλες που ο planner δεν μπορούσε να φτάσει, όχι μπάλες που
  χάθηκαν στην εκτέλεση.
- **Δεν επικυρώθηκε ακόμη:** δεύτερο **mission** (νέα εντολή `collect_route`
  μετά τον τερματισμό του πρώτου) — το run αυτό κάλυψε δύο scan/plan/execute
  κύκλους **μέσα** στο ίδιο mission. Ο Gazebo έσκασε πριν προλάβουμε, σε
  **GUI bug άσχετο με τη στοίβα μας**: `libSelectEntities.so` →
  `SelectEntities::eventFilter` → `HandleEntitySelection` → `HighlightNode` →
  `OgreVisual::LocalBoundingBox` → Ogre assert → abort, δηλαδή επιλογή
  οντότητας στο viewport. Ο server έφυγε μαζί με το GUI (ίδιο launch entry).

## #38 — Root cause των διαδοχικών routes: το finalize είναι fire-and-forget

**Πλαίσιο:** δεύτερος σταδιακός PC+Pi κύκλος (2026-08-03), φρέσκο Map Court,
`collect_route` mission. Πρώτος κύκλος route: plan 7, `route_completed`, 2
retained. Δεύτερος κύκλος: scan OK, plan 12, και **20 ms** μετά την είσοδο σε
`executing_route` → `aborted_tracking / path_failed`. Στο Pi log, μία γραμμή:

```
[controller_node] ERROR: collection controller reset rejected: invalid_lifecycle
```

**Αλυσίδα αιτιότητας (τεκμηριωμένη):**

1. `collection_path_follower_port.py:279` — όταν το Nav2 goal πετύχει, ο κώδικας
   **σκοπεύει** να ελέγξει το finalize:
   `if not self._finalize(): return self._fail(PATH_FAILED, "collection
   controller rejected terminal finalize")`.
2. `collection_executor_node_factory.py:542` — το `finalize_sender` κάνει
   `call_async(...)` και **`return True` ανεξαρτήτως απάντησης**. Άρα ο έλεγχος
   του βήματος 1 είναι **νεκρός κώδικας**. Μπήκε ως workaround για το Jazzy
   «Executor is already spinning» (το σχόλιο το δηλώνει)· στο Humble ο nested
   spin περνούσε και ο έλεγχος **δούλευε**.
3. `collection_execution_context_contract.cpp:142` — `finalize()` με
   `action_outcome == SUCCEEDED` απαιτεί `terminal_ready`, αλλιώς
   `kTerminalNotReached`. Το Nav2 δηλώνει επιτυχία μέσω goal checker
   (`collection_goal_checker.xy_goal_tolerance: 0.30`), που **δεν ταυτίζεται**
   με το `terminal_ready_` του collection controller. Όταν αποκλίνουν, το
   finalize απορρίπτεται και το lifecycle **μένει `kExecuting`**.
4. Ο executor δεν το μαθαίνει ποτέ· αναφέρει `route_completed` και πάει για
   rescan. Ο controller έχει κολλήσει σε `kExecuting`.
5. Το επόμενο route: reset-before-load → `invalid_lifecycle` (το `reset()`
   σωστά απορρίπτει σε `kExecuting`) → το load δεν στέλνεται καν →
   `path_failed`.

**Γιατί είναι διαλείπον:** εξαρτάται αποκλειστικά από το αν είχε τεθεί το
`terminal_ready` όταν ο goal checker δήλωσε επιτυχία. Στον κύκλο της ίδιας
βραδιάς με 4/4 captures το finalize πέρασε και το δεύτερο load δούλεψε· εδώ όχι.

**Σχέση με το #37:** το reset-before-load είναι σωστό αλλά θεραπεύει το σύμπτωμα
μόνο όταν το προηγούμενο context έχει όντως τερματίσει. Το `context_already_
consumed` (παλιό σύμπτωμα) και το `invalid_lifecycle` (νέο) είναι **δύο όψεις
του ίδιου defect**: κανείς δεν ελέγχει την απάντηση του finalize.

**Fix (ΔΕΝ έχει υλοποιηθεί):** να γίνει το finalize ack-aware όπως ήδη είναι το
load/reset — δηλαδή future που ελέγχεται σε **επόμενες κλήσεις του timer**, όχι
nested spin (`load_outcome_provider` είναι το υπάρχον πρότυπο). Σε απόρριψη:
fail-loud, και είτε retry με `CANCELED` outcome είτε ρητό escalation, ώστε να
μην αφήνεται ποτέ ο controller σε `kExecuting`.

**Δευτερεύοντα ευρήματα του ίδιου κύκλου:**

- Το `ros2 launch` **δεν** καταρρέει όταν πεθάνει ο Gazebo: έμειναν ζωντανά
  launch + domain_bridge + perception_node με νεκρό sim.
- Το Gazebo GUI κρασάρει σε **επιλογή οντότητας** στο viewport
  (`libSelectEntities.so` → `HandleEntitySelection` → `HighlightNode` →
  `OgreVisual::LocalBoundingBox` → Ogre assert). Άσχετο με τη στοίβα μας.
- **Τα map frames δεν επαναλαμβάνονται μεταξύ restarts:** `survey_start_pose`
  yaw ήταν 0.2662 rad στο ένα survey και 0.0062 στο επόμενο (~15° διαφορά).
  Άρα επαναχρησιμοποίηση `court_boundary.json` μετά από restart σε **mapping**
  mode είναι άκυρη. Ο σωστός δρόμος —και η απαίτηση «όχι survey κάθε φορά»—
  είναι `SLAM_MODE=localization` πάνω στο σωσμένο `map_artifact`· ο μηχανισμός
  **υπάρχει ήδη** (`run_pi.sh:10`, `slam_localization.launch.py`,
  `slam_toolbox.yaml mode: mapping | localization`) και δεν έχει δοκιμαστεί
  distributed. **ΕΠΟΜΕΝΟ ΤΕΣΤ.**
- `pose_drift_m` = **0.427** σε αυτόν τον κύκλο (0.429 στον προηγούμενο):
  σταθερό ~43 cm, με per-crossing `lateral_error_m ≈ 0`. Το spatial offset είναι
  πλέον μετρήσιμο και επαναλήψιμο, όχι εντύπωση.

### #38 — Fix υλοποιήθηκε (2026-08-03)

Το finalize έγινε **ack-driven**, με το πρότυπο του `load_outcome_provider`
(future που ελέγχεται σε επόμενα timer ticks, **ποτέ** nested spin).

- **Transport** (`collection_executor_node_factory.py`): το `finalize_sender`
  κρατά πλέον το future (`self.finalize_future`)· νέο `finalize_outcome_provider`
  επιστρέφει `None` όσο εκκρεμεί, αλλιώς `("accepted", None)` ή
  `("rejected", detail)` με το `detail` και το `rejection_code` του controller,
  και κάνει log το σφάλμα. Το `load_sender` μηδενίζει το `finalize_future`, ώστε
  ack προηγούμενου route να μη διαβαστεί ποτέ ως απάντηση του τρέχοντος.
- **Port** (`collection_path_follower_port.py`): νέα φάση `finalizing`. Το
  `_tick_executing` δεν δηλώνει πια COMPLETED μόνο και μόνο επειδή το Nav2 είπε
  `succeeded` — καλεί `_begin_finalize()` (αποστολή **ακριβώς μία φορά**) και
  περνά στο `_tick_finalizing()`. Εκεί:
  - ack εκκρεμές → παραμένει RUNNING με το τελευταίο παρατηρημένο progress·
    πάνω από `finalize_ack_timeout_s` (default **5 s**) → FAILED
    «finalize ack timed out».
  - ack **rejected** → FAILED με το **πραγματικό rejection detail** του
    controller (π.χ. `terminal_not_reached`), όχι γενικό label.
  - ack **accepted** → **δεν αρκεί**: το lifecycle πρέπει να φύγει από
    `EXECUTING`/`SAFETY_PAUSED` (δηλαδή ο controller να απελευθερώσει όντως το
    context). Αν επιμείνει πέρα από το timeout → FAILED «still holds the
    context». Ο έλεγχος είναι χρονικά ανεκτικός επίτηδες, γιατί το state
    δημοσιεύεται ασύγχρονα μετά το ack — αυστηρός άμεσος έλεγχος θα έκανε race.
  - malformed outcome → `PathFollowerPortError` (wiring error, όχι σιωπηλή
    επιτυχία).
- **Tests:** 5 νέα pure tests (pending ack κρατά RUNNING χωρίς επαναποστολή,
  ack timeout, accepted-αλλά-κολλημένο context, accepted που ολοκληρώνεται μόλις
  απελευθερωθεί, malformed outcome) + ενημέρωση του υπάρχοντος rejection test
  ώστε να επιβεβαιώνει ότι περνά το rejection reason. **476 pure pytest**
  (464 + 12 console σε χωριστή διεργασία), ROS build OK, `git diff --check`
  καθαρό.
- **ΕΚΚΡΕΜΕΙ live επαλήθευση:** δύο συνεχόμενα routes χωρίς restart (βήμα 3 της
  συμφωνημένης σειράς). Το fix είναι επικυρωμένο μόνο σε pure tests.

## #39 — Το stale Jazzy launch test: η RPP δεν μπορεί να ακολουθήσει με στατικό robot

**Πλαίσιο:** το `test_collection_controller_server_isolated.launch.py` είχε 2
failures (ένα test + το cascade στο tearDown του) από τη μετάβαση σε Jazzy. Το
πρώτο μισό διορθώθηκε στο #37 (`TwistStamped` αντί `Twist`, γιατί το
`nav2_params.yaml` έχει `enable_stamped_cmd_vel: true`). Έμενε το survey σκέλος
του `test_collection_is_forward_only_and_survey_rpp_needs_no_context`, που
abort-άριζε με `Resulting plan has 0 poses in it` (πηγή:
`libnav2_regulated_pure_pursuit_controller.so`).

**Τι αποκλείστηκε με μετρήσεις** (standalone launch probes με τον ίδιο
controller_server και το ίδιο `nav2_params.yaml`):

- **ΟΧΙ** το `use_sim_time` του nested `local_costmap`: τα logs του τυπώνουν
  wall-clock timestamps, άρα δεν τρέχει σε sim time (η αρχική υπόθεση ήταν λάθος).
- **ΟΧΙ** το προηγούμενο collection goal: αναπαράγεται σε test που στέλνει
  **μόνο** το survey goal.
- **ΟΧΙ** το per-pose `header.frame_id` ούτε τα `header.stamp`: δοκιμάστηκαν και
  οι τέσσερις συνδυασμοί, όλοι αποτυγχάνουν εξίσου.
- **ΝΑΙ** η θέση του robot ως προς το plan. Ντετερμινιστικό: με τον harness
  παρκαρισμένο στο **τέρμα** (x=4.0, δηλαδή η τελευταία pose του plan) κάθε goal
  σκάει με «0 poses»· με το ίδιο ακριβώς setup στην **αρχή** (x=0.0) τρέχουν όλα
  κανονικά (60-80 εντολές ταχύτητας, κανένα σφάλμα). Η μόνη διαφορά μεταξύ των
  δύο probes ήταν το αρχικό `cls.pose_x`.

**Η ακριβής εσωτερική συνθήκη της RPP δεν καρφώθηκε.** Οι αριθμοί δεν κλείνουν
με καμία εκδοχή του `max_robot_pose_search_dist` / `getCostmapMaxExtent()` που
δοκιμάστηκε black-box (π.χ. στο sweep probe το x=4.0 **δούλευε** όταν το robot
είχε ξεκινήσει από 0.0 και ανέβει σταδιακά). Καταγράφεται ως παρατήρηση, όχι ως
εξήγηση.

**Το πραγματικό πρόβλημα είναι η προδιαγραφή του test:** ο harness δημοσιεύει
σταθερή pose και **δεν κινείται ποτέ**, ενώ η RPP κλαδεύει το plan καθώς το
robot προχωρά. Το να της ζητάς να «φτάσει» σε goal που το robot ήδη κατέχει, με
robot χωρίς δυναμική, δεν είναι σενάριο που έχει νόημα να επιβεβαιώνεται.

**Fix:** το survey σκέλος τρέχει πλέον με το robot στην **αρχή** του plan και
επιβεβαιώνει το συμβόλαιο που δηλώνει το ίδιο το όνομα του test — ότι το
`FollowPath` (RPP) **δέχεται το goal και υπολογίζει εντολές χωρίς κανένα
collection execution context** — και ότι δεν τερματίζει πρόωρα· μετά το ακυρώνει
ρητά. Το `assertEqual(status, 4)` αφαιρέθηκε: απαιτούσε robot που κινείται.
Η αντιδιαστολή με το `test_missing_context_and_hash_mismatch_do_not_fallback_to_rpp`
(που επιβεβαιώνει ότι το **collection** controller ΑΠΟΡΡΙΠΤΕΙ χωρίς context)
διατηρείται ακέραιη.

**Αποτέλεσμα:** `37 tests, 0 errors, 0 failures, 2 skipped`, σταθερά σε **4
διαδοχικές εκτελέσεις** (το bug ήταν χρονικά/κατάστασης εξαρτημένο, οπότε ένα
πράσινο run δεν θα αρκούσε).

## #40 — Off-route ανακάλυψη: τα μπαλάκια που βλέπει οδηγώντας δεν χάνονται πια

**Το λογικό λάθος (εντοπισμός χρήστη, επαληθευμένο στον κώδικα):** το
`forward_frame` — ο **μόνος** δρόμος με τον οποίο μια ανίχνευση έμπαινε στο
snapshot του planner — καλούνταν αποκλειστικά μέσα στο `ScanSessionDriver.result`,
δηλαδή μόνο σε κατάσταση `SCANNING` και μόνο στα 18 διακριτά headings. Στο
`EXECUTING_ROUTE` δεν προωθούνταν **τίποτα**: ό,τι έβλεπε το robot οδηγώντας
κατέληγε μόνο στο operator `BallMap` του controller_node, που είναι για την
οθόνη. Δύο ακόμη σημεία έκαναν την απώλεια μόνιμη: κάθε scan κάνει `fsm.reset()`
+ `session.start()` (καθαρό snapshot, τίποτα δεν μεταφέρεται), και το rescan
καλεί το **ίδιο** `_begin_navigation()` με τον **ίδιο** σταθερό `scan_pose`.
Άρα μπάλα που δεν φαίνεται από το T ήταν αόρατη για πάντα, όσες φορές κι αν
περνούσε δίπλα της το robot.

**Λύση (σχεδίαση χρήστη):** δεν αγγίζουμε τη λογική του 360. Κατά την εκτέλεση
της διαδρομής χτίζεται **δεύτερη, ανεξάρτητη λίστα** με ό,τι ανακαλύπτεται
εκτός διαδρομής· στο τέλος της διαδρομής το follow-up pass σχεδιάζεται από αυτήν.
Το παγωμένο plan που εκτελείται δεν επηρεάζεται ποτέ, οπότε μια θορυβώδης
θέση-εν-κινήσει το χειρότερο που κάνει είναι να χάσει τη δική της σύλληψη.

**Δεν ξαναγράφτηκε καμία επικύρωση.** Ο `CollectionSnapshotRuntimeAdapter` και ο
`ScanSnapshotBuilder` είναι αγνωστικοί ως προς το τι είναι «βήμα»: το `add()`
δέχεται `scan_step_id`. Στο 360 τα βήματα είναι headings· εδώ είναι
**viewpoints** — νέο βήμα κάθε `drive_viewpoint_spacing_m` (0.75 m) διαδρομής.
Έτσι ο κανόνας `min_distinct_scan_steps: 2` εξακολουθεί να απαιτεί δύο
διαφορετικά σημεία θέασης, που **εν κινήσει είναι αυστηρότερος**: βάση
τριγωνισμού μέτρων αντί για 20 μοίρες επί τόπου. Ίδια ισχύουν το φίλτρο μισού
γηπέδου, ο έλεγχος covariance και το `perception_spatial_validation`.

**Το coverage gate.** Το coverage μετριέται από τα βήματα που παρήγαγαν
αποδεκτή παρατήρηση (`collection_scan_snapshot.py:141`). Αν δηλώναμε ως
αναμενόμενα όλα τα viewpoints της διαδρομής, όσα πέρασε χωρίς να δει τίποτα θα
έριχναν το ποσοστό κάτω από το κατώφλι. Τα αναμενόμενα βήματα παράγονται
επομένως **από τις ίδιες τις παρατηρήσεις** — μια διαδρομή δεν έχει έννοια
«κάλυψης τομέα» και δεν πρέπει να διαβάζεται ως τρύπα σε σάρωση.

**Φραγή: ένας κύκλος (απόφαση χρήστη).** Το off-route pass καταλαμβάνει την
υπάρχουσα θέση του recovery pass και καταναλώνει τον **ίδιο** προϋπολογισμό
`follow_up.max_total_runs: 2`. Αν η λίστα έχει στόχους → planning **από την
τρέχουσα θέση**, χωρίς επιστροφή στο T και χωρίς δεύτερο 360. Αν είναι κενή →
πέφτει πίσω στο σημερινό rescan. Ποτέ μετά από abort (ίδια προϋπόθεση με το
`_can_follow_up`). Επιπλέον, στόχοι μέσα σε `drive_known_merge_radius_m` (0.50 m)
από μπάλα που ήξερε ήδη η ολοκληρωμένη διαδρομή απορρίπτονται, αλλιώς η αποστολή
θα κυκλωνόταν πάνω στο ίδιο γήπεδο.

- **Νέα αρχεία:** `collection_drive_observation.py` (καθαρό: viewpoint stepping,
  buffer, συναρμολόγηση snapshot, dedup) + `_LiveDriveObserver` στο node factory
  (ROS-facing). Ο executor δέχεται προαιρετικό `drive_observer` port, οπότε η
  αλλαγή είναι backward compatible.
- **Νέες ROS params:** `collection_route.drive_viewpoint_spacing_m: 0.75`,
  `collection_route.drive_known_merge_radius_m: 0.50`.
- **Tests:** 12 νέα pure για τον πυρήνα (viewpoint stepping, δύο-viewpoints
  κανόνας, coverage-gate ταυτότητα, dedup, renumbering, fail-loud) + 5 για το
  state machine (observer μόνο στο execution, off-route αντί rescan, fallback,
  φραγή budget, ποτέ μετά από abort). **493 pure pytest** (481 + 12 console),
  ROS build OK, `git diff --check` καθαρό.
- **ΕΚΚΡΕΜΕΙ live:** δεν έχει τρέξει distributed. Το επόμενο run θα δείξει πόσα
  βρίσκει πραγματικά η διαδρομή — και **δεν διορθώνει το offset των 43 cm**:
  βελτιώνει την **κάλυψη**, όχι την ακρίβεια.

## #41 — Live: το ack-aware finalize αποκάλυψε σταθερό `terminal_not_reached`

**Setup:** PC sim + Pi brain σε **`SLAM_MODE=localization`** πάνω στον σωσμένο
χάρτη `court_1785705204`, **χωρίς νέο survey**. Δύο runs.

**Τι ΕΠΙΒΕΒΑΙΩΘΗΚΕ:**

- **Το «survey μία φορά ανά γήπεδο» δουλεύει.** Το boundary της προηγούμενης
  συνεδρίας φορτώθηκε, το T υπολογίστηκε, το robot πλοηγήθηκε και σχεδίασε
  **9 και 10 στόχους** — η καλύτερη κάλυψη μέχρι τώρα (έναντι 3 και 7 σε
  mapping mode).
- **Το localization μειώνει το offset:** `pose_drift_m` **0.328** και **0.339**,
  έναντι 0.427/0.429 σε mapping mode. Περίπου −22%.
- **Το ack-aware finalize (#38) δουλεύει όπως σχεδιάστηκε.** Και τα δύο runs
  τερμάτισαν με `path_failed` και ρητό detail:
  `collection controller rejected terminal finalize: terminal_not_reached (code 5)`.
  Πριν το fix, αυτή η απόρριψη καταπινόταν και εμφανιζόταν **ένα route
  αργότερα** ως `invalid_lifecycle`. Δεν είναι regression· είναι η διάγνωση.

**Δύο λάθος υποθέσεις, καταγεγραμμένες ως λάθος:**

1. *«Ο core απορρίπτει updates πέρα από το terminal»* — καταρρίφθηκε: το gtest
   με pose πέρα από το τέλος πετά `collection_controller_profile_failure`, όχι
   `terminal_not_reached`, άρα δεν είναι αυτό το μονοπάτι.
2. *«Race λόγω ασυμμετρίας μανδάλωσης»* — το `collection_goal_checker` έχει
   `stateful: true` ενώ ο controller ξαναϋπολόγιζε το `terminal_ready` σε κάθε
   update. Μπήκε latch (`terminal_ready_ |= result.terminal_ready`, commit
   ebfe91c) και το live run **ξανα-απέτυχε ίδια**. Άρα το `terminal_ready`
   **δεν γίνεται ποτέ true**, δεν χάνεται. Το latch κρατήθηκε γιατί είναι σωστό
   ανεξάρτητα (37 C++ tests πράσινα, το gate δεν χαλάρωσε), αλλά **δεν ήταν
   αυτό το bug**.

**Τι ΔΕΝ ξέρουμε ακόμη.** Η συνθήκη στο `collection_tracking_core.cpp:148` έχει
**δύο** σκέλη και δεν ξέρουμε ποιο πέφτει:

```cpp
projection.progress_s + tol >= plan_.terminal_progress_s   // (α) μήκος τόξου
&& terminal_distance_m <= tol                              // (β) ευκλείδεια
```

Με `tol = terminal_progress_tolerance_m = 0.30`.

**Επόμενο βήμα: μέτρηση, όχι τρίτη υπόθεση.** Να δημοσιευτούν και τα δύο σκέλη
στο `CollectionControllerState` (τελευταίο `progress_s` έναντι
`terminal_progress_s`, και `terminal_distance_m`), ώστε **ένα** run να απαντήσει
ποιο από τα δύο αποτυγχάνει και με πόσο. Χωρίς αυτό, κάθε επόμενη αλλαγή είναι
τυφλή.

**Πλατφόρμα:** σταθερή σε όλο το session — 0 node deaths, 0 UDP errors, clean
shutdown, ο controller_node επιβίωσε μετά το fix του drive-observer buffer
(f5cf8d6).

## #42 — Πρώτο καθαρό route σε localization + πρώτο live off-route pass

**Setup:** PC sim + Pi σε `SLAM_MODE=localization` πάνω στον `court_1785705204`,
**χωρίς survey**, με το terminal instrumentation (#41) και το latch (ebfe91c).

**Αποτέλεσμα — καθαρός τερματισμός για πρώτη φορά σε αυτή τη σειρά runs:**

```
  0.0    navigating_to_scan_pose
  4.1    scanning
 23.2    planning              -> 10 στόχοι
 42.2    executing_route
188.5    route_completed        <- ΚΑΘΑΡΟ, κανένα terminal_not_reached
190.0    evaluating_results
190.0    planning               <- ΑΠΕΥΘΕΙΑΣ, χωρίς navigating_to_scan_pose
190.1    incomplete_targets
```

- **Το finalize έγινε δεκτό.** `route_completed`, `failure_detail: None`. Τα δύο
  προηγούμενα runs (ίδιο setup, χωρίς latch) απέτυχαν και τα δύο με
  `terminal_not_reached`. **Δεν είναι αποδεικτικό** — το φαινόμενο ήταν εξαρχής
  διαλείπον — αλλά είναι η πρώτη επιτυχία μετά το latch.
- **Το off-route pass λειτούργησε live.** Η μετάβαση `route_completed → planning`
  **χωρίς** ενδιάμεσο `navigating_to_scan_pose` είναι δυνατή **μόνο** από το
  `_begin_off_route_pass`. Δηλαδή το follow-up σχεδιάστηκε από τις μπάλες που
  βρέθηκαν **οδηγώντας**, από την τρέχουσα θέση, χωρίς επιστροφή στο T και χωρίς
  δεύτερο 360 — γλιτώνοντας το ταξίδι και τα ~90 s της σάρωσης.
- **Τερματισμός `incomplete_targets` 0.06 s μετά:** ο planner δεν βρήκε
  εκτελέσιμη διαδρομή για τους off-route στόχους (`EMPTY_NO_FEASIBLE_TARGETS →
  INCOMPLETE_TARGETS`, #37). Σωστή συμπεριφορά: στόχοι υπήρχαν, διαδρομή όχι.
- **4/10 retained**, `crossed_unconfirmed=0`, **`pose_drift_m = 0.319`** — το
  χαμηλότερο που έχει μετρηθεί (0.427/0.429 mapping, 0.328/0.339 localization).

**Ανοιχτό/ανεπιβεβαίωτο:** το info log `off-route discovery: N new target(s)`
δεν εντοπίστηκε στο Pi log, παρότι η μετάβαση καταστάσεων αποδεικνύει ότι το
μονοπάτι εκτελέστηκε. Πιθανό ζήτημα επιπέδου/ροής logging, όχι λογικής. Πριν
δηλωθεί το χαρακτηριστικό «επιβεβαιωμένο», χρειάζεται run που να δείχνει τον
**αριθμό** των off-route στόχων και, ιδανικά, ένα follow-up route που όντως
εκτελείται.

**Δεν έχει ακόμη επικυρωθεί:** δεύτερο *mission* (νέα εντολή `collect_route`)
μετά από καθαρό τερματισμό — το τεστ που εξαρχής κίνησε το finalize fix.

## #43 — Το instrumentation απάντησε: το route κόβεται στη μέση από τον goal checker

**Run:** φρέσκος sim (μπάλες στις αρχικές θέσεις), Pi σε localization χωρίς
survey. Plan 9 στόχοι, retained 2, `aborted_tracking`. Το detail (#41):

```
terminal_not_reached (code 5)
[progress_s=29.211  terminal_progress_s=57.626
 terminal_distance_m=0.299  terminal_ready=False]
```

**Απάντηση:** πέφτει το σκέλος **(α), το μήκος τόξου** — και όχι οριακά: το
robot βρίσκεται στα **29.2 από 57.6 m**, δηλαδή **στη μέση της διαδρομής**.
Ταυτόχρονα η **ευκλείδεια** απόσταση από το τελικό σημείο είναι **0.299 m**,
οριακά **μέσα** στο `xy_goal_tolerance: 0.30`.

**Ρίζα:** οι διαδρομές συλλογής είναι **βρόχοι** (forward-only + min_turn_radius
1.25 m) και περνούν **δίπλα από το ίδιο τους το τέρμα στα μισά**. Ο
`nav2_controller::SimpleGoalChecker` ελέγχει **μόνο** ευκλείδεια απόσταση από το
τελικό pose — άρα δηλώνει «έφτασα» στο μέσο, και με `stateful: true` το
μανδαλώνει. Το FollowPath goal τερματίζει επιτυχώς και **η διαδρομή κόβεται στη
μέση**.

Αυτό εξηγεί ταυτόχρονα: (i) γιατί το finalize απορριπτόταν —ο controller είχε
δίκιο, η διαδρομή **δεν** είχε τελειώσει· (ii) γιατί ήταν διαλείπον — εξαρτάται
από το αν η συγκεκριμένη γεωμετρία βρόχου περνά μέσα σε 0.30 m από το τέρμα της·
(iii) γιατί το latch (ebfe91c) δεν βοήθησε — το `terminal_ready` δεν έγινε ποτέ
true, σωστά.

**Το ack-aware finalize (#38) αποδεικνύεται σωστό δύο φορές:** όχι μόνο
εντόπισε την απόρριψη, αλλά η απόρριψη ήταν **ουσιαστικά σωστή**. Πριν, το route
ανέφερε ψευδώς `route_completed` **στα μισά** και κανείς δεν το έβλεπε.

**Fix direction (ΔΕΝ υλοποιήθηκε):** ο goal checker της συλλογής δεν επιτρέπεται
να τερματίζει με κριτήριο εγγύτητας μόνο. Χρειάζεται goal checker που απαιτεί
και **πρόοδο κατά μήκος της διαδρομής** (ή ρητή σύζευξη με το `terminal_ready`
του collection controller). Μείωση του `xy_goal_tolerance` **δεν** αρκεί: ένας
βρόχος μπορεί να περάσει αυθαίρετα κοντά στο τέρμα του.

## #44 — Παρατήρηση χρήστη: αστοχία εκατοστών στην κορυφή του funnel

Στο ίδιο run, με τα μάτια στο GUI: οι μπάλες **3, 4 και 5** αστοχούν **για
εκατοστά** και **χτυπούν στην κορυφή του funnel** αντί να μπουν.

**Σημασία:** αυτό **δεν** είναι το spatial offset. Με απόκλιση 30+ cm η μπάλα
δεν θα άγγιζε καν το χωνί. Η οριζόντια ευθυγράμμιση είναι πλέον σχεδόν σωστή
(συνεπές με `pose_drift_m` 0.319-0.339 σε localization, και με
`lateral_error ≈ 0` στα crossings). Το υπόλοιπο σφάλμα είναι **γεωμετρία
ύψους/χείλους**: η μπάλα συναντά το **χείλος εισόδου** αντί να περάσει από κάτω.

**Δύο ανεξάρτητα ανοιχτά, όχι ένα:**
- **Α (#43):** η διαδρομή κόβεται στη μέση — λογισμικό, goal checker.
- **Β (#44):** το χωνί απορρίπτει σωστές προσεγγίσεις — μηχανική γεωμετρία,
  το «μπάλα-στο-χωνί» του αρχικού πλάνου. Χρειάζεται μέτρηση ύψους χείλους
  έναντι τροχιάς μπάλας, όχι αλλαγή planner.

## #45 — Progress-aware collection goal checker: υλοποιήθηκε

Ο γενικός `nav2_controller::SimpleGoalChecker` αντικαταστάθηκε **μόνο** για τα
collection FollowPath goals από νέο plugin
`tennis_robot_collection_controller::CollectionProgressGoalChecker`. Το
`general_goal_checker` των survey/navigation routes δεν άλλαξε.

Ο Nav2 `GoalChecker` δεν λαμβάνει την path στο interface του. Αντί να
ξαναϋπολογίζεται δεύτερο, δυνητικά διαφορετικό projection, το plugin καταναλώνει
το ήδη canonical state του `CollectionNav2Controller` από
`CollectionFollowPath/state`. Επιτυχία επιτρέπεται μόνο όταν ισχύουν **όλα**:

- φρέσκο state heartbeat (`state_timeout_s: 0.50`), από lifecycle `EXECUTING`,
  χωρίς typed failure και με δεσμευμένα `plan_id/path_sha256`·
- `terminal_ready == true` από τον collection tracking core·
- `progress_s + progress_tolerance_m >= terminal_progress_s`·
- XY και yaw μέσα στις ανεξάρτητες goal tolerances.

Το `reset()` αδειάζει το cached state, άρα terminal telemetry προηγούμενου goal
δεν μπορεί να εγκρίνει το επόμενο. `SAFETY_PAUSED`, stale, malformed ή failed
state απορρίπτονται fail-closed. Ειδικά στο #43, η εγγύτητα 0.299 m δεν αρκεί
πλέον: το `29.211 + 0.30 < 57.626` κρατά το FollowPath ενεργό.

**Tests:** νέο 5-case gtest καλύπτει midpoint proximity rejection, αληθινό
terminal success, απαίτηση του controller terminal verdict, reset isolation και
paused/failed rejection. Το plugin φορτώνεται μέσω pluginlib και μέσα σε
πραγματικό isolated Jazzy `controller_server`. Native Jazzy build PASS, **7/7**
CTest targets PASS (τα 2 fixture-dependent parity cases SKIP όπως προβλέπεται),
**475** pure pytest + **12** console tests PASS, `git diff --check` καθαρό.

**ΕΚΚΡΕΜΕΙ live:** ένα πλήρες localization collection mission. Acceptance:
περνά το προηγούμενο midpoint χωρίς `Reached the goal!`, φτάνει terminal progress,
το ack-aware finalize γίνεται accepted και εμφανίζονται οι υπόλοιπες 9–10
προσεγγίσεις. Μόνο τότε αξιολογείται το ανεξάρτητο funnel-lip θέμα του #44.

## #46 — Δεύτερο mission: boundary-contact passes και forward U-turn

**Live input (2026-08-03):** το δεύτερο `collect_route` ξεκίνησε και ολοκλήρωσε
18/18 scan steps, επιβεβαιώνοντας τρεις στόχους στα `(6.064,-0.678)`,
`(8.018,-0.690)`, `(8.086,1.744)`. Ο planner επέστρεψε
`empty_no_feasible_targets`: έναν `turn_radius` και δύο `keepout`. Οι δύο
τελευταίες μπάλες απείχαν αντίστοιχα **0.432 m** και **0.347 m** από τον
surveyed άξονα του φιλέ, άρα κόπηκαν από το isotropic `0.50 m` keepout πριν
παραχθεί tangent pass.

**Boundary recovery:** για target που είναι έξω από το πραγματικό net/fence
polygon αλλά μέσα μόνο στο inflated boundary keepout, το Phase 3A δοκιμάζει
single-ball pass παράλληλο στο obstacle. Η centerline μετατοπίζεται προς το
γήπεδο κατά `0.205 m`, την configured θέση του εξωτερικού funnel cheek στο
στόμιο. Η μπάλα μένει στο πραγματικό planned crossing και επομένως το cheek
προβλέπεται να την αγγίξει — ακριβώς το πείραμα «ξεκολλάει από φιλέ/φράχτη;».
Δεν μειώθηκε το canonical clearance `0.50 m`: ολόκληρο το μετατοπισμένο pass,
οι connectors και το terminal ελέγχονται όπως πριν. Πραγματική επικάλυψη με
το net/fence, οποιοδήποτε bench/post/other keepout ή αποτυχία του μετατοπισμένου
διαδρόμου παραμένει deterministic `keepout`. Contact candidates δεν μπαίνουν
σε shared pass.

**Turn blocker:** replay με τις ακριβείς τρεις θέσεις και start pose
`(1.958,-0.067,3.1204)` έδειξε ότι η ακτίνα `1.25 m` δεν ήταν το πρόβλημα. Το
παλιό `max_connector_arc_angle_rad=1.5` απαγόρευε το forward U-turn επειδή το
robot άρχιζε σχεδόν αντίθετα από όλους τους στόχους. Με ίδια ακτίνα, CSC-only,
continuous collision και self-intersection guards, bounded caps `3.0 rad` ανά
arc / `6.0 rad` total δίνουν **FEASIBLE 3/3**, μήκος replay περίπου `25.15 m`.

**Tests:** regression για τις ακριβείς live συντεταγμένες, too-close keepout,
static-obstacle non-bypass, tangent-only contact και shared-pass isolation.
`321` collection tests, `481` pure tests και `12` console tests PASS. Εκκρεμεί
distributed Gazebo run για να κριθεί το πραγματικό contact του funnel.

## #47 — Live acceptance του #45: πρώτο `route_completed` με πραγματικές συλλογές

**Setup (2026-08-12):** PC = Gazebo GUI μόνο (`TENNIS_LAUNCH_BRAIN=false
./run_native.sh`, `ROS_DOMAIN_ID=42`), Pi = brain σε
`SLAM_MODE=localization` πάνω στο αποθηκευμένο `court_1785705204.posegraph`
και στο `court_boundary.json` του **ίδιου** survey. Trigger `collect_route`
από το panel του Pi. Ο Pi ξαναχτίστηκε ώστε να έχει τον planner του #46 (το
colcon install αντιγράφει, δεν κάνει symlink — χωρίς rebuild ο κόμβος τρέχει
το παλιό `collection_route_planner_v2.py`).

**Run 1 — το acceptance του #45 πέρασε.** `navigating_to_scan_pose 705.8 →
scanning 710.0 → planning 729.1 → executing_route 838.4 → **route_completed**
992.7`. 9/9 στόχοι planned, `planning_status: feasible`, καμία περικοπή στο
midpoint, κανένα tracking abort σε 154 s εκτέλεσης. Το ack-aware finalize
πέρασε και ο executor μπήκε μόνος του σε δεύτερο run. Αυτό ήταν ακριβώς το
εκκρεμές acceptance του #45.

**Πρώτες πραγματικές συλλογές distributed:** `confirmed 3` (targets 4, 7, 8),
`basket_retained 4`, `beam_credits 3`. Τα confirmations ήρθαν με
`lateral_error` `0.013 / 0.024 / 0.009 m` και `heading_error` κάτω από
`0.025 rad` — ο διάδρομος πέφτει **πάνω** στη μπάλα. Το spatial offset του
#32/#15 δεν είναι πια ο περιοριστικός παράγοντας σε localization
(`pose_drift_m 0.042` τρέχον, `0.343` στο execution snapshot).

**Το #44 απομονώθηκε ποσοτικά.** Τέσσερις στόχοι (1, 2, 6, 9) έμειναν
`execution_status: executing` με **15-16 crossing samples** και **0
confirmations**: το robot διέσχισε κανονικά τον διάδρομό τους και η μπάλα
δεν μπήκε. Δεν είναι πλάνο, δεν είναι tracking, δεν είναι frame — είναι
**γεωμετρία χείλους**, όπως το είδε ο χρήστης στο #44. Δύο ακόμα (3, 5)
έμειναν `planned` με `crossing_samples 0` παρόλο που το route ολοκληρώθηκε·
θέλει ξεχωριστό κοίταγμα ποιο segment τα κάλυπτε. Επίσης `basket_retained 4`
έναντι `beam_credits 3`: μία μπάλα μπήκε χωρίς beam confirmation (ανοιχτό
basket-beam reconciliation).

**Run 2 — νέα αστοχία, οριακό heading gate σε pass.** Το follow-up run
σχεδίασε από το off-route scan `.../drive-1` (μηχανισμός #40), 9 planned /
2 skipped, `planning_status: partial`, και σταμάτησε 21 s μετά την εκκίνηση:
`route_outcome: aborted_tracking`, `failure_reason 15 = heading_error_exceeded`,
segment `pass:...drive-1/target-11:12`, `progress 7.432 m`,
`lateral_error 0.023 m`, `heading_error -0.153 rad`. Το capture-grade gate
είναι `max_heading_error_rad: 0.15`, άρα η υπέρβαση είναι **0.003 rad**. Με
lateral error 2.3 cm το robot ήταν πάνω στη διαδρομή — πρόκειται για
pure-pursuit ταλάντωση heading στο ίδιο μοτίβο με #29 και τον curvature spike
του #34, αυτή τη φορά σε **pass** segment όπου το gate είναι σκόπιμα σφιχτό
για ακρίβεια capture. Δεν άλλαξε τιμή σε αυτό το session: η επιλογή είναι
είτε χαλάρωση του gate (κινδυνεύει η ακρίβεια που μόλις επιβεβαιώθηκε στα
0.009-0.024 m) είτε φιλτράρισμα/υστέρηση του heading error αντί για στιγμιαίο
hard gate.

**Το #46 δεν δοκιμάστηκε live:** καμία μπάλα αυτού του layout δεν ήταν μέσα
στο inflated net/fence keepout, οπότε δεν παρήχθη boundary-recovery candidate.
Παραμένει εκκρεμές acceptance.

## #48 — Επανάληψη χωρίς αλλαγές: ο σχεδιασμός αναπαράγεται, η εκτέλεση όχι· και η ζώνη θανάτου

**Setup:** πανομοιότυπη επανάληψη του #47 με **μηδενικές αλλαγές** (ίδιο commit,
ίδιο map `court_1785705204`, ίδιο `court_boundary.json`, φρέσκος κόσμος, restart
και των δύο stacks). Τα artifacts του πρώτου run αρχειοθετήθηκαν στο Pi
(`runtime/run_archive/20260812_run1/`) πριν γραφτούν από πάνω.

**Αναπαραγωγιμότητα — διχασμένη:**

| | run #47 | run #48 |
| --- | --- | --- |
| scanning → planning | 19.1 s | 19.2 s |
| planned / status | 9 / `feasible` | 9 / `feasible` |
| collector_starting → executing_route | 109.3 s | 104.7 s |
| confirmed | 3 | 5 |
| retained | 4 | 6 |
| outcome | `route_completed` | `aborted_tracking` |

Ο **σχεδιασμός** είναι ντετερμινιστικός και επαναλήψιμος. Η **εκτέλεση** δεν
είναι: 3 έναντι 5 συλλήψεων με ίδιο πλάνο. Συνέπεια για τη μεθοδολογία: ο
αριθμός συλλογών **δεν** είναι έγκυρο κριτήριο αξιολόγησης αλλαγής — χρειάζεται
γεωμετρικό κριτήριο (πόσες μπάλες βρέθηκαν εντός/εκτός διαδρόμου), όχι score.

**Παρατήρηση χρήστη στο GUI (η κρίσιμη):** δύο μπάλες **χτυπήθηκαν από το σώμα
του ρομπότ** και μετακινήθηκαν αντί να οδηγηθούν στο χωνί. Η διαδρομή περνά
δίπλα τους αντί να τις καλύψει, και η καμπύλη που παράγεται τις σπρώχνει.

**Επιβεβαίωση στον κώδικα — δύο ελαττώματα που αλληλοτροφοδοτούνται.**

*(1) Ζώνη θανάτου άγνωστη στον planner.* Οι τροχοί είναι στα `wheel_y = ±0.35 m`
(URDF) και το `capture_half_width_m` είναι `0.17`. Άρα για πλευρική απόσταση
μπάλας από τον άξονα πορείας: `≤0.17` σύλληψη, **`0.17–0.35+` χτύπημα από
τροχό/σασί**, `>0.5` καθαρό πέρασμα. Το `_segment_is_collision_free`
(`collection_route_planner_v2.py`) ελέγχει **μόνο** `navigable_polygon` και
`court.obstacles`· οι μπάλες δεν εμφανίζονται ποτέ σε αυτόν τον έλεγχο, ούτε ως
εμπόδιο ούτε ως κόστος. Κάθε connector μπορεί να διασχίσει ελεύθερα τη μεσαία
ζώνη. Επιπλέον, μια σπρωγμένη μπάλα ακυρώνει το **παγωμένο** πλάνο: το επόμενο
πέρασμα στοχεύει συντεταγμένη που δεν ισχύει πια.

*(2) Το shared pass δεν σχηματίζεται σχεδόν ποτέ.* Το merging ομαδοποιεί ανά
**ακριβές** `heading_rad` (`collection_route_shared_pass.py`, `by_heading` +
`member.heading_rad != heading`), και οι κατευθύνσεις παράγονται από
`heading_sample_count: 16` → πλέγμα `22.5°`, άρα έως `11.25°` απόκλιση από την
πραγματική ευθεία δύο μπαλών. Η απαιτούμενη ακρίβεια όμως είναι `≤9.8°` στα 2 m,
`≤6.5°` στα 3 m, `≤3.9°` στα 5 m. Σε τυπικές αποστάσεις γηπέδου η ευθεία **δεν
υπάρχει καν ως υποψήφια**. Επιβεβαίωση από τα δεδομένα: και οι **5** συλλήψεις
του #48 ήρθαν από **5 ξεχωριστά μονήρη περάσματα**, κανένα `shared:`· στο #47
σχηματίστηκε ακριβώς **ένα**.

*Σύνθεση:* χωρίς κοινά περάσματα → 9 χωριστά περάσματα → 9 connectors →
**57.7 m** για 9 μπάλες (6.4 m/μπάλα) → πολλαπλές διελεύσεις μέσα από τη ζώνη
θανάτου → σπρωγμένες μπάλες → μπαγιάτικο πλάνο.

**Χωριστό ανοιχτό:** το abort του #48 έγινε στο **terminal** segment στα
`57.658 m` διαδρομής `57.7 m` — 4 cm πριν το τέλος — με
`trajectory_tube_exceeded` ΚΑΙ `lat_err 0.000, head_err 0.000`. Μηδενικά
σφάλματα με ταυτόχρονη παραβίαση tube είναι αντιφατικό· συγγενές των #41/#43,
δική του διερεύνηση.

**Επίσης:** το run είχε `pending 4` — τέσσερις μη επιβεβαιωμένες ανιχνεύσεις που
δεν είναι ούτε στόχοι ούτε εμπόδια, άρα το ρομπότ τις πατάει με βεβαιότητα.

**Status:** καμία αλλαγή κώδικα. Υπό αναθεώρηση το σύνολο κανόνων παραγωγής
διαδρομής (απόφαση χρήστη). Υποψήφιοι κανόνες: (A) ακριβείς ανά ζεύγος
κατευθύνσεις στο σύνολο υποψηφίων· (B) μπάλα σε ζώνη `(0.17, 0.5]` από segment =
παραβίαση· (C) μπάλα εντός `0.17` σε ξένο segment → promotion στο covered set·
(D) pending ανιχνεύσεις ως αποφευκτέες· (E) όρος κόστους κάλυψης ανά μέτρο.

> **ΔΙΟΡΘΩΣΗ (βλ. #49):** η εξήγηση «το πλέγμα των 22.5° εμποδίζει τα shared
> passes» που καταγράφεται παραπάνω είναι **λανθασμένη**. Offline replay με τα
> πραγματικά δεδομένα έδειξε ότι 11/18 ζεύγη είναι συγχωνεύσιμα ακόμη και στο
> πλέγμα. Η πραγματική αιτία είναι ο προϋπολογισμός του διαδρόμου σύλληψης.

## #49 — Η πραγματική αιτία των 9 χωριστών περασμάτων: ο διάδρομος είναι ±2.6 cm

**Μέθοδος:** offline replay του σαρωμένου στιγμιότυπου του #48 μέσα από τον
**πραγματικό** planner (`analyze_snapshot` + `generate_shared_passes`) με το
πραγματικό `court_boundary.json` του Pi και το πραγματικό
`config/collection_route.yaml`. Καμία προσομοίωση, καμία ROS εξάρτηση.

**Πρώτο εύρημα — το πλέγμα δεν φταίει.** Και οι 9 μπάλες παίρνουν και τις 16
κατευθύνσεις ως υποψήφιες (`candidates=16`, `reason=None`, κανένα tangent
restriction). Με το ονομαστικό `capture_half_width_m 0.17`, 13/18 ζεύγη περνούν
όλους τους γεωμετρικούς ελέγχους. Ο generator όμως παρήγαγε **8 υποψήφιους,
όλους με την ίδια μπάλα (t8)**, και το ζεύγος `t12+t13` — γεωμετρικά καθαρό στα
`0°/180°` με spread `0.174 m` — δεν σχηματίστηκε ποτέ.

**Ρίζα:** το `_build_shared_candidate` δεν συγκρίνει με το ονομαστικό
`capture_half_width_m` αλλά με το `effective_capture_half_width_m` του κάθε
υποψήφιου, που υπολογίζεται στο `_effective_capture_half_width` ως:

```
0.170  capture_half_width_m
-0.033  ball_radius_m
-0.040  tracking_lateral_error_bound_m
-0.050  capture_safety_margin_m
=0.047  πριν από κάθε αβεβαιότητα        ← 72% του διαδρόμου έχει ήδη καταναλωθεί
-2σ     confidence_multiplier * projected_stddev (0.014 … 0.029 στο run)
=0.018 … 0.033 m   (μέσος όρος 0.0265)
```

**Ο planner σχεδιάζει για διάδρομο ±2.6 cm.** Κοινή ευθεία απαιτεί **και τις
δύο** μπάλες μέσα σε αυτόν, πράγμα που πρακτικά δεν συμβαίνει — εξ ου 9 χωριστά
περάσματα, 9 connectors, 57.7 m.

**Ευαισθησία στα ίδια δεδομένα** (ζεύγη συγχωνεύσιμα από 36):

| effective half-width | ζεύγη |
| --- | --- |
| 0.027 m (τρέχον) | 2-3 |
| 0.075 m | 7 |
| 0.100 m | 9 |
| 0.137 m | 13 |
| 0.170 m | 16 |

**Η αντίφαση με τη μετρημένη πραγματικότητα:** το φυσικό στόμιο του χωνιού είναι
`±0.205 m` (παρειές στο URDF) και οι live συλλήψεις έγιναν με πλευρικά σφάλματα
`0.009 / 0.013 / 0.024 m`. Το ρομπότ πιάνει αποδεδειγμένα μπάλες σε αποστάσεις
μεγαλύτερες από τον διάδρομο που ο planner επιτρέπει στον εαυτό του. Υπάρχει και
διπλή συντηρητικότητα: το `0.17` είναι ήδη 3.5 cm κάτω από το φυσικό `0.205`, και
από πάνω αφαιρείται δεύτερο ρητό margin `0.05`.

**Συνέπεια για τους κανόνες:** ο (A) υποβαθμίζεται σε βελτίωση δεύτερης τάξης —
στα `0.137` το υπάρχον πλέγμα βρίσκει ήδη 13 ζεύγη. Προτεραιότητα παίρνει η
διόρθωση του προϋπολογισμού, και ο (B) γίνεται **πιο** κρίσιμος: όσο πλαταίνει ο
διάδρομος, τόσο περισσότερες μπάλες βρίσκονται κοντά σε περάσματα και πρέπει να
μαζεύονται αντί να σπρώχνονται. Ο (B) είναι επίσης ο κανόνας που αναγκάζει τον
connector να βγει **έξω** από το σύμπλεγμα, δηλαδή η εξωτερική καμπύλη που
ζήτησε ο χρήστης.

**Status:** καμία αλλαγή τιμών. Επόμενο βήμα προς έγκριση: offline σύγκριση
τρέχοντος έναντι διορθωμένου προϋπολογισμού στα ίδια δεδομένα, με μετρικές
αριθμό περασμάτων / συνολικό μήκος / μέτρα ανά μπάλα. Τα scripts του replay
είναι στο scratchpad της συνεδρίας, όχι στο repo.
