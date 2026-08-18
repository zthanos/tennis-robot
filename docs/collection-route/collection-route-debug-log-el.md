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

## #50 — Μετρήσεις: ο σχεδιασμός αθωώνεται, το πρόβλημα είναι ανάντη

**Μέθοδος:** offline, πραγματικός planner, πραγματικό `court_boundary.json`,
πραγματικές θέσεις και covariances του #48. Δύο πειράματα: σάρωση του
προϋπολογισμού διαδρόμου, και γεωμετρική έκθεση κάθε μπάλας στη διαδρομή.

**Πείραμα 1 — προϋπολογισμός διαδρόμου.** Το κέρδος είναι μέτριο και **μη
μονότονο**:

| παραλλαγή | eff_w | περάσματα | shared | μήκος | m/μπάλα |
| --- | --- | --- | --- | --- | --- |
| baseline | 0.047 | 8 | 1 | 52.9 | 5.9 |
| capture_safety_margin 0.00 | 0.097 | 7 | 2 | 47.6 | 5.3 |
| half 0.205 + margin 0.02 | 0.112 | 7 | 2 | 67.9 | 7.5 |
| half 0.205 + margin 0.01 | 0.122 | 7 | 2 | 59.3 | 6.6 |

Καλύτερη περίπτωση 8→7 περάσματα και 52.9→47.6 m· σε άλλη ρύθμιση η διαδρομή
**χειροτερεύει** (67.9 m). Η πρόβλεψη του #49 ότι το πλάτεμα θα έδινε «λίγα
μακριά ευθύγραμμα περάσματα» **δεν επιβεβαιώνεται**. Διορθώνεται επίσης το #48:
το baseline δίνει 8 περάσματα με 1 shared, όχι 9 μονήρη.

**Πείραμα 2 — έκθεση μπαλών.** Πλησιέστερη προσέγγιση οποιουδήποτε **άλλου**
segment σε κάθε μπάλα: **8 από τις 9 στα 0.300 m με πλευρική μετατόπιση
0.000 m**. Το 0.300 είναι ακριβώς το `minimum_run_out_m` — ο connector ξεκινά
0.3 m *μπροστά* από τη μπάλα, δηλαδή η μπάλα είναι **πίσω** από το ρομπότ, όχι
δίπλα. Από τις 4 pending μόνο μία (0.49 m) είναι οριακή.

**Συμπέρασμα: το σχεδιασμένο πλάνο δεν οδηγεί το σώμα πάνω σε καμία μπάλα.**
Ο υποψήφιος κανόνας B (ζώνη θανάτου `0.17-0.35`) δεν έχει πού να εφαρμοστεί στα
δεδομένα αυτού του run, και ο A δίνει μικρό κέρδος. Η βρόχωση είναι πραγματική
αλλά **δεν** είναι η αιτία των χτυπημάτων που παρατήρησε ο χρήστης.

**Άρα το χτύπημα γεννιέται στην εκτέλεση:** στη διαφορά ανάμεσα στην
εκτιμώμενη και την πραγματική θέση της μπάλας. Σφάλμα perception ~0.20-0.25 m
τοποθετεί τον κεντραρισμένο διάδρομο πλευρικά πάνω στη μπάλα, οπότε τη βρίσκει
η παρειά ή ο τροχός αντί για το στόμιο — ακριβώς η παρατηρούμενη εικόνα.

**Σφάλματα μεθόδου που εντοπίστηκαν και διορθώθηκαν πριν το συμπέρασμα:**
(1) συνένωση σημείων διαφορετικών segments δημιουργούσε ψεύτικη χορδή πάνω από
τη μπάλα (ψευδές `0.000 m`)· (2) η «απόσταση από πολυγραμμή» δεν διακρίνει το
«πίσω μου» από το «δίπλα μου» — χρειάζεται πλευρική προβολή.

**Status:** καμία αλλαγή τιμών ή κανόνων. Το επόμενο βήμα μετατοπίζεται από τον
planner στο **ground-truth telemetry της perception** (πρόταση χρήστη): μέτρηση
του σφάλματος εκτιμώμενης έναντι πραγματικής θέσης μπάλας, ανά διέλευση, σε
διανύσματα στο frame του χωνιού.

## #51 — Παρατήρηση χρήστη: η καμπύλη είναι νεκρή μεταφορά· το 76% της διαδρομής δεν μαζεύει

**Παρατήρηση χρήστη:** «ο σχεδιασμός χρησιμοποιεί την καμπύλη για πέρασμα σε
κάθε μπάλα· η καμπύλη πρέπει να λαμβάνει υπ' όψιν αν δίνει τροχιά για μάζεμα
περισσοτέρων». Το #50 απάντησε σε **άλλο** ερώτημα (αν το πλάνο *χτυπά* μπάλες)
και το συμπέρασμά του διατυπώθηκε υπερβολικά ως «αθωώνεται ο σχεδιασμός». Ισχύει
μόνο για τη συγκεκριμένη κατηγορία· η παρακάτω είναι διαφορετική και υπαρκτή.

**Δομικός περιορισμός:** σύλληψη επιτρέπεται **μόνο** σε ευθύγραμμο
`FUNNEL_PASS`. Ο `CONNECTOR` έχει εξ ορισμού κενό `covered_ball_ids` και δεν
εξετάζεται ποτέ αν διέρχεται από μπάλες.

**Μέτρηση στη διαδρομή του #48 (52.8 m):**

| τύπος | μήκος | ποσοστό |
| --- | --- | --- |
| connector | 39.6 m | 75% |
| funnel_pass | 12.7 m | 24% |
| terminal | 0.5 m | 1% |

**Το 76% της κίνησης απαγορεύεται δομικά να μαζέψει.** Αναλύοντας τους CSC
connectors σε τόξα και ευθείες:

- **18.8 m είναι ευθεία ή σχεδόν** (`R > 5 m`, pure-pursuit lead ~`0.015 rad`) —
  **ήδη capture-grade** στο σφιχτό gate `0.15`, χωρίς καμία χαλάρωση·
- 21.3 m είναι τόξα στην ελάχιστη ακτίνα `1.25 m` (lead ~`0.24 rad`, όντως
  ακατάλληλα για σύλληψη)·
- **μπάλες που μαζεύονται σε καμπύλη: 0.**

**Χωρητικότητα:** από τα 9 ευθύγραμμα τμήματα μέσα σε connectors, **4 είναι
μακρύτερα από τα 1.3 m** που απαιτεί ένα crossing (`run_in 1.0` + `run_out 0.3`):
`4.58`, `4.57`, `3.91`, `2.57` m — σύνολο **15.6 m** αξιοποιήσιμης ευθείας,
χωρητικότητα έως 10 επιπλέον διελεύσεων.

**Η αλυσίδα εκτέλεσης το υποστηρίζει ήδη — καμία αλλαγή συμβολαίου ή C++:**
το `_build_segment` του `collection_execution_context_builder.py` σειριοποιεί
`planned_crossings` γενικά για κάθε τύπο segment, και ο
`collection_tracking_core.cpp` τα επικυρώνει ανά segment (μονοτονία, όρια
progress, `required_run_out_m`) **χωρίς κανέναν έλεγχο τύπου segment**. Λείπει
μόνο η πλήρωσή τους από τον planner.

**Προτεινόμενη κλιμάκωση:**
1. *Ευκαιριακό:* αφού επιλεγεί connector, έλεγχος αν τα ευθύγραμμα τμήματά του
   περνούν εντός διαδρόμου από ακάλυπτη μπάλα → προσάρτηση crossing. Δωρεάν,
   καμία αλλαγή στην αναζήτηση.
2. *Κατευθυνόμενο (η «πράσινη γραμμή»):* η κάλυψη πάνω σε connector μπαίνει στο
   κόστος, ώστε ο solver να **επιλέγει** γεωμετρία connector που σαρώνει μπάλες·
   υποψήφιοι connectors που περνούν από ακάλυπτη μπάλα ως via-point.
3. Profile: το crossing-bearing τμήμα χρειάζεται capture-grade gate αντί για το
   χαλαρό `connector_max_heading_error_rad 0.5` — τα ευθύγραμμα τμήματα το
   περνούν ούτως ή άλλως (lead `0.015`).

**Status:** καμία αλλαγή κώδικα. Το εύρημα αναβαθμίζει την προτεραιότητα του
planner έναντι του perception telemetry του #50.

## #52 — Σύλληψη πάνω σε connector: υλοποιήθηκε, αλλά ο στόχος δεν την αποτιμά

**Υλοποίηση (working tree, ΔΕΝ έχει γίνει commit):** οι connectors μπορούν πλέον
να μαζεύουν. Νέα `swept_crossings_for_path` βρίσκει μπάλες που περνούν από τον
διάδρομο του χωνιού σε τμήματα αρκετά ήπια (`capture_minimum_turn_radius_m 2.5`,
από το pure-pursuit lead `lookahead/2R` έναντι του capture gate `0.15`), με
πλήρες παράθυρο run-in/run-out **μέσα** στο ίδιο segment όπως απαιτεί ο tracking
core. Νέο `sweep_radius_multipliers [1.0, 3.0, 6.0]` παράγει και ήπιες ακτίνες —
πριν κάθε τόξο ήταν στο σφιχτό ελάχιστο 1.25 m. Ο solver μετράει την κάλυψη
ακμών (`current_covered`, reachable bound, disjointness, ordering) και ο
connector που φέρει crossing παίρνει **capture-grade** profile αντί για το
χαλαρό transit gate. Χαλάρωσε συνειδητά η αναλλοίωτη «only funnel passes may
have covered balls»· ο terminal connector μένει καθαρή μεταφορά.

**Διόρθωση που προέκυψε από test:** μπάλα 3A-unreachable (keepout) δεν επιτρέπεται
ποτέ να σαρωθεί — το να περνάς από πάνω της είναι ακριβώς το απαγορευμένο. Ο
graph builder φιλτράρει τις επιλέξιμες μπάλες σε όσες έχουν pass candidate.

**Μέτρηση στα πραγματικά δεδομένα του #48 (A/B με monkeypatch της σάρωσης):**

| παραλλαγή | passes | swept | μήκος | m/μπάλα | collect% | στροφή rad |
| --- | --- | --- | --- | --- | --- | --- |
| σάρωση OFF (προ αλλαγής) | 8 | 0 | 52.9 | 5.9 | 24% | 17.1 |
| σάρωση ON, σφιχτές ακτίνες | 7 | 1 | 60.5 | 6.7 | 39% | 18.3 |
| σάρωση ON + ήπιες ακτίνες | 7 | 1 | 60.1 | 6.7 | 40% | 22.0 |
| shipped, budget 10000 | 8 | 0 | 52.9 | 5.9 | 24% | 17.1 |
| shipped, budget 40000 | 8 | 0 | 52.9 | 5.9 | 24% | 17.1 |

**Δύο συμπεράσματα, και τα δύο αρνητικά για την αλλαγή ως έχει:**

1. Η επιδείνωση σε `60.1 m / 22.0 rad` στο default `max_search_expansions 3000`
   είναι **artifact αναζήτησης**: η αλλαγή μεγαλώνει τον χώρο ακμών και τα 3000
   expansions δεν επαρκούν πια. Με 10000 ο solver ξαναβρίσκει την ίδια διαδρομή.
2. Με επαρκές budget ο solver επιλέγει **να μη σαρώσει καθόλου**. Η κάλυψη είναι
   9/9 και στις δύο περιπτώσεις, η σαρωτική διαδρομή είναι μακρύτερη, και το
   `weight_pass_count 1.0` που γλιτώνει ένα πέρασμα δεν αντισταθμίζει +7 m.

**Δηλαδή η υδραυλική δουλεύει και είναι συμβατή με το συμβόλαιο (καμία αλλαγή
C++ — ο tracking core επικύρωνε ήδη crossings ανά segment χωρίς έλεγχο τύπου),
αλλά ο στόχος βελτιστοποίησης δεν αποτιμά το κέρδος.** Η ευκαιριακή σάρωση
πληρώνει μόνο όταν μια μπάλα τυχαίνει να βρίσκεται σε connector· δεν κάνει τη
διαδρομή συντομότερη από μόνη της.

**Τι θα χρειαζόταν για την «πράσινη γραμμή»:** όρος στόχου που αποτιμά **κάλυψη
ανά μέτρο** (ή ουσιαστικά μεγαλύτερο `weight_pass_count`), ώστε το γλίτωμα ενός
περάσματος με το run-in/run-out και τις στροφές του να μετράει όσο πραγματικά
αξίζει· και παραγωγή connectors **στοχευμένων** σε ακάλυπτες μπάλες αντί για
ευκαιριακή ανίχνευση πάνω σε γεωμετρία που επιλέχθηκε για άλλο λόγο.

**Tests:** 5 νέα (σάρωση σε ευθύγραμμο τμήμα, μπάλα εκτός διαδρόμου, crossing
ποτέ σε σφιχτό τόξο, unreachable ποτέ δεν σαρώνεται, ήπιες ακτίνες παράγουν
διακριτές ακμές). **463 pure tests PASS.** Τρία υπάρχοντα tests ενημερώθηκαν
γιατί κωδικοποιούσαν το παλιό συμβόλαιο, ένα σενάριο budget-exhaustion χρειάστηκε
κλειστό διάδρομο για να μείνει για αυτό που ονομάζεται.

**Status:** ΑΝΑΜΟΝΗ ΑΠΟΦΑΣΗΣ. Ο κώδικας είναι στο working tree χωρίς commit,
γιατί στο shipped budget παράγει χειρότερη διαδρομή.

## #53 — Οροφή και μοχλοί: το περιθώριο υπάρχει, αλλά όχι εκεί που ψάχναμε

**Δάπεδο διαδρομής (ακριβής υπολογισμός στη διάταξη του #48):** ανοιχτό TSP από
τη start pose πάνω στις 9 μπάλες = **25.0 m**. Με την υποχρεωτική γεωμετρία
περασμάτων (9 x 1.3 m run-in/run-out) το αφελές δάπεδο είναι **36.7 m**. Η
πραγματική διαδρομή είναι **52.9 m** — δηλαδή **16 m πάνω από το δάπεδο**.
Περιθώριο υπάρχει.

**Οροφή συγχώνευσης — και κορεσμός.** Ελάχιστος αριθμός ευθύγραμμων περασμάτων
που καλύπτουν και τις 9 μπάλες, ανά πλάτος διαδρόμου:

| διάδρομος (half-width) | ελάχιστα περάσματα | μεγαλύτερη ομάδα |
| --- | --- | --- |
| 0.027 m (σημερινό effective) | 5 | 2 |
| 0.047 m | **4** | 3 |
| 0.17 m | 4 | 3 |
| 0.205 m | 4 | 3 |
| 0.40 m | 4 | 3 |

**Πάνω από ~0.05 m το πλάτεμα του διαδρόμου δεν αγοράζει τίποτα.** Οι μπάλες
απλώς δεν είναι πιο συγγραμμικές. Αυτό κλείνει οριστικά τη γραμμή «πλάτυνε τον
διάδρομο» των #49/#50: το ταβάνι είναι 4 περάσματα και πιάνεται ήδη στα 0.047.

**Μοχλοί που δοκιμάστηκαν (πραγματικός planner, budget 10000):**

| παραλλαγή | passes | shared | swept | μήκος | στροφή |
| --- | --- | --- | --- | --- | --- |
| σημερινό | 8 | 1 | 0 | 52.9 | 17.1 |
| `weight_pass_count` 2 / 4 / 8 | 8 | 1 | 0 | 52.9 | 17.1 |
| margin 0.03 / 0.02 / 0.01 | 8 | 1 | 0 | ~52.9 | ~17 |
| margin 0.0, σάρωση OFF | 7 | 2 | 0 | **47.6** | 14.3 |
| margin 0.0, σάρωση ON | 7 | 1 | 1 | 47.3 | 14.3 |

**Τρία αρνητικά αποτελέσματα, όλα χρήσιμα:**

1. **Το `weight_pass_count` είναι αδρανές** — 1 έως 8 δίνουν την ίδια διαδρομή.
   Ο solver δεν επιλέγει ανάμεσα σε 8 και 5 περάσματα· **η 5-πέρασμα διαδρομή δεν
   υπάρχει στον γράφο**. Το πρόβλημα είναι παραγωγή υποψηφίων, όχι κοστολόγηση.
2. **Η σάρωση σε connector (#52) αποδίδει 0.3 m από τα 5.3 m** της βελτίωσης
   (0.6%). Ο μηχανισμός είναι σωστός και δοκιμασμένος, αλλά δεν δικαιολογείται
   από τα δικά του νούμερα.
3. **Το κέρδος του margin είναι γκρεμός στο ακριβώς 0.0**, όχι κλίση (0.03/0.02/
   0.01 δεν αλλάζουν τίποτα). Εξαρτάται από ένα συγκεκριμένο ζεύγος που περνά το
   κατώφλι — **προσαρμοσμένο στη συγκεκριμένη διάταξη**, και το πληρώνει
   ξοδεύοντας ολόκληρο το περιθώριο ασφαλείας. Δεν προτείνεται ως ρύθμιση.

**Ο μόνος μοχλός που δεν έχει δοκιμαστεί και που δείχνει η ανάλυση κάλυψης:**
οι **ακριβείς ανά ζεύγος κατευθύνσεις** (κανόνας A του #48). Το πλέγμα των 16
κατευθύνσεων αρκεί για ζεύγη αλλά **όχι για τριάδες**: η κάλυψη 4 περασμάτων
απαιτεί ομάδες των 3, που θέλουν την ακριβή ευθεία. Ο planner πετυχαίνει 7-8 εκεί
που η γεωμετρία επιτρέπει 4.

**Status:** ο κώδικας του #52 παραμένει uncommitted. Επόμενο βήμα με τεκμηρίωση:
κανόνας A.

## #54 — Κανόνας A: οι ακριβείς ευθείες δόθηκαν, αλλά η ομαδοποίηση δεν τις δέχεται

**Υλοποίηση (working tree, ΔΕΝ έχει γίνει commit):** το `_pairwise_headings`
προσθέτει στο σύνολο υποψηφίων κάθε μπάλας την **ακριβή** κατεύθυνση προς κάθε
άλλη μπάλα, και προς τις δύο φορές διαδρομής. Υποψήφιοι ανά scan: 288 (από ~150).

**Μέτρηση στα πραγματικά δεδομένα του #48:**

| παραλλαγή | passes | shared | max/pass | swept | μήκος | στροφή |
| --- | --- | --- | --- | --- | --- | --- |
| πριν τον A | 8 | 1 | 2 | 0 | 52.9 | 17.1 |
| κανόνας A, budget 10000 | 7 | 2 | **2** | 0 | **64.9** | 19.5 |
| κανόνας A, budget 3000 | 7 | 2 | 2 | 0 | 70.4 | 19.1 |
| κανόνας A + margin 0.0 | 6 | 1 | 2 | 2 | **47.1** | **13.1** |

**Ο κανόνας A μόνος του χειροτερεύει τη διαδρομή** (52.9 → 64.9 m): προσθέτει
υποψηφίους που αραιώνουν την αναζήτηση και επιτρέπει συγχωνεύσεις γεωμετρικά
έγκυρες αλλά ακριβές σε διαδρομή. Μαζί με `margin 0.0` δίνει το καλύτερο μέχρι
τώρα (47.1 m / 13.1 rad έναντι 47.3 / 14.3 του margin μόνο), δηλαδή συνεισφορά
~0.2 m και ~1.2 rad.

**Η οροφή δεν πιάστηκε και ξέρουμε γιατί.** Η μεγαλύτερη ομάδα ανά πέρασμα μένει
**2 μπάλες**, ποτέ 3, ενώ η ανάλυση κάλυψης του #53 δείχνει ότι στα 0.047 m
υπάρχουν ομάδες των 3 που δίνουν 4 περάσματα. Αιτία στον κώδικα συγχώνευσης: το
`generate_shared_passes` ομαδοποιεί με **κλειδί ακριβούς float**
(`by_heading[candidate.heading_rad]`) και το `_build_shared_candidate`
απορρίπτει με `member.heading_rad != heading`. Για τρεις **σχεδόν** συγγραμμικές
μπάλες οι κατευθύνσεις i→j, i→k, j→k διαφέρουν ελαφρώς και δεν πέφτουν ποτέ στο
ίδιο κλειδί. Ο κανόνας A παρήγαγε τις σωστές ευθείες· η ομαδοποίηση δεν μπορεί
να τις αξιοποιήσει πέρα από ζεύγη.

**Τι θα χρειαζόταν πραγματικά:** ομαδοποίηση με **ανοχή** ή προσαρμογή ευθείας
(least-squares) στην ομάδα αντί για ισότητα float, ώστε τρεις σχεδόν
συγγραμμικές μπάλες να συγχωνεύονται γύρω από την κοινή τους ευθεία. Αυτό είναι
αλλαγή στο `generate_shared_passes`, όχι στο σύνολο κατευθύνσεων.

**Tests:** 463 pure PASS. Ένα test του candidate cap ενημερώθηκε ώστε να καλύπτει
και τις δύο συμπεριφορές (routable cap → PARTIAL, starved cap → μη εκτελέσιμο
PLANNING_TIMEOUT), γιατί ο πλουσιότερος χώρος υποψηφίων άλλαξε ποιο cap δίνει τι.

**Status:** ΑΝΑΜΟΝΗ ΑΠΟΦΑΣΗΣ — ο κανόνας A δεν προτείνεται για commit ως έχει
(μόνος του είναι οπισθοδρόμηση 12 m).

## #55 — Απαρίθμηση ευθειών: το δομικό ελάττωμα λύθηκε, αλλά ο περιοριστής είναι ο router

**Αλλαγή (working tree, ΔΕΝ έχει γίνει commit).** Το `collection_route_shared_pass`
ξαναγράφτηκε: αντί για συγχώνευση ανά-μπάλα υποψηφίων με **ισότητα float**
κατεύθυνσης, απαριθμεί τις **ευθείες που ορίζουν οι ίδιες οι μπάλες**, κρατά τις
μέγιστες ομάδες, προσαρμόζει ευθεία (principal axis) στην ομάδα και εκδίδει
**κατευθυνόμενο** πέρασμα ανά φορά διαδρομής. Η επιλογή ποιες ευθείες θα
χρησιμοποιηθούν **δεν** γίνεται εδώ: λιγότερα περάσματα δεν σημαίνει συντομότερη
διαδρομή, οπότε το καλύπτον υποσύνολο το διαλέγει ο router με το πλήρες κόστος.
Ο κανόνας A (#54) αφαιρέθηκε — οι ευθείες ζευγών παράγονται πλέον από την
απαρίθμηση. Τα `crossing_positions` κρατούν τις **πραγματικές** θέσεις μπαλών,
ώστε το `predicted_lateral_error` να είναι αληθινό αντί για μηδέν.

**Το δομικό ελάττωμα λύθηκε — αποδεδειγμένα.** Νέο test: τρεις **σχεδόν**
συγγραμμικές μπάλες (offsets 0.02 και −0.015) ομαδοποιούνται πλέον σε ένα
πέρασμα. Με την ισότητα float αυτό ήταν αδύνατο.

**Αλλά στα πραγματικά δεδομένα δεν έχει πού να δαγκώσει.** Η ανάλυση του #53
δείχνει, στον πραγματικό διάδρομο, **μηδέν** τριάδες στα 0.027 και **μία** στα
0.047. Η μέτρηση το επιβεβαιώνει: μέγιστη ομάδα ανά πέρασμα παραμένει **2**.

**Και η διαδρομή χειροτερεύει:**

| παραλλαγή | passes | shared | max | μήκος | στροφή |
| --- | --- | --- | --- | --- | --- |
| πριν (exact merge) | 8 | 1 | 2 | **52.9** | 17.1 |
| απαρίθμηση, budget 3000 | 6 | 3 | 2 | 64.0 | 15.6 |
| απαρίθμηση, budget 10000 | 7 | 2 | 2 | 62.1 | 19.0 |
| απαρίθμηση, budget 30000 | 6 | 3 | 2 | 61.4 | 16.1 |
| απαρίθμηση, budget 80000 | 7 | 2 | 2 | 59.6 | 17.4 |
| απαρίθμηση, cap 200/400/800 | 7 | 2 | 2 | 62.1 | 19.0 |

**Το cap δεν φταίει** (200/400/800 ταυτόσημα) και **το budget δεν αρκεί**: στα
80000 expansions φτάνει 59.6 m και δεν επιστρέφει ποτέ στα 52.9.

**Το συμπέρασμα ολόκληρης της έρευνας #49-#55:** ο περιοριστικός παράγοντας
**δεν** είναι η παραγωγή υποψηφίων. Δοκιμάστηκαν τέσσερις ανεξάρτητες
βελτιώσεις — πλάτεμα διαδρόμου (#49/#50), σάρωση σε connector (#52), ακριβείς
ανά ζεύγος κατευθύνσεις (#54), απαρίθμηση ευθειών (#55) — και **καμία** δεν
βελτίωσε τη διαδρομή· οι δύο τελευταίες τη χειροτέρευσαν, επειδή μεγαλώνουν τον
χώρο και το DFS branch-and-bound δεν τον εκμεταλλεύεται. Επιπλέον, 7 περάσματα
στα 62.1 m έναντι 8 στα 52.9 m αποδεικνύουν ότι **ο αριθμός περασμάτων δεν είναι
ο οδηγός του κόστους**: το κόστος είναι στη μεταφορά και την ευθυγράμμιση.

**Επόμενο βήμα με τεκμηρίωση:** ο **router**, όχι οι υποψήφιοι. Το δάπεδο είναι
36.7 m (TSP 25.0 + γεωμετρία περασμάτων 11.7) και ο solver παράγει 52.9-62.1.
Το DFS με coverage-first λεξικογραφικό score και admissible bound δεν κλείνει
αυτό το κενό σε καμία ρύθμιση που δοκιμάστηκε.

**Tests:** 467 pure PASS. Τα 7 tests της συγχώνευσης ξαναγράφτηκαν για το νέο
συμβόλαιο (+4 νέα: σχεδόν-συγγραμμική τριάδα, δύο φορές διαδρομής, πραγματικές
θέσεις crossing, αποκλεισμός τρίτης μπάλας εκτός διαδρόμου). Το candidate-cap
test χρειάστηκε τρίτη μπάλα, γιατί πλέον ένας υποψήφιος καλύπτει δύο μπάλες.

**Status:** ΑΝΑΜΟΝΗ ΑΠΟΦΑΣΗΣ — αρχιτεκτονικά σωστό, μετρήσιμα χειρότερο στα
δεδομένα που έχουμε.

## #56 — Ιεραρχική δρομολόγηση σε μπλοκ: το πρώτο πραγματικό κέρδος

**Ιδέα (χρήστη):** συσταδοποίηση των μπαλών σε μπλοκ των 2..n, με τις μεμονωμένες
ως μπλοκ μεγέθους 1, και σύνδεση των μπλοκ για την τελική διαδρομή — κάθε μπλοκ
σαν αυτοτελής μονάδα με είσοδο και έξοδο.

**Δομή στα πραγματικά δεδομένα.** Single-linkage στις 9 μπάλες του #48:

| κατώφλι | μπλοκ | μεγέθη | διατάξεις |
| --- | --- | --- | --- |
| 1.5 m | 6 | 2+2+2+1+1+1 | 720 |
| 2.0 m | 5 | 4+2+1+1+1 | 120 |
| 2.5 m | 4 | 6+1+1+1 | 24 |

Η διάταξη γίνεται **εξαντλητικά λύσιμη**, εκεί που ο επίπεδος solver δεν
συγκλίνει ούτε στα 80000 expansions (#55).

**Πρωτότυπο (scratchpad, ΟΧΙ παραγωγή):** διαμέριση → ανά μπλοκ εξαντλητική
εσωτερική λύση (καλύπτοντα σύνολα από τους υποψηφίους του #55 + μεταθέσεις, με
Dubins connectors) → καθολική διάταξη μπλοκ εξαντλητικά. Κάθε μπλοκ εξάγει έως 6
εναλλακτικές `(είσοδος, έξοδος, μήκος)` και **ο καθολικός sequencer διαλέγει** —
δεν κλειδώνει το μπλοκ μόνο του, γιατί έχει μετρηθεί δύο φορές ότι η τοπική
βελτιστοποίηση χάνει καθολικά (#54, #55).

**Αποτέλεσμα (μήκος διαδρομής, ίδιο scan):**

| grid | thr 1.5 | thr 2.0 | thr 2.5 | thr 3.0 |
| --- | --- | --- | --- | --- |
| 4 | 50.4 | 49.2 | 45.8 | 45.8 |
| 8 | 45.1 (77 s) | 51.0 | 45.2 | 45.0 |
| 16 | 45.1 (82 s) | 53.5 | **43.5 (0.6 s)** | 44.2 |

**Βέλτιστο: 43.5 m σε 0.6 s** (κατώφλι 2.5, grid 16) έναντι **52.9 m σε 13-25 s**
του επίπεδου planner. Δηλαδή **−9.4 m (−18%)** και **~30x ταχύτερα**.

**Το κενό προς το δάπεδο έκλεισε κατά 58%:** από 16.2 m πάνω από τα 36.7 m σε
6.8 m.

Η ταχύτητα έχει ξεχωριστή σημασία: στο Pi το planning έτρωγε ~100 s sim time και
πάγωνε το status publishing (#48).

**Πρώτη προσέγγιση της έρευνας #49-#56 που βελτιώνει τη διαδρομή.** Οι
προηγούμενες τέσσερις (πλάτεμα διαδρόμου, σάρωση connector, ακριβείς
κατευθύνσεις, απαρίθμηση ευθειών) δεν βελτίωσαν ή χειροτέρευσαν. Και το #55
ανασταίνεται: η απαρίθμηση ευθειών ήταν άχρηστη σε επίπεδη αναζήτηση 9 μπαλών,
αλλά **μέσα σε μπλοκ 3-4 μπαλών είναι φθηνή και αξιοποιήσιμη**.

**Επιφυλάξεις που πρέπει να λυθούν στην παραγωγή:**
- Τα αποτελέσματα είναι **μη μονότονα** στο κατώφλι (2.0 χειρότερο από 1.5 και
  2.5 στα grid 8/16). Αιτία: το πρωτότυπο κρατά μόνο 6 εναλλακτικές ανά μπλοκ με
  χονδρικό dedup σε στρογγυλεμένη pose, οπότε χάνει καλές επιλογές.
- Η διαμέριση **απαγορεύει** διαδρομές που μπλέκουν μπλοκ· μπορεί να αποκλείσει
  το πραγματικό βέλτιστο. Το αντάλλαγμα μετρήθηκε και είναι υπέρ μας.
- Το κατώφλι είναι νέα ελεύθερη παράμετρος (2.5 m εδώ, να επικυρωθεί σε άλλες
  διατάξεις).
- Η αρχική συνδυαστική εξερράγη (>10 min)· έγινε 0.5 s με παραγωγή κάθε
  καλύπτοντος συνόλου **μία** φορά (canonical pivot) και memoization των Dubins.

**Status:** πρωτότυπο μόνο. Η παραγωγή θέλει νέο router· ο κώδικας του #55
(απαρίθμηση ευθειών) γίνεται ο intra-block generator και δικαιολογείται πλέον.

## #57 — Επαναξιολόγηση του block router: το μπλοκ έγινε περιορισμός, όχι ευρετικό

**Αφορμή (χρήστης):** ο σκοπός της συσταδοποίησης ήταν να **ανακαλύπτει** ομάδες
μπαλών που μαζεύονται σε ένα σχεδόν ευθύ πέρασμα — όχι να γίνει το μπλοκ
αδιαίρετη μονάδα δρομολόγησης. Ζητήθηκε επαναξιολόγηση πριν από οποιαδήποτε
διόρθωση.

**Μέθοδος:** μόνο μετρήσεις, καμία αλλαγή κώδικα. (α) offline replay του
πραγματικού scan `runtime/route_audit/clean_current_20260728_1315` (10 μπάλες,
πραγματικό `court_boundary.json`, τρέχον `collection_route.yaml`) μέσα από τον
block router και μέσα από τον επίπεδο `solve_global_route`· (β) εχθρικές
γεωμετρίες στο συνθετικό γήπεδο των fixtures· (γ) instrumentation του
`partition_balls` / `block_options` / `_sequence_blocks`.

**Πραγματικό scan — ο block router χάνει σε κάθε άξονα ποιότητας:**

| router | κάλυψη | μήκος | connector | pass | στροφή | περάσματα | χρόνος |
| --- | --- | --- | --- | --- | --- | --- | --- |
| επίπεδος (global solver) | 10/10 | **55.9** | 40.1 | 15.3 | **15.2** | **6** (1,3,2,1,1,2) | 22.6 s |
| μπλοκ (τρέχων) | 10/10 | 58.7 | 44.3 | 14.0 | 23.7 | 8 (1,1,1,1,1,3,1,1) | **8.0 s** |

Το κέρδος των 43.5 m του #56 **δεν αναπαράγεται** στην παραγωγή. Μένει μόνο το
κέρδος ταχύτητας, το οποίο προέρχεται από τα lazy/memoized Dubins (`best_connector`)
και **όχι** από τη διαμέριση: ο επίπεδος χτίζει 480.000 ακμές εκ των προτέρων.

**Εχθρικές γεωμετρίες — ο block router καταστρέφει εκτελέσιμες διαδρομές:**

| διάταξη | μπλοκ | επίπεδος |
| --- | --- | --- |
| 3 συγγραμμικές ανά 4 m (spread 8 m > threshold 2.5) | 3 χωριστά περάσματα | 2 περάσματα + **connector που μαζεύει** τη μεσαία |
| δακτύλιος 6 μπαλών (R=1 m) | **ΜΗΔΕΝ διαδρομή, 0/6** | partial 2/6, 8.7 m |
| 3 σκόρπιες, cap υποψηφίων 10 | **ΜΗΔΕΝ διαδρομή, 0/3** | partial 2/3, 9.7 m |
| μπάλα πάνω στη γραμμή μεταφοράς (5 μπάλες) | partial **1/5** | **feasible 5/5**, ένα 3-ball pass + connector που μαζεύει 2 |
| A1 → B → A2 (4 μπάλες) | partial **2/4** | **feasible 4/4** |

**Πού ακριβώς σπάει (instrumented):**

1. *Πλήρης μάσκα.* `_sequence_blocks` δέχεται μόνο `mask == full`. Στις 3 σκόρπιες
   και τα 3 μπλοκ **routable**, καμία διάταξη δεν κλείνει και επιστρέφεται
   `None` → μηδενική διαδρομή αντί για την προφανή μερική.
2. *Πλήρης κάλυψη μπλοκ.* Το `block_options` δέχεται μόνο cover που καλύπτει
   **όλο** το μπλοκ και του οποίου **κάθε** διαδοχικό ζεύγος συνδέεται. Στον
   δακτύλιο των 6 (57 usable single passes, καμία μπάλα χωρίς υποψήφιο) βγάζει
   `options=0` — ο σφιχτός κύκλος είναι ακριβώς εκεί που τα CSC connectors
   αποτυγχάνουν — και **χάνονται και οι 6 μπάλες**. Το ίδιο για το ζεύγος
   `b1,b2` στα 0.6 m.
3. *Κατώφλι ως γεωμετρία.* Το `partition_balls` απορρίπτει κοινό πέρασμα με
   spread > `cluster_threshold_m`, και το `block_options` κρατά μόνο υποψήφιους
   `⊆ block`: μια έγκυρη ευθεία που περνά από δύο μπλοκ **πετιέται**.
4. *`maximum_block_balls` με λάθος σημασιολογία.* Περιορίζει τον αριθμό
   **περασμάτων** στο cover, όχι τις μπάλες: μπλοκ 6 μπαλών με όριο 5 δεν έχει
   καμία λύση από μονήρη περάσματα.
5. *Απαρίθμηση covers εξαρτώμενη από τη σειρά.* Το `start_index` στο `build()`
   χάνει έγκυρα covers. Απόδειξη στα σύνολα: στόχος `{a,b,c}`, usable
   `[('b',), ('c','a')]` (ακριβώς η σειρά που παράγει το `_merge_candidates`,
   ταξινομημένη κατά `covered_ball_ids`) → **κανένα** cover· με αντεστραμμένη
   σειρά → βρίσκεται.
6. *Οι connectors δεν μαζεύουν πια.* Ο block router καλεί `best_connector`
   **χωρίς** `balls=`, οπότε `swept_crossings` είναι πάντα κενό: η δυνατότητα του
   #52 χάθηκε και μια μπάλα που την πατά ο connector προγραμματίζεται σαν
   ανέγγιχτη.
7. *Χωρίς πραγματικό όριο αναζήτησης.* Το `max_search_expansions` **δεν
   διαβάζεται** από το `collection_route_blocks`. Το κόστος είναι
   `2^blocks × options²` με `options` απεριόριστα: 3 μπάλες με cap 200 δεν
   τερμάτισαν σε 5 λεπτά· το μπλοκ των 6 στο πραγματικό scan κόστισε 5.0 s μόνο
   και μόνο επειδή το (λανθασμένο) όριο 5 έκοψε σχεδόν όλα τα covers.
8. *Λάθος αποδόσεις αιτίας.* Το `_ball_results` του blocks δίνει
   `PLANNING_BUDGET` σε **όλες** τις αμάζευτες όταν έχει χτυπήσει το cap
   υποψηφίων, και δεν παράγει ποτέ `TURN_RADIUS`· η διάκριση
   turn-radius/clearance/budget/route-conflict του `_turning_only_unreachable`
   χάθηκε.
9. *Επικίνδυνο pose-tolerance dominance.* Το `_non_dominated` θεωρεί ίδιες τις
   poses σε 0.05 m/0.05 rad — διαφορά που αλλάζει τη σύνδεση/σύγκρουση.
10. *Σπάσιμο σχήματος.* Το `block_routing` μπήκε ως υποχρεωτικό group με
    `schema_version` αμετάβλητο (`collection-route/v1`): **κανένα** αποθηκευμένο
    artifact δεν φορτώνει πια (`DomainValidationError` στο
    `collection_route_adaptive_replay`).
11. *4 tests ήδη κόκκινα* στο working tree, μεταξύ τους το
    `test_budget_exhaustion_returns_executable_partial_plan` — δηλαδή η ίδια η
    αναλλοίωτη «ποτέ μηδενική διαδρομή από budget» έχει ήδη σπάσει.

**Συμπέρασμα:** το πρόβλημα δεν είναι λίστα ελαττωμάτων αλλά η αφαίρεση. Το
μπλοκ πρέπει να είναι ευρετικό **παραγωγής υποψηφίων και σειράς επέκτασης**, όχι
ατομική μονάδα δρομολόγησης. Το πρωτεύον αντικείμενο είναι το **πέρασμα/διάδρομος**.

**Status:** καμία αλλαγή κώδικα — αναμονή έγκρισης του διορθωμένου μοντέλου
(αναζήτηση πάνω σε περάσματα, clusters ως ευρετικό + προαιρετικές macro-κινήσεις,
best-so-far αναλλοίωτη, lazy memoized connectors με `balls=`, φραγμένο
`max_search_expansions`).

### #57β — Διορθώσεις του μοντέλου από τον χρήστη (πριν την υλοποίηση)

Τρεις διορθώσεις στο προτεινόμενο μοντέλο του #57, εγκεκριμένες ως προδιαγραφή:

1. **Χωρίς απαίτηση ξένων συνόλων.** Ένα πέρασμα παραμένει έγκυρος διάδοχος
   ακόμη κι αν επικαλύπτει ήδη μαζεμένες μπάλες. Κριτήριο:
   `new_coverage = (pass ∪ connector swept) − already_covered ≠ ∅` και εκτελέσιμη
   γεωμετρία. Η φυσική κατάσταση είναι «ποιες μπάλες μαζεύτηκαν», όχι «ποιο
   πέρασμα κατέχει ποια μπάλα».
   *Συνέπεια στο συμβόλαιο:* το `CollectionRoutePlan._validate_segments`
   (γραμμή 806) απαιτεί κάθε μπάλα να εμφανίζεται σε **ένα** segment. Άρα κάθε
   segment δηλώνει μόνο το `new_coverage` του· η επαναδιέλευση πάνω από ήδη
   μαζεμένη μπάλα είναι γεωμετρία χωρίς δήλωση (και φυσικά ακίνδυνη: η μπάλα
   είναι πια στο καλάθι).
2. **Μονοτονία ως προς το budget.** `budget₂ > budget₁ ⇒ coverage₂ ≥ coverage₁`
   και, με ίση κάλυψη, `cost₂ ≤ cost₁`. Εξασφαλίζεται δομικά: η σειρά επέκτασης
   είναι ανεξάρτητη του budget και το incumbent είναι running max σε ολική
   διάταξη· το budget μόνο **σταματά** τον βρόχο, ποτέ δεν αλλάζει τη διακλάδωση.
3. **`PLANNING_BUDGET` = επιστημική αβεβαιότητα**, όχι «δεν εξετάστηκε ποτέ».
   Γεωμετρικός κωδικός μόνο όταν η αποτυχία **αποδείχθηκε**· αλλιώς
   `PLANNING_BUDGET`. Απαιτεί συσσώρευση evidence ανά μπάλα (rejection codes από
   candidate/connector/terminal) και bounded απόδειξη μη-εφικτότητας στο τέλος.

**Δύο ρητές αρχιτεκτονικές αναλλοίωτες:**

- Οι ευρετικές επηρεάζουν **τι εξερευνάται πρώτα**, ποτέ **τι είναι εφικτό**.
- Καμία συσταδοποίηση, διαμέριση, cap υποψηφίων, pruning ή budget δεν επιτρέπεται
  να μετατρέψει γνωστή εκτελέσιμη μερική διαδρομή σε μηδενική.

## #58 — Ο router περασμάτων: υλοποίηση της διορθωμένης αφαίρεσης

**Αλλαγή (working tree).** Το `collection_route_blocks` **διαγράφηκε**. Στη θέση
του μπήκε το `collection_route_router`: φραγμένη anytime best-first αναζήτηση
**πάνω σε περάσματα**, με κατάσταση `(covered_mask, tail)`, ακριβή κυριαρχία
(dominance) ανά κατάσταση, incumbent που ενημερώνεται **πριν** από κάθε
pruning, και clusters μόνο ως (α) σειρά επέκτασης και (β) προαιρετικά macros.

**Νέα/αλλαγμένα αρχεία**
- `collection_route_router.py` (νέο) — η αναζήτηση.
- `collection_route_evidence.py` (νέο) — `BallEvidence` + επιστημική απόδοση αιτίας.
- `collection_route_plan_builder.py` (νέο) — κατασκευή segments με `declare_only`
  και `plan_id` από την πλήρη γεωμετρία· ο `global_solver` καταναλώνει το ίδιο.
- `collection_route_cost.py` (νέο) — ένας τύπος κόστους, γραμμικός/αθροιστικός.
- `collection_route_schema_migration.py` (νέο) — offline migration v1→v2.
- `collection_route_types.py` — `schema_version: collection-route/v2`,
  `block_routing` → `cluster_heuristics`, νέοι κωδικοί `CONNECTOR_CLEARANCE`,
  `NO_TERMINAL`.
- `collection_route_connector_graph.py` — `link_poses` επιστρέφει
  `ConnectorAttempt(edge, rejections)`· τα `balls` είναι πλέον **υποχρεωτικά**
  στη διαδρομή του router, ώστε να μην ξαναχαθεί η σάρωση σε connector.

**Οι τρεις διορθώσεις του #57β υλοποιημένες**
1. *Χωρίς ξένα σύνολα.* Κίνηση επιτρεπτή όταν
   `(pass ∪ connector swept) − covered ≠ ∅`. Επειδή το συμβόλαιο του plan απαιτεί
   κάθε μπάλα σε **ένα** segment, κάθε segment δηλώνει μόνο το `new_coverage`·
   ένα πέρασμα πάνω από ήδη μαζεμένη μπάλα οδηγείται κανονικά και δεν δηλώνεται.
   Πέρασμα που δεν δηλώνει τίποτα εκδίδεται ως `transit_pass_segment`.
2. *Μονοτονία.* Η σειρά επέκτασης δεν διαβάζει ποτέ το budget· το budget μόνο
   σταματά τον βρόχο. Μετρήθηκε στο πραγματικό scan: 20→60.82, 50→60.82,
   200→54.92, 3000→**49.85 m**, κάλυψη 10/10 σε όλα.
3. *`PLANNING_BUDGET` = αβεβαιότητα.* Γεωμετρικός κωδικός μόνο όταν η αναζήτηση
   **εξάντλησε** το μέτωπο. Απόκλιση από το εγκεκριμένο σχέδιο: το bounded
   «proof pass» **δεν** χρειάστηκε — η ίδια η εξαντλητική αναζήτηση παρέχει την
   απόδειξη, φθηνότερα και με την ίδια εγγύηση.

**Μετρήσεις στο πραγματικό scan** (10 μπάλες, ίδιο boundary/config):

| planner | κάλυψη | μήκος | στροφή | περάσματα | χρόνος |
| --- | --- | --- | --- | --- | --- |
| επίπεδος | 10/10 | 55.92 | 15.21 | 6 | 22.6 s |
| μπλοκ | 10/10 | 58.73 | 23.74 | 8 | 8.0 s |
| **router (3000 exp)** | **10/10** | **49.85** | 17.77 | 8 | 18.1 s |
| router (200 exp) | 10/10 | 54.92 | 20.07 | 8 | 10.2 s |
| router (shipped 1 s wall clock) | 10/10 | 60.82 | 20.88 | 6 | 2.0 s |

**Το ανοιχτό θέμα είναι η ταχύτητα, όχι η ποιότητα.** Με χρόνο, ο router κερδίζει
−6.1 m έναντι του επίπεδου (−10.9%) και −8.9 m έναντι του block. Αλλά με το
shipped `maximum_planning_time_s: 1.0` η αναζήτηση κόβεται στα **8 expansions**
και δίνει 60.82 m — χειρότερα και από τους δύο. Αιτία: κάθε expansion συνδέει
την ουρά με **όλους** τους 200 υποψηφίους (~0.25 s το πρώτο). Δύο διορθώσεις για
τη Φάση 5, και οι δύο διατηρούν τη σημασιολογία: (α) lazy/partial expansion
(ο κόμβος επιστρέφει στο μέτωπο αντί να παράγει όλους τους διαδόχους μαζί),
(β) φθηνό φίλτρο απόστασης πριν το CSC (αποκλείει μόνο ό,τι θα απέρριπτε ούτως ή
άλλως το `max_connector_length_m`). **Η στροφή (17.77) παραμένει πάνω από το
15.21 του επίπεδου** — δεν έγινε καμία ρύθμιση βαρών, σκόπιμα.

**Tests:** 384 pure PASS (+51 νέα: 33 adversarial router, 6 optimality έναντι
brute force, 6 plan builder, 6 migration) + 3 real-scan quality. Τα 4 κόκκινα
του #57 έκλεισαν. Δύο υπάρχοντα tests ξαναγράφτηκαν επειδή η **παλιά** τους
προϋπόθεση έπαψε να ισχύει: το budget=1 δεν παράγει πια μερική διαδρομή (ο
router κλείνει διαδρομή στο πρώτο expansion), οπότε το «μερικό» πλάνο του
executor προέρχεται πλέον από γεωμετρία.

**Status:** Φάση 4 ολοκληρωμένη, αναμονή για Φάση 5 (επιδόσεις + σύγκριση σε
πραγματικές διατάξεις).

## #59 — Φάση 5: πού πήγαινε ο χρόνος, και τι κερδίζει η συσταδοποίηση στην πράξη

**Προφίλ πριν από κάθε αλλαγή** (πραγματικό scan, 200 υποψήφιοι, 3000 expansions,
instrumented ⇒ ~3× πιο αργό από το καθαρό run):

| μετρικό | τιμή |
| --- | --- |
| link requests από τον router | 676.346 (cache hit 95.5%, misses 30.281) |
| Dubins alternatives | 363.372 |
| απορρίφθηκαν **πριν** τη materialization | 351.684 (96.8%) |
| materialized paths | 11.688 |
| segment collision checks | 165.977 |
| `_segment_hits_inflated_polygon` | 1.492.871 |
| **χρόνος σε collision checking** | **48.8 s / 59.0 s = 82.8%** |
| κόστος ανά expansion (πρώτα 20) | 400-580 ms |

Το αναλυτικό gate δούλευε ήδη· ο πραγματικός ένοχος ήταν ο **έλεγχος
σύγκρουσης**, και μέσα σε αυτόν το ότι το `_polygon_edges` ξαναέχτιζε τα ίδια
ζεύγη ακμών 1,5 εκατ. φορές.

**5B — Επιτάχυνση με ταυτόσημες ετυμηγορίες.** (α) Παράγωγη γεωμετρία γηπέδου
(ακμές, AABB, εσωτερικά ημιεπίπεδα) υπολογισμένη **μία** φορά και κρατημένη πάνω
στο `CourtModel` ως μη-πεδίο. (β) Για **κυρτό** navigable polygon, ο έλεγχος
«εντός με clearance» γίνεται μία σάρωση ημιεπιπέδων και για τα δύο άκρα — από
την κυρτότητα προκύπτει ότι καλύπτει και τη διέλευση και την απόσταση από το
σύνορο. (γ) AABB early-out ανά εμπόδιο. (δ) Ευκλείδειο κάτω φράγμα πριν το
Dubins (`euclid > max_connector_length_m` ⇒ καμία διαδρομή δεν χωράει).

*Απόδειξη διατήρησης σημασιολογίας:* ο παλιός ορισμός κρατήθηκε στα tests και
συγκρίνεται με τον νέο σε ~30.000 τυχαία τμήματα ανά γήπεδο (ορθογώνιο, ρόμβος,
**μη κυρτό** L, με/χωρίς εμπόδια), συν τμήματα κολλημένα στο σύνορο και γύρω από
κορυφές εμποδίων: **μηδέν διαφωνίες**. Η διαδρομή του πραγματικού scan και όλοι
οι μετρητές (link/Dubins/materialize/collision) έμειναν **αριθμητικά ίδιοι**.

| | πριν | μετά |
| --- | --- | --- |
| instrumented runtime | 58.97 s | **11.24 s** |
| collision checking | 48.84 s (82.8%) | 1.63 s (14.5%) |
| segment collision check | 47.25 s | 0.35 s |
| καθαρό run, 3000 expansions | 18.06 s | **3.06 s** |

**5C — Οκνηρή, επαναλήψιμη επέκταση.** Μια κατάσταση δεν συνδέεται πια με τους
~200 υποψηφίους πριν προχωρήσει η αναζήτηση: παραδίδει φραγμένη παρτίδα
διαδόχων και **επιστρέφει στο μέτωπο κρατώντας τους υπόλοιπους**. Κανένας
διάδοχος δεν πετιέται· άδειο μέτωπο εξακολουθεί να σημαίνει πλήρη απαρίθμηση.
Νέο test: για κάθε batch (1, 3, 8, ∞) η **εξαντλητική** λύση είναι ίδια, και το
COMPLETE βέλτιστο ταυτίζεται με brute force σε κάθε batch.

**5E — Η συσταδοποίηση: τα macros πληρώνουν, το ordering bias όχι.** Με σταθερό
χρονικό παράθυρο στο πραγματικό scan:

| ρύθμιση | 1η διαδρομή | 1η 10/10 | ≤58.73 | ≤55.92 | ≤52 | καλύτερο |
| --- | --- | --- | --- | --- | --- | --- |
| clusters=on macros=on | 0.13 | 0.13 | 0.19 | 0.23 | 0.85 | 45.03 |
| clusters=off macros=on | 0.13 | 0.13 | 0.16 | **0.21** | **0.77** | 45.03 |
| clusters=on macros=off | 0.00 | 0.00 | 0.73 | **6.55** | 6.68 | 45.03 |
| clusters=off macros=off | 0.00 | 0.00 | 0.69 | 2.65 | 3.25 | **50.76** |

Χωρίς macros η διαδρομή είτε αργεί 10-30× να φτάσει το baseline είτε δεν το
φτάνει ποτέ. Το cluster bias στην προτεραιότητα του μετώπου **καθυστερούσε**
(6.55 s έναντι 2.65 s χωρίς αυτό) και **αφαιρέθηκε**. Η συσταδοποίηση παραμένει —
ως γεννήτρια macros, που είναι άλλος μηχανισμός.

**5F/5G — Σχήμα διαδρομής: όλη η καμπυλότητα είναι στους connectors.**

| | router (shipped) | flat |
| --- | --- | --- |
| μήκος | 45.03 | 54.49 |
| στροφή | 14.07 | 15.83 |
| περάσματα | 6 (avg 1.67 μπάλες) | 6 (avg 1.67) |
| **στροφή μέσα στα περάσματα** | **0.000 rad** | **0.000 rad** |
| στροφή στους connectors | 14.07 (100%) | 15.83 (100%) |

Τα περάσματα είναι εξ ορισμού ευθύγραμμα. Άρα η «περίσσεια καμπυλότητας» του #58
**δεν** οφειλόταν σε κατακερματισμό υποψηφίων ούτε στο objective: οφειλόταν στη
γεωμετρία των connectors και στο πόσο βαθιά είχε προλάβει να ψάξει ο router.
**Καμία ρύθμιση βαρών δεν δικαιολογείται** — το gate καμπυλότητας περνιέται πλέον.

**Σημείο λειτουργίας.** Το `successor_batch_size` είναι πραγματικό trade-off:
batch=1 είναι το καλύτερο στο (συσταδοποιημένο) πραγματικό scan αλλά **χάνει
κάλυψη** σε σκόρπιες τυχαίες διατάξεις. Επιλέχθηκε **batch=4 / 150k expansions**,
το μόνο σημείο που κερδίζει και στα δύο:

| | κάλυψη | μήκος | στροφή | χρόνος |
| --- | --- | --- | --- | --- |
| flat baseline | 10/10 | 55.92 | 15.21 | 22.6 s |
| block router (#56-#57) | 10/10 | 58.73 | 23.74 | 8.0 s |
| Φάση 4 (shipped 1 s) | 10/10 | 60.82 | 20.88 | ~2 s |
| **Φάση 5 (shipped)** | **10/10** | **45.03** | **14.07** | **3.1 s** |

**Τυχαίες διατάξεις (24 layouts, 5-14 μπάλες):** κάλυψη ≥ flat σε **24/24**,
ντετερμινιστικό 24/24, καμία διακοπή από wall clock, χρόνος 1.14 s έναντι 3.21 s
του flat. **Αλλά** μέσο μήκος 45.32 έναντι 40.59 του flat (+11.6%) και στροφή
16.24 έναντι 13.68. Δηλαδή: ο router κερδίζει καθαρά εκεί που υπάρχει **δομή**
(συγγραμμικές/συσταδοποιημένες μπάλες — ο λόγος που φτιάχτηκε) και υστερεί σε
μήκος σε τελείως σκόρπιες διατάξεις, χωρίς ποτέ να χάνει κάλυψη.

**Tests:** 553 pure PASS, 2 skipped (+20: 11 collision-equivalence, 5 lazy
expansion, 4 batch-invariant optimality). Ο πλήρης κύκλος τρέχει σε 17 s.

**Status:** Φάση 5 ολοκληρωμένη. Ανοιχτό και τεκμηριωμένο: το κενό μήκους σε
σκόρπιες διατάξεις. Υποψία (ΑΜΕΤΡΗΤΗ): με σταθερή κάλυψη το μέτωπο ξοδεύει
budget σε πολλές ισοδύναμης κάλυψης καταστάσεις, ενώ το DFS του flat κλαδεύει με
το admissible cost bound. Απαιτεί έγκριση πριν από οποιαδήποτε αλλαγή στρατηγικής.

## #60 — Φάση 6 (διαγνωστική): το κενό δεν είναι «σκόρπιες διατάξεις», είναι μέγεθος έναντι budget

**Καμία αλλαγή στον planner.** Όλα τα εργαλεία είναι εξωτερικά (monkeypatching).

**6A — κατανομή 24 διατάξεων.** Ο μέσος όρος +11.6% του #59 κρύβει τη διανομή:

| κατηγορία | πλήθος |
| --- | --- |
| router καλύτερος | 2 |
| ίσος (<0.5%) | **11** |
| 0-5% χειρότερος | 3 |
| 5-10% | 2 |
| 10-20% | 3 |
| >20% | 3 |

**Διάμεσος: 0.0%.** Στις 11 «ίσες» διατάξεις το μήκος είναι **ακριβώς** ίδιο με
του flat (11/11), δηλαδή βρίσκεται η ίδια διαδρομή. Κάλυψη ≥ flat σε **24/24**.

**Κρίσιμος διαχωρισμός:** διατάξεις που **χτύπησαν** το όριο 150k expansions →
μέσο κενό **+15.4%**· όσες τερμάτισαν κάτω από το όριο → **+0.0%**.

**Μεθοδολογικό εύρημα:** το layout-19 (+77.5%, το χειρότερο) μαζεύει **10 μπάλες
έναντι 9** του flat — μεγαλύτερη διαδρομή για μεγαλύτερη κάλυψη, δηλαδή σωστή
συμπεριφορά, όχι αστοχία. Χωρίς αυτό, το κενό πέφτει από +11.6% σε **+5.4%**.

**6B — σύγκλιση με το budget.** Σε κάθε μετρημένη διάταξη η ποιότητα βελτιώνεται
μονότονα και φτάνει ή ξεπερνά το flat:

| layout | flat | 1k | 25k | 150k | 500k | 1.5M / COMPLETE |
| --- | --- | --- | --- | --- | --- | --- |
| 11 | 26.71 | 41.51 | 39.29 | 30.59 | **26.71** (COMPLETE) | — |
| 3 | 35.29 | 39.17 | 38.72 | 36.29 | **33.95** (COMPLETE) | — |
| 6 | 42.79 | 42.97 | 39.42 | 39.42 | 33.63 | **33.63** (COMPLETE) |
| 1 | 61.06 | 52.94* | 81.98 | 70.15 | 66.82 | **53.91** |
| 16 | 68.12 | 80.71* | 97.92 | 90.32 | 86.45 | 79.95 |

*χαμηλότερη κάλυψη σε εκείνο το σημείο.

**6C — COMPLETE έναντι flat έναντι brute force.** Και στις 5 περιπτώσεις
**Case 1** (πρόβλημα αναζήτησης, όχι αναπαράστασης):

| layout | flat | router COMPLETE |
| --- | --- | --- |
| 3 | 35.29 | **33.95** |
| 6 | 42.79 | **33.63** |
| 9 | 14.81 | 14.81 |
| 11 | 26.71 | 26.71 |
| 15 | 11.30 | 11.30 = **brute force 60.383 cost** |

**6D — η υπόθεση των connectors ΚΑΤΑΡΡΙΦΘΗΚΕ με μέτρηση.** Υποψία ήταν ότι ο
router κρατά μόνο τον **συντομότερο** connector ανά ζεύγος poses ενώ ο flat έχει
και τους 12. Μετρήθηκε: **0 από 51** connectors πραγματικών flat διαδρομών είναι
μη-αναπαραστάσιμοι, και σε **348** δειγματοληπτημένα ζεύγη ο συντομότερος είναι
**πάντα** και ο φθηνότερος και ποτέ δεν σαρώνει λιγότερες μπάλες. Οι δύο planners
μοιράζονται ακριβώς τον ίδιο χώρο υποψηφίων.

**6E — πού πάει το budget.** Η μέγιστη κάλυψη βρίσκεται πολύ νωρίς (expansion
26-9159) και **96-100% του budget ξοδεύεται μετά** από αυτό — αλλά **παράγει**
βελτιώσεις (π.χ. layout-16: −94.4 κόστος, layout-11: −84.7). Άρα η υπόθεση του
#59 («σπαταλιέται σε ισοδύναμης κάλυψης καταστάσεις») **δεν επιβεβαιώνεται όπως
διατυπώθηκε**: το budget αποδίδει, απλώς αργά. Τα pops μοιράζονται σε **μεσαία**
επίπεδα κάλυψης (layout-16 στο 1.5M: 40% στο 9, 30% στο 10, 19% στο 8, ενώ το
μέγιστο είναι 12). Το **μέτωπο μένει μικροσκοπικό (7-23 στοιχεία)** με 132-200
υποψηφίους: dominance + admissible bounds κόβουν σχεδόν τα πάντα, οπότε η
αναζήτηση ξανα-επισκέπτεται λίγες καταστάσεις και τρώει το budget σε re-pops
(με batch=4 μια κατάσταση χρειάζεται ~50 pops για να εξαντληθεί).

**6F — αποσύνθεση κόστους (6 χειρότερες):** μήκος περασμάτων +3.21 m,
**connectors +14.36 m**, ίδιος αριθμός περασμάτων (8.33 έναντι 8.50) και
connectors (8.67 έναντι 8.50). Δηλαδή **ίδια δομή, χειρότερη σειρά** — και το 6D
απέδειξε ότι κάθε connector ξεχωριστά είναι ο ίδιος. Άρα: **σειριοποίηση**.

**6G — συσχετίσεις.** Το κενό συσχετίζεται με **μέγεθος**, όχι με διασπορά:

| χαρακτηριστικό | r |
| --- | --- |
| reachable μπάλες | +0.55 |
| πλήθος υποψηφίων | +0.55 |
| πλήθος μπαλών | +0.50 |
| χτύπησε το cap | +0.43 |
| COMPLETE | **−0.43** |
| **διασπορά (dispersion)** | **−0.12** |
| **μέση απόσταση πλησιέστερου** | **−0.10** |
| πλήθος συστάδων | +0.06 |

Ο όρος «scattered-layout gap» του #59 ήταν **λάθος**: δεν υπάρχει συσχέτιση με
τη διασπορά. Είναι κενό **μεγέθους έναντι budget**.

**6H — τα macros δεν αλλοιώνουν το βέλτιστο.** Σε 5 διατάξεις, COMPLETE με και
χωρίς macros δίνει **ταυτόσημο plan_id**, με διαφορετικό αριθμό expansions
(π.χ. 416.759 έναντι 391.770) — δηλαδή αλλάζουν τη σειρά ανακάλυψης, όχι τον
χώρο λύσεων, όπως απαιτεί το συμβόλαιο.

**Ταξινόμηση:** **SEARCH_EFFICIENCY** (κύριο) + **BENCHMARK_ARTIFACT**
(δευτερεύον: διάμεσος 0.0%, ένα layout με ασύμβατη κάλυψη). **ΟΧΙ**
CANDIDATE_SPACE, **ΟΧΙ** CONNECTOR_GEOMETRY, **ΟΧΙ** COST_MODEL — και τα τρία
αποκλείστηκαν με μέτρηση.

**Status:** διάγνωση ολοκληρωμένη, καμία διόρθωση. Τεκμηριωμένη πρόταση:
προσαρμογή του `successor_batch_size` στο μέγεθος του προβλήματος (σε ίσο χρόνο
2 s στις 6 χειρότερες διατάξεις, b=ALL κερδίζει το b=4 σχεδόν παντού:
18: 98.34→87.37, 16: 90.00→75.24, 13: 87.07→75.63).

## #61 — Φάση 7: το adaptive batching υλοποιήθηκε, μετρήθηκε και **ΑΠΟΡΡΙΦΘΗΚΕ**

**Συμπέρασμα πρώτα:** η προσαρμοστική παρτίδα διαδόχων **δεν ενεργοποιείται**.
Η υπόθεση «το πλήθος υποψηφίων προβλέπει τη χρήσιμη παρτίδα» **καταρρίφθηκε σε
ίσο έργο**. Ο κώδικας υπάρχει και ελέγχεται, αλλά το shipped configuration μένει
`fixed / 4`, δηλαδή **καμία αλλαγή συμπεριφοράς** έναντι της Φάσης 5.

**7B — πίνακας σε ίσο wall clock (2 s, 24 διατάξεις), coverage-matched:**

| ομάδα (υποψήφιοι) | n | b=4 | b=8 | b=16 | b=32 | b=64 | ALL |
| --- | --- | --- | --- | --- | --- | --- | --- |
| μικρές (≤80) | 5 | +0.0 | +0.0 | +0.0 | +0.0 | +0.0 | +0.0 |
| μεσαίες (81-140) | 9 | −0.0 | −0.5 | −0.5 | −0.5 | −0.5 | −0.5 |
| μεγάλες (141-200) | 10 | +8.8 | +9.2 | +5.1 | +3.0 | +3.0 | **+1.5** |

**Η κάλυψη ήταν ταυτόσημη σε κάθε παρτίδα σε κάθε ομάδα** — το batching δεν
κοστίζει ποτέ μπάλες. Ο μηχανισμός του #60 επιβεβαιώθηκε και **διορθώνεται** από
μεγάλες παρτίδες:

| ομάδα | b=4 (pops/visit, evals/pop, frontier) | ALL |
| --- | --- | --- |
| μεγάλες | **134.1 / 4.0 / 16** | **1.6 / 207.1 / 155** |

**7B (δεύτερο πείραμα) — σε ίσο ΕΡΓΟ (150k successor evaluations), 7 διατάξεις
με 194-200 υποψηφίους η καθεμία:**

| περίπτωση | flat | b=4 | b=16 | b=64 | ALL | καλύτερο |
| --- | --- | --- | --- | --- | --- | --- |
| **πραγματικό scan** | 51.61 | **49.85** | 54.38 | 54.92 | 54.11 | b=4 |
| layout-1 | 61.06 | 79.81 | 72.58 | **61.16** | 61.81 | b=64 |
| layout-12 | 92.69 | 108.00 | 106.18 | 97.97 | **97.24** | ALL |
| layout-13 | 78.62 | 87.30 | 86.24 | **81.78** | 81.78 | b=64 |
| layout-16 | 68.12 | 97.32 | **62.25** | 88.35 | 75.24 | **b=16** |
| layout-18 | 72.01 | 102.77 | 106.83 | 104.34 | **99.68** | ALL |
| layout-19 | 43.66 | **53.41** | 53.41 | 55.44 | 55.44 | b=4 |

**Στο ίδιο πλήθος υποψηφίων η καλύτερη παρτίδα είναι 4, 16, 64 ή ALL ανάλογα με
τη διάταξη**, και το layout-16 είναι **μη μονότονο** (97.32 → 62.25 → 88.35 →
75.24). Δηλαδή η παρτίδα λειτουργεί ως τυχαιοποιητής της σειράς αναζήτησης, όχι
ως ελεγχόμενη παράμετρος. Πληρούται ακριβώς η συνθήκη τερματισμού που έθεσε ο
χρήστης → **απόρριψη αντί για πολύπλοκη λογική κατωφλίων**.

Σημείωση μεθόδου: το φαινομενικά καθαρό trend του πρώτου πίνακα οφείλεται εν
μέρει στο ότι σε ίσο *χρόνο* οι μεγάλες παρτίδες κάνουν λίγο περισσότερη δουλειά
ανά δευτερόλεπτο· σε ίσο *έργο* το trend διαλύεται σε θόρυβο ανά διάταξη.

**Τι κρατήθηκε (δικαιολογημένο ανεξάρτητα):**
1. **Το budget μετρά successor evaluations, όχι pops.** Αλλιώς το ίδιο νούμερο
   σημαίνει τελείως διαφορετικό έργο ανά pacing. Ισοδύναμη μετατροπή:
   `max_search_expansions` 150000 → **600000** (150k visits × batch 4).
   Το πραγματικό scan αναπαράγεται **ακριβώς**: 45.03 m / 14.07 rad / 10-10 /
   3.0 s / ίδιο plan_id.
2. **Τηλεμετρία μηχανισμού**: `state_pops`, `state_resumptions`, `batch_size`.
3. **`SuccessorBatchPolicy` fixed/adaptive** με το adaptive υλοποιημένο,
   δοκιμασμένο και **απενεργοποιημένο**.

**Διορθώθηκε ένα λανθασμένο test, όχι ο planner.** Το
`test_more_budget_is_never_worse` έλεγχε μονοτονία στο **μήκος**· η εγγύηση είναι
στο **κόστος**. Μετρήθηκε: budget 50 → 6.001 m / κόστος 21.596· budget 10⁶ →
6.026 m / κόστος **20.971**. Η διαδρομή είναι 2.5 cm μακρύτερη και **φθηνότερη**
(λιγότερη στροφή) — ακριβώς αυτό που ορίζει το λεξικογραφικό αντικείμενο. Νέο
`plan_objective_cost` και τα σχετικά tests ελέγχουν πλέον το σωστό μέγεθος.

**7D — ταυτότητα στο COMPLETE:** adaptive και fixed {1, 4, 16, 64, ALL} δίνουν
**ταυτόσημο plan_id** σε όλες τις δοκιμαστικές διατάξεις.

**Tests:** 555 pure PASS, 3 skipped.

**Status:** το adaptive batching απορρίφθηκε με στοιχεία. Το κενό των μεγάλων
προβλημάτων παραμένει ανοιχτό και **δεν** λύνεται με pacing.

## #62 — Φάση 8 (διαγνωστική): η υπόθεση του post-proof cost search **ΑΠΟΡΡΙΦΘΗΚΕ**

**Καμία αλλαγή στην παραγωγή.** Όλα με εξωτερικό patching.

**8A/8B — τι φράγμα κάλυψης υπάρχει ήδη.** Ο router χρησιμοποιεί
`node.mask | eligible_mask`, δηλαδή **σταθερά** το πλήθος των επιλέξιμων μπαλών.
Είναι admissible (ένας διάδοχος συλλέγει μόνο μπάλες που εμφανίζονται σε κάποιον
υποψήφιο, και οι connectors σαρώνουν μόνο `sweepable`), αλλά δεν σφίγγει ποτέ.
Συνέπεια που δεν είχε καταγραφεί: το **cost pruning ενεργοποιείται μόνο όταν
`reachable == best_coverage`**, δηλαδή **ακριβώς** όταν αποδεικνύεται η μέγιστη
κάλυψη. Άρα το branch-and-bound του 8G **υπάρχει ήδη**· λείπει μόνο η *διάταξη*.

| περίπτωση | elig | cov | αποδείχθηκε | maxcov@ | proof@ | % budget μετά |
| --- | --- | --- | --- | --- | --- | --- |
| πραγματικό scan | 10 | 10 | ναι | 85 | 85 | 100.0 |
| layout-13 | 12 | 12 | ναι | 2.001 | 2.001 | 99.7 |
| layout-18 | 13 | 13 | ναι | 13.984 | 13.984 | 97.7 |
| layout-16 | 12 | 12 | ναι | 36.636 | 36.636 | 93.9 |
| layout-1 | 10 | 9 | **όχι** | 20.432 | — | — |
| layout-11 | 8 | 6 | **όχι** | 102 | — | — |
| layout-12 | 14 | 13 | **όχι** | 8.053 | — | — |
| layout-19 | 11 | 10 | **όχι** | 335.204 | — | — |

Όπου αποδεικνύεται, αποδεικνύεται **αμέσως** (gap 0) και **πολύ νωρίς** (85 από
600.000 evaluations στο πραγματικό scan). Όπου δεν αποδεικνύεται, **δεν υπάρχει
φθηνή ενίσχυση**: κάθε ασυλλέκτη επιλέξιμη μπάλα είναι **ατομικά προσβάσιμη**
(μετρήθηκε: `t10`, `t5/t6`, `t0`, `t1` όλες έχουν εφικτό inbound connector). Τις
αποκλείει η **από κοινού** δομή της διαδρομής, όχι η γεωμετρία τους — άρα ένα
forward-reachability φράγμα δεν σφίγγει σε καμία περίπτωση.

**8C — κατανομή μετά την απόδειξη.** Ταξινόμηση κάθε επίσκεψης:
**99.2-99.7% «coverage-capable ΚΑΙ cost-competitive»**, 0.3-0.8% dominated,
**0% coverage-incapable**. Δηλαδή, με βάση το ίδιο το incumbent της στιγμής,
σχεδόν κάθε κατάσταση **μπορούσε** ακόμη να το βελτιώσει.

**8D — πόσο διακριτικό είναι το φράγμα κόστους.** `g(state)` είναι admissible
(κάθε μελλοντικός όρος είναι μη αρνητικός) και ο router χρησιμοποιεί ήδη το
ισχυρότερο `g + υποχρεωτικό terminal`. Ως **τιμή** είναι σφιχτό: ο λόγος
`lb / τελικό κόστος` έχει διάμεσο **0.78-1.06**, και έναντι του **τελικού**
incumbent το **71%** των post-proof καταστάσεων του πραγματικού scan είναι
prunable. Έναντι του **τότε** incumbent μόνο 0.5%. Δηλαδή το φράγμα δεν είναι
αδύναμο σε τιμή· **το incumbent βελτιώνεται πολύ αργά για να δαγκώσει**.

**8E — προσομοίωση (ίδιο budget 600k evaluations):**

| περίπτωση | flat | τρέχον | cost_first | capable_cost |
| --- | --- | --- | --- | --- |
| πραγματικό scan | 48.24 | **45.03** | 57.34 | 57.34 |
| layout-13 | 78.37 | **87.07** | 113.92 | 113.92 |
| layout-16 | 67.61 | **90.32** | 98.22 | 98.22 |
| layout-18 | 71.93 | **99.99** | 107.79 | 107.79 |
| layouts 1/11/19 | — | αμετάβλητα (δεν αποδεικνύεται ποτέ) | | |

**Η διάταξη με βάση το φράγμα κόστους είναι ΧΕΙΡΟΤΕΡΗ παντού.** Ο λόγος είναι
δομικός: το `g + terminal` είναι κόστος **προθέματος**, χωρίς όρο ολοκλήρωσης
(h). Η διάταξη με βάση αυτό εξερευνά πρώτα ρηχές, φθηνές καταστάσεις — Dijkstra
χωρίς ευρετική — ενώ οι πλήρεις διαδρομές βρίσκονται βαθιά. Το coverage-first
τουλάχιστον οδηγεί προς ολοκλήρωση. `capable_cost` ταυτίζεται με `cost_first`
επειδή το φράγμα κάλυψης είναι σταθερό, άρα καμία κατάσταση δεν είναι ποτέ
coverage-incapable (0% στο 8C).

**8F τηρήθηκε:** καμία προσομοίωση δεν περιόρισε την αναζήτηση στις καταστάσεις
μέγιστης κάλυψης· κρατήθηκαν όλες όσες `coverage_upper_bound >= C*`.

**Ταξινόμηση:** **COST_BOUND_TOO_WEAK** ως προς τη **διάταξη** (το φράγμα δεν
έχει όρο ολοκλήρωσης, άρα δεν διακρίνει «κοντά στο τέλος» από «μόλις ξεκίνησε»),
με **COVERAGE_PROOF_TOO_LATE** για τα layouts όπου η απόδειξη δεν έρχεται ποτέ —
και αποδείχθηκε ότι **δεν υπάρχει φθηνό ισχυρότερο φράγμα κάλυψης**.
Πρακτικό συμπέρασμα: **CURRENT_SEARCH_SUFFICIENT**.

**Status:** η υπόθεση απορρίφθηκε με μέτρηση. Καμία αλλαγή στρατηγικής δεν
δικαιολογείται από αυτά τα δεδομένα. Ένα ουσιαστικά ισχυρότερο φράγμα θα
απαιτούσε χαλάρωση τύπου MST/assignment πάνω στους υπόλοιπους υποψηφίους —
πραγματική αρχιτεκτονική πολυπλοκότητα, με μετρημένο διαθέσιμο περιθώριο ~0 στο
πραγματικό scan.

## #63 — Φάση 9: όργανα για «τι σχεδιάστηκε» έναντι «τι οδηγήθηκε»

**Καμία αλλαγή στον planner.** Ο planner είναι πλέον παγωμένη βάση (#62).

**Τι υπήρχε ήδη και τι έλειπε.** Ο controller δημοσιεύει ήδη στο `/state`
`has_active_crossing`, `active_ball_id`, `lateral_error_m`, `heading_error_rad`,
`measured_speed_mps`, και ο node factory τα δειγματοληπτεί σε
`crossing_telemetry`. Το `COLLECTION_ROUTE_AUDIT_DIR` σώζει ήδη snapshot+plan.
**Έλειπαν τρία πράγματα:** (α) η **πραγματική τροχιά**, (β) η **απόδοση των
δεσμών (beams) σε συγκεκριμένη μπάλα**, (γ) οτιδήποτε συνδέει τα τρία.

**Νέα (offline/διαγνωστικά):**
- `collection_execution_trace.py` — σχήμα ίχνους (trajectory / crossing / beam
  edges / ball observations) + recorder με αραίωση.
- `collection_execution_recorder.py` — σύνδεση στον κόμβο, **opt-in** με
  `COLLECTION_EXECUTION_TRACE_DIR`. **Καμία νέα συνδρομή, κανένα timer, κανένα
  topic**: τροφοδοτείται από το ήδη υπάρχον callback του `/state` και από την
  ήδη cached pose.
- `collection_execution_evaluator.py` — **offline** αξιολόγηση.
- `scripts/sim_debug/collection_execution_report.py` — read-only αναφορά.

**Γεωμετρία (9D).** Ο αξιολογητής μετρά με το **φυσικό** στόμιο
(`intake_mouth_contact`, x=0.876, ημιπλάτος **0.205**) και όχι με τον
συντηρητικό διάδρομο του planner (`capture_half_width_m` 0.17 μείον
αβεβαιότητες → ~0.047, #49). Η διαφορά είναι σκόπιμη και τεκμηριωμένη: εδώ
ρωτάμε τι **έκανε** η μηχανή, όχι τι επέτρεψε στον εαυτό του ο planner.

**Ταξινόμηση εκβάσεων (9F)** με 7 κατηγορίες, όλες προσβάσιμες και ελεγμένες.
Δύο κανόνες: (1) η εξαφάνιση **δεν** είναι συλλογή — μόνο ακμή `confirmed`
μετρά· (2) `OBSERVATION_UNCERTAIN` είναι **έγκυρη απάντηση**, προτιμότερη από
το να κατηγορηθεί ο συλλέκτης χωρίς στοιχεία.

**Διόρθωση που βρέθηκε γράφοντας τα tests:** το `SweptCrossing.observed`
σύγχεε το «υπάρχουν δείγματα» με το «έγινε διέλευση», ώστε μια καθαρή αστοχία
tracking αναφερόταν ως `OBSERVATION_UNCERTAIN`. Επίσης ο ανιχνευτής διαταραχής
ανέφερε ως «διαταραχή» τα περάσματα του σασί **μετά** τη συλλογή της μπάλας
(το σασί ακολουθεί εκεί που πέρασε το στόμιο)· τώρα μετρώνται μόνο προσεγγίσεις
**πριν** από την προσπάθεια της ίδιας της μπάλας.

**Κόστος τηλεμετρίας (9M), μετρημένο:** 2570 callbacks (45 m στα 0.35 m/s, 20 Hz)
→ **2.5 μs ανά callback**, 6.3 ms συνολικά, **429 poses + 6 crossings**
(αραίωση 0.10 m / 0.5 s), **68.5 KiB** ανά διαδρομή, γράφεται **μία φορά στο
τέλος** — τίποτα δεν ρέει στο δίκτυο.

**Στατικό φράγμα:** το `collection_execution_evaluator` μπήκε στη λίστα των
modules που **απαγορεύεται** να εισαχθούν σε live κώδικα (μαζί με το
`collection_capture_geometry`), γιατί δουλεύει με το φυσικό στόμιο. Το
recording (`_trace`, `_recorder`) μένει live και δεν περιέχει καθόλου γεωμετρία.

**Tests:** 576 pure PASS, 3 skipped (+21: 13 evaluator, 8 recorder).

**Status:** τα όργανα είναι έτοιμα και επικυρωμένα σε **ντετερμινιστικά
σενάρια εκτέλεσης** (συνθετική τροχιά με δηλωμένο σφάλμα tracking). **ΔΕΝ**
έχουν τρέξει ακόμη σε ζωντανό sim — αυτό είναι το επόμενο βήμα και το μόνο που
μπορεί να δώσει πραγματικά στοιχεία για τον μηχανισμό.

## #64 — Φάση 10: απόπειρα ζωντανής εκτέλεσης — ΔΕΝ ΟΛΟΚΛΗΡΩΘΗΚΕ

**Δεν υπάρχουν αποτελέσματα Φάσης 10.** Καμία μήτρα, καμία μετρική tracking,
κανένα στοιχείο για beams. Ό,τι ακολουθεί είναι διαπιστώσεις περιβάλλοντος.

**Εύρημα 1 — το GPU headless rendering σπάει το Gazebo σε αυτό το μηχάνημα.**
Με `UBUNTU_GPU=true` (προεπιλογή) και `GAZEBO_HEADLESS=true`:

```
libEGL warning: egl: failed to create dri2 screen
[Err] Unable to create the rendering window: eglInitialize failed for device
      EGL_EXT_device_drm ... /dev/dri/card2
[ERROR] [gz-1]: process has died ... exit code -11   (SIGSEGV)
```

Χωρίς Gazebo δεν υπάρχει `/scan`, οπότε ο `lifecycle_manager_slam` περιμένει
**επ' άπειρον** το `slam_toolbox/get_state` — εικόνα που μοιάζει με «κόλλησε το
SLAM» ενώ η αιτία είναι ο renderer.

**Εύρημα 2 — με `UBUNTU_GPU=false` η στοίβα σηκώνεται πλήρως.** Ήρθαν πάνω
controller_node, perception_node (YOLO ONNX, calibration gazebo-v3),
sensor_snapshot_node, diff_drive, IR beams, `/clock`. Το `collect_route` έγινε
δεκτό και ξεκίνησε (`state: navigating_to_scan_pose`).

**Εύρημα 3 — το run δεν προχώρησε πέρα από το `navigating_to_scan_pose` σε
190 s** και μετά **το σκότωσα εγώ**: είχα τυλίξει το `run_ubuntu.sh` σε
`timeout 1200`, που στα 20 λεπτά κατέβασε τον container μαζί με το τρέχον route.
Δικό μου λάθος εργαλείου, όχι εύρημα του συστήματος. Αν το nav στο scan pose
είναι απλώς αργό κάτω από llvmpipe ή κολλάει, **παραμένει άγνωστο**.

**Συνέπεια:** δεν παρήχθη κανένα `audit`/`trace` artifact, γιατί το run δεν
έφτασε ποτέ στο planning. Η ακεραιότητα τηλεμετρίας (10C) **δεν** επικυρώθηκε
ζωντανά.

**Διορθώθηκε στο εργαλείο:** το `run_phase10_scenario.sh` τεκμηριώνει πλέον ότι
η στοίβα πρέπει να σηκώνεται **πρώτα, detached, χωρίς `timeout`**, και ότι
χρειάζεται `UBUNTU_GPU=false`.

**Status:** ΑΝΑΜΟΝΗ. Η καμπάνια χρειάζεται μια συνεδρία αφιερωμένη στο ζωντανό
sim, με τη στοίβα σηκωμένη ανεξάρτητα από τον agent.

## #65 — Preflight Φάσης 10: το Nav2 δεν σηκώνεται στο Docker/Humble — αιτία: όνομα plugin

**ΜΠΛΟΚΑΡΙΣΜΑ ΚΑΜΠΑΝΙΑΣ.** Το preflight σταμάτησε στην πύλη 5 (πλοήγηση στο
scan pose). Καμία αλλαγή στην παραγωγή.

**Η αιτία, μία γραμμή:**

```
[FATAL] [planner_server]: Failed to create global planner. Exception: According to
the loaded plugin descriptions the class nav2_smac_planner::SmacPlanner2D with base
class type nav2_core::GlobalPlanner does not exist. Declared types are
  nav2_navfn_planner/NavfnPlanner  nav2_smac_planner/SmacPlanner2D  ...
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

`config/nav2_params.yaml:226` δηλώνει `nav2_smac_planner::SmacPlanner2D` (μορφή
**Jazzy**), ενώ ο container τρέχει **Humble**, όπου το pluginlib εξάγει
`nav2_smac_planner/SmacPlanner2D` (με κάθετο). Το σχόλιο **ακριβώς από πάνω**
λέει «Humble's pluginlib export uses the slash-qualified class name» — δηλαδή
σχόλιο και τιμή αντιφάσκουν. Εισήχθη στο `49eb908` (2026-07-23, «convert Nav2
plugin class names to Jazzy :: format»), που ήταν σωστό για το native Jazzy και
έσπασε σιωπηλά τη διαδρομή Docker/Humble.

**Συνέπεια:** `planner_server` και `bt_navigator` μένουν **unconfigured**, δεν
παράγεται καμία εντολή ταχύτητας, το ρομπότ **δεν κουνιέται καθόλου** (μετρήθηκε
2.9e-9 m σε 10 s, `/cmd_vel_nav` χωρίς μηνύματα) και κάθε `collect_route` κολλάει
στο `navigating_to_scan_pose` — 332 s στη μέτρηση, ντετερμινιστικά. **Δεν ήταν
αργό sim· ήταν νεκρό Nav2.**

**Οι υπόλοιπες πύλες που πρόλαβαν να μετρηθούν — όλες ΠΕΡΑΣΑΝ:**

| πύλη | αποτέλεσμα |
| --- | --- |
| renderer/σταθερότητα | GPU + **GUI** (d3d12/WSLg), RTF **1.00**, καμία πτώση διεργασίας |
| GPU headless | **ΑΠΟΡΡΙΠΤΕΤΑΙ** — `gz sim` SIGSEGV (#64) |
| camera / depth / detections | 28.1 / 24.0 / **3.7 Hz** |
| ανίχνευση μπάλας | confidence **0.74** και **0.94** στα 1.42 m και 1.90 m |
| court_boundary provenance | **ίδιο map session**: basename `court_1783283137` = αυτό που φορτώνει το SLAM· court 23.77x10.97 m (κανονισμού), `status OK` |
| φορτίο | 905% CPU σε 24 πυρήνες, 1.7 GiB |

**Δεύτερο εύρημα, δικό μου λάθος στα σενάρια:** οι συντεταγμένες των σεναρίων
είναι στο **map** frame, ενώ το `gz set_pose` δέχεται **world**. Διαφέρουν κατά
το κέντρο του διχτυού (**8.08 m** στο x). Έβαζα τις μπάλες **πίσω από το δίχτυ**,
όπου το ρομπότ δεν τις βλέπει ποτέ — εικόνα ταυτόσημη με «χαλασμένο perception».
Διορθώθηκε στο `run_phase10_scenario.sh` (μετασχηματισμός από το
`court_boundary.json`) και οι διατάξεις μετακινήθηκαν κοντά στο scan pose
(map 1.68, 0) ώστε να πέφτουν στο **βαθμονομημένο εύρος 1.02-2.98 m**.

**Δύο δρόμοι, απόφαση χρήστη:**
1. **Native Jazzy** (`run_native.sh`) — εκεί το `::` είναι σωστό και εκεί έτρεξε
   το end-to-end collect_route (#WS1, 278c457). Καμία αλλαγή κώδικα.
2. Να γίνει το `nav2_params.yaml` distro-aware ή να επιστρέψει στο `/` για τον
   Humble container — **αλλαγή παραγωγής, θέλει έγκριση**.

**Status:** η καμπάνια 5x3 ΔΕΝ ξεκίνησε. Πύλη 5 απέτυχε με μετρημένη αιτία.

## #66 — Native Jazzy: η στοίβα δουλεύει, το route ολοκληρώνεται, **4/4 συλλογές** — μπλοκάρει το δικό μου trace wiring

**Κανονικό runtime πλέον ρητά: native Ubuntu 24.04 + ROS 2 Jazzy.** Το
Docker/Humble χαρακτηρίστηκε obsolete και το `run_ubuntu.sh` **αρνείται να
ξεκινήσει** χωρίς `ALLOW_OBSOLETE_HUMBLE_DOCKER=true` (με το μήνυμα να εξηγεί την
αιτία του #65). CLAUDE.md, `setup_env.sh` και το μήνυμα του xacro δείχνουν Jazzy.

**Επαλήθευση plugin (χωρίς αλλαγή):** το εγκατεστημένο Jazzy
`nav2_smac_planner` δηλώνει
`<class type="nav2_smac_planner::SmacPlanner2D" base_class_type="nav2_core::GlobalPlanner">`
— δηλαδή το `nav2_params.yaml:226` **είναι σωστό** και έμεινε ως έχει.

**Πύλες που ΠΕΡΑΣΑΝ στο native Jazzy:**

| πύλη | αποτέλεσμα |
| --- | --- |
| Nav2 lifecycle | `planner_server` **active**, `controller_server` **active**, `bt_navigator` **active** |
| SLAM | `lifecycle_manager_slam: Managed nodes are active` |
| πλοήγηση σε scan pose | **ΕΦΤΑΣΕ** — pose (1.568, −0.035) έναντι στόχου (1.682, −0.004), σφάλμα ~0.12 m, σε 17.2 s |
| 360 scan | ολοκληρώθηκε (`scanning` 17.2 s → `planning` 22.95 s), βρήκε **4 μπάλες** |
| planning | `feasible`, search `complete`, **20.37 m**, 7 segments (3 passes με 1+2+1 μπάλες, 3 connectors, terminal), και οι 4 μπάλες `covered/selected` |
| εκτέλεση | έτρεξε πλήρως, **confirmed 4**, `basket_retained 4`, `crossed_unconfirmed 0` |

**Δηλαδή: πρώτη end-to-end επιτυχής συλλογή με τον νέο planner — 4 από 4.**

**Δύο ευρήματα:**

1. **Αναπαράχθηκε το ανοιχτό #48**: το route έκοψε στο **terminal** segment με
   `trajectory_tube_exceeded | seg terminal progress 20.347m lat_err 0.000m
   head_err 0.000rad` — δηλαδή **2 cm πριν το τέλος** διαδρομής 20.37 m, με
   **μηδενικό** πλευρικό και γωνιακό σφάλμα. Ίδια αντίφαση με το #48 (τότε στα
   57.658/57.7 m). Οι 4 συλλογές είχαν ήδη γίνει· το abort είναι στο τέλος.
   **Δεν αγγίχτηκε** — είναι ανοιχτό θέμα του controller.

2. **ΜΠΛΟΚΑΡΙΣΜΑ: το Phase 9 trace δεν γράφτηκε.** Το audit γράφτηκε σωστά
   (`collection-scan-94818000000`, plan `route-ac1cec44e787a8cb`), το
   `COLLECTION_EXECUTION_TRACE_DIR` **έφτασε** στο περιβάλλον του node, το
   `ExecutionTraceCapture` **κατασκευάζεται** — αλλά **ποτέ δεν καλείται
   `start(plan)` ούτε `finish()`**. Άρα ο recorder μένει αδρανής (`active`
   False), κανένα δείγμα δεν καταγράφεται και κανένα αρχείο δεν γράφεται.
   Σφάλμα **δικό μου, στο wiring της Φάσης 9** — τα unit tests κάλυπταν το
   `ExecutionTraceCapture` απευθείας, όχι τη σύνδεσή του στον executor.

**Status:** ΔΕΝ ΕΤΟΙΜΟ για την καμπάνια 5x3. Μία πύλη πέφτει, με γνωστή και
μικρή αιτία. Καμία αλλαγή στο collection system.

## #67 — Φάση 9B: ο executor οδηγεί πλέον τον κύκλο ζωής του trace — ζωντανή επαλήθευση

**Χαρτογράφηση πριν την αλλαγή.** Η διαδρομή αρχίζει πραγματικά στο
`_tick_collector_start`, εκεί που καλείται `_path_follower.start(self.plan)` και
`_drive_observer.start()` — **όχι** στο planning. Κάθε τερματισμός (COMPLETED,
COMPLETED_NO_TARGETS, INCOMPLETE_TARGETS, ABORTED_SCAN/PLANNING/COLLECTOR/
SAFETY/TRACKING) περνά από **ένα** σημείο: το `_transition`.

**Η αλλαγή (μόνο instrumentation):**
- `CollectionRouteExecutor` δέχεται προαιρετικό `execution_trace` — ακριβώς το
  μοτίβο του υπάρχοντος `drive_observer`.
- `start(plan)` δίπλα στο `_drive_observer.start()`.
- `finish()` μέσα στο `_transition` όταν η νέα κατάσταση είναι τερματική —
  **και στις επιτυχίες και στα abort**. Το σύνολο τερματικών βγήκε σε
  `_TERMINAL_STATES` και το χρησιμοποιεί και το `is_terminal`.
- Κάθε κλήση σε try/except: **το instrumentation δεν ρίχνει ποτέ διαδρομή**.
- Πέρασμα μέσω `collection_executor_assembly` και node factory.

**Tests (10 νέα, integration στο επίπεδο του executor):** επιτυχής διαδρομή,
abort με `trajectory_tube_exceeded`, safety abort, tracing απενεργοποιημένο
(no-op), διπλό τερματικό γεγονός (idempotent), δύο διαδοχικές διαδρομές =
δύο ξεχωριστά artifacts, εκρηκτικό instrumentation, ταυτότητα audit/trace, και
ο evaluator που **αρνείται** να ενώσει trace άλλου plan.
**Απόδειξη ότι πιάνουν το σφάλμα:** με αφαιρεμένο το wiring **8 από 10
αποτυγχάνουν**· με το wiring περνούν όλα. 586 pure PASS συνολικά.

**Ζωντανή επαλήθευση (native Jazzy, GPU+GUI):**

| | |
| --- | --- |
| audit | `collection-scan-59590000000` |
| trace | `collection-run-59590000000-route-1dd8607f0b195b26.trace.json` |
| **ταυτότητα** | plan_id **ταιριάζει**, scan_id **ταιριάζει**, run_id παρόν |
| samples / crossings | **189 / 67** |
| συνέχεια | βήμα 0.100-0.269 m (μέσος 0.112), 0.05-0.48 s |
| μήκος | 21.02 m ανακατασκευασμένο έναντι 20.33 m σχεδιασμένο |
| segments | **και τα 7 του trace υπάρχουν στο plan** |
| μέγεθος | 73.7 KiB, γράφεται μία φορά στο τέλος |
| αποτέλεσμα | **abort** `trajectory_tube_exceeded` στα 20.309/20.33 m — **και το trace γράφτηκε** |
| επίδραση | 81.9 s / 4 confirmed, έναντι 82.7 s / 4 confirmed χωρίς tracing — **καμία** |

**Ο evaluator κατανάλωσε τα ζωντανά artifacts** και έβγαλε πραγματικές μετρικές:
cross-track max 0.086-0.307 m (χειρότερο ο connector-1: RMS 0.174), 3 από 4
μπάλες **σαρώθηκαν** από το στόμιο (clearance 0.031-0.048 m, πλευρικά −0.139 /
+0.028 / −0.073 m), και η `target-3` **δεν** σαρώθηκε (ελάχιστη προσέγγιση
0.103 m) → `planned_but_tracking_missed`.

**ΚΕΝΟ ΠΟΥ ΑΠΟΜΕΝΕΙ — και είναι μικρότερο απ' ό,τι νόμιζα.** Το trace έχει
**0 beam events**: το `record_beams()` δεν έχει καλούντα. Άρα ο evaluator δεν
βλέπει καμία επιβεβαίωση και βγάζει `observation_uncertain` για 3 μπάλες, ενώ
το run **μάζεψε 4 στις 4**.

**Αλλά η απόδειξη υπάρχει ήδη αλλού:** το `robot_status.json` κρατά
`collect_route.confirmations` με **πλήρη απόδοση ανά μπάλα**:

```
id=1 ball=/target-4 assoc=intake_lead_crossing prog=5.87  lat=0.0054 head=-0.0055
id=2 ball=/target-3 assoc=intake_lead_crossing prog=12.35 lat=0.0258 head=0.0264
id=3 ball=/target-2 assoc=intake_lead_crossing prog=13.03 lat=0.0055 head=0.0173
id=4 ball=/target-1 assoc=intake_lead_crossing prog=19.53 lat=0.0019 head=-0.0228
```

συν `execution_outcomes` με `execution_status: confirmed` ανά μπάλα. Δηλαδή το
**συμβόλαιο επιβεβαίωσης υπάρχει** — απλώς δεν περνά στο trace. Ενδιαφέρον:
η `target-3` επιβεβαιώθηκε με lat 0.0258 ενώ ο evaluator την είδε **εκτός**
στομίου (0.103 m) — αξίζει διερεύνηση, αλλά **όχι σε αυτή τη φάση**.

**Status:** πύλες 1-6 της Φάσης 9B περνούν. Η καμπάνια παραμένει **NOT READY**:
χωρίς τις επιβεβαιώσεις στο trace, η μήτρα των 15 runs θα έδειχνε 0%
συλλογές ενώ μαζεύονται 4/4 — χειρότερο από καθόλου δεδομένα.

## #68 — Οι επιβεβαιώσεις φτάνουν στο trace· και η εξήγηση του target-3

**Καμία αλλαγή στο collection system.** Μόνο evidence pipeline.

**Διαδρομή της αυθεντικής επιβεβαίωσης (Θέμα Α).** Εντοπίστηκε:

```
IrReadings (basket IR ζεύγος) → _confirmed_beam_broken → hardware latch +
_consume_entry_for_confirmation() → collection_count += 1 →
_record_route_confirmation(now_s) → _route_confirmation_context(now_s)
   → association ∈ {active_crossing, intake_lead_crossing, recent_crossing, unassigned}
→ _collect_route_confirmations → robot_status.collect_route.confirmations
```

Το **`_record_route_confirmation`** είναι το πρώτο σημείο όπου υπάρχει πλήρως
αποδοσμένη επιβεβαίωση — εκεί μπήκε το instrumentation. **Δεν** διαβάζεται το
`robot_status.json` πίσω μέσα.

**Trace schema v2.** Νέο `ConfirmationEvent` (t, id, ball, association, segment,
progress, crossing_progress, lateral, heading, speed), **ξεχωριστό** από το
`BeamEvent`: το beam είναι φυσική μετάβαση, η επιβεβαίωση είναι απόδοση σε
μπάλα. Ο reader δέχεται και v1 (χωρίς confirmations) → αξιολογείται συντηρητικά.

**Evaluator.** Όπου υπάρχει αυθεντική επιβεβαίωση, **δεν** συμβουλεύεται καμία
ευρετική χρονισμού. Η φυσική ανακατασκευή παραμένει **ανεξάρτητη** απόδειξη, και
όταν οι δύο διαφωνούν εκδίδεται νέα κατηγορία
**`CONFIRMED_WITHOUT_RECONSTRUCTED_CROSSING`** — καμία από τις δύο δεν
ξαναγράφεται.

**ΡΙΖΙΚΗ ΑΙΤΙΑ target-3 (Θέμα Β) — δεν είναι bug του evaluator.** Από τα
artifacts του #67:

| στοιχείο | τιμή |
| --- | --- |
| σχεδιασμένο crossing | progress **12.349**, segment `pass-1:...target-3+target-2` |
| επιβεβαίωση | progress **11.638**, association **`intake_lead_crossing`** |
| πλησιέστερο δείγμα | t=114.31, **progress 11.003, segment `connector-1`** |
| μπάλα σε robot frame εκεί | x=**0.816** (στόμιο 0.876), y=**−0.2885** (ημιπλάτος 0.205) |
| clearance | **0.1028 m** |

Δηλαδή **η μπάλα μαζεύτηκε πάνω στον connector-1, 1.35 m ΠΡΙΝ** το σχεδιασμένο
της πέρασμα — ακριβώς η περίπτωση που ο controller ονομάζει
`intake_lead_crossing` («το στόμιο είναι 0.876 m μπροστά από το base_footprint,
άρα ENTRY/CONFIRMED συμβαίνουν πριν το παράθυρο crossing του κέντρου»).

**Και τα δύο «lateral» μετρούν διαφορετικά πράγματα:**

| υποσύστημα | γεωμετρία αναφοράς | frame | σημασία |
| --- | --- | --- | --- |
| planner swept crossing | κεντρική γραμμή περάσματος | map | προβολή μπάλας στη γραμμή· `predicted_lateral_error` = **0.0000** εδώ |
| controller lateral_error | `base_footprint` έναντι σχεδιασμένης διαδρομής | map | **σφάλμα παρακολούθησης** = 0.0258 |
| confirmation lateral | το ίδιο με πάνω, τη στιγμή του beam | map | 0.0258 |
| Phase 9 evaluator | **μπάλα → τμήμα στομίου** (x=0.876, ±0.205) | robot | **απόσταση σύλληψης** = 0.1028 |

Άρα **δεν υπάρχει αντίφαση**: το 0.0258 λέει «το ρομπότ ακολούθησε καλά τη
διαδρομή του», το 0.1028 λέει «το στόμιο πέρασε 10 cm από την **πιστευόμενη**
θέση της μπάλας». Η πιστευόμενη θέση φέρει την αβεβαιότητα του snapshot
(cov xx=0.0104 → σ≈**0.10 m**), δηλαδή η διαφορά είναι **1σ** perception.
Δευτερεύον: η δειγματοληψία 0.10-0.12 m εισάγει έως 0.058 m διαμήκη σφάλμα.

**Καμία ανοχή δεν διευρύνθηκε, κανένα footprint δεν άλλαξε.**

**Επαλήθευση στα πραγματικά δεδομένα του #67** (με τις δικές του επιβεβαιώσεις):

```
target-1  collected_by_different_segment            swept ✓  conf ✓
target-2  planned_and_executed_collected            swept ✓  conf ✓
target-3  confirmed_without_reconstructed_crossing  swept ✗  conf ✓  clear 0.103
target-4  planned_and_executed_collected            swept ✓  conf ✓
```

**Καμία μπάλα δεν είναι πια `observation_uncertain`** και η ασυμφωνία
εμφανίζεται και στη μήτρα (`planned=yes executed=no confirmed=yes : 1`).

**Tests:** 592 pure PASS (+5: επιβεβαίωση φτάνει στο trace, μη-αποδοσμένη
απορρίπτεται, evaluator πιστεύει την αυθεντική, ασυμφωνία εμφανίζεται, v1 trace
διαβάζεται συντηρητικά).

**ΑΝΟΙΧΤΟ — ζωντανή επαλήθευση.** Τρεις προσπάθειες σήμερα **δεν** μάζεψαν
μπάλα: `nav2 goal aborted` (46 s), μετά **3× `aborted_scan`** με
`navigation_failed` στα 0.16 s. Ενδιάμεσα το `gz set_pose` στο ίδιο το ρομπότ
**χάλασε το localization** (pose κολλημένο στο 4.034, 2.496). Ακόμη και μετά από
καθαρή επανεκκίνηση, το run σταμάτησε σε `aborted_scan` στα 26 s. Τα δύο
πρωινά runs (#66, #67) είχαν τρέξει πλήρως — **το περιβάλλον υποβαθμίστηκε, όχι
ο κώδικας**. Μάθημα: **ποτέ `set_pose` στο ρομπότ** ενώ τρέχει SLAM localization.

**Status:** ο αγωγός στοιχείων είναι πλήρης και ελεγμένος offline πάνω σε
**πραγματικά** δεδομένα, αλλά λείπει **ένα ζωντανό run που να μαζεύει ≥1 μπάλα**
ώστε να αποδειχθεί ότι το wiring εκπέμπει `ConfirmationEvent` ζωντανά.

## #69 — Πύλη Φάσης 10: ΠΕΡΑΣΕ. Και η αιτία όλων των αποτυχιών του #68

**Καμία αλλαγή κώδικα.** Μόνο καθαρό περιβάλλον και ένα run.

**Η ΑΙΤΙΑ ΤΩΝ ΑΠΟΤΥΧΙΩΝ ΤΟΥ #68: 58 ζόμπι διεργασίες.** Το `/clock` είχε
**4 publishers**, όλοι `sim_clock_relay` — τέσσερις ολόκληρες στοίβες ζωντανές
ταυτόχρονα στο `ROS_DOMAIN_ID=42`, η παλαιότερη επί **6 ώρες 23 λεπτά**. Κάθε
μία δημοσίευε τον δικό της sim χρόνο, οπότε οι συνδρομητές έβλεπαν τον χρόνο να
πηγαίνει **πίσω**:

```
[bt_navigator] [WARN] [tf2_buffer]: Detected jump back in time. Clearing TF buffer.
```

συνεχώς → άχρηστο TF → `navigation_failed` στα 0.16 s → `aborted_scan`. Τα
`kill` μου έπιαναν μόνο `gz sim|controller_node|ros2 launch|run_native` και
**αστοχούσαν σε όλες** τις διεργασίες `install_jazzy/tennis_robot/...`,
`twist_mux`, `nav2_*`, `slam_toolbox`. Δηλαδή: **δικό μου λάθος υγιεινής
διεργασιών, όχι υποβάθμιση του συστήματος.**

**Μετά από πλήρη καθαρισμό, health gate ΚΑΘΑΡΟ:**

| έλεγχος | πριν | μετά |
| --- | --- | --- |
| `/clock` publishers | **4** | **1** |
| "jump back in time" | εκατοντάδες | **0** |
| planner/controller/bt_navigator | active | **active** |
| RTF | 0.477 | **0.998** |

**Το run (μία μπάλα στο map (3.2, 0.0), το ρομπότ ΔΕΝ μετακινήθηκε ποτέ με
set_pose):** scan → plan → execute → **2 confirmed** → follow-up scan →
`completed_no_targets` σε 105 s. Το scan βρήκε 3 μπάλες (η μία τοποθετημένη, δύο
του κόσμου εντός εμβέλειας).

**ΑΠΟΔΕΙΞΗ ΤΗΣ ΠΥΛΗΣ — `ConfirmationEvent` στο persisted trace:**

```
schema: collection-execution-trace/v2      len(confirmations) = 2
id=1 ball=…/target-3 assoc=intake_lead_crossing seg=pass-0:…/target-3
     progress=5.337  crossing_progress=5.785  lateral=0.004883
id=2 ball=…/target-1 assoc=intake_lead_crossing seg=pass-1:line:…target-2+target-1
     progress=16.268 crossing_progress=16.809 lateral=0.0000384
```

**Ταυτίζονται byte-προς-byte με το `robot_status.collect_route.confirmations`.**
Ταυτότητες: plan_id ✓, scan_id ✓, run_id παρόν.

**Ο evaluator χρησιμοποιεί την αυθεντική επιβεβαίωση, όχι ευρετική:** το trace
έχει **0 beam edges**, άρα ένα «confirmed» μπορεί να προέρχεται **μόνο** από το
`ConfirmationEvent`.

**Αποτέλεσμα ανά μπάλα — πλήρως συνεπές:**

| μπάλα | swept | clear | conf | έκβαση |
| --- | --- | --- | --- | --- |
| target-1 | ✓ | 0.019 | ✓ | `planned_and_executed_collected` |
| target-3 | ✓ | 0.041 | ✓ | `planned_and_executed_collected` |
| target-2 | ✗ | **0.013** | ✗ | `planned_but_tracking_missed` |

**Καμία `observation_uncertain`, καμία ασυμφωνία.** Η target-2 είναι γνήσια
αστοχία ξυστά: το στόμιο πέρασε **1.3 cm** από τη μπάλα χωρίς να τη διασχίσει,
και δεν επιβεβαιώθηκε — οι δύο ανεξάρτητες αποδείξεις **συμφωνούν**.

**Tracking:** cross-track max 0.069-0.249 m (χειρότερο ο connector-1, RMS 0.166).
**Επίδραση tracing:** καμία — η διαδρομή ολοκληρώθηκε κανονικά με follow-up.

**Status:** **READY** για την καμπάνια 5x3. Υποχρεωτικός κανόνας πριν από κάθε
run: **σκότωσε τα πάντα με το πλήρες pattern** (`install_jazzy`, `twist_mux`,
`nav2_*`, `slam_toolbox`, `gz sim`) και **επιβεβαίωσε `/clock` publishers == 1**.

---

## #70 — Καμπάνια Φάσης 10, run 0: το trace κρατούσε **μόνο την τελευταία** διαδρομή κάθε session

**Πλαίσιο.** Πρώτο run της καμπάνιας (`straight_sweep` r1, 3 μπάλες στο map
(3.2/3.9/4.6, 0.0)). Η στοίβα πέρασε την πύλη υγείας (`/clock` publishers = 1,
0 time jumps, Nav2 active, map→base_footprint παρόν) και η διαδρομή ολοκληρώθηκε.

**Το εύρημα.** Το session εκτέλεσε **δύο** διαδρομές:

```
run 1  plan route-27b0ec9668b4fad5  planned 2  confirmed 0  route_completed
run 2  plan route-3810005377773e36  planned 1  confirmed 1  route_completed
```

αλλά στον δίσκο βρέθηκε **ένα μόνο** trace — αυτό της **δεύτερης**. Η πρώτη
διαδρομή, εκείνη που εκτελεί το μεγαλύτερο μέρος του σχεδίου, χάθηκε ολόκληρη.

**Αιτία (state machine, όχι τύχη).** Η ακολουθία ενός session με follow-up είναι:

```
executing_route → route_completed → collector_stopping → evaluating_results
                → navigating_to_scan_pose → … → incomplete_targets
```

`ROUTE_COMPLETED` είναι **ανά-διαδρομή** κατάσταση και **δεν** ανήκει στα
`_TERMINAL_STATES`· τερματική κατάσταση το session φτάνει **μία φορά**, στο
τέλος. Το flush του trace κρεμόταν αποκλειστικά από τα `_TERMINAL_STATES`, άρα:
η `start()` της δεύτερης διαδρομής αντικαθιστούσε τον recorder της πρώτης πριν
αυτός γραφτεί ποτέ.

**Γιατί δεν το έπιασε η πύλη #69.** Εκεί η δεύτερη διαδρομή ήταν
`completed_no_targets` με **κενό** plan: δεν έγινε ποτέ `start()` δεύτερου
recorder, οπότε το τελικό flush έγραψε τα δεδομένα της **πρώτης**. Η πύλη πέρασε
για τον σωστό λόγο πάνω σε μια διαμόρφωση που έκρυβε το σφάλμα.

**Η διόρθωση (μόνο όργανα).** Ξεχωριστό σύνολο flush, ώστε η σημασιολογία του
`is_terminal` — που είναι συμπεριφορά παραγωγής και τη διαβάζει ο controller —
να μείνει **αμετάβλητη**:

```python
_TRACE_FLUSH_STATES = _TERMINAL_STATES | {ExecutorState.ROUTE_COMPLETED}
...
if state in _TRACE_FLUSH_STATES:
    self._end_execution_trace()
```

Κανένας planner/controller/collector/perception/confirmation/evaluator κανόνας
δεν άλλαξε· αλλάζει μόνο **πότε γράφεται** ό,τι έχει ήδη καταγραφεί.

**Test που το κλειδώνει:**
`test_every_route_of_a_multi_run_session_persists_its_own_trace` — session δύο
διαδρομών με διαφορετικά snapshots/plan ids· απαιτεί **δύο** αρχεία trace με τα
δικά τους samples. Χωρίς τη διόρθωση αποτυγχάνει (`assert 1 == 2`).

**ΠΑΓΙΔΑ (ξανά):** το colcon install **αντιγράφει**. Χωρίς rebuild του
`tennis_robot` η ζωντανή στοίβα τρέχει τον παλιό κώδικα.

**Συνέπεια για την καμπάνια.** Το run 0 απορρίπτεται (δεν προσμετράται στις 3
επαναλήψεις) και και τα 15 έγκυρα runs τρέχουν με τον διορθωμένο κώδικα, ίδιο σε
όλες τις επαναλήψεις. Ο evaluator τρέχει πλέον **ανά διαδρομή**, ζευγαρώνοντας
audit↔trace με βάση το `plan_id` και όχι τη σειρά των αρχείων.

**Status:** διορθωμένο, κλειδωμένο με test, η καμπάνια ξεκινά από την αρχή.

---

## #71 — Καμπάνια Φάσης 10: 15/15 έγκυρα runs. Ο ένοχος είναι **ασυμφωνία πλαισίου**, όχι ο μηχανισμός

**Πρωτόκολλο.** 5 σενάρια x 3 επαναλήψεις, καθένα σε δική του καθαρή native
Jazzy στοίβα. Πύλη υγείας πριν από κάθε run: πλήρες kill pattern, `/clock`
publishers **= 1**, **0** `Detected jump back in time`, Nav2 lifecycle active,
map→base_footprint παρόν, **καμία** μετακίνηση του ρομπότ. **15/15 πέρασαν με
την πρώτη — 0 άκυρα.** Καμία αλλαγή σε planner/controller/collector/perception/
confirmation/evaluator κατά τη διάρκεια.

**Ακεραιότητα αποδείξεων:** 28 runtime confirmations, 28 στα traces — **ταύτιση
σε 15/15 runs**. 2 `unassigned` (straight_sweep-r3, connector_collects-r1).

**Πρωτογενή αθροίσματα (67 planned crossings, 15 runs):**

| μέγεθος | τιμή |
| --- | --- |
| μπάλες που το στόμιο **παρουσίασε** (clearance ≤ 5 cm) | 43/67 = **64%** |
| μπάλες που **έμειναν στο καλάθι** (Gazebo truth) | **33** |
| ποσοστό κατάποσης δοθείσης παρουσίασης | 33/43 = **77%** |
| planned_but_tracking_missed | 17 (25%) |
| observation_uncertain | 21 (31%) |
| collected_by_different_segment | 12 (18%) |
| confirmed_without_reconstructed_crossing | 11 (16%) |

**ΤΟ ΕΥΡΗΜΑ — δύο μετρήσεις του ίδιου μεγέθους διαφέρουν κατά 10x.**

Πάνω σε **ευθείες** collection passes (η γεωμετρία είναι ευθύγραμμο τμήμα, άρα η
διακριτοποίηση είναι ακριβής):

```
κάθετη απόσταση καταγεγραμμένης πόζας από την σχεδιασμένη γραμμή
    n=686   median 0.144 m   p90 0.270 m   max 0.781 m
cross-track που αναφέρει ο ίδιος ο controller τις ίδιες στιγμές
            median 0.012 m   max 0.026 m
```

Ίδιο νούμερο σε connectors (0.144 m) — άρα **δεν** είναι υστέρηση (η υστέρηση θα
φούσκωνε τις καμπύλες, όχι τις ευθείες). Ο planner τοποθετεί **κάθε** crossing
ακριβώς πάνω στη μπάλα (offset **0.0000 m** και στα 66). Άρα η αστοχία δεν είναι
ούτε σχεδιασμός ούτε κέρδη ελεγκτή: είναι **το πού νομίζει ο tracking core ότι
βρίσκεται σε σχέση με το πλαίσιο στο οποίο ζουν οι μπάλες**.

Το μισό πλάτος του στομίου είναι **0.205 m**. Διαφωνία 0.14-0.27 m είναι ακριβώς
η κλίμακα που κρίνει αν η μπάλα μπαίνει ή περνά δίπλα.

**Ανεξάρτητη επιβεβαίωση από το ίδιο το runtime:** `pose_source=slam_tf`,
`pose_error_m` (SLAM vs Gazebo truth, βαθμονομημένο στην αρχή του route) =
0.009-0.535 m, με **0.29-0.31 m** στα μεγάλα runs.

**Παράδειγμα (two_passes_with_connector, ίδιο και στις 3 επαναλήψεις).** Η
target-2 χάνεται κατά **0.30 m** ντετερμινιστικά. Τη στιγμή t=137.2 ο controller
λέει `lat_err 0.0156 m` και crossing 17 mm μπροστά· η καταγεγραμμένη πόζα βάζει
τη μπάλα στο **(-0.311, -0.254)** του robot frame — πίσω και δεξιά από τη βάση.

**Τερματικό anomaly:** 13/17 routes τελείωσαν με `path_failed` —
`trajectory_tube_exceeded` **7** (ΟΛΑ στο `terminal` segment, 0.02-0.05 m πριν το
τέλος, lat_err **0.000**, με τις συλλογές ήδη ολοκληρωμένες → κοσμητικό) και
`heading_error_exceeded` **6** (πραγματικές πρόωρες διακοπές· μόνο 4 από τις 17
χαμένες μπάλες ήταν πέρα από το σημείο διακοπής).

**Διαταραχή:** 63 προσεγγίσεις καταγράφηκαν, **0** με μετρημένη μετατόπιση — δεν
υπάρχουν παρατηρήσεις μετά την προσπάθεια. **Κενό απόδειξης, ΟΧΙ μηδέν.** Σε 8
προσεγγίσεις το σώμα πέρασε με body clearance ≈ 0.000 m.

**Passes vs connectors: ΑΝΑΠΑΝΤΗΤΟ.** Ο planner ανέθεσε **65/67** crossings σε
passes και μόλις **2** σε connectors. Δεν υπάρχει δείγμα για σύγκριση.

**Ποιότητα οργάνων (να μη διορθωθεί χωρίς εντολή):** (α) τα pose samples είναι
ανά ~10.8 cm, άρα clearance κάτω από ~5.4 cm **δεν διακρίνεται**· (β) το
`measured_speed_mps` των confirmations είναι 0.0 σε 27/28 ενώ το progress
προχωρά με 0.35 m/s — το πεδίο είναι αναξιόπιστο· (γ) κανένα beam edge δεν
καταγράφεται, άρα ο evaluator **δεν μπορεί** ποτέ να βγάλει
`EXECUTED_CROSSING_NOT_COLLECTED` — γι' αυτό 21 μπάλες πέφτουν σε
`observation_uncertain`.

**Status:** η αρχική υπόθεση «φταίει ο μηχανισμός συλλογής» **ΔΕΝ
ΥΠΟΣΤΗΡΙΖΕΤΑΙ**: όταν η μπάλα παρουσιάζεται στο στόμιο, καταπίνεται στο 77%.
Επόμενο υποσύστημα: **συνέπεια πόζας/πλαισίου μεταξύ tracking core, SLAM και του
frame των μπαλών**.

---

## #72 — Φάση 11 (διαγνωστική): η διαδρομή **παγώνει στο `odom`** και το `map` φεύγει από κάτω της

**Η ερώτηση.** Ποια πόζα, σε ποιο πλαίσιο και με ποια χρονοσήμανση χρησιμοποιεί
πραγματικά ο tracking core; Απάντηση **με μέτρηση**, όχι από ονόματα topic.

**11A — η διαδρομή των δεδομένων.**

```
controller_node → collection_executor_node_factory._ExecutionPlanTransformer
    lookup_transform("odom", plan.map_frame, time 0.0)   ← ΜΙΑ φορά
    transform_collection_plan(plan, rigid)               ← plan map → odom
  → follow_path_sender(map_frame=plan.map_frame …)       ← στέλνει ήδη "odom"
  → Nav2 controller_server → CollectionNav2Controller::setPlan(path)
  → make_tracking_plan(): path.poses[i].position.x/y ΩΣ ΕΧΟΥΝ, χωρίς μετασχηματισμό
  → computeVelocityCommands(pose …): pose.pose.position.x/y ΩΣ ΕΧΟΥΝ
  → CollectionTrackingCore::update(): project_monotonically() → lateral_error_m
```

`TrackingInput`/`TrackingPoint` **δεν έχουν πλαίσιο ούτε χρονοσήμανση**: ο core
συγκρίνει δύο γυμνά ζεύγη συντεταγμένων και εμπιστεύεται τον καλούντα. Δεν
μπορεί να ελέγξει το συμβόλαιο, άρα ένα λάθος πλαισίου του είναι αόρατο.

**11B/11C — τι δημοσιεύεται τώρα.** Το υπάρχον `CollectionControllerState` (20
Hz, ήδη publish σε κάθε update) πήρε τα δύο γεωμετρικά αντικείμενα από τα οποία
υπολογίστηκε το `lateral_error_m`, με **ρητά ονόματα πλαισίου**:
`tracker_pose_frame_id/stamp/x/y/yaw`, `tracker_path_frame_id/stamp`,
`tracker_reference_x/y/yaw`. Ο `TrackingResult` κουβαλά το σημείο αναφοράς σε
**κάθε** έξοδο, και στα aborts. Κανένας υπολογισμός ελέγχου δεν άλλαξε.

**11D/11E — μετρήσεις (1545 tracking updates, ζωντανό run):**

| έλεγχος | αποτέλεσμα |
| --- | --- |
| tracker self-consistency (lateral) | **6.9e-18 m** max |
| tracker self-consistency (heading) | **6.7e-05 rad** max |
| tracker pose vs `odom→base_footprint` | **0.000000 m** — είναι ακριβώς αυτή |
| tracker pose vs `map→base_footprint` | med **0.2554**, max **0.4581 m** |
| `map→odom ⊕ odom→base − map→base` | med **4.7e-16 m** |
| ηλικία πόζας στον υπολογισμό | med **10 ms** → 3.5 mm στα 0.35 m/s |
| ηλικία διαδρομής (παγωμένη στο setPlan) | έως **65.2 s** |

**Η καθυστέρηση ΔΕΝ εξηγεί τίποτα** (3.5 mm έναντι 0.14-0.44 m).

**11F — η ντετερμινιστική αστοχία, αναπαραγμένη.** `two_passes_with_connector`,
target-2:

```
μπάλα (map)                      ( 3.729, -1.144)
planned crossing (map)           ( 3.729, -1.144)   ← ταυτόσημα: ο planner είναι σωστός
tracker pose      [odom]         ( 3.726, -1.137)
tracker reference [odom]         ( 3.741, -1.132)
reported cross-track 0.0154 m  |  recomputed 0.0154 m   ← ο core λέει αλήθεια
map→odom τη στιγμή εκείνη        (-0.3225, +0.1952)  |0.3770|
κέντρο διαδρόμου εκφρασμένο σε map ( 3.431, -0.896)
→ κέντρο διαδρόμου vs μπάλα      **0.3882 m**
```

**Η ΑΙΤΙΑ.** Ο `_ExecutionPlanTransformer` («Freeze map→odom once») πάγωσε
`odom←map = (+0.0302, +0.0133), |0.0330 m|` στο t=73.544. Κατά τη διαδρομή το
ζωντανό `map→odom` περιπλανιέται στο 0.15-0.46 m. **Η μετατόπιση του διαδρόμου
είναι ακριβώς η διαφορά:**

```
progress   |map→odom| ζωντανό   μετατόπιση από το παγωμένο
   5.60           0.2166                 0.2223
  18.70           0.4564                 0.4364
  21.30           0.2178                 0.2327
```

Οι μπάλες μένουν στο `map`· η διαδρομή είναι καρφωμένη στο `odom`. Το `odom`
είναι πλαίσιο νεκρού λογισμού που **φυσικά** παρασύρεται· το `map→odom` είναι
ακριβώς η μέτρηση αυτής της παράσυρσης — και η παγωμένη διαδρομή δεν τη βλέπει
ποτέ.

**Φυσική επιβεβαίωση (Gazebo truth vs τις μπάλες που όντως τοποθετήθηκαν),
πλευρική θέση στο επίπεδο του στομίου (μισό πλάτος 0.205 m):**

| μπάλα | y στο στόμιο | έκβαση |
| --- | --- | --- |
| ball_01 | **-0.065** | ΜΕΣΑ → **συλλέχθηκε** |
| ball_02 | +0.177 | μέσα, 2.8 cm από το χείλος → όχι |
| ball_00 | -0.221 | έξω κατά 1.6 cm → όχι |
| ball_03 | **-0.397** | έξω κατά 19 cm → όχι (η target-2) |

Χωρίς την παράσυρση κάθε y θα ήταν ≈ 0 (ο planner κεντράρει τον διάδρομο στη
μπάλα). Με μετατόπιση 0.22-0.44 m οι μπάλες σπρώχνονται προς και πέρα από το
χείλος — **ακριβώς αυτό που μετρήθηκε**.

**11H — ταξινόμηση:** `TRACKER_USES_ODOM_CORRECTLY_BUT_MAP_DRIFTS` (κύρια), με
`LOCALIZATION_DRIFT` ως μέγεθος (το `map→odom` φτάνει 0.456 m σε ~100 s).
**ΑΠΟΡΡΙΠΤΟΝΤΑΙ:** `TF_TIMESTAMP_ERROR` (10 ms), `TELEMETRY_MISREPORTING`
(6.9e-18), `PATH_FRAME_TRANSFORM_BUG` (ο μετασχηματισμός είναι σωστός — απλώς
γίνεται μία φορά), `POSE_SOURCE_MISMATCH` μέσα στον core (pose και path είναι
**και τα δύο** odom).

**11G — έλεγχοι που το κλειδώνουν:** 3 gtest (το `lateral_error_m` και το
`heading_error_rad` πρέπει να αναπαράγονται από το δημοσιευμένο σημείο
αναφοράς· το σημείο αναφοράς συνοδεύει και το tube abort) + 8 pytest στον
`analyze_frame_diagnosis.py` (κάθε ένας σπάει μία ιδιότητα και απαιτεί να τον
πιάσει ο αντίστοιχος έλεγχος).

**Status:** διαγνωστικό ΜΟΝΟ. Καμία αλλαγή σε gains, ανοχές, γεωμετρία ή SLAM.
Ο tracking core **δικαιώνεται**: υπολογίζει ακριβώς ό,τι δηλώνει. Το σφάλμα
είναι αρχιτεκτονικό — ο διάδρομος αγκυρώνεται σε πλαίσιο που παρασύρεται, ενώ οι
στόχοι μένουν στο `map`.

---

## #73 — Φάση 12: η διαδρομή αγκυρώνεται πλέον στο `map`. Το συμβόλαιο ισχύει — και **αποκαλύφθηκε** το πραγματικό σφάλμα παρακολούθησης

**Τι άλλαξε.**

1. `_ExecutionPlanTransformer` → `_ExecutionFrameContract`: **δεν** μετασχηματίζει
   πλέον τη διαδρομή σε `odom`. Επιστρέφει το plan αυτούσιο (ίδιο plan_id, ίδια
   segments/ids/progress/crossings/γεωμετρία) και **καταγράφει** το `map→odom`
   που παλιά πάγωνε, ως παρατηρησιμότητα.
2. `CollectionNav2Controller::pose_in_plan_frame()`: νέος adapter στο σύνορο ROS.
   Φέρνει την πόζα του Nav2 (`odom`) στο πλαίσιο της διαδρομής (`map`) σε **κάθε**
   control update, με lookup στη **χρονοσήμανση της ίδιας της πόζας** (fallback
   στο latest μόνο σε ExtrapolationException, και **καταγράφεται**).
3. **Σκληρό συμβόλαιο πλαισίου**: πόζα και διαδρομή σε διαφορετικά ή ανώνυμα
   πλαίσια → `std::runtime_error`, ποτέ σιωπηλή σύγκριση συντεταγμένων.
4. Ο `CollectionTrackingCore` **δεν** άγγιξε ROS/TF: παραμένει καθαρά
   μαθηματικός πάνω σε ομόφρακτες συντεταγμένες. Ο adapter είναι το frame-aware
   στρώμα.
5. Καμία αλλαγή σε gains, ανοχές, cmd_vel routing, διεπαφή εντολών.

**Ζωντανή επαλήθευση (3 επαναλήψεις `two_passes_with_connector`):**

| μέτρηση | αποτέλεσμα |
| --- | --- |
| frames (pose, path) | **(`map`, `map`)** σε 100% των updates |
| tracker self-consistency | διατηρήθηκε ακριβής |
| ηλικία transform στη χρήση | med **10 ms**, max 38 ms· **0** fallback σε latest |
| διορθώσεις `map→odom` κατά τη διαδρομή | 6 συνολικά, **0.143-0.269 m** (όλες > tube) |
| μεταβολή cross-track τη στιγμή της διόρθωσης | **+0.0000 έως +0.0008 m** |
| commanded linear | σταθερό 0.350 m/s |
| commanded angular | max 0.587 rad/s (ίδια τάξη με πριν) |

**ΑΠΑΝΤΗΣΗ ΣΤΟ ΕΡΩΤΗΜΑ 8: οι διορθώσεις εντοπισμού ΔΕΝ προκαλούν άλματα.** Το
`map→base` που χρησιμοποιεί τώρα ο tracker είναι **συνεχές** — αυτός ακριβώς
είναι ο ρόλος του `map→odom`. Διόρθωση 0.269 m άλλαξε το cross-track κατά
**0.0008 m**. Καμία αστάθεια εντολών.

**ΟΜΩΣ — και τα 3 runs κόβουν στο ίδιο σημείο:**

```
aborted_tracking | trajectory_tube_exceeded | seg connector-0
r1 progress 1.092 m   r2 1.111 m   r3 1.012 m
```

Το cross-track ξεκινά στο **0.0000** (ο διάδρομος ξεκινά ακριβώς πάνω στο ρομπότ)
και αυξάνεται **ομαλά** σε 0.082 m μέσα σε 0.9 m διαδρομής, ξεπερνώντας το tube
των **0.10 m** στο ~1.0 m. Καμία μπάλα δεν προσεγγίστηκε.

**Η ερμηνεία.** Δεν είναι ασυνέχεια — είναι **ορατότητα**. Ο διάδρομος καρφωμένος
στο `odom` μετακινούνταν μαζί με το ρομπότ, οπότε το cross-track που μετρούσε ο
tracker ήταν ~0.012 m: μετρούσε την απόκλιση από μια διαδρομή που παρασυρόταν
μαζί του. Στο `map` μετρά την **πραγματική** απόκλιση από τον σταθερό διάδρομο.

**Και αυτό το νούμερο ήταν ήδη γνωστό:** ο evaluator της Φάσης 10, που
ανακατασκεύαζε το cross-track από τις πόζες `map` έναντι της διαδρομής `map`,
είχε μετρήσει **median max cross-track 0.159 m στα passes και 0.190 m στους
connectors** (#71 §5). Ένα tube 0.10 m **δεν μπορεί** να περάσει σε πλαίσιο
`map`: η ανοχή ήταν βαθμονομημένη πάνω σε ένα σήμα που έκρυβε το σφάλμα.

**ΣΤΑΣΗ ΚΑΤ' ΕΝΤΟΛΗ (§8).** Δεν πειράχθηκε tube, gains ούτε προστέθηκε
εξομάλυνση. Το πρωτεύον κριτήριο αποδοχής (target-2 mouth presentation) **δεν
μετρήθηκε** — η διαδρομή τερματίζει πριν φτάσει σε οποιοδήποτε crossing.

**Status:** το σφάλμα πλαισίου **διορθώθηκε και κλειδώθηκε** (9 pytest + 3 gtest
συμβολαίου πλαισίου, 608 pytest σύνολο). Εκκρεμεί **απόφαση χρήστη**: οι ανοχές
παρακολούθησης πρέπει να ξαναβαθμονομηθούν έναντι του **αληθινού** σφάλματος στο
`map`, ή το πραγματικό σφάλμα 0.16-0.19 m να μειωθεί. Και τα δύο είναι αλλαγές
ελεγκτή που απαιτούν έγκριση.

---

## #74 — Φάση 13 (διαγνωστική): το σφάλμα είναι **κόψιμο στροφής στους connectors** — και η αγκύρωση στο `map` έφερε τις μπάλες **μέσα** στο στόμιο

**Διαγνωστικό override (ΠΡΟΣΩΡΙΝΟ, ΑΦΑΙΡΕΘΗΚΕ).** `COLLECTION_DIAGNOSTIC_TUBE_M=0.60`
(env-gated, τα shipped 0.1 / 0.2 m αμετάβλητα στο YAML). Επιλέχθηκε ως η μικρότερη
τιμή που δεν κόβει, βάσει του μετρημένου φακέλου της Φάσης 10 (max 0.579 m).
Καμία άλλη ανοχή/gain δεν πειράχθηκε· τα σφάλματα καταγράφηκαν **χωρίς clipping**.

**Runs:** 3× `two_passes_with_connector` + 1× `real_scan` (51.7 m). Frames
(`map`,`map`) σε 100% των updates και στα 4.

**ΦΑΚΕΛΟΣ ΣΦΑΛΜΑΤΟΣ (3045 samples):**

| segment | median | p90 | p95 | max |
| --- | ---: | ---: | ---: | ---: |
| straight pass | **0.0106** | 0.0265 | 0.0369 | 0.1016 |
| **pass CORE** (>0.3 m από κάθε άκρο) | **0.0089** | 0.0253 | 0.0350 | **0.0426** |
| connector | **0.0632** | 0.1339 | 0.1480 | **0.2135** |

**Οι ευθείες παρακολουθούνται στο ~1 cm. Οι connectors είναι 6-15× χειρότεροι.**

**Εξέλιξη μέσα στη στροφή (ίδιο μοτίβο και στα 4 runs):**

```
είσοδος connector      cross-track 0.0000-0.0156
κορυφή καμπυλότητας    1.32-1.84 1/m  →  0.041-0.156
χειρότερο              0.149-0.180 (λίγο μετά την κορυφή)
έξοδος                 0.008-0.042
πρώτη ευθεία μετά      median 0.011-0.016
```

Αυξάνεται όταν αρχίζει η στροφή, κορυφώνεται στη στροφή, **επανέρχεται σχεδόν
στο μηδέν στην επόμενη ευθεία** — ορισμός κοψίματος στροφής.

**Το lookahead ΔΕΝ ενοχοποιείται.** `corr(cross-track, lookahead) = -0.083`,
`corr(cross-track, |curvature|) = +0.350`, `corr(speed) = -0.029`. Ο λόγος
`lookahead × curvature` (= lookahead/ακτίνα) έχει median **0.26**, p95 0.69,
max 0.88 — **το lookahead είναι ΜΙΚΡΟΤΕΡΟ από την ακτίνα στροφής παντού**.

**Η ΑΠΟΔΕΙΞΗ ΠΟΥ ΜΕΤΡΑΕΙ — φυσική παρουσίαση μπάλας (Gazebo truth, real_scan):**

```
target-3  -0.062 m  ΜΕΣΑ      target-2  -0.012 m  ΜΕΣΑ  ✓confirmed
target-6  +0.152 m  ΜΕΣΑ      target-5  +0.126 m  ΜΕΣΑ  ✓confirmed
target-4  +0.110 m  ΜΕΣΑ      target-7  +0.030 m  ΜΕΣΑ
```

**6 στις 6 μπάλες που προσεγγίστηκαν βρέθηκαν ΜΕΣΑ στο στόμιο (±0.205 m).**
Σύγκριση με τη Φάση 12/#72 (odom-frozen): +0.177 / -0.221 / -0.397 m — στο χείλος
ή έξω. **Το σφάλμα παρουσίασης έπεσε από 0.22-0.40 m σε 0.01-0.15 m.**
2 συλλογές επιβεβαιώθηκαν (τα υπόλοιπα είναι μηχανισμός, εκτός σκοπού).

**Ανάλυση ανοχών (ΜΟΝΟ offline, τίποτα δεν άλλαξε):**

| tube | samples > tube | ποια segments |
| ---: | ---: | --- |
| 0.10 | 24.6% | connector **και** pass |
| 0.15 | 3.7% | μόνο connector |
| 0.20 | **0.1%** | μόνο connector |
| 0.25 / 0.30 | 0.0% | κανένα |

**Και η ερώτηση-κλειδί: όχι, η αύξηση του tube ΔΕΝ κρύβει αποτυχία συλλογής.**
Το υπολειπόμενο σφάλμα ζει στους connectors, όπου **δεν υπάρχουν μπάλες**· στις
ευθείες όπου γίνεται η σύλληψη το σφάλμα είναι 0.009 m median, 0.043 m max.

**ΤΟ ΝΕΟ ΣΗΜΕΙΟ ΔΙΑΚΟΠΗΣ ΔΕΝ ΕΙΝΑΙ ΤΟ TUBE.** Και τα 4 runs έκοψαν σε
`heading_error_exceeded` στην **είσοδο ευθείας μετά από connector**:

| pass entry | αρχικό | στα 0.2 m | στα 0.5 m | έκβαση |
| --- | ---: | ---: | ---: | --- |
| 4 passes που πέρασαν | 0.072-0.184 | 0.052-0.086 | 0.022-0.037 | OK |
| 4 passes που έκοψαν | **0.302-0.324** | **0.185-0.237** | — | abort |

Το `required_entry_m = 0.2 m` (0.57 s στα 0.35 m/s) δεν αρκεί για να πέσει ένα
σφάλμα εισόδου 0.30 rad κάτω από το capture gate 0.15 rad. Όσα μπήκαν με <0.19
rad ευθυγραμμίστηκαν άνετα.

**Ταξινόμηση:** `CONNECTOR_CORNER_CUTTING` (φάκελος πλευρικού σφάλματος) +
`TRANSITION_ERROR` (τα aborts). **ΑΠΟΡΡΙΠΤΟΝΤΑΙ:** `LOOKAHEAD_TOO_AGGRESSIVE`
(lookahead/ακτίνα 0.26 median), `GLOBAL_MAP_TRACKING_ERROR` (pass core 0.0089 m),
`SPEED_CURVATURE_MISMATCH` (corr με ταχύτητα -0.029, ταχύτητα σταθερή 0.35).

**Status:** διαγνωστικό ΜΟΝΟ. **Το override αφαιρέθηκε, οι production ανοχές
(0.1 / 0.2 m) είναι αμετάβλητες στο YAML και στον κώδικα** — επιβεβαιωμένο με
`git diff` και rebuild. Εκκρεμεί απόφαση: Option A (ανοχές) ή Option B (ελεγκτής).

---

## #75 — Φάση 14: βαθμονόμηση των δύο πυλών. **Πρώτη ολοκληρωμένη διαδρομή** (99.9%)

**§1 — Τι είναι οι δύο πύλες (τεκμηρίωση πριν την αλλαγή).**

| | `max_lateral_error_m` = 0.10 | `safety.trajectory_tube_radius_m` = 0.20 |
| --- | --- | --- |
| που επιβάλλεται | **C++ tracking core**, ανά sample, ανά segment profile | Python executor/port |
| τι κάνει | `kTrajectoryTubeExceeded` → **abort διαδρομής** | υπολογίζει `trajectory_tube_ok` στο `PathFollowerResult` |
| που καταναλώνεται | άμεσα, σφάλμα εκτέλεσης | **μόνο** στο `_can_resume_after_pause` (προϋπόθεση resume μετά από safety pause) |
| ευθύνεται για τα aborts των Φάσεων 12/13 | **ΝΑΙ** | όχι |

Δεν είναι το ίδιο invariant σε δύο επίπεδα: το ένα είναι εσωτερική πύλη ελεγκτή,
το άλλο συμβόλαιο executor για επανεκκίνηση. **Άρα χρειαζόταν διόρθωση μόνο το
0.10.** Το εξωτερικό 0.20 έμεινε ως έχει — και τώρα τα δύο επίπεδα συμφωνούν.

**§2/§3 — Οι αλλαγές (μόνο δύο τιμές στο `collection_route.yaml`):**

```
planning.default_execution_profile.max_lateral_error_m   0.1 → 0.2
planning.default_execution_profile.required_entry_m      0.2 → 0.8
```

Αμετάβλητα: `max_heading_error_rad 0.15`, `required_run_in_m 1.0`,
`trajectory_tube_radius_m 0.2`, gains, lookahead, ταχύτητες, planner.
Το `required_entry_m` χρησιμοποιείται **μόνο** στο heading grace του core
(`collection_tracking_core.cpp:188`) — δεν αγγίζει γεωμετρία planner. Στους
transit connectors δεν έχει καμία επίδραση (το grace ισχύει μόνο σε segments με
crossings).

**§4 — Το invariant, επιβεβλημένο στον κώδικα.** Νέος έλεγχος στον constructor
του core: capture segment του οποίου το **πρώτο crossing πέφτει μέσα** στο
alignment allowance **απορρίπτεται** (`capture segment shorter than its heading
entry allowance`). Καμία σιωπηλή παραίτηση από την πύλη πάνω στη μπάλα.

**§8 — Ντετερμινιστικό σενάριο, 3/3:**

| run | progress | heading @ 1ο crossing | έκβαση |
| --- | --- | --- | --- |
| r1 | **21.64 / 23.39 (92.5%)** | 0.0001 / 0.0001 rad | 1 confirmed |
| r2 | **21.38 / 23.07 (92.6%)** | 0.0006 / 0.0007 rad | 2 confirmed |
| r3 | **21.50 / 23.07 (93.2%)** | 0.0073 / 0.0002 / 0.0058 rad | 2 confirmed |

Σύγκλιση heading στην είσοδο pass (8 εισόδους): **0.064-0.323 rad στην είσοδο →
0.040-0.193 στα 0.2 m → 0.009-0.040 στα 0.5 m → 0.0001-0.011 στα 0.8 m →
0.0001-0.0073 στο crossing.** Το προηγούμενο σημείο διακοπής (5.95-6.03 m) το
πέρασαν **και τα 3**.

**§9 — Connector envelope έναντι του κατωφλίου 0.20 m:** max **0.1684**,
p95 0.1444, περιθώριο **+0.0316 m**. **Κανένα abort δεν προήλθε από τη γεωμετρία
connector.**

**§10 — real_scan, 3 runs:**

| run | progress | confirmed | truth | έκβαση |
| --- | --- | --- | --- | --- |
| r1 | **49.73 / 49.78 = 99.9%** | 4/10 | 5 | **`route_completed`** |
| r2 | 48.50 / 48.54 = 99.9% | 4/11 | 5 | `non_monotonic_progress` στο τέλος |
| r3 | 15.06 / 50.26 = 30.0% | 1/9 | 1 | `trajectory_tube_exceeded` |

Φάση 13 (tube 0.60, διαγνωστικό): 30.7/51.7 = **59%**. Τώρα **99.9%** — και η
**πρώτη πλήρης ολοκλήρωση διαδρομής** όλης αυτής της γραμμής δουλειάς.
17 pass entries, heading στο 1ο crossing **0.0001-0.0508 rad** — όλα εντός 0.15.

**§12 — Φυσική παρουσίαση (Gazebo truth):** ντετερμινιστικό **9 μέσα / 3 έξω**,
real_scan **19 μέσα / 2 έξω / 6 μη προσεγγισμένες**. Καμία οπισθοδρόμηση έναντι
της Φάσης 13. Οι 3 «έξω» του ντετερμινιστικού είναι όλες η **target-1** στα
+0.32/+0.33 m — η πρώτη μπάλα της διαδρομής, σταθερά, και στα 3 runs.

**ΤΑ ΕΝΑΠΟΜΕΙΝΑΝΤΑ ABORTS ΔΕΝ ΕΙΝΑΙ ΤΟ TUBE — ΕΙΝΑΙ ΑΛΜΑΤΑ ΕΝΤΟΠΙΣΜΟΥ.**
Κάθε abort έπεται **μεγάλης ασυνεχούς διόρθωσης `map→odom`**:

```
ντετερμινιστικό  r1 0.347 m στο 21.61 → abort 21.64
                 r2 0.313 m στο 21.32 → abort 21.38
                 r3 0.361 m στο 21.35 → abort 21.50
real_scan        r3 0.265 m στο 15.01 → abort 15.06
                 r1/r2 0.233/0.242 m στο ~14.6 → ΕΠΙΒΙΩΣΑΝ
```

Σε 247 διορθώσεις: median 0.022, p95 0.100, **μόνο 6 (2.4%) ξεπερνούν τα
0.20 m**. Το tube δεν είναι στενό για την παρακολούθηση — δέχεται χτύπημα από
σπάνιο άλμα εντοπισμού. **ΔΕΝ διευρύνθηκε ξανά** (εντολή §9).

**Status:** και οι 9 κριτήρια αποδοχής της Φάσης 14 ικανοποιούνται. Επόμενο
θέμα: **ασυνέχειες εντοπισμού 0.23-0.36 m** — και το `failure()` του core
μηδενίζει `lateral_error_m`/`heading_error_rad`, οπότε το δείγμα που ενεργοποιεί
το abort **δεν δημοσιεύεται ποτέ**. Χωρίς αυτό δεν διαγιγνώσκεται.

---

## #76 — Φάση 15 (διαγνωστική): η τηλεμετρία αποτυχίας λέει πλέον αλήθεια — και ο ένοχος είναι **άλμα εντοπισμού**, όχι η παρακολούθηση

**§1/§2 — Το ελάττωμα και η διόρθωση.** Το `failure()` επέστρεφε
default-constructed `TrackingResult`: το **μοναδικό** update που ενεργοποιεί μια
πύλη ανέφερε `lateral 0.000 / heading 0.000`. Κάθε abort στα logs φαινόταν
τέλεια παρακολούθηση.

Διόρθωση: το `path_heading_error` υπολογίζεται **πριν** από κάθε πύλη (καθαρή
συνάρτηση — δεν μετακινεί το σημείο αποτυχίας) και ένας decorator `with_geometry`
προσαρτά σε **κάθε** έξοδο μετά την προβολή: lateral, heading, progress,
reference point, previous/raw projection progress. Νέο `has_geometry` ξεχωρίζει
«μετρήθηκε» από «δεν υπολογίστηκε ποτέ» — **η απουσία δεν γίνεται μηδέν**.

**Ζωντανή απόδειξη:**

```
ΠΡΙΝ: trajectory_tube_exceeded | seg connector-2 ... lat_err 0.000m head_err 0.000rad
ΤΩΡΑ: trajectory_tube_exceeded | seg connector-2 ... lat_err 0.206m head_err 0.194rad
```

**§3 — 6 νέα gtest**, μαζί με ένα που κλειδώνει ότι **το σημείο αποτυχίας δεν
μετακινήθηκε**: 0.199 τρέχει / 0.201 κόβει, 0.149 τρέχει / 0.151 κόβει.

**§4/§6 — 6 runs (3 two_passes + 3 real_scan): 5 aborts, 1 πλήρης ολοκλήρωση**
(`two_passes-r1`: `route_completed` → follow-up scan → `completed_no_targets`,
2 confirmed). **Όλα τα aborts είναι `trajectory_tube_exceeded`.**
Το `non_monotonic_progress` και το τερματικό ανώμαλο των 2-5 cm **ΔΕΝ
αναπαρήχθησαν**.

**§5 — Κατανομή.** Διορθώσεις `map→odom` (n=178): median 0.0349, p90 0.0873,
p95 0.1005, p99 0.2640, max 0.3473 m. Πιο καθαρό μέγεθος είναι το **βήμα της
πόζας που δίνεται στον core**: κανονικό median **0.0139 m**, max **0.11 m** —
τα άλματα **0.17-0.35 m**. Διμερής διαχωρισμός, άρα «μεγάλο» = **>0.15 m**,
από τα δεδομένα και όχι αυθαίρετα.

**§7 — Χρονισμός: κατηγορία A σε 5/5.** Σε **κάθε** abort η αποτυχία συμβαίνει
στο **ίδιο** update με το άλμα:

```
prog 16.122  lat 0.0101  pose step 0.0110  truth err 0.0162
prog 16.329  lat 0.2844  pose step 0.3433  truth err 0.3195   <-- FAIL
```

Το ρομπότ κινήθηκε φυσικά 1.4-2.5 cm· η **εκτίμηση** μετακινήθηκε 0.24-0.35 m.

**§8 — Σύγκριση με Gazebo truth: δύο διακριτές κατηγορίες.**

| άλμα | βήμα | σφάλμα vs truth | abort |
| --- | ---: | --- | --- |
| πρώιμα (prog 0.7-1.4 m), 5 συμβάντα | 0.17-0.27 | **0.39-0.44 → 0.015-0.19 (ΠΡΟΣ)** | όχι |
| όψιμα, 5 από 6 | 0.24-0.35 | **0.006-0.075 → 0.32-0.35 (ΜΑΚΡΙΑ)** | ναι |
| όψιμο, 1 (real_scan-r3) | 0.337 | **0.307 → 0.070 (ΠΡΟΣ)** | **ναι** |

Τα πρώιμα είναι **έγκυρη** σύγκλιση μετά το scan. Τα όψιμα, στα 5/6, πετούν την
εκτίμηση **μακριά** από την αλήθεια ενώ ο εντοπισμός ήταν ήδη ακριβής στο ~1 cm.
Και το 6ο — μια **έγκυρη** επαναφορά — **κόβει κι αυτό**.

**§8 (γεωμετρία στο abort):** lateral **0.205, 0.206, 0.284, 0.302, 0.321 m** —
όλα γνήσια πάνω από την πύλη 0.20. **Η πύλη κάνει ακριβώς τη δουλειά της· το
πρόβλημα είναι η είσοδός της.**

**§11/§12 — Ταξινόμηση:** `SLAM_LOCALIZATION_JUMP` (κύρια, 5/6 όψιμα άλματα
μακριά από την αλήθεια) **και** `VALID_RELOCALIZATION_CAUSES_TRANSIENT_PATH_ERROR`
(1/6 — και είναι ο μηχανισμός με τον οποίο **ακόμη και σωστή** διόρθωση κόβει τη
διαδρομή). **ΑΠΟΡΡΙΠΤΟΝΤΑΙ:** `TRACKER_RECOVERY_INSUFFICIENT` (δεν δίνεται
κανένα update ανάκαμψης — κόβει στο ίδιο), `PROGRESS_PROJECTION_INSTABILITY`
(το reference point μετακινήθηκε 0.12 m ενώ η πόζα 0.33 — η προβολή ακολούθησε
την πόζα, δεν πήδηξε), `NOT_LOCALIZATION_RELATED`.

**Status:** διαγνωστικό. Καμία αλλαγή σε gains, ανοχές ή SLAM. Οι ανοχές της
Φάσης 14 (0.20 m / 0.8 m / 0.15 rad) παραμένουν ως έχουν. Το τερματικό ανώμαλο
δεν αναπαρήχθη — μέρος του μυστηρίου του («μηδενικό σφάλμα αλλά tube exceeded»)
**ήταν** αυτό ακριβώς το ελάττωμα τηλεμετρίας.

---

## #77 — Φάση 16: ανίχνευση re-anchoring από κινηματική. Μερικώς επαρκής — και η αιτία του `non_monotonic_progress` **βρέθηκε**

**§1 — Ο κανόνας, από τα δεδομένα.**

```
plausible_step_m = segment.max_speed_mps * elapsed_s + 0.10
re-anchoring     = elapsed_s > 0  &&  pose_step_m > plausible_step_m
```

Το 0.10 m δεν είναι αυθαίρετο: σε **7597** μετρημένα updates το βήμα πόζας δεν
ξεπέρασε ποτέ τα **0.111 m**, από τα οποία το πολύ 0.037 m εξηγείται κινηματικά
στις παρατηρούμενες περιόδους — τα υπόλοιπα είναι θόρυβος εκτίμησης. Το
μικρότερο άλμα που μετρήθηκε ποτέ ήταν **0.209 m**. Με dt=0.04 s το όριο βγαίνει
0.148 m: πάνω από κάθε κανονική κίνηση, κάτω από κάθε άλμα. **Χωρίς
`elapsed_s` δεν γίνεται καμία δήλωση** — δεν μπορείς να πεις ότι ένα βήμα ήταν
αδύνατο χωρίς να ξέρεις σε πόση ώρα έγινε. Το Gazebo truth **δεν** χρησιμοποιείται.

**§2/§3 — Σημασιολογία.** Καμία ανοχή δεν άλλαξε (0.20 / 0.8 / 0.15 / 1.0 / 0.20).
Σε ανιχνευμένο re-anchoring: η γεωμετρία υπολογίζεται και δημοσιεύεται κανονικά,
το update σημαίνεται ρητά, και **μόνο** η ετυμηγορία του tube αναβάλλεται — για
**ένα** update. Καμία εξομάλυνση πόζας, κανένα πάγωμα `map→odom`, καμία αλλαγή
διαδρομής ή εντολών.

**§5 — Επαναλαμβανόμενα άλματα, ρητά:** μια αναβολή δεν μπορεί να ακολουθηθεί από
δεύτερη (`deferred_previous_update_`). `jump→jump→jump` ⇒ το πρώτο αναβάλλεται,
το δεύτερο **κόβει**. `jump→ordinary→jump` ⇒ δύο ανεξάρτητα γεγονότα, ένα
deferral το καθένα. **Αδύνατο να προκύψει ατέρμονη χάρη, εξ ορισμού.**

**§6 — Οι άλλες πύλες ΔΕΝ κατασταλθηκαν.** Μετρήθηκε τι κάνει μια ασυνέχεια:
heading ±0.0004 rad σε ευθεία (πύλη 0.15), έως +0.207 rad σε connector (πύλη
0.5, δεν πυροδότησε ποτέ)· progress **0 από 16** άλματα πήγε προς τα πίσω. Καμία
απόδειξη δεν δικαιολογεί καταστολή τους.

**§8 — 38 gtest** (10 νέα), όλα περνούν: κανονική κίνηση ποτέ δεν παίρνει χάρη·
σταδιακή απόκλιση >0.20 m κόβει κανονικά· ένα άλμα 0.30 m αναβάλλεται· **αν το
σφάλμα παραμείνει, κόβει στο επόμενο update**· έγκυρο re-anchoring αντιμετωπίζεται
ίδια με εσφαλμένο (ο ελεγκτής **δεν** κρίνει αν ο SLAM είχε δίκιο)· ντετερμινισμός.

**§9/§10 — ΗΤΑΝ ΑΡΚΕΤΟ ΕΝΑ UPDATE; ΜΕΡΙΚΩΣ.** 5 deferrals πάνω από την πύλη:

| υπόλοιπο lateral μετά το άλμα | ανάκαμψη σε 1 update |
| --- | --- |
| 0.2026 m (real_scan-r2) | **ΝΑΙ** → 0.1955, και η διαδρομή **ΟΛΟΚΛΗΡΩΘΗΚΕ με 6 μπάλες** |
| 0.2348, 0.3080, 0.3190, 0.313 m | **ΟΧΙ** — κόβει στο επόμενο |

Η χάρη σώζει τις **οριακές** περιπτώσεις. Ένα υπόλοιπο 0.23-0.32 m σημαίνει ότι
η εκτίμηση κάθεται 1/3 μέτρου εκτός διαδρόμου· το ρομπότ πρέπει να το κλείσει
**φυσικά**, που θέλει ~1 s (≈25 updates), όχι ένα.

**Ζωντανά αποτελέσματα:** ντετερμινιστικό 99.9% / 95.2% / 92.9% (2/1/2 μπάλες)
έναντι 99.9% / 93.5% / 94.8% (2/1/2) της Φάσης 15 — **καμία οπισθοδρόμηση**.
real_scan: 41% / **route_completed με 6 confirmed, 6 retained** / 30% — έναντι
30/33/40% με 1/1/2 της Φάσης 15. **Το r2 είναι το καλύτερο real_scan αποτέλεσμα
μέχρι σήμερα.**

**§12 — Ο SLAM δεν κρύφτηκε:** τα μεγάλα άλματα εξακολουθούν να καταγράφονται
έναντι truth. 5/6 όψιμες διορθώσεις **μακριά** από την αλήθεια παραμένει
ανοιχτό ελάττωμα.

**ΜΠΟΝΟΥΣ — η αιτία του `non_monotonic_progress` ΒΡΕΘΗΚΕ** (real_scan-r1, με την
αληθινή πλέον τηλεμετρία):

```
prog 19.5560  prev 19.5560  raw  9.7422  delta -9.8138 m  lat 0.0142  pose step 0.0139
```

Η **ακατέργαστη** προβολή πήδηξε **9.81 m πίσω** ενώ η πόζα κινήθηκε κανονικά
0.0139 m. Είναι αυτο-εγγύτητα διαδρομής: η διαδρομή επιστρέφει κοντά στον εαυτό
της και το `progress_projection_window_m = 10.0` είναι **οριακά** αρκετά φαρδύ
ώστε να δεχτεί σημείο 9.81 m πίσω. **`PROGRESS_PROJECTION_INSTABILITY`, ΟΧΙ
localization** (`NOT_LOCALIZATION_RELATED`: βήμα πόζας 0.0139 m).

**Status:** ο μηχανισμός κρατιέται — όλα τα κριτήρια ασφάλειας ισχύουν, καμία
οπισθοδρόμηση, η τηλεμετρία κάνει κάθε αναβολή ρητή. Δεν λύνει όμως το βασικό:
τα **εσφαλμένα άλματα SLAM 0.24-0.35 m** πρέπει να διορθωθούν στην πηγή τους.

---

## #78 — Φάση 17A: η αιτία του `non_monotonic_progress` ήταν ο **ανιχνευτής**, όχι η προβολή

**§1 — Τι κάνει σήμερα ο αλγόριθμος.** Το `project_monotonically` παράγει **δύο**
προβολές:

| | εύρος αναζήτησης | χρήση |
| --- | --- | --- |
| **bounded** (αποδεκτή) | `[last_progress, last_progress + 10.0]` — **μόνο εμπρός** | γίνεται το `progress_s` |
| **raw** (ανιχνευτής) | **όλη η διαδρομή**, φιλτραρισμένη με «όχι πάνω από 10 m πίσω» | τροφοδοτεί το `non_monotonic_progress` |

**Η αποδεκτή πρόοδος δεν πήγε ΠΟΤΕ πίσω** — είναι μονότονη εξ ορισμού. Στο
ζωντανό συμβάν η αποδεκτή έμεινε **19.5560** και μόνο η raw έδειξε 9.7422. Άρα το
abort ήταν **ψευδώς θετικό του ανιχνευτή**, όχι άλμα προόδου.

**Γιατί επιτράπηκε:** το `progress_projection_window_m = 10.0` έκανε **δύο**
δουλειές — ορίζοντα αναζήτησης εμπρός **και** ανοχή «πόσο πίσω μετράει» — και η
διαδρομή επέστρεφε **9.81 m** κοντά στον εαυτό της, οριακά μέσα στο παράθυρο.
Σε αυτο-τομή οι δύο κλάδοι συμπίπτουν και το αυστηρό `<` κρατά τον **πρώτο**,
δηλαδή τον παλαιότερο.

**§3 — Το invariant (από το συμβόλαιο, όχι από το παράδειγμα).** Το συμβόλαιο
είναι **μόνο εμπρός** (`allow_reversing = false`, επιβεβλημένο στο
`valid_profile`), άρα η αποδεκτή πρόοδος δεν μπορεί να μειωθεί καθόλου. Ο
ανιχνευτής υπάρχει για να πιάνει **φυσική** κίνηση προς τα πίσω, και:

> Ένας raw υποψήφιος μετράει ως ένδειξη κίνησης προς τα πίσω **μόνο** αν βρίσκεται
> εκεί όπου το ρομπότ θα μπορούσε φυσικά να έχει φτάσει από την προηγούμενη
> αποδεκτή πρόοδο.

```
reach_m = plan_max_speed_mps * elapsed_s + 0.10
admissible  <=>  |raw_progress - last_progress| <= reach_m
```

Στα 0.04 s: **±0.124 m**. Ένας κλάδος 9.81 m μακριά δεν είναι το ρομπότ.
**Χωρίς `elapsed_s` δεν υπάρχει κινηματική βάση** → μένει το παλιό παράθυρο, άρα
καμία αλλαγή συμπεριφοράς για καλούντες χωρίς χρόνο.

**§5 — Τι απορρίφθηκε:** (A) μικρότερο παράθυρο — συντονισμός σε ένα περιστατικό·
(B) σκέτο clamp μονοτονίας — **ήδη ισχύει**, δεν ήταν εκεί το πρόβλημα·
(D) topology-aware προβολή — μεγαλύτερη αλλαγή χωρίς πρόσθετη απόδειξη.
Επιλέχθηκε το (C).

**§11 — 48 gtest** (10 νέα). Το `AReachableNeighbourhoodRejectsTheSelfNearBranch`
**αποτυγχάνει** όταν αφαιρεθεί η διόρθωση (επαληθεύτηκε με προσωρινή επαναφορά).
Καλύπτονται: αναπαραγωγή του σφάλματος στο παλιό μονοπάτι, απόρριψη του
αυτο-εγγύς κλάδου, μονοτονία αποδεκτής προόδου, **γνήσια** κίνηση προς τα πίσω
εξακολουθεί να πιάνεται, jitter, ακίνητο ρομπότ, στροφή/μετάβαση, re-anchoring,
ντετερμινισμός.

**§12 — Το ζωντανό περιστατικό, ξαναπαιγμένο:** raw 9.7422 vs previous 19.5560
⇒ |9.8138| **έξω** από το ±0.1216 ⇒ απορρίπτεται ως ένδειξη ⇒ **κανένα**
`non_monotonic_progress`· αποδεκτή πρόοδος 19.5560 (αμετάβλητη), lateral 0.0142,
ετυμηγορία **RUNNING**.

**§13/§14 — Ζωντανά (7 runs):**

| | progress | μπάλες | abort |
| --- | --- | --- | --- |
| two_passes r1 | 93.3% | 2 | tube (άλμα SLAM) |
| two_passes r2 | **99.9%** | **3/4** | κανένα |
| two_passes r3 | 93.0% | 2 | tube (άλμα SLAM) |
| real_scan r1 | 90.1% | 5 | heading |
| real_scan r2 | **99.9%** | 5 conf / **7 retained** | κανένα |
| real_scan r3 | 19.5% | 0 | tube (άλμα SLAM) |

**`non_monotonic_progress`: 0 σε 7 runs.** Η αποδεκτή πρόοδος **μονότονη σε όλα**.
Το φίλτρο δουλεύει συνεχώς (577-3182 απορρίψεις ανά run) και στο real_scan-r3
απέρριψε υποψήφιο **raw 0.000 έναντι previous 9.925** — ακριβώς ο μηχανισμός του
#77, ζωντανά, πιασμένος.

**ΑΝΟΙΧΤΟ ΠΡΟΣ ΑΠΟΦΑΣΗ:** το ζωντανό `terminal_progress_tolerance_m` είναι
**0.30 m** (nav2_params), ενώ το reach είναι ±0.12-0.15 m. Άρα κανένας αποδεκτός
raw υποψήφιος δεν μπορεί πια να απέχει 0.30 m πίσω: **ο έλεγχος
`non_monotonic_progress` είναι πρακτικά ανενεργός στην παραγωγή**. Δεδομένου ότι
η αποδεκτή πρόοδος είναι ούτως ή άλλως μονότονη εξ ορισμού, αυτό δεν επηρεάζει
την ασφάλεια — αλλά είναι απώλεια διαγνωστικού και **δεν** ρυθμίστηκε.

**Status:** το `non_monotonic_progress` **λύθηκε**. Τα εναπομείναντα aborts είναι
τα γνωστά **εσφαλμένα άλματα SLAM 0.24-0.37 m** (#76/#77) — αμετάβλητα, ανοιχτά.

---

## #79 — Φάση 17B (διαγνωστική): η αιτία των «αλμάτων SLAM» είναι η **οδομετρία**, όχι ο scan matcher

**§1/§2 — Ο πραγματικός αγωγός (native Jazzy, από τα αρχεία, όχι από defaults):**

```
/diff_drive_controller/odom  --(ΜΟΝΟ vx)-->  ekf_filter_node (robot_localization)
/imu/data                    --(ΜΟΝΟ yaw-rate gyro z)-->      "
      → odom → base_footprint   (two_d_mode, world_frame=odom, 30 Hz)
slam_toolbox  localization_slam_toolbox_node  → map → odom @ 50 Hz
      map: runtime/maps/court_1783283137.posegraph  (φορτωμένος, mode=localization)
      base_frame base_link · scan /scan · use_sim_time true
      minimum_travel_distance 0.25 m · minimum_travel_heading 0.15 rad
      correlation_search_space_dimension 0.5 · do_loop_closing true (loop_search_max 3.0 m)
LiDAR: 360°, 500 δείγματα, 0.15-12.0 m, 31.25 Hz, z=0.578 m
```

**Η οδομετρία ΔΕΝ παρατηρεί ούτε πλευρική ταχύτητα ούτε απόλυτη κατεύθυνση:**
καθαρό dead reckoning με vx τροχών (skid-steer!) + ολοκλήρωση gyro.

**§3 — 43 μεγάλα άλματα (>0.15 m) σε 19 runs, φάσεις 15-17.** Κατανομή: 28 προς
την αλήθεια, 15 μακριά. Συχνότητα διορθώσεων: median **0.70 s** — ταιριάζει
ακριβώς με το `minimum_travel_distance 0.25 m` στα 0.35 m/s, άρα **κάθε διόρθωση
είναι ένα scan-match update**.

**§4/§10 — Ο διαχωριστής που ψάχναμε: η ΟΔΟΜΕΤΡΙΑ ΠΑΡΑΣΥΡΕΤΑΙ ΤΟΣΟ ΟΣΟ ΚΑΙ ΟΙ
ΔΙΟΡΘΩΣΕΙΣ.** Βαθμονομώντας κάθε πλαίσιο στα πρώτα 20 δείγματα:

| run | σφάλμα odom vs truth | σφάλμα map vs truth |
| --- | --- | --- |
| two_passes-r3 | med **0.296**, max **0.434 m** | med ~0.43, max 0.515 |
| real_scan-r3 | med 0.173, max 0.302 m | max 0.475 |
| real_scan-r2 | med 0.235, max **0.464 m** | max 0.520 |

Μέσα στα **πρώτα 2 m** διαδρομής η odom έχει ήδη **0.18-0.26 m** σφάλμα — εκεί
όπου ο connector-0 κάνει τη σφιχτή στροφή (καμπυλότητα έως 1.84 1/m), δηλαδή
ακριβώς εκεί που η skid-steer οδομετρία είναι χειρότερη.

**§5 — Ο άξονας:** και οι 43 διορθώσεις είναι σχεδόν καθαρά κατά **x** (|dx|/|step|
median **0.979** για τις καλές, **0.998** για τις κακές· διόρθωση yaw median
0.002-0.012 rad). Και το σφάλμα της οδομετρίας συσσωρεύεται στον **ίδιο άξονα**
(κλάσμα κατά x: 0.43-0.75). **Οι διορθώσεις κινούνται εκεί όπου η odom παρασύρεται.**

**§6 — Η ΣΥΜΜΕΤΡΙΑ ΑΠΟΡΡΙΠΤΕΤΑΙ ΜΕ ΜΕΤΡΗΣΗ.** Ray-cast 360° πάνω στη γεωμετρία
του κόσμου (fence_north/south y=±8.5, fence_east/west x=±16.5, net x=0) και
μέτρηση της ευαισθησίας της σάρωσης σε μετατόπιση 0.10 m:

| θέση | κατά x | κατά y | λόγος |
| --- | ---: | ---: | ---: |
| σημείο κακών αλμάτων (-4.75,+0.30) | 0.2789 | 0.2714 | **0.97** |
| αφετηρία (-7.32,-0.16) | 0.0748 | 0.0807 | 1.08 |
| (-9.05,-1.25) | 0.0783 | 0.0792 | 1.01 |

**Η παρατηρησιμότητα είναι συμμετρική** (0.93-1.08). Δεν υπάρχει ολίσθηση κατά
μήκος τοίχου. Ο ισχυρισμός «συμμετρία γηπέδου» **δεν στηρίζεται**. (Εξαίρεση: στο
κέντρο του γηπέδου, 0% x-constraint — αλλά εκεί δεν συμβαίνουν τα άλματα.)

**§8/§13 — Πρωτεύουσα ταξινόμηση: `ODOMETRY_DRIFT`.** Η odom παρασύρεται 0.17-0.46 m
ανά run, ο matcher τη διορθώνει με βήματα ίδιου μεγέθους στον ρυθμό των scan
updates, και επειδή το σφάλμα που κυνηγά είναι της ίδιας τάξης με τη διόρθωση,
άλλοτε πλησιάζει την αλήθεια και άλλοτε την προσπερνά. **ΑΠΟΡΡΙΠΤΟΝΤΑΙ:**
`ENVIRONMENT_AMBIGUITY` (μετρημένη συμμετρική ευαισθησία),
`TIMESTAMP/TF_ERROR` (Φάση 15: ηλικία πόζας 10 ms = 3.5 mm),
`POSE_GRAPH_JUMP` (localization mode με φορτωμένο χάρτη· οι διορθώσεις έρχονται
στον ρυθμό των scan updates, όχι σε loop-closure γεγονότα).

**§10 — Υλικό ή προσομοίωση;** **Και τα δύο, χειρότερα στο υλικό.** Η αιτία είναι
δομική: skid-steer vx + ολοκλήρωση gyro, χωρίς πλευρική ή απόλυτη παρατήρηση.
Το πραγματικό ρομπότ έχει τους ίδιους τέσσερις κινητήριους τροχούς και θα
ολισθαίνει περισσότερο σε χώμα.

**§11 — Η μικρότερη διόρθωση με απόδειξη:** η odom πρέπει να παρασύρεται λιγότερο
ΠΡΙΝ ο matcher κληθεί να τη διορθώσει. Με σειρά προτεραιότητας:
(α) βαθμονόμηση/διόρθωση της vx των τροχών στις στροφές (εκεί γεννιέται το
σφάλμα)· (β) πύκνωση των scan updates (`minimum_travel_distance` 0.25 → μικρότερο)
ώστε κάθε διόρθωση να είναι μικρότερη — **αλλά αυτό είναι SLAM tuning και δεν
έγινε**. Καμία παράμετρος δεν άλλαξε σε αυτή τη φάση.

**Status:** διαγνωστικό ΜΟΝΟ. Καμία αλλαγή σε SLAM, EKF, ελεγκτή, planner.

---

## #80 — Φάση 18 (μερική): οι τροχοί υπερεκτιμούν την απόσταση κατά **14-16% ΣΕ ΕΥΘΕΙΑ**

**§1 — Ο ακριβής δρόμος `vx` (από τα τρέχοντα αρχεία):**

```
4 joints (front/rear x left/right) -> diff_drive_controller/DiffDriveController
   left_wheel_names  = [rear_left, front_left]     (μέσος όρος ανά πλευρά)
   right_wheel_names = [rear_right, front_right]
   wheel_radius   = 0.085 m   ΤΑΥΤΙΖΕΤΑΙ με το URDF/SDF (0.085) -> ΟΧΙ λάθος παραμέτρου
   wheel_separation = 1.0 m   (γεωμετρικό track 0.70 — σκόπιμα διογκωμένο 1.43x
                               για skid-steer· επηρεάζει ΜΟΝΟ το omega, ΟΧΙ το vx)
   open_loop=false · publish_rate 50 Hz · enable_odom_tf=false
   vx = r*(omega_L + omega_R)/2      <- ανεξάρτητο του wheel_separation
-> /diff_drive_controller/odom --(ΜΟΝΟ vx)--> ekf_filter_node
   /imu/data --(ΜΟΝΟ yaw-rate)--> ekf  ->  odom -> base_footprint (30 Hz, two_d_mode)
```

**§3 — ΤΟ ΕΥΡΗΜΑ. Ευθεία γραμμή, μηδενική καμπυλότητα:**

| leg | truth m | wheel m | odom m | truth/wheel |
| --- | ---: | ---: | ---: | ---: |
| v=0.20 | 0.846 | 0.984 | 0.850 | **0.860** |
| v=0.35 | 2.317 | 2.779 | 2.331 | **0.834** |
| v=0.50 | 3.464 | 3.980 | 3.485 | **0.870** |

**Οι τροχοί υπερεκτιμούν κατά ~14-16% ΧΩΡΙΣ ΚΑΜΙΑ ΣΤΡΟΦΗ**, και ο λόγος είναι
πρακτικά **ανεξάρτητος ταχύτητας** (0.86 / 0.83 / 0.87) — υπογραφή καθαρού
σφάλματος κλίμακας, όχι φαινομένου εξαρτώμενου από την κίνηση.

**Αυτό αρκεί για να εξηγήσει τη Φάση 17B:** 2 m διαδρομής x 14% = **0.28 m** —
ακριβώς η παρατηρημένη παράσυρση 0.18-0.26 m στα πρώτα 2 m. **Δεν χρειάζεται
όρος καμπυλότητας.**

**§8 — Μέτρηση ή ολίσθηση;** Ξεκάθαρα ολίσθηση:
* `odom_vx − wheel_vx` = **+0.001 m/s** median → η ερμηνεία των τροχών από τον
  controller είναι **ακριβής**·
* `wheel_vx / commanded_v` = **1.0000** → οι τροχοί γυρίζουν **ακριβώς** όσο τους
  ζητήθηκε·
* `truth < wheel` κατά 14-16% → **το έδαφος διαφωνεί**.

Άρα: **wheel-to-ground slip / ενεργός ακτίνα κύλισης**, ΟΧΙ σφάλμα μέτρησης ή
παραμέτρου.

**§9 — Η κατεύθυνση καθαρίζεται από τη λίστα υπόπτων:** ολοκληρωμένο gyro
+0.0191 rad έναντι truth +0.0000 σε **232 s**· |EKF yaw − truth| median
**0.0096**, max 0.0172 rad.

**ΑΚΥΡΗ ΜΕΤΡΗΣΗ — ΤΟ ΣΑΡΩΜΑ ΚΑΜΠΥΛΟΤΗΤΑΣ ΔΕΝ ΕΓΙΝΕ.** Το harness οδήγησε το
ρομπότ **πάνω στο δίχτυ** (truth: x −8.000 → −0.894, δηλαδή 0.9 m από το δίχτυ
στο x=0) στα 29 s, ακριβώς όταν ξεκινούσε η πρώτη στροφή. Από εκεί το truth
πάγωσε (ακίνητο ρομπότ), οι τροχοί συνέχισαν να γυρίζουν και το odom «ταξίδεψε»
ως τα (20.3, 17.7) — 27 m φανταστικής διαδρομής. **Τα δεδομένα μετά τα 29 s είναι
άχρηστα.** Οι ενότητες 4-7, 10, 11 (καμπυλότητα, αριστερά/δεξιά, entry/exit,
connector-0, ταχύτητα, μοντέλα) **ΔΕΝ μετρήθηκαν**.

**§14 — ΣΥΝΘΗΚΗ ΔΙΑΚΟΠΗΣ ΕΝΕΡΓΟΠΟΙΗΘΗΚΕ** (πρώτη της λίστας): «το σφάλμα
κλίμακας σε ευθεία εξηγεί το μεγαλύτερο μέρος της παράσυρσης». Σταματώ.

**Ταξινόμηση: `GLOBAL_TRANSLATION_SCALE_ERROR`** (μηχανισμός: ολίσθηση τροχού
προς έδαφος στο Gazebo). Το `CURVATURE_DEPENDENT_SLIP` **δεν αποκλείεται** ως
πρόσθετος όρος — απλώς δεν χρειάζεται για να εξηγηθεί το μέγεθος, και δεν
μετρήθηκε.

**Status:** διαγνωστικό. Καμία αλλαγή σε SLAM/EKF/controller/planner.

---

## #81 — Φάση 18A2: με σωστό harness η κλίμακα είναι **0.928, όχι 0.86** — και ένας σταθερός συντελεστής αρκεί

**§1 — Διόρθωση harness.** Κάθε leg ξεκινά από **επαληθευμένο reset** (`gz set_pose`
στο (−8.0, −3.0, +π/2), έλεγχος truth εντός 0.05 m) μέσα σε καθαρό κουτί στο
δυτικό μισό — δίχτυ στο x=0, φράχτες x=±16.5 / y=±8.5, όλα μακριά. **18/19 legs
επαληθεύτηκαν** και ξεκινούν στο (−8.000, −3.000) με πρώτο βήμα 0.001-0.017 m.
Sim-only στοίβα (`TENNIS_LAUNCH_BRAIN=false`) → **κανένα SLAM δεν τρέχει**, άρα
το set_pose δεν ενοχλεί εντοπισμό.

**§2 — ΤΟ ΠΡΟΗΓΟΥΜΕΝΟ ΝΟΥΜΕΡΟ ΗΤΑΝ ΜΟΛΥΣΜΕΝΟ.** Νέα ευθεία βάση (4 legs):

```
0.9276 · 0.9300 · 0.9237 · 0.9280 (held-out)   ->  k_straight = 0.9276  (εύρος 0.006)
```

έναντι 0.834-0.870 της #80 (εύρος 0.036). Αιτίες της μόλυνσης: χωρίς reset, το
ρομπότ πλησίαζε/ακουμπούσε το δίχτυ, και το παράθυρο ολοκλήρωσης των τροχών δεν
ταίριαζε με το παράθυρο των δειγμάτων truth. **Ισχύει το 0.9276: υπερεκτίμηση
7.2%, όχι 14-16%.**

**Συνέπεια:** 2 m x 7.2% = **0.145 m** — εξηγεί μεγάλο μέρος, **όχι όλη**, την
παράσυρση 0.18-0.26 m της #79.

**§3/§4 — Διαμήκης μετατόπιση στο σώμα (αυτό που ολοκληρώνει η οδομετρία):**

| leg | κ | lon m | lat m | wheel m | ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| ευθείες (4) | 0 | — | 0.000 | — | **0.9237-0.9300** |
| στροφές (14) | 0.25-1.80 | — | 0.15-0.68 | — | 0.872-1.000 |

Η **πλευρική** ολίσθηση αυξάνεται μονότονα με την καμπυλότητα (7.5% → 32% της
διαμήκους) — η φυσική είναι υπαρκτή — αλλά **δεν** μεταφράζεται σε πρόσθετο
διαμήκες σφάλμα κλίμακας.

**§5/§6 — Υπόλοιπο μετά τον σταθερό συντελεστή:** διάμεσο **+0.59%**, εύρος
−6.4%..+7.3%, **correlation(|κ|, residual) = −0.23** (ασθενής και **αρνητική** —
το αντίθετο από «η ολίσθηση μεγαλώνει με την καμπυλότητα»).

**§10 — HELD-OUT, το πιο πειστικό στοιχείο:**

```
holdout_straight_v0.28   residual +0.05%
holdout_turn_k0.75       residual +0.14%
holdout_turn_k1.25       residual +0.04%
```

Τρεις τροχιές που δεν συμμετείχαν στην εκτίμηση: **σχεδόν τέλεια** πρόβλεψη.

**§7 — Αριστερά/δεξιά:** διαφορές 0.0023 / 0.0036 / 0.0207 / 0.0371 / 0.1224 στα
|κ| = 1.50 / 0.50 / 0.25 / 1.00 / 1.80. **Ασυνεπείς** — αν υπήρχε δομική
ασυμμετρία θα μεγάλωνε με το |κ|· εδώ το μικρότερο (0.0023) είναι στο κ=1.50.
Υπογραφή διασποράς μεταξύ εκτελέσεων, όχι ασυμμετρίας. (Ένα δείγμα ανά συνθήκη.)

**§11 — Ταχύτητα στο κ=1.00:** ratio 0.9596 (v=0.20), 0.9011/0.9403 (v=0.35),
0.9384 (v=0.50). Καμία μονότονη τάση· εντός της ίδιας διασποράς.

**§9 — ΤΟ connector-0 ΔΕΝ ΑΝΑΠΑΡΗΧΘΗ.** Μοναδικό leg με `reset_ok=False`: η
διαδρομή του περιλαμβάνει το ίδιο το teleport (βήμα 4.963 m) και δίνει ratio
1.542. **Άκυρο.**

**ΑΠΟΦΑΣΗ: `MODEL_1_CONSTANT_SCALE_SUFFICIENT`** — ένας σταθερός συντελεστής
0.9276 εξηγεί ευθείες **και** στροφές, και γενικεύει σε held-out τροχιές.
`MODEL_2` δεν δικαιολογείται (correlation −0.23). `ASYMMETRIC` δεν στηρίζεται.

**ΟΜΩΣ: ΔΕΝ επαρκεί για τη Φάση 18B** — το κριτήριο αποδοχής #3 (ανεξάρτητη
αναπαραγωγή του connector-0) **απέτυχε**, και κάθε συνθήκη στροφής έχει **ένα
μόνο δείγμα** με διασπορά ±6%.

**Status:** χαρακτηρισμός. Καμία αλλαγή σε wheel radius, odometry, EKF, SLAM,
controller, planner, gates.

---

## #82 — Φάση 18A3: με επαναλήψεις, το μοντέλο **ΔΕΝ ΕΙΝΑΙ ΣΤΑΘΕΡΟ**

**§1 — Το harness διορθώθηκε.** Πύλη καθίζησης πριν από κάθε leg: το μοντέλο
πρέπει να είναι **τοποθετημένο** (truth εντός 0.05 m), **ακίνητο** (μετατόπιση
truth < 2 mm ανά δείγμα), οι **τέσσερις τροχοί ήσυχοι** (|ω| < 0.05 rad/s) και να
παραμείνουν έτσι **0.6 s**· μόνο τότε καταγράφονται baselines και ξεκινά η
μέτρηση. Ασυνέχεια > 0.25 m μέσα στο παράθυρο **απορρίπτει** αυτόματα το leg.
**51 legs, 0 απορρίψεις**, και τα τρία connector replays πλέον **έγκυρα**.

**§2 — connector-0 x3 (ΕΓΚΥΡΑ):**

| rep | truth path | wheel | ratio | resid @k=0.9276 | odom vs truth |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 3.609 | 4.130 | 0.8739 | −6.15% | +0.114 m raw → **−0.129 m** διορθωμένο |
| 2 | 3.527 | 4.130 | 0.8540 | −8.62% | +0.093 → **−0.145 m** |
| 3 | 3.576 | 4.130 | 0.8658 | −7.14% | +0.118 → **−0.124 m** |

**Ο σταθερός συντελεστής υπερδιορθώνει τον connector**: το σφάλμα αλλάζει πρόσημο
(+0.11 → −0.13 m) αντί να μηδενιστεί.

**§3 — ΤΟ ΚΡΙΣΙΜΟ: Η ΕΥΘΕΙΑ ΒΑΣΗ ΔΕΝ ΑΝΑΠΑΡΑΓΕΤΑΙ ΜΕΤΑΞΥ ΕΚΤΕΛΕΣΕΩΝ.**

```
run A2 (1 rep):  0.9237 0.9276 0.9300   mean 0.9271  sd 0.0032
run B  (3 reps): 0.8761 ... 0.9122      mean 0.9005  sd 0.0113
```

Διαφορά **0.027** μεταξύ εκτελέσεων — **8x** μεγαλύτερη από τη διασπορά εντός της
πρώτης εκτέλεσης. Ίδιος κώδικας, ίδια αφετηρία, ίδιο harness. **Το κριτήριο
αποδοχής #1 της §8 αποτυγχάνει.**

**§4/§5 — ΚΑΙ ΕΜΦΑΝΙΖΕΤΑΙ ΤΑΣΗ ΚΑΜΠΥΛΟΤΗΤΑΣ.** Πάνω στους μέσους όρους
συνθηκών (και με τη **δική** της εκτέλεσης βάση 0.9057):

```
correlation(|κ|, residual) = -0.878     slope -3.89 %/(1/m)
εύρος μεταξύ συνθηκών 6.80%   vs   μέση διασπορά εντός συνθήκης 2.25%
```

Η τάση είναι **3x** μεγαλύτερη από τον θόρυβο επανάληψης: −0.5% στο κ=0.25-0.50,
−5% έως −6.6% στο κ=1.0-1.8. **Το κριτήριο #2 αποτυγχάνει.** Στην εκτέλεση A2 η
ίδια συσχέτιση ήταν **−0.23** (ανύπαρκτη). Οι δύο εκτελέσεις λένε διαφορετικά
πράγματα.

**§6 — Αριστερά/δεξιά:** διαφορές 0.64-1.41%, **εντός** της διασποράς εντός
συνθήκης (0.34-7.72%). Συμμετρικό — κανένα signed μοντέλο δεν δικαιολογείται.

**§7 — Συνέπεια connector:** residual −6.2..−8.6% έναντι −5.4..−6.6% των
ελεγχόμενων στροφών κ=1.0-1.8. **CONSISTENT** — ο connector λέει την ίδια
ιστορία με τις στροφές αντίστοιχης καμπυλότητας. Δεν υπάρχει ξεχωριστό
transition φαινόμενο.

**§8 — Held-out με σταθερό k=0.9276:** −6.24% (ευθεία), −9.85% (κ=0.75),
−15.44% (κ=1.25). **Κριτήριο #5 αποτυγχάνει.**

**Επαναληψιμότητα εντός συνθήκης:** sd 0.34-7.72%, χειρότερη στις υψηλές
καμπυλότητες (7.72% στο κ=1.80_L, 5.36% στο κ=1.00_R). Τρεις επαναλήψεις **δεν
αρκούν** σε αυτό το επίπεδο θορύβου.

**ΑΠΟΦΑΣΗ: `MODEL_NOT_STABLE`.** Δεν είναι ότι χρειάζεται πιο σύνθετο μοντέλο —
είναι ότι **η ίδια η μέτρηση δεν επαναλαμβάνεται**: η ευθεία βάση μετακινείται
0.027 μεταξύ εκτελέσεων, και η τάση καμπυλότητας εμφανίζεται στη μία εκτέλεση
(−0.878) και όχι στην άλλη (−0.23). Οποιοσδήποτε συντελεστής προσαρμοστεί τώρα
θα κωδικοποιήσει τον θόρυβο μιας εκτέλεσης.

**Status:** χαρακτηρισμός. Καμία αλλαγή σε wheel radius, diff-drive, EKF, SLAM,
controller, planner, gates. Το `k = 0.9276` **ΔΕΝ** συνιστάται πλέον.

---

## #83 — Φάση 19A (σχεδιαστική): το abort **ΕΙΝΑΙ** το σωστό συμβόλαιο — η ανάκτηση καλύπτεται ήδη

**§1-§3 — Γεωμετρία μετά το άλμα, και τα 13 συμβάντα πάνω από το tube:**

| πληθυσμός | n | lateral | απόσταση ως το επόμενο crossing | έκβαση |
| --- | ---: | --- | --- | --- |
| **connector** | 6 | 0.202-0.284 m | **1.5-6.8 m** | 3/3 «συνέχισαν» με τον κώδικα Φάσης 16 |
| **capture pass** | 7 | **0.302-0.366 m** | **0.34-0.84 m** | **7/7 abort** |

**Καθαρός διαχωρισμός.** Και με τον σημερινό κώδικα (Φάση 16 + 17A) **κάθε** abort
είναι περίπτωση pass· **καμία** περίπτωση connector δεν κόβει πια.

**§3/§11 — Γεωμετρική εφικτότητα εμπρόσθιας ανάκτησης** (κλείσιμο της πλευρικής
απόκλισης ΚΑΙ επιστροφή κάτω από την πύλη 0.15 rad **πάνω στη μπάλα**):

```
pass:      χρειάζεται 21-42 deg μέση απόκλιση και 1.8-10.7 1/m αιχμή καμπυλότητας
           μέσα σε 0.34-0.84 m  ->  ΑΝΕΦΙΚΤΟ (όριο ελεγκτή 2.5 1/m)
connector: χρειάζεται 2.4-7.7 deg και 0.02-0.36 1/m σε 1.5-6.8 m, χωρίς μπάλα
           μπροστά  ->  εφικτό, και ΗΔΗ γίνεται
```

**§5 — Οι υποψήφιες συμπεριφορές (A stop-and-rotate, B μειωμένη ταχύτητα,
C σημείο επανεισόδου, D abort) δεν αξιολογούνται περαιτέρω**: δεν υπάρχει
περίπτωση στα δεδομένα όπου θα βοηθούσαν. Ο πληθυσμός που ΜΠΟΡΕΙ να ανακτηθεί
ανακτάται ήδη με τη χάρη ενός update· ο πληθυσμός που κόβει **δεν μπορεί** να
ανακτηθεί εμπρόσθια πριν τη μπάλα.

**§7 — Προστασία crossing:** αυτό ακριβώς είναι που καθιστά την ανάκτηση αδύνατη
στα passes. Το να διασχίσει μπάλα με 21-42 deg σφάλμα κατεύθυνσης θα παραβίαζε
την capture-grade γεωμετρία· το να «ανακτήσει» παραιτούμενο από την πύλη θα ήταν
ακριβώς η σιωπηλή παραίτηση που απαγορεύει η §7.

**§15 — Εναλλακτική «πυκνότερος εντοπισμός»:** οι **συνηθισμένες** διορθώσεις
είναι ήδη μικρές και αναλογικές της παράσυρσης — median **0.030 m** έναντι
αναμενόμενης παράσυρσης 0.042 m στα 0.44 m διαδρομής (λόγος **0.7x**). Οι
επιβλαβείς είναι το **2.7%** (28/1050) που συγκεντρώνονται στα **0.212 m** —
δηλαδή στην κλίμακα του `correlation_search_space_dimension` (0.5 m = ±0.25),
**όχι** της παράσυρσης. Μείωση του `minimum_travel_distance` θα σμίκρυνε τις
συνηθισμένες (που ήδη δεν ενοχλούν) χωρίς απόδειξη ότι αφαιρεί τις ακραίες.

**§16 — Υλικό:** στο χώμα η ολίσθηση θα είναι χειρότερη και οι διορθώσεις
συχνότερες, αλλά η **γεωμετρία** δεν αλλάζει: μια απόκλιση 0.3 m με μπάλα 0.5 m
μπροστά παραμένει μη ανακτήσιμη εμπρόσθια. Το συμπέρασμα μεταφέρεται.

**ΑΠΟΦΑΣΗ: `ABORT_IS_CORRECT`.** Δεν προτείνεται κατάσταση
`RELOCALIZATION_RECOVERY`. Η μία αλλαγή που θα είχε αξία δεν είναι ελεγκτής αλλά
**σχέδιο**: να παραιτείται από τη συγκεκριμένη μπάλα και να συνεχίζει η διαδρομή
αντί να τερματίζει ολόκληρη — αλλά αυτό είναι σημασιολογία διαδρομής, εκτός
σκοπού και **δεν** υλοποιείται.

**Status:** σχεδιαστικό μόνο. Καμία αλλαγή σε ελεγκτή, SLAM, planner, collector.

---

## #84 — Φάση 20A (σχεδιαστική): το abort **ακυρώνει το 42%** της διαδρομής — και ο μηχανισμός συνέχισης **υπάρχει ήδη, απλώς είναι κλειδωμένος**

**§1 — Η διαδρομή του abort.** `_terminal_completion_state()`: `ABORTED_TRACKING`
περνά αυτούσιο σε τερματική κατάσταση αποστολής. Και:

```python
def _can_follow_up(self):
    return (self.route_outcome is ExecutorState.ROUTE_COMPLETED   # <-- η πύλη
            and policy.enabled and self.run_count < policy.max_total_runs)
```

**Μετά από ΟΠΟΙΟΔΗΠΟΤΕ abort δεν γίνεται follow-up scan, δεύτερο run, τίποτα.**
Η αποστολή τελειώνει. Επιβεβαιώνεται στα δεδομένα: **15/15** abort runs έχουν
`run_history` με ένα μόνο run.

**§8/§14 — Το κόστος, μετρημένο:**

| | |
| --- | --- |
| aborted routes | **15** |
| planned crossings σε αυτές | 106 |
| crossings που **δεν προσεγγίστηκαν ποτέ** | **44 (42%)** |

Η αξία είναι **συγκεντρωμένη**: τα `two_passes` κόβουν στο 93-96% της διαδρομής
(1 crossing χαμένο το καθένα), αλλά τα `real_scan` κόβουν στο **20-43%** και
χάνουν **5-8** crossings· 37 από τα 44 χαμένα crossings είναι εκεί.

**§10 — ΚΑΙ Ο ΜΗΧΑΝΙΣΜΟΣ ΥΠΑΡΧΕΙ ΗΔΗ.** Το follow-up κάνει ακριβώς αυτό που
ζητά η §5/§8B: *navigate to scan pose → 360 scan → replan από την ΤΡΕΧΟΥΣΑ πόζα
πάνω στις υπόλοιπες μπάλες → execute*, με **την πλήρη απόδειξη του planner**
(clearance, ακτίνα στροφής, run-in, forward-only, terminal). Μετρημένο overhead
μετάβασης: **1.6-5.6 s**. Δουλεύει: 5 runs το εκτέλεσαν.

**§3 — Η σωστή μονάδα εγκατάλειψης.** ΟΧΙ μία μπάλα: στα 7 μη ανακτήσιμα
συμβάντα (#83) το ρομπότ είναι 0.30-0.37 m εκτός με τη μπάλα 0.34-0.84 m
μπροστά· η **είσοδος του pass** έχει ήδη χαθεί, οπότε το να παραλειφθεί μόνο το
πρώτο crossing αφήνει το ρομπότ στο ίδιο μη εκτελέσιμο pass. Μονάδα = **το
υπόλοιπο του τρέχοντος pass**, και η επιλογή του τι ακολουθεί ανήκει στον
planner του follow-up, όχι στον executor.

**§6/§8 — Τοπική συνέχιση (A) vs replan υπολοίπου (B).** Η (A) απαιτεί νέα
γεννήτρια connector μέσα στον executor, επιλογή σημείου επανεισόδου, νέα
απόδειξη ασφάλειας και νέα σημασιολογία καταστάσεων — και η §5 ρητά προειδοποιεί
να μην φτιαχτεί δεύτερη ad-hoc γεννήτρια. Η (B) **υπάρχει, δοκιμασμένη, με
προϋπολογισμό** (`follow_up.max_total_runs`).

**§2 — Ταξινόμηση αστοχιών (τι θα επιτρεπόταν να συνεχίσει):**
*επιτρεπτό*: `trajectory_tube_exceeded` / `heading_error_exceeded` μετά από
θετικά αναγνωρισμένο re-anchoring, με έγκυρη πόζα και υγιή frames.
*ΟΧΙ*: safety/keepout, collector fault, απώλεια εντοπισμού, αδυναμία Nav2,
`profile_unenforceable`.

**ΑΠΟΦΑΣΗ: `REPLAN_REMAINING_JUSTIFIED`** — αλλά **χωρίς νέα μηχανή**: η
μικρότερη αλλαγή είναι να **χαλαρώσει η πύλη** `_can_follow_up()` ώστε να δέχεται
και ένα *ταξινομημένα επιτρεπτό* tracking abort, με τον υπάρχοντα προϋπολογισμό
runs αμετάβλητο. Καμία ανοχή, κανένα gate, καμία γεωμετρία δεν αλλάζει.

**Status:** σχεδιαστικό. Καμία υλοποίηση.

---

## #85 — Φάση 20B: το follow-up ξεκλειδώνεται για **ταξινομημένα** tracking aborts

**§1 — Η αλλαγή (μία πύλη, μία συνάρτηση):**

```python
_SKIPPABLE_TRACKING_DETAILS = ("trajectory_tube_exceeded", "heading_error_exceeded")

def _is_skippable_tracking_abort(outcome, reason, detail):
    if outcome is not ExecutorState.ABORTED_TRACKING: return False
    if reason is not ExecutorReasonCode.PATH_FAILED:  return False   # SAFETY_RESUME_INVALID
    return any(label in (detail or "") for label in _SKIPPABLE_TRACKING_DETAILS)

def _can_follow_up(self):
    policy = self.plan.configuration_snapshot.follow_up
    if not policy.enabled or self.run_count >= policy.max_total_runs: return False
    if self.route_outcome is ExecutorState.ROUTE_COMPLETED: return True
    return _is_skippable_tracking_abort(self.route_outcome, self.terminal_reason,
                                        self.terminal_detail)
```

Καμία άλλη αλλαγή: ούτε connector στον executor, ούτε άλμα δεικτών, ούτε
κατάσταση ανάκτησης, ούτε planner, ούτε ανοχές. Ο προϋπολογισμός
`max_total_runs` μένει **ο ίδιος** και παραμένει η μόνη προστασία από κύκλο
abort→rescan→abort.

**§3 — Παραμένουν τερματικά:** safety/keepout, collector fault, scan/navigation
failure, planning failure, `SAFETY_RESUME_INVALID` (καταλήγει κι αυτό σε
ABORTED_TRACKING — γι' αυτό ελέγχεται ο **reason code**, όχι μόνο η κατάσταση),
και κάθε μη αναγνωρισμένη ετικέτα (π.χ. `curvature_exceeded`,
`non_monotonic_progress`).

**§4/§5 — 15 νέα tests** (4 ταξινόμησης + 5 session-level integration + 6
παραμετρικά), 622 pytest περνούν. Καλύπτονται: tube abort → follow-up· heading
abort → follow-up· safety-resume → τερματικό· άγνωστη ετικέτα → τερματικό·
budget εξαντλημένο → τερματικό με 2 runs.

**§6 — Replay των 15 καταγεγραμμένων aborts:** **11 γίνονται δεκτά**, 4 μένουν
τερματικά (1 με `non_monotonic_progress` = μη ταξινομημένο, 3 με εξαντλημένο
budget). **38 crossings ξαναγίνονται προσβάσιμα** από τα 44 που χάνονταν.

**§7 — Ζωντανά, 2 έγκυρα runs:**

```
two_passes r1: run 1 aborted_tracking (2/4 confirmed)  ->  ΣΥΝΕΧΙΣΕ
               run 2 νέο scan + νέο plan  ->  completed_no_targets
               δύο ξεχωριστά audit artifacts (..._run-2)
two_passes r2: run 1 route_completed, run 2 abort, budget 2/2  ->  τερματικό
```

Το r1 είναι **ακριβώς** η επιδιωκόμενη συμπεριφορά: το abort δεν τερματίζει πια
την αποστολή.

**ΑΚΥΡΑ: 4 runs (two_passes r3, real_scan r1-r3) — «INVALID nav2 boot».** Ο
`planner_server` δεν σηκώθηκε ποτέ (`Waiting for service planner_server/get_state`
επ' άπειρον), μνήμη 76 GB ελεύθερη. Υποβάθμιση περιβάλλοντος μετά από πολλές
ώρες κυκλικών εκκινήσεων — **άσχετο με την αλλαγή**, που ζει κατάντη του Nav2.
**Η επικύρωση real_scan (§12) ΔΕΝ παραδόθηκε.**

**Status:** υλοποιημένο και κλειδωμένο με tests· η ζωντανή επικύρωση είναι
**μερική**.

---

## #86 — Φάση 21: ζωντανή επικύρωση 6/6 — το follow-up **ανακτά** χαμένη κάλυψη

**Η αιτία των αποτυχιών boot του #85 βρέθηκε και ήταν περιβαλλοντική:** **355**
ορφανά segments `/dev/shm/fastrtps_*` από δεκάδες κύκλους στοίβας. Καθαρισμός
(μόνο δικά μας) πριν από κάθε προσπάθεια → **6/6 έγκυρα με την πρώτη**, καμία
άκυρη. Υγεία: residual 0, `/clock` publishers **1**, time jumps **0**, σε όλα.

**Οι έξι αποστολές:**

| mission | planned | confirmed | retained | runs | τελική κατάσταση |
| --- | ---: | ---: | ---: | ---: | --- |
| two_passes r1 | 4 | 2 | 2 | 1(+scan) | `completed_no_targets` |
| two_passes r2 | 4 | 2 | 2 | 1(+scan) | `completed_no_targets` |
| two_passes r3 | 4 | 1 | 1 | 1(+scan) | `completed_no_targets` |
| real_scan r1 | 16 | **3** | 4 | 2 | aborted_tracking |
| real_scan r2 | 11 | 0 | 0 | 2 | `completed` |
| real_scan r3 | 11 | 6 | 6 | 2 | aborted_tracking |
| **ΣΥΝΟΛΟ** | **50** | **14** | **15** | | |

**Η ΑΠΟΔΕΙΞΗ ΑΞΙΑΣ — real_scan r1:**

```
run 1  plan 65f913dd  9 planned  ->  aborted_tracking (heading_error_exceeded)  1 confirmed
       ΠΑΛΙΑ ΣΥΜΠΕΡΙΦΟΡΑ: τέλος αποστολής εδώ, 8 μπάλες πεταμένες
run 2  plan dd4eee96  7 planned (οι ανεπίλυτες)  ->  +2 confirmed
τελικό: 3 confirmed, 4 retained
```

**Τριπλασιασμός** της συλλογής εκείνης της αποστολής χάρη στο follow-up.

**Και στα 3 two_passes**: `aborted_tracking (trajectory_tube_exceeded)` →
`navigating_to_scan_pose` → `scanning` → `planning` → **`completed_no_targets`**.
Με την παλιά πύλη θα τερμάτιζαν ως `aborted_tracking`. Το rescan δεν βρήκε άλλες
μπάλες, οπότε η ανακτημένη κάλυψη εκεί είναι 0 — αλλά η αποστολή **κλείνει
καθαρά** αντί να πέφτει.

**§10 — Η ταξινόμηση κρατά:** στο real_scan r3 το run 2 έκοψε με
`non_monotonic_progress` — **ΔΕΝ** είναι στο εγκεκριμένο σύνολο, **κανένα** τρίτο
run δεν ξεκίνησε. Σωστά τερματικό.

**§5 — Το σκληρό invariant ισχύει:** σε **και τις τρεις** πολυ-run αποστολές,
**0** στόχοι του run 2 βρίσκονται κοντά (<0.25 m) σε επιβεβαιωμένη συλλογή του
run 1. Καμία μπάλα δεν ξανασχεδιάστηκε.

**§9 — Προϋπολογισμός:** και οι 3 real_scan έφτασαν τα 2 runs και σταμάτησαν.
Κανένας κύκλος.

**§11 — Ακεραιότητα:** κάθε run έχει **δικό του** plan_id, audit και trace
(επαληθεύτηκε: 6/6 «audit=yes trace=yes» σε κάθε καταχώρηση run_history). Η
αποστολή ανακατασκευάζεται ως run 1 → abort, run 2 → follow-up.

**Status:** Η Φάση 20B επικυρώθηκε ζωντανά. Η αποτελεσματικότητα συλλογής
παραμένει χαμηλή (14/50 confirmed) — αυτό είναι η **ανοιχτή γραμμή
μηχανισμού/perception** (#71: 77% κατάποση όταν η μπάλα παρουσιάζεται), όχι
ελάττωμα εκτέλεσης.

---

## #87 — ΠΑΓΩΜΑ: `COLLECTION_SIMULATION_FROZEN_FOR_HARDWARE_HANDOFF`

Το τεκμήριο παγώματος είναι το
[`docs/validation/collection-simulation-freeze.md`](../validation/collection-simulation-freeze.md).
Περιέχει: τι πάγωσε (αρχιτεκτονική vs production default vs sim-only calibration
vs ανοιχτό για υλικό vs γνωστός περιορισμός), το canonical runtime και το
preflight, τις παγωμένες παραμέτρους **με τη σημασία τους**, το συμβόλαιο
πλαισίων, τα συμβόλαια ανθεκτικότητας, τη σημασιολογία follow-up, τη βάση
αναφοράς της Φάσης 21, τα ανοιχτά θέματα, τις παραδοχές μόνο-για-προσομοίωση, τη
σειρά H1-H8, τις πύλες αποδοχής υλικού, το διατηρούμενο regression set, το
Hardware Handoff Matrix και τους κανόνες επανανοίγματος.

**Επαληθεύτηκε από τον κώδικα, όχι από τις αναφορές φάσεων:**
* production diff = **δύο τιμές** (`required_entry_m 0.2→0.8`,
  `max_lateral_error_m 0.1→0.2`) συν τις εγκεκριμένες ρυθμίσεις αναζήτησης του
  planner από προηγούμενες φάσεις·
* **κανένα** ίχνος `COLLECTION_DIAGNOSTIC_TUBE_M` και **κανένας** συντελεστής
  βαθμονόμησης από Gazebo (0.9276 / 0.0788 / effective radius) στο `ros2_ws/`·
* `controllers.yaml` και URDF **αμετάβλητα** (`wheel_radius 0.085` και στα δύο)·
* η λίστα επιλεξιμότητας follow-up διαβάστηκε από το
  `_is_skippable_tracking_abort`, όχι από τις αναφορές.

**Tests στο πάγωμα:** Python **622 passed / 3 skipped**· C++ **68 passed**
(48 tracking core + 8 runtime + 5 goal checker + 7 canonicalization) / 2 skipped.

**Το 14/50 της Φάσης 21 ΔΕΝ είναι μέτρο του μηχανισμού.** Ο παρονομαστής
αναμειγνύει planned / reached / presented / confirmed / retained — πέντε
διαφορετικές ευθύνες υποσυστημάτων. Το τεκμήριο τις κρατά χωριστές.

**Status:** ΠΑΓΩΜΕΝΟ. Επόμενο βήμα: H1 (κίνηση βάσης και ασφάλεια) στο υλικό.
