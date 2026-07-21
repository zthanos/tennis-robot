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
