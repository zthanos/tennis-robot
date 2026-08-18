uint8 IDLE=0
uint8 CONTEXT_LOADED=1
uint8 VALIDATING_FOLLOW_PATH=2
uint8 EXECUTING=3
uint8 SAFETY_PAUSED=4
uint8 SUCCEEDED=5
uint8 FAILED=6
uint8 CONSUMED=7

uint8 FAILURE_NONE=0
uint8 FAILURE_MISSING_CONTEXT=1
uint8 FAILURE_PATH_HASH_MISMATCH=2
uint8 FAILURE_CONTEXT_ACTIVATION_TIMEOUT=3
uint8 FAILURE_PROFILE_UNENFORCEABLE=4
uint8 FAILURE_SPEED_BELOW_MIN=5
uint8 FAILURE_SPEED_ABOVE_MAX=6
uint8 FAILURE_RUN_IN_INSUFFICIENT=7
uint8 FAILURE_RUN_OUT_INSUFFICIENT=8
uint8 FAILURE_CURVATURE_EXCEEDED=9
uint8 FAILURE_TRAJECTORY_TUBE_EXCEEDED=10
uint8 FAILURE_NON_MONOTONIC_PROGRESS=11
uint8 FAILURE_REVERSE_REQUIRED=12
uint8 FAILURE_STANDALONE_ROTATE_REQUIRED=13
uint8 FAILURE_SAFETY_RESUME_INVALID=14
uint8 FAILURE_HEADING_ERROR_EXCEEDED=15
uint8 FAILURE_TERMINAL_NOT_REACHED=16

string plan_id
string path_sha256
uint8 lifecycle_state
float64 progress_s
string active_segment_id
bool has_active_crossing
string active_ball_id
float64 active_crossing_progress_s
float64 measured_speed_mps
float64 lateral_error_m
float64 heading_error_rad
tennis_robot_msgs/CollectionProfileComplianceVerdict profile_verdict
uint8 failure_reason
# Terminal diagnosis: finalize is rejected as terminal_not_reached when either
# half of the tracking core's terminal condition fails, and the rejection alone
# does not say which. Publish both so one run identifies it.
float64 terminal_progress_s
float64 terminal_distance_m
bool terminal_ready

# ── Frame diagnosis (Phase 11) — diagnostic only, nothing here feeds control ──
# The tracking core takes bare coordinate pairs and trusts the caller to supply
# pose and path in one frame; it cannot check that itself.  These fields publish
# the two geometric objects each reported lateral_error_m was actually computed
# from, together with the frame and stamp each arrived in, so the comparison can
# be reproduced offline instead of inferred from topic names.
string tracker_pose_frame_id      # header.frame_id of the pose Nav2 supplied
float64 tracker_pose_stamp_s      # header.stamp of that pose
float64 tracker_pose_x_m          # the x handed to the core, verbatim
float64 tracker_pose_y_m
float64 tracker_pose_yaw_rad
float64 tracker_update_stamp_s    # when the core computed this result
string tracker_path_frame_id      # header.frame_id of the path given to setPlan
float64 tracker_path_stamp_s
bool tracker_has_reference        # false when no projection was produced
float64 tracker_reference_x_m     # the path point lateral_error_m measured to
float64 tracker_reference_y_m
float64 tracker_reference_yaw_rad # path tangent there
float64 tracker_transform_stamp_s  # stamp of the transform used to reach the plan frame
float64 tracker_transform_age_s    # how old that transform was when it was used
bool tracker_transform_was_latest  # true when the pose-stamp lookup extrapolated
float64 commanded_linear_mps       # the command actually returned this update
float64 commanded_angular_rad_s
float64 tracker_lookahead_x_m          # pure-pursuit target point actually used
float64 tracker_lookahead_y_m
float64 tracker_lookahead_distance_m   # straight-line distance to it
float64 tracker_commanded_curvature_per_m
# Failure evidence (Phase 15).  lateral_error_m / heading_error_rad above are the
# values the core actually computed, including on the update that trips a gate;
# tracker_has_geometry distinguishes "measured" from "never computed" so an
# absent value is never read as a zero one.
bool tracker_has_geometry
float64 tracker_previous_progress_s        # progress the core held before this update
float64 tracker_raw_projection_progress_s  # unbounded nearest-point projection
bool tracker_has_raw_projection
# Re-anchoring detection (Phase 16).  A discontinuous revision of the estimated
# map pose looks identical to the robot leaving the corridor unless the step is
# checked against what the robot could physically have travelled.  When the step
# is impossible the tube verdict is deferred for exactly one update -- these
# fields make that explicit, so a log line showing lateral 0.31 m cannot be
# mistaken for either normal tracking or a real excursion.
float64 tracker_pose_step_m
float64 tracker_pose_step_yaw_rad
float64 tracker_elapsed_s
float64 tracker_plausible_step_bound_m
bool tracker_reanchoring_detected
bool tracker_tube_verdict_deferred
# Progress-projection continuity (Phase 17A).  The regression check consults the
# raw nearest point on the whole route; on a route that returns near itself that
# can be metres behind without the robot having moved.  These expose the
# reachable neighbourhood the raw candidate had to lie in, and whether a nearer
# but unreachable candidate was rejected -- so a constrained projection is
# visible rather than looking like an ordinary one.
float64 tracker_projection_reach_m
bool tracker_raw_projection_constrained
