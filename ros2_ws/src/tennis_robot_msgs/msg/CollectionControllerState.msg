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
