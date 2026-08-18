#ifndef TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_TRACKING_CORE_HPP_
#define TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_TRACKING_CORE_HPP_

#include <optional>
#include <string>
#include <vector>

namespace tennis_robot_collection_controller
{

struct TrackingPoint
{
  double x_m{};
  double y_m{};
  double progress_s{};
  double heading_rad{};
};

struct TrackingExecutionProfile
{
  double nominal_speed_mps{};
  double min_speed_mps{};
  double max_speed_mps{};
  double nominal_speed_warning_tolerance_mps{};
  double required_entry_m{};
  double required_run_in_m{};
  double required_run_out_m{};
  double max_curvature_per_m{};
  double max_lateral_error_m{};
  double max_heading_error_rad{};
  bool allow_reversing{};
  bool allow_standalone_rotate{};
};

struct CollectionControllerTuning
{
  double lookahead_distance_m{};
  double max_angular_velocity_rad_s{};
  double progress_projection_window_m{};
  double crossing_speed_window_m{};
  double progress_tolerance_m{};
};

struct TrackingPlannedCrossing
{
  std::string ball_id;
  double progress_s{};
};

struct TrackingSegment
{
  std::string segment_id;
  double progress_start_s{};
  double progress_end_s{};
  TrackingExecutionProfile profile;
  std::vector<TrackingPlannedCrossing> planned_crossings;
};

struct CollectionTrackingPlan
{
  std::vector<TrackingPoint> path;
  std::vector<TrackingSegment> segments;
  double terminal_progress_s{};
  CollectionControllerTuning tuning;
};

enum class TrackingFailureCode
{
  kNone,
  kProfileUnenforceable,
  kSpeedBelowMin,
  kSpeedAboveMax,
  kRunInInsufficient,
  kRunOutInsufficient,
  kCurvatureExceeded,
  kTrajectoryTubeExceeded,
  kNonMonotonicProgress,
  kHeadingErrorExceeded,
  kReverseRequired,
  kStandaloneRotateRequired,
};

enum class NominalTracking
{
  kNotMeasured,
  kWithinTolerance,
  kDeviated,
};

struct ProfileComplianceVerdict
{
  bool hard_compliant{true};
  TrackingFailureCode hard_violation_reason{TrackingFailureCode::kNone};
  NominalTracking nominal_tracking{NominalTracking::kNotMeasured};
  double measured_speed_mps{};
  double nominal_speed_error_mps{};
};

struct CrossingMeasurement
{
  std::string ball_id;
  double progress_s{};
  ProfileComplianceVerdict verdict;
};

struct TrackingInput
{
  double x_m{};
  double y_m{};
  double heading_rad{};
  double measured_speed_mps{};
  bool safety_hold{};
  /// Seconds since the previous update.  Zero means "unknown", and without it
  /// no re-anchoring judgement is made: a pose step can only be called
  /// physically impossible if the time it happened in is known.
  double elapsed_s{};
};

enum class TrackingStatus
{
  kRunning,
  kSafetyHold,
  kCompleted,
  kFailed,
};

struct ForwardVelocityCommand
{
  double linear_x_mps{};
  double angular_z_rad_s{};
};

struct TrackingResult
{
  TrackingStatus status{TrackingStatus::kFailed};
  ForwardVelocityCommand command;
  double progress_s{};
  double lateral_error_m{};
  double heading_error_rad{};
  double remaining_run_in_m{};
  double remaining_run_out_m{};
  bool terminal_ready{false};
  // Diagnosis for terminal_not_reached: the Euclidean distance to the terminal
  // point, the half of the terminal condition the progress figure cannot show.
  double terminal_distance_m{-1.0};
  std::optional<CrossingMeasurement> crossing_measurement;
  TrackingFailureCode failure{TrackingFailureCode::kNone};
  // Frame diagnosis (Phase 11): the path point this update actually measured
  // `lateral_error_m` against.  The core is frame-agnostic by construction --
  // it compares two bare coordinate pairs and trusts the caller to supply them
  // in one frame -- so the only way to check that contract from outside is to
  // publish the second object alongside the first.  Diagnostic only: no
  // control decision reads these.
  double reference_x_m{};
  double reference_y_m{};
  double reference_heading_rad{};
  bool has_reference{false};
  // Pure-pursuit geometry of this update (Phase 13 diagnosis).  Published so
  // corner cutting can be related to the lookahead point and the curvature the
  // controller actually commanded, rather than assumed from the algorithm name.
  double lookahead_x_m{};
  double lookahead_y_m{};
  double lookahead_distance_m{};
  double commanded_curvature_per_m{};
  bool has_pursuit_geometry{false};
  // Failure evidence (Phase 15).  A failing update used to return a default
  // result, so `lateral_error_m` and `heading_error_rad` read 0.000 for the one
  // update that actually tripped a gate -- the sample most worth seeing
  // (debug log #75).  These now carry what was really computed; `has_geometry`
  // says whether they are measurements or simply were never computed, so an
  // absent value is never mistaken for a zero one.
  bool has_geometry{false};
  double previous_progress_s{};
  // Unbounded nearest-point projection, the quantity the non-monotonic check
  // actually compares against `previous_progress_s`.
  double raw_projection_progress_s{};
  bool has_raw_projection{false};
  // Re-anchoring detection (Phase 16).  A discontinuous revision of the
  // estimated map pose is not the robot leaving the corridor, but the tracker
  // sees both as a step in cross-track.  These report the kinematic test that
  // separates them; the tube verdict is deferred for exactly one update when it
  // fires (debug log #76).
  double pose_step_m{};
  double pose_step_yaw_rad{};
  double elapsed_s{};
  double plausible_step_bound_m{};
  bool reanchoring_detected{false};
  bool tube_verdict_deferred{false};
  // Progress-projection continuity (Phase 17A).  The regression check compares
  // the *raw* nearest point on the whole route against the accepted progress;
  // on a route that returns near itself the global nearest can be metres behind
  // without the robot having moved (debug log #77).  This is the kinematic
  // neighbourhood the raw candidate had to lie in to count as evidence of
  // backward motion.
  double projection_reach_m{};
  bool raw_projection_constrained{false};
};

/// ROS-free, stateful controller core. The supplied plan is never mutated.
class CollectionTrackingCore
{
public:
  explicit CollectionTrackingCore(CollectionTrackingPlan plan);

  TrackingResult update(const TrackingInput & input);
  double last_progress_s() const;

private:
  struct Projection;

  Projection project_monotonically(double x_m, double y_m, double elapsed_s) const;
  const TrackingSegment * active_segment(double progress_s) const;
  TrackingPoint point_at(double progress_s) const;
  double tangent_at(double progress_s) const;
  std::optional<TrackingPlannedCrossing> next_crossing(double progress_s) const;
  TrackingResult failure(TrackingFailureCode code, double progress_s) const;

  CollectionTrackingPlan plan_;
  double last_progress_s_{};
  bool safety_held_{false};
  double last_input_x_m_{};
  double last_input_y_m_{};
  double last_input_heading_rad_{};
  bool has_last_input_{false};
  /// True when the previous update already had its tube verdict deferred.  One
  /// deferral cannot be followed by another, so a stream of discontinuities can
  /// never add up to an open-ended grace period.
  bool deferred_previous_update_{false};
  /// Fastest speed any segment of this plan permits: the upper bound on how far
  /// the robot can travel along the route between two updates.
  double plan_max_speed_mps_{};
};

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_TRACKING_CORE_HPP_
