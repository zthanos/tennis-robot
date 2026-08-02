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
  std::optional<CrossingMeasurement> crossing_measurement;
  TrackingFailureCode failure{TrackingFailureCode::kNone};
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

  Projection project_monotonically(double x_m, double y_m) const;
  const TrackingSegment * active_segment(double progress_s) const;
  TrackingPoint point_at(double progress_s) const;
  double tangent_at(double progress_s) const;
  std::optional<TrackingPlannedCrossing> next_crossing(double progress_s) const;
  TrackingResult failure(TrackingFailureCode code, double progress_s) const;

  CollectionTrackingPlan plan_;
  double last_progress_s_{};
  bool safety_held_{false};
};

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_TRACKING_CORE_HPP_
