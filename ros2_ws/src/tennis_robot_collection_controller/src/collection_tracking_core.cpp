#include "tennis_robot_collection_controller/collection_tracking_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace tennis_robot_collection_controller
{
namespace
{

constexpr double kEpsilon = 1e-9;

bool finite(const double value) { return std::isfinite(value); }

double wrap_angle(double angle)
{
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle <= -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

bool valid_profile(const TrackingExecutionProfile & profile)
{
  return finite(profile.nominal_speed_mps) && finite(profile.min_speed_mps) &&
    finite(profile.max_speed_mps) && finite(profile.nominal_speed_warning_tolerance_mps) &&
    finite(profile.required_entry_m) && finite(profile.required_run_in_m) &&
    finite(profile.required_run_out_m) &&
    finite(profile.max_curvature_per_m) && finite(profile.max_lateral_error_m) &&
    finite(profile.max_heading_error_rad) && profile.min_speed_mps > 0.0 &&
    profile.nominal_speed_mps >= profile.min_speed_mps &&
    profile.nominal_speed_mps <= profile.max_speed_mps &&
    profile.nominal_speed_warning_tolerance_mps >= 0.0 &&
    profile.required_entry_m >= 0.0 && profile.required_run_in_m >= 0.0 &&
    profile.required_run_out_m >= 0.0 &&
    profile.max_curvature_per_m > 0.0 && profile.max_lateral_error_m >= 0.0 &&
    profile.max_heading_error_rad >= 0.0 && !profile.allow_reversing &&
    !profile.allow_standalone_rotate;
}

bool valid_tuning(const CollectionControllerTuning & tuning)
{
  return finite(tuning.lookahead_distance_m) && tuning.lookahead_distance_m > 0.0 &&
    finite(tuning.max_angular_velocity_rad_s) && tuning.max_angular_velocity_rad_s > 0.0 &&
    finite(tuning.progress_projection_window_m) && tuning.progress_projection_window_m > 0.0 &&
    finite(tuning.crossing_speed_window_m) && tuning.crossing_speed_window_m > 0.0 &&
    finite(tuning.progress_tolerance_m) && tuning.progress_tolerance_m > 0.0;
}

}  // namespace

struct CollectionTrackingCore::Projection
{
  double progress_s{};
  double lateral_error_m{};
  bool raw_projection_behind{};
  bool found{};
};

CollectionTrackingCore::CollectionTrackingCore(CollectionTrackingPlan plan)
: plan_(std::move(plan))
{
  if (!valid_tuning(plan_.tuning) || plan_.path.size() < 2U || plan_.segments.empty()) {
    throw std::invalid_argument("invalid collection tracking plan");
  }
  for (std::size_t index = 0; index < plan_.path.size(); ++index) {
    const auto & point = plan_.path[index];
    if (!finite(point.x_m) || !finite(point.y_m) || !finite(point.progress_s) ||
      !finite(point.heading_rad) ||
      (index > 0U && point.progress_s <= plan_.path[index - 1U].progress_s))
    {
      throw std::invalid_argument("invalid collection path");
    }
  }
  if (!finite(plan_.terminal_progress_s) ||
    plan_.terminal_progress_s < plan_.path.front().progress_s ||
    plan_.terminal_progress_s > plan_.path.back().progress_s)
  {
    throw std::invalid_argument("invalid terminal progress");
  }
  for (const auto & segment : plan_.segments) {
    if (segment.segment_id.empty() || !finite(segment.progress_start_s) ||
      !finite(segment.progress_end_s) || segment.progress_end_s <= segment.progress_start_s ||
      !valid_profile(segment.profile))
    {
      throw std::invalid_argument("invalid collection segment profile");
    }
    double previous_crossing = -std::numeric_limits<double>::infinity();
    for (const auto & crossing : segment.planned_crossings) {
      if (crossing.ball_id.empty() || !finite(crossing.progress_s) ||
        crossing.progress_s <= previous_crossing ||
        crossing.progress_s < segment.progress_start_s ||
        crossing.progress_s > segment.progress_end_s ||
        segment.progress_end_s - crossing.progress_s + kEpsilon < segment.profile.required_run_out_m)
      {
        throw std::invalid_argument("invalid crossing or required run-out");
      }
      previous_crossing = crossing.progress_s;
    }
  }
  last_progress_s_ = plan_.path.front().progress_s;
}

TrackingResult CollectionTrackingCore::update(const TrackingInput & input)
{
  if (!finite(input.x_m) || !finite(input.y_m) || !finite(input.heading_rad) ||
    !finite(input.measured_speed_mps))
  {
    return failure(TrackingFailureCode::kProfileUnenforceable, last_progress_s_);
  }
  if (input.safety_hold) {
    safety_held_ = true;
    TrackingResult result;
    result.status = TrackingStatus::kSafetyHold;
    result.progress_s = last_progress_s_;
    return result;
  }

  const auto projection = project_monotonically(input.x_m, input.y_m);
  if (projection.raw_projection_behind) {
    return failure(TrackingFailureCode::kNonMonotonicProgress, last_progress_s_);
  }
  if (!projection.found) {
    return failure(TrackingFailureCode::kProfileUnenforceable, last_progress_s_);
  }
  const auto * segment = active_segment(projection.progress_s);
  if (segment == nullptr) {
    return failure(TrackingFailureCode::kProfileUnenforceable, projection.progress_s);
  }
  const auto next = next_crossing(projection.progress_s);
  const double remaining_run_in = next ? std::max(0.0, next->progress_s - projection.progress_s) : 0.0;
  const double remaining_run_out = next ? std::max(0.0, segment->progress_end_s - next->progress_s) : 0.0;
  if (safety_held_ && next && remaining_run_in + kEpsilon < segment->profile.required_run_in_m) {
    return failure(TrackingFailureCode::kRunInInsufficient, projection.progress_s);
  }
  safety_held_ = false;
  if (projection.lateral_error_m > segment->profile.max_lateral_error_m + kEpsilon) {
    return failure(TrackingFailureCode::kTrajectoryTubeExceeded, projection.progress_s);
  }
  if (input.measured_speed_mps < -kEpsilon) {
    return failure(TrackingFailureCode::kReverseRequired, projection.progress_s);
  }
  const auto terminal = point_at(plan_.terminal_progress_s);
  const double terminal_distance_m =
    std::hypot(terminal.x_m - input.x_m, terminal.y_m - input.y_m);
  if (projection.progress_s + plan_.tuning.progress_tolerance_m + kEpsilon >=
    plan_.terminal_progress_s &&
    terminal_distance_m <= plan_.tuning.progress_tolerance_m + kEpsilon)
  {
    last_progress_s_ = plan_.terminal_progress_s;
    TrackingResult result;
    result.status = TrackingStatus::kCompleted;
    result.terminal_ready = true;
    result.terminal_distance_m = terminal_distance_m;
    result.progress_s = last_progress_s_;
    result.lateral_error_m = projection.lateral_error_m;
    result.remaining_run_in_m = remaining_run_in;
    result.remaining_run_out_m = remaining_run_out;
    return result;
  }

  const double path_heading_error = wrap_angle(tangent_at(projection.progress_s) - input.heading_rad);
  // A connector may hand a straight capture pass a small, transient heading
  // error even though both frozen paths are tangent-continuous.  Allow the
  // already-configured entry distance to perform that alignment; the strict
  // capture-grade gate resumes for the remainder of the run-in and is always
  // active before the first crossing.  Lateral tube, curvature, reverse and
  // standalone-rotation guards remain active throughout this entry interval.
  const bool capture_segment = !segment->planned_crossings.empty();
  const double heading_grace_end_s = capture_segment && next ?
    std::min(
      segment->progress_start_s + segment->profile.required_entry_m,
      next->progress_s - kEpsilon) :
    segment->progress_start_s;
  const bool inside_heading_entry_grace =
    capture_segment && projection.progress_s + kEpsilon < heading_grace_end_s;
  if (!inside_heading_entry_grace &&
    std::abs(path_heading_error) > segment->profile.max_heading_error_rad + kEpsilon)
  {
    auto result = failure(TrackingFailureCode::kHeadingErrorExceeded, projection.progress_s);
    result.lateral_error_m = projection.lateral_error_m;
    result.heading_error_rad = path_heading_error;
    result.remaining_run_in_m = remaining_run_in;
    result.remaining_run_out_m = remaining_run_out;
    return result;
  }

  const auto lookahead = point_at(std::min(
    plan_.terminal_progress_s, projection.progress_s + plan_.tuning.lookahead_distance_m));
  const double target_angle = std::atan2(lookahead.y_m - input.y_m, lookahead.x_m - input.x_m);
  const double lookahead_bearing_error = wrap_angle(target_angle - input.heading_rad);
  const double target_distance = std::hypot(lookahead.x_m - input.x_m, lookahead.y_m - input.y_m);
  if (target_distance <= kEpsilon) {
    return failure(TrackingFailureCode::kStandaloneRotateRequired, projection.progress_s);
  }
  if (std::cos(lookahead_bearing_error) <= kEpsilon) {
    return failure(TrackingFailureCode::kReverseRequired, projection.progress_s);
  }
  const double curvature = 2.0 * std::sin(lookahead_bearing_error) / target_distance;
  if (std::abs(curvature) > segment->profile.max_curvature_per_m + kEpsilon) {
    return failure(TrackingFailureCode::kCurvatureExceeded, projection.progress_s);
  }
  const double angular_velocity = segment->profile.nominal_speed_mps * curvature;
  if (std::abs(angular_velocity) > plan_.tuning.max_angular_velocity_rad_s + kEpsilon) {
    return failure(TrackingFailureCode::kProfileUnenforceable, projection.progress_s);
  }

  TrackingResult result;
  result.status = TrackingStatus::kRunning;
  result.terminal_distance_m = terminal_distance_m;
  result.command.linear_x_mps = segment->profile.nominal_speed_mps;
  result.command.angular_z_rad_s = angular_velocity;
  result.progress_s = projection.progress_s;
  result.lateral_error_m = projection.lateral_error_m;
  result.heading_error_rad = path_heading_error;
  result.remaining_run_in_m = remaining_run_in;
  result.remaining_run_out_m = remaining_run_out;
  if (next && std::abs(next->progress_s - projection.progress_s) <= plan_.tuning.crossing_speed_window_m + kEpsilon) {
    ProfileComplianceVerdict verdict;
    verdict.measured_speed_mps = input.measured_speed_mps;
    verdict.nominal_speed_error_mps = input.measured_speed_mps - segment->profile.nominal_speed_mps;
    if (input.measured_speed_mps < segment->profile.min_speed_mps) {
      verdict.hard_compliant = false;
      verdict.hard_violation_reason = TrackingFailureCode::kSpeedBelowMin;
      result.crossing_measurement = CrossingMeasurement{next->ball_id, next->progress_s, verdict};
      result.status = TrackingStatus::kFailed;
      result.failure = TrackingFailureCode::kSpeedBelowMin;
      return result;
    }
    if (input.measured_speed_mps > segment->profile.max_speed_mps) {
      verdict.hard_compliant = false;
      verdict.hard_violation_reason = TrackingFailureCode::kSpeedAboveMax;
      result.crossing_measurement = CrossingMeasurement{next->ball_id, next->progress_s, verdict};
      result.status = TrackingStatus::kFailed;
      result.failure = TrackingFailureCode::kSpeedAboveMax;
      return result;
    }
    verdict.nominal_tracking = std::abs(verdict.nominal_speed_error_mps) >
      segment->profile.nominal_speed_warning_tolerance_mps ? NominalTracking::kDeviated :
      NominalTracking::kWithinTolerance;
    result.crossing_measurement = CrossingMeasurement{next->ball_id, next->progress_s, verdict};
  }
  last_progress_s_ = projection.progress_s;
  return result;
}

double CollectionTrackingCore::last_progress_s() const { return last_progress_s_; }

CollectionTrackingCore::Projection CollectionTrackingCore::project_monotonically(
  const double x_m, const double y_m) const
{
  Projection raw;
  Projection bounded;
  double raw_distance = std::numeric_limits<double>::infinity();
  double bounded_distance = std::numeric_limits<double>::infinity();
  const double window_end = last_progress_s_ + plan_.tuning.progress_projection_window_m;
  for (std::size_t index = 1; index < plan_.path.size(); ++index) {
    const auto & start = plan_.path[index - 1U];
    const auto & end = plan_.path[index];
    const double dx = end.x_m - start.x_m;
    const double dy = end.y_m - start.y_m;
    const double length_squared = dx * dx + dy * dy;
    const double unclamped_t = ((x_m - start.x_m) * dx + (y_m - start.y_m) * dy) / length_squared;
    const double raw_t = std::clamp(unclamped_t, 0.0, 1.0);
    const double raw_x = start.x_m + raw_t * dx;
    const double raw_y = start.y_m + raw_t * dy;
    const double current_raw_distance = std::hypot(x_m - raw_x, y_m - raw_y);
    const double raw_progress = start.progress_s + raw_t * (end.progress_s - start.progress_s);
    // The raw projection drives the backward-motion (non-monotonic) check.  A
    // global nearest-point search is fooled by self-crossing loop routes: where
    // the path returns physically close to a much earlier segment, the global
    // nearest snaps back and falsely reports regression.  Restrict it to a
    // window behind the tracked progress so it detects genuine on-path
    // regression while ignoring distant self-intersections.
    const bool within_regression_window =
      raw_progress + plan_.tuning.progress_projection_window_m + kEpsilon >= last_progress_s_;
    if (within_regression_window && current_raw_distance < raw_distance) {
      raw_distance = current_raw_distance;
      raw = Projection{raw_progress, current_raw_distance, false, true};
    }

    const double low_s = std::max(start.progress_s, last_progress_s_);
    const double high_s = std::min(end.progress_s, window_end);
    if (low_s > high_s + kEpsilon) { continue; }
    const double low_t = (low_s - start.progress_s) / (end.progress_s - start.progress_s);
    const double high_t = (high_s - start.progress_s) / (end.progress_s - start.progress_s);
    const double bounded_t = std::clamp(unclamped_t, low_t, high_t);
    const double bounded_x = start.x_m + bounded_t * dx;
    const double bounded_y = start.y_m + bounded_t * dy;
    const double current_bounded_distance = std::hypot(x_m - bounded_x, y_m - bounded_y);
    if (current_bounded_distance < bounded_distance) {
      bounded_distance = current_bounded_distance;
      bounded = Projection{
        start.progress_s + bounded_t * (end.progress_s - start.progress_s),
        current_bounded_distance, false, true};
    }
  }
  bounded.raw_projection_behind = raw.found &&
    raw.progress_s + plan_.tuning.progress_tolerance_m + kEpsilon < last_progress_s_;
  return bounded;
}

const TrackingSegment * CollectionTrackingCore::active_segment(const double progress_s) const
{
  for (const auto & segment : plan_.segments) {
    if (progress_s + kEpsilon >= segment.progress_start_s &&
      progress_s <= segment.progress_end_s + kEpsilon)
    {
      return &segment;
    }
  }
  return nullptr;
}

TrackingPoint CollectionTrackingCore::point_at(const double progress_s) const
{
  for (std::size_t index = 1; index < plan_.path.size(); ++index) {
    const auto & start = plan_.path[index - 1U];
    const auto & end = plan_.path[index];
    if (progress_s <= end.progress_s + kEpsilon) {
      const double t = std::clamp(
        (progress_s - start.progress_s) / (end.progress_s - start.progress_s), 0.0, 1.0);
      return TrackingPoint{
        start.x_m + t * (end.x_m - start.x_m), start.y_m + t * (end.y_m - start.y_m),
        progress_s, wrap_angle(start.heading_rad + t * wrap_angle(end.heading_rad - start.heading_rad))};
    }
  }
  return plan_.path.back();
}

double CollectionTrackingCore::tangent_at(const double progress_s) const
{
  for (std::size_t index = 1; index < plan_.path.size(); ++index) {
    if (progress_s <= plan_.path[index].progress_s + kEpsilon) {
      const auto & start = plan_.path[index - 1U];
      const auto & end = plan_.path[index];
      const double t = std::clamp(
        (progress_s - start.progress_s) / (end.progress_s - start.progress_s), 0.0, 1.0);
      return wrap_angle(start.heading_rad + t * wrap_angle(end.heading_rad - start.heading_rad));
    }
  }
  return plan_.path.back().heading_rad;
}

std::optional<TrackingPlannedCrossing> CollectionTrackingCore::next_crossing(
  const double progress_s) const
{
  for (const auto & segment : plan_.segments) {
    for (const auto & crossing : segment.planned_crossings) {
      if (crossing.progress_s + kEpsilon >= progress_s) { return crossing; }
    }
  }
  return std::nullopt;
}

TrackingResult CollectionTrackingCore::failure(const TrackingFailureCode code, const double progress_s) const
{
  TrackingResult result;
  result.status = TrackingStatus::kFailed;
  result.progress_s = progress_s;
  result.failure = code;
  return result;
}

}  // namespace tennis_robot_collection_controller
