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

// Localization noise floor for the re-anchoring test.  Across 7597 measured
// updates the map-frame pose step never exceeded 0.111 m, of which at most
// 0.037 m is kinematically explainable at the observed update periods -- the
// rest is estimate noise.  The smallest discontinuity ever observed was
// 0.209 m, so this margin sits clear of normal noise and well below any real
// jump (debug log #76).
constexpr double kPoseNoiseMarginM = 0.10;

// Same noise floor, applied to route progress: a raw projection this far from
// the accepted progress is still plausibly the robot, anything beyond it is a
// different part of the route (debug log #77).
constexpr double kProgressReachMarginM = 0.10;

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
  // The point on the plan path that `lateral_error_m` was measured to.  Carried
  // out of the projection purely so the comparison can be reproduced offline.
  double reference_x_m{};
  double reference_y_m{};
  // Set after the scan, never through aggregate initialisation -- keep these
  // last so the brace-init sites above stay positional and correct.
  double raw_progress_s{};
  bool has_raw{};
  double reach_m{};
  bool raw_constrained{};
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
    // The heading-alignment allowance at the entry of a capture segment must
    // end before the first ball can be crossed: relaxing the capture-grade gate
    // right up to a crossing would waive the requirement exactly where it
    // matters.  A pass too short to hold both is rejected loudly rather than
    // executed with a silently waived gate.
    if (!segment.planned_crossings.empty() &&
      segment.planned_crossings.front().progress_s - segment.progress_start_s + kEpsilon <
      segment.profile.required_entry_m)
    {
      throw std::invalid_argument("capture segment shorter than its heading entry allowance");
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
  for (const auto & segment : plan_.segments) {
    plan_max_speed_mps_ = std::max(plan_max_speed_mps_, segment.profile.max_speed_mps);
  }
}

TrackingResult CollectionTrackingCore::update(const TrackingInput & input)
{
  if (!finite(input.x_m) || !finite(input.y_m) || !finite(input.heading_rad) ||
    !finite(input.measured_speed_mps))
  {
    // Nothing was computed from a non-finite pose: `has_geometry` stays false
    // rather than reporting a fabricated zero.
    auto result = failure(TrackingFailureCode::kProfileUnenforceable, last_progress_s_);
    result.previous_progress_s = last_progress_s_;
    return result;
  }
  if (input.safety_hold) {
    safety_held_ = true;
    TrackingResult result;
    result.status = TrackingStatus::kSafetyHold;
    result.progress_s = last_progress_s_;
    return result;
  }

  // Could the robot physically have produced this map-frame pose delta in this
  // much time?  The bound comes from the plan's own speed contract plus a
  // localization noise margin; with no elapsed time there is nothing to judge
  // against, so no claim is made.
  const bool had_previous_input = has_last_input_;
  const double pose_step_m = has_last_input_ ?
    std::hypot(input.x_m - last_input_x_m_, input.y_m - last_input_y_m_) : 0.0;
  const double pose_step_yaw_rad = has_last_input_ ?
    wrap_angle(input.heading_rad - last_input_heading_rad_) : 0.0;
  last_input_x_m_ = input.x_m;
  last_input_y_m_ = input.y_m;
  last_input_heading_rad_ = input.heading_rad;
  has_last_input_ = true;

  const auto projection = project_monotonically(input.x_m, input.y_m, input.elapsed_s);
  const double previous_progress_s = last_progress_s_;
  // Computed here, before any gate, so every failure below can report the
  // geometry it was judged on.  Pure functions of the projection: moving the
  // computation earlier cannot move the failure point.
  const double path_heading_error = projection.found ?
    wrap_angle(tangent_at(projection.progress_s) - input.heading_rad) : 0.0;
  double plausible_step_bound_m = 0.0;
  bool reanchoring_detected = false;
  const auto with_geometry = [&](TrackingResult result) {
    result.pose_step_m = pose_step_m;
    result.plausible_step_bound_m = plausible_step_bound_m;
    result.reanchoring_detected = reanchoring_detected;
    result.pose_step_yaw_rad = pose_step_yaw_rad;
    result.elapsed_s = input.elapsed_s;
    result.previous_progress_s = previous_progress_s;
    result.raw_projection_progress_s = projection.raw_progress_s;
    result.has_raw_projection = projection.has_raw;
    result.projection_reach_m = projection.reach_m;
    result.raw_projection_constrained = projection.raw_constrained;
    if (!projection.found) { return result; }
    result.lateral_error_m = projection.lateral_error_m;
    result.heading_error_rad = path_heading_error;
    result.reference_x_m = projection.reference_x_m;
    result.reference_y_m = projection.reference_y_m;
    result.reference_heading_rad = tangent_at(projection.progress_s);
    result.has_reference = true;
    result.has_geometry = true;
    return result;
  };
  if (projection.raw_projection_behind) {
    // The regression verdict compares the raw projection with the previous
    // progress; both now travel with the failure, together with the lateral and
    // heading errors measured at the same instant.
    auto result = with_geometry(
      failure(TrackingFailureCode::kNonMonotonicProgress, last_progress_s_));
    return result;
  }
  if (!projection.found) {
    return with_geometry(failure(TrackingFailureCode::kProfileUnenforceable, last_progress_s_));
  }
  const auto * segment = active_segment(projection.progress_s);
  if (segment == nullptr) {
    return with_geometry(failure(TrackingFailureCode::kProfileUnenforceable, projection.progress_s));
  }
  const auto next = next_crossing(projection.progress_s);
  const double remaining_run_in = next ? std::max(0.0, next->progress_s - projection.progress_s) : 0.0;
  const double remaining_run_out = next ? std::max(0.0, segment->progress_end_s - next->progress_s) : 0.0;
  if (safety_held_ && next && remaining_run_in + kEpsilon < segment->profile.required_run_in_m) {
    return with_geometry(failure(TrackingFailureCode::kRunInInsufficient, projection.progress_s));
  }
  safety_held_ = false;
  // The bound is the plan's own speed contract over the elapsed time, plus the
  // measured localization noise floor.  Unknown elapsed time makes no claim.
  plausible_step_bound_m = input.elapsed_s > 0.0 ?
    segment->profile.max_speed_mps * input.elapsed_s + kPoseNoiseMarginM : 0.0;
  reanchoring_detected = had_previous_input && input.elapsed_s > 0.0 &&
    pose_step_m > plausible_step_bound_m;
  const bool outside_tube =
    projection.lateral_error_m > segment->profile.max_lateral_error_m + kEpsilon;
  // A pose delta the robot could not physically have travelled is a revision of
  // the estimate, not the robot leaving the corridor.  Defer that one verdict --
  // and only if the previous update was not itself deferred, so consecutive
  // discontinuities can never compound into an open-ended grace period.
  const bool defer_tube = outside_tube && reanchoring_detected && !deferred_previous_update_;
  deferred_previous_update_ = defer_tube;
  const auto with_deferral = [&](TrackingResult result) {
    result.tube_verdict_deferred = defer_tube;
    return result;
  };
  if (outside_tube && !defer_tube) {
    return with_geometry(
      failure(TrackingFailureCode::kTrajectoryTubeExceeded, projection.progress_s));
  }
  if (input.measured_speed_mps < -kEpsilon) {
    return with_geometry(failure(TrackingFailureCode::kReverseRequired, projection.progress_s));
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
    return with_geometry(result);
  }

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
    return with_geometry(result);
  }

  const auto lookahead = point_at(std::min(
    plan_.terminal_progress_s, projection.progress_s + plan_.tuning.lookahead_distance_m));
  const double target_angle = std::atan2(lookahead.y_m - input.y_m, lookahead.x_m - input.x_m);
  const double lookahead_bearing_error = wrap_angle(target_angle - input.heading_rad);
  const double target_distance = std::hypot(lookahead.x_m - input.x_m, lookahead.y_m - input.y_m);
  if (target_distance <= kEpsilon) {
    return with_geometry(failure(TrackingFailureCode::kStandaloneRotateRequired, projection.progress_s));
  }
  if (std::cos(lookahead_bearing_error) <= kEpsilon) {
    return with_geometry(failure(TrackingFailureCode::kReverseRequired, projection.progress_s));
  }
  const double curvature = 2.0 * std::sin(lookahead_bearing_error) / target_distance;
  if (std::abs(curvature) > segment->profile.max_curvature_per_m + kEpsilon) {
    return with_geometry(failure(TrackingFailureCode::kCurvatureExceeded, projection.progress_s));
  }
  const double angular_velocity = segment->profile.nominal_speed_mps * curvature;
  if (std::abs(angular_velocity) > plan_.tuning.max_angular_velocity_rad_s + kEpsilon) {
    return with_geometry(failure(TrackingFailureCode::kProfileUnenforceable, projection.progress_s));
  }

  TrackingResult result;
  result.status = TrackingStatus::kRunning;
  result.lookahead_x_m = lookahead.x_m;
  result.lookahead_y_m = lookahead.y_m;
  result.lookahead_distance_m = target_distance;
  result.commanded_curvature_per_m = curvature;
  result.has_pursuit_geometry = true;
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
      return with_geometry(result);
    }
    if (input.measured_speed_mps > segment->profile.max_speed_mps) {
      verdict.hard_compliant = false;
      verdict.hard_violation_reason = TrackingFailureCode::kSpeedAboveMax;
      result.crossing_measurement = CrossingMeasurement{next->ball_id, next->progress_s, verdict};
      result.status = TrackingStatus::kFailed;
      result.failure = TrackingFailureCode::kSpeedAboveMax;
      return with_geometry(result);
    }
    verdict.nominal_tracking = std::abs(verdict.nominal_speed_error_mps) >
      segment->profile.nominal_speed_warning_tolerance_mps ? NominalTracking::kDeviated :
      NominalTracking::kWithinTolerance;
    result.crossing_measurement = CrossingMeasurement{next->ball_id, next->progress_s, verdict};
  }
  last_progress_s_ = projection.progress_s;
  return with_deferral(with_geometry(result));
}

double CollectionTrackingCore::last_progress_s() const { return last_progress_s_; }

CollectionTrackingCore::Projection CollectionTrackingCore::project_monotonically(
  const double x_m, const double y_m, const double elapsed_s) const
{
  Projection raw;
  Projection bounded;
  double raw_distance = std::numeric_limits<double>::infinity();
  double bounded_distance = std::numeric_limits<double>::infinity();
  bool constrained_candidate_seen = false;
  const double window_end = last_progress_s_ + plan_.tuning.progress_projection_window_m;
  // How far along the route could the robot possibly be, in either direction,
  // since the previous update?  Only candidates inside that neighbourhood are
  // evidence about where the robot *is*; anything further is a different part of
  // a route that happens to pass close by.  With no elapsed time there is no
  // kinematic basis, so the legacy window stands and behaviour is unchanged.
  const double reach_m = elapsed_s > 0.0 ?
    plan_max_speed_mps_ * elapsed_s + kProgressReachMarginM :
    plan_.tuning.progress_projection_window_m;
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
    // global nearest-point search is fooled by a route that returns near an
    // earlier part of itself: the global nearest snaps back metres and is
    // reported as regression while the robot moved a centimetre.  Admitting
    // only candidates the robot could physically have reached separates
    // "moved backward along the route" from "another branch passes close by".
    const bool within_regression_window =
      std::abs(raw_progress - last_progress_s_) <= reach_m + kEpsilon;
    if (!within_regression_window && current_raw_distance < raw_distance) {
      constrained_candidate_seen = true;
    }
    if (within_regression_window && current_raw_distance < raw_distance) {
      raw_distance = current_raw_distance;
      raw = Projection{raw_progress, current_raw_distance, false, true, raw_x, raw_y};
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
        current_bounded_distance, false, true, bounded_x, bounded_y};
    }
  }
  bounded.raw_projection_behind = raw.found &&
    raw.progress_s + plan_.tuning.progress_tolerance_m + kEpsilon < last_progress_s_;
  // Carried for diagnosis: the non-monotonic verdict is a comparison between
  // this and `last_progress_s_`, and neither was previously reportable.
  bounded.raw_progress_s = raw.progress_s;
  bounded.has_raw = raw.found;
  bounded.reach_m = reach_m;
  // True when a nearer candidate existed outside the reachable neighbourhood --
  // the evidence that a self-near branch was rejected rather than never seen.
  bounded.raw_constrained = constrained_candidate_seen;
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
