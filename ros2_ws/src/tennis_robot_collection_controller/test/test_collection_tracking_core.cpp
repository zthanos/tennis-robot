#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tennis_robot_collection_controller/collection_tracking_core.hpp"

namespace tc = tennis_robot_collection_controller;

namespace
{

tc::TrackingExecutionProfile profile()
{
  return tc::TrackingExecutionProfile{
    1.0, 0.8, 1.2, 0.1, 1.0, 1.0, 2.0, 0.5, 1.0, false, false};
}

tc::CollectionTrackingPlan plan()
{
  return tc::CollectionTrackingPlan{
    {{0.0, 0.0, 0.0}, {5.0, 0.0, 5.0}},
    {{"pass-a", 0.0, 5.0, profile(), {{"ball-a", 3.0}}}},
    5.0,
    {1.0, 2.0, 10.0, 0.25, 0.05}};
}

tc::TrackingInput input(const double x_m, const double y_m, const double heading_rad,
  const double measured_speed_mps, const bool safety_hold = false)
{
  return {x_m, y_m, heading_rad, measured_speed_mps, safety_hold};
}

}  // namespace

TEST(CollectionTrackingCore, NominalForwardTrackingProducesForwardCommand)
{
  tc::CollectionTrackingCore core(plan());
  const auto result = core.update(input(1.0, 0.0, 0.0, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_DOUBLE_EQ(result.command.linear_x_mps, 1.0);
  EXPECT_DOUBLE_EQ(result.command.angular_z_rad_s, 0.0);
  EXPECT_GT(result.command.linear_x_mps, 0.0);
}

TEST(CollectionTrackingCore, CurvatureBoundFailsRatherThanClippingSpeed)
{
  auto constrained = plan();
  constrained.path = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.41421356237}, {5.0, 1.0, 5.41421356237}};
  constrained.segments[0].progress_end_s = 5.41421356237;
  constrained.segments[0].planned_crossings[0].progress_s = 3.0;
  constrained.terminal_progress_s = 5.41421356237;
  constrained.segments[0].profile.max_curvature_per_m = 0.2;
  tc::CollectionTrackingCore core(constrained);
  const auto result = core.update(input(0.0, 0.0, 0.0, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kFailed);
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kCurvatureExceeded);
  EXPECT_DOUBLE_EQ(result.command.linear_x_mps, 0.0);
}

TEST(CollectionTrackingCore, HeadingGateUsesPathYawNotIntentionalLookaheadBearing)
{
  auto curved = plan();
  curved.path = {
    {0.0, 0.0, 0.0, 0.0},
    {0.707, -0.293, 0.765, -0.785},
    {1.0, -1.0, 1.530, -1.570}};
  curved.segments[0].progress_end_s = 1.530;
  curved.segments[0].planned_crossings.clear();
  curved.segments[0].profile.max_heading_error_rad = 0.1;
  curved.terminal_progress_s = 1.530;

  tc::CollectionTrackingCore aligned(curved);
  const auto result = aligned.update(input(0.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_NE(result.command.angular_z_rad_s, 0.0);
  EXPECT_NEAR(result.heading_error_rad, 0.0, 1e-9);

  tc::CollectionTrackingCore misaligned(curved);
  EXPECT_EQ(misaligned.update(input(0.0, 0.0, 0.2, 0.0)).failure,
    tc::TrackingFailureCode::kHeadingErrorExceeded);
}

TEST(CollectionTrackingCore, CrossingSpeedBelowAndAboveBoundsAreHardFailures)
{
  tc::CollectionTrackingCore below_core(plan());
  const auto below = below_core.update(input(3.0, 0.0, 0.0, 0.7));
  EXPECT_EQ(below.failure, tc::TrackingFailureCode::kSpeedBelowMin);
  ASSERT_TRUE(below.crossing_measurement.has_value());
  EXPECT_FALSE(below.crossing_measurement->verdict.hard_compliant);

  tc::CollectionTrackingCore above_core(plan());
  const auto above = above_core.update(input(3.0, 0.0, 0.0, 1.3));
  EXPECT_EQ(above.failure, tc::TrackingFailureCode::kSpeedAboveMax);
  ASSERT_TRUE(above.crossing_measurement.has_value());
  EXPECT_FALSE(above.crossing_measurement->verdict.hard_compliant);
}

TEST(CollectionTrackingCore, NominalDeviationInsideHardBoundsIsTelemetryOnly)
{
  tc::CollectionTrackingCore core(plan());
  const auto result = core.update(input(3.0, 0.0, 0.0, 1.15));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  ASSERT_TRUE(result.crossing_measurement.has_value());
  EXPECT_TRUE(result.crossing_measurement->verdict.hard_compliant);
  EXPECT_EQ(result.crossing_measurement->verdict.nominal_tracking, tc::NominalTracking::kDeviated);
}

TEST(CollectionTrackingCore, TubeAndProgressViolationsAreTyped)
{
  tc::CollectionTrackingCore tube_core(plan());
  EXPECT_EQ(tube_core.update(input(1.0, 0.6, 0.0, 1.0)).failure,
    tc::TrackingFailureCode::kTrajectoryTubeExceeded);

  tc::CollectionTrackingCore progress_core(plan());
  ASSERT_EQ(progress_core.update(input(2.0, 0.0, 0.0, 1.0)).status, tc::TrackingStatus::kRunning);
  EXPECT_EQ(progress_core.update(input(1.0, 0.0, 0.0, 1.0)).failure,
    tc::TrackingFailureCode::kNonMonotonicProgress);

  tc::CollectionTrackingCore jitter_core(plan());
  ASSERT_EQ(jitter_core.update(input(2.0, 0.0, 0.0, 1.0)).status,
    tc::TrackingStatus::kRunning);
  EXPECT_EQ(jitter_core.update(input(1.98, 0.0, 0.0, 1.0)).status,
    tc::TrackingStatus::kRunning);
}

TEST(CollectionTrackingCore, ResumeRequiresRemainingRunInAndHoldDoesNotMeasureCrossingSpeed)
{
  tc::CollectionTrackingCore core(plan());
  const auto hold = core.update(input(2.9, 0.0, 0.0, 0.1, true));
  EXPECT_EQ(hold.status, tc::TrackingStatus::kSafetyHold);
  EXPECT_FALSE(hold.crossing_measurement.has_value());
  EXPECT_EQ(core.update(input(2.9, 0.0, 0.0, 1.0)).failure,
    tc::TrackingFailureCode::kRunInInsufficient);

  tc::CollectionTrackingCore resumable_core(plan());
  ASSERT_EQ(resumable_core.update(input(1.0, 0.0, 0.0, 1.0, true)).status, tc::TrackingStatus::kSafetyHold);
  EXPECT_EQ(resumable_core.update(input(1.0, 0.0, 0.0, 1.0)).status, tc::TrackingStatus::kRunning);
}

TEST(CollectionTrackingCore, TerminalConditionAndNoReverseOrStandaloneRotateCommand)
{
  tc::CollectionTrackingCore terminal_core(plan());
  const auto terminal = terminal_core.update(input(4.96, 0.0, 0.0, 1.0));
  EXPECT_EQ(terminal.status, tc::TrackingStatus::kCompleted);
  EXPECT_TRUE(terminal.terminal_ready);
  EXPECT_DOUBLE_EQ(terminal.progress_s, 5.0);

  tc::CollectionTrackingCore forward_core(plan());
  const auto forward = forward_core.update(input(1.0, 0.0, 0.4, 1.0));
  EXPECT_EQ(forward.status, tc::TrackingStatus::kRunning);
  EXPECT_GT(forward.command.linear_x_mps, 0.0);
  EXPECT_NE(forward.command.angular_z_rad_s, 0.0);

  tc::CollectionTrackingCore reverse_core(plan());
  EXPECT_EQ(reverse_core.update(input(1.0, 0.0, 0.0, -0.1)).failure,
    tc::TrackingFailureCode::kReverseRequired);
}

TEST(CollectionTrackingCore, TerminalProgressAloneCannotCompleteAwayFromEndpoint)
{
  // The final segment passes the earlier point (0, 0) before ending at
  // (0, 0.4). A projection near terminal progress is insufficient unless the
  // robot is also physically inside the terminal tolerance.
  auto near_return = plan();
  near_return.path = {
    {0.0, 0.0, 0.0, 0.0},
    {2.0, 0.0, 2.0, 0.0},
    {0.0, 0.0, 4.0, M_PI},
    {0.0, 0.4, 4.4, M_PI_2}};
  near_return.segments[0].progress_end_s = 4.4;
  near_return.segments[0].planned_crossings.clear();
  near_return.segments[0].profile.max_lateral_error_m = 1.0;
  near_return.segments[0].profile.max_heading_error_rad = M_PI;
  near_return.segments[0].profile.max_curvature_per_m = 100.0;
  near_return.terminal_progress_s = 4.4;
  near_return.tuning.progress_projection_window_m = 10.0;
  near_return.tuning.progress_tolerance_m = 0.3;

  tc::CollectionTrackingCore core(near_return);
  const auto away = core.update(input(0.0, 0.0, M_PI_2, 1.0));
  EXPECT_NE(away.status, tc::TrackingStatus::kCompleted);

  tc::CollectionTrackingCore terminal_core(near_return);
  const auto terminal = terminal_core.update(input(0.0, 0.4, M_PI_2, 1.0));
  EXPECT_EQ(terminal.status, tc::TrackingStatus::kCompleted);
}

TEST(CollectionTrackingCore, SelfCrossingLoopDoesNotFalselyReportNonMonotonicProgress)
{
  // A figure-eight (A*sin 2t, A*sin t) passes through the origin at t=0, pi and
  // 2*pi.  When the robot reaches the mid-path crossing (t=pi) a *global*
  // nearest-point search snaps back to the t=0 origin (progress 0) and would
  // falsely report backward motion.  The windowed check must ignore that distant
  // self-intersection while still tracking the loop.  Large scale keeps curvature
  // gentle; the lenient profile leaves only the non-monotonic guard in play.
  const double amplitude = 10.0;
  const int steps = 64;  // t = 0..2pi with a sample exactly at t = pi (i = 32)
  std::vector<tc::TrackingPoint> path;
  double progress = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  for (int i = 0; i <= steps; ++i) {
    const double t = 2.0 * M_PI * static_cast<double>(i) / steps;
    const double x = amplitude * std::sin(2.0 * t);
    const double y = amplitude * std::sin(t);
    if (i > 0) { progress += std::hypot(x - prev_x, y - prev_y); }
    const double heading = std::atan2(amplitude * std::cos(t), 2.0 * amplitude * std::cos(2.0 * t));
    path.push_back({x, y, progress, heading});
    prev_x = x;
    prev_y = y;
  }
  const double total = path.back().progress_s;
  const tc::TrackingExecutionProfile lenient{
    1.0, 0.1, 5.0, 0.1, 0.0, 0.0, 100.0, 5.0, 3.14159, false, false};
  const tc::CollectionTrackingPlan loop_plan{
    path, {{"loop", 0.0, total, lenient, {}}}, total,
    {1.0, 100.0, 10.0, 0.25, 0.05}};
  tc::CollectionTrackingCore core(loop_plan);

  bool reached_crossing = false;
  bool saw_non_monotonic = false;
  for (int i = 0; i <= steps; ++i) {
    const auto & p = path[i];
    const auto result = core.update(input(p.x_m, p.y_m, p.heading_rad, 1.0));
    if (result.failure == tc::TrackingFailureCode::kNonMonotonicProgress) {
      saw_non_monotonic = true;
    }
    // The mid-path crossing sample must not abort as a false regression.
    if (i == steps / 2 && result.failure != tc::TrackingFailureCode::kNonMonotonicProgress) {
      reached_crossing = true;
    }
  }
  EXPECT_FALSE(saw_non_monotonic);
  EXPECT_TRUE(reached_crossing);
}
