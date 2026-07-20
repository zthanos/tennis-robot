#include <gtest/gtest.h>

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
    {1.0, 2.0, 10.0, 0.25}};
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
  EXPECT_EQ(terminal_core.update(input(5.0, 0.0, 0.0, 1.0)).status, tc::TrackingStatus::kCompleted);

  tc::CollectionTrackingCore forward_core(plan());
  const auto forward = forward_core.update(input(1.0, 0.0, 0.4, 1.0));
  EXPECT_EQ(forward.status, tc::TrackingStatus::kRunning);
  EXPECT_GT(forward.command.linear_x_mps, 0.0);
  EXPECT_NE(forward.command.angular_z_rad_s, 0.0);

  tc::CollectionTrackingCore reverse_core(plan());
  EXPECT_EQ(reverse_core.update(input(1.0, 0.0, 0.0, -0.1)).failure,
    tc::TrackingFailureCode::kReverseRequired);
}
