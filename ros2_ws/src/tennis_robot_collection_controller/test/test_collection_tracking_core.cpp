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
    1.0, 0.8, 1.2, 0.1, 0.2, 1.0, 1.0, 2.0, 0.5, 1.0, false, false};
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

TEST(CollectionTrackingCore, HeadingGateAllowsBoundedPassEntryGraceThenBecomesStrict)
{
  auto entry = plan();
  entry.segments[0].profile.max_heading_error_rad = 0.15;
  entry.segments[0].profile.required_entry_m = 0.2;

  tc::CollectionTrackingCore inside_grace(entry);
  const auto aligning = inside_grace.update(input(0.01, 0.0, -0.164, 1.0));
  EXPECT_EQ(aligning.status, tc::TrackingStatus::kRunning);
  EXPECT_EQ(aligning.failure, tc::TrackingFailureCode::kNone);

  tc::CollectionTrackingCore after_grace(entry);
  const auto still_misaligned = after_grace.update(input(0.21, 0.0, -0.164, 1.0));
  EXPECT_EQ(still_misaligned.status, tc::TrackingStatus::kFailed);
  EXPECT_EQ(still_misaligned.failure, tc::TrackingFailureCode::kHeadingErrorExceeded);
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
    1.0, 0.1, 5.0, 0.1, 0.0, 0.0, 0.0, 100.0, 5.0, 3.14159, false, false};
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

// ── Frame diagnosis (Phase 11) ──────────────────────────────────────────────
//
// The core compares two bare coordinate pairs, so a frame mismatch between them
// is invisible to it and shows up only as an error value that cannot be
// reproduced from the objects it claims to have compared.  These tests pin the
// published diagnostic to the reported error: if the reference point ever stops
// being the point the projection actually measured to, they fail.

TEST(CollectionTrackingCore, ReportedLateralErrorIsReproducibleFromTheReferencePoint)
{
  tc::CollectionTrackingCore core(plan());
  for (const double offset : {0.0, 0.02, -0.03, 0.08}) {
    tc::CollectionTrackingCore fresh(plan());
    const auto result = fresh.update(input(1.0, offset, 0.0, 1.0));
    ASSERT_TRUE(result.has_reference);
    const double recomputed = std::hypot(1.0 - result.reference_x_m, offset - result.reference_y_m);
    EXPECT_NEAR(recomputed, result.lateral_error_m, 1e-9)
      << "lateral_error_m must be the distance to the published reference point";
  }
}

TEST(CollectionTrackingCore, ReportedHeadingErrorIsReproducibleFromTheReferenceTangent)
{
  auto turning = plan();
  turning.path = {{0.0, 0.0, 0.0, 0.0}, {2.0, 0.0, 2.0, 0.0}, {4.0, 2.0, 4.83, 0.785398163}};
  turning.segments[0].progress_end_s = 4.83;
  turning.segments[0].planned_crossings[0].progress_s = 3.0;
  turning.terminal_progress_s = 4.83;
  tc::CollectionTrackingCore core(turning);
  const double heading = 0.1;
  const auto result = core.update(input(1.0, 0.05, heading, 1.0));
  ASSERT_TRUE(result.has_reference);
  double recomputed = result.reference_heading_rad - heading;
  while (recomputed > M_PI) { recomputed -= 2.0 * M_PI; }
  while (recomputed <= -M_PI) { recomputed += 2.0 * M_PI; }
  EXPECT_NEAR(recomputed, result.heading_error_rad, 1e-9)
    << "heading_error_rad must be the published reference tangent minus the pose yaw";
}

TEST(CollectionTrackingCore, ReferencePointAccompaniesATrajectoryTubeFailure)
{
  // The tube abort is exactly where the two objects must be inspectable: the
  // core says "you are too far from the path", and the evidence for that claim
  // is the path point it measured to.
  tc::CollectionTrackingCore core(plan());
  const auto result = core.update(input(1.0, 0.9, 0.0, 1.0));
  ASSERT_EQ(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  EXPECT_TRUE(result.has_reference);
  EXPECT_NEAR(std::hypot(1.0 - result.reference_x_m, 0.9 - result.reference_y_m), 0.9, 1e-9);
}

// ── calibrated gates (Phase 14) ─────────────────────────────────────────────
//
// Map-anchored execution made the true error visible: straight passes track to
// ~0.01 m while connectors legitimately reach ~0.15 m while turning, and a pass
// entered from a connector starts at ~0.30 rad of heading error (debug log #74).
// These tests pin the two gates to that measured envelope.

namespace
{

tc::CollectionTrackingPlan plan_with_gates(const double tube_m, const double entry_m,
  const double heading_gate_rad = 1.0)
{
  auto shaped = plan();
  shaped.segments[0].profile.max_lateral_error_m = tube_m;
  shaped.segments[0].profile.required_entry_m = entry_m;
  shaped.segments[0].profile.max_heading_error_rad = heading_gate_rad;
  return shaped;
}

}  // namespace

TEST(CollectionTrackingCore, StraightPassErrorSitsFarInsideTheCalibratedTube)
{
  // The measured straight-pass core: median 0.009 m, max 0.043 m.
  tc::CollectionTrackingCore core(plan_with_gates(0.2, 0.0));
  for (const double lateral : {0.009, 0.026, 0.043}) {
    tc::CollectionTrackingCore fresh(plan_with_gates(0.2, 0.0));
    const auto result = fresh.update(input(1.0, lateral, 0.0, 1.0));
    EXPECT_EQ(result.status, tc::TrackingStatus::kRunning) << "lateral " << lateral;
  }
}

TEST(CollectionTrackingCore, LegitimateConnectorDeviationNoLongerAborts)
{
  // 0.15 m is inside the measured connector envelope (p95 0.148, max 0.214) and
  // used to abort against the old 0.10 m gate.
  tc::CollectionTrackingCore core(plan_with_gates(0.2, 0.0));
  const auto result = core.update(input(1.0, 0.15, 0.0, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_NEAR(result.lateral_error_m, 0.15, 1e-9);
}

TEST(CollectionTrackingCore, DeviationBeyondTheCalibratedTubeStillAborts)
{
  tc::CollectionTrackingCore core(plan_with_gates(0.2, 0.0));
  const auto result = core.update(input(1.0, 0.25, 0.0, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kFailed);
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
}

TEST(CollectionTrackingCore, ConnectorEntryHeadingIsAllowedToConvergeAcrossTheAllowance)
{
  // Entering at 0.30 rad against a 0.15 rad capture gate: inside the 0.8 m
  // allowance this must run, so the controller has room to align.
  auto shaped = plan_with_gates(0.2, 0.8, 0.15);
  tc::CollectionTrackingCore core(shaped);
  const auto entering = core.update(input(0.1, 0.0, -0.30, 1.0));
  EXPECT_EQ(entering.status, tc::TrackingStatus::kRunning);
  EXPECT_NEAR(std::abs(entering.heading_error_rad), 0.30, 1e-6);
}

TEST(CollectionTrackingCore, HeadingConvergedInsideTheAllowanceProceedsNormally)
{
  auto shaped = plan_with_gates(0.2, 0.8, 0.15);
  tc::CollectionTrackingCore core(shaped);
  core.update(input(0.1, 0.0, -0.30, 1.0));
  // By 0.5 m the measured runs are at 0.02-0.04 rad; past the allowance that
  // must be accepted by the full capture gate.
  const auto aligned = core.update(input(1.0, 0.0, -0.03, 1.0));
  EXPECT_EQ(aligned.status, tc::TrackingStatus::kRunning);
}

TEST(CollectionTrackingCore, HeadingStillExcessiveAfterTheAllowanceAborts)
{
  auto shaped = plan_with_gates(0.2, 0.8, 0.15);
  tc::CollectionTrackingCore core(shaped);
  core.update(input(0.1, 0.0, -0.30, 1.0));
  // Past 0.8 m and still at 0.30 rad: the capture-grade gate applies in full.
  const auto result = core.update(input(1.0, 0.0, -0.30, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kFailed);
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kHeadingErrorExceeded);
}

TEST(CollectionTrackingCore, AnAlreadyAlignedEntryBehavesExactlyAsBefore)
{
  auto shaped = plan_with_gates(0.2, 0.8, 0.15);
  tc::CollectionTrackingCore core(shaped);
  const auto result = core.update(input(0.1, 0.0, -0.02, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_NEAR(std::abs(result.heading_error_rad), 0.02, 1e-6);
}

TEST(CollectionTrackingCore, NoBallIsCrossableWhileTheAllowanceIsStillActive)
{
  // The invariant of Phase 14 §4: a capture segment whose first crossing sits
  // inside the alignment allowance is rejected, never executed with the
  // capture gate silently waived up to the ball.
  auto shaped = plan_with_gates(0.2, 0.8, 0.15);
  shaped.segments[0].planned_crossings[0].progress_s = 0.5;   // inside 0.8 m
  EXPECT_THROW(tc::CollectionTrackingCore core(shaped), std::invalid_argument);

  // 1.0 m of run-in -- what the planner actually produces -- is accepted.
  auto valid = plan_with_gates(0.2, 0.8, 0.15);
  valid.segments[0].planned_crossings[0].progress_s = 1.0;
  EXPECT_NO_THROW(tc::CollectionTrackingCore core(valid));
}

// ── truthful failure telemetry (Phase 15) ───────────────────────────────────
//
// A failing update used to return a default-constructed result, so the one
// sample that actually tripped a gate reported lateral 0.000 and heading 0.000 --
// every abort detail in the logs read as perfect tracking (debug log #75).
// These tests pin the reported evidence to what was really computed, and pin the
// failure point so that improving the evidence cannot move it.

TEST(CollectionTrackingCore, ATubeFailureReportsTheLateralErrorThatTrippedIt)
{
  auto shaped = plan();
  shaped.segments[0].profile.max_lateral_error_m = 0.20;
  tc::CollectionTrackingCore core(shaped);
  const auto result = core.update(input(1.0, 0.23, 0.0, 1.0));
  ASSERT_EQ(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  EXPECT_TRUE(result.has_geometry);
  EXPECT_NEAR(result.lateral_error_m, 0.23, 1e-9) << "the tripping value must be reported";
  EXPECT_NEAR(result.heading_error_rad, 0.0, 1e-9);
  EXPECT_NEAR(result.progress_s, 1.0, 1e-9);
  EXPECT_TRUE(result.has_reference);
  EXPECT_NEAR(std::hypot(1.0 - result.reference_x_m, 0.23 - result.reference_y_m), 0.23, 1e-9);
}

TEST(CollectionTrackingCore, AHeadingFailureReportsBothErrorsThatWereMeasured)
{
  auto shaped = plan();
  shaped.segments[0].profile.max_heading_error_rad = 0.15;
  shaped.segments[0].profile.required_entry_m = 0.0;
  tc::CollectionTrackingCore core(shaped);
  const auto result = core.update(input(1.0, 0.04, -0.31, 1.0));
  ASSERT_EQ(result.failure, tc::TrackingFailureCode::kHeadingErrorExceeded);
  EXPECT_TRUE(result.has_geometry);
  EXPECT_NEAR(result.heading_error_rad, 0.31, 1e-9);
  EXPECT_NEAR(result.lateral_error_m, 0.04, 1e-9) << "lateral is measured too, not zeroed";
}

TEST(CollectionTrackingCore, ANonMonotonicFailureReportsBothProgressValuesItCompared)
{
  tc::CollectionTrackingCore core(plan());
  core.update(input(3.0, 0.0, 0.0, 1.0));          // advance
  const auto result = core.update(input(1.0, 0.05, 0.0, 1.0));   // fall back
  ASSERT_EQ(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
  EXPECT_TRUE(result.has_raw_projection);
  EXPECT_NEAR(result.previous_progress_s, 3.0, 1e-6);
  EXPECT_NEAR(result.raw_projection_progress_s, 1.0, 1e-6);
  EXPECT_LT(result.raw_projection_progress_s, result.previous_progress_s);
  EXPECT_TRUE(result.has_geometry);
  // The reported lateral error is the distance to the *monotonic* projection,
  // which after a backward jump is pinned at the previous progress and is
  // therefore large.  That is the true measurement and is exactly the signature
  // worth seeing: it distinguishes a backward jump from a lateral excursion.
  EXPECT_NEAR(result.lateral_error_m,
    std::hypot(3.0 - 1.0, 0.0 - 0.05), 1e-6);
}

TEST(CollectionTrackingCore, AFailureWithNothingComputedSaysSoInsteadOfReportingZero)
{
  tc::CollectionTrackingCore core(plan());
  const auto result = core.update(
    input(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kFailed);
  EXPECT_FALSE(result.has_geometry) << "absent evidence must not look like zero error";
  EXPECT_FALSE(result.has_reference);
}

TEST(CollectionTrackingCore, TruthfulTelemetryDoesNotMoveTheFailurePoint)
{
  // The gates are unchanged: just inside still runs, just outside still fails,
  // on the same input, for both the lateral and the heading gate.
  auto shaped = plan();
  shaped.segments[0].profile.max_lateral_error_m = 0.20;
  shaped.segments[0].profile.max_heading_error_rad = 0.15;
  shaped.segments[0].profile.required_entry_m = 0.0;
  {
    tc::CollectionTrackingCore core(shaped);
    EXPECT_EQ(core.update(input(1.0, 0.199, 0.0, 1.0)).status, tc::TrackingStatus::kRunning);
  }
  {
    tc::CollectionTrackingCore core(shaped);
    EXPECT_EQ(core.update(input(1.0, 0.201, 0.0, 1.0)).failure,
      tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  }
  {
    tc::CollectionTrackingCore core(shaped);
    EXPECT_EQ(core.update(input(1.0, 0.0, -0.149, 1.0)).status, tc::TrackingStatus::kRunning);
  }
  {
    tc::CollectionTrackingCore core(shaped);
    EXPECT_EQ(core.update(input(1.0, 0.0, -0.151, 1.0)).failure,
      tc::TrackingFailureCode::kHeadingErrorExceeded);
  }
}

TEST(CollectionTrackingCore, ASuccessfulUpdateIsUnchangedApartFromAddedDiagnostics)
{
  tc::CollectionTrackingCore core(plan());
  const auto result = core.update(input(1.0, 0.01, 0.02, 1.0));
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_DOUBLE_EQ(result.command.linear_x_mps, 1.0);
  EXPECT_NEAR(result.lateral_error_m, 0.01, 1e-9);
  EXPECT_NEAR(result.heading_error_rad, -0.02, 1e-9);
  EXPECT_TRUE(result.has_geometry);
  EXPECT_NEAR(result.previous_progress_s, 0.0, 1e-9);
}

// ── re-anchoring detection (Phase 16) ───────────────────────────────────────
//
// Every abort measured in Phase 15 happened on the exact update where the
// estimated map pose stepped 0.21-0.35 m while the robot physically moved
// ~0.014 m (debug log #76).  The tracker cannot tell that from the robot
// leaving the corridor -- both are a step in cross-track -- so it is separated
// here on kinematics: could the robot have travelled this far in this time?

namespace
{

tc::TrackingInput moving(const double x_m, const double y_m, const double heading_rad,
  const double elapsed_s)
{
  return {x_m, y_m, heading_rad, 1.0, false, elapsed_s};
}

tc::CollectionTrackingPlan tube_plan()
{
  auto shaped = plan();
  shaped.segments[0].profile.max_lateral_error_m = 0.20;
  shaped.segments[0].profile.max_heading_error_rad = 1.0;
  return shaped;   // max_speed_mps 1.2 -> bound 1.2*dt + 0.10
}

}  // namespace

TEST(CollectionTrackingCore, OrdinaryMotionIsNeverClassifiedAsReAnchoring)
{
  // The measured normal envelope: 0.0139 m median, 0.111 m worst, at ~0.04 s.
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  for (const double step : {0.0139, 0.0543, 0.111}) {
    const auto result = core.update(moving(1.0 + step, 0.0, 0.0, 0.04));
    EXPECT_FALSE(result.reanchoring_detected) << "normal step " << step;
    EXPECT_FALSE(result.tube_verdict_deferred);
  }
}

TEST(CollectionTrackingCore, AGradualDepartureBeyondTheTubeStillAbortsNormally)
{
  // Physically plausible drift off the corridor: no grace, no deferral.
  tc::CollectionTrackingCore core(tube_plan());
  tc::TrackingResult result;
  double lateral = 0.0;
  for (int step = 0; step < 12 && result.failure == tc::TrackingFailureCode::kNone; ++step) {
    lateral += 0.02;
    result = core.update(moving(1.0 + 0.01 * step, lateral, 0.0, 0.04));
  }
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  EXPECT_FALSE(result.reanchoring_detected);
  EXPECT_GT(result.lateral_error_m, 0.20);
}

TEST(CollectionTrackingCore, ASingleLocalizationJumpDefersTheTubeVerdict)
{
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  // 0.30 m in 0.04 s against a bound of 1.2*0.04 + 0.10 = 0.148 m.
  const auto result = core.update(moving(1.0, 0.30, 0.0, 0.04));
  EXPECT_TRUE(result.reanchoring_detected);
  EXPECT_TRUE(result.tube_verdict_deferred);
  EXPECT_NE(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  EXPECT_NEAR(result.lateral_error_m, 0.30, 1e-6) << "the real error is still reported";
  EXPECT_NEAR(result.plausible_step_bound_m, 0.148, 1e-9);
  EXPECT_NEAR(result.pose_step_m, 0.30, 1e-6);
}

TEST(CollectionTrackingCore, ErrorThatPersistsAfterAJumpAbortsOnTheNextUpdate)
{
  // The safety gate: deferring a verdict must not grant tolerance to a robot
  // that really is off the corridor.
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  ASSERT_TRUE(core.update(moving(1.0, 0.30, 0.0, 0.04)).tube_verdict_deferred);
  const auto next = core.update(moving(1.01, 0.30, 0.0, 0.04));
  EXPECT_EQ(next.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
  EXPECT_FALSE(next.tube_verdict_deferred);
  EXPECT_GT(next.lateral_error_m, 0.20);
}

TEST(CollectionTrackingCore, AJumpBackTowardTheCorridorIsTreatedTheSameWay)
{
  // 1 of the 6 measured late jumps was a valid re-anchoring toward truth and
  // aborted anyway.  The controller must not try to judge whether localization
  // was right -- only whether the motion was physically possible.
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.30, 0.0, 0.04));
  const auto result = core.update(moving(1.0, 0.0, 0.0, 0.04));
  EXPECT_TRUE(result.reanchoring_detected);
  EXPECT_EQ(result.status, tc::TrackingStatus::kRunning);
  EXPECT_NEAR(result.lateral_error_m, 0.0, 1e-6);
}

TEST(CollectionTrackingCore, ConsecutiveJumpsCannotCompoundIntoOpenEndedGrace)
{
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  EXPECT_TRUE(core.update(moving(1.0, 0.30, 0.0, 0.04)).tube_verdict_deferred);
  // Second discontinuity in a row, still outside the tube: the deferral is not
  // available again and the abort stands.
  const auto second = core.update(moving(1.0, 0.62, 0.0, 0.04));
  EXPECT_TRUE(second.reanchoring_detected);
  EXPECT_FALSE(second.tube_verdict_deferred);
  EXPECT_EQ(second.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
}

TEST(CollectionTrackingCore, AnOrdinaryUpdateBetweenTwoJumpsRestoresTheDeferral)
{
  // jump -> ordinary -> jump: two independent events, each allowed one verdict
  // deferral.  Only *consecutive* deferrals are refused.
  tc::CollectionTrackingCore core(tube_plan());
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  EXPECT_TRUE(core.update(moving(1.0, 0.30, 0.0, 0.04)).tube_verdict_deferred);
  const auto ordinary = core.update(moving(1.02, 0.02, 0.0, 0.04));
  EXPECT_FALSE(ordinary.tube_verdict_deferred);
  EXPECT_EQ(ordinary.status, tc::TrackingStatus::kRunning);
  EXPECT_TRUE(core.update(moving(1.02, 0.32, 0.0, 0.04)).tube_verdict_deferred);
}

TEST(CollectionTrackingCore, WithoutElapsedTimeNoReAnchoringClaimIsMade)
{
  // A step cannot be called impossible without knowing how long it took.
  tc::CollectionTrackingCore core(tube_plan());
  core.update(input(1.0, 0.0, 0.0, 1.0));
  const auto result = core.update(input(1.0, 0.30, 0.0, 1.0));
  EXPECT_FALSE(result.reanchoring_detected);
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kTrajectoryTubeExceeded);
}

TEST(CollectionTrackingCore, TheHeadingGateIsUnaffectedByReAnchoring)
{
  // Measured: a discontinuity moves heading error by ~0.0004 rad on a straight
  // pass.  Nothing here suppresses the heading gate.
  auto shaped = tube_plan();
  shaped.segments[0].profile.max_heading_error_rad = 0.15;
  shaped.segments[0].profile.required_entry_m = 0.0;
  tc::CollectionTrackingCore core(shaped);
  core.update(moving(1.0, 0.0, 0.0, 0.04));
  const auto result = core.update(moving(1.0, 0.30, -0.40, 0.04));
  EXPECT_TRUE(result.reanchoring_detected);
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kHeadingErrorExceeded);
}

TEST(CollectionTrackingCore, TheSameSequenceProducesTheSameEvents)
{
  const std::vector<tc::TrackingInput> sequence{
    moving(1.0, 0.0, 0.0, 0.04), moving(1.0, 0.30, 0.0, 0.04),
    moving(1.01, 0.02, 0.0, 0.04), moving(1.02, 0.34, 0.0, 0.04),
  };
  std::vector<std::tuple<int, bool, bool>> first;
  std::vector<std::tuple<int, bool, bool>> second;
  for (auto * sink : {&first, &second}) {
    tc::CollectionTrackingCore core(tube_plan());
    for (const auto & step : sequence) {
      const auto result = core.update(step);
      sink->emplace_back(static_cast<int>(result.failure), result.reanchoring_detected,
        result.tube_verdict_deferred);
    }
  }
  EXPECT_EQ(first, second);
}

// ── progress-projection continuity (Phase 17A) ──────────────────────────────
//
// Live failure (debug log #77): accepted progress 19.5560 m, raw projection
// 9.7422 m, a 9.81 m "regression" while the robot moved 0.0139 m and sat
// 0.0142 m from the path.  The route returned near an earlier branch and the
// global nearest-point search preferred it.  Accepted progress was never at
// risk -- the bounded projection cannot move backward -- so the defect was
// entirely in the evidence the regression check consulted.

namespace
{

/// A route that comes back alongside itself, matching the live geometry: out
/// 5 m along y=0, around, and back along y=0.30.  The two legs are 0.30 m apart
/// but ~9 m apart in route progress -- inside the legacy 10 m window, which is
/// exactly how the live 19.556 -> 9.742 m false regression arose.
tc::CollectionTrackingPlan self_near_plan()
{
  std::vector<tc::TrackingPoint> path;
  double progress = 0.0;
  const auto push = [&](double x, double y, double heading) {
    if (!path.empty()) {
      progress += std::hypot(x - path.back().x_m, y - path.back().y_m);
    }
    path.push_back({x, y, progress, heading});
  };
  for (int step = 0; step <= 25; ++step) { push(0.2 * step, 0.0, 0.0); }          // 0 .. 5 m
  push(5.2, 0.15, 1.5708);
  push(5.2, 0.30, 1.5708);
  // The return leg crosses the outbound one: at the crossing the two branches
  // are coincident, which is where a global nearest-point search can pick
  // either -- and, on ties, keeps the earlier one.
  for (int step = 0; step <= 25; ++step) {
    const double ratio = step / 25.0;
    push(5.0 - 5.0 * ratio, 0.30 - 0.60 * ratio, 3.14159);
  }
  const double total = path.back().progress_s;
  const tc::TrackingExecutionProfile lenient{
    0.6, 0.1, 0.6, 0.1, 0.0, 0.0, 0.0, 100.0, 0.20, 3.0, false, false};
  return tc::CollectionTrackingPlan{
    path, {{"loop", 0.0, total, lenient, {}}}, total, {1.0, 100.0, 10.0, 0.25, 0.05}};
}

/// Drive along the outbound leg up to `x_target`.
void advance_to(tc::CollectionTrackingCore & core, const double x_target,
  const double elapsed_s)
{
  for (double x = 0.0; x <= x_target; x += 0.014) {
    core.update({x, 0.0, 0.0, 0.35, false, elapsed_s});
  }
}

/// Drive right around onto the return leg, ending near its far end -- where the
/// outbound leg passes 0.30 m away and ~9 m earlier in route progress.
void advance_onto_return_leg(tc::CollectionTrackingCore & core, const double elapsed_s)
{
  advance_to(core, 5.0, elapsed_s);
  for (const auto & pose : std::vector<std::array<double, 3>>{
      {{5.1, 0.02, 0.5}}, {{5.18, 0.09, 1.2}}, {{5.2, 0.16, 1.5708}},
      {{5.2, 0.26, 2.4}}, {{5.12, 0.30, 3.14159}}}) {
    core.update({pose[0], pose[1], pose[2], 0.35, false, elapsed_s});
  }
  for (double x = 5.0; x >= 2.55; x -= 0.014) {
    core.update({x, 0.30 - 0.60 * (5.0 - x) / 5.0, 3.14159, 0.35, false, elapsed_s});
  }
}

}  // namespace

TEST(CollectionTrackingCore, WithoutElapsedTimeTheSelfNearBranchStillFoolsTheCheck)
{
  // The defect as it stood: with no kinematic basis the legacy 10 m window
  // admits a candidate 9.8 m behind.  This is the "fails without the fix" case.
  tc::CollectionTrackingCore core(self_near_plan());
  advance_onto_return_leg(core, 0.0);
  // At the crossing the outbound branch is coincident and several metres behind.
  const auto result = core.update({2.50, 0.0, 3.14159, 0.35, false, 0.0});
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
  EXPECT_LT(result.raw_projection_progress_s, result.previous_progress_s - 1.0);
}

TEST(CollectionTrackingCore, AReachableNeighbourhoodRejectsTheSelfNearBranch)
{
  tc::CollectionTrackingCore core(self_near_plan());
  advance_onto_return_leg(core, 0.04);
  const auto result = core.update({2.50, 0.0, 3.14159, 0.35, false, 0.04});
  EXPECT_NE(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress)
    << "a branch 9.8 m away is not the robot moving backward";
  EXPECT_TRUE(result.raw_projection_constrained)
    << "the nearer-but-unreachable candidate must be recorded as rejected";
  // 0.6 m/s * 0.04 s + 0.10 m margin.
  EXPECT_NEAR(result.projection_reach_m, 0.124, 1e-9);
}

TEST(CollectionTrackingCore, AcceptedProgressNeverMovesBackward)
{
  tc::CollectionTrackingCore core(self_near_plan());
  advance_onto_return_leg(core, 0.04);
  const double before = core.last_progress_s();
  const auto result = core.update({2.50, 0.0, 3.14159, 0.35, false, 0.04});
  EXPECT_GE(result.progress_s, before - 1e-9)
    << "the bounded projection is clamped forward by construction";
}

TEST(CollectionTrackingCore, GenuineBackwardMotionIsStillDetected)
{
  // A real slip backward along the route, inside the reachable neighbourhood
  // and beyond the progress tolerance, must still be caught.
  tc::CollectionTrackingCore core(self_near_plan());
  advance_to(core, 3.0, 0.04);
  const auto result = core.update({2.90, 0.0, 0.0, 0.35, false, 0.04});
  EXPECT_EQ(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
}

TEST(CollectionTrackingCore, NumericalJitterIsNotReportedAsRegression)
{
  tc::CollectionTrackingCore core(self_near_plan());
  advance_to(core, 3.0, 0.04);
  const double settled = core.last_progress_s();
  // Inside progress_tolerance_m (0.05): jitter, not regression.
  const auto result = core.update({settled - 0.02, 0.0, 0.0, 0.35, false, 0.04});
  EXPECT_NE(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
}

TEST(CollectionTrackingCore, AStationaryRobotIsNotReportedAsRegression)
{
  tc::CollectionTrackingCore core(self_near_plan());
  advance_to(core, 3.0, 0.04);
  const double settled = core.last_progress_s();
  for (int step = 0; step < 5; ++step) {
    const auto result = core.update({settled, 0.0, 0.0, 0.0, false, 0.04});
    EXPECT_NE(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
  }
}

TEST(CollectionTrackingCore, NormalForwardTrackingIsUnchangedByTheInvariant)
{
  tc::CollectionTrackingCore core(self_near_plan());
  double previous = 0.0;
  for (double x = 0.2; x <= 4.5; x += 0.014) {
    const auto result = core.update({x, 0.0, 0.0, 0.35, false, 0.04});
    ASSERT_EQ(result.status, tc::TrackingStatus::kRunning) << "at x=" << x;
    EXPECT_GE(result.progress_s, previous - 1e-9);
    previous = result.progress_s;
  }
  EXPECT_NEAR(previous, 4.5, 0.05);
}

TEST(CollectionTrackingCore, ProjectionSurvivesTheTurnBetweenTheTwoLegs)
{
  // The segment transition itself -- through the curve joining the legs -- must
  // keep advancing, which is where an over-tight neighbourhood would stall.
  tc::CollectionTrackingCore core(self_near_plan());
  advance_to(core, 4.9, 0.04);
  const std::vector<std::array<double, 3>> corner{
    {{5.0, 0.0, 0.0}}, {{5.15, 0.06, 0.9}}, {{5.2, 0.15, 1.5708}},
    {{5.2, 0.24, 2.4}}, {{5.1, 0.30, 3.14159}}, {{4.96, 0.295, 3.14159}},
  };
  double previous = core.last_progress_s();
  for (const auto & pose : corner) {
    const auto result = core.update({pose[0], pose[1], pose[2], 0.35, false, 0.04});
    EXPECT_NE(result.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
    EXPECT_GE(result.progress_s, previous - 1e-9);
    previous = result.progress_s;
  }
  EXPECT_GT(previous, 5.0) << "the route must progress past the turn";
}

TEST(CollectionTrackingCore, ALocalizationJumpDoesNotSnapProgressToAnotherBranch)
{
  // normal -> re-anchoring -> normal, on the self-near geometry: the pose step
  // is impossible, but progress must neither regress nor snap to the far branch.
  tc::CollectionTrackingCore core(self_near_plan());
  advance_to(core, 3.0, 0.04);
  const double before = core.last_progress_s();
  const auto jumped = core.update({3.30, 0.0, 0.0, 0.35, false, 0.04});
  EXPECT_TRUE(jumped.reanchoring_detected);
  EXPECT_NE(jumped.failure, tc::TrackingFailureCode::kNonMonotonicProgress);
  EXPECT_GE(jumped.progress_s, before - 1e-9);
  const auto after = core.update({3.32, 0.0, 0.0, 0.35, false, 0.04});
  EXPECT_EQ(after.status, tc::TrackingStatus::kRunning);
  EXPECT_GE(after.progress_s, jumped.progress_s - 1e-9);
}

TEST(CollectionTrackingCore, TheSelfNearSequenceIsDeterministic)
{
  std::vector<std::pair<int, double>> first;
  std::vector<std::pair<int, double>> second;
  for (auto * sink : {&first, &second}) {
    tc::CollectionTrackingCore core(self_near_plan());
    advance_onto_return_leg(core, 0.04);
    for (const double x : {2.52, 2.50, 2.48}) {
      const auto result = core.update({x, 0.30 - 0.60 * (5.0 - x) / 5.0, 3.14159, 0.35, false, 0.04});
      sink->emplace_back(static_cast<int>(result.failure), result.progress_s);
    }
  }
  EXPECT_EQ(first, second);
}
