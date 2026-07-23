#include <limits>

#include <gtest/gtest.h>

#include "tennis_robot_collection_controller/collection_path_canonicalization.hpp"
#include "tennis_robot_collection_controller/collection_execution_context_contract.hpp"

namespace tc = tennis_robot_collection_controller;

nav_msgs::msg::Path make_path()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped first;
  first.pose.position.x = 1.0;
  first.pose.position.y = -2.0;
  first.pose.orientation.w = 1.0;
  path.poses.push_back(first);
  return path;
}

TEST(CollectionPathCanonicalizationV1, IgnoresHeaderTimestamp)
{
  auto first = make_path();
  auto second = first;
  first.header.stamp.sec = 1;
  second.header.stamp.sec = 999;
  EXPECT_EQ(tc::canonicalize_collection_path_v1(first), tc::canonicalize_collection_path_v1(second));
  EXPECT_EQ(tc::collection_path_sha256_v1(first), tc::collection_path_sha256_v1(second));
}

TEST(CollectionPathCanonicalizationV1, ChangesForAnyBoundField)
{
  auto first = make_path();
  auto changed_pose = first;
  changed_pose.poses[0].pose.position.x = 1.5;
  auto changed_frame = first;
  changed_frame.header.frame_id = "odom";
  EXPECT_NE(tc::collection_path_sha256_v1(first), tc::collection_path_sha256_v1(changed_pose));
  EXPECT_NE(tc::collection_path_sha256_v1(first), tc::collection_path_sha256_v1(changed_frame));
}

TEST(CollectionPathCanonicalizationV1, RejectsNonFiniteValues)
{
  auto path = make_path();
  path.poses[0].pose.position.y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(tc::canonicalize_collection_path_v1(path), tc::CanonicalizationError);
}

TEST(CollectionPathCanonicalizationV1, RejectsInvalidUtf8FrameId)
{
  auto path = make_path();
  path.header.frame_id = std::string("\xc3\x28", 2);
  EXPECT_THROW(tc::canonicalize_collection_path_v1(path), tc::CanonicalizationError);
}

TEST(CollectionExecutionContextContract, HashMismatchPreservesLoadedContextAndTimeoutClearsIt)
{
  tc::CollectionExecutionContextContract contract("collection-execution-context/v1");
  EXPECT_EQ(contract.begin_matching_follow_path(std::string(64, 'a'), 0.0).rejection,
    tc::ContextRejectionCode::kInvalidLifecycle);
  const tc::CollectionExecutionContextIdentity context{
    "collection-execution-context/v1", "plan-a", std::string(64, 'a'), 5.0};
  EXPECT_TRUE(contract.load(context, 10.0).accepted);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kContextLoaded);
  EXPECT_EQ(contract.begin_matching_follow_path(std::string(64, 'b'), 11.0).rejection,
    tc::ContextRejectionCode::kPlanHashMismatch);
  ASSERT_NE(contract.context(), nullptr);
  EXPECT_EQ(contract.context()->plan_id, "plan-a");
  EXPECT_EQ(contract.begin_matching_follow_path(std::string(64, 'a'), 15.0).rejection,
    tc::ContextRejectionCode::kContextActivationTimeout);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kIdle);
  EXPECT_EQ(contract.context(), nullptr);
}

TEST(CollectionExecutionContextContract, TerminalConsumesUntilExplicitResetAndSafetyHoldIsBound)
{
  tc::CollectionExecutionContextContract contract("collection-execution-context/v1");
  const tc::CollectionExecutionContextIdentity context{
    "collection-execution-context/v1", "plan-a", std::string(64, 'a'), 5.0};
  ASSERT_TRUE(contract.load(context, 0.0).accepted);
  ASSERT_TRUE(contract.begin_matching_follow_path(std::string(64, 'a'), 1.0).accepted);
  EXPECT_EQ(contract.set_safety_hold("wrong", std::string(64, 'a'), true, true).rejection,
    tc::ContextRejectionCode::kPlanHashMismatch);
  EXPECT_TRUE(contract.set_safety_hold("plan-a", std::string(64, 'a'), true, true).emit_zero_velocity);
  EXPECT_EQ(contract.set_safety_hold("plan-a", std::string(64, 'a'), false, false).rejection,
    tc::ContextRejectionCode::kResumeContractFailed);
  ASSERT_TRUE(contract.set_safety_hold("plan-a", std::string(64, 'a'), false, true).accepted);
  EXPECT_TRUE(contract.terminal_success().emit_zero_velocity);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kConsumed);
  EXPECT_EQ(contract.load(context, 2.0).rejection, tc::ContextRejectionCode::kContextAlreadyConsumed);
  EXPECT_TRUE(contract.reset().accepted);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kIdle);
}

TEST(CollectionExecutionContextContract, FinalizeRequiresMatchingExecutingContext)
{
  tc::CollectionExecutionContextContract contract("collection-execution-context/v1");
  const tc::CollectionExecutionContextIdentity context{
    "collection-execution-context/v1", "plan-a", std::string(64, 'a'), 5.0};
  EXPECT_EQ(contract.finalize("plan-a", std::string(64, 'a'), 0, false).rejection,
    tc::ContextRejectionCode::kMissingContext);
  ASSERT_TRUE(contract.load(context, 0.0).accepted);
  EXPECT_EQ(contract.finalize("plan-a", std::string(64, 'a'), 0, false).rejection,
    tc::ContextRejectionCode::kInvalidLifecycle);
  ASSERT_TRUE(contract.begin_matching_follow_path(std::string(64, 'a'), 1.0).accepted);
  EXPECT_EQ(contract.finalize("plan-a", std::string(64, 'a'), 0, false).rejection,
    tc::ContextRejectionCode::kTerminalNotReached);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kExecuting);
  EXPECT_EQ(contract.finalize("other", std::string(64, 'a'), 0, false).rejection,
    tc::ContextRejectionCode::kPlanHashMismatch);
  EXPECT_TRUE(contract.finalize("plan-a", std::string(64, 'a'), 0, true).emit_zero_velocity);
  EXPECT_EQ(contract.lifecycle(), tc::ContextLifecycle::kConsumed);
}
