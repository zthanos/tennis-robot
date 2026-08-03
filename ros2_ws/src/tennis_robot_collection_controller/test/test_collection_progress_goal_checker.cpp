#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "tennis_robot_collection_controller/collection_progress_goal_checker.hpp"
#include "tennis_robot_msgs/msg/collection_controller_state.hpp"

namespace tc = tennis_robot_collection_controller;
namespace trm = tennis_robot_msgs;

class CollectionProgressGoalCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto suffix = std::to_string(++counter_);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_test_" + suffix);
    publisher_node_ = std::make_shared<rclcpp::Node>("goal_checker_publisher_" + suffix);
    const auto prefix = "checker_" + suffix;
    node_->declare_parameter(prefix + ".xy_goal_tolerance", 0.30);
    node_->declare_parameter(prefix + ".yaw_goal_tolerance", 3.14);
    node_->declare_parameter(prefix + ".progress_tolerance_m", 0.05);
    node_->declare_parameter(prefix + ".state_timeout_s", 1.0);
    node_->declare_parameter(prefix + ".controller_state_topic", "goal_state_" + suffix);
    checker_.initialize(node_, prefix, nullptr);
    publisher_ = publisher_node_->create_publisher<trm::msg::CollectionControllerState>(
      "goal_state_" + suffix, 10);
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(publisher_node_);
  }

  void TearDown() override
  {
    checker_.reset();
    executor_.remove_node(publisher_node_);
    executor_.remove_node(node_->get_node_base_interface());
    publisher_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  trm::msg::CollectionControllerState state(double progress, bool terminal_ready) const
  {
    trm::msg::CollectionControllerState state;
    state.plan_id = "plan-a";
    state.path_sha256 = "hash-a";
    state.lifecycle_state = trm::msg::CollectionControllerState::EXECUTING;
    state.failure_reason = trm::msg::CollectionControllerState::FAILURE_NONE;
    state.progress_s = progress;
    state.terminal_progress_s = 10.0;
    state.terminal_ready = terminal_ready;
    return state;
  }

  void publish(const trm::msg::CollectionControllerState & state)
  {
    publisher_->publish(state);
    for (int attempt = 0; attempt < 20; ++attempt) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  bool reached(double query_x = 0.10, double goal_x = 0.0)
  {
    geometry_msgs::msg::Pose query;
    geometry_msgs::msg::Pose goal;
    geometry_msgs::msg::Twist velocity;
    query.position.x = query_x;
    query.orientation.w = 1.0;
    goal.position.x = goal_x;
    goal.orientation.w = 1.0;
    return checker_.isGoalReached(query, goal, velocity);
  }

  static int counter_;
  tc::CollectionProgressGoalChecker checker_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<trm::msg::CollectionControllerState>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

int CollectionProgressGoalCheckerTest::counter_ = 0;

TEST_F(CollectionProgressGoalCheckerTest, RejectsEndpointProximityAtMidRoute)
{
  // Even an inconsistent true terminal verdict cannot bypass the explicit
  // along-path progress gate.
  publish(state(5.0, true));
  EXPECT_FALSE(reached());
}

TEST_F(CollectionProgressGoalCheckerTest, AcceptsOnlyTerminalProgressAndProximity)
{
  publish(state(9.96, true));
  EXPECT_TRUE(reached());
  EXPECT_FALSE(reached(0.31));
}

TEST_F(CollectionProgressGoalCheckerTest, RequiresControllerTerminalVerdict)
{
  publish(state(10.0, false));
  EXPECT_FALSE(reached());
}

TEST_F(CollectionProgressGoalCheckerTest, ResetCannotReusePreviousRouteState)
{
  publish(state(10.0, true));
  ASSERT_TRUE(reached());
  checker_.reset();
  EXPECT_FALSE(reached());
}

TEST_F(CollectionProgressGoalCheckerTest, RejectsPausedOrFailedState)
{
  auto paused = state(10.0, true);
  paused.lifecycle_state = trm::msg::CollectionControllerState::SAFETY_PAUSED;
  publish(paused);
  EXPECT_FALSE(reached());
  auto failed = state(10.0, true);
  failed.failure_reason = trm::msg::CollectionControllerState::FAILURE_TERMINAL_NOT_REACHED;
  publish(failed);
  EXPECT_FALSE(reached());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
