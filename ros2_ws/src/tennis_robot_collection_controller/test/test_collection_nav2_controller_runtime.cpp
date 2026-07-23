#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "tennis_robot_collection_controller/collection_nav2_controller.hpp"
#include "tennis_robot_collection_controller/collection_path_canonicalization.hpp"
#include "tennis_robot_msgs/msg/collection_controller_state.hpp"
#include "tennis_robot_msgs/srv/finalize_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/load_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/reset_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/set_collection_safety_hold.hpp"

namespace tc = tennis_robot_collection_controller;
namespace trm = tennis_robot_msgs;

class CollectionNav2ControllerRuntimeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto suffix = std::to_string(++counter_);
    controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("collection_controller_test_" + suffix);
    client_node_ = std::make_shared<rclcpp::Node>("collection_client_test_" + suffix);
    executor_.add_node(controller_node_->get_node_base_interface());
    executor_.add_node(client_node_);
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    controller_ = std::dynamic_pointer_cast<tc::CollectionNav2Controller>(
      plugin_loader_->createSharedInstance("tennis_robot_collection_controller::CollectionNav2Controller"));
    ASSERT_NE(controller_, nullptr);
    controller_->configure(controller_node_, "collection", nullptr, nullptr);
    controller_->activate();
    load_ = client_node_->create_client<trm::srv::LoadCollectionExecutionContext>("collection/load_collection_execution_context");
    reset_ = client_node_->create_client<trm::srv::ResetCollectionExecutionContext>("collection/reset_collection_execution_context");
    hold_ = client_node_->create_client<trm::srv::SetCollectionSafetyHold>("collection/set_collection_safety_hold");
    finalize_ = client_node_->create_client<trm::srv::FinalizeCollectionExecutionContext>("collection/finalize_collection_execution_context");
    state_subscription_ = client_node_->create_subscription<trm::msg::CollectionControllerState>("collection/state", 10,
      [this](trm::msg::CollectionControllerState::SharedPtr state) { telemetry_.push_back(*state); });
    ASSERT_TRUE(load_->wait_for_service(std::chrono::seconds(1)));
  }

  void TearDown() override
  {
    controller_->deactivate(); controller_->cleanup(); controller_.reset(); plugin_loader_.reset();
    executor_.remove_node(client_node_); executor_.remove_node(controller_node_->get_node_base_interface());
    state_subscription_.reset(); client_node_.reset(); controller_node_.reset();
  }

  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request)
  {
    auto future = client->async_send_request(request);
    for (int attempt = 0; attempt < 100 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++attempt) {
      executor_.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    return future.get();
  }

  nav_msgs::msg::Path path() const
  {
    nav_msgs::msg::Path path; path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped first; first.pose.orientation.w = 1.0;
    geometry_msgs::msg::PoseStamped last; last.pose.position.x = 5.0; last.pose.orientation.w = 1.0;
    path.poses = {first, last}; return path;
  }

  trm::msg::CollectionExecutionContext context() const
  {
    trm::msg::CollectionExecutionContext context;
    context.context_schema_version = "collection-execution-context/v1";
    context.plan_id = "plan-a"; context.path_sha256 = tc::collection_path_sha256_v1(path());
    context.context_activation_timeout_s = 5.0; context.terminal_progress_s = 5.0;
    context.terminal_pose.position.x = 5.0; context.terminal_pose.orientation.w = 1.0;
    context.configuration_snapshot_json = "{}";
    context.controller_tuning.lookahead_distance_m = 1.0;
    context.controller_tuning.max_angular_velocity_rad_s = 2.0;
    context.controller_tuning.progress_projection_window_m = 10.0;
    context.controller_tuning.crossing_speed_window_m = 0.25;
    context.controller_tuning.terminal_progress_tolerance_m = 0.01;
    trm::msg::CollectionExecutionSegment segment;
    segment.segment_id = "pass-a"; segment.progress_start_s = 0.0; segment.progress_end_s = 5.0;
    auto & profile = segment.execution_profile;
    profile.nominal_speed_mps = 1.0; profile.min_speed_mps = 0.8; profile.max_speed_mps = 1.2;
    profile.nominal_speed_warning_tolerance_mps = 0.1; profile.max_acceleration_mps2 = 1.0;
    profile.max_deceleration_mps2 = 1.0; profile.required_entry_m = 0.0;
    profile.required_run_in_m = 1.0; profile.required_run_out_m = 1.0;
    profile.max_curvature_per_m = 2.0; profile.max_lateral_error_m = 0.5;
    profile.max_heading_error_rad = 1.0; profile.allow_reversing = false; profile.allow_standalone_rotate = false;
    trm::msg::CollectionPlannedCrossing crossing;
    crossing.ball_id = "ball-a"; crossing.position_x_m = 3.0; crossing.position_y_m = 0.0;
    crossing.progress_s = 3.0; crossing.heading_rad = 0.0; crossing.predicted_lateral_error = 0.0;
    segment.planned_crossings = {crossing}; context.segments = {segment}; return context;
  }

  void load_and_set_plan()
  {
    auto request = std::make_shared<trm::srv::LoadCollectionExecutionContext::Request>(); request->context = context();
    ASSERT_TRUE(call<trm::srv::LoadCollectionExecutionContext>(load_, request)->accepted); controller_->setPlan(path());
  }

  geometry_msgs::msg::PoseStamped pose(const double x, const double y = 0.0) const
  {
    geometry_msgs::msg::PoseStamped pose; pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.orientation.w = 1.0; return pose;
  }

  void spin() { for (int index = 0; index < 5; ++index) { executor_.spin_some(); } }

  static int counter_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> plugin_loader_;
  std::shared_ptr<tc::CollectionNav2Controller> controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<trm::srv::LoadCollectionExecutionContext>::SharedPtr load_;
  rclcpp::Client<trm::srv::ResetCollectionExecutionContext>::SharedPtr reset_;
  rclcpp::Client<trm::srv::SetCollectionSafetyHold>::SharedPtr hold_;
  rclcpp::Client<trm::srv::FinalizeCollectionExecutionContext>::SharedPtr finalize_;
  rclcpp::Subscription<trm::msg::CollectionControllerState>::SharedPtr state_subscription_;
  std::vector<trm::msg::CollectionControllerState> telemetry_;
};

int CollectionNav2ControllerRuntimeTest::counter_ = 0;

TEST_F(CollectionNav2ControllerRuntimeTest, ServiceLifecycleAndSemanticRejection)
{
  auto invalid = context(); invalid.controller_tuning.lookahead_distance_m = 0.0;
  auto invalid_request = std::make_shared<trm::srv::LoadCollectionExecutionContext::Request>(); invalid_request->context = invalid;
  const auto rejected = call<trm::srv::LoadCollectionExecutionContext>(load_, invalid_request);
  EXPECT_FALSE(rejected->accepted); EXPECT_EQ(rejected->rejection_code, trm::srv::LoadCollectionExecutionContext::Response::INVALID_CONTEXT);
  auto request = std::make_shared<trm::srv::LoadCollectionExecutionContext::Request>(); request->context = context();
  EXPECT_TRUE(call<trm::srv::LoadCollectionExecutionContext>(load_, request)->accepted);
  EXPECT_EQ(call<trm::srv::LoadCollectionExecutionContext>(load_, request)->rejection_code, trm::srv::LoadCollectionExecutionContext::Response::CONTROLLER_NOT_IDLE);
  auto mismatched_path = path(); mismatched_path.poses.back().pose.position.x = 4.0;
  EXPECT_THROW(controller_->setPlan(mismatched_path), std::runtime_error);
  auto reset_request = std::make_shared<trm::srv::ResetCollectionExecutionContext::Request>();
  EXPECT_TRUE(call<trm::srv::ResetCollectionExecutionContext>(reset_, reset_request)->accepted);
}

TEST_F(CollectionNav2ControllerRuntimeTest, ForwardCommandTelemetryAndHold)
{
  load_and_set_plan(); geometry_msgs::msg::Twist velocity; velocity.linear.x = 1.0;
  const auto command = controller_->computeVelocityCommands(pose(1.0), velocity, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0); spin(); ASSERT_FALSE(telemetry_.empty());
  EXPECT_EQ(telemetry_.back().plan_id, "plan-a"); EXPECT_EQ(telemetry_.back().path_sha256, context().path_sha256);
  EXPECT_EQ(telemetry_.back().active_segment_id, "pass-a");
  auto hold_request = std::make_shared<trm::srv::SetCollectionSafetyHold::Request>();
  hold_request->plan_id = "plan-a"; hold_request->path_sha256 = context().path_sha256; hold_request->hold = true;
  ASSERT_TRUE(call<trm::srv::SetCollectionSafetyHold>(hold_, hold_request)->accepted);
  const auto stopped = controller_->computeVelocityCommands(pose(1.0), velocity, nullptr);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x, 0.0); EXPECT_DOUBLE_EQ(stopped.twist.angular.z, 0.0);
}

TEST_F(CollectionNav2ControllerRuntimeTest, HardFailureConsumesAndPublishesTypedTelemetry)
{
  load_and_set_plan(); geometry_msgs::msg::Twist velocity; velocity.linear.x = 1.0;
  EXPECT_THROW(controller_->computeVelocityCommands(pose(1.0, 0.6), velocity, nullptr), std::runtime_error);
  spin(); ASSERT_FALSE(telemetry_.empty());
  EXPECT_EQ(telemetry_.back().failure_reason, trm::msg::CollectionControllerState::FAILURE_TRAJECTORY_TUBE_EXCEEDED);
  auto reset_request = std::make_shared<trm::srv::ResetCollectionExecutionContext::Request>();
  EXPECT_TRUE(call<trm::srv::ResetCollectionExecutionContext>(reset_, reset_request)->accepted);
}

TEST_F(CollectionNav2ControllerRuntimeTest, TerminalFinalizeIsGatedAndThenAccepted)
{
  load_and_set_plan();
  auto request = std::make_shared<trm::srv::FinalizeCollectionExecutionContext::Request>();
  request->plan_id = "plan-a"; request->path_sha256 = context().path_sha256;
  request->action_outcome = trm::srv::FinalizeCollectionExecutionContext::Request::SUCCEEDED;
  EXPECT_EQ(call<trm::srv::FinalizeCollectionExecutionContext>(finalize_, request)->rejection_code, trm::srv::FinalizeCollectionExecutionContext::Response::TERMINAL_NOT_REACHED);
  geometry_msgs::msg::Twist velocity; velocity.linear.x = 1.0;
  const auto terminal = controller_->computeVelocityCommands(pose(5.0), velocity, nullptr);
  EXPECT_DOUBLE_EQ(terminal.twist.linear.x, 0.0);
  EXPECT_TRUE(call<trm::srv::FinalizeCollectionExecutionContext>(finalize_, request)->accepted);
  EXPECT_EQ(call<trm::srv::FinalizeCollectionExecutionContext>(finalize_, request)->rejection_code, trm::srv::FinalizeCollectionExecutionContext::Response::INVALID_LIFECYCLE);
}

TEST_F(CollectionNav2ControllerRuntimeTest, CrossingNominalDeviationIsTelemetryOnly)
{
  load_and_set_plan();
  geometry_msgs::msg::Twist velocity; velocity.linear.x = 1.15;
  const auto command = controller_->computeVelocityCommands(pose(3.0), velocity, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
  spin(); ASSERT_FALSE(telemetry_.empty());
  const auto & state = telemetry_.back();
  EXPECT_EQ(state.plan_id, "plan-a"); EXPECT_EQ(state.path_sha256, context().path_sha256);
  EXPECT_EQ(state.active_segment_id, "pass-a"); EXPECT_TRUE(state.has_active_crossing);
  EXPECT_EQ(state.active_ball_id, "ball-a"); EXPECT_DOUBLE_EQ(state.active_crossing_progress_s, 3.0);
  EXPECT_EQ(state.profile_verdict.nominal_tracking,
    trm::msg::CollectionProfileComplianceVerdict::NOMINAL_DEVIATED);
  EXPECT_TRUE(state.profile_verdict.hard_compliant);
  EXPECT_EQ(state.failure_reason, trm::msg::CollectionControllerState::FAILURE_NONE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
