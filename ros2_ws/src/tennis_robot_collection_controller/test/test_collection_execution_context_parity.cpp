// Phase 6B Part 4 — cross-language parity.
//
// Proves the REAL C++ controller accepts a CollectionExecutionContext + path
// produced by the pure Python Phase 6B serializer.  The Python side writes a
// fixture JSON (scripts/emit_collection_parity_fixture.py) from a real
// plan_collection_route plan; this test reconstructs the wire messages and:
//   (i)   collection_path_sha256_v1(path) == the Python-computed path_sha256,
//   (ii)  the Load service ACCEPTS the context (valid_load_context, incl. the
//         nlohmann canonical-JSON round-trip on configuration_snapshot_json),
//   (iii) setPlan(path) is accepted (path hash match + make_tracking_plan).
//
// Fixture path comes from the COLLECTION_PARITY_FIXTURE env var.  This is test
// scaffolding only: no controller implementation is modified.

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tennis_robot_collection_controller/collection_path_canonicalization.hpp"
#include "tennis_robot_msgs/msg/collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/load_collection_execution_context.hpp"

namespace tc = tennis_robot_collection_controller;
namespace trm = tennis_robot_msgs;
using json = nlohmann::json;

namespace
{

json load_fixture()
{
  const char * path = std::getenv("COLLECTION_PARITY_FIXTURE");
  if (path == nullptr) {
    throw std::runtime_error("COLLECTION_PARITY_FIXTURE env var is not set");
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error(std::string("cannot open fixture: ") + path);
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return json::parse(buffer.str());
}

geometry_msgs::msg::Pose pose_from_array(const json & array)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = array.at(0).get<double>();
  pose.position.y = array.at(1).get<double>();
  pose.position.z = array.at(2).get<double>();
  pose.orientation.x = array.at(3).get<double>();
  pose.orientation.y = array.at(4).get<double>();
  pose.orientation.z = array.at(5).get<double>();
  pose.orientation.w = array.at(6).get<double>();
  return pose;
}

nav_msgs::msg::Path build_path(const json & fixture)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = fixture.at("map_frame").get<std::string>();
  for (const auto & pose_array : fixture.at("poses")) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.pose = pose_from_array(pose_array);
    path.poses.push_back(stamped);
  }
  return path;
}

trm::msg::CollectionExecutionContext build_context(const json & fixture)
{
  const auto & source = fixture.at("context");
  trm::msg::CollectionExecutionContext context;
  context.context_schema_version = source.at("context_schema_version").get<std::string>();
  context.plan_id = source.at("plan_id").get<std::string>();
  context.path_sha256 = source.at("path_sha256").get<std::string>();
  context.context_activation_timeout_s = source.at("context_activation_timeout_s").get<double>();
  context.terminal_progress_s = source.at("terminal_progress_s").get<double>();
  context.terminal_pose = pose_from_array(source.at("terminal_pose"));
  context.configuration_snapshot_json = source.at("configuration_snapshot_json").get<std::string>();

  const auto & tuning = source.at("controller_tuning");
  context.controller_tuning.lookahead_distance_m = tuning.at("lookahead_distance_m").get<double>();
  context.controller_tuning.max_angular_velocity_rad_s = tuning.at("max_angular_velocity_rad_s").get<double>();
  context.controller_tuning.progress_projection_window_m = tuning.at("progress_projection_window_m").get<double>();
  context.controller_tuning.crossing_speed_window_m = tuning.at("crossing_speed_window_m").get<double>();
  context.controller_tuning.terminal_progress_tolerance_m = tuning.at("terminal_progress_tolerance_m").get<double>();

  for (const auto & segment_source : source.at("segments")) {
    trm::msg::CollectionExecutionSegment segment;
    segment.segment_id = segment_source.at("segment_id").get<std::string>();
    segment.segment_type = segment_source.at("segment_type").get<int>();
    segment.progress_start_s = segment_source.at("progress_start_s").get<double>();
    segment.progress_end_s = segment_source.at("progress_end_s").get<double>();
    const auto & profile = segment_source.at("execution_profile");
    auto & wire = segment.execution_profile;
    wire.nominal_speed_mps = profile.at("nominal_speed_mps").get<double>();
    wire.min_speed_mps = profile.at("min_speed_mps").get<double>();
    wire.max_speed_mps = profile.at("max_speed_mps").get<double>();
    wire.nominal_speed_warning_tolerance_mps = profile.at("nominal_speed_warning_tolerance_mps").get<double>();
    wire.max_acceleration_mps2 = profile.at("max_acceleration_mps2").get<double>();
    wire.max_deceleration_mps2 = profile.at("max_deceleration_mps2").get<double>();
    wire.required_entry_m = profile.at("required_entry_m").get<double>();
    wire.required_run_in_m = profile.at("required_run_in_m").get<double>();
    wire.required_run_out_m = profile.at("required_run_out_m").get<double>();
    wire.max_curvature_per_m = profile.at("max_curvature_per_m").get<double>();
    wire.max_lateral_error_m = profile.at("max_lateral_error_m").get<double>();
    wire.max_heading_error_rad = profile.at("max_heading_error_rad").get<double>();
    wire.allow_reversing = profile.at("allow_reversing").get<bool>();
    wire.allow_standalone_rotate = profile.at("allow_standalone_rotate").get<bool>();
    for (const auto & crossing_source : segment_source.at("planned_crossings")) {
      trm::msg::CollectionPlannedCrossing crossing;
      crossing.ball_id = crossing_source.at("ball_id").get<std::string>();
      crossing.position_x_m = crossing_source.at("position_x_m").get<double>();
      crossing.position_y_m = crossing_source.at("position_y_m").get<double>();
      crossing.progress_s = crossing_source.at("progress_s").get<double>();
      crossing.heading_rad = crossing_source.at("heading_rad").get<double>();
      crossing.predicted_lateral_error = crossing_source.at("predicted_lateral_error").get<double>();
      segment.planned_crossings.push_back(crossing);
    }
    context.segments.push_back(segment);
  }
  return context;
}

}  // namespace

class CollectionExecutionContextParityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (std::getenv("COLLECTION_PARITY_FIXTURE") == nullptr) {
      GTEST_SKIP() << "COLLECTION_PARITY_FIXTURE is not set; run the parity harness";
    }
    fixture_ = load_fixture();
    controller_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("collection_parity_controller");
    client_node_ = std::make_shared<rclcpp::Node>("collection_parity_client");
    executor_.add_node(controller_node_->get_node_base_interface());
    executor_.add_node(client_node_);
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    controller_ = plugin_loader_->createSharedInstance(
      "tennis_robot_collection_controller::CollectionNav2Controller");
    ASSERT_NE(controller_, nullptr);
    controller_->configure(controller_node_, "collection", nullptr, nullptr);
    controller_->activate();
    load_ = client_node_->create_client<trm::srv::LoadCollectionExecutionContext>(
      "collection/load_collection_execution_context");
    ASSERT_TRUE(load_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override
  {
    if (controller_) {
      controller_->deactivate();
      controller_->cleanup();
    }
    controller_.reset();
    plugin_loader_.reset();
    if (client_node_) {
      executor_.remove_node(client_node_);
    }
    if (controller_node_) {
      executor_.remove_node(controller_node_->get_node_base_interface());
    }
    client_node_.reset();
    controller_node_.reset();
  }

  trm::srv::LoadCollectionExecutionContext::Response::SharedPtr call_load(
    const trm::srv::LoadCollectionExecutionContext::Request::SharedPtr & request)
  {
    auto future = load_->async_send_request(request);
    for (int attempt = 0; attempt < 500 &&
      future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++attempt)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    return future.get();
  }

  json fixture_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> plugin_loader_;
  std::shared_ptr<nav2_core::Controller> controller_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr controller_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<trm::srv::LoadCollectionExecutionContext>::SharedPtr load_;
};

TEST_F(CollectionExecutionContextParityTest, PythonSha256MatchesCppCanonicalization)
{
  const auto path = build_path(fixture_);
  const auto expected = fixture_.at("path_sha256").get<std::string>();
  const auto actual = tc::collection_path_sha256_v1(path);
  EXPECT_EQ(actual, expected);
}

TEST_F(CollectionExecutionContextParityTest, RealControllerAcceptsPythonContextAndPath)
{
  auto request = std::make_shared<trm::srv::LoadCollectionExecutionContext::Request>();
  request->context = build_context(fixture_);
  const auto response = call_load(request);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->accepted)
    << "Load rejected: code=" << static_cast<int>(response->rejection_code)
    << " detail=" << response->detail;
  EXPECT_EQ(response->rejection_code, trm::srv::LoadCollectionExecutionContext::Response::ACCEPTED);

  // setPlan must accept the matching path (sha match + make_tracking_plan).
  EXPECT_NO_THROW(controller_->setPlan(build_path(fixture_)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
