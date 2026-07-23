#ifndef TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_NAV2_CONTROLLER_HPP_
#define TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_NAV2_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tennis_robot_collection_controller/collection_execution_context_contract.hpp"
#include "tennis_robot_collection_controller/collection_tracking_core.hpp"
#include "tennis_robot_msgs/msg/collection_controller_state.hpp"
#include "tennis_robot_msgs/msg/collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/finalize_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/load_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/reset_collection_execution_context.hpp"
#include "tennis_robot_msgs/srv/set_collection_safety_hold.hpp"

namespace tennis_robot_collection_controller
{

class CollectionNav2Controller : public nav2_core::Controller
{
public:
  CollectionNav2Controller() = default;
  ~CollectionNav2Controller() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using LoadService = tennis_robot_msgs::srv::LoadCollectionExecutionContext;
  using ResetService = tennis_robot_msgs::srv::ResetCollectionExecutionContext;
  using HoldService = tennis_robot_msgs::srv::SetCollectionSafetyHold;
  using FinalizeService = tennis_robot_msgs::srv::FinalizeCollectionExecutionContext;

  void handle_load(const std::shared_ptr<LoadService::Request> request,
                   std::shared_ptr<LoadService::Response> response);
  void handle_reset(const std::shared_ptr<ResetService::Request> request,
                    std::shared_ptr<ResetService::Response> response);
  void handle_hold(const std::shared_ptr<HoldService::Request> request,
                   std::shared_ptr<HoldService::Response> response);
  void handle_finalize(const std::shared_ptr<FinalizeService::Request> request,
                       std::shared_ptr<FinalizeService::Response> response);
  geometry_msgs::msg::TwistStamped zero_command() const;
  void publish_state(const TrackingResult & result, TrackingFailureCode failure);
  void publish_lifecycle_state();
  void consume_with_failure(TrackingFailureCode failure, const TrackingResult & result);

  mutable std::mutex mutex_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::string name_;
  std::unique_ptr<CollectionExecutionContextContract> context_;
  std::optional<tennis_robot_msgs::msg::CollectionExecutionContext> wire_context_;
  std::unique_ptr<CollectionTrackingCore> tracking_core_;
  TrackingResult last_core_result_;
  bool has_last_core_result_{false};
  bool terminal_ready_{false};
  bool active_{false};
  rclcpp::Publisher<tennis_robot_msgs::msg::CollectionControllerState>::SharedPtr state_publisher_;
  rclcpp::Service<LoadService>::SharedPtr load_service_;
  rclcpp::Service<ResetService>::SharedPtr reset_service_;
  rclcpp::Service<HoldService>::SharedPtr hold_service_;
  rclcpp::Service<FinalizeService>::SharedPtr finalize_service_;
};

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_NAV2_CONTROLLER_HPP_
