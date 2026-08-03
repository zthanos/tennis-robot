#ifndef TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PROGRESS_GOAL_CHECKER_HPP_
#define TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PROGRESS_GOAL_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tennis_robot_msgs/msg/collection_controller_state.hpp"

namespace tennis_robot_collection_controller
{

class CollectionProgressGoalChecker : public nav2_core::GoalChecker
{
public:
  CollectionProgressGoalChecker() = default;
  ~CollectionProgressGoalChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  using ControllerState = tennis_robot_msgs::msg::CollectionControllerState;

  void state_callback(const ControllerState::SharedPtr state);

  mutable std::mutex mutex_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<ControllerState>::SharedPtr state_subscription_;
  std::optional<ControllerState> state_;
  rclcpp::Time state_received_at_;
  double xy_goal_tolerance_{0.30};
  double yaw_goal_tolerance_{3.14};
  double progress_tolerance_m_{0.30};
  double state_timeout_s_{0.50};
};

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_PROGRESS_GOAL_CHECKER_HPP_
