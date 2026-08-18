#include "tennis_robot_collection_controller/collection_progress_goal_checker.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace tennis_robot_collection_controller
{
namespace
{

template<typename T>
T parameter(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.0;
}

double yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}  // namespace

void CollectionProgressGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("collection progress goal checker parent node expired");
  }

  const auto prefix = plugin_name + ".";
  xy_goal_tolerance_ = parameter(node_, prefix + "xy_goal_tolerance", 0.30);
  yaw_goal_tolerance_ = parameter(node_, prefix + "yaw_goal_tolerance", 3.14);
  progress_tolerance_m_ = parameter(node_, prefix + "progress_tolerance_m", 0.30);
  state_timeout_s_ = parameter(node_, prefix + "state_timeout_s", 0.50);
  const auto state_topic = parameter(
    node_, prefix + "controller_state_topic", std::string("CollectionFollowPath/state"));

  if (!std::isfinite(xy_goal_tolerance_) || xy_goal_tolerance_ <= 0.0 ||
    !std::isfinite(yaw_goal_tolerance_) || yaw_goal_tolerance_ < 0.0 ||
    !std::isfinite(progress_tolerance_m_) || progress_tolerance_m_ <= 0.0 ||
    !std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0 || state_topic.empty())
  {
    throw std::runtime_error("collection progress goal checker parameters are invalid");
  }

  state_subscription_ = node_->create_subscription<ControllerState>(
    state_topic, rclcpp::QoS(10),
    std::bind(&CollectionProgressGoalChecker::state_callback, this, std::placeholders::_1));
  reset();
}

void CollectionProgressGoalChecker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.reset();
  state_received_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void CollectionProgressGoalChecker::state_callback(const ControllerState::SharedPtr state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = *state;
  state_received_at_ = node_->now();
}

bool CollectionProgressGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  ControllerState state;
  rclcpp::Time received_at(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_) {
      return false;
    }
    state = *state_;
    received_at = state_received_at_;
  }

  const double state_age_s = (node_->now() - received_at).seconds();
  if (!std::isfinite(state_age_s) || state_age_s < 0.0 || state_age_s > state_timeout_s_ ||
    state.lifecycle_state != ControllerState::EXECUTING ||
    state.failure_reason != ControllerState::FAILURE_NONE ||
    state.plan_id.empty() || state.path_sha256.empty() ||
    !state.terminal_ready || !std::isfinite(state.progress_s) || state.progress_s < 0.0 ||
    !std::isfinite(state.terminal_progress_s) || state.terminal_progress_s < 0.0 ||
    state.progress_s + progress_tolerance_m_ < state.terminal_progress_s ||
    !finite_pose(query_pose) || !finite_pose(goal_pose))
  {
    return false;
  }

  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;
  if (dx * dx + dy * dy > xy_goal_tolerance_ * xy_goal_tolerance_) {
    return false;
  }

  const double yaw_error = std::atan2(
    std::sin(yaw(query_pose.orientation) - yaw(goal_pose.orientation)),
    std::cos(yaw(query_pose.orientation) - yaw(goal_pose.orientation)));
  return std::abs(yaw_error) <= yaw_goal_tolerance_;
}

bool CollectionProgressGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double unset = std::numeric_limits<double>::lowest();
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = unset;
  pose_tolerance.orientation.x = unset;
  pose_tolerance.orientation.y = unset;
  pose_tolerance.orientation.z = yaw_goal_tolerance_;
  pose_tolerance.orientation.w = unset;
  vel_tolerance.linear.x = unset;
  vel_tolerance.linear.y = unset;
  vel_tolerance.linear.z = unset;
  vel_tolerance.angular.x = unset;
  vel_tolerance.angular.y = unset;
  vel_tolerance.angular.z = unset;
  return true;
}

}  // namespace tennis_robot_collection_controller

PLUGINLIB_EXPORT_CLASS(
  tennis_robot_collection_controller::CollectionProgressGoalChecker,
  nav2_core::GoalChecker)
