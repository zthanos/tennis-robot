#include "tennis_robot_collection_controller/collection_nav2_controller.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include "pluginlib/class_list_macros.hpp"
#include "tennis_robot_collection_controller/collection_path_canonicalization.hpp"

namespace tennis_robot_collection_controller
{
namespace
{

constexpr char kContextSchemaVersion[] = "collection-execution-context/v1";

bool finite(const double value) { return std::isfinite(value); }

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return finite(quaternion.x) && finite(quaternion.y) && finite(quaternion.z) &&
    finite(quaternion.w) && std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w) > 0.0;
}

bool valid_profile(const tennis_robot_msgs::msg::CollectionExecutionProfile & profile)
{
  return finite(profile.nominal_speed_mps) && finite(profile.min_speed_mps) &&
    finite(profile.max_speed_mps) && finite(profile.nominal_speed_warning_tolerance_mps) &&
    finite(profile.max_acceleration_mps2) && finite(profile.max_deceleration_mps2) &&
    finite(profile.required_entry_m) && finite(profile.required_run_in_m) &&
    finite(profile.required_run_out_m) && finite(profile.max_curvature_per_m) &&
    finite(profile.max_lateral_error_m) && finite(profile.max_heading_error_rad) &&
    profile.min_speed_mps > 0.0 && profile.nominal_speed_mps >= profile.min_speed_mps &&
    profile.nominal_speed_mps <= profile.max_speed_mps &&
    profile.nominal_speed_warning_tolerance_mps >= 0.0 && profile.max_acceleration_mps2 > 0.0 &&
    profile.max_deceleration_mps2 > 0.0 && profile.required_entry_m >= 0.0 &&
    profile.required_run_in_m >= 0.0 && profile.required_run_out_m >= 0.0 &&
    profile.max_curvature_per_m > 0.0 && profile.max_lateral_error_m >= 0.0 &&
    profile.max_heading_error_rad >= 0.0 && !profile.allow_reversing &&
    !profile.allow_standalone_rotate;
}

bool valid_tuning(const tennis_robot_msgs::msg::CollectionControllerTuning & tuning)
{
  return finite(tuning.lookahead_distance_m) && tuning.lookahead_distance_m > 0.0 &&
    finite(tuning.max_angular_velocity_rad_s) && tuning.max_angular_velocity_rad_s > 0.0 &&
    finite(tuning.progress_projection_window_m) && tuning.progress_projection_window_m > 0.0 &&
    finite(tuning.crossing_speed_window_m) && tuning.crossing_speed_window_m > 0.0 &&
    finite(tuning.terminal_progress_tolerance_m) && tuning.terminal_progress_tolerance_m > 0.0;
}

bool valid_load_context(const tennis_robot_msgs::msg::CollectionExecutionContext & context, std::string & detail)
{
  if (!valid_tuning(context.controller_tuning) || !finite(context.terminal_progress_s) ||
    context.terminal_progress_s < 0.0 || !finite(context.terminal_pose.position.x) ||
    !finite(context.terminal_pose.position.y) || !finite(context.terminal_pose.position.z) ||
    !valid_quaternion(context.terminal_pose.orientation) || context.segments.empty())
  {
    detail = "invalid_tuning_or_terminal";
    return false;
  }
  try {
    const auto json = nlohmann::json::parse(context.configuration_snapshot_json);
    if (json.dump() != context.configuration_snapshot_json) {
      detail = "configuration_snapshot_not_canonical";
      return false;
    }
  } catch (const nlohmann::json::exception &) {
    detail = "configuration_snapshot_invalid";
    return false;
  }
  double previous_end = 0.0;
  for (std::size_t index = 0; index < context.segments.size(); ++index) {
    const auto & segment = context.segments[index];
    if (segment.segment_id.empty() || !finite(segment.progress_start_s) ||
      !finite(segment.progress_end_s) || segment.progress_end_s <= segment.progress_start_s ||
      !valid_profile(segment.execution_profile) ||
      (index == 0U && std::abs(segment.progress_start_s) > context.controller_tuning.terminal_progress_tolerance_m) ||
      (index > 0U && std::abs(segment.progress_start_s - previous_end) > context.controller_tuning.terminal_progress_tolerance_m))
    {
      detail = "invalid_segment";
      return false;
    }
    double previous_crossing = -1.0;
    for (const auto & crossing : segment.planned_crossings) {
      if (crossing.ball_id.empty() || !finite(crossing.position_x_m) || !finite(crossing.position_y_m) ||
        !finite(crossing.progress_s) || !finite(crossing.heading_rad) ||
        !finite(crossing.predicted_lateral_error) || crossing.progress_s <= previous_crossing ||
        crossing.progress_s < segment.progress_start_s || crossing.progress_s > segment.progress_end_s)
      {
        detail = "invalid_crossing";
        return false;
      }
      previous_crossing = crossing.progress_s;
    }
    previous_end = segment.progress_end_s;
  }
  if (std::abs(previous_end - context.terminal_progress_s) > context.controller_tuning.terminal_progress_tolerance_m) {
    detail = "terminal_progress_inconsistent";
    return false;
  }
  return true;
}

TrackingExecutionProfile to_tracking_profile(const tennis_robot_msgs::msg::CollectionExecutionProfile & profile)
{
  return {profile.nominal_speed_mps, profile.min_speed_mps, profile.max_speed_mps,
    profile.nominal_speed_warning_tolerance_mps, profile.required_run_in_m,
    profile.required_run_out_m, profile.max_curvature_per_m, profile.max_lateral_error_m,
    profile.max_heading_error_rad, profile.allow_reversing, profile.allow_standalone_rotate};
}

CollectionTrackingPlan make_tracking_plan(const tennis_robot_msgs::msg::CollectionExecutionContext & context,
  const nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 2U) { throw std::invalid_argument("path_too_short"); }
  CollectionTrackingPlan plan;
  plan.tuning = {context.controller_tuning.lookahead_distance_m,
    context.controller_tuning.max_angular_velocity_rad_s,
    context.controller_tuning.progress_projection_window_m,
    context.controller_tuning.crossing_speed_window_m,
    context.controller_tuning.terminal_progress_tolerance_m};
  double progress = 0.0;
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & pose = path.poses[index].pose;
    if (!finite(pose.position.x) || !finite(pose.position.y) || !finite(pose.position.z) ||
      !valid_quaternion(pose.orientation))
    {
      throw std::invalid_argument("path_pose_invalid");
    }
    if (index > 0U) {
      const auto & previous = path.poses[index - 1U].pose.position;
      const double step = std::hypot(pose.position.x - previous.x, pose.position.y - previous.y);
      if (step <= 0.0) { throw std::invalid_argument("path_progress_invalid"); }
      progress += step;
    }
    plan.path.push_back({pose.position.x, pose.position.y, progress,
      yaw_from_quaternion(pose.orientation)});
  }
  if (std::abs(progress - context.terminal_progress_s) > context.controller_tuning.terminal_progress_tolerance_m) {
    throw std::invalid_argument("path_terminal_progress_mismatch");
  }
  const auto & terminal = path.poses.back().pose.position;
  if (std::hypot(terminal.x - context.terminal_pose.position.x,
      terminal.y - context.terminal_pose.position.y,
      terminal.z - context.terminal_pose.position.z) > context.controller_tuning.terminal_progress_tolerance_m)
  {
    throw std::invalid_argument("path_terminal_pose_mismatch");
  }
  plan.terminal_progress_s = context.terminal_progress_s;
  for (const auto & wire_segment : context.segments) {
    TrackingSegment segment{wire_segment.segment_id, wire_segment.progress_start_s,
      wire_segment.progress_end_s, to_tracking_profile(wire_segment.execution_profile), {}};
    for (const auto & crossing : wire_segment.planned_crossings) {
      segment.planned_crossings.push_back({crossing.ball_id, crossing.progress_s});
    }
    plan.segments.push_back(std::move(segment));
  }
  return plan;
}

std::uint8_t lifecycle_code(const ContextLifecycle lifecycle)
{
  return static_cast<std::uint8_t>(lifecycle);
}

std::uint8_t failure_code(const TrackingFailureCode code)
{
  using State = tennis_robot_msgs::msg::CollectionControllerState;
  switch (code) {
    case TrackingFailureCode::kNone: return State::FAILURE_NONE;
    case TrackingFailureCode::kProfileUnenforceable: return State::FAILURE_PROFILE_UNENFORCEABLE;
    case TrackingFailureCode::kSpeedBelowMin: return State::FAILURE_SPEED_BELOW_MIN;
    case TrackingFailureCode::kSpeedAboveMax: return State::FAILURE_SPEED_ABOVE_MAX;
    case TrackingFailureCode::kRunInInsufficient: return State::FAILURE_RUN_IN_INSUFFICIENT;
    case TrackingFailureCode::kRunOutInsufficient: return State::FAILURE_RUN_OUT_INSUFFICIENT;
    case TrackingFailureCode::kCurvatureExceeded: return State::FAILURE_CURVATURE_EXCEEDED;
    case TrackingFailureCode::kTrajectoryTubeExceeded: return State::FAILURE_TRAJECTORY_TUBE_EXCEEDED;
    case TrackingFailureCode::kNonMonotonicProgress: return State::FAILURE_NON_MONOTONIC_PROGRESS;
    case TrackingFailureCode::kHeadingErrorExceeded: return State::FAILURE_HEADING_ERROR_EXCEEDED;
    case TrackingFailureCode::kReverseRequired: return State::FAILURE_REVERSE_REQUIRED;
    case TrackingFailureCode::kStandaloneRotateRequired: return State::FAILURE_STANDALONE_ROTATE_REQUIRED;
  }
  return State::FAILURE_PROFILE_UNENFORCEABLE;
}

}  // namespace

void CollectionNav2Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = parent.lock();
  if (!node_) { throw std::runtime_error("collection controller parent node expired"); }
  name_ = std::move(name);
  context_ = std::make_unique<CollectionExecutionContextContract>(kContextSchemaVersion);
  state_publisher_ = node_->create_publisher<tennis_robot_msgs::msg::CollectionControllerState>(name_ + "/state", 10);
  load_service_ = node_->create_service<LoadService>(name_ + "/load_collection_execution_context",
    std::bind(&CollectionNav2Controller::handle_load, this, std::placeholders::_1, std::placeholders::_2));
  reset_service_ = node_->create_service<ResetService>(name_ + "/reset_collection_execution_context",
    std::bind(&CollectionNav2Controller::handle_reset, this, std::placeholders::_1, std::placeholders::_2));
  hold_service_ = node_->create_service<HoldService>(name_ + "/set_collection_safety_hold",
    std::bind(&CollectionNav2Controller::handle_hold, this, std::placeholders::_1, std::placeholders::_2));
  finalize_service_ = node_->create_service<FinalizeService>(name_ + "/finalize_collection_execution_context",
    std::bind(&CollectionNav2Controller::handle_finalize, this, std::placeholders::_1, std::placeholders::_2));
}

void CollectionNav2Controller::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false; tracking_core_.reset(); wire_context_.reset(); state_publisher_.reset();
  load_service_.reset(); reset_service_.reset(); hold_service_.reset(); finalize_service_.reset();
  context_.reset(); node_.reset(); name_.clear(); has_last_core_result_ = false; terminal_ready_ = false;
}

void CollectionNav2Controller::activate() { std::lock_guard<std::mutex> lock(mutex_); active_ = true; }
void CollectionNav2Controller::deactivate() { std::lock_guard<std::mutex> lock(mutex_); active_ = false; }

void CollectionNav2Controller::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !context_ || !wire_context_) { throw std::runtime_error("collection_controller_no_executing_context"); }
  const auto transition = context_->begin_matching_follow_path(collection_path_sha256_v1(path), node_->now().seconds());
  if (!transition.accepted) { throw std::runtime_error("collection_controller_path_context_failure"); }
  try {
    tracking_core_ = std::make_unique<CollectionTrackingCore>(make_tracking_plan(*wire_context_, path));
    has_last_core_result_ = false; terminal_ready_ = false;
    publish_lifecycle_state();
  } catch (const std::exception &) {
    TrackingResult failed; failed.progress_s = 0.0;
    consume_with_failure(TrackingFailureCode::kProfileUnenforceable, failed);
    throw std::runtime_error("collection_controller_profile_unenforceable");
  }
}

geometry_msgs::msg::TwistStamped CollectionNav2Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker *)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !context_ || !tracking_core_) { throw std::runtime_error("collection_controller_no_executing_context"); }
  const bool held = context_->lifecycle() == ContextLifecycle::kSafetyPaused;
  if (!held && context_->lifecycle() != ContextLifecycle::kExecuting) {
    throw std::runtime_error("collection_controller_no_executing_context");
  }
  const TrackingResult result = tracking_core_->update({pose.pose.position.x, pose.pose.position.y,
    yaw_from_quaternion(pose.pose.orientation), std::hypot(velocity.linear.x, velocity.linear.y), held});
  last_core_result_ = result; has_last_core_result_ = true; terminal_ready_ = result.terminal_ready;
  publish_state(result, result.failure);
  if (result.status == TrackingStatus::kSafetyHold || result.status == TrackingStatus::kCompleted) { return zero_command(); }
  if (result.status == TrackingStatus::kFailed) {
    RCLCPP_ERROR(node_->get_logger(),
      "collection_controller_profile_failure: code=%u progress_m=%.3f lateral_error_m=%.3f heading_error_rad=%.3f speed_mps=%.3f",
      static_cast<unsigned int>(failure_code(result.failure)), result.progress_s,
      result.lateral_error_m, result.heading_error_rad,
      std::hypot(velocity.linear.x, velocity.linear.y));
    consume_with_failure(result.failure, result);
    throw std::runtime_error("collection_controller_profile_failure");
  }
  auto command = zero_command();
  command.twist.linear.x = result.command.linear_x_mps;
  command.twist.angular.z = result.command.angular_z_rad_s;
  return command;
}

void CollectionNav2Controller::setSpeedLimit(const double &, const bool &) {}

void CollectionNav2Controller::handle_load(const std::shared_ptr<LoadService::Request> request,
  std::shared_ptr<LoadService::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::string detail;
  if (!valid_load_context(request->context, detail)) {
    response->accepted = false; response->rejection_code = LoadService::Response::INVALID_CONTEXT; response->detail = detail; return;
  }
  const auto & wire = request->context;
  const auto transition = context_->load({wire.context_schema_version, wire.plan_id, wire.path_sha256,
    wire.context_activation_timeout_s}, node_->now().seconds());
  response->accepted = transition.accepted;
  if (transition.accepted) { wire_context_ = wire; tracking_core_.reset(); terminal_ready_ = false; response->rejection_code = LoadService::Response::ACCEPTED; response->detail = "accepted"; publish_lifecycle_state(); }
  else if (transition.rejection == ContextRejectionCode::kUnsupportedSchema) { response->rejection_code = LoadService::Response::UNSUPPORTED_SCHEMA; response->detail = "unsupported_schema"; }
  else if (transition.rejection == ContextRejectionCode::kPathHashInvalid) { response->rejection_code = LoadService::Response::PATH_HASH_INVALID; response->detail = "path_hash_invalid"; }
  else if (transition.rejection == ContextRejectionCode::kContextAlreadyConsumed) { response->rejection_code = LoadService::Response::CONTEXT_ALREADY_CONSUMED; response->detail = "context_already_consumed"; }
  else if (transition.rejection == ContextRejectionCode::kControllerNotIdle) { response->rejection_code = LoadService::Response::CONTROLLER_NOT_IDLE; response->detail = "controller_not_idle"; }
  else { response->rejection_code = LoadService::Response::INVALID_CONTEXT; response->detail = "invalid_context"; }
}

void CollectionNav2Controller::handle_reset(const std::shared_ptr<ResetService::Request>, std::shared_ptr<ResetService::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_); const auto transition = context_->reset(); response->accepted = transition.accepted;
  response->rejection_code = transition.accepted ? ResetService::Response::ACCEPTED : ResetService::Response::INVALID_LIFECYCLE;
  response->detail = transition.accepted ? "accepted" : "invalid_lifecycle";
  if (transition.accepted) { tracking_core_.reset(); wire_context_.reset(); has_last_core_result_ = false; terminal_ready_ = false; publish_lifecycle_state(); }
}

void CollectionNav2Controller::handle_hold(const std::shared_ptr<HoldService::Request> request, std::shared_ptr<HoldService::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto transition = context_->set_safety_hold(request->plan_id, request->path_sha256, request->hold, true);
  response->accepted = transition.accepted;
  if (transition.accepted) { response->rejection_code = HoldService::Response::ACCEPTED; response->detail = "accepted"; publish_lifecycle_state(); }
  else if (transition.rejection == ContextRejectionCode::kMissingContext) { response->rejection_code = HoldService::Response::MISSING_CONTEXT; response->detail = "missing_context"; }
  else if (transition.rejection == ContextRejectionCode::kPlanHashMismatch) { response->rejection_code = HoldService::Response::PLAN_HASH_MISMATCH; response->detail = "plan_hash_mismatch"; }
  else if (transition.rejection == ContextRejectionCode::kResumeContractFailed) { response->rejection_code = HoldService::Response::RESUME_CONTRACT_FAILED; response->detail = "resume_contract_failed"; }
  else { response->rejection_code = HoldService::Response::INVALID_LIFECYCLE; response->detail = "invalid_lifecycle"; }
}

void CollectionNav2Controller::handle_finalize(const std::shared_ptr<FinalizeService::Request> request,
  std::shared_ptr<FinalizeService::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool terminal_valid = wire_context_ && terminal_ready_ && has_last_core_result_ &&
    std::abs(last_core_result_.progress_s - wire_context_->terminal_progress_s) <= wire_context_->controller_tuning.terminal_progress_tolerance_m &&
    last_core_result_.failure == TrackingFailureCode::kNone;
  const auto transition = context_->finalize(request->plan_id, request->path_sha256, request->action_outcome, terminal_valid);
  response->accepted = transition.accepted;
  if (transition.accepted) { response->rejection_code = FinalizeService::Response::ACCEPTED; response->detail = "accepted"; publish_lifecycle_state(); }
  else if (request->action_outcome > FinalizeService::Request::FAILED) { response->rejection_code = FinalizeService::Response::INVALID_ACTION_OUTCOME; response->detail = "invalid_action_outcome"; }
  else if (transition.rejection == ContextRejectionCode::kTerminalNotReached) { response->rejection_code = FinalizeService::Response::TERMINAL_NOT_REACHED; response->detail = "terminal_not_reached"; }
  else if (transition.rejection == ContextRejectionCode::kMissingContext) { response->rejection_code = FinalizeService::Response::MISSING_CONTEXT; response->detail = "missing_context"; }
  else if (transition.rejection == ContextRejectionCode::kPlanHashMismatch) { response->rejection_code = FinalizeService::Response::PLAN_HASH_MISMATCH; response->detail = "plan_hash_mismatch"; }
  else { response->rejection_code = FinalizeService::Response::INVALID_LIFECYCLE; response->detail = "invalid_lifecycle"; }
}

void CollectionNav2Controller::publish_state(const TrackingResult & result, const TrackingFailureCode failure)
{
  if (!state_publisher_ || !wire_context_) { return; }
  tennis_robot_msgs::msg::CollectionControllerState state;
  state.plan_id = wire_context_->plan_id; state.path_sha256 = wire_context_->path_sha256;
  state.lifecycle_state = lifecycle_code(context_->lifecycle()); state.progress_s = result.progress_s;
  state.measured_speed_mps = has_last_core_result_ ? last_core_result_.crossing_measurement ?
    last_core_result_.crossing_measurement->verdict.measured_speed_mps : 0.0 : 0.0;
  state.lateral_error_m = result.lateral_error_m; state.heading_error_rad = result.heading_error_rad;
  state.failure_reason = failure_code(failure);
  if (result.crossing_measurement) {
    state.has_active_crossing = true; state.active_ball_id = result.crossing_measurement->ball_id;
    state.active_crossing_progress_s = result.crossing_measurement->progress_s;
    const auto & verdict = result.crossing_measurement->verdict;
    state.profile_verdict.hard_compliant = verdict.hard_compliant;
    state.profile_verdict.measured_speed_mps = verdict.measured_speed_mps;
    state.profile_verdict.nominal_speed_error_mps = verdict.nominal_speed_error_mps;
    state.profile_verdict.nominal_tracking = verdict.nominal_tracking == NominalTracking::kDeviated ? 1U : 0U;
    state.profile_verdict.hard_violation_reason = verdict.hard_compliant ? 0U :
      (failure == TrackingFailureCode::kSpeedBelowMin ? 1U : 2U);
  }
  for (const auto & segment : wire_context_->segments) {
    if (result.progress_s >= segment.progress_start_s && result.progress_s <= segment.progress_end_s) {
      state.active_segment_id = segment.segment_id; break;
    }
  }
  state_publisher_->publish(state);
}

void CollectionNav2Controller::publish_lifecycle_state()
{
  if (!state_publisher_ || !context_) { return; }
  tennis_robot_msgs::msg::CollectionControllerState state;
  state.lifecycle_state = lifecycle_code(context_->lifecycle());
  state.failure_reason = tennis_robot_msgs::msg::CollectionControllerState::FAILURE_NONE;
  if (wire_context_) { state.plan_id = wire_context_->plan_id; state.path_sha256 = wire_context_->path_sha256; }
  state_publisher_->publish(state);
}

void CollectionNav2Controller::consume_with_failure(const TrackingFailureCode failure, const TrackingResult & result)
{
  publish_state(result, failure);
  context_->terminal_failure();
  terminal_ready_ = false;
}

geometry_msgs::msg::TwistStamped CollectionNav2Controller::zero_command() const
{
  geometry_msgs::msg::TwistStamped command; command.header.stamp = node_->now(); command.header.frame_id = "base_link"; return command;
}

}  // namespace tennis_robot_collection_controller

PLUGINLIB_EXPORT_CLASS(tennis_robot_collection_controller::CollectionNav2Controller, nav2_core::Controller)
