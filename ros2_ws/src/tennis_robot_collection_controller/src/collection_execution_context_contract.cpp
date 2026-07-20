#include "tennis_robot_collection_controller/collection_execution_context_contract.hpp"

#include <cmath>
#include <regex>
#include <utility>

namespace tennis_robot_collection_controller
{
namespace
{

bool valid_sha256(const std::string & value)
{
  return std::regex_match(value, std::regex("[0-9a-f]{64}"));
}

ContextTransition rejected(ContextRejectionCode code)
{
  return ContextTransition{false, code, false};
}

ContextTransition accepted(bool emit_zero_velocity = false)
{
  return ContextTransition{true, ContextRejectionCode::kAccepted, emit_zero_velocity};
}

}  // namespace

CollectionExecutionContextContract::CollectionExecutionContextContract(
  std::string supported_schema_version)
: supported_schema_version_(std::move(supported_schema_version))
{
}

ContextTransition CollectionExecutionContextContract::load(
  CollectionExecutionContextIdentity context, double monotonic_now_s)
{
  if (lifecycle_ == ContextLifecycle::kConsumed) {
    return rejected(ContextRejectionCode::kContextAlreadyConsumed);
  }
  if (lifecycle_ != ContextLifecycle::kIdle) {
    return rejected(ContextRejectionCode::kControllerNotIdle);
  }
  if (!std::isfinite(monotonic_now_s) || context.plan_id.empty() ||
    !std::isfinite(context.context_activation_timeout_s) ||
    context.context_activation_timeout_s <= 0.0)
  {
    return rejected(ContextRejectionCode::kInvalidContext);
  }
  if (context.context_schema_version != supported_schema_version_) {
    return rejected(ContextRejectionCode::kUnsupportedSchema);
  }
  if (!valid_sha256(context.path_sha256)) {
    return rejected(ContextRejectionCode::kPathHashInvalid);
  }
  context_ = std::move(context);
  has_context_ = true;
  loaded_at_s_ = monotonic_now_s;
  lifecycle_ = ContextLifecycle::kContextLoaded;
  return accepted();
}

ContextTransition CollectionExecutionContextContract::begin_matching_follow_path(
  const std::string & path_sha256, double monotonic_now_s)
{
  if (lifecycle_ != ContextLifecycle::kContextLoaded || !has_context_) {
    return rejected(ContextRejectionCode::kInvalidLifecycle);
  }
  if (activation_expired(monotonic_now_s)) {
    has_context_ = false;
    lifecycle_ = ContextLifecycle::kIdle;
    return rejected(ContextRejectionCode::kContextActivationTimeout);
  }
  if (path_sha256 != context_.path_sha256) {
    return rejected(ContextRejectionCode::kPlanHashMismatch);
  }
  lifecycle_ = ContextLifecycle::kValidatingFollowPath;
  lifecycle_ = ContextLifecycle::kExecuting;
  return accepted();
}

ContextTransition CollectionExecutionContextContract::set_safety_hold(
  const std::string & plan_id, const std::string & path_sha256, bool hold,
  bool resume_contract_valid)
{
  if (!has_context_) {
    return rejected(ContextRejectionCode::kMissingContext);
  }
  if (plan_id != context_.plan_id || path_sha256 != context_.path_sha256) {
    return rejected(ContextRejectionCode::kPlanHashMismatch);
  }
  if (hold && lifecycle_ == ContextLifecycle::kExecuting) {
    lifecycle_ = ContextLifecycle::kSafetyPaused;
    return accepted(true);
  }
  if (!hold && lifecycle_ == ContextLifecycle::kSafetyPaused) {
    if (!resume_contract_valid) {
      return rejected(ContextRejectionCode::kResumeContractFailed);
    }
    lifecycle_ = ContextLifecycle::kExecuting;
    return accepted();
  }
  return rejected(ContextRejectionCode::kInvalidLifecycle);
}

ContextTransition CollectionExecutionContextContract::terminal_success()
{
  if (lifecycle_ != ContextLifecycle::kExecuting) {
    return rejected(ContextRejectionCode::kInvalidLifecycle);
  }
  lifecycle_ = ContextLifecycle::kSucceeded;
  lifecycle_ = ContextLifecycle::kConsumed;
  return accepted(true);
}

ContextTransition CollectionExecutionContextContract::terminal_failure()
{
  if (lifecycle_ != ContextLifecycle::kExecuting && lifecycle_ != ContextLifecycle::kSafetyPaused) {
    return rejected(ContextRejectionCode::kInvalidLifecycle);
  }
  lifecycle_ = ContextLifecycle::kFailed;
  lifecycle_ = ContextLifecycle::kConsumed;
  return accepted(true);
}

ContextTransition CollectionExecutionContextContract::finalize(
  const std::string & plan_id, const std::string & path_sha256,
  std::uint8_t action_outcome, const bool terminal_ready)
{
  if (!has_context_) {
    return rejected(ContextRejectionCode::kMissingContext);
  }
  if (plan_id != context_.plan_id || path_sha256 != context_.path_sha256) {
    return rejected(ContextRejectionCode::kPlanHashMismatch);
  }
  if (lifecycle_ != ContextLifecycle::kExecuting) {
    return rejected(ContextRejectionCode::kInvalidLifecycle);
  }
  if (action_outcome > 2U) {
    return rejected(ContextRejectionCode::kInvalidContext);
  }
  if (action_outcome == 0U && !terminal_ready) {
    return rejected(ContextRejectionCode::kTerminalNotReached);
  }
  lifecycle_ = action_outcome == 0U ? ContextLifecycle::kSucceeded : ContextLifecycle::kFailed;
  lifecycle_ = ContextLifecycle::kConsumed;
  return accepted(true);
}

ContextTransition CollectionExecutionContextContract::reset()
{
  if (lifecycle_ == ContextLifecycle::kExecuting || lifecycle_ == ContextLifecycle::kSafetyPaused) {
    return rejected(ContextRejectionCode::kInvalidLifecycle);
  }
  has_context_ = false;
  context_ = CollectionExecutionContextIdentity{};
  lifecycle_ = ContextLifecycle::kIdle;
  return accepted();
}

ContextLifecycle CollectionExecutionContextContract::lifecycle() const
{
  return lifecycle_;
}

const CollectionExecutionContextIdentity * CollectionExecutionContextContract::context() const
{
  return has_context_ ? &context_ : nullptr;
}

bool CollectionExecutionContextContract::activation_expired(double monotonic_now_s) const
{
  return !std::isfinite(monotonic_now_s) || monotonic_now_s - loaded_at_s_ >= context_.context_activation_timeout_s;
}

}  // namespace tennis_robot_collection_controller
