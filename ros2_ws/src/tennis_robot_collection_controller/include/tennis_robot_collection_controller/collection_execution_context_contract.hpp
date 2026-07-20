#ifndef TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_EXECUTION_CONTEXT_CONTRACT_HPP_
#define TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_EXECUTION_CONTEXT_CONTRACT_HPP_

#include <cstdint>
#include <string>

namespace tennis_robot_collection_controller
{

enum class ContextLifecycle
{
  kIdle,
  kContextLoaded,
  kValidatingFollowPath,
  kExecuting,
  kSafetyPaused,
  kSucceeded,
  kFailed,
  kConsumed,
};

enum class ContextRejectionCode
{
  kAccepted,
  kControllerNotIdle,
  kInvalidContext,
  kUnsupportedSchema,
  kPathHashInvalid,
  kContextAlreadyConsumed,
  kContextActivationTimeout,
  kMissingContext,
  kPlanHashMismatch,
  kInvalidLifecycle,
  kResumeContractFailed,
  kTerminalNotReached,
};

struct CollectionExecutionContextIdentity
{
  std::string context_schema_version;
  std::string plan_id;
  std::string path_sha256;
  double context_activation_timeout_s{};
};

struct ContextTransition
{
  bool accepted{};
  ContextRejectionCode rejection{ContextRejectionCode::kAccepted};
  bool emit_zero_velocity{};
};

/// Pure C0 lifecycle contract; this is not a nav2_core::Controller plugin.
class CollectionExecutionContextContract
{
public:
  explicit CollectionExecutionContextContract(std::string supported_schema_version);

  ContextTransition load(CollectionExecutionContextIdentity context, double monotonic_now_s);
  ContextTransition begin_matching_follow_path(const std::string & path_sha256, double monotonic_now_s);
  ContextTransition set_safety_hold(
    const std::string & plan_id, const std::string & path_sha256, bool hold,
    bool resume_contract_valid);
  ContextTransition terminal_success();
  ContextTransition terminal_failure();
  ContextTransition finalize(
    const std::string & plan_id, const std::string & path_sha256,
    std::uint8_t action_outcome, bool terminal_ready);
  ContextTransition reset();

  ContextLifecycle lifecycle() const;
  const CollectionExecutionContextIdentity * context() const;

private:
  bool activation_expired(double monotonic_now_s) const;

  std::string supported_schema_version_;
  ContextLifecycle lifecycle_{ContextLifecycle::kIdle};
  CollectionExecutionContextIdentity context_{};
  bool has_context_{false};
  double loaded_at_s_{};
};

}  // namespace tennis_robot_collection_controller

#endif  // TENNIS_ROBOT_COLLECTION_CONTROLLER__COLLECTION_EXECUTION_CONTEXT_CONTRACT_HPP_
