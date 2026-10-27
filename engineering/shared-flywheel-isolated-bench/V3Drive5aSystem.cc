#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/msgs/wrench.pb.h>
#include <sdf/Element.hh>

namespace phase2_v3
{
class Drive5aSystem final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->outputCsv = _sdf->Get<std::string>("output_csv", "/tmp/v3_5a.csv").first;
    this->caseName = _sdf->Get<std::string>("case_name", "unknown").first;
    this->groundMu = _sdf->Get<double>("ground_mu", 0.7).first;
    this->wheelRadius = _sdf->Get<double>("wheel_radius", 0.085).first;
    this->halfTrack = _sdf->Get<double>("half_track", 0.297).first;
    this->torqueLimit = _sdf->Get<double>("torque_limit", 3.73).first;
    this->kp = _sdf->Get<double>("speed_kp", 0.8).first;
    this->ki = _sdf->Get<double>("speed_ki", 0.35).first;
    this->samplePeriod = _sdf->Get<double>("sample_period", 0.01).first;
    this->turnsOnly = _sdf->Get<bool>("turns_only", false).first;
    this->retentionOnly = _sdf->Get<bool>("retention_only", false).first;
    this->yawFeedbackEnabled = _sdf->Get<bool>("yaw_feedback_enabled", false).first;
    this->yawKp = _sdf->Get<double>("yaw_kp", 2.0).first;
    this->yawKi = _sdf->Get<double>("yaw_ki", 0.8).first;
    this->yawCorrectionLimit = _sdf->Get<double>("yaw_correction_limit", 1.5).first;
    this->tiltHoldEnabled = _sdf->Get<bool>("tilt_hold_enabled", false).first;
    this->tiltTarget = _sdf->Get<double>("tilt_target_rad", -0.174532925199).first;
    this->tiltKp = _sdf->Get<double>("tilt_hold_kp", 80.0).first;
    this->tiltKd = _sdf->Get<double>("tilt_hold_kd", 8.0).first;
    this->tiltTorqueLimit = _sdf->Get<double>("tilt_torque_limit", 12.6).first;
    this->stream.open(this->outputCsv);
    this->stream << "time_s,scenario,case_name,mu,target_v_m_s,target_w_rad_s,"
      "x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,vx_body_m_s,vy_body_m_s,wz_body_rad_s,"
      "ax_body_m_s2,ay_body_m_s2,alpha_z_rad_s2,effective_w_rad_s,yaw_error_rad_s";
    for (const auto &name : this->shortNames)
      this->stream << ',' << name << "_speed_rad_s," << name << "_target_rad_s,"
        << name << "_command_nm," << name << "_slip_ratio,"
        << name << "_wrench_force_n," << name << "_wrench_torque_nm,"
        << name << "_wrench_fx_n," << name << "_wrench_fy_n," << name << "_wrench_fz_n,"
        << name << "_wrench_tx_nm," << name << "_wrench_ty_nm," << name << "_wrench_tz_nm,"
        << name << "_centre_z_m";
    this->stream << ",tilt_wrench_force_n,tilt_wrench_torque_nm,"
      "tilt_wrench_fx_n,tilt_wrench_fy_n,tilt_wrench_fz_n,"
      "tilt_wrench_tx_nm,tilt_wrench_ty_nm,tilt_wrench_tz_nm,tilt_position_rad,tilt_velocity_rad_s,tilt_command_nm\n";
    this->Resolve(_ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero())
      return;
    if (!this->Resolve(_ecm))
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const auto command = this->CommandAt(time);
    if (this->tiltHoldEnabled)
    {
      if (!this->tiltInitialized)
      {
        this->tilt.ResetPosition(_ecm, {this->tiltTarget});
        this->tiltInitialized = true;
      }
      const auto tiltPosition = this->tilt.Position(_ecm);
      const auto tiltVelocity = this->tilt.Velocity(_ecm);
      const double position = tiltPosition && !tiltPosition->empty() ? tiltPosition->at(0) : this->tiltTarget;
      const double velocity = tiltVelocity && !tiltVelocity->empty() ? tiltVelocity->at(0) : 0.0;
      this->tiltCommand = std::clamp(this->tiltKp * (this->tiltTarget-position) - this->tiltKd * velocity,
        -this->tiltTorqueLimit, this->tiltTorqueLimit);
      this->tilt.SetForce(_ecm, {this->tiltCommand});
    }
    if (command.name != this->previousScenario)
    {
      this->integrals.fill(0.0);
      this->yawIntegral = 0.0;
      this->previousScenario = command.name;
    }
    double measuredYaw = 0.0;
    const auto pose = this->base.WorldPose(_ecm);
    const auto angularWorld = this->base.WorldAngularVelocity(_ecm);
    if (pose && angularWorld)
      measuredYaw = pose->Rot().Inverse().RotateVector(*angularWorld).Z();
    this->lastYawError = command.w - measuredYaw;
    this->effectiveYawCommand = command.w;
    if (this->yawFeedbackEnabled && std::abs(command.w) > 1e-9)
    {
      this->yawIntegral = std::clamp(this->yawIntegral + this->lastYawError * dt, -1.5, 1.5);
      const double correction = std::clamp(
        this->yawKp * this->lastYawError + this->yawKi * this->yawIntegral,
        -this->yawCorrectionLimit, this->yawCorrectionLimit);
      this->effectiveYawCommand += correction;
    }
    const double leftTarget = (command.v - this->effectiveYawCommand * this->halfTrack) / this->wheelRadius;
    const double rightTarget = (command.v + this->effectiveYawCommand * this->halfTrack) / this->wheelRadius;
    for (std::size_t i = 0; i < this->joints.size(); ++i)
    {
      const auto velocity = this->joints[i].Velocity(_ecm);
      const double actual = velocity && !velocity->empty() ? velocity->at(0) : 0.0;
      const double target = i < 2 ? leftTarget : rightTarget;
      const double error = target - actual;
      this->integrals[i] = std::clamp(this->integrals[i] + error * dt, -3.0, 3.0);
      this->commands[i] = std::clamp(this->kp * error + this->ki * this->integrals[i],
          -this->torqueLimit, this->torqueLimit);
      if (time >= this->EndTime())
        this->commands[i] = 0.0;
      this->joints[i].SetForce(_ecm, {this->commands[i]});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->resolved || this->done)
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    if (time + 1e-12 < this->nextSample)
      return;
    this->nextSample = time + this->samplePeriod;
    const auto pose = this->base.WorldPose(_ecm);
    const auto linearWorld = this->base.WorldLinearVelocity(_ecm);
    const auto angularWorld = this->base.WorldAngularVelocity(_ecm);
    if (!pose || !linearWorld || !angularWorld)
      return;
    const auto linear = pose->Rot().Inverse().RotateVector(*linearWorld);
    const auto angular = pose->Rot().Inverse().RotateVector(*angularWorld);
    double ax = 0.0, ay = 0.0, alpha = 0.0;
    if (this->havePreviousVelocity)
    {
      const double dt = std::max(1e-6, time - this->previousSampleTime);
      ax = (linear.X() - this->previousLinear.X()) / dt;
      ay = (linear.Y() - this->previousLinear.Y()) / dt;
      alpha = (angular.Z() - this->previousAngularZ) / dt;
    }
    this->previousLinear = linear;
    this->previousAngularZ = angular.Z();
    this->previousSampleTime = time;
    this->havePreviousVelocity = true;
    const auto command = this->CommandAt(time);
    const double leftTarget = (command.v - this->effectiveYawCommand * this->halfTrack) / this->wheelRadius;
    const double rightTarget = (command.v + this->effectiveYawCommand * this->halfTrack) / this->wheelRadius;
    const auto euler = pose->Rot().Euler();
    this->stream << time << ',' << command.name << ',' << this->caseName << ',' << this->groundMu
      << ',' << command.v << ',' << command.w << ',' << pose->Pos().X() << ',' << pose->Pos().Y()
      << ',' << pose->Pos().Z() << ',' << euler.X() << ',' << euler.Y() << ',' << euler.Z()
      << ',' << linear.X() << ',' << linear.Y() << ',' << angular.Z()
      << ',' << ax << ',' << ay << ',' << alpha << ',' << this->effectiveYawCommand
      << ',' << this->lastYawError;
    for (std::size_t i = 0; i < this->joints.size(); ++i)
    {
      const auto velocity = this->joints[i].Velocity(_ecm);
      const double actual = velocity && !velocity->empty() ? velocity->at(0) : 0.0;
      const double target = i < 2 ? leftTarget : rightTarget;
      const double contactSpeed = linear.X() - angular.Z() * this->wheelY[i];
      const double surfaceSpeed = this->wheelRadius * actual;
      const double slip = std::abs(surfaceSpeed - contactSpeed) /
        std::max({std::abs(surfaceSpeed), std::abs(contactSpeed), 0.05});
      const auto wrench = this->joints[i].TransmittedWrench(_ecm);
      double forceNorm = 0.0, torqueNorm = 0.0;
      std::array<double, 3> force{0, 0, 0}, torque{0, 0, 0};
      if (wrench && !wrench->empty())
      {
        const auto &value = wrench->at(0);
        forceNorm = std::sqrt(std::pow(value.force().x(), 2) +
          std::pow(value.force().y(), 2) + std::pow(value.force().z(), 2));
        torqueNorm = std::sqrt(std::pow(value.torque().x(), 2) +
          std::pow(value.torque().y(), 2) + std::pow(value.torque().z(), 2));
        force = {value.force().x(), value.force().y(), value.force().z()};
        torque = {value.torque().x(), value.torque().y(), value.torque().z()};
      }
      this->stream << ',' << actual << ',' << target << ',' << this->commands[i]
        << ',' << slip << ',' << forceNorm << ',' << torqueNorm
        << ',' << force[0] << ',' << force[1] << ',' << force[2]
        << ',' << torque[0] << ',' << torque[1] << ',' << torque[2];
      const auto wheelPose = this->wheelLinks[i].WorldPose(_ecm);
      this->stream << ',' << (wheelPose ? wheelPose->Pos().Z() : 0.0);
    }
    double tiltForce = 0.0, tiltTorque = 0.0;
    std::array<double, 3> tiltForceVector{0, 0, 0}, tiltTorqueVector{0, 0, 0};
    const auto tiltWrench = this->tilt.TransmittedWrench(_ecm);
    if (tiltWrench && !tiltWrench->empty())
    {
      const auto &value = tiltWrench->at(0);
      tiltForce = std::sqrt(std::pow(value.force().x(), 2) +
        std::pow(value.force().y(), 2) + std::pow(value.force().z(), 2));
      tiltTorque = std::sqrt(std::pow(value.torque().x(), 2) +
        std::pow(value.torque().y(), 2) + std::pow(value.torque().z(), 2));
      tiltForceVector = {value.force().x(), value.force().y(), value.force().z()};
      tiltTorqueVector = {value.torque().x(), value.torque().y(), value.torque().z()};
    }
    const auto tiltPosition = this->tilt.Position(_ecm);
    const auto tiltVelocity = this->tilt.Velocity(_ecm);
    this->stream << ',' << tiltForce << ',' << tiltTorque
      << ',' << tiltForceVector[0] << ',' << tiltForceVector[1] << ',' << tiltForceVector[2]
      << ',' << tiltTorqueVector[0] << ',' << tiltTorqueVector[1] << ',' << tiltTorqueVector[2]
      << ',' << (tiltPosition && !tiltPosition->empty() ? tiltPosition->at(0) : 0.0)
      << ',' << (tiltVelocity && !tiltVelocity->empty() ? tiltVelocity->at(0) : 0.0)
      << ',' << this->tiltCommand << '\n';
    if (time >= this->EndTime() && !this->done)
    {
      this->stream.flush();
      std::ofstream marker(this->outputCsv + ".done");
      marker << "complete\n";
      this->done = true;
    }
  }

private:
  struct Command { std::string name; double v; double w; };
  Command CommandAt(double t) const
  {
    if (this->retentionOnly)
    {
      if (t < 6.0) return {"settle", 0, 0};
      if (t < 8.0) return {"straight_0p6", 0.6, 0};
      if (t < 10.0) return {"brake_from_0p6", 0, 0};
      if (t < 13.0) return {"moving_turn", 0.6, 0.6};
      return {"final_stop", 0, 0};
    }
    if (this->turnsOnly)
    {
      if (t < 2.0) return {"settle", 0, 0};
      if (t < 7.0) return {"spin_0p5", 0, 0.5};
      if (t < 9.0) return {"stop_after_spin_0p5", 0, 0};
      if (t < 14.0) return {"spin_1p0", 0, 1.0};
      if (t < 16.0) return {"stop_after_spin_1p0", 0, 0};
      if (t < 22.0) return {"moving_turn", 0.4, 0.6};
      return {"final_stop", 0, 0};
    }
    if (t < 2.0) return {"settle", 0, 0};
    if (t < 7.0) return {"straight_0p3", 0.3, 0};
    if (t < 9.0) return {"stop_after_0p3", 0, 0};
    if (t < 14.0) return {"straight_0p6", 0.6, 0};
    if (t < 17.0) return {"brake_from_0p6", 0, 0};
    if (t < 22.0) return {"spin_0p5", 0, 0.5};
    if (t < 24.0) return {"stop_after_spin_0p5", 0, 0};
    if (t < 29.0) return {"spin_1p0", 0, 1.0};
    if (t < 31.0) return {"stop_after_spin_1p0", 0, 0};
    if (t < 37.0) return {"moving_turn", 0.4, 0.6};
    return {"final_stop", 0, 0};
  }

  double EndTime() const { return this->retentionOnly ? 15.0 : (this->turnsOnly ? 24.0 : 40.0); }

  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->resolved)
      return true;
    const std::array<std::string, 4> jointNames{
      "rear_left_wheel_joint", "front_left_wheel_joint",
      "rear_right_wheel_joint", "front_right_wheel_joint"};
    for (std::size_t i = 0; i < jointNames.size(); ++i)
    {
      const auto entity = this->model.JointByName(_ecm, jointNames[i]);
      if (entity == gz::sim::kNullEntity)
        return false;
      this->joints[i] = gz::sim::Joint(entity);
      this->joints[i].EnableVelocityCheck(_ecm);
      this->joints[i].EnableTransmittedWrenchCheck(_ecm);
    }
    const std::array<std::string, 4> linkNames{
      "rear_left_wheel_link", "front_left_wheel_link",
      "rear_right_wheel_link", "front_right_wheel_link"};
    for (std::size_t i = 0; i < linkNames.size(); ++i)
    {
      const auto entity = this->model.LinkByName(_ecm, linkNames[i]);
      if (entity == gz::sim::kNullEntity)
        return false;
      this->wheelLinks[i] = gz::sim::Link(entity);
    }
    const auto tiltEntity = this->model.JointByName(_ecm, "launcher_tilt_joint");
    const auto baseEntity = this->model.LinkByName(_ecm, "base_link");
    if (tiltEntity == gz::sim::kNullEntity || baseEntity == gz::sim::kNullEntity)
      return false;
    this->tilt = gz::sim::Joint(tiltEntity);
    this->tilt.EnablePositionCheck(_ecm);
    this->tilt.EnableVelocityCheck(_ecm);
    this->tilt.EnableTransmittedWrenchCheck(_ecm);
    this->base = gz::sim::Link(baseEntity);
    this->base.EnableVelocityChecks(_ecm);
    this->resolved = true;
    return true;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link base;
  gz::sim::Joint tilt;
  std::array<gz::sim::Joint, 4> joints;
  std::array<gz::sim::Link, 4> wheelLinks;
  const std::array<std::string, 4> shortNames{"rear_left", "front_left", "rear_right", "front_right"};
  const std::array<double, 4> wheelY{0.297, 0.297, -0.297, -0.297};
  std::array<double, 4> integrals{0, 0, 0, 0};
  std::array<double, 4> commands{0, 0, 0, 0};
  std::string outputCsv;
  std::string caseName;
  std::string previousScenario;
  double groundMu{0.7};
  double wheelRadius{0.085};
  double halfTrack{0.297};
  double torqueLimit{3.73};
  double kp{0.8};
  double ki{0.35};
  double samplePeriod{0.01};
  double yawKp{2.0};
  double yawKi{0.8};
  double yawCorrectionLimit{1.5};
  double yawIntegral{0.0};
  double effectiveYawCommand{0.0};
  double lastYawError{0.0};
  double tiltTarget{-0.174532925199};
  double tiltKp{80.0};
  double tiltKd{8.0};
  double tiltTorqueLimit{12.6};
  double tiltCommand{0.0};
  double nextSample{0.0};
  double previousSampleTime{0.0};
  gz::math::Vector3d previousLinear{0, 0, 0};
  double previousAngularZ{0.0};
  bool havePreviousVelocity{false};
  bool resolved{false};
  bool done{false};
  bool turnsOnly{false};
  bool retentionOnly{false};
  bool yawFeedbackEnabled{false};
  bool tiltHoldEnabled{false};
  bool tiltInitialized{false};
  std::ofstream stream;
};
}

GZ_ADD_PLUGIN(phase2_v3::Drive5aSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(phase2_v3::Drive5aSystem, "phase2_v3::Drive5aSystem")
