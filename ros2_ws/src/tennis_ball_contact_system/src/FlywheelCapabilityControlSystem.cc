#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <sdf/Element.hh>

namespace tennis_ball_contact_system
{
class FlywheelCapabilityControlSystem final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->ballLinkName = _sdf->Get<std::string>("ball_link", "ball_link").first;
    this->wheelNames[0] = _sdf->Get<std::string>("left_wheel_link", "flywheel_left_link").first;
    this->wheelNames[1] = _sdf->Get<std::string>("right_wheel_link", "flywheel_right_link").first;
    this->targets[0] = _sdf->Get<double>("left_target_rad_s", 100.0).first;
    this->targets[1] = _sdf->Get<double>("right_target_rad_s", -100.0).first;
    this->effortLimit = _sdf->Get<double>("effort_limit_nm", 0.62).first;
    this->speedKp = _sdf->Get<double>("speed_kp_nm_per_rad_s", 0.05).first;
    this->jointDamping = _sdf->Get<double>("joint_damping_nm_s_rad", 0.002).first;
    this->minimumInjectionTime = _sdf->Get<double>("minimum_injection_time_s", 4.0).first;
    this->settleTolerance = _sdf->Get<double>("settle_tolerance_rad_s", 1.0).first;
    this->settleDuration = _sdf->Get<double>("settle_duration_s", 0.2).first;
    this->holdPose = gz::math::Pose3d(
        _sdf->Get<gz::math::Vector3d>("hold_position_world", {0, 0, 0.35}).first,
        gz::math::Quaterniond::Identity);
    this->injectionVelocity = _sdf->Get<gz::math::Vector3d>(
        "injection_velocity_world", {1, 0, 0}).first;
    this->topic = _sdf->Get<std::string>(
        "state_topic", "/flywheel/capability_state").first;
    if (_sdf->HasElement("state_csv"))
    {
      this->stateCsv.open(_sdf->Get<std::string>("state_csv"));
      this->stateCsv << "time_s,injected,settled,ball_x_m,ball_y_m,ball_z_m,"
        "ball_vx_m_s,ball_vy_m_s,ball_vz_m_s,ball_wx_rad_s,ball_wy_rad_s,ball_wz_rad_s,"
        "left_speed_rad_s,right_speed_rad_s,left_torque_nm,right_torque_nm,"
        "left_target_rad_s,right_target_rad_s,injection_time_s,left_motor_work_j,right_motor_work_j,"
        "left_x_m,left_y_m,left_z_m,left_axis_x,left_axis_y,left_axis_z,"
        "right_x_m,right_y_m,right_z_m,right_axis_x,right_axis_y,right_axis_z\n";
    }
    this->ballEntity = this->model.LinkByName(_ecm, this->ballLinkName);
    if (this->ballEntity != gz::sim::kNullEntity)
    {
      gz::sim::Link(this->ballEntity).EnableVelocityChecks(_ecm);
    }
    this->publisher = this->node.Advertise<gz::msgs::Double_V>(this->topic);
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero())
      return;
    if (!this->Resolve(_ecm))
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    std::array<double, 2> speeds{0, 0};
    std::array<double, 2> torques{0, 0};
    std::array<gz::math::Vector3d, 2> wheelPositions;
    std::array<gz::math::Vector3d, 2> wheelAxes;
    bool withinTolerance = true;
    for (std::size_t index = 0; index < 2; ++index)
    {
      gz::sim::Link wheel(this->wheelEntities[index]);
      const auto pose = wheel.WorldPose(_ecm);
      const auto angular = wheel.WorldAngularVelocity(_ecm);
      if (!pose || !angular)
        return;
      const auto axis = pose->Rot().RotateVector(gz::math::Vector3d::UnitZ);
      wheelPositions[index] = pose->Pos();
      wheelAxes[index] = axis;
      speeds[index] = angular->Dot(axis);
      const double feedforward = this->jointDamping * this->targets[index];
      torques[index] = std::clamp(
          feedforward + this->speedKp * (this->targets[index] - speeds[index]),
          -this->effortLimit, this->effortLimit);
      wheel.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, torques[index] * axis);
      this->motorWork[index] += torques[index] * speeds[index] * dt;
      withinTolerance = withinTolerance &&
          std::abs(this->targets[index] - speeds[index]) <= this->settleTolerance;
    }
    this->settledFor = withinTolerance ? this->settledFor + dt : 0.0;
    const bool settled = this->settledFor >= this->settleDuration;
    if (settled && !this->spinupRecorded)
    {
      this->spinupTime = time;
      this->spinupRecorded = true;
    }

    gz::sim::Link ball(this->ballEntity);
    if (!this->injected)
    {
      // A pose command makes the pre-launch state deterministic without
      // changing the ball model's gravity or contact properties. Remove the
      // command atomically when injection begins so physics owns the ball.
      this->model.SetWorldPoseCmd(_ecm, this->holdPose);
      ball.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
      ball.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
      if (time >= this->minimumInjectionTime && settled)
      {
        _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(
            this->model.Entity());
        _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
            this->ballEntity);
        _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
            this->ballEntity);
        this->injected = true;
        this->injectionTime = time;
      }
    }
    else if (!this->injectionVelocityApplied && time >= this->injectionTime + 3.0 * dt)
    {
      // Pose and velocity commands cannot be consumed reliably by the physics
      // system in the same PreUpdate. Allow the pose-removal command to pass
      // through the system update pipeline before applying the one-shot
      // injection velocity.
      ball.SetLinearVelocity(_ecm, this->injectionVelocity);
      ball.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
      this->injectionVelocityApplied = true;
      this->velocityCommandRemovalPending = true;
    }
    else if (this->velocityCommandRemovalPending)
    {
      // Velocity commands are single-step launch conditions, not a kinematic
      // drive. Removing them hands the resulting state back to physics.
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
          this->ballEntity);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
          this->ballEntity);
      this->velocityCommandRemovalPending = false;
    }

    const auto ballPose = ball.WorldPose(_ecm);
    const auto ballLinear = ball.WorldLinearVelocity(_ecm);
    const auto ballAngular = ball.WorldAngularVelocity(_ecm);
    if (!ballPose || !ballLinear || !ballAngular)
      return;
    gz::msgs::Double_V message;
    const std::array<double, 33> values{
        time, this->injected ? 1.0 : 0.0, settled ? 1.0 : 0.0,
        ballPose->Pos().X(), ballPose->Pos().Y(), ballPose->Pos().Z(),
        ballLinear->X(), ballLinear->Y(), ballLinear->Z(),
        ballAngular->X(), ballAngular->Y(), ballAngular->Z(),
        speeds[0], speeds[1], torques[0], torques[1],
        this->targets[0], this->targets[1], this->injectionTime,
        this->motorWork[0], this->motorWork[1],
        wheelPositions[0].X(), wheelPositions[0].Y(), wheelPositions[0].Z(),
        wheelAxes[0].X(), wheelAxes[0].Y(), wheelAxes[0].Z(),
        wheelPositions[1].X(), wheelPositions[1].Y(), wheelPositions[1].Z(),
        wheelAxes[1].X(), wheelAxes[1].Y(), wheelAxes[1].Z()};
    for (const double value : values)
      message.add_data(value);
    this->publisher.Publish(message);
    if (this->stateCsv)
    {
      for (std::size_t index = 0; index < values.size(); ++index)
      {
        if (index > 0)
          this->stateCsv << ',';
        this->stateCsv << values[index];
      }
      this->stateCsv << '\n';
    }
  }

private:
  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->ballEntity == gz::sim::kNullEntity)
      return false;
    for (std::size_t index = 0; index < 2; ++index)
    {
      if (this->wheelEntities[index] == gz::sim::kNullEntity)
      {
        this->wheelEntities[index] = _ecm.EntityByComponents(
            gz::sim::components::Link(), gz::sim::components::Name(this->wheelNames[index]));
        if (this->wheelEntities[index] != gz::sim::kNullEntity)
          gz::sim::Link(this->wheelEntities[index]).EnableVelocityChecks(_ecm);
      }
      if (this->wheelEntities[index] == gz::sim::kNullEntity)
        return false;
    }
    return true;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity ballEntity{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 2> wheelEntities{
      gz::sim::kNullEntity, gz::sim::kNullEntity};
  std::string ballLinkName{"ball_link"};
  std::array<std::string, 2> wheelNames{"flywheel_left_link", "flywheel_right_link"};
  std::array<double, 2> targets{100.0, -100.0};
  std::array<double, 2> motorWork{0, 0};
  double effortLimit{0.62};
  double speedKp{0.05};
  double jointDamping{0.002};
  double minimumInjectionTime{4.0};
  double settleTolerance{1.0};
  double settleDuration{0.2};
  double settledFor{0.0};
  double spinupTime{-1.0};
  double injectionTime{-1.0};
  bool spinupRecorded{false};
  bool injected{false};
  bool injectionVelocityApplied{false};
  bool velocityCommandRemovalPending{false};
  gz::math::Pose3d holdPose;
  gz::math::Vector3d injectionVelocity{1, 0, 0};
  std::string topic{"/flywheel/capability_state"};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::ofstream stateCsv;
};
}  // namespace tennis_ball_contact_system

GZ_ADD_PLUGIN(
    tennis_ball_contact_system::FlywheelCapabilityControlSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    tennis_ball_contact_system::FlywheelCapabilityControlSystem,
    "tennis_ball_contact_system::FlywheelCapabilityControlSystem")
