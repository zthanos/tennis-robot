#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "tennis_ball_contact_system/ContactModel.hh"

namespace tennis_ball_contact_system
{
class TennisBallContactSystem final :
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
    gz::sim::Model model(_entity);
    this->ballLinkName = _sdf->Get<std::string>("ball_link", "ball_link").first;
    this->wheelLinkNames[0] = _sdf->Get<std::string>(
        "left_wheel_link", "flywheel_left_link").first;
    this->wheelLinkNames[1] = _sdf->Get<std::string>(
        "right_wheel_link", "flywheel_right_link").first;
    this->groundEnabled = _sdf->Get<bool>("enable_ground", true).first;
    this->groundHeight = _sdf->Get<double>("ground_height", 0.0).first;
    this->topic = _sdf->Get<std::string>(
        "telemetry_topic", "/tennis_ball/compliant_contacts").first;
    if (_sdf->HasElement("telemetry_csv"))
    {
      this->telemetryCsv.open(_sdf->Get<std::string>("telemetry_csv"));
      this->telemetryCsv << "time_s,contact_id,normal_force_n,elastic_force_n,damping_force_n,"
        "tangential_speed_m_s,tangential_force_x_n,tangential_force_y_n,tangential_force_z_n,"
        "friction_limit_n,ball_torque_x_nm,ball_torque_y_nm,ball_torque_z_nm,"
        "wheel_torque_x_nm,wheel_torque_y_nm,wheel_torque_z_nm,contact_x_m,contact_y_m,contact_z_m,"
        "normal_x,normal_y,normal_z,compression_m,compression_rate_m_s\n";
    }
    if (_sdf->HasElement("friction_coefficient"))
      this->frictionCoefficient = _sdf->Get<double>("friction_coefficient");

    this->parameters.radius = _sdf->Get<double>("ball_radius", 0.033).first;
    this->parameters.loadingExponent = _sdf->Get<double>(
        "loading_exponent", 1.5).first;
    this->parameters.unloadingExponent = _sdf->Get<double>(
        "unloading_exponent", 1.9866353710127362).first;
    this->parameters.loadingStiffness = _sdf->Get<double>(
        "loading_stiffness", 107309.29404174259).first;
    this->parameters.dynamicDamping = _sdf->Get<double>(
        "dynamic_damping", 4692.375890562493).first;
    this->parameters.maximumCompression = _sdf->Get<double>(
        "maximum_compression", 0.035).first;
    this->parameters.forceCap = _sdf->Get<double>("force_cap", 5000.0).first;

    this->ballEntity = model.LinkByName(_ecm, this->ballLinkName);
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
    if (!this->ResolveLinks(_ecm))
      return;

    gz::sim::Link ball(this->ballEntity);
    const auto ballPose = ball.WorldPose(_ecm);
    const auto ballLinear = ball.WorldLinearVelocity(_ecm);
    const auto ballAngular = ball.WorldAngularVelocity(_ecm);
    if (!ballPose || !ballLinear || !ballAngular)
      return;

    gz::msgs::Double_V telemetry;
    for (std::size_t index = 0; index < this->wheelEntities.size(); ++index)
    {
      if (this->wheelEntities[index] == gz::sim::kNullEntity)
        continue;
      gz::sim::Link wheel(this->wheelEntities[index]);
      const auto wheelPose = wheel.WorldPose(_ecm);
      const auto wheelLinear = wheel.WorldLinearVelocity(_ecm);
      const auto wheelAngular = wheel.WorldAngularVelocity(_ecm);
      if (!wheelPose || !wheelLinear || !wheelAngular)
        continue;

      const auto axis = wheelPose->Rot().RotateVector(gz::math::Vector3d::UnitZ);
      const auto geometry = SphereFiniteCylinder(
          ballPose->Pos(), this->parameters.radius,
          wheelPose->Pos(), axis, 0.100, 0.025);
      if (!geometry.active)
      {
        this->wheelStates[index] = State{};
        continue;
      }

      const auto ballArm = geometry.point - ballPose->Pos();
      const auto wheelArm = geometry.point - wheelPose->Pos();
      const auto ballPointVelocity = *ballLinear + ballAngular->Cross(ballArm);
      const auto wheelPointVelocity = *wheelLinear + wheelAngular->Cross(wheelArm);
      const auto relative = ballPointVelocity - wheelPointVelocity;
      const double compressionRate = -relative.Dot(geometry.normal);
      const auto normal = StepNormal(
          this->parameters, this->wheelStates[index],
          geometry.compression, compressionRate);

      const auto tangentialVelocity = relative - relative.Dot(geometry.normal) * geometry.normal;
      gz::math::Vector3d tangentialForce = gz::math::Vector3d::Zero;
      double frictionLimit = std::numeric_limits<double>::quiet_NaN();
      if (this->frictionCoefficient)
      {
        frictionLimit = *this->frictionCoefficient * normal.force;
        if (tangentialVelocity.Length() > 1e-15)
        {
          const double magnitude = frictionLimit * std::tanh(
              tangentialVelocity.Length() / 0.05);
          tangentialForce = -magnitude * tangentialVelocity.Normalized();
        }
      }
      const auto ballForce = normal.force * geometry.normal + tangentialForce;
      const auto wheelForce = -ballForce;
      const auto ballTorque = ballArm.Cross(tangentialForce);
      const auto wheelTorque = wheelArm.Cross(-tangentialForce);
      ball.AddWorldWrench(_ecm, ballForce, ballTorque);
      wheel.AddWorldWrench(_ecm, wheelForce, wheelTorque);
      this->AppendTelemetry(
          telemetry, std::chrono::duration<double>(_info.simTime).count(),
          static_cast<double>(index), normal, geometry,
          tangentialVelocity, tangentialForce, frictionLimit,
          ballTorque, wheelTorque);
    }

    if (this->groundEnabled)
    {
      const double compression = this->parameters.radius -
          (ballPose->Pos().Z() - this->groundHeight);
      const double compressionRate = -ballLinear->Z();
      const auto normal = StepNormal(
          this->parameters, this->groundState, compression, compressionRate);
      if (normal.force > 0.0)
      {
        ball.AddWorldForce(_ecm, gz::math::Vector3d(0, 0, normal.force));
        Geometry geometry;
        geometry.active = true;
        geometry.compression = std::max(compression, 0.0);
        geometry.point = gz::math::Vector3d(
            ballPose->Pos().X(), ballPose->Pos().Y(), this->groundHeight);
        geometry.normal = gz::math::Vector3d::UnitZ;
        geometry.region = "ground";
        this->AppendTelemetry(
            telemetry, std::chrono::duration<double>(_info.simTime).count(),
            2.0, normal, geometry,
            gz::math::Vector3d::Zero, gz::math::Vector3d::Zero,
            std::numeric_limits<double>::quiet_NaN(),
            gz::math::Vector3d::Zero, gz::math::Vector3d::Zero);
      }
    }
    if (telemetry.data_size() > 0)
      this->publisher.Publish(telemetry);
  }

private:
  bool ResolveLinks(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->ballEntity == gz::sim::kNullEntity)
      return false;
    for (std::size_t index = 0; index < this->wheelEntities.size(); ++index)
    {
      if (this->wheelEntities[index] == gz::sim::kNullEntity)
      {
        this->wheelEntities[index] = _ecm.EntityByComponents(
            gz::sim::components::Link(),
            gz::sim::components::Name(this->wheelLinkNames[index]));
        if (this->wheelEntities[index] != gz::sim::kNullEntity)
        {
          gz::sim::Link(this->wheelEntities[index]).EnableVelocityChecks(_ecm);
          // The analytical plugin owns all flywheel contact response. Remove
          // only the two wheel collision entities; cradle and every unrelated
          // collision remain native. URDF <gazebo reference> bitmask tags are
          // emitted at invalid link scope by sdformat, so they cannot provide
          // a reliable pair filter in the supported Harmonic stack.
          for (const auto collision : _ecm.ChildrenByComponents(
              this->wheelEntities[index], gz::sim::components::Collision()))
          {
            const auto collisionName = _ecm.Component<gz::sim::components::Name>(collision);
            auto tyreToken = this->wheelLinkNames[index];
            const auto linkSuffix = tyreToken.rfind("_link");
            if (linkSuffix != std::string::npos)
              tyreToken.replace(linkSuffix, 5, "_col_collision");
            if (collisionName && collisionName->Data().find(tyreToken) != std::string::npos)
              _ecm.RequestRemoveEntity(collision);
          }
        }
      }
    }
    return true;
  }

  void AppendVector(gz::msgs::Double_V &_message, const gz::math::Vector3d &_value)
  {
    _message.add_data(_value.X());
    _message.add_data(_value.Y());
    _message.add_data(_value.Z());
  }

  void AppendTelemetry(
      gz::msgs::Double_V &_message, const double _time, const double _contactId,
      const NormalSample &_normal, const Geometry &_geometry,
      const gz::math::Vector3d &_tangentialVelocity,
      const gz::math::Vector3d &_tangentialForce,
      const double _frictionLimit,
      const gz::math::Vector3d &_ballTorque,
      const gz::math::Vector3d &_wheelTorque)
  {
    _message.add_data(_contactId);
    _message.add_data(_normal.force);
    _message.add_data(_tangentialVelocity.Length());
    this->AppendVector(_message, _tangentialForce);
    _message.add_data(_frictionLimit);
    this->AppendVector(_message, _ballTorque);
    this->AppendVector(_message, _wheelTorque);
    this->AppendVector(_message, _geometry.point);
    this->AppendVector(_message, _geometry.normal);
    _message.add_data(_geometry.compression);
    _message.add_data(_normal.compressionRate);
    if (this->telemetryCsv)
    {
      this->telemetryCsv << _time << ',' << _contactId << ',' << _normal.force << ','
        << _normal.elasticForce << ',' << _normal.dampingForce << ','
        << _tangentialVelocity.Length() << ','
        << _tangentialForce.X() << ',' << _tangentialForce.Y() << ',' << _tangentialForce.Z() << ','
        << _frictionLimit << ','
        << _ballTorque.X() << ',' << _ballTorque.Y() << ',' << _ballTorque.Z() << ','
        << _wheelTorque.X() << ',' << _wheelTorque.Y() << ',' << _wheelTorque.Z() << ','
        << _geometry.point.X() << ',' << _geometry.point.Y() << ',' << _geometry.point.Z() << ','
        << _geometry.normal.X() << ',' << _geometry.normal.Y() << ',' << _geometry.normal.Z() << ','
        << _geometry.compression << ',' << _normal.compressionRate << '\n';
    }
  }

  Parameters parameters;
  State groundState;
  std::array<State, 2> wheelStates;
  gz::sim::Entity ballEntity{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 2> wheelEntities{
      gz::sim::kNullEntity, gz::sim::kNullEntity};
  std::string ballLinkName{"ball_link"};
  std::array<std::string, 2> wheelLinkNames{
      "flywheel_left_link", "flywheel_right_link"};
  bool groundEnabled{true};
  double groundHeight{0.0};
  std::optional<double> frictionCoefficient;
  std::string topic{"/tennis_ball/compliant_contacts"};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::ofstream telemetryCsv;
};
}  // namespace tennis_ball_contact_system

GZ_ADD_PLUGIN(
    tennis_ball_contact_system::TennisBallContactSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    tennis_ball_contact_system::TennisBallContactSystem,
    "tennis_ball_contact_system::TennisBallContactSystem")
