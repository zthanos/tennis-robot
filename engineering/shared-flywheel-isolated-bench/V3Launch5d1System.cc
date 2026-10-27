#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>

namespace phase2_v3
{
class Launch5d1System final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->ballModel = gz::sim::Model(_entity);
    this->ballName = _sdf->Get<std::string>("ball_link", "launch_ball_link").first;
    this->surfaceSpeed = _sdf->Get<double>("surface_speed_m_s");
    this->tiltTarget = _sdf->Get<double>("tilt_target_rad");
    this->minimumRelease = _sdf->Get<double>("minimum_release_time_s", 3.5).first;
    this->duration = _sdf->Get<double>("duration_s", 7.0).first;
    this->holdPose = gz::math::Pose3d(
        _sdf->Get<gz::math::Vector3d>("hold_position_world"),
        gz::math::Quaterniond::Identity);
    if (_sdf->HasElement("release_position_world"))
    {
      this->releasePose = gz::math::Pose3d(
          _sdf->Get<gz::math::Vector3d>("release_position_world"),
          gz::math::Quaterniond::Identity);
      this->hasReleasePose = true;
    }
    this->feedVelocity = _sdf->Get<gz::math::Vector3d>("feed_velocity_world");
    this->outputPath = _sdf->Get<std::string>("output_csv");
    this->output.open(this->outputPath);
    this->contacts.open(_sdf->Get<std::string>("contacts_csv"));
    this->contacts << "time_s,sensor_entity,collision_1,collision_2,point_count,max_force_n,max_depth_m\n";
    this->output << "time_s,released,settled,ball_x_m,ball_y_m,ball_z_m,"
      "ball_vx_m_s,ball_vy_m_s,ball_vz_m_s,ball_wx_rad_s,ball_wy_rad_s,ball_wz_rad_s,"
      "base_x_m,base_y_m,base_z_m,base_yaw_rad,tilt_rad,"
      "left_speed_rad_s,right_speed_rad_s,left_target_rad_s,right_target_rad_s,"
      "left_current_a,right_current_a,left_torque_nm,right_torque_nm,"
      "left_voltage_v,right_voltage_v,release_time_s\n";
    this->ballEntity = this->ballModel.LinkByName(_ecm, this->ballName);
    if (this->ballEntity != gz::sim::kNullEntity)
      gz::sim::Link(this->ballEntity).EnableVelocityChecks(_ecm);
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->done) return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    _ecm.Each<gz::sim::components::ContactSensorData>(
      [this, time](const gz::sim::Entity &entity,
          const gz::sim::components::ContactSensorData *data)
      {
        for (int i = 0; i < data->Data().contact_size(); ++i)
        {
          const auto &contact = data->Data().contact(i);
          double maxForce = 0.0, maxDepth = 0.0;
          for (int j = 0; j < contact.wrench_size(); ++j)
          {
            const auto &force = contact.wrench(j).body_1_wrench().force();
            maxForce = std::max(maxForce, std::sqrt(force.x()*force.x()+force.y()*force.y()+force.z()*force.z()));
          }
          for (int j = 0; j < contact.depth_size(); ++j) maxDepth = std::max(maxDepth, contact.depth(j));
          this->contacts << time << ',' << entity << ',' << contact.collision1().name() << ','
            << contact.collision2().name() << ',' << contact.position_size() << ',' << maxForce << ',' << maxDepth << '\n';
        }
        return true;
      });
    if (time >= this->duration) this->contacts.flush();
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->done || _info.paused || _info.dt <= std::chrono::steady_clock::duration::zero() || !this->Resolve(_ecm))
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (!this->tiltInitialized)
    {
      this->tilt.ResetPosition(_ecm, {this->tiltTarget});
      this->tiltInitialized = true;
    }
    const auto tiltPosition = this->tilt.Position(_ecm);
    const auto tiltVelocity = this->tilt.Velocity(_ecm);
    const double tilt = tiltPosition && !tiltPosition->empty() ? tiltPosition->at(0) : this->tiltTarget;
    const double tiltRate = tiltVelocity && !tiltVelocity->empty() ? tiltVelocity->at(0) : 0.0;
    this->tiltIntegral = std::clamp(this->tiltIntegral + (this->tiltTarget - tilt) * dt, -0.1575, 0.1575);
    // Positive preload represents the P16 holding the raised module against
    // its launch-side load; the command remains inside the accepted 12.6 Nm.
    this->tilt.SetForce(_ecm, {std::clamp(240.0 * (this->tiltTarget - tilt) +
      80.0 * this->tiltIntegral - 12.0 * tiltRate + 1.2, -12.6, 12.6)});

    const std::array<double, 2> target{this->surfaceSpeed / 0.100, -this->surfaceSpeed / 0.100};
    std::array<double, 2> speed{};
    bool within = std::abs(tilt - this->tiltTarget) < 0.00174533;
    for (std::size_t i = 0; i < 2; ++i)
    {
      const auto velocity = this->flywheel[i].Velocity(_ecm);
      speed[i] = velocity && !velocity->empty() ? velocity->at(0) : 0.0;
      const double desiredCurrent = std::clamp(
        5.0 * (target[i] - speed[i]) + 0.002 * target[i] / 0.031, -20.0, 20.0);
      const double backEmf = speed[i] / (270.0 * 2.0 * M_PI / 60.0);
      this->voltage[i] = std::clamp(backEmf + 0.039 * desiredCurrent, -12.8, 12.8);
      this->current[i] = std::clamp((this->voltage[i] - backEmf) / 0.039, -20.0, 20.0);
      this->torque[i] = 0.031 * this->current[i];
      this->flywheel[i].SetForce(_ecm, {this->torque[i]});
      within = within && std::abs(target[i] - speed[i]) <= 1.0;
    }
    this->settledFor = within ? this->settledFor + dt : 0.0;
    const bool settled = this->settledFor >= 0.2;

    for (std::size_t i = 0; i < 4; ++i)
    {
      const auto velocity = this->drive[i].Velocity(_ecm);
      const double actual = velocity && !velocity->empty() ? velocity->at(0) : 0.0;
      this->drive[i].SetForce(_ecm, {std::clamp(-0.8 * actual, -3.73, 3.73)});
    }

    gz::sim::Link ball(this->ballEntity);
    if (!this->released)
    {
      this->ballModel.SetWorldPoseCmd(_ecm, this->holdPose);
      ball.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
      ball.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
      if (time >= this->minimumRelease && settled)
      {
        if (this->hasReleasePose)
        {
          this->ballModel.SetWorldPoseCmd(_ecm, this->releasePose);
          ball.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
          ball.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
          this->placementPending = true;
        }
        else
        {
          _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(this->ballModel.Entity());
          _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(this->ballEntity);
          _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(this->ballEntity);
        }
        this->released = true;
        this->releaseTime = time;
      }
    }
    else if (this->placementPending)
    {
      _ecm.RemoveComponent<gz::sim::components::WorldPoseCmd>(this->ballModel.Entity());
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(this->ballEntity);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(this->ballEntity);
      this->placementPending = false;
    }
    else if (!this->feedApplied && time >= this->releaseTime + 3.0 * dt)
    {
      ball.SetLinearVelocity(_ecm, this->feedVelocity);
      ball.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
      this->feedApplied = true;
      this->removeVelocity = true;
    }
    else if (this->removeVelocity)
    {
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(this->ballEntity);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(this->ballEntity);
      this->removeVelocity = false;
    }

    if (time + 1e-12 < this->nextSample) return;
    this->nextSample = time + 0.001;
    const auto ballPose = ball.WorldPose(_ecm);
    const auto linear = ball.WorldLinearVelocity(_ecm);
    const auto angular = ball.WorldAngularVelocity(_ecm);
    const auto basePose = this->base.WorldPose(_ecm);
    if (!ballPose || !linear || !angular || !basePose) return;
    this->output << time << ',' << this->released << ',' << settled << ','
      << ballPose->Pos().X() << ',' << ballPose->Pos().Y() << ',' << ballPose->Pos().Z() << ','
      << linear->X() << ',' << linear->Y() << ',' << linear->Z() << ','
      << angular->X() << ',' << angular->Y() << ',' << angular->Z() << ','
      << basePose->Pos().X() << ',' << basePose->Pos().Y() << ',' << basePose->Pos().Z() << ','
      << basePose->Rot().Euler().Z() << ',' << tilt << ','
      << speed[0] << ',' << speed[1] << ',' << target[0] << ',' << target[1] << ','
      << this->current[0] << ',' << this->current[1] << ',' << this->torque[0] << ',' << this->torque[1] << ','
      << this->voltage[0] << ',' << this->voltage[1] << ',' << this->releaseTime << '\n';
    if (time >= this->duration && !this->done)
    {
      this->output.flush();
      this->contacts.flush();
      std::ofstream(this->outputPath + ".done") << "complete\n";
      this->done = true;
    }
  }

private:
  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->resolved) return true;
    if (this->ballEntity == gz::sim::kNullEntity) return false;
    auto namedJoint = [&_ecm](const std::string &name) {
      return _ecm.EntityByComponents(gz::sim::components::Joint(), gz::sim::components::Name(name));
    };
    const std::array<std::string, 2> flyNames{"flywheel_left_joint", "flywheel_right_joint"};
    const std::array<std::string, 4> driveNames{"rear_left_wheel_joint", "front_left_wheel_joint",
                                                "rear_right_wheel_joint", "front_right_wheel_joint"};
    for (std::size_t i = 0; i < 2; ++i)
    {
      const auto entity = namedJoint(flyNames[i]); if (entity == gz::sim::kNullEntity) return false;
      this->flywheel[i] = gz::sim::Joint(entity); this->flywheel[i].EnableVelocityCheck(_ecm);
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
      const auto entity = namedJoint(driveNames[i]); if (entity == gz::sim::kNullEntity) return false;
      this->drive[i] = gz::sim::Joint(entity); this->drive[i].EnableVelocityCheck(_ecm);
    }
    const auto tiltEntity = namedJoint("launcher_tilt_joint");
    const auto baseEntity = _ecm.EntityByComponents(gz::sim::components::Link(), gz::sim::components::Name("base_link"));
    if (tiltEntity == gz::sim::kNullEntity || baseEntity == gz::sim::kNullEntity) return false;
    this->tilt = gz::sim::Joint(tiltEntity); this->tilt.EnablePositionCheck(_ecm); this->tilt.EnableVelocityCheck(_ecm);
    this->base = gz::sim::Link(baseEntity); this->resolved = true; return true;
  }

  gz::sim::Model ballModel{gz::sim::kNullEntity};
  gz::sim::Entity ballEntity{gz::sim::kNullEntity};
  gz::sim::Joint tilt; std::array<gz::sim::Joint, 2> flywheel; std::array<gz::sim::Joint, 4> drive;
  gz::sim::Link base;
  std::string ballName{"launch_ball_link"}, outputPath;
  std::ofstream output, contacts;
  std::array<double, 2> current{}, torque{}, voltage{};
  gz::math::Pose3d holdPose, releasePose; gz::math::Vector3d feedVelocity;
  double surfaceSpeed{12}, tiltTarget{0.261799}, minimumRelease{3.5}, duration{7},
    tiltIntegral{0}, settledFor{0}, releaseTime{-1}, nextSample{0};
  bool tiltInitialized{false}, resolved{false}, released{false}, feedApplied{false},
    removeVelocity{false}, hasReleasePose{false}, placementPending{false}, done{false};
};
}

GZ_ADD_PLUGIN(phase2_v3::Launch5d1System, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::Launch5d1System, "phase2_v3::Launch5d1System")
