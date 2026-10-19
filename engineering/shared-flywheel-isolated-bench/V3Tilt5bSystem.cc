#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <sdf/Element.hh>

namespace phase2_v3
{
class Tilt5bSystem final : public gz::sim::System,
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
    this->outputCsv = _sdf->Get<std::string>("output_csv", "/tmp/v3_5b.csv").first;
    this->contactsCsv = _sdf->Get<std::string>("contacts_csv", "/tmp/v3_5b_contacts.csv").first;
    this->caseName = _sdf->Get<std::string>("case_name", "unknown").first;
    this->dynamicTilt = _sdf->Get<bool>("dynamic_tilt", true).first;
    this->samplePeriod = _sdf->Get<double>("sample_period", 0.01).first;
    this->dwell = _sdf->Get<double>("dwell_s", 3.0).first;
    this->rate = _sdf->Get<double>("angular_rate_rad_s", 0.04836115082).first;
    this->kp = _sdf->Get<double>("tilt_kp", 240.0).first;
    this->ki = _sdf->Get<double>("tilt_ki", 80.0).first;
    this->kd = _sdf->Get<double>("tilt_kd", 12.0).first;
    this->torqueLimit = _sdf->Get<double>("tilt_torque_limit", 12.6).first;
    this->telemetry.open(this->outputCsv);
    this->contacts.open(this->contactsCsv);
    this->telemetry << "time_s,case_name,phase,command_rad,measured_rad,velocity_rad_s,command_nm,"
      "base_x_m,base_y_m,base_z_m,base_roll_rad,base_pitch_rad,base_yaw_rad,"
      "module_x_m,module_y_m,module_z_m,module_roll_rad,module_pitch_rad,module_yaw_rad\n";
    this->contacts << "time_s,case_name,sensor_entity,collision_1,collision_2,point_count,max_force_n,max_depth_m\n";
    this->Resolve(_ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero() || !this->Resolve(_ecm))
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    if (!this->initialized)
    {
      // Start 0.01 deg inside the lower limit so the first DART constraint
      // projection is not misclassified as physical over-travel.
      this->tilt.ResetPosition(_ecm, {Deg(-2.99)});
      this->initialized = true;
    }
    if (!this->dynamicTilt)
    {
      this->command = 0.0;
      this->tilt.SetForce(_ecm, {0.0});
      return;
    }
    const auto target = this->TargetAt(time);
    const auto position = this->tilt.Position(_ecm);
    const auto velocity = this->tilt.Velocity(_ecm);
    const double q = position && !position->empty() ? position->at(0) : Deg(-3.0);
    const double qd = velocity && !velocity->empty() ? velocity->at(0) : 0.0;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    this->integral = std::clamp(this->integral + (target.value-q)*dt,
      -this->torqueLimit/this->ki, this->torqueLimit/this->ki);
    this->command = std::clamp(this->kp * (target.value-q) + this->ki*this->integral - this->kd * qd,
      -this->torqueLimit, this->torqueLimit);
    this->tilt.SetForce(_ecm, {this->command});
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
    const auto target = this->dynamicTilt ? this->TargetAt(time) : Target{"spawn_stability", Deg(-3.0)};
    const auto position = this->tilt.Position(_ecm);
    const auto velocity = this->tilt.Velocity(_ecm);
    const auto basePose = this->base.WorldPose(_ecm);
    const auto modulePose = this->module.WorldPose(_ecm);
    if (!basePose || !modulePose)
      return;
    const auto be = basePose->Rot().Euler();
    const auto me = modulePose->Rot().Euler();
    this->telemetry << time << ',' << this->caseName << ',' << target.phase << ',' << target.value << ','
      << (position && !position->empty() ? position->at(0) : 0.0) << ','
      << (velocity && !velocity->empty() ? velocity->at(0) : 0.0) << ',' << this->command << ','
      << basePose->Pos().X() << ',' << basePose->Pos().Y() << ',' << basePose->Pos().Z() << ','
      << be.X() << ',' << be.Y() << ',' << be.Z() << ','
      << modulePose->Pos().X() << ',' << modulePose->Pos().Y() << ',' << modulePose->Pos().Z() << ','
      << me.X() << ',' << me.Y() << ',' << me.Z() << '\n';

    _ecm.Each<gz::sim::components::ContactSensorData>(
      [this, time](const gz::sim::Entity &_entity,
          const gz::sim::components::ContactSensorData *_data)
      {
        for (int i = 0; i < _data->Data().contact_size(); ++i)
        {
          const auto &contact = _data->Data().contact(i);
          double maxForce = 0.0, maxDepth = 0.0;
          for (int w = 0; w < contact.wrench_size(); ++w)
          {
            const auto &f = contact.wrench(w).body_1_wrench().force();
            maxForce = std::max(maxForce, std::sqrt(f.x()*f.x()+f.y()*f.y()+f.z()*f.z()));
          }
          for (int d = 0; d < contact.depth_size(); ++d)
            maxDepth = std::max(maxDepth, contact.depth(d));
          this->contacts << time << ',' << this->caseName << ',' << _entity << ','
            << contact.collision1().name() << ',' << contact.collision2().name() << ','
            << contact.position_size() << ',' << maxForce << ',' << maxDepth << '\n';
        }
        return true;
      });

    if (time >= this->EndTime())
    {
      this->telemetry.flush(); this->contacts.flush();
      std::ofstream marker(this->outputCsv + ".done");
      marker << "complete\n";
      this->done = true;
    }
  }

private:
  struct Target { std::string phase; double value; };
  static double Deg(double value) { return value * 3.14159265358979323846 / 180.0; }

  Target TargetAt(double time) const
  {
    const std::array<double, 7> points{-3.0, 0.0, 15.0, 20.0, 15.0, 0.0, -3.0};
    double cursor = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i)
    {
      if (time < cursor + this->dwell)
        return {"dwell_" + std::to_string(i) + "_" + std::to_string(static_cast<int>(points[i])), Deg(points[i])};
      cursor += this->dwell;
      if (i + 1 < points.size())
      {
        const double a = Deg(points[i]), b = Deg(points[i+1]);
        const double duration = std::abs(b-a) / this->rate;
        if (time < cursor + duration)
        {
          const double fraction = std::clamp((time-cursor)/duration, 0.0, 1.0);
          return {"move_" + std::to_string(i) + "_to_" + std::to_string(i+1), a+(b-a)*fraction};
        }
        cursor += duration;
      }
    }
    return {"complete", Deg(-3.0)};
  }

  double EndTime() const
  {
    return this->dynamicTilt ? 7.0*this->dwell + 2.0*Deg(23.0)/this->rate + 0.5 : 5.0;
  }

  bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->resolved) return true;
    const auto joint = this->model.JointByName(_ecm, "launcher_tilt_joint");
    const auto baseEntity = this->model.LinkByName(_ecm, "base_link");
    auto moduleEntity = this->model.LinkByName(_ecm, "flywheel_launcher_frame_link");
    if (moduleEntity == gz::sim::kNullEntity)
      moduleEntity = this->model.LinkByName(_ecm, "launcher_yoke_link");
    if (joint == gz::sim::kNullEntity || baseEntity == gz::sim::kNullEntity || moduleEntity == gz::sim::kNullEntity)
      return false;
    this->tilt = gz::sim::Joint(joint);
    this->tilt.EnablePositionCheck(_ecm);
    this->tilt.EnableVelocityCheck(_ecm);
    this->base = gz::sim::Link(baseEntity);
    this->module = gz::sim::Link(moduleEntity);
    this->resolved = true;
    return true;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Joint tilt;
  gz::sim::Link base, module;
  std::ofstream telemetry, contacts;
  std::string outputCsv, contactsCsv, caseName;
  double samplePeriod{0.01}, dwell{3.0}, rate{0.04836115082};
  double kp{240.0}, ki{80.0}, kd{12.0}, torqueLimit{12.6}, integral{0.0}, command{0.0}, nextSample{0.0};
  bool dynamicTilt{true}, initialized{false}, resolved{false}, done{false};
};
}

GZ_ADD_PLUGIN(phase2_v3::Tilt5bSystem, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::Tilt5bSystem, "phase2_v3::Tilt5bSystem")
