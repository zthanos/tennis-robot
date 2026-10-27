#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

namespace phase2_v3
{
class Basket5c2Observer final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->outputCsv = _sdf->Get<std::string>("output_csv");
    this->duration = _sdf->Get<double>("duration_s");
    this->samplePeriod = _sdf->Get<double>("sample_period_s", 0.01).first;
    this->expectedBallCount = _sdf->Get<int>("expected_ball_count");
    this->stream.open(this->outputCsv);
    this->stream << "time_s,ball_name,world_x_m,world_y_m,world_z_m,base_x_m,base_y_m,base_z_m,vx_m_s,vy_m_s,vz_m_s\n";
    this->Resolve(_ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (static_cast<int>(this->balls.size()) != this->expectedBallCount)
      this->Resolve(_ecm);
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->done) return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    if (time + 1e-12 < this->nextSample) return;
    this->nextSample = time + this->samplePeriod;
    if (!this->resolved || static_cast<int>(this->balls.size()) != this->expectedBallCount) return;
    const auto basePose = this->base.WorldPose(_ecm);
    if (!basePose) return;
    for (const auto &ball : this->balls)
    {
      const auto pose = ball.link.WorldPose(_ecm);
      const auto velocity = ball.link.WorldLinearVelocity(_ecm);
      if (!pose || !velocity) continue;
      const auto relative = basePose->Rot().Inverse().RotateVector(pose->Pos()-basePose->Pos());
      this->stream << time << ',' << ball.name << ',' << pose->Pos().X() << ','
        << pose->Pos().Y() << ',' << pose->Pos().Z() << ',' << relative.X() << ','
        << relative.Y() << ',' << relative.Z() << ',' << velocity->X() << ','
        << velocity->Y() << ',' << velocity->Z() << '\n';
    }
    if (time >= this->duration)
    {
      this->stream.flush();
      std::ofstream(this->outputCsv + ".done") << "complete\n";
      this->done = true;
    }
  }

private:
  struct Ball { std::string name; gz::sim::Link link; };
  void Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    const auto baseEntity = this->model.LinkByName(_ecm, "base_link");
    if (baseEntity == gz::sim::kNullEntity) return;
    this->base = gz::sim::Link(baseEntity);
    this->balls.clear();
    _ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
      [this, &_ecm](const gz::sim::Entity &entity,
          const gz::sim::components::Link *, const gz::sim::components::Name *name)
      {
        const auto &value = name->Data();
        if (value == "capture_ball_link" || value.rfind("capture_ball_", 0) == 0 ||
            value.rfind("basket_ball_", 0) == 0)
        {
          gz::sim::Link link(entity);
          link.EnableVelocityChecks(_ecm);
          this->balls.push_back({value, link});
        }
        return true;
      });
    this->resolved = true;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link base;
  std::vector<Ball> balls;
  std::ofstream stream;
  std::string outputCsv;
  double duration{12.0};
  double samplePeriod{0.01};
  double nextSample{0.0};
  int expectedBallCount{0};
  bool resolved{false};
  bool done{false};
};
}

GZ_ADD_PLUGIN(phase2_v3::Basket5c2Observer,
    gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::Basket5c2Observer, "phase2_v3::Basket5c2Observer")
