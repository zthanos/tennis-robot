#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace phase2_v3
{
class PlatenP261System final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->initialZ = _sdf->Get<double>("initial_z");
    this->speed = _sdf->Get<double>("speed_m_s", 0.003333).first;
    this->hold = _sdf->Get<double>("hold_s", 1.0).first;
    this->duration = _sdf->Get<double>("duration_s", 9.0).first;
    this->path = _sdf->Get<std::string>("output_csv");
    this->output.open(this->path);
    this->output << "time_s,upper_platen_z_m\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->done || _info.paused || _info.dt <= std::chrono::steady_clock::duration::zero())
      return;
    const double time = std::chrono::duration<double>(_info.simTime).count();
    const double travel = time > this->hold ? this->speed * (time - this->hold) : 0.0;
    const double z = this->initialZ - travel;
    this->model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(0, 0, z, 0, 0, 0));
    this->output << time << ',' << z << '\n';
    if (time >= this->duration)
    {
      this->output.close();
      std::ofstream(this->path + ".done") << "done\n";
      this->done = true;
    }
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  double initialZ{0.0};
  double speed{0.003333};
  double hold{1.0};
  double duration{9.0};
  std::string path;
  std::ofstream output;
  bool done{false};
};
}

GZ_ADD_PLUGIN(phase2_v3::PlatenP261System, gz::sim::System,
    gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::PlatenP261System,
    "phase2_v3::PlatenP261System")
