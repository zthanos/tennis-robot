#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>
namespace phase2_v3 {
class ReboundP261Observer final : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager&) override {
    gz::sim::Model m(e); entity=m.LinkByName(ecm,s->Get<std::string>("ball_link","ball_link").first); gz::sim::Link(entity).EnableVelocityChecks(ecm);
    path=s->Get<std::string>("output_csv"); duration=s->Get<double>("duration_s",1.8).first; out.open(path); out<<"time_s,z_m,vz_m_s\n";
  }
  void PostUpdate(const gz::sim::UpdateInfo&i,const gz::sim::EntityComponentManager&ecm) override {
    if(done||i.paused) return;
    gz::sim::Link l(entity);
    auto p=l.WorldPose(ecm);
    auto v=l.WorldLinearVelocity(ecm);
    if(!p||!v)return;
    double t=std::chrono::duration<double>(i.simTime).count(); out<<t<<','<<p->Pos().Z()<<','<<v->Z()<<'\n'; if(t>=duration){out.close();std::ofstream(path+".done")<<"done\n";done=true;}
  }
 private: gz::sim::Entity entity{gz::sim::kNullEntity}; std::string path; std::ofstream out; double duration{1.8}; bool done{false};
}; }
GZ_ADD_PLUGIN(phase2_v3::ReboundP261Observer,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::ReboundP261Observer,"phase2_v3::ReboundP261Observer")
