#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

namespace phase2_v3
{
class Intake5cSystem final : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->outputCsv = _sdf->Get<std::string>("output_csv");
    this->contactsCsv = _sdf->Get<std::string>("contacts_csv");
    this->approachSpeed = _sdf->Get<double>("approach_speed_m_s");
    this->surfaceSpeed = _sdf->Get<double>("surface_speed_m_s");
    this->driveStart = _sdf->Get<double>("drive_start_s", 2.0).first;
    this->driveProfile = _sdf->Get<std::string>("drive_profile", "continuous").first;
    this->recoveryStart = _sdf->Get<double>("recovery_start_s", 2.75).first;
    this->stopDuration = _sdf->Get<double>("stop_duration_s", 1.0).first;
    this->reverseSpeed = _sdf->Get<double>("reverse_speed_m_s", -0.3).first;
    this->reverseDuration = _sdf->Get<double>("reverse_duration_s", 0.35).first;
    this->trackedBallLink = _sdf->Get<std::string>("tracked_ball_link", "ball_link").first;
    this->duration = _sdf->Get<double>("duration_s", 6.0).first;
    this->telemetry.open(this->outputCsv);
    this->contacts.open(this->contactsCsv);
    this->telemetry << "time_s,base_x_m,base_y_m,base_z_m,base_roll_rad,base_pitch_rad,base_yaw_rad,"
      "ball_x_m,ball_y_m,ball_z_m,ball_vx_m_s,ball_vy_m_s,ball_vz_m_s,"
      "ball_base_x_m,ball_base_y_m,ball_base_z_m,"
      "ball_launcher_x_m,ball_launcher_y_m,ball_launcher_z_m,ball_launcher_vx_m_s,ball_launcher_vy_m_s,ball_launcher_vz_m_s,"
      "left_speed_rad_s,right_speed_rad_s,left_target_rad_s,right_target_rad_s,"
      "left_current_a,right_current_a,left_torque_nm,right_torque_nm,left_voltage_v,right_voltage_v,"
      "drive_target_m_s,drive_actual_m_s,tilt_rad,tilt_command_nm,"
      "jam_detected,jam_trigger_s,detector_event_duration_s,detector_peak_current_sum_a,"
      "detector_deficit_impulse_rpm_s,recovery_phase\n";
    this->contacts << "time_s,sensor_entity,collision_1,collision_2,point_count,max_force_n,max_depth_m,"
      "force_x_n,force_y_n,force_z_n,point_x_m,point_y_m,point_z_m\n";
    this->Resolve(_ecm);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero() || !this->Resolve(_ecm)) return;
    const double t=std::chrono::duration<double>(_info.simTime).count();
    const double dt=std::chrono::duration<double>(_info.dt).count();
    if (!this->initialized) { this->tilt.ResetPosition(_ecm,{Deg(-2.99)}); this->initialized=true; }
    const auto tq=this->tilt.Position(_ecm), tv=this->tilt.Velocity(_ecm);
    const double q=tq&&!tq->empty()?tq->at(0):Deg(-3), qd=tv&&!tv->empty()?tv->at(0):0;
    this->tiltIntegral=std::clamp(this->tiltIntegral+(Deg(-3)-q)*dt,-0.1575,0.1575);
    this->tiltCommand=std::clamp(240.0*(Deg(-3)-q)+80.0*this->tiltIntegral-12.0*qd,-12.6,12.6);
    this->tilt.SetForce(_ecm,{this->tiltCommand});

    std::array<double,2> flyOmega{};
    for (std::size_t i=0;i<2;++i) {
      const auto v=this->flywheel[i].Velocity(_ecm);
      flyOmega[i]=v&&!v->empty()?v->at(0):0;
    }
    this->UpdateJamDetector(t,dt,flyOmega);
    if (this->jamDetected && !this->recoveryAnchored) {
      const auto pose=this->base.WorldPose(_ecm);
      if (pose) {
        this->recoveryX=pose->Pos().X(); this->recoveryY=pose->Pos().Y();
        this->recoveryYaw=pose->Rot().Euler().Z(); this->recoveryAnchored=true;
      }
    }

    double leftDrive=t>=this->driveStart?this->approachSpeed:0.0;
    double rightDrive=leftDrive;
    std::array<double,2> flyTargets{-this->surfaceSpeed/0.100,this->surfaceSpeed/0.100};
    if (this->driveProfile == "stop_1s" && t >= this->recoveryStart &&
        t < this->recoveryStart + this->stopDuration)
      leftDrive=rightDrive=0.0;
    else if (this->driveProfile == "reverse_75mm" && t >= this->recoveryStart &&
        t < this->recoveryStart + this->reverseDuration)
      leftDrive=rightDrive=this->reverseSpeed;
    if (this->jamDetected) {
      const auto baseVelocity=this->base.WorldLinearVelocity(_ecm);
      const auto basePose=this->base.WorldPose(_ecm);
      const double forwardSpeed=(baseVelocity&&basePose)?
        basePose->Rot().Inverse().RotateVector(*baseVelocity).X():this->approachSpeed;
      const bool wheelOnly=this->driveProfile=="p256_reverse_both"||this->driveProfile=="p256_asym_left";
      if(wheelOnly&&this->unjamStart<0){
        leftDrive=rightDrive=0.0;this->recoveryPhase=5;
        if(std::abs(forwardSpeed)<0.02)this->unjamStart=t;
      }
      const double elapsed=this->unjamStart>=0?t-this->unjamStart:t-this->jamTrigger;
      if (this->driveProfile == "p256_reverse_both") {
        if (this->unjamStart>=0 && elapsed < 0.3) {
          leftDrive=rightDrive=0.0; flyTargets={20.0,-20.0}; this->recoveryPhase=1;
        } else if(this->unjamStart>=0) { this->recoveryPhase=4; }
      } else if (this->driveProfile == "p256_asym_left") {
        if (this->unjamStart>=0 && elapsed < 0.4) {
          leftDrive=rightDrive=0.0; flyTargets={10.0,60.0}; this->recoveryPhase=2;
        } else if(this->unjamStart>=0) { this->recoveryPhase=4; }
      } else if (this->driveProfile == "p256_reverse150_yaw12" && this->recoveryAnchored) {
        const auto pose=this->base.WorldPose(_ecm);
        if (pose) {
          const double displacement=std::hypot(pose->Pos().X()-this->recoveryX,pose->Pos().Y()-this->recoveryY);
          const double yawDelta=std::abs(std::atan2(std::sin(pose->Rot().Euler().Z()-this->recoveryYaw),
                                                    std::cos(pose->Rot().Euler().Z()-this->recoveryYaw)));
          if (this->recoveryPhase < 3 && displacement < 0.150) {
            leftDrive=rightDrive=-0.30; this->recoveryPhase=1;
          } else if (yawDelta < Deg(12.0)) {
            leftDrive=-0.18; rightDrive=0.18; this->recoveryPhase=3;
          } else { leftDrive=rightDrive=this->approachSpeed; this->recoveryPhase=4; }
        }
      }
    }
    this->currentDriveTarget=0.5*(leftDrive+rightDrive);
    for (std::size_t i=0;i<4;++i) {
      const auto v=this->drive[i].Velocity(_ecm); const double actual=v&&!v->empty()?v->at(0):0;
      const double target=(i<2?leftDrive:rightDrive)/0.085, error=target-actual;
      this->driveIntegral[i]=std::clamp(this->driveIntegral[i]+error*dt,-3.0,3.0);
      this->driveCommand[i]=std::clamp(0.8*error+0.35*this->driveIntegral[i],-3.73,3.73);
      this->drive[i].SetForce(_ecm,{this->driveCommand[i]});
    }
    for (std::size_t i=0;i<2;++i) {
      const double omega=flyOmega[i];
      const double desiredCurrent=std::clamp(5.0*(flyTargets[i]-omega)+0.002*flyTargets[i]/0.031,-20.0,20.0);
      const double backEmf=omega/(270.0*2.0*3.14159265358979323846/60.0);
      this->flyVoltage[i]=std::clamp(backEmf+0.039*desiredCurrent,-12.8,12.8);
      this->flyCurrent[i]=std::clamp((this->flyVoltage[i]-backEmf)/0.039,-20.0,20.0);
      this->flyTorque[i]=0.031*this->flyCurrent[i];
      this->flywheel[i].SetForce(_ecm,{this->flyTorque[i]});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->resolved || this->done) return;
    const double t=std::chrono::duration<double>(_info.simTime).count();
    if (t+1e-12<this->nextSample)
      return;
    this->nextSample=t+0.005;
    const auto bp=this->base.WorldPose(_ecm), lp=this->launcher.WorldPose(_ecm), ballp=this->ball.WorldPose(_ecm);
    const auto bv=this->base.WorldLinearVelocity(_ecm), lv=this->launcher.WorldLinearVelocity(_ecm), la=this->launcher.WorldAngularVelocity(_ecm), ballv=this->ball.WorldLinearVelocity(_ecm);
    if(!bp||!lp||!ballp||!bv||!lv||!la||!ballv) return;
    const auto arm=ballp->Pos()-lp->Pos();
    const auto baseRel=bp->Rot().Inverse().RotateVector(ballp->Pos()-bp->Pos());
    const auto relp=lp->Rot().Inverse().RotateVector(arm);
    const auto relv=lp->Rot().Inverse().RotateVector(*ballv-*lv-la->Cross(arm));
    const auto be=bp->Rot().Euler();
    std::array<double,2> fw{}; for(std::size_t i=0;i<2;++i){auto v=this->flywheel[i].Velocity(_ecm);fw[i]=v&&!v->empty()?v->at(0):0;}
    const double driveActual=bp->Rot().Inverse().RotateVector(*bv).X();
    auto tq=this->tilt.Position(_ecm);
    this->telemetry<<t<<','<<bp->Pos().X()<<','<<bp->Pos().Y()<<','<<bp->Pos().Z()<<','<<be.X()<<','<<be.Y()<<','<<be.Z()<<','
      <<ballp->Pos().X()<<','<<ballp->Pos().Y()<<','<<ballp->Pos().Z()<<','<<ballv->X()<<','<<ballv->Y()<<','<<ballv->Z()<<','
      <<baseRel.X()<<','<<baseRel.Y()<<','<<baseRel.Z()<<','
      <<relp.X()<<','<<relp.Y()<<','<<relp.Z()<<','<<relv.X()<<','<<relv.Y()<<','<<relv.Z()<<','
      <<fw[0]<<','<<fw[1]<<','<<-this->surfaceSpeed/0.1<<','<<this->surfaceSpeed/0.1<<','
      <<this->flyCurrent[0]<<','<<this->flyCurrent[1]<<','<<this->flyTorque[0]<<','<<this->flyTorque[1]<<','<<this->flyVoltage[0]<<','<<this->flyVoltage[1]<<','
      <<this->currentDriveTarget<<','<<driveActual<<','<<(tq&&!tq->empty()?tq->at(0):0)<<','<<this->tiltCommand<<','
      <<(this->jamDetected?1:0)<<','<<(this->jamDetected?this->jamTrigger:-1.0)<<','<<this->lastEventDuration<<','
      <<this->lastEventPeakCurrentSum<<','<<this->lastEventImpulse<<','<<this->recoveryPhase<<'\n';
    _ecm.Each<gz::sim::components::ContactSensorData>([this,t](const gz::sim::Entity &e,const gz::sim::components::ContactSensorData *d){
      for(int i=0;i<d->Data().contact_size();++i){const auto &c=d->Data().contact(i);double mf=0,md=0,fx=0,fy=0,fz=0;for(int w=0;w<c.wrench_size();++w){const auto &f=c.wrench(w).body_1_wrench().force();const double m=std::sqrt(f.x()*f.x()+f.y()*f.y()+f.z()*f.z());if(m>mf){mf=m;fx=f.x();fy=f.y();fz=f.z();}}for(int q=0;q<c.depth_size();++q)md=std::max(md,c.depth(q));double px=0,py=0,pz=0;if(c.position_size()){px=c.position(0).x();py=c.position(0).y();pz=c.position(0).z();}this->contacts<<t<<','<<e<<','<<c.collision1().name()<<','<<c.collision2().name()<<','<<c.position_size()<<','<<mf<<','<<md<<','<<fx<<','<<fy<<','<<fz<<','<<px<<','<<py<<','<<pz<<'\n';}return true;});
    if(t>=this->duration){this->telemetry.flush();this->contacts.flush();std::ofstream(this->outputCsv+".done")<<"complete\n";this->done=true;}
  }

private:
  static double Deg(double x){return x*3.14159265358979323846/180.0;}
  void UpdateJamDetector(const double t,const double dt,const std::array<double,2> &omega){
    if(this->jamDetected||t<2.1)return;
    const double rpmScale=60.0/(2.0*3.14159265358979323846);
    const double deficit=0.5*((60.0-std::abs(omega[0]))+(60.0-std::abs(omega[1])))*rpmScale;
    const double currentSum=std::abs(this->flyCurrent[0])+std::abs(this->flyCurrent[1]);
    if(deficit>5.0){
      if(!this->eventActive){this->eventActive=true;this->eventStart=t;this->eventImpulse=0;this->eventPeakCurrentSum=0;}
      this->eventImpulse+=deficit*dt;this->eventPeakCurrentSum=std::max(this->eventPeakCurrentSum,currentSum);
      this->belowSince=-1;
    }else if(this->eventActive){
      if(this->belowSince<0)this->belowSince=t;
      if(t-this->belowSince>=0.015){
        this->lastEventDuration=this->belowSince-this->eventStart;
        this->lastEventImpulse=this->eventImpulse;
        this->lastEventPeakCurrentSum=this->eventPeakCurrentSum;
        if(this->lastEventDuration>=0.050 && (this->lastEventPeakCurrentSum<30.0 || this->lastEventImpulse>1.70)){
          this->jamDetected=true;this->jamTrigger=t;
        }
        this->eventActive=false;this->belowSince=-1;
      }
    }
  }
  bool Resolve(gz::sim::EntityComponentManager &_ecm){
    if(this->resolved)return true;
    const std::array<std::string,4> dn{"rear_left_wheel_joint","front_left_wheel_joint","rear_right_wheel_joint","front_right_wheel_joint"};
    for(std::size_t i=0;i<4;++i){auto e=this->model.JointByName(_ecm,dn[i]);if(e==gz::sim::kNullEntity)return false;this->drive[i]=gz::sim::Joint(e);this->drive[i].EnableVelocityCheck(_ecm);}
    const std::array<std::string,2> fn{"flywheel_left_joint","flywheel_right_joint"};
    for(std::size_t i=0;i<2;++i){auto e=this->model.JointByName(_ecm,fn[i]);if(e==gz::sim::kNullEntity)return false;this->flywheel[i]=gz::sim::Joint(e);this->flywheel[i].EnableVelocityCheck(_ecm);}
    auto te=this->model.JointByName(_ecm,"launcher_tilt_joint"),be=this->model.LinkByName(_ecm,"base_link"),le=this->model.LinkByName(_ecm,"flywheel_launcher_frame_link");
    auto balle=_ecm.EntityByComponents(gz::sim::components::Link(),gz::sim::components::Name(this->trackedBallLink));
    if(te==gz::sim::kNullEntity||be==gz::sim::kNullEntity||le==gz::sim::kNullEntity||balle==gz::sim::kNullEntity)return false;
    this->tilt=gz::sim::Joint(te);this->tilt.EnablePositionCheck(_ecm);this->tilt.EnableVelocityCheck(_ecm);
    this->base=gz::sim::Link(be);this->launcher=gz::sim::Link(le);this->ball=gz::sim::Link(balle);
    this->base.EnableVelocityChecks(_ecm);this->launcher.EnableVelocityChecks(_ecm);this->ball.EnableVelocityChecks(_ecm);this->resolved=true;return true;}
  gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Joint tilt;std::array<gz::sim::Joint,4> drive;std::array<gz::sim::Joint,2> flywheel;gz::sim::Link base,launcher,ball;
  std::array<double,4> driveIntegral{},driveCommand{};std::array<double,2> flyCurrent{},flyTorque{},flyVoltage{};
  std::ofstream telemetry,contacts;std::string outputCsv,contactsCsv,trackedBallLink{"ball_link"},driveProfile{"continuous"};
  double approachSpeed{0.35},surfaceSpeed{2.35},driveStart{2},duration{6},nextSample{0},tiltIntegral{0},tiltCommand{0};
  double recoveryStart{2.75},stopDuration{1.0},reverseSpeed{-0.3},reverseDuration{0.35},currentDriveTarget{0};
  bool initialized{false},resolved{false},done{false},eventActive{false},jamDetected{false},recoveryAnchored{false};
  double eventStart{0},eventImpulse{0},eventPeakCurrentSum{0},belowSince{-1},jamTrigger{-1};
  double lastEventDuration{0},lastEventImpulse{0},lastEventPeakCurrentSum{0};
  double recoveryX{0},recoveryY{0},recoveryYaw{0},unjamStart{-1};int recoveryPhase{0};
};
}
GZ_ADD_PLUGIN(phase2_v3::Intake5cSystem,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(phase2_v3::Intake5cSystem,"phase2_v3::Intake5cSystem")
