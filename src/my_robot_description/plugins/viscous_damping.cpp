// Physical passive damping: apply tau = -b * joint_velocity each physics step.
// b is N m s/rad. This passive torque is separate from bounded motor effort.

// Physical viscous model: tau = -b * last constrained joint velocity.
// Applied at every physics PreUpdate, after motor commands and before physics.
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <vector>

namespace biped {
class ViscousDamping : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
  public gz::sim::ISystemPreUpdate {
  struct Joint {std::string name; double coefficient; double maxPower{-1e100};unsigned samples{};double maxMotor{};};
  gz::sim::Entity model{};
  std::vector<Joint> joints;
  std::ofstream output;
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 50000;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    model=entity;
    for (auto p=sdf->FindElement("joint");p;p=p->GetNextElement("joint")) {
      const auto name=p->GetAttribute("name")->GetAsString();
      const double b=p->Get<double>("damping");
      if(!std::isfinite(b)||b<0)throw std::runtime_error("Invalid physical damping coefficient");
      joints.push_back({name,b});
    }
    const unsigned expected=sdf->HasElement("expected_joints")?sdf->Get<unsigned>("expected_joints"):6u;
    if((expected!=6u && expected!=10u)||joints.size()!=expected)
      throw std::runtime_error("Biped damping configured joint count mismatch (supported: 6 or explicit 10)");
    if(const char *path=std::getenv("BIPED_DAMPING_LOG")) {
      output.open(path);if(!output)throw std::runtime_error("Cannot open physical damping log");
      output << std::setprecision(17) << "sim_time,joint,coefficient,velocity,torque,power,max_power_in_block,steps,motor_nm,total_nm,max_abs_motor_in_block_nm\n";
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    namespace c=gz::sim::components;
    for(auto &j:joints) {
      const auto entity=ecm.EntityByComponents(c::Joint(),c::Name(j.name),c::ParentEntity(model));
      if(entity==gz::sim::kNullEntity)continue;
      const auto *velocity=ecm.Component<c::JointVelocity>(entity);
      auto *force=ecm.Component<c::JointForceCmd>(entity);
      // Until an effort interface is active, keep the original startup hold.
      if(!force||force->Data().empty()||!velocity||velocity->Data().empty())continue;
      const double v=velocity->Data()[0],tau=-j.coefficient*v;
      if(!std::isfinite(v)||!std::isfinite(tau))throw std::runtime_error("Invalid damper state");
      const double motor=force->Data()[0];
      force->Data()[0]+=tau;
      j.maxMotor=std::max(j.maxMotor,std::abs(motor));
      const double power=tau*v;
      j.maxPower=std::max(j.maxPower,power);++j.samples;
      if(info.iterations%10==0) {
        if(output)output << std::chrono::duration<double>(info.simTime).count() << ',' << j.name << ','
          << j.coefficient << ',' << v << ',' << tau << ',' << power << ',' << j.maxPower << ',' << j.samples << ','
          << motor << ',' << force->Data()[0] << ',' << j.maxMotor << '\n';
        j.maxPower=-1e100;j.samples=0;j.maxMotor=0;
      }
    }
    if(output && info.iterations%100==0)output.flush();
  }
};
}
GZ_ADD_PLUGIN(biped::ViscousDamping,gz::sim::System,
 biped::ViscousDamping::ISystemConfigure,biped::ViscousDamping::ISystemConfigurePriority,
 biped::ViscousDamping::ISystemPreUpdate)
