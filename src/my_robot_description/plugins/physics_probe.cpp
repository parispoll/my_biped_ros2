// Diagnostic physics probe for requested joint inputs and backend force accounting.
// Use recorded values to investigate actuation; this is not a walking controller.

// Optional read-only observer: no command or state components are modified.
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <string>

namespace biped {
class PhysicsProbe : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
  using Entity = gz::sim::Entity;
  struct Input {
    Entity entity{};
    double last{}, sum{}, minimum{1e100}, maximum{-1e100};
    unsigned count{};
    bool velocityPresent{};
    double velocityCommand{};
  };
  std::map<std::string, Input> joints;
  std::map<std::string, Entity> links;
  std::ofstream output;
  double externalLast{}, externalSum{};
  unsigned externalCount{};

 public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    output.open(sdf->Get<std::string>("output"), std::ios::out);
    if (!output) throw std::runtime_error("Cannot open physics-probe output");
    output << std::setprecision(17);
    output << "sim_time,joint,steps,input_last_nm,input_mean_nm,input_min_nm,input_max_nm,"
      "velocity_cmd_present,velocity_cmd,q_rad,v_rad_s,fx_joint,fy_joint,fz_joint,"
      "tx_joint,ty_joint,tz_joint,ty_world,anchor_x,anchor_y,anchor_z,"
      "gravity_per_ankle_nm,upper_mass_kg,external_fx_last_n,external_fx_mean_n,"
      "friction_setting,damping_setting,lower_rad,upper_rad\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    namespace c = gz::sim::components;
    // Only observe the humanoid in this isolated world.
    ecm.Each<c::Link,c::Name>([&](const Entity &id,const c::Link *,const c::Name *name) {
      const auto &n=name->Data();
      if (n=="base_link" || n=="left_thigh" || n=="right_thigh" || n=="left_shin" || n=="right_shin" || n=="left_foot" || n=="right_foot") links[n]=id;
      return true;
    });
    ecm.Each<c::Joint,c::Name>([&](const Entity &id,const c::Joint *,const c::Name *name) {
      const auto &n=name->Data();
      if (n.rfind("left_",0)!=0 && n.rfind("right_",0)!=0) return true;
      auto *force=ecm.Component<c::JointForceCmd>(id);
      if (!force || force->Data().empty()) return true; // not yet claimed for effort
      auto &row=joints[n]; row.entity=id; row.last=force->Data()[0];
      row.sum+=row.last; row.minimum=std::min(row.minimum,row.last);
      row.maximum=std::max(row.maximum,row.last); ++row.count;
      const auto *velocity=ecm.Component<c::JointVelocityCmd>(id);
      row.velocityPresent=velocity && !velocity->Data().empty();
      row.velocityCommand=row.velocityPresent?velocity->Data()[0]:0;
      return true;
    });
    externalLast=0;
    if (links.count("base_link")) {
      if (const auto *w=ecm.Component<c::ExternalWorldWrenchCmd>(links.at("base_link")))
        externalLast=w->Data().force().x();
    }
    externalSum+=externalLast; ++externalCount;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || info.iterations%10 || links.size()!=7) return;
    namespace c = gz::sim::components;
    const auto anchor=(gz::sim::worldPose(links.at("left_foot"),ecm).Pos()+
                       gz::sim::worldPose(links.at("right_foot"),ecm).Pos())/2;
    double gravityMoment=0, mass=0;
    for (const auto &[name,id]:links) {
      if (name=="left_foot" || name=="right_foot") continue;
      const auto *inertial=ecm.Component<c::Inertial>(id);
      if (!inertial) return;
      const auto &inertia=inertial->Data();
      const auto com=(gz::sim::worldPose(id,ecm)*inertia.Pose()).Pos();
      const double m=inertia.MassMatrix().Mass();
      mass+=m; gravityMoment+=m*9.81*(com.X()-anchor.X());
    }
    const double time=std::chrono::duration<double>(info.simTime).count();
    for (auto &[name,row]:joints) {
      if (!row.count) continue;
      const auto id=row.entity;
      const auto *q=ecm.Component<c::JointPosition>(id);
      const auto *v=ecm.Component<c::JointVelocity>(id);
      const auto *w=ecm.Component<c::JointTransmittedWrench>(id);
      const auto *axis=ecm.Component<c::JointAxis>(id);
      if (!q || !v || !w || !axis || q->Data().empty() || v->Data().empty()) continue;
      const std::string side=name.rfind("left_",0)==0?"left_":"right_";
      const std::string part=name.substr(side.size());
      const auto child=links.at(side+(part=="hip"?"thigh":part=="knee"?"shin":"foot"));
      // In this URDF each child's origin coincides with its parent joint origin.
      const auto pose=gz::sim::worldPose(child,ecm);
      const auto &f=w->Data().force(); const auto &t=w->Data().torque();
      const auto worldTorque=pose.Rot().RotateVector(gz::math::Vector3d(t.x(),t.y(),t.z()));
      output << time << ',' << name << ',' << row.count << ',' << row.last << ','
        << row.sum/row.count << ',' << row.minimum << ',' << row.maximum << ','
        << row.velocityPresent << ',' << row.velocityCommand << ',' << q->Data()[0] << ',' << v->Data()[0] << ','
        << f.x() << ',' << f.y() << ',' << f.z() << ',' << t.x() << ',' << t.y() << ',' << t.z() << ','
        << worldTorque.Y() << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ',' << pose.Pos().Z() << ','
        << gravityMoment/2 << ',' << mass << ',' << externalLast << ',' << externalSum/externalCount << ','
        << axis->Data().Friction() << ',' << axis->Data().Damping() << ',' << axis->Data().Lower() << ',' << axis->Data().Upper() << '\n';
      row.sum=0;row.minimum=1e100;row.maximum=-1e100;row.count=0;
    }
    externalSum=0;externalCount=0;output.flush();
  }
};
}
GZ_ADD_PLUGIN(biped::PhysicsProbe,gz::sim::System,
             biped::PhysicsProbe::ISystemConfigure,
             biped::PhysicsProbe::ISystemPreUpdate,
             biped::PhysicsProbe::ISystemPostUpdate)
