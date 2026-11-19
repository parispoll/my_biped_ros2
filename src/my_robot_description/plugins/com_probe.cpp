// Read-only physics probe: publish whole-body COM and sole-corner geometry.
// All geometry is in the Gazebo world frame (metres); mass is in kg.

// Read-only, same-physics-update whole-body COM and sole geometry observer.
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <sdf/Box.hh>
#include <array>
#include <cmath>
#include <map>

namespace biped {
class ComProbe : public gz::sim::System, public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher=node.Advertise<gz::msgs::Double_V>("/biped/com_state");
  double last=-1;
 public:
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if (now>=last && now-last<.02) return;
    last=now;
    namespace c=gz::sim::components;
    gz::sim::Entity model=gz::sim::kNullEntity;
    ecm.Each<c::Model,c::Name>([&](const auto &id,const c::Model *,const c::Name *name) {
      if(name->Data()=="my_biped") model=id;
      return true;
    });
    if(model==gz::sim::kNullEntity) return;
    std::map<std::string,gz::sim::Entity> links;
    ecm.Each<c::Link,c::Name,c::ParentEntity>([&](const auto &id,const c::Link *,const c::Name *name,const c::ParentEntity *parent) {
      if(parent->Data()==model) links[name->Data()]=id;
      return true;
    });
    if(links.size()<7 || !links.count("left_foot") || !links.count("right_foot")) return;
    double mass=0;
    gz::math::Vector3d weighted=gz::math::Vector3d::Zero;
    for(const auto &[name,id]:links) {
      const auto *inertial=ecm.Component<c::Inertial>(id);
      if(!inertial) return;
      const auto &inertia=inertial->Data();
      const double m=inertia.MassMatrix().Mass();
      const auto center=(gz::sim::worldPose(id,ecm)*inertia.Pose()).Pos();
      if(!std::isfinite(m) || m<=0 || !center.IsFinite()) return;
      weighted+=m*center;mass+=m;
    }
    std::array<gz::math::Vector3d,8> corners;
    for(unsigned side=0;side<2;++side) {
      const auto foot=links.at(side==0?"left_foot":"right_foot");
      bool found=false;
      ecm.Each<c::Collision,c::Geometry,c::ParentEntity>([&](const auto &id,const c::Collision *,const c::Geometry *geometry,const c::ParentEntity *parent) {
        if(parent->Data()!=foot || found) return true;
        const auto *box=geometry->Data().BoxShape();
        if(!box) return true;
        const auto half=box->Size()/2;
        const auto pose=gz::sim::worldPose(id,ecm);
        const std::array<std::array<int,2>,4> signs={{{-1,-1},{1,-1},{1,1},{-1,1}}};
        for(unsigned i=0;i<4;++i)
          corners[side*4+i]=pose.Pos()+pose.Rot().RotateVector({signs[i][0]*half.X(),signs[i][1]*half.Y(),-half.Z()});
        found=true;return true;
      });
      if(!found) return;
    }
    const auto center=weighted/mass;
    gz::msgs::Double_V message;
    // Schema v1: time, total kg, world COM xyz, left then right sole corners xyz.
    message.add_data(now);message.add_data(mass);
    for(double v:{center.X(),center.Y(),center.Z()}) message.add_data(v);
    for(const auto &point:corners)
      for(double v:{point.X(),point.Y(),point.Z()}) message.add_data(v);
    publisher.Publish(message);
  }
};
}
GZ_ADD_PLUGIN(biped::ComProbe,gz::sim::System,gz::sim::ISystemPostUpdate)
