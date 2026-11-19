// Read-only contact-state probe: publish fresh physics-stamped contacts.
// Explicit empty packets distinguish a genuinely airborne foot from sensor silence.

// Read-only contact heartbeat from the physics component, never from silence.
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <array>
#include <string>

namespace biped {
class ContactStateProbe : public gz::sim::System,
                          public gz::sim::ISystemPostUpdate {
  gz::transport::Node node;
  std::array<gz::transport::Node::Publisher,2> publishers{
    node.Advertise<gz::msgs::Contacts>("/biped/state/left_foot/contacts"),
    node.Advertise<gz::msgs::Contacts>("/biped/state/right_foot/contacts")};
  std::array<bool,2> seenContact{false,false};
  double last=-1;
 public:
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || info.dt<=std::chrono::steady_clock::duration::zero()) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(now<last) seenContact={false,false};
    if(now>=last && now-last<.01-1e-9) return;
    last=now;
    namespace c=gz::sim::components;
    ecm.Each<c::Collision,c::Name,c::ParentEntity,c::ContactSensorData>(
      [&](const auto &,const c::Collision *,const c::Name *name,
          const c::ParentEntity *parent,const c::ContactSensorData *data) {
        const auto *linkParent=ecm.Component<c::ParentEntity>(parent->Data());
        if(!linkParent) return true;
        const auto *modelName=ecm.Component<c::Name>(linkParent->Data());
        if(!modelName || modelName->Data()!="my_biped") return true;
        for(unsigned side=0;side<2;++side){
          if(name->Data()!=(side==0?"left_foot_collision":"right_foot_collision")) continue;
          // Initial empty / absent components are unknown until this backend
          // has actually produced contact for the collision after startup/reset.
          if(data->Data().contact_size()>0) seenContact[side]=true;
          if(!seenContact[side]) continue;
          gz::msgs::Contacts message=data->Data();
          const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
          message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
          message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
          publishers[side].Publish(message);
        }
        return true;
      });
  }
};
}
GZ_ADD_PLUGIN(biped::ContactStateProbe,gz::sim::System,gz::sim::ISystemPostUpdate)
