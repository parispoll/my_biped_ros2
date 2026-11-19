// DART-backend diagnostic instrumentation used in the earlier physics studies.
// Retained for reproducibility; it is not the primary walking execution path.

// Optional DART 6.13 diagnostic interposer. It calls the original getter and
// observes const joint accessors only; no forces, state or constraints are set.
#include <dart/dynamics/BodyNode.hpp>
#include <dart/config.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/math/Geometry.hpp>
#include <dlfcn.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <stdexcept>
static_assert(DART_MAJOR_VERSION==6 && DART_MINOR_VERSION==13,
              "The optional DART probe must be reviewed for this DART version");

namespace {
struct FdState { double command{}, force{}, acceleration{}, velocity{}, damping{}; };
std::mutex auditMutex;
std::map<std::string,FdState> states;
}

void dart::dynamics::Skeleton::computeForwardDynamics() {
  using Function=void(*)(dart::dynamics::Skeleton *);
  static auto original=reinterpret_cast<Function>(dlsym(RTLD_NEXT,"_ZN4dart8dynamics8Skeleton22computeForwardDynamicsEv"));
  if (!original) throw std::runtime_error("DART probe cannot resolve original forward dynamics");
  std::map<std::string,double> velocities;
  for (std::size_t i=0;i<getNumJoints();++i) {
    auto *j=getJoint(i);
    if(j->getNumDofs()==1) velocities[j->getName()]=j->getVelocity(0);
  }
  original(this);
  std::lock_guard<std::mutex> lock(auditMutex);
  for (std::size_t i=0;i<getNumJoints();++i) {
    auto *j=getJoint(i);
    if(j->getNumDofs()!=1) continue;
    const auto &name=j->getName();
    const double v=velocities.at(name),acc=j->getAcceleration(0);
    states[name]={j->getCommand(0),j->getForce(0),acc,v,
                  -j->getDampingCoefficient(0)*(v+getTimeStep()*acc)};
  }
}


const Eigen::Vector6d &dart::dynamics::BodyNode::getBodyForce() const {
  using Getter=const Eigen::Vector6d &(*)(const dart::dynamics::BodyNode *);
  static Getter original=reinterpret_cast<Getter>(dlsym(RTLD_NEXT,"_ZNK4dart8dynamics8BodyNode12getBodyForceEv"));
  if (!original) throw std::runtime_error("DART probe cannot resolve original getBodyForce");
  const auto &result=original(this);
  static const char *path=std::getenv("BIPED_DART_PROBE_OUTPUT");
  if (!path) return result;
  const auto *joint=this->getParentJoint();
  const auto &name=joint->getName();
  if (name.find("left_")==std::string::npos && name.find("right_")==std::string::npos) return result;
  std::lock_guard<std::mutex> lock(auditMutex);
  static std::ofstream output(path);
  static std::map<std::string,unsigned long> counts;
  static bool header=false;
  if (!header) {
    if (!output) throw std::runtime_error("Cannot open DART probe output");
    output << "wall_time,joint,sequence,q_rad,v_rad_s,command_nm,force_nm,constraint_impulse_nm_s,"
      "friction_nm,damping_nm_s_rad,dt_s,actuator_type,transmitted_ty_nm,body_constraint_ty_impulse,fd_command_nm,fd_force_nm,fd_acceleration_rad_s2,fd_velocity_rad_s,implicit_damping_nm\n";
    output << std::setprecision(17);header=true;
  }
  const auto count=++counts[name];
  if (count%10) return result;
  const auto wrench=dart::math::dAdT(joint->getTransformFromChildBodyNode(),result);
  const auto wall=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  output << wall << ',' << name << ',' << count << ',' << joint->getPosition(0) << ','
    << joint->getVelocity(0) << ',' << joint->getCommand(0) << ',' << joint->getForce(0) << ','
    << joint->getConstraintImpulse(0) << ',' << joint->getCoulombFriction(0) << ','
    << joint->getDampingCoefficient(0) << ',' << joint->getSkeleton()->getTimeStep() << ','
    << static_cast<int>(joint->getActuatorType()) << ',' << wrench[1] << ','
    << this->getConstraintImpulse()[1];
  const auto fd=states[name];
  output << ',' << fd.command << ',' << fd.force << ',' << fd.acceleration << ',' << fd.velocity << ',' << fd.damping << '\n';
  if (count%100==0) output.flush();
  return result;
}
