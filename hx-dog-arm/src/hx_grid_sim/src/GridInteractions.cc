#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include "hx_grid_sim/InteractionState.hh"

namespace hx_grid {
namespace comp = gz::sim::components;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;
class GridInteractions final : public gz::sim::System,
                              public gz::sim::ISystemConfigure,
                              public gz::sim::ISystemPreUpdate {
  struct Cabinet {
    std::string name;
    CabinetState state;
    double temperature{30}, meter{0};
    Entity model{kNullEntity}, press{kNullEntity}, rotor{kNullEntity}, lamp{kNullEntity};
    bool lastFault{false}, lastArmed{false}, initialized{false};
  };
  Entity world{kNullEntity}, door{kNullEntity}, hinge{kNullEntity}, handle{kNullEntity}, card{kNullEntity};
  std::string scene;
  std::vector<Cabinet> cabinets;
  DoorState doorState;
  bool hasDoor{false}, doorInitialized{false}, lastLatch{true};
  double lastPublish{-1}, faultInterval{0}, lastFaultTime{0};
  size_t nextFault{0};
  gz::transport::Node node;
  gz::transport::Node::Publisher truth;
  std::mutex mutex;
  std::vector<std::string> faultRequests;

  Entity ModelByName(const std::string &name, const EntityComponentManager &ecm) const {
    return ecm.EntityByComponents(comp::Name(name), comp::ParentEntity(world));
  }
  double Position(Entity entity, const EntityComponentManager &ecm) const {
    auto p = ecm.Component<comp::JointPosition>(entity);
    return p && !p->Data().empty() ? p->Data()[0] : 0.;
  }
  void EnableJoint(Entity joint, EntityComponentManager &ecm) {
    if (joint != kNullEntity) gz::sim::Joint(joint).EnablePositionCheck(ecm);
  }
  void Limits(Entity joint, double lo, double hi, EntityComponentManager &ecm) {
    std::vector<gz::math::Vector2d> values{{lo, hi}};
    auto c = ecm.Component<comp::JointPositionLimitsCmd>(joint);
    if (c) c->Data() = values;
    else ecm.CreateComponent(joint, comp::JointPositionLimitsCmd(values));
  }
  void Lamp(Cabinet &c, EntityComponentManager &ecm) {
    if (c.lamp == kNullEntity) return;
    gz::msgs::Visual msg;
    msg.set_id(c.lamp);
    auto mat = msg.mutable_material();
    for (auto color : {mat->mutable_ambient(), mat->mutable_diffuse(), mat->mutable_emissive()}) {
      color->set_r(c.state.fault ? 1. : .02);
      color->set_g(c.state.fault ? .02 : 1.);
      color->set_b(.02); color->set_a(1.);
    }
    auto cmd = ecm.Component<comp::VisualCmd>(c.lamp);
    if (cmd) { cmd->Data() = msg; ecm.SetChanged(c.lamp, comp::VisualCmd::typeId, gz::sim::ComponentState::OneTimeChange); }
    else ecm.CreateComponent(c.lamp, comp::VisualCmd(msg));
  }
 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &, gz::sim::EventManager &) override {
    world = entity;
    scene = sdf->Get<std::string>("scene_id");
    faultInterval = sdf->Get<double>("fault_interval_s", 0.).first;
    hasDoor = sdf->Get<int>("access_control", 0).first != 0;
    auto el = sdf->FindElement("cabinet");
    while (el) {
      Cabinet c;
      c.name = el->Get<std::string>("name");
      c.state.fault = el->Get<int>("fault") != 0;
      c.state.rotary = el->Get<int>("rotary") != 0;
      c.temperature = el->Get<double>("temperature");
      c.meter = el->Get<double>("meter");
      cabinets.push_back(c);
      el = el->GetNextElement("cabinet");
    }
    truth = node.Advertise<gz::msgs::StringMsg>("/evaluation/ground_truth");
    node.Subscribe<gz::msgs::StringMsg>("/evaluation/inject_fault", [this](const gz::msgs::StringMsg &msg) {
      std::lock_guard<std::mutex> lock(mutex);
      if (faultRequests.size() < 64) faultRequests.push_back(msg.data());
    });
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, EntityComponentManager &ecm) override {
    if (info.paused) return;
    double now = std::chrono::duration<double>(info.simTime).count();
    double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    std::vector<std::string> requests;
    { std::lock_guard<std::mutex> lock(mutex); requests.swap(faultRequests); }
    if (faultInterval > 0 && now-lastFaultTime >= faultInterval && !cabinets.empty()) {
      requests.push_back(cabinets[nextFault++ % cabinets.size()].name);
      lastFaultTime = now;
    }
    for (auto &c : cabinets) {
      if (c.model == kNullEntity) {
        c.model = ModelByName(c.name, ecm);
        if (c.model == kNullEntity) continue;
        gz::sim::Model m(c.model);
        c.press = m.JointByName(ecm, "press_joint");
        c.rotor = m.JointByName(ecm, "rotary_joint");
        EnableJoint(c.press, ecm); EnableJoint(c.rotor, ecm);
        c.lamp = ecm.EntityByComponents(comp::Visual(), comp::Name("indicator"),
                                        comp::ParentEntity(m.LinkByName(ecm, "cabinet")));
      }
      if (c.press == kNullEntity) continue;
      for (const auto &r : requests) if (r == c.name) c.state.InjectFault();
      c.state.Step(dt, Position(c.press, ecm), Position(c.rotor, ecm));
      if (!c.initialized || c.lastArmed != c.state.armed) {
        Limits(c.press, 0., c.state.armed ? .012 : 0., ecm);
        c.lastArmed = c.state.armed;
      }
      if (!c.initialized || c.lastFault != c.state.fault) {
        Lamp(c, ecm); c.lastFault = c.state.fault;
      }
      c.initialized = true;
    }
    if (hasDoor) {
      if (door == kNullEntity) {
        door = ModelByName("access_door", ecm);
        card = ModelByName("authorized_card", ecm);
        if (door != kNullEntity) {
          gz::sim::Model m(door);
          hinge = m.JointByName(ecm, "hinge"); handle = m.JointByName(ecm, "handle_joint");
          EnableJoint(hinge, ecm); EnableJoint(handle, ecm);
        }
      }
      if (hinge != kNullEntity && handle != kNullEntity) {
        bool near = card != kNullEntity &&
          (gz::sim::worldPose(card, ecm).Pos() - gz::math::Vector3d(-.21, -1.22, 1.)).Length() <= .10;
        doorState.Step(now, dt, near, Position(handle, ecm), Position(hinge, ecm));
        if (!doorInitialized || lastLatch != doorState.latched) {
          Limits(hinge, doorState.latched ? 0. : -1.6, 0., ecm);
          lastLatch = doorState.latched; doorInitialized = true;
        }
      }
    }
    if (now-lastPublish < .1) return;
    lastPublish = now;
    std::ostringstream out;
    out << std::boolalpha << std::setprecision(10) << "{\"scene\":\"" << scene
        << "\",\"sim_time\":" << now << ",\"cabinets\":[";
    bool first = true;
    for (const auto &c : cabinets) {
      if (!first) out << ',';
      first = false;
      out << "{\"id\":\"" << c.name << "\",\"fault\":" << c.state.fault
          << ",\"armed\":" << c.state.armed << ",\"services\":" << c.state.services
          << ",\"press_m\":" << Position(c.press, ecm)
          << ",\"rotation_rad\":" << Position(c.rotor, ecm)
          << ",\"meter_kwh\":" << c.meter << ",\"temperature_c\":" << c.temperature << '}';
    }
    out << "]";
    if (hasDoor) out << ",\"door\":{\"authorized\":" << doorState.Authorized(now)
                     << ",\"latched\":" << doorState.latched
                     << ",\"opened\":" << doorState.opened
                     << ",\"hinge_rad\":" << Position(hinge, ecm) << '}';
    out << '}';
    gz::msgs::StringMsg msg; msg.set_data(out.str()); truth.Publish(msg);
  }
};
}
GZ_ADD_PLUGIN(hx_grid::GridInteractions, gz::sim::System,
              hx_grid::GridInteractions::ISystemConfigure,
              hx_grid::GridInteractions::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hx_grid::GridInteractions, "hx_grid::GridInteractions")
