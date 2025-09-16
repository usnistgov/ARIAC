#include "ariac_plugins/tool_stand_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::ToolStandPlugin,
  gz::sim::System,
  ariac_plugins::ToolStandPlugin::ISystemConfigure,
  ariac_plugins::ToolStandPlugin::ISystemPreUpdate
)

namespace ariac_plugins{
void ToolStandPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  tool_stand_model = gz::sim::Model(_entity);
  tool_stand_link = tool_stand_model.LinkByName(_ecm, "tool_holder_link");

  gz_node = std::make_shared<gz::transport::Node>();

  std::string lock_service = "/tool_stand/lock";
  if(!gz_node->Advertise(lock_service, &ToolStandPlugin::lock_cb, this)){
    gzmsg << "Error advertising /tool_stand/lock";
  }

  std::string unlock_service = "/tool_stand/unlock";
  if(!gz_node->Advertise(unlock_service, &ToolStandPlugin::unlock_cb, this)){
    gzmsg << "Error advertising /tool_stand/unlock";
  }
}

void ToolStandPlugin::PreUpdate(const gz::sim::UpdateInfo &update_info,
      gz::sim::EntityComponentManager &_ecm)
{
  for (auto& [tool, lock_state] : lock_states){
    if(lock_state == ToolStandLockState::LOCK_REQUESTED){
      detachable_joints[tool] = _ecm.CreateEntity();

      _ecm.CreateComponent(detachable_joints[tool], gz::sim::components::DetachableJoint({tool_stand_link, tool_link_entity[tool], "fixed"}));

      lock_state = ToolStandLockState::LOCKED;
    } else if (lock_state == ToolStandLockState::UNLOCK_REQUESTED){
      _ecm.RequestRemoveEntity(detachable_joints[tool]);

      detachable_joints[tool] = gz::sim::kNullEntity;

      lock_state = ToolStandLockState::UNLOCKED;
    }
  }    
}

bool ToolStandPlugin::lock_cb(
  const gz::msgs::Entity &req, 
  gz::msgs::Boolean &res)
{
  int tool = std::stoi(req.name());
  if (tool_link_entity.find(tool) == tool_link_entity.end()){
    gzmsg << "Invalid tool type requested\n";
    res.set_data(false);
    return false;
  } else if (lock_states[tool] == ToolStandLockState::LOCKED) {
    gzmsg << "Requested tool already locked\n";
    res.set_data(false);
    return false;
  }
  
  tool_link_entity[tool] = req.id();
  lock_states[tool] = ToolStandLockState::LOCK_REQUESTED;

  res.set_data(true);
  return true;
}

bool ToolStandPlugin::unlock_cb(
  const gz::msgs::Entity &req, 
  gz::msgs::Boolean &res
){
  int tool = std::stoi(req.name());
  if (tool_link_entity.find(tool) == tool_link_entity.end()){
    gzmsg << "Invalid tool type requested\n";
    res.set_data(false);
    return false;
  } else if (lock_states[tool] == ToolStandLockState::UNLOCKED) {
    gzmsg << "Requested tool already unlocked\n";
    res.set_data(false);
    return false;
  }
  
  tool_link_entity[tool] = req.id();
  lock_states[tool] = ToolStandLockState::UNLOCK_REQUESTED;
  
  res.set_data(true);
  return true;
}
}