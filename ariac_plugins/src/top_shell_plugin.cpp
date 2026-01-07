#include <ariac_plugins/top_shell_plugin.hpp>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  ariac_plugins::TopShellPlugin,
  gz::sim::System,
  ariac_plugins::TopShellPlugin::ISystemPreUpdate,
  ariac_plugins::TopShellPlugin::ISystemConfigure
)

namespace ariac_plugins{

void TopShellPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  top_shell_model = gz::sim::Model(_entity);

  gz_node = std::make_shared<gz::transport::Node>();

  // Subscribe to slot contact topics
  std::string topic = "/world/ariac/model/" + top_shell_model.Name(_ecm) + "/link/slot_{n}_link/sensor/collision_detector/contact";

  std::string name1 = topic;
  std::string name4 = topic;
  topic_names[1] = name1.replace(topic.find("{n}"), 3, std::to_string(1));
  topic_names[4] = name4.replace(topic.find("{n}"), 3, std::to_string(4));
  
  gz_node->Subscribe(topic_names[1], &TopShellPlugin::slot_1_contact_msg_cb, this);
  gz_node->Subscribe(topic_names[4], &TopShellPlugin::slot_4_contact_msg_cb, this);

  shell_base_link = top_shell_model.LinkByName(_ecm, "base_link");
}

void TopShellPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(lock_state == TopShellLockState::LOCKED){
    return;
  }
  
  if(cell_in_slot[1].first && cell_in_slot[4].first){
    // Lock cell in place
    std::optional<gz::sim::Entity> cell_entity = _ecm.EntityByName(cell_in_slot[1].second);
    
    if (!cell_entity.has_value()) {
      throw std::runtime_error("Unable to locate cell model");
    }

    auto cell_link = gz::sim::Model(cell_entity.value()).LinkByName(_ecm, "base_link");

    if (cell_link == 0) {
      throw std::runtime_error("Unable to locate cell link");
    }

    lock_joint = _ecm.CreateEntity();

    _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({cell_link, shell_base_link, "fixed"}));

    auto module_shell_entity = get_base_module_shell(_ecm, cell_entity.value());

    auto module = _ecm.Component<Module>(module_shell_entity);
    auto currentState = module->Data();
    
    currentState.top_shell_entity = top_shell_model.Entity();

    module->SetData(
      currentState,
      ariac_components::Module::equal
    );

    lock_state = TopShellLockState::LOCKED;
  }
}

gz::sim::Entity TopShellPlugin::get_base_module_shell(
  const gz::sim::EntityComponentManager &_ecm, 
  const gz::sim::Entity cell_entity
){
  gz::sim::Entity result;

  _ecm.Each<gz::sim::components::Module>(
    [&](
      const gz::sim::Entity &entity,
      const gz::sim::components::Module *module_state_comp
    ) -> bool {
      for (const auto& pair : module_state_comp->Data().cell_entities) {
        if(cell_entity == pair.second) {
          result = entity;
          return false;
        }
      }
      return true;
    }
  );

  return result;
}

void TopShellPlugin::slot_1_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[1] = std::make_pair(true, cell.value());
  } else {
    cell_in_slot[1] = std::make_pair(false, "");
  }
}

void TopShellPlugin::slot_4_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[4] = std::make_pair(true, cell.value());
  } else {
    cell_in_slot[4] = std::make_pair(false, "");
  }
}

std::optional<std::string> TopShellPlugin::get_cell_in_contact(const gz::msgs::Contacts &_gz_contacts_msg)
{
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();
    if (collision.find("cell") != std::string::npos){
      return collision.substr(0, collision.find("::"));
    }
  }

  return std::nullopt;
}

} // namespace ariac_plugins