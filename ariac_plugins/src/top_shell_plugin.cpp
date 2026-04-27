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

  std::optional<gz::sim::Entity> assembly_conveyor_entity_opt = _ecm.EntityByName("assembly_conveyor");

  if(!assembly_conveyor_entity_opt.has_value()){
    throw std::runtime_error("Could not find assembly conveyor entity");
  }

  gz::sim::Entity assembly_conveyor_entity = assembly_conveyor_entity_opt.value();
  section_3_link_entity = gz::sim::Model(assembly_conveyor_entity).LinkByName(_ecm, "section_3_belt");

  std::string base_topic = "/world/ariac/model/" + top_shell_model.Name(_ecm) + "/link/base_link/sensor/contact_sensor/contact";

  gz_node->Subscribe(base_topic, &TopShellPlugin::base_contact_msg_cb, this);

  shell_base_link = top_shell_model.LinkByName(_ecm, "base_link");
}

void TopShellPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(teleport_state != TopShellTeleportState::FINISHED){
    switch(teleport_state){
      case TopShellTeleportState::IDLE:
        break;
      case TopShellTeleportState::READY:
      {
        if (bottom_shell_entity == gz::sim::kNullEntity){
          break;
        }

        gz::sim::Entity bottom_shell_has_parent = false;
        _ecm.Each<gz::sim::components::DetachableJoint>(
          [&](
            const gz::sim::Entity &entity,
            const gz::sim::components::DetachableJoint *detachable_joint
          ) -> bool {
            if(detachable_joint->Data().childLink == gz::sim::Model(bottom_shell_entity).LinkByName(_ecm, "base_link")){
              bottom_shell_has_parent = true;
              return false;
            }
            return true;
          }
        );

        if(bottom_shell_has_parent){
          break;
        }
        
        std::optional<gz::math::Pose3d> bottom_shell_pose = gz::sim::Link(gz::sim::Model(bottom_shell_entity).LinkByName(_ecm, "base_link")).WorldPose(_ecm);
        
        if(!bottom_shell_pose.has_value()){
          break;
        }

        gz::math::Pose3d target_pose = flipped_bottom_shell_pose;

        target_pose.SetX(bottom_shell_pose.value().X());
        target_pose.SetY(bottom_shell_pose.value().Y());
        target_pose.SetZ(bottom_shell_pose.value().Z());
        gz::sim::Model(bottom_shell_entity).SetWorldPoseCmd(_ecm, target_pose);
        teleport_state = TopShellTeleportState::JOINT_NEEDED;

        teleport_step = _info.iterations;
        
        break;
      }
      case TopShellTeleportState::JOINT_NEEDED:

        lock_joint = _ecm.CreateEntity();

        _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({
          section_3_link_entity, 
          gz::sim::Model(bottom_shell_entity).LinkByName(_ecm, "base_link"), 
          "fixed"}));

        teleport_state = TopShellTeleportState::JOINT_REMOVAL;

        break;
      case TopShellTeleportState::JOINT_REMOVAL:
        if(_info.iterations - teleport_step < 100){
          break;
        }

        _ecm.RequestRemoveEntity(lock_joint);

        lock_joint = gz::sim::kNullEntity;

        teleport_state = TopShellTeleportState::FINISHED;
        break;
      
      case TopShellTeleportState::FINISHED:
        break;

      default:
        break;
    }
  }
  
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
    bottom_shell_entity = module_shell_entity;
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

void TopShellPlugin::base_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  if(teleport_state != TopShellTeleportState::IDLE){
    return;
  }
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();
    if (collision.find("section_3_belt") != std::string::npos){
      teleport_state = TopShellTeleportState::READY;
    }
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