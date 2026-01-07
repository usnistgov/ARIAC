#include <ariac_plugins/bottom_shell_plugin.hpp>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    ariac_plugins::BottomShellPlugin,
    gz::sim::System,
    ariac_plugins::BottomShellPlugin::ISystemPreUpdate,
    ariac_plugins::BottomShellPlugin::ISystemConfigure
  )

namespace ariac_plugins{

void BottomShellPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  bottom_shell_model = gz::sim::Model(_entity);
  bottom_shell_entity = _entity;

  gz_node = std::make_shared<gz::transport::Node>();

  // Subscribe to slot contact topics
  std::string topic = "/world/ariac/model/" + bottom_shell_model.Name(_ecm) + "/link/slot_{n}_link/sensor/collision_detector/contact";

  for (int i=1; i<=4; i++) {
    std::string name = topic;
    topic_names[i] = name.replace(topic.find("{n}"), 3, std::to_string(i));
  }
  
  gz_node->Subscribe(topic_names[1], &BottomShellPlugin::slot_1_contact_msg_cb, this);
  gz_node->Subscribe(topic_names[2], &BottomShellPlugin::slot_2_contact_msg_cb, this);
  gz_node->Subscribe(topic_names[3], &BottomShellPlugin::slot_3_contact_msg_cb, this);
  gz_node->Subscribe(topic_names[4], &BottomShellPlugin::slot_4_contact_msg_cb, this);

  // Subscribe to base contact topic
  std::string base_topic = "/world/ariac/model/" + bottom_shell_model.Name(_ecm) + "/link/base_link/sensor/contact_sensor/contact";

  gz_node->Subscribe(base_topic, &BottomShellPlugin::base_contact_msg_cb, this);

  shell_base_link = bottom_shell_model.LinkByName(_ecm, "base_link");

  if(!_ecm.EntityByName("assembly_conveyor").has_value()){
    throw std::runtime_error("Unable to find assembly conveyor");
  }

  gz::sim::Entity conveyor_entity = _ecm.EntityByName("assembly_conveyor").value();

  section_3_link_entity = gz::sim::Model(conveyor_entity).LinkByName(_ecm, "section_3_belt");
}

void BottomShellPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(!component_set){
    if (!_ecm.EntityHasComponentType(bottom_shell_entity, Module::typeId)) {
      _ecm.CreateComponent(bottom_shell_entity, Module());
    }

    auto module = _ecm.Component<Module>(bottom_shell_entity);
    if (module)
    {
      auto currentState = module->Data();
      currentState.bottom_shell_entity = bottom_shell_entity;
      module->SetData(currentState, ariac_components::Module::equal);
    }
    component_set = true;
  }
  for (const auto& [slot, cell] : cell_in_slot) {
    if (slot_teleported[slot]) { continue;};
    if (slot_locked[slot]) {continue;}

    if (cell.first) {
      // Lock cell in place
      std::optional<gz::sim::v8::Entity> cell_entity = _ecm.EntityByName(cell.second);
      
      if (!cell_entity.has_value()) {
        throw std::runtime_error("Unable to locate cell model");
      }

      cells_to_teleport.push(std::make_pair(slot, gz::sim::Model(cell_entity.value())));
    }
  }

  while(!cells_to_lock.empty()){
    CellToLock cell_to_lock = cells_to_lock.front();

    auto cell_link = cell_to_lock.cell_model.LinkByName(_ecm, "base_link");
    lock_joints[cell_to_lock.slot] = _ecm.CreateEntity();

    _ecm.CreateComponent(lock_joints[cell_to_lock.slot], gz::sim::components::DetachableJoint({shell_base_link, cell_link, "fixed"}));

    auto module = _ecm.Component<Module>(bottom_shell_entity);

    cells_to_lock.pop();

    if (module){
      auto currentState = module->Data();
        
      currentState.cell_entities[cell_to_lock.slot] = cell_to_lock.cell_model.Entity();

      if (cell_to_lock.direction == "up"){
        currentState.cell_orientation[cell_to_lock.slot] = ariac_components::CellOrientation::UP;
      } else {
        currentState.cell_orientation[cell_to_lock.slot] = ariac_components::CellOrientation::DOWN;
      }
      module->SetData(
        currentState,
        ariac_components::Module::equal
      );

      slot_locked[cell_to_lock.slot] = true;

      gz_node->Unsubscribe(topic_names[cell_to_lock.slot]);
    }
  }

  while(!cells_to_teleport.empty()){
    auto opt_shell_world_pose = gz::sim::Link(gz::sim::Model(bottom_shell_entity).LinkByName(_ecm, "base_link")).WorldPose(_ecm);
    if(!opt_shell_world_pose.has_value()){
      break;
    }
    std::pair<int, gz::sim::Model> slot_cell_model_pair = cells_to_teleport.front();

    int slot = slot_cell_model_pair.first;
    gz::sim::Model cell_model = slot_cell_model_pair.second;
    
    gz::math::Vector3d cell_rotate_vector;
    gz::math::Vector3d shell_rotate_vector;
    auto opt_cell_world_pose = gz::sim::Link(cell_model.LinkByName(_ecm, "base_link")).WorldPose(_ecm);

    gz::math::Pose3d base_pose;
    
    float cell_z_offset = 0.0;
    std::string direction;
    if(opt_cell_world_pose.has_value()){
      shell_rotate_vector = opt_shell_world_pose.value().Rot().RotateVector(gz::math::Vector3d::UnitZ);
      cell_rotate_vector = opt_cell_world_pose.value().Rot().RotateVector(gz::math::Vector3d::UnitZ);

      if(shell_rotate_vector.Dot(cell_rotate_vector) < 0){
        base_pose = up_down_poses["down"];
        direction = "down";
        cell_z_offset = 0.07;
      } else {
        direction = "up";
        base_pose = up_down_poses["up"];
      }
    } else {
      break;
    }

    gz::math::Pose3d cell_pose = gz::math::Pose3d(
      opt_shell_world_pose.value().X() + slot_offsets[slot],
      opt_shell_world_pose.value().Y(),
      opt_shell_world_pose.value().Z() + cell_z_offset + 0.0031,
      base_pose.Roll(),
      base_pose.Pitch(),
      opt_cell_world_pose.value().Yaw()
    );
    
    cell_model.SetWorldPoseCmd(_ecm, cell_pose);

    slot_teleported[slot] = true;

    cells_to_teleport.pop();

    CellToLock cell_to_lock;
    cell_to_lock.cell_model = cell_model;
    cell_to_lock.direction = direction;
    cell_to_lock.slot = slot;
    cells_to_lock.push(cell_to_lock);
  }

  switch(teleport_state){
    case BottomShellTeleportState::IDLE:
      break;
    case BottomShellTeleportState::READY:
      bottom_shell_model.SetWorldPoseCmd(_ecm, section_3_pose);
      teleport_state = BottomShellTeleportState::JOINT_NEEDED;

      teleport_step = _info.iterations;
      
      gzmsg << "Teleported shell\n";
      break;
    case BottomShellTeleportState::JOINT_NEEDED:

      gzmsg << "Creating joint\n";

      lock_joint = _ecm.CreateEntity();

      _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({
        section_3_link_entity, 
        bottom_shell_model.LinkByName(_ecm, "base_link"), 
        "fixed"}));

      teleport_state = BottomShellTeleportState::JOINT_REMOVAL;
      gzmsg << "Joint created\n";
      break;
    case BottomShellTeleportState::JOINT_REMOVAL:
      if(_info.iterations - teleport_step < 100){
        break;
      }


      gzmsg << "Deleting joint\n";

      _ecm.RequestRemoveEntity(lock_joint);

      lock_joint = gz::sim::kNullEntity;

      teleport_state = BottomShellTeleportState::FINISHED;
      break;
    
    case BottomShellTeleportState::FINISHED:
      break;

    default:
      break;
  }

}

void BottomShellPlugin::slot_1_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[1] = std::make_pair(true, cell.value());
  }
}

void BottomShellPlugin::slot_2_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[2] = std::make_pair(true, cell.value());
  }
}

void BottomShellPlugin::slot_3_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[3] = std::make_pair(true, cell.value());
  }
}

void BottomShellPlugin::slot_4_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  auto cell = get_cell_in_contact(_gz_contacts_msg);

  if (cell.has_value()) {
    cell_in_slot[4] = std::make_pair(true, cell.value());
  }
}

void BottomShellPlugin::base_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();
    if (collision.find("section_3_belt") != std::string::npos && teleport_state == BottomShellTeleportState::IDLE){
      teleport_state = BottomShellTeleportState::READY;
    }
  }
}

std::optional<std::string> BottomShellPlugin::get_cell_in_contact(const gz::msgs::Contacts &_gz_contacts_msg)
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