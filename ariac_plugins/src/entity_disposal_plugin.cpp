#include "ariac_plugins/entity_disposal_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::EntityDisposalPlugin,
  gz::sim::System,
  ariac_plugins::EntityDisposalPlugin::ISystemConfigure,
  ariac_plugins::EntityDisposalPlugin::ISystemPreUpdate
  )

using namespace ariac_plugins;

EntityDisposalPlugin::~EntityDisposalPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void EntityDisposalPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{   

  auto model = gz::sim::Model(_entity);

  object_name = model.Name(_ecm);

  auto it = name_to_model_type.find(object_name);
  if (it != name_to_model_type.end()) {
    model_type = it->second;
  }

  contact_link = _sdf->Get<std::string>("contact_link");

  gz_contact_topic = "/world/ariac/model/" + object_name + "/link/" + contact_link + "/sensor/collision_detector/contact";

  gz_node = std::make_shared<gz::transport::Node>();

  gz_node->Subscribe(gz_contact_topic, &EntityDisposalPlugin::contact_msg_cb, this);

  // Create ROS node
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  ros_node = rclcpp::Node::make_shared(object_name +"_disposal_plugin");

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
    executor->spin_once();
    }
    };

  thread_executor_spin = std::thread(spin);

}

void EntityDisposalPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{  

  if (!model_in_contact.has_contact){
    return;
  }

  auto model_entity = _ecm.EntityByName(model_in_contact.name); 

  // Handles object remaining in contact after being deleted
  if (!model_entity.has_value()){
    model_in_contact.has_contact = false;
    model_in_contact.name = "";
    return;
  }

  // Handle penalty
  std::optional<ariac_components::Cell> cell = std::nullopt;
  if (_ecm.EntityHasComponentType(model_entity.value(), gz::sim::components::Cell::typeId)) {
    cell = _ecm.Component<gz::sim::components::Cell>(model_entity.value())->Data();
  }
  
  if (
    model_type != ModelType::RecyclingBin &&
    !(model_type == ModelType::InspectionBin && cell.value().defective)
  ){
    auto penalty = handle_penalty(cell, _info.simTime.count());
    gz::sim::Entity penalty_entity = _ecm.CreateEntity();
    _ecm.CreateComponent(penalty_entity, gz::sim::components::Penalty(penalty));
  }
  
  // Create a vector of all detachable joints in the world
  std::vector<DetachJointInfo> detachable_joints;

  _ecm.Each<gz::sim::components::DetachableJoint>(
    [&](const gz::sim::Entity &joint_entity, const gz::sim::components::DetachableJoint *joint) -> bool
    {
      DetachJointInfo info;

      info.joint = joint_entity;
      info.child = joint->Data().childLink;
      info.parent = joint->Data().parentLink;

      detachable_joints.push_back(info);

      return true;
    }
  );

  // Loop through the detachable joints found to find the link which is never a child link (the ancestor)
  bool has_parent;

  // Setting intial parent link as model base link
  gz::sim::Entity parent_base_link = gz::sim::Model(model_entity.value()).LinkByName(_ecm, "base_link"); 

  do {
    has_parent = false;
    for (auto joint : detachable_joints){
      if (joint.child == parent_base_link){
        parent_base_link = joint.parent; 
        has_parent = true;
      }
    }
  } while (has_parent);

  // Create a vector of all links connected to the ancestor
  bool has_child = true;
  std::vector<gz::sim::Entity> connected_links = {parent_base_link};

  do {
    has_child = false;
    for (auto joint : detachable_joints){
      if (std::find(connected_links.begin(), connected_links.end(), joint.child) != connected_links.end()){
        continue; // continue if the child is already in our connected links vector
      }
      if (std::find(connected_links.begin(), connected_links.end(), joint.parent) != connected_links.end()){
        connected_links.push_back(joint.child);
        has_child = true;
      }
    }
  } while (has_child);

  // Find all model entities for links in connected links and remove them
  for (auto link : connected_links){
    auto parent_entity = _ecm.ParentEntity(link);
    auto entity_name = _ecm.Component<gz::sim::components::Name>(parent_entity);
    _ecm.RequestRemoveEntity(parent_entity); // Remove all marked entities
    gzmsg <<  entity_name->Data() << " removed from " << object_name << std::endl;
  }
}
      
void EntityDisposalPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();

  if (collision.find("cell") != std::string::npos || collision.find("shell") != std::string::npos){
    model_in_contact.name = collision.substr(0, collision.find("::"));
    model_in_contact.has_contact = true;
    }
  }
}

ariac_components::Penalty EntityDisposalPlugin::handle_penalty(
  std::optional<ariac_components::Cell> cell, double time)
{

  ariac_components::Penalty penalty;
  
  switch (model_type) {
  case ModelType::InspectionBin:
    if (cell.has_value()) {
      penalty = ariac_components::Penalty{
        ariac_components::PenaltyType::GOOD_CELL_IN_INSPECTION_BIN,
        time,
        "Non-defective cell: " + model_in_contact.name + " dropped into inspection bin"
      };
    }
    break;
  
  case ModelType::InspectionConveyorBin:
    penalty = ariac_components::Penalty{
      ariac_components::PenaltyType::CELL_IN_CONVEYOR_BIN,
      time,
      model_in_contact.name + " dropped in inspection conveyor bin"
    };
    break;
  
  case ModelType::Other:
    penalty = ariac_components::Penalty{
      ariac_components::PenaltyType::OBJECT_ON_INVALID_SURFACE,
      time,
      model_in_contact.name + " in contact with " + object_name
    };
    break;
  
  default:
    penalty = ariac_components::Penalty{
      ariac_components::PenaltyType::OBJECT_ON_INVALID_SURFACE,
      -1,
      "INVALID PENALTY. IGNORE"
    };
    break;
  }

  return penalty;
}


