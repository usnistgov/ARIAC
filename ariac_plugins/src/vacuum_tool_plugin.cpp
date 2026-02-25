#include "ariac_plugins/vacuum_tool_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::VacuumToolPlugin,
  gz::sim::System,
  ariac_plugins::VacuumToolPlugin::ISystemConfigure,
  ariac_plugins::VacuumToolPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

VacuumToolPlugin::~VacuumToolPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}


void VacuumToolPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{ 
  // Read SDF tags
  std::string ros_namespace = "";
  if (_sdf->HasElement("ros")){
    if (_sdf->GetElementImpl("ros")->HasElement("namespace")){
      ros_namespace = _sdf->GetElementImpl("ros")->Get<std::string>("namespace");
    }
  }

  tool_type = _sdf->Get<int>("tool_type");

  // Read trial component
  if (!_ecm.EntityHasComponentType(world_entity, gz::sim::components::Trial::typeId)){
    throw(std::runtime_error("Could not find trial component"));
  }

  auto trial_component = _ecm.Component<gz::sim::components::Trial>(world_entity);
  if (trial_component == nullptr) {
    throw(std::runtime_error("Could not find trial component"));
  }

  // Get challenges
  for (auto const &m: trial_component->Data().vacuum_tool_malfunctions) {
    if (m.tool == tool_type) {
      malfunctions.push_back(std::make_pair(m.grasp_occurrence, false));
    }
  }
  
  // GZ setup
  auto model = gz::sim::Model(_entity);
  gripper_base_link = model.LinkByName(_ecm, "base_link");

  gz_node = std::make_shared<gz::transport::Node>();

  std::vector<std::string> topic_names; 
  std::string topic = model.Name(_ecm) + "/suction_{n}_link/contact_sensor";

  int suction_cup_count = tool_type == VacuumTools::VG_2 ? 2 : 4;
  for(int i = 1; i <= suction_cup_count; i++){
    std::string name = topic;
    topic_names.push_back(name.replace(topic.find("{n}"), 3, std::to_string(i)));
  }
  
  if (tool_type == VacuumTools::VG_2) {
    gz_node->Subscribe(topic_names[0], &VacuumToolPlugin::contact_sensor_1_cb, this);
    gz_node->Subscribe(topic_names[1], &VacuumToolPlugin::contact_sensor_2_cb, this);
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_1_joint")));
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_2_joint")));
  } else if(tool_type == VacuumTools::VG_4) {
    gz_node->Subscribe(topic_names[0], &VacuumToolPlugin::contact_sensor_1_cb, this);
    gz_node->Subscribe(topic_names[1], &VacuumToolPlugin::contact_sensor_2_cb, this);
    gz_node->Subscribe(topic_names[2], &VacuumToolPlugin::contact_sensor_3_cb, this);
    gz_node->Subscribe(topic_names[3], &VacuumToolPlugin::contact_sensor_4_cb, this);
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_1_joint")));
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_2_joint")));
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_3_joint")));
    suction_cup_joints.push_back(gz::sim::Joint(gz::sim::Model(model).JointByName(_ecm, "suction_4_joint")));
  }

  // ROS setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared(model.Name(_ecm) + "_plugin_node", ros_namespace);

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  if (tool_type == VacuumTools::VG_2) {
    attach_srv = ros_node->create_service<Trigger>(
      "grasp",
      std::bind(&VacuumToolPlugin::vg_2_attach_cb, this, std::placeholders::_1, std::placeholders::_2)
    );
  } else if(tool_type == VacuumTools::VG_4) {
    attach_srv = ros_node->create_service<Trigger>(
      "grasp",
      std::bind(&VacuumToolPlugin::vg_4_attach_cb, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  detach_srv = ros_node->create_service<Trigger>(
    "release",
    std::bind(&VacuumToolPlugin::detach_object_cb, this, std::placeholders::_1, std::placeholders::_2)
  );

  lock_tool_to_stand();
}

void VacuumToolPlugin::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  switch (lock_state)
  {
  case VacuumToolLockState::UNLOCKED: {
    for(gz::sim::Joint joint : suction_cup_joints){
      joint.SetVelocity(_ecm, {-0.0008});
    }
    break;
  }
  case VacuumToolLockState::LOCK_REQUESTED: {
    // Get link entity for bottom shell
    auto model = _ecm.EntityByName(attach_shell_name);
    
    if (!model.has_value()) {
      gzerr << "Unable to locate shell model: " << attach_shell_name << std::endl;
      lock_state = VacuumToolLockState::UNLOCKED;
      break;
    }
    
    auto shell_link = gz::sim::Model(model.value()).LinkByName(_ecm, "base_link");

    // Lock joint
    lock_joint = _ecm.CreateEntity();

    _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({gripper_base_link, shell_link, "fixed"}));

    lock_state = VacuumToolLockState::LOCKED;
    
    grasp_occurrence++;
    
    break;
  }
  case VacuumToolLockState::UNLOCK_REQUESTED:
    // Unlock joint
    _ecm.RequestRemoveEntity(lock_joint);
    lock_joint = gz::sim::kNullEntity;

    lock_state = VacuumToolLockState::UNLOCKED;
    for(gz::sim::Joint joint : suction_cup_joints){
      joint.ResetPosition(_ecm, {0.0});
    }
    break;
  }

  // If malfunction is active check if should be cleared
  if (malfunction_active && std::all_of(pad_contacts.begin(), pad_contacts.end(),
      [](const auto& entry) { return !entry.second.in_contact; })) {
    clear_malfunction();
  }
}

void VacuumToolPlugin::vg_2_attach_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  if(lock_state == VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Already holding object";
    return;
  }
  
  if (!pad_contacts[1].in_contact && !pad_contacts[2].in_contact) {
    response->success = false;
    response->message = "Suction cups must be in contact with the shell";
    return;
  }
  
  if (should_malfunction()) {
    response->success = false;
    response->message = "Vacuum gripper is malfunctioning";
    return;
  }

  attach_shell_name = pad_contacts[1].model_name;
  
  lock_state = VacuumToolLockState::LOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::LOCKED);
  response->message = response->success ? "Top shell attached" : "Unable to grasp object";
}

void VacuumToolPlugin::vg_4_attach_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  if(lock_state == VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Already holding object";
    return;
  }
  
  if ((!pad_contacts[1].in_contact && !pad_contacts[2].in_contact) || (!pad_contacts[3].in_contact && !pad_contacts[4].in_contact)) {
    response->success = false;
    response->message = "Suction cups must be in contact with the shell";
    return;
  }  
  
  if (should_malfunction()) {
    response->success = false;
    response->message = "Vacuum gripper is malfunctioning";
    return;
  }

  if(pad_contacts[1].model_name.find("bottom") != std::string::npos){
    attach_shell_name = pad_contacts[1].model_name;
  } else if (pad_contacts[2].model_name.find("bottom") != std::string::npos){
    attach_shell_name = pad_contacts[2].model_name;
  } else if (pad_contacts[3].model_name.find("bottom") != std::string::npos){
    attach_shell_name = pad_contacts[3].model_name;
  } else {
    attach_shell_name = pad_contacts[4].model_name;
  }

  lock_state = VacuumToolLockState::LOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::LOCKED);
  response->message = response->success ? "Module attached" : "Unable to grasp object";
}

void VacuumToolPlugin::detach_object_cb(const TriggerReqPtr request, TriggerResPtr response)
{
  if(lock_state != VacuumToolLockState::LOCKED){
    response->success = false;
    response->message = "Tool not holding object";
    return;
  }
  
  lock_state = VacuumToolLockState::UNLOCK_REQUESTED;

  response->success = wait_for_state(VacuumToolLockState::UNLOCKED);
  response->message = response->success ? "Object detached" : "Unable to release object";
}

bool VacuumToolPlugin::lock_tool_to_stand(){
  gz::msgs::Entity req;
  req.set_type(gz::msgs::Entity::LINK);
  req.set_name(std::to_string(tool_type));
  req.set_id(gripper_base_link);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 10000;

  bool executed = gz_node->Request("/tool_stand/lock", req, timeout, res, result);

  if (executed){
    if(result){
      RCLCPP_INFO_STREAM(ros_node->get_logger(), tool_type << " successfully locked to tool stand");
    } else {
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), tool_type << " could not be locked to tool stand");
    }
  } else {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Service to lock " << tool_type << " timed out");
  }

  return result;
}

void VacuumToolPlugin::contact_sensor_1_cb(const gz::msgs::StringMsg_V &msg)
{
  auto shell = shell_in_contact(msg);

  pad_contacts[1] = PadContact{shell.has_value(), shell.has_value() ? shell.value() : ""};
}

void VacuumToolPlugin::contact_sensor_2_cb(const gz::msgs::StringMsg_V &msg)
{
  auto shell = shell_in_contact(msg);

  pad_contacts[2] = PadContact{shell.has_value(), shell.has_value() ? shell.value() : ""};
}

void VacuumToolPlugin::contact_sensor_3_cb(const gz::msgs::StringMsg_V &msg)
{
  auto shell = shell_in_contact(msg);

  pad_contacts[3] = PadContact{shell.has_value(), shell.has_value() ? shell.value() : ""};
}

void VacuumToolPlugin::contact_sensor_4_cb(const gz::msgs::StringMsg_V &msg)
{
  auto shell = shell_in_contact(msg);

  pad_contacts[4] = PadContact{shell.has_value(), shell.has_value() ? shell.value() : ""};
}

std::optional<std::string> VacuumToolPlugin::shell_in_contact(const gz::msgs::StringMsg_V &msg)
{
  auto data = msg.data();

  if(data.empty()){
    return std::nullopt;
  }

  return data.at(0);
}


bool VacuumToolPlugin::wait_for_state(VacuumToolLockState state)
{
  rclcpp::Time start_time = ros_node->now();
  rclcpp::Rate rate(100);
  while(rclcpp::ok()){
    if (lock_state == state) {
      break;
    } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(3.0)) {
      gzerr << "Timed out during request\n";
      return false;
    }
    rate.sleep();
  };

  return lock_state == state;
}



bool VacuumToolPlugin::should_malfunction()
{
  // Check if malfunction challenge should activate based on current grasp occurrence
  if (malfunctions.empty()) {
    return false;
  }

  malfunction_active = std::find_if(malfunctions.begin(), malfunctions.end(), 
    [this](const auto& p) { return !p.second && p.first == grasp_occurrence; }
  ) != malfunctions.end();
  
  return malfunction_active;
}

void VacuumToolPlugin::clear_malfunction()
{
  // Clear malfunction challenge based on current grasp occurrence
  auto it = std::find_if(malfunctions.begin(), malfunctions.end(),
    [this](const auto& p) { return p.first == grasp_occurrence; });

  if (it != malfunctions.end()) {
    malfunctions[std::distance(malfunctions.begin(), it)].second = true;
  }

}
