#include "ariac_plugins/tool_changer_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::ToolChangerPlugin,
  gz::sim::System,
  ariac_plugins::ToolChangerPlugin::ISystemConfigure,
  ariac_plugins::ToolChangerPlugin::ISystemPreUpdate,
  ariac_plugins::ToolChangerPlugin::ISystemUpdate
)

using namespace ariac_plugins;

ToolChangerPlugin::~ToolChangerPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}


void ToolChangerPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  // Read sdf tags
  std::string ros_namespace = "";
  if (_sdf->HasElement("ros")){
    if (_sdf->GetElementImpl("ros")->HasElement("namespace")){
      ros_namespace = _sdf->GetElementImpl("ros")->Get<std::string>("namespace");
    }
  }

  if (!rclcpp::ok()){
      rclcpp::init(0, nullptr);
  }
  
  std::string tool_changer_link_name = "tool_changer_link";

  std::string robot_ee_contact_topic = "/world/ariac/model/assembly_robot_2/link/tool_changer_link/sensor/contact_sensor/contact";

  auto model = gz::sim::Model(_entity);
  
  tool_changer_link = model.LinkByName(_ecm, tool_changer_link_name);

  current_status.attached_tool = VacuumTools::NONE;

  gz_node = std::make_shared<gz::transport::Node>();

  gz_node->Subscribe(robot_ee_contact_topic, &ToolChangerPlugin::tool_changer_contact_msg_cb, this);

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

  attach_tool_srv = ros_node->create_service<AttachToolSrv>(
    "attach_tool",
    std::bind(&ToolChangerPlugin::attach_tool_srv_cb, this, std::placeholders::_1, std::placeholders::_2)
  );

  detach_tool_srv = ros_node->create_service<Trigger>(
    "detach_tool",
    std::bind(&ToolChangerPlugin::detach_tool_srv_cb, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create publisher and timer
  status_pub = ros_node->create_publisher<ToolChangerStatus>("status", 10);

  pub_timer = ros_node->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ToolChangerPlugin::publish_status_cb, this)
  );

  gz::sim::Entity vg_2_model = _ecm.EntityByComponents(
    gz::sim::components::Model(), gz::sim::components::Name("vg_2"));
  
  tool_link_entities[VacuumTools::VG_2] = _ecm.EntityByComponents(
    gz::sim::components::Link(), gz::sim::components::ParentEntity(vg_2_model),
    gz::sim::components::Name("base_link")
  );
  
  gz::sim::Entity vg_4_model = _ecm.EntityByComponents(
    gz::sim::components::Model(), gz::sim::components::Name("vg_4"));

  tool_link_entities[VacuumTools::VG_4] = _ecm.EntityByComponents(
    gz::sim::components::Link(), gz::sim::components::ParentEntity(vg_4_model),
    gz::sim::components::Name("base_link")
  );

  tool_links[VacuumTools::VG_2] = gz::sim::Link(tool_link_entities[VacuumTools::VG_2]);
  tool_links[VacuumTools::VG_4] = gz::sim::Link(tool_link_entities[VacuumTools::VG_4]);

  initial_poses[VacuumTools::VG_2] = tool_links[VacuumTools::VG_2].WorldPose(_ecm).value();
  initial_poses[VacuumTools::VG_4] = tool_links[VacuumTools::VG_4].WorldPose(_ecm).value();

  contact_link_joints[VacuumTools::VG_2] = gz::sim::Joint(gz::sim::Model(vg_2_model).JointByName(_ecm, "contact_joint"));
  contact_link_joints[VacuumTools::VG_4] = gz::sim::Joint(gz::sim::Model(vg_4_model).JointByName(_ecm, "contact_joint"));
}

void ToolChangerPlugin::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  if(lock_state == ToolLockState::LOCK_REQUESTED){
    unlock_tool_from_stand(current_status.attached_tool);

    lock_joint = _ecm.CreateEntity();
    _ecm.CreateComponent(
      lock_joint, 
      gz::sim::components::DetachableJoint({tool_changer_link, tool_link_entities[current_status.attached_tool], "fixed"})
    );

    contact_link_joints[current_status.attached_tool].ResetPosition(_ecm, {-0.002});

    lock_state = ToolLockState::LOCKED;
  } else if (lock_state == ToolLockState::UNLOCK_REQUESTED) {
    _ecm.RequestRemoveEntity(lock_joint);
    lock_joint = gz::sim::kNullEntity;

    lock_state = ToolLockState::UNLOCKED;

    lock_tool_to_stand(current_status.attached_tool);

    contact_link_joints[current_status.attached_tool].ResetPosition(_ecm, {0.0});
  
    current_status.attached_tool = VacuumTools::NONE;
  }
}

void ToolChangerPlugin::Update(const gz::sim::UpdateInfo &update_info, gz::sim::EntityComponentManager &_ecm)
{
  current_poses[VacuumTools::VG_2] = tool_links[VacuumTools::VG_2].WorldPose(_ecm).value();
  current_poses[VacuumTools::VG_4] = tool_links[VacuumTools::VG_4].WorldPose(_ecm).value();
}

void ToolChangerPlugin::attach_tool_srv_cb(const AttachToolReqPtr request, AttachToolResPtr response){
  if(current_status.attached_tool != VacuumTools::NONE){
    response->success = false;
    response->message = "Tool already attached";
    return;
  } 
  
  if(!tool_in_contact[request->tool]){
    response->message = "Robot not in contact with tool";
    response->success = false;
    return;
  }
  
  current_status.attached_tool = request->tool;
  lock_state = ToolLockState::LOCK_REQUESTED;
  response->message = "Tool attached";
  response->success = true;
  
}

void ToolChangerPlugin::publish_status_cb()
{
  status_pub->publish(current_status);
}

void ToolChangerPlugin::detach_tool_srv_cb(const TriggerReqPtr request, TriggerResPtr response){
  if(current_status.attached_tool == VacuumTools::NONE){
    response->success = false;
    response->message = "No tool attached";
    return;
  } else if (!current_poses[current_status.attached_tool].Equal(initial_poses[current_status.attached_tool], 0.005)){
    response->success = false;
    response->message = "Attached tool not in holder";
    return;
  }

  lock_state = ToolLockState::UNLOCK_REQUESTED;
  response->success = true;
  response->message = "Tool detached";
}

void ToolChangerPlugin::tool_changer_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();

    if (collision.find("vg_2") != std::string::npos){
      tool_in_contact[VacuumTools::VG_2] = true;
      return;
    }

    if (collision.find("vg_4") != std::string::npos) {
      tool_in_contact[VacuumTools::VG_4] = true;
      return;
    }  
  }
  tool_in_contact[VacuumTools::VG_2] = false;
  tool_in_contact[VacuumTools::VG_4] = false;
}

bool ToolChangerPlugin::lock_tool_to_stand(int tool){
  gz::msgs::Entity req;
  req.set_type(gz::msgs::Entity::LINK);
  req.set_name(std::to_string(tool));
  req.set_id(tool_link_entities[tool]);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 10000;

  bool executed = gz_node->Request("/tool_stand/lock", req, timeout, res, result);

  if (executed){
    if(result){
      RCLCPP_INFO_STREAM(ros_node->get_logger(), tool << " successfully locked to tool stand");
    } else {
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), tool << " could not be locked to tool stand");
    }
  } else {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Service to lock " << tool << " timed out");
  }

  return result;
}

bool ToolChangerPlugin::unlock_tool_from_stand(int tool){
  gz::msgs::Entity req;
  req.set_type(gz::msgs::Entity::LINK);
  req.set_name(std::to_string(tool));
  req.set_id(tool_link_entities[tool]);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;

  bool executed = gz_node->Request("/tool_stand/unlock", req, timeout, res, result);

  if (executed){
    if(result){
      RCLCPP_INFO_STREAM(ros_node->get_logger(), tool << " successfully unlocked from tool stand");
    } else {
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), tool << " could not be unlocked from tool stand");
    }
  } else {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Service to unlock " << tool << " timed out");
  }

  return result;
}

