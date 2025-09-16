#include <ariac_plugins/shell_insertion_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::ShellInsertionPlugin,
  gz::sim::System,
  ariac_plugins::ShellInsertionPlugin::ISystemConfigure,
  ariac_plugins::ShellInsertionPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

ShellInsertionPlugin::~ShellInsertionPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void ShellInsertionPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  // Read trial component
  if (!_ecm.EntityHasComponentType(world_entity, gz::sim::components::Trial::typeId)){
    throw(std::runtime_error("Could not find trial component"));
  }

  auto trial_component = _ecm.Component<gz::sim::components::Trial>(world_entity);
  if (trial_component == nullptr) {
    throw(std::runtime_error("Could not find trial component"));
  }

  rng.seed(trial_component->Data().seed);
  
  // ROS Setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  // Create ROS node
  ros_node = rclcpp::Node::make_shared("shell_feed_plugin");

  // Setup parameters
  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  // Spin up executor thread
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  // Subscribe to the compeition status topic
  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 
    10,
    std::bind(&ShellInsertionPlugin::competition_status_cb, this, std::placeholders::_1)
  );

  // Create GZ Node
  gz_node = std::make_shared<gz::transport::Node>();

  gz_node->Subscribe(
    "/world/ariac/model/assembly_table/link/stand/sensor/contact_detector/contact",
    &ShellInsertionPlugin::table_contact_msg_cb,
    this
  );

  gz_node->Subscribe(
    "/world/ariac/model/assembly_conveyor/link/section_1_belt/sensor/contact_detector/contact", 
    &ShellInsertionPlugin::conveyor_contact_msg_cb, 
    this
  );

  std::string share_path = ament_index_cpp::get_package_share_directory("ariac_gz");

  model_paths[ShellTypes::TOP] = share_path + "/models/battery_module/top_shell/model.sdf";
  model_paths[ShellTypes::BOTTOM]= share_path + "/models/battery_module/bottom_shell/model.sdf";
}

void ShellInsertionPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) { return; }

  switch(competition_state) {

  case CompetitionStates::PREPARING:
    break;

  case CompetitionStates::READY:
    break;
  
  case CompetitionStates::STARTED:
  case CompetitionStates::ORDERS_COMPLETE:
    if (advertise_services) {
      insert_bottom_shell_srv = ros_node->create_service<Trigger>(
        "insert_bottom_shell",
        std::bind(&ShellInsertionPlugin::insert_bottom_shell_cb, this, std::placeholders::_1, std::placeholders::_2)
      );

      insert_top_shell_srv = ros_node->create_service<Trigger>(
        "insert_top_shell",
        std::bind(&ShellInsertionPlugin::insert_top_shell_cb, this, std::placeholders::_1, std::placeholders::_2)
      );
      advertise_services = false;
      reset_services = true;
    
      break;
    }

    for (const auto &[shell, time] : last_contact_time) {
      if (_info.simTime.count() - time > 1E7){ // 100 milliseconds
        shell_present[shell] = false;
      }
    }

    break;

  case CompetitionStates::ENDED:
    if (reset_services) {
      insert_bottom_shell_srv.reset();
      insert_top_shell_srv.reset();
    }
    break;
  }

}

void ShellInsertionPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg)
{
  competition_state = msg->competition_state;
}

void ShellInsertionPlugin::insert_bottom_shell_cb(const TriggerReqPtr, TriggerResPtr res)
{
  if (shell_present[ShellTypes::BOTTOM]) {
    res->message = "A bottom shell is already on the conveyor";
    res->success = false;
    return;
  }

  spawn_shell(ShellTypes::BOTTOM);
  res->message = "A bottom shell was placed on the conveyor";
  res->success = true;
}

void ShellInsertionPlugin::insert_top_shell_cb(const TriggerReqPtr, TriggerResPtr res)
{
  if (shell_present[ShellTypes::TOP]) {
    res->message = "A top shell is already on the table";
    res->success = false;
    return;
  }

  spawn_shell(ShellTypes::TOP);
  res->message = "A top shell was placed on the table";
  res->success = true;
}

bool ShellInsertionPlugin::shell_in_contact(const gz::msgs::Contacts &msg)
{
  for (int i = 0; i < msg.contact_size(); ++i){
    std::string collision = msg.contact(i).collision2().name();

    if (collision.find("shell") != std::string::npos){
      return true;
    }
  }
  return false;  
}

void ShellInsertionPlugin::table_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
{
  if (shell_in_contact(_gz_contacts_msg)) {
    shell_present[ShellTypes::TOP] = true;
    last_contact_time[ShellTypes::TOP] = rclcpp::Time(
      _gz_contacts_msg.header().stamp().sec(), 
      _gz_contacts_msg.header().stamp().nsec()
    ).nanoseconds();
  }
}

void ShellInsertionPlugin::conveyor_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
{
  if (shell_in_contact(_gz_contacts_msg)) {
    shell_present[ShellTypes::BOTTOM] = true;
    last_contact_time[ShellTypes::BOTTOM] = rclcpp::Time(
      _gz_contacts_msg.header().stamp().sec(), 
      _gz_contacts_msg.header().stamp().nsec()
    ).nanoseconds();
  }
}

std::optional<std::string> ShellInsertionPlugin::generate_shell_sdf(ShellTypes shell)
{ 
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(model_paths[shell].c_str()) != tinyxml2::XML_SUCCESS) {
    return std::nullopt;
  }

  // Convert from XML to string
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  
  return printer.CStr();
}

void ShellInsertionPlugin::spawn_shell(ShellTypes shell){
  gz::msgs::EntityFactory req;

  std::string name = shell_names[shell] + "_" + std::to_string(shell_counts[shell]);

  req.set_name(name);

  auto sdf = generate_shell_sdf(shell);
  
  if (!sdf.has_value()){
    return;
    gzerr << "Unable to read model file for shell";
  }

  req.set_sdf(sdf.value());

  gz::math::Quaterniond q;
  gz::math::Vector3d v = shell_positions[shell];
  
  if (shell == ShellTypes::TOP) {
    v += gz::math::Vector3d({x_offset_distrubution(rng), y_offset_distrubution(rng), 0.0});
    q.SetFromEuler({M_PI, 0.0, angle_distribution(rng)});
  }


  gz::math::Pose3d pose(v, q);
  
  gz::msgs::Set(req.mutable_pose(), pose);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  bool executed = gz_node->Request("/world/ariac/create", req, timeout, res, result);

  if (executed) {
    if (!result && res.data()) {
      gzerr << "Failed request to create entity.";
      return;
    }
  } else {
    gzerr << "Request to create entity from create service timed out.";
    return;
  }

  shell_counts[shell]++;
}