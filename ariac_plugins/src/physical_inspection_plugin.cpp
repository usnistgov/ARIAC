#include <ariac_plugins/physical_inspection_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::PhysicalInspectionPlugin,
  gz::sim::System,
  ariac_plugins::PhysicalInspectionPlugin::ISystemPreUpdate,
  ariac_plugins::PhysicalInspectionPlugin::ISystemConfigure
)

using namespace ariac_plugins;

PhysicalInspectionPlugin::~PhysicalInspectionPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void PhysicalInspectionPlugin::Configure(
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

  max_speed = (_sdf->HasElement("max_speed")) ? _sdf->Get<double>("max_speed") : 0.5;
  
  std::string joint_name = (_sdf->HasElement("joint_name")) ? _sdf->Get<std::string>("joint_name") : "door_joint";

  // Create inspection results component
  model_entity = _entity;
  _ecm.CreateComponent(model_entity, gz::sim::components::InspectionResults(inspection_results));

  // Read defect config file
  std::string share_dir = ament_index_cpp::get_package_share_directory("ariac_setup");
  YAML::Node config = YAML::LoadFile(share_dir + "/config/defects.yaml"); 
  
  YAML::Node defect_types_node = config["DEFECT_TYPES"];

  if (!defect_types_node.IsDefined() || !defect_types_node.IsMap()){
    throw std::runtime_error("Defect Types not found in config");
  }

  for (const auto& defect_type_node : defect_types_node){
    int defect_type = defect_type_node.first.as<int>();
    
    YAML::Node defects_list_node = defect_type_node.second["DEFECTS"];

    if (!defects_list_node.IsDefined() || !defects_list_node.IsSequence()){
      throw std::runtime_error("Error reading defects in config");
    }

    std::vector<ariac_interfaces::msg::CellDefect> defects_vector;

    for (const auto& defect : defects_list_node){
      ariac_interfaces::msg::CellDefect d;

      if (!defect["TYPE"].IsDefined() || !defect["THETA"].IsDefined() || !defect["Z"].IsDefined()) {
        throw std::runtime_error("Defect not properly structured");
      }

      d.defect_type = defect["TYPE"].as<int>();
      d.theta = defect["THETA"].as<double>();
      d.z = defect["Z"].as<double>(); 

      defects_vector.push_back(d);
    }

    defect_info[defect_type] = defects_vector;
  }

  // Create belt joint
  model = gz::sim::Model(_entity);
  door_joint = gz::sim::Joint(model.JointByName(_ecm, joint_name));
  door_joint.EnablePositionCheck(_ecm, true);

  // Create GZ node
  gz_node = std::make_shared<gz::transport::Node>();

  // Subscribe to gz contact topic 
  std::string gz_contact_topic = "/world/ariac/model/inspection_conveyor/link/conveyor_belt/sensor/contact_sensor/contact";
  gz_node->Subscribe(gz_contact_topic, &PhysicalInspectionPlugin::conveyor_contact_msg_cb, this);

  // Create ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared("physical_inspection_plugin", ros_namespace);

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this]() {
    while (rclcpp::ok()) { executor->spin_once(); }
  };

  thread_executor_spin = std::thread(spin);

  // Advertise service
  submission_srv = ros_node->create_service<SubmissionSrv>(
    "submit", 
    std::bind(
      &PhysicalInspectionPlugin::submission_cb,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  
}

void PhysicalInspectionPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) {return;}

  _ecm.SetComponentData<gz::sim::components::InspectionResults>(model_entity, inspection_results);

  switch (status) {

  case InspectionStatus::DOOR_CLOSED:
    // Do nothing
    door_joint.SetVelocity(_ecm, {-max_speed});
    break;

  case InspectionStatus::PROCESSING: {
    // Find the x position of the cell closest to the front of the door
    double x_position = 0;

    current_cell = gz::sim::kNullEntity;
    current_cell_data = std::nullopt;

    for (const std::string& name : cells_on_conveyor) {
      if (!_ecm.EntityByName(name).has_value()) { continue; }

      gz::sim::Entity entity = _ecm.EntityByName(name).value();

      auto pose = gz::sim::Link(gz::sim::Model(entity).LinkByName(_ecm, "base_link")).WorldPose(_ecm);

      if (!pose.has_value()) { 
        gzwarn << "Unable to get pose for model " << name << std::endl;
        continue; 
      }
      
      double cell_x = pose.value().Pos().X();

      if (cell_x > x_position && cell_x < door_x) {
        current_cell = entity;
        x_position = cell_x;
      }
    }

    if (current_cell == gz::sim::kNullEntity) {
      status = InspectionStatus::FINISHED_PROCESSING;
      gzwarn << "Unable to locate correct cell to open door for.\n";
      break;
    }

    bool has_cell_component = _ecm.EntityHasComponentType(current_cell, gz::sim::components::Cell::typeId);
    if (!has_cell_component) {
      status = InspectionStatus::FINISHED_PROCESSING;
      gzwarn << "Unable to get cell component.\n";
      break;
    }

    auto cell_component = _ecm.Component<gz::sim::components::Cell>(current_cell);
    if (!cell_component) {
      status = InspectionStatus::FINISHED_PROCESSING;
      gzwarn << "Cell component is null ptr.\n";
      break;
    }
    
    current_cell_data = cell_component->Data();

    status = InspectionStatus::FINISHED_PROCESSING;

    break;
  }

  case InspectionStatus::FINISHED_PROCESSING:
    // Do nothing
    break;

  case InspectionStatus::WAITING_FOR_CELL: {
    // Wait until cell is at open_position

    auto pose = gz::sim::Link(gz::sim::Model(current_cell).LinkByName(_ecm, "base_link")).WorldPose(_ecm);

    if (!pose.has_value()) { 
      gzwarn << "Unable to get pose for cell\n";
    }

    if (pose.value().Pos().X() >= cell_positions["open_door"]) {
      status = InspectionStatus::DOOR_OPENING;
    }

    break;
  }

  case InspectionStatus::DOOR_OPENING:
    door_joint.SetVelocity(_ecm, {max_speed});

    // Check if door has fully opened
    if (door_joint.Position(_ecm).has_value() && door_joint.Position(_ecm).value().size() > 0) {
      if (door_joint.Position(_ecm).value()[0] >= opened_position) {
        door_joint.SetVelocity(_ecm, {0.0});
        status = InspectionStatus::DOOR_OPEN;
      }
    }

    break;
  
  case InspectionStatus::DOOR_OPEN: {
    // Wait until cell is at close_position
    auto pose = gz::sim::Link(gz::sim::Model(current_cell).LinkByName(_ecm, "base_link")).WorldPose(_ecm);

    if (!pose.has_value()) { 
      gzwarn << "Unable to get pose for cell\n";
    }

    if (pose.value().Pos().X() >= cell_positions["close_door"]) {
      status = InspectionStatus::DOOR_CLOSING;
    }

    break;
  }
  
  case InspectionStatus::DOOR_CLOSING:
    door_joint.SetVelocity(_ecm, {-max_speed});

    // Check if door has fully closed
    if (door_joint.Position(_ecm).has_value() && door_joint.Position(_ecm).value().size() > 0) {
      if (door_joint.Position(_ecm).value()[0] <= closed_position) {
        door_joint.SetVelocity(_ecm, {0.0});
        status = InspectionStatus::DOOR_CLOSED;
      }
    }
    break;  
  }
}

void PhysicalInspectionPlugin::submission_cb(
  const SubmissionSrvReqPtr request,
  SubmissionSrvResPtr response)
{
  if (cells_on_conveyor.empty()) {
    response->success = false;
    response->message = "There are no cells currently on the conveyor.";
    return;
  }

  if (status != InspectionStatus::DOOR_CLOSED) {
    response->success = false;
    response->message = "Door is not closed";
    return;
  }

  status = InspectionStatus::PROCESSING;

  while (status == InspectionStatus::PROCESSING) { }

  validate_report(request->report);

  if (!current_cell_data.has_value()) {
    response->success = false;
    response->message = "Unable to process report.";
    return;
  }

  if (request->report.passed) {
    status = InspectionStatus::WAITING_FOR_CELL;
    response->message = "Passing inspection report received. Door will open for cell.";
  } else {
    status = InspectionStatus::DOOR_CLOSED;
    response->message = "Failing inspection report received. Door will remain closed for cell.";
  }

  response->success = true;
}

void PhysicalInspectionPlugin::conveyor_contact_msg_cb(
  const gz::msgs::Contacts &_gz_contacts_msg)
{
  // Save list of cell names on conveyor
  std::vector<std::string> cell_names;
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();

    if (collision.find("cell") == std::string::npos) { continue; }

    cell_names.push_back(collision.substr(0, collision.find("::")));
  }

  cells_on_conveyor = cell_names;
}

void PhysicalInspectionPlugin::validate_report(const ariac_interfaces::msg::InspectionReport &report) {
  if (!current_cell_data.has_value()) { return; }

  inspection_results.num_reports_submitted++;

  double report_time = ros_node->get_clock()->now().nanoseconds()/1e9 - current_cell_data->time_created;

  inspection_results.avg_report_time += (report_time - inspection_results.avg_report_time) / inspection_results.num_reports_submitted;

  auto cell = current_cell_data.value();

  if (report.passed != cell.defective) { 
    gzmsg << "Inspection report correct\n";
    inspection_results.num_correct_reports++; 
  }

  if (!cell.defective) { return; }

  if (defect_info.find(cell.defect_type) == defect_info.end()) {
    gzwarn << "Defect info for this defect type not in config\n";
    return;
  }

  auto defect_list = defect_info[cell.defect_type];

  int num_correct_defects = 0;

  if (report.defects.empty()) {
    gzmsg << "No defects reported\n";
    return;
  } else if (report.defects.size() != defect_list.size()) {
    gzmsg << "Incorrect number of defects reported\n";
    return;
  }

  for (const ariac_interfaces::msg::CellDefect &reported_defect : report.defects) {
    // Check if defect matches any defect in defect list
    for (const ariac_interfaces::msg::CellDefect &defect : defect_list) {
      if (reported_defect.defect_type != defect.defect_type) { continue; }
      
      // Check if location is correct
      bool height_correct = abs(reported_defect.z - defect.z) <= report_height_threshold;

      double defect_angle = defect.theta + cell.rotation;
      bool rotation_correct = abs(angles::shortest_angular_distance(reported_defect.theta, defect_angle) <= report_angle_threshold);

      

      if (height_correct && rotation_correct) {
        gzmsg << "Defect correctly identified\n";
        num_correct_defects++;
        break;
      }
    }
  }

  if (num_correct_defects == defect_list.size()) {
    gzmsg << "Inspection report classification correct\n";
    inspection_results.num_correct_report_classifications++;
  }
}
