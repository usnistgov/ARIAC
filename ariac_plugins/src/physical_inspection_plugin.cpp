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

  update_cells(_ecm);

  _ecm.SetComponentData<gz::sim::components::InspectionResults>(model_entity, inspection_results);

  switch (door_status) {
    case DoorStatus::CLOSED:
      door_joint.SetVelocity(_ecm, {0.0});
      break;

    case DoorStatus::OPENING:
      door_joint.SetVelocity(_ecm, {max_speed});

      // Check if door has fully opened
      if (door_joint.Position(_ecm).has_value() && door_joint.Position(_ecm).value().size() > 0) {
        if (door_joint.Position(_ecm).value()[0] >= opened_position) {
          door_joint.SetVelocity(_ecm, {0.0});
          door_status = DoorStatus::OPEN;
        }
      }

      break;

    case DoorStatus::OPEN: {
      if (cell_index_at_door == -1){
        break;
      }

      if (cells_on_conveyor[cell_index_at_door].entity == gz::sim::kNullEntity){
        break;
      }

      if (cells_on_conveyor[cell_index_at_door].x_pose >= cell_positions["close_door"]) {
        door_status = DoorStatus::CLOSING;
      }

      break;
    }

    case DoorStatus::CLOSING:
      door_joint.SetVelocity(_ecm, {-max_speed});

      // Check if door has fully closed
      if (door_joint.Position(_ecm).has_value() && door_joint.Position(_ecm).value().size() > 0) {
        if (door_joint.Position(_ecm).value()[0] <= closed_position) {
          cells_on_conveyor.erase(cells_on_conveyor.begin() + cell_index_at_door);
          door_joint.SetVelocity(_ecm, {0.0});
          door_status = DoorStatus::CLOSED;
        }
      }
      break;  
  }

  switch (inspection_status) {

    case InspectionStatus::PROCESSING: {

      int current_cell_index = cell_index_next(true);

      if (cells_on_conveyor[current_cell_index].entity == gz::sim::kNullEntity) {
        inspection_status = InspectionStatus::FINISHED_PROCESSING;
        gzwarn << "All cells on conveyor have report.\n";
        break;
      }

      bool has_cell_component = _ecm.EntityHasComponentType(cells_on_conveyor[current_cell_index].entity, gz::sim::components::Cell::typeId);
      if (!has_cell_component) {
        inspection_status = InspectionStatus::FINISHED_PROCESSING;
        gzwarn << "Unable to get cell component.\n";
        break;
      }

      auto cell_component = _ecm.Component<gz::sim::components::Cell>(cells_on_conveyor[current_cell_index].entity);
      if (!cell_component) {
        inspection_status = InspectionStatus::FINISHED_PROCESSING;
        gzwarn << "Cell component is null ptr.\n";
        break;
      }
      
      cells_on_conveyor[current_cell_index].cell_component = cell_component->Data();

      reported_index = current_cell_index;

      inspection_status = InspectionStatus::FINISHED_PROCESSING;

      break;
    }

    case InspectionStatus::FINISHED_PROCESSING:
      // Do nothing
      break;

    case InspectionStatus::WAITING_FOR_CELL: {
      // Wait until cell is at open_position

      int next_cell_index = cell_index_next();

      if (next_cell_index != -1){
        if(cells_on_conveyor[next_cell_index].x_pose >= cell_positions["open_door"] && cells_on_conveyor[next_cell_index].open_door) {
          cell_index_at_door = next_cell_index;
          door_status = DoorStatus::OPENING;
          set_to_opening = false;
        } else if (cells_on_conveyor[next_cell_index].report_submitted && !cells_on_conveyor[next_cell_index].open_door) {
          // Remove cell from list if it should not be opened
          cells_on_conveyor.erase(cells_on_conveyor.begin() + next_cell_index);
        }
      }

      break;
    }
  }
}

void PhysicalInspectionPlugin::update_cells(gz::sim::EntityComponentManager &_ecm){
  // Update entity
  for (Cell& cell : cells_on_conveyor) {
    if (cell.entity != gz::sim::kNullEntity) {continue;}
    if (!_ecm.EntityByName(cell.name).has_value()) { continue; }

    gz::sim::Entity entity = _ecm.EntityByName(cell.name).value();

    if (entity == gz::sim::kNullEntity){
      continue;
    }

    cell.entity = entity;
  }

  std::vector<int> cell_indicies_to_remove = {};
  for (int i = 0; i < cells_on_conveyor.size(); i++) {
    if (cells_on_conveyor[i].entity == gz::sim::kNullEntity) {continue;}

    auto link_entity = gz::sim::Model(cells_on_conveyor[i].entity).LinkByName(_ecm, "base_link");
    if (link_entity == gz::sim::kNullEntity) {
      cell_indicies_to_remove.push_back(i);
      continue;
    }
    auto pose = gz::sim::Link(link_entity).WorldPose(_ecm);

    if (!pose.has_value()) {
      gzwarn << "Unable to get pose for model " << cells_on_conveyor[i].name << std::endl;
      continue;
    }
    
    double cell_x = pose.value().Pos().X();

    cells_on_conveyor[i].x_pose = cell_x;
  }

  for(int i = cell_indicies_to_remove.size() - 1; i >= 0; i--){
    cells_on_conveyor.erase(cells_on_conveyor.begin()+cell_indicies_to_remove[i]);
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

  inspection_status = InspectionStatus::PROCESSING;

  while (inspection_status == InspectionStatus::PROCESSING) { }

  if(reported_index == -1){
    response->success = false;
    response->message = "Unable to find cell for report.";
    return;
  }

  validate_report(request->report);

  if (!cells_on_conveyor[reported_index].cell_component.has_value()) {
    response->success = false;
    response->message = "Unable to process report.";
    return;
  }

  cells_on_conveyor[reported_index].report_submitted = true;

  inspection_status = InspectionStatus::WAITING_FOR_CELL;
  if (request->report.passed) {
    cells_on_conveyor[reported_index].open_door = true;
    response->message = "Passing inspection report received. Door will open for cell.";
  } else {
    door_status = DoorStatus::CLOSED;
    response->message = "Failing inspection report received. Door will remain closed for cell.";
  }

  reported_index = -1;

  response->success = true;
}

void PhysicalInspectionPlugin::conveyor_contact_msg_cb(
  const gz::msgs::Contacts &_gz_contacts_msg)
{
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    Cell cell_struct;
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();

    if (collision.find("cell") == std::string::npos) { continue; }
    
    std::string cell_name = collision.substr(0, collision.find("::"));
    if (std::find(already_noticed_cells.begin(), already_noticed_cells.end(), cell_name) != already_noticed_cells.end()) {continue;}

    already_noticed_cells.push_back(cell_name);
    cell_struct.name = cell_name;
    cells_on_conveyor.push_back(cell_struct);
  }
}

void PhysicalInspectionPlugin::validate_report(const ariac_interfaces::msg::InspectionReport &report) {
  if (!cells_on_conveyor[reported_index].cell_component.has_value()) { return; }

  inspection_results.num_reports_submitted++;

  double report_time = ros_node->get_clock()->now().nanoseconds()/1e9 - cells_on_conveyor[reported_index].cell_component->time_created;

  inspection_results.avg_report_time += (report_time - inspection_results.avg_report_time) / inspection_results.num_reports_submitted;

  auto cell = cells_on_conveyor[reported_index].cell_component.value();

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

int PhysicalInspectionPlugin::cell_index_next(bool without_report){
  if (cells_on_conveyor.size() == 0){
    return -1;
  }
  
  int min_index=-1;
  float smallest_difference = INFINITY, difference;

  for (int i = 0; i < cells_on_conveyor.size(); i++) {
    if(without_report && cells_on_conveyor[i].report_submitted){
      continue;
    }

    // Small buffer so the door can open
    difference = cell_positions["open_door"] - cells_on_conveyor[i].x_pose + 0.005;

    if(difference > 0 && difference < smallest_difference){
      smallest_difference = difference;
      min_index = i;
    }
  }
  
  return min_index;
}