#include "ariac_plugins/cell_feed_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::CellFeedPlugin,
  gz::sim::System,
  ariac_plugins::CellFeedPlugin::ISystemConfigure,
  ariac_plugins::CellFeedPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

CellFeedPlugin::~CellFeedPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}
  
void CellFeedPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_event_mgr)
{
  // Read trial component
  if (
    !_ecm.EntityHasComponentType(world_entity, gz::sim::components::Trial::typeId) && 
    _ecm.Component<gz::sim::components::Trial>(world_entity) == nullptr)
  {
    throw(std::runtime_error("Could not find trial component"));
  }

  auto trial_info = _ecm.Component<gz::sim::components::Trial>(world_entity);

  seed = trial_info->Data().seed;
  defect_rate = trial_info->Data().defect_rate;
  possible_defects = trial_info->Data().possible_defects;

  // Create feed results component
  model_entity = _entity;
  _ecm.CreateComponent(model_entity, gz::sim::components::FeedResults(feed_results));

  // Read sdf tags
  std::string ros_namespace = "";
  if (_sdf->HasElement("ros")){
    if (_sdf->GetElementImpl("ros")->HasElement("namespace")){
      ros_namespace = _sdf->GetElementImpl("ros")->Get<std::string>("namespace");
    }
  }

  // Find Defects yaml file
  std::string share_dir = ament_index_cpp::get_package_share_directory("ariac_setup");
  YAML::Node config = YAML::LoadFile(share_dir + "/config/defects.yaml"); 
  
  YAML::Node defect_type_node = config["DEFECT_TYPES"];

  if (!defect_type_node.IsDefined()){
    throw std::runtime_error("Defect Types not found in config");
  }

  for (const auto& defect_type : defect_type_node){
    if (!possible_defects.empty() && std::find(possible_defects.begin(), possible_defects.end(), defect_type.first.as<int>()) == possible_defects.end()) {
      continue;
    }

    YAML::Node stl = defect_type.second["DAE_FILE"];

    if (!stl.IsDefined()){
      throw std::runtime_error("Collada not found in defect config");
    } else if (!stl.IsScalar()) {
      throw std::runtime_error("Collada not string");
    }
    
    defect_type_to_visual[defect_type.first.as<int>()] = stl.as<std::string>();
    
    defect_types.push_back(defect_type.first.as<int>());
  }

  // Create Distributions 
  rng.seed(seed); // create a single RNG instance

  defect_distribution.param(std::uniform_real_distribution<double>::param_type(0.0, 1.0));
  defect_type_distribution.param(std::uniform_int_distribution<>::param_type(0, defect_types.size()-1));
  rotation_distribution.param(std::uniform_real_distribution<double>::param_type(-M_PI, M_PI));
  voltage_offset.param(std::normal_distribution<double>::param_type(0.0, voltage_std_dev));

  // Set SDF path for battery cell
  sdf_path = ament_index_cpp::get_package_share_directory("ariac_gz") + "/models/battery_cell/model.sdf";

  // Create ROS Node
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  // Create ROS node
  ros_node = rclcpp::Node::make_shared("cell_feed_plugin", ros_namespace);

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  ros_node->declare_parameter<double>("feed_rate", 0.0);

  param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(ros_node);

  auto param_cb = [this](const rclcpp::Parameter & p) {
    if (!configured) {
      status_msg.feed_rate = p.as_double();
      configured = true;
    } 
  };

  cb_handle = param_subscriber->add_parameter_callback("feed_rate", param_cb);

  // Spin up executor thread
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){ executor->spin_once(); }
  };

  thread_executor_spin = std::thread(spin);
  
  // Create GZ Node
  gz_node = std::make_shared<gz::transport::Node>();

  // Subscribe to the compeition status topic
  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 10, std::bind(&CellFeedPlugin::competition_status_cb, this, std::placeholders::_1)
  );

  // Create publisher and timer
  status_pub = ros_node->create_publisher<FeederStatusMsg>("status", 10);

  pub_timer = ros_node->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CellFeedPlugin::publish_status_cb, this)
  );

  control_feed_srv = ros_node->create_service<ControlSrv>(
    "control", 
    std::bind(&CellFeedPlugin::control_feed_cb, this, std::placeholders::_1, std::placeholders::_2)
  );

  status_msg.cell_type = CellTypes::NONE;
}

void CellFeedPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{

  if (_info.paused) { return; }

  switch(competition_state) {

  case CompetitionStates::PREPARING:  
  case CompetitionStates::READY:
    break;
  
  case CompetitionStates::STARTED:
  case CompetitionStates::ORDERS_COMPLETE:
    // Update feed results component
    _ecm.SetComponentData<gz::sim::components::FeedResults>(model_entity, feed_results);

    switch(feed_status) {

    case FeedStatus::WAIT_FOR_CELL:
      if (status_msg.cell_type == CellTypes::NONE) { break; }

      if (_info.simTime.count()/1E9 - last_spawn_time > 1/status_msg.feed_rate) {
        feed_status = FeedStatus::CREATE_CELL;
      } else {
        break;
      }
      
    case FeedStatus::CREATE_CELL: {
      last_spawn_time = _info.simTime.count()/1E9;

      current_cell = get_next_cell();  
      current_cell.time_created = last_spawn_time;
      
      spawn_cell(current_cell);

      feed_status = FeedStatus::ADD_CELL_COMPONENT;
      break;
    }

    case FeedStatus::ADD_CELL_COMPONENT:

      if (!_ecm.EntityByName(current_cell.cell_name).has_value()) { break; }

      current_cell.cell_entity = _ecm.EntityByName(current_cell.cell_name).value();
          _ecm.CreateComponent<gz::sim::components::Cell>(
          current_cell.cell_entity,
          gz::sim::components::Cell(current_cell)
      );

      feed_status = FeedStatus::WAIT_FOR_CELL;

      break;

    }

    break;
  
  case CompetitionStates::ENDED:
    if (status_msg.cell_type != CellTypes::NONE) {
      control_feed_srv.reset();
      status_msg.cell_type = CellTypes::NONE;
    }
    
    break;
  }
}

void CellFeedPlugin::control_feed_cb(const ControlSrvReqPtr request, ControlSrvResPtr response)
{
  if (competition_state != CompetitionStates::STARTED) {
    response->success = false;
    response->message = "Competition is not running";
    return;
  }

  if (request->cell_type == CellTypes::NONE) {
    response->success = true;
    response->message = "Stopping cell feed.";
  } else if (request->cell_type != CellTypes::LI_ION && request->cell_type != CellTypes::NIMH) {
    response->success = false;
    response->message = "Not a valid cell type";
  } else {
    response->success = true;
    response->message = "Starting cell feed with " + cell_names[request->cell_type] + " cells";
  }
  
  status_msg.cell_type = request->cell_type;
}

void CellFeedPlugin::publish_status_cb()
{
  status_pub->publish(status_msg);
}

ariac_components::Cell CellFeedPlugin::get_next_cell()
{
  ariac_components::Cell cell;

  cell.cell_type = status_msg.cell_type;

  cell.cell_name = cell_names[cell.cell_type] + "_cell_" + std::to_string(feed_results.cell_counts[cell.cell_type]);
  
  cell.defective = (defect_distribution(rng) <= defect_rate) ? true : false;
  
  cell.defect_type = cell.defective ? defect_types[defect_type_distribution(rng)] : 0;

  cell.rotation = rotation_distribution(rng);

  cell.voltage = nominal_voltages[cell.cell_type] + voltage_offset(rng);

  feed_results.cell_counts[cell.cell_type]++;

  return cell;
}

void CellFeedPlugin::spawn_cell(ariac_components::Cell cell){
  gz::msgs::EntityFactory req;

  req.set_name(cell.cell_name); // Names the cell

  auto xml = generate_cell_sdf(cell);
  
  if (!xml.has_value()){
    return;
  }

  req.set_sdf(xml.value());

  gz::math::Pose3d pose = {
    battery_spawn_location.X(),
    battery_spawn_location.Y(),
    battery_spawn_location.Z(),
    0.0,
    0.0,
    cell.rotation
  };

  gz::msgs::Set(req.mutable_pose(), pose);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  bool executed = gz_node->Request("/world/ariac/create", req, timeout, res, result);

  if (executed) {
    if (!result && res.data()) {
      RCLCPP_ERROR(ros_node->get_logger(), "Failed request to create entity.");
    }
  } else {
    RCLCPP_ERROR(ros_node->get_logger(), "Request to create entity from create service timed out.");
  }

}

std::optional<std::string> CellFeedPlugin::generate_cell_sdf(ariac_components::Cell cell){ 
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Failed to load file: " << sdf_path);
    return std::nullopt;
  }

  auto root = doc.RootElement();

  if (!root) {
    RCLCPP_ERROR(ros_node->get_logger(), "No root element in SDF.");
    return std::nullopt;
  }

  std::string visual_path = "model://battery_cell/meshes/" + cell_names[cell.cell_type];

  // Change visual to correct model for defect type

  if (!cell.defective){
    visual_path += "/base.glb";
  } else {
    visual_path += "/defect_" + std::to_string(cell.defect_type) + ".glb";
  } 

  // Set color based on type  
  auto current_element = root;
  for (std::string tag : {"model", "link", "visual", "geometry", "mesh", "uri"}){
    current_element = current_element->FirstChildElement(tag.c_str());
    
    if(!current_element){
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Unable to find xml tag: " << tag);
      return std::nullopt;
    }

    if (tag == "uri") {
      current_element->SetText(visual_path.c_str());
    }
  }
  
  // Convert from XML to string
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  
  return printer.CStr();
}

void CellFeedPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg){
  competition_state = msg->competition_state;
}