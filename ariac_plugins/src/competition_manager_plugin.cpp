#include "ariac_plugins/competition_manager_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::CompetitionManagerPlugin,
  gz::sim::System,
  ariac_plugins::CompetitionManagerPlugin::ISystemConfigure,
  ariac_plugins::CompetitionManagerPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

CompetitionManagerPlugin::~CompetitionManagerPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void CompetitionManagerPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm, 
  gz::sim::EventManager &)
{
  // Handle building trial component
  std::string trial_config_file = _sdf->Get<std::string>("trial_config_file");
  read_trial(trial_config_file);

  competitor_name = _sdf->Get<std::string>("competitor_name");
  sensor_cost = _sdf->Get<int>("sensor_cost");
  
  if (_sdf->HasElement("db_path")) {
    db_manager = std::make_unique<ariac_db::DatabaseManager>(_sdf->Get<std::string>("db_path"));
    connected_to_db = connect_to_database(trial_config_file);

    if (!connected_to_db) {
      shutdown_gazebo();
    }
  }

  competition_time.remaining.sec = trial.time_limit;

  _ecm.CreateComponent(_entity, gz::sim::components::Trial(trial));

  gz_node = std::make_shared<gz::transport::Node>();

  // ROS Setup
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  // Create ROS node
  ros_node = rclcpp::Node::make_shared("competition_manager_plugin");

  // Setup parameters
  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  ros_node->declare_parameter<bool>("controllers_ready", false);
  ros_node->declare_parameter<bool>("sensors_ready", false);

  // Create services
  start_competition_srv = ros_node->create_service<Trigger>(
      "start_competition",
      std::bind(&CompetitionManagerPlugin::start_competition_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  end_competition_srv = ros_node->create_service<EndCompetition>(
      "end_competition",
      std::bind(&CompetitionManagerPlugin::end_competition_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  submit_kitting_srv = ros_node->create_service<Trigger>(
      "submit_kitting_order",
      std::bind(&CompetitionManagerPlugin::submit_kitting_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  submit_high_priority_srv = ros_node->create_service<SubmitHighPriorityOrder>(
      "submit_high_priority_order",
      std::bind(&CompetitionManagerPlugin::submit_high_priority_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  submit_module_srv = ros_node->create_service<Trigger>(
      "submit_module_order",
      std::bind(&CompetitionManagerPlugin::submit_module_cb, this,
                std::placeholders::_1, std::placeholders::_2));

  // Setup publishers
  competition_status_pub = ros_node->create_publisher<CompetitionStatus>("competition_status", 10);
  
  high_priority_pub = ros_node->create_publisher<HighPriorityOrderMsg>("high_priority_orders", 10);

  agv1_info_sub = ros_node->create_subscription<AGVStatus>("/agv1/info", 10, std::bind(&CompetitionManagerPlugin::agv1_station_cb, this, std::placeholders::_1));
  agv2_info_sub = ros_node->create_subscription<AGVStatus>("/agv2/info", 10, std::bind(&CompetitionManagerPlugin::agv2_station_cb, this, std::placeholders::_1));
  agv3_info_sub = ros_node->create_subscription<AGVStatus>("/agv3/info", 10, std::bind(&CompetitionManagerPlugin::agv3_station_cb, this, std::placeholders::_1));

  status_pub_timer = ros_node->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CompetitionManagerPlugin::publish_status, this)
  );

  // Spin up executor thread
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this]() {
    while (rclcpp::ok()) {
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);
}

void CompetitionManagerPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info, 
  gz::sim::EntityComponentManager &_ecm) 
{
  if (_info.paused) {
    return;
  }

  switch (competition_state) {
  case CompetitionStates::PREPARING:
    bool controllers_ready, sensors_ready;

    ros_node->get_parameter("controllers_ready", controllers_ready);
    ros_node->get_parameter("sensors_ready", sensors_ready);

    if (controllers_ready && sensors_ready) {
      competition_state = CompetitionStates::READY;
    }

    break;

  case CompetitionStates::READY:
    break;

  case CompetitionStates::STARTED:
  case CompetitionStates::ORDERS_COMPLETE:
  {
    // Update time
    update_competition_time();

    // Handle publish high priority order
    for (const auto order : high_priority_orders) {
      if (order.published) {
        continue;
      }

      if (ros_node->get_clock()->now() >= competition_time.start + rclcpp::Duration::from_seconds(order.announcement_time)) {
        HighPriorityOrderMsg msg;
        msg.id = order.id;

        high_priority_pub->publish(msg);
      }
    }

    // Handle module submission
    if (module_submission_response.status == SubmissionStatus::REQUESTED) {
      handle_module_order_submission(_ecm);
    } else if (bottom_shell_to_lock.has_value()){
      gzwarn << "Locking module to shelf\n";
      lock_to_shelf(_ecm, bottom_shell_to_lock.value());
      bottom_shell_to_lock = std::nullopt;
    }
    
    break;
  
  }

  case CompetitionStates::ENDED:
    if (!end_handled) {
      competition_ended_time = _info.realTime.count();
      handle_competition_end(_ecm);
      end_handled = true;
    } else if (shutdown && _info.realTime.count() - competition_ended_time > 5E9) {
      shutdown_gazebo();
    }
    break;
  }
}

void CompetitionManagerPlugin::start_competition_cb(
  const TriggerReqPtr req,
  TriggerResPtr res) 
{
  if (competition_state == CompetitionStates::PREPARING) {
    res->message = "Environment not ready to start yet";
    res->success = false;
    return;
  }
  if (competition_state == CompetitionStates::STARTED || competition_state == CompetitionStates::ORDERS_COMPLETE) {
    res->message = "Competition has already started";
    res->success = false;
    return;
  }
  if (competition_state == CompetitionStates::ENDED) {
    res->message = "Competition has already ended";
    res->success = false;
    return;
  }

  competition_time.start = ros_node->get_clock()->now();
  end_time = ros_node->get_clock()->now() + rclcpp::Duration::from_seconds(trial.time_limit);
  competition_state = CompetitionStates::STARTED;

  res->message = "Starting competition";
  res->success = true;
}

void CompetitionManagerPlugin::end_competition_cb(
  const EndCompetitionReqPtr req,
  EndCompetitionResPtr res)
{
  if (competition_state == CompetitionStates::PREPARING || competition_state == CompetitionStates::READY) {
    res->message = "Competition has not started yet";
    res->success = false;
    return;
  }

  if (competition_state == CompetitionStates::ENDED) {
    res->message = "Competition has already ended";
    res->success = false;
    return;
  }

  shutdown = req->shutdown_gazebo;

  res->message = "Ending competition";
  res->success = true;
  competition_state = CompetitionStates::ENDED;
}

void CompetitionManagerPlugin::submit_kitting_cb(
  const TriggerReqPtr req,
  TriggerResPtr res) 
{

  if (num_submitted_orders[ariac_db::OrderType::KIT] == trial.num_kits) {
    res->success = false;
    res->message = "All desired kits already submitted";
    return;
  }

  int agv_at_shipping = get_agv_at_shipping();

  rclcpp::Time start_time = ros_node->now();

  if (agv_at_shipping == -1) {
    res->message = "No AGV at shipping station";
    res->success = false;
    return;
  }

  gz::msgs::Int32 gz_req;
  gz_req.set_data(CellTypes::LI_ION);
  gz::msgs::Boolean gz_res;
  bool result;
  unsigned int timeout = 6000;

  start_time = ros_node->now();
  bool executed = gz_node->Request("/agv"+std::to_string(agv_at_shipping)+"/handle_kit", gz_req, timeout, gz_res, result);

  if(!executed){
    res->success = false;
    res->message = "GZ SRV for handling kit not executed";
    return;
  }

  if(!gz_res.data()){
    res->success = false;
    res->message = "Kit not successfully submitted";
    return;
  }

  ariac_db::OrderSubmissionData submission;
  submission.order_type = ariac_db::OrderType::KIT;
  submission.announcement_time = 0.0;
  submission.submission_time = (start_time - competition_time.start).nanoseconds() / 1E9;
  order_submissions.push_back(submission);

  num_submitted_orders[ariac_db::OrderType::KIT]++;

  res->success = true;
  res->message = "Kit successfully submitted";

  if (orders_complete()) {
    competition_state = CompetitionStates::ORDERS_COMPLETE;
  }

}

void CompetitionManagerPlugin::submit_high_priority_cb(
  const SubmitHighPriorityOrderReqPtr req,
  SubmitHighPriorityOrderResPtr res) 
{
  std::optional<HighPriorityOrder *> order = std::nullopt;
  for (auto &o : high_priority_orders) {
    if (o.id == req->id && !o.submitted) {
      order = &o;
      break;
    }
  }

  if (!order.has_value()) {
    res->success = false;
    res->message = "Order ID is incorrect or already submitted";
    return;
  }

  int agv_at_shipping = get_agv_at_shipping();

  rclcpp::Time start_time = ros_node->now();

  if (agv_at_shipping == -1) {
    res->message = "No AGV at shipping station";
    res->success = false;
    return;
  }

  gz::msgs::Int32 gz_req;
  gz_req.set_data(CellTypes::NIMH);
  gz::msgs::Boolean gz_res;
  bool result;
  unsigned int timeout = 6000;

  start_time = ros_node->now();
  bool executed = gz_node->Request("/agv"+std::to_string(agv_at_shipping)+"/handle_kit", gz_req, timeout, gz_res, result);

  if(!executed){
    res->success = false;
    res->message = "GZ SRV for handling high priority kit not executed";
    return;
  }

  if(!gz_res.data()){
    res->success = false;
    res->message = "High priority kit not successfully submitted";
    return;
  }

  ariac_db::OrderSubmissionData submission;
  submission.order_type = ariac_db::OrderType::HIGH_PRIORITY;
  submission.announcement_time = 0.0;
  submission.submission_time = (start_time - competition_time.start).nanoseconds() / 1E9;
  order_submissions.push_back(submission);

  num_submitted_orders[ariac_db::OrderType::HIGH_PRIORITY]++;

  res->success = true;
  res->message = "High priority kit successfully submitted";

  if (orders_complete()) {
    competition_state = CompetitionStates::ORDERS_COMPLETE;
  }
}

void CompetitionManagerPlugin::submit_module_cb(
  const TriggerReqPtr req,
  TriggerResPtr res) 
{
  module_submission_response.status = SubmissionStatus::REQUESTED;

  rclcpp::Time start_time = ros_node->now();
  while (rclcpp::ok()) {
    if (module_submission_response.status != SubmissionStatus::REQUESTED) {
      break;
    } else if (ros_node->now() - start_time >
               rclcpp::Duration::from_seconds(5.0)) {
      res->success = false;
      res->message = "Timed out while processing submission";
      return;
    }
  };

  res->message = module_submission_response.message;

  if (module_submission_response.status == SubmissionStatus::SUCCESSFUL) {

    if (num_submitted_orders[ariac_db::OrderType::MODULE] == trial.num_modules) {
      res->success = false;
      res->message = "All desired modules already submitted";
      module_submission_response.status = SubmissionStatus::NOT_REQUESTED;
      return;
    }

    ariac_db::OrderSubmissionData submission;
    submission.order_type = ariac_db::OrderType::MODULE;
    submission.announcement_time = 0.0;
    submission.submission_time = (start_time - competition_time.start).nanoseconds() / 1E9;
    order_submissions.push_back(submission);

    num_submitted_orders[ariac_db::OrderType::MODULE]++;


    res->success = true;

    if (orders_complete()) {
      competition_state = CompetitionStates::ORDERS_COMPLETE;
    }
  } else {
    res->success = false;
  }

  module_submission_response.status = SubmissionStatus::NOT_REQUESTED;
}

void CompetitionManagerPlugin::publish_status() {
  // Check if all orders are announced and complete
  CompetitionStatus status;
  status.competition_state = competition_state;
  status.run_id = run_id;
  status.time = competition_time;
  
  status.num_kits = trial.num_kits;
  status.num_modules = trial.num_modules;

  status.num_kits_remaining = status.num_kits - num_submitted_orders[ariac_db::OrderType::KIT];
  status.num_modules_remaining = status.num_modules - num_submitted_orders[ariac_db::OrderType::MODULE];

  competition_status_pub->publish(status);
}

std::vector<ariac_components::Module>
CompetitionManagerPlugin::get_modules_in_bbox(
  gz::sim::EntityComponentManager &_ecm,
  gz::math::AxisAlignedBox bbox)
{
  std::vector<ariac_components::Module> modules;

  _ecm.Each<gz::sim::components::Module>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Module *module) -> bool {
        auto pose =
            gz::sim::Link(gz::sim::Model(entity).LinkByName(_ecm, "base_link"))
                .WorldPose(_ecm);
        if (pose.has_value() && bbox.Contains(pose.value().Pos())) {
          modules.push_back(module->Data());
        }
        return true;
      });

  return modules;
}

SubmissionResponse CompetitionManagerPlugin::check_module(
  gz::sim::EntityComponentManager &_ecm,
  ariac_components::Module module)
{
  SubmissionResponse response;

  // Check that top shell is present
  if (module.top_shell_entity == gz::sim::kNullEntity) {
    response.message = "Top shell is missing";
    response.status = SubmissionStatus::FAIL;
    return response;
  }

  // Check cells
  double total_voltage = 0.0;
  for (const auto &[slot, entity] : module.cell_entities) {
    if (entity == gz::sim::kNullEntity) {
      response.message = "Missing cell in slot " + std::to_string(slot);
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    auto component = _ecm.Component<gz::sim::components::Cell>(entity);

    if (component == nullptr) {
      response.message = "Cell in slot " + std::to_string(slot) +
                         " does not have cell component";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    auto cell = component->Data();

    if (cell.cell_type != CellTypes::LI_ION) {
      response.message =
          "Cell in slot " + std::to_string(slot) + " is the wrong type";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    if (cell.defective) {
      response.message =
          "Cell in slot " + std::to_string(slot) + " is defective";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    if (abs(cell.voltage - nominal_voltages[CellTypes::LI_ION]) > CellTypes::CELL_VOLTAGE_TOLERANCE) {
      response.message = "A cell has a voltage outside of allowed tolerance";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    total_voltage += cell.voltage;
  }

  // Check that all cells are oriented correctly
  for (const auto &[slot, orientation] : module.cell_orientation) {
    ariac_components::CellOrientation desired_orientation;
    if (slot % 2 == 1) {
      desired_orientation = ariac_components::CellOrientation::UP;
    } else {
      desired_orientation = ariac_components::CellOrientation::DOWN;
    }

    if (orientation != desired_orientation) {
      response.message =
          "Cell in slot " + std::to_string(slot) + " is oriented incorrectly";
      response.status = SubmissionStatus::FAIL;
      return response;
    }
  }

  // Check that module voltage is correct
  if (abs(total_voltage - (nominal_voltages[CellTypes::LI_ION] * 4)) > CellTypes::KIT_VOLTAGE_TOLERANCE) {
    response.message = "Total voltage of " + std::to_string(total_voltage) +
                       " is not within allowed tolerance";
    response.status = SubmissionStatus::FAIL;
    return response;
  }

  // Check that all bottom welds are present
  for (const auto &[slot, weld] : module.bottom_welds) {
    if (!weld) {
      response.message =
          "Missing weld on bottom shell slot " + std::to_string(slot);
      response.status = SubmissionStatus::FAIL;
      return response;
    }
  }

  // Check that all top welds are present
  for (const auto &[slot, weld] : module.top_welds) {
    if (!weld) {
      response.message =
          "Missing weld on top shell slot " + std::to_string(slot);
      response.status = SubmissionStatus::FAIL;
      return response;
    }
  }

  response.message = "Module submitted succesfully";
  response.status = SubmissionStatus::SUCCESSFUL;
  return response;
}

void CompetitionManagerPlugin::handle_module_order_submission(
  gz::sim::EntityComponentManager &_ecm)
{

  auto modules = get_modules_in_bbox(_ecm, module_submission_bbox);

  if (modules.empty()) {
    module_submission_response.message = "No module found in submission area";
    module_submission_response.status = SubmissionStatus::FAIL;
    return;
  }

  if (modules.size() != 1) {
    module_submission_response.message = "Multiple modules in submission area";
    module_submission_response.status = SubmissionStatus::FAIL;
    return;
  }

  auto module = modules.front();

  module_submission_response = check_module(_ecm, module);

  // Teleport module to shelf
  std::string shelf_name = "assembly_module_shelf";
  std::vector<gz::math::Pose3d> slots = ariac_components::ShelfSlot::MODULE_SHELF_SLOTS;

  auto shelf_entity_opt = _ecm.EntityByName(shelf_name);
  
  if(!shelf_entity_opt.has_value()){
    throw std::runtime_error("Could not find module shelf entity");
  }

  ariac_components::ShelfSlot shelf_slot;

  if(!_ecm.EntityHasComponentType(shelf_entity_opt.value(), gz::sim::components::ShelfSlot::typeId)){
    _ecm.CreateComponent<gz::sim::components::ShelfSlot>(shelf_entity_opt.value(), gz::sim::components::ShelfSlot(shelf_slot));
  } else {
    auto component = _ecm.Component<gz::sim::components::ShelfSlot>(shelf_entity_opt.value());
    if (component == nullptr) {
      throw std::runtime_error("Could not find shelf slot component");
    }
    shelf_slot = component->Data();
  }
  
  auto shelf_base_link_entity = gz::sim::Model(shelf_entity_opt.value()).LinkByName(_ecm, "base_link");
  if (shelf_base_link_entity == gz::sim::kNullEntity) {
    throw std::runtime_error("Unable to find shelf base link");
  }

  gz::sim::Link shelf_base_link = gz::sim::Link(shelf_base_link_entity);
  auto shelf_world_pose_opt = shelf_base_link.WorldPose(_ecm);
  if (!shelf_world_pose_opt.has_value()){
    throw std::runtime_error("Could not find world pose for shelf_base_link");
  }
  
  auto bottom_shell_model = gz::sim::Model(module.bottom_shell_entity);

  bottom_shell_model.SetWorldPoseCmd(_ecm, shelf_world_pose_opt.value() * slots[shelf_slot.index]);
  shelf_slot.index++;
  
  _ecm.SetComponentData<gz::sim::components::ShelfSlot>(shelf_entity_opt.value(), shelf_slot);

  bottom_shell_to_lock = module.bottom_shell_entity;
}

void CompetitionManagerPlugin::handle_competition_end(
  gz::sim::EntityComponentManager &_ecm) 
{
  rclcpp::Duration execution_duration = ros_node->get_clock()->now() - rclcpp::Time(competition_time.start);

  auto conveyor_entity = _ecm.EntityByName("inspection_conveyor");
  
  if (!conveyor_entity.has_value()) {
    gzerr << "Unable to get conveyor entity\n";
    return;
  }

  auto feed_results = _ecm.Component<gz::sim::components::FeedResults>(conveyor_entity.value())->Data();

  auto inspection_results = _ecm.Component<gz::sim::components::InspectionResults>(conveyor_entity.value())->Data();

  std::vector<ariac_db::PenaltyData> penalties;
  std::vector<double> agv_collision_times;

  double competition_start_ns = rclcpp::Time(competition_time.start).nanoseconds();

  _ecm.Each<gz::sim::components::Penalty>(
    [&](const gz::sim::Entity &,
        const gz::sim::components::Penalty *penalty) -> bool {
      auto p = penalty->Data();
      if (p.type == ariac_components::PenaltyType::AGV_COLLISION) {
        for (const double t : agv_collision_times) {
          if (abs(p.time - t) < 1e9) {
            return true; // ignore penalty
          }
        }
        agv_collision_times.push_back(p.time);
      }

      ariac_db::PenaltyData penalty_data;
      penalty_data.description = p.description;
      penalty_data.time = (p.time - competition_start_ns) / 1E9;
      penalty_data.type = static_cast<int>(p.type);
      penalties.push_back(penalty_data);
      return true;
    }
  );

  if (connected_to_db) {
    ariac_db::RunData run;
    run.completed = true;
    run.aborted = !orders_complete();
    run.sensor_cost = sensor_cost;
    run.duration = execution_duration.nanoseconds() / 1E9;
    run.total_cells = feed_results.cell_counts[CellTypes::LI_ION] + feed_results.cell_counts[CellTypes::NIMH];
    run.defective_cells = feed_results.num_defective;
    run.num_reports_submitted = inspection_results.num_reports_submitted;
    run.avg_report_time = inspection_results.avg_report_time;
    run.num_correct_reports = inspection_results.num_correct_reports;
    run.num_correct_report_classifications = inspection_results.num_correct_report_classifications;

    db_manager->updateRun(run_id, run);

    for (const auto &penalty : penalties) {
      db_manager->insertPenalty(run_id, penalty);
    }

    for (const auto &order : order_submissions) {
      db_manager->insertOrderSubmission(run_id, order);
    }

    db_manager->disconnect();
  }
}

bool CompetitionManagerPlugin::connect_to_database(std::string trial_config)
{
  if (!db_manager->connect()){ return false; }

  competitor_id = db_manager->getCompetitorId(competitor_name);
  
  if (competitor_id == -1) {
    competitor_id = db_manager->insertCompetitor(competitor_name);
  }
  
  trial_id = db_manager->getTrialId(trial.id);
  
  if (trial_id == -1) {
    // Trial doesn't exist, create new one with config hash
    std::string current_config_hash = db_manager->hashFileSHA256(trial_config);

    if (current_config_hash.empty()) {
      gzerr << "Failed to calculate hash for trial config file: " << trial_config << '\n';
      return false;
    }

    ariac_db::TrialData db_trial;
    db_trial.trial_id = trial.id;
    db_trial.config_hash = current_config_hash;
    db_trial.num_kits = trial.num_kits;
    db_trial.num_modules = trial.num_modules;
    db_trial.num_high_priority = high_priority_orders.size();
    db_trial.seed = trial.seed;
    db_trial.time_limit = trial.time_limit;

    trial_id = db_manager->insertTrial(db_trial);

    if (trial_id == -1) {
      gzerr << "Failed to insert trial into database" << '\n';;
      return false;
    }
  } else {
    // Trial exists in database, check if configuration hash matches
    ariac_db::TrialData existing_trial = db_manager->getTrial(trial.id);

    if (existing_trial.id == -1) {
      gzerr << "Failed to retrieve existing trial data from database" << '\n';
      return false;
    }

    // Calculate hash of current trial config file
    std::string current_config_hash = db_manager->hashFileSHA256(trial_config);

    if (current_config_hash.empty()) {
      gzerr << "Failed to calculate hash for trial config file: " << trial_config << '\n';
      return false;
    }

    // Handle backward compatibility: if existing hash is empty, update it
    if (existing_trial.config_hash.empty()) {
      gzwarn << "Existing trial has no config hash. Updating with current "
                "configuration hash for backward compatibility."
              << '\n';
      if (!db_manager->updateTrialConfigHash(trial.id, current_config_hash)) {
        gzerr << "Failed to update trial config hash in database"
              << '\n';
        return false;
      }
    }
    // Compare hashes only if existing hash is not empty
    else if (existing_trial.config_hash != current_config_hash) {
      gzerr << "Existing trial in the database has different configuration. "
            << "Expected hash: " << existing_trial.config_hash
            << ", Current hash: " << current_config_hash
            << ". Please change the trial ID." << '\n';
      return false;
    }
  }

  // Insert run
  ariac_db::RunData run;
  run.completed = false;
  
  run_id = db_manager->insertRun(trial_id, competitor_id, run);

  return true;
}

void CompetitionManagerPlugin::shutdown_gazebo() {
  gz::msgs::ServerControl req;

  req.set_stop(true);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 2000;

  bool executed =
      gz_node->Request("/server_control", req, timeout, res, result);
}

void CompetitionManagerPlugin::read_trial(std::string filepath) {
  YAML::Node trial_yaml;

  trial_yaml = YAML::LoadFile(filepath);

  trial.id = trial_yaml["ID"].as<std::string>();
  trial.seed = trial_yaml["SEED"].as<int>();
  trial.defect_rate = trial_yaml["DEFECT_RATE"].as<double>();
  trial.time_limit = trial_yaml["TIME_LIMIT"].as<int>();
  trial.num_kits = trial_yaml["NUM_KITS"].as<int>();
  trial.num_modules = trial_yaml["NUM_MODULES"].as<int>();

  if (trial_yaml["POSSIBLE_DEFECTS"]) {
    for (const auto &defect : trial_yaml["POSSIBLE_DEFECTS"]) {
      trial.possible_defects.push_back(defect.as<int>());
    }
  }

  if (!trial_yaml["CHALLENGES"]) {
    return;
  }

  YAML::Node challenges = trial_yaml["CHALLENGES"];

  if (challenges["CONVEYOR_MALFUNCTIONS"]) {
    for (const auto &challenge : challenges["CONVEYOR_MALFUNCTIONS"]) {
      ariac_components::ConveyorMalfunction conveyor_malfunction;
      conveyor_malfunction.start_time = challenge["START_TIME"].as<int>();
      conveyor_malfunction.duration = challenge["DURATION"].as<int>();

      trial.conveyor_malfunctions.push_back(conveyor_malfunction);
    }
  }

  if (challenges["VACUUM_TOOL_MALFUNCTIONS"]) {
    for (const auto &challenge : challenges["VACUUM_TOOL_MALFUNCTIONS"]) {
      ariac_components::VacuumToolMalfunction vacuum_tool_malfunction;
      vacuum_tool_malfunction.tool = challenge["TOOL"].as<int>();
      vacuum_tool_malfunction.grasp_occurrence =
          challenge["GRASP_OCCURRENCE"].as<int>();

      trial.vacuum_tool_malfunctions.push_back(vacuum_tool_malfunction);
    }
  }

  if (challenges["VOLTAGE_TESTER_MALFUNCTIONS"]) {
    for (const auto &challenge : challenges["VOLTAGE_TESTER_MALFUNCTIONS"]) {
      ariac_components::VoltageTesterMalfunction voltage_tester_malfunction;
      voltage_tester_malfunction.start_time = challenge["START_TIME"].as<int>();
      voltage_tester_malfunction.duration = challenge["DURATION"].as<int>();
      voltage_tester_malfunction.tester = challenge["TESTER"].as<int>();

      trial.voltage_tester_malfunctions.push_back(voltage_tester_malfunction);
    }
  }

  if (challenges["HIGH_PRIORITY_ORDERS"]) {
    for (const auto &high_priority_order : challenges["HIGH_PRIORITY_ORDERS"]) {
      HighPriorityOrder order;
      order.id = high_priority_order["ID"].as<std::string>();
      order.announcement_time = high_priority_order["START_TIME"].as<double>();
      order.published = false;

      high_priority_orders.push_back(order);
    }
  }
}

void CompetitionManagerPlugin::agv1_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
  agv_locations[1] = msg->station_id;
}
void CompetitionManagerPlugin::agv2_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
  agv_locations[2] = msg->station_id;
}
void CompetitionManagerPlugin::agv3_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
  agv_locations[3] = msg->station_id;
}

int CompetitionManagerPlugin::get_agv_at_shipping(){
  for(const auto& [agv, location] : agv_locations){
    if(location == AGVStations::SHIPPING){
      return agv;
    }
  }
  return -1;
}

std::string CompetitionManagerPlugin::create_temp_file() {
  // Template must end in "XXXXXX"
  char temp_path[] = "/tmp/ariac_log_XXXXXX.txt";

  // mkstemp replaces XXXXXX with a unique suffix and opens the file
  int fd = mkstemps(temp_path, 4);
  if (fd == -1) {
    return "";
  }

  // Optional: close the file descriptor if you prefer using std::ofstream
  close(fd);

  return std::string(temp_path); // return the filename
}

void CompetitionManagerPlugin::update_competition_time()
{
  auto current_time = ros_node->get_clock()->now();
  if (current_time >= end_time) {
    competition_time.elapsed = rclcpp::Duration::from_seconds(trial.time_limit);
    competition_time.remaining = rclcpp::Duration::from_seconds(0.0);
    competition_state = CompetitionStates::ENDED;
  }

  competition_time.elapsed = current_time - competition_time.start;
  competition_time.remaining = end_time - current_time;
}

bool CompetitionManagerPlugin::orders_complete() {
  return num_submitted_orders[ariac_db::OrderType::KIT] == trial.num_kits &&
    num_submitted_orders[ariac_db::OrderType::MODULE] == trial.num_modules &&
    num_submitted_orders[ariac_db::OrderType::HIGH_PRIORITY] == high_priority_orders.size();
}

void CompetitionManagerPlugin::lock_to_shelf(
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity bottom_shell_entity)
{
  gz::sim::Entity floor_link = _ecm.EntityByComponents(gz::sim::components::Name("floor"), gz::sim::components::Link());

  if (floor_link == gz::sim::kNullEntity) {
    throw std::runtime_error("Unable to locate floor link");
  }

  auto bottom_shell_link = gz::sim::Model(bottom_shell_entity).LinkByName(_ecm, "base_link");

  if (bottom_shell_link == gz::sim::kNullEntity) {
    throw std::runtime_error("Unable to get base link for bottom_shell");
  }

  _ecm.CreateComponent(_ecm.CreateEntity(), gz::sim::components::DetachableJoint({floor_link, bottom_shell_link, "fixed"}));
}