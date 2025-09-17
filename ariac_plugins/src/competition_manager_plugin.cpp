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

  if (detachable_joints_to_delete.size() > 0) {
    for (auto ent : detachable_joints_to_delete) {
      _ecm.RequestRemoveEntity(ent);
    }
    detachable_joints_to_delete.clear();
  } else if (cells_to_delete.size() > 0) {
    for (auto ent : cells_to_delete) {
      _ecm.RequestRemoveEntity(ent);
    }
    cells_to_delete.clear();
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

  case CompetitionStates::STARTED: {
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

    // Handle kit submission
    if (kitting_submission_response.status == SubmissionStatus::REQUESTED) {
      gzmsg << "Handling kitting order\n";
      handle_kit_order_submission(_ecm);
    }

    // Handle module submission
    if (module_submission_response.status == SubmissionStatus::REQUESTED) {
      handle_module_order_submission(_ecm);
    }

    // Handle high priority submission
    if (high_priority_submission_response.status == SubmissionStatus::REQUESTED) {
      handle_high_priority_order_submission(_ecm);
    }

    break;
  }

  case CompetitionStates::ORDERS_COMPLETE:
    update_competition_time();
    break;

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

  kitting_submission_response.status = SubmissionStatus::REQUESTED;

  rclcpp::Time start_time = ros_node->now();
  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    if (kitting_submission_response.status != SubmissionStatus::REQUESTED) {
      break;
    } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(5.0)) {
      res->success = false;
      res->message = "Timed out while processing submission";
      return;
    }
    rate.sleep();
  };

  res->message = kitting_submission_response.message;

  if (kitting_submission_response.status == SubmissionStatus::SUCCESSFUL) {
    ariac_db::OrderSubmissionData submission;
    submission.order_type = ariac_db::OrderType::KIT;
    submission.announcement_time = 0.0;
    submission.submission_time = (start_time - competition_time.start).nanoseconds() / 1E9;
    order_submissions.push_back(submission);

    num_submitted_orders[ariac_db::OrderType::KIT]++;

    res->success = true;

    if (orders_complete()) {
      competition_state = CompetitionStates::ORDERS_COMPLETE;
    }
  } else {
    res->success = false;
  }

  kitting_submission_response.status = SubmissionStatus::NOT_REQUESTED;
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

  high_priority_submission_response.status = SubmissionStatus::REQUESTED;

  rclcpp::Time start_time = ros_node->now();
  while (rclcpp::ok()) {
    if (high_priority_submission_response.status != SubmissionStatus::REQUESTED) {
      break;
    } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(5.0)) {
      res->success = false;
      res->message = "Timed out while processing submission";
      return;
    }
  };

  res->message = high_priority_submission_response.message;

  if (high_priority_submission_response.status == SubmissionStatus::SUCCESSFUL) {
    res->success = true;
    order.value()->submitted = true;

    ariac_db::OrderSubmissionData submission;
    submission.order_type = ariac_db::OrderType::HIGH_PRIORITY;
    submission.announcement_time = order.value()->announcement_time;
    submission.submission_time = (start_time - competition_time.start).nanoseconds() / 1E9;
    order_submissions.push_back(submission);

    num_submitted_orders[ariac_db::OrderType::HIGH_PRIORITY]++;

    if (orders_complete()) {
      competition_state = CompetitionStates::ORDERS_COMPLETE;
    }

  } else {
    res->success = false;
  }

  high_priority_submission_response.status = SubmissionStatus::NOT_REQUESTED;
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
  status.time = competition_time;
  
  if (competition_state == CompetitionStates::STARTED || competition_state == CompetitionStates::ENDED){
    status.num_kits = trial.num_kits;
    status.num_modules = trial.num_modules;

    status.num_kits_remaining = status.num_kits - num_submitted_orders[ariac_db::OrderType::KIT];
    status.num_modules_remaining = status.num_modules - num_submitted_orders[ariac_db::OrderType::MODULE];

    status.run_id = run_id;
  }

  competition_status_pub->publish(status);
}

std::vector<ariac_components::Cell> CompetitionManagerPlugin::get_cells_in_bbox(
  gz::sim::EntityComponentManager &_ecm, 
  gz::math::AxisAlignedBox bbox) 
{
  std::vector<ariac_components::Cell> cells;

  _ecm.Each<gz::sim::components::Cell>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Cell *cell) -> bool {
        auto pose =
            gz::sim::Link(gz::sim::Model(entity).LinkByName(_ecm, "base_link"))
                .WorldPose(_ecm);
        if (pose.has_value() && bbox.Contains(pose.value().Pos())) {
          cells.push_back(cell->Data());
        }
        return true;
      });

  return cells;
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

SubmissionResponse CompetitionManagerPlugin::check_kit(
  int cell_type, 
  std::vector<ariac_components::Cell> submission_cells)
{
  SubmissionResponse response;

  if (submission_cells.empty()) {
    response.message = "No cells at shipping station";
    response.status = SubmissionStatus::FAIL;
    return response;
  }

  if (submission_cells.size() != 4) {
    response.message =
        "Kit has " + std::to_string(submission_cells.size()) + " cells";
    response.status = SubmissionStatus::FAIL;
    return response;
  }

  double total_voltage = 0.0;
  for (const auto &cell : submission_cells) {
    if (cell.defective) {
      response.message = "A defective cell is in the kit";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    if (cell.cell_type != cell_type) {
      response.message = "A cell with the wrong type is in the kit";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    if (abs(cell.voltage - nominal_voltages[cell_type]) > CellTypes::CELL_VOLTAGE_TOLERANCE) {
      response.message = "A cell has a voltage outside of allowed tolerance";
      response.status = SubmissionStatus::FAIL;
      return response;
    }

    total_voltage += cell.voltage;
  }

  if (abs(total_voltage - (nominal_voltages[cell_type] * 4)) > CellTypes::KIT_VOLTAGE_TOLERANCE) {
    response.message = "Total voltage of " + std::to_string(total_voltage) +
                       " is not within allowed tolerance";
    response.status = SubmissionStatus::FAIL;
    return response;
  }

  response.message = "Kit submitted succesfully";
  response.status = SubmissionStatus::SUCCESSFUL;
  return response;
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

void CompetitionManagerPlugin::handle_kit_order_submission(
  gz::sim::EntityComponentManager &_ecm) 
{
  auto submission_cells = get_cells_in_bbox(_ecm, shipping_bbox);

  std::vector<gz::sim::Entity> kit_detachable_joints;
  _ecm.Each<gz::sim::components::DetachableJoint>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::DetachableJoint *detachable_joint)
          -> bool {
        auto data = detachable_joint->Data();
        for (auto cell : submission_cells) {
          auto base_link =
              gz::sim::Model(cell.cell_entity).LinkByName(_ecm, "base_link");
          if (data.childLink == base_link || data.parentLink == base_link) {
            kit_detachable_joints.push_back(entity);
          }
        }
        return kit_detachable_joints.size() != 4;
      });

  kitting_submission_response = check_kit(CellTypes::LI_ION, submission_cells);

  if (kitting_submission_response.status == SubmissionStatus::SUCCESSFUL) {
    detachable_joints_to_delete = kit_detachable_joints;
    for (const auto &cell : submission_cells) {
      cells_to_delete.push_back(cell.cell_entity);
    }
  }
}

void CompetitionManagerPlugin::handle_high_priority_order_submission(
  gz::sim::EntityComponentManager &_ecm)
{
  auto submission_cells = get_cells_in_bbox(_ecm, shipping_bbox);

  high_priority_submission_response =
      check_kit(CellTypes::NIMH, submission_cells);

  if (high_priority_submission_response.status ==
      SubmissionStatus::SUCCESSFUL) {
    for (const auto &cell : submission_cells) {
      _ecm.RequestRemoveEntity(cell.cell_entity);
    }
  }
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

  for (const auto &[slot, entity] : module.cell_entities) {
    _ecm.RequestRemoveEntity(entity);
  }

  _ecm.RequestRemoveEntity(module.bottom_shell_entity);
  _ecm.RequestRemoveEntity(module.top_shell_entity);
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
