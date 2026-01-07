#include <ariac_plugins/linear_conveyor_plugin.hpp>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    ariac_plugins::LinearConveyorPlugin,
    gz::sim::System,
    ariac_plugins::LinearConveyorPlugin::ISystemPreUpdate,
    ariac_plugins::LinearConveyorPlugin::ISystemConfigure)

using namespace ariac_plugins;

LinearConveyorPlugin::~LinearConveyorPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void LinearConveyorPlugin::Configure(
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

  controllable = (_sdf->HasElement("controllable")) ? _sdf->Get<bool>("controllable") : true;

  bidirectional = (_sdf->HasElement("bidirectional")) ? _sdf->Get<bool>("bidirectional") : false;

  bool can_malfunction = (_sdf->HasElement("can_malfunction")) ? _sdf->Get<bool>("can_malfunction") : false;

  max_speed = (_sdf->HasElement("max_speed")) ? _sdf->Get<double>("max_speed") : 0.1;

  travel = (_sdf->HasElement("travel")) ? _sdf->Get<double>("travel") : 0.005;
  
  std::string joint_name = (_sdf->HasElement("joint_name")) ? _sdf->Get<std::string>("joint_name") : ""; 

  // Create belt joint
  model = gz::sim::Model(_entity);
  belt_joint = gz::sim::Joint(model.JointByName(_ecm, joint_name));
  belt_joint.EnablePositionCheck(_ecm, true);

  // Read trial component
  if (can_malfunction) {
    if (!_ecm.EntityHasComponentType(world_entity, gz::sim::components::Trial::typeId)){
      throw(std::runtime_error("Could not find trial component"));
    }

    auto trial_component = _ecm.Component<gz::sim::components::Trial>(world_entity);
    if (trial_component == nullptr) {
      throw(std::runtime_error("Could not find trial component"));
    }

    // Get challenges
    for (auto const &m: trial_component->Data().conveyor_malfunctions) {
      malfunctions.push_back(std::make_pair(m , false));
    }
  }

  // Create ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared("linear_conveyor_node", ros_namespace);

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  ros_node->declare_parameter<double>("conveyor_speed", 0.0);

  param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(ros_node);

  auto param_cb = [this](const rclcpp::Parameter & p) {
    if (!configured && p.as_double() <= max_speed) {
      speed = p.as_double();
      configured = true;
    } 
  };

  cb_handle = param_subscriber->add_parameter_callback("conveyor_speed", param_cb);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this]() {
    while (rclcpp::ok()) {
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  // Publisher
  current_status.speed = 0.0;
  current_status.direction = ConveyorStatus::FORWARD;
  current_status.operating_status = OperatingStates::OPERATIONAL;

  state_publisher = ros_node->create_publisher<ConveyorStatus>("status", 10);
  publish_timer = ros_node->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&LinearConveyorPlugin::publish_state_cb, this));

  // Subscribe to the compeition status topic
  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 
    10,
    std::bind(&LinearConveyorPlugin::competition_status_cb, this, std::placeholders::_1)
  );

  // Services
  if (!controllable){
    return;
  }

  if (!bidirectional){
    control_srv = ros_node->create_service<ConveyorControl>("control", std::bind(&LinearConveyorPlugin::control_cb, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    bi_control_srv = ros_node->create_service<BiConveyorControl>("control", std::bind(&LinearConveyorPlugin::bi_control_cb, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void LinearConveyorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                   gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused) {return;}

  if (competition_state != CompetitionStates::STARTED && competition_state != CompetitionStates::ORDERS_COMPLETE)
  {
    belt_joint.SetVelocity(_ecm, {0.0});
    return;
  }

  // Check if malfunction should be initiated
  if (!malfunctions.empty() && competiton_start_time.has_value()) {
    for (auto &malfunction: malfunctions) {

      if (malfunction.second) { continue; } // Malfunction has already been handled

      if (ros_node->get_clock()->now() - competiton_start_time.value() > rclcpp::Duration::from_seconds(malfunction.first.start_time)) {
        malfunction_end_time = ros_node->get_clock()->now() + rclcpp::Duration::from_seconds(malfunction.first.duration);
        malfunction.second = true;
        gzwarn << "Conveyor is malfunctioning\n"; 
      } 
    }
  }

  if (malfunction_end_time.has_value()) {
    // Check if malfunction duration has passed 
    if (ros_node->get_clock()->now() > malfunction_end_time.value()) {
      malfunction_end_time = std::nullopt;
      operating_status = OperatingStates::OPERATIONAL;
      gzwarn << "Conveyor is working properly again\n"; 
    } else {
      current_status.speed = 0.0;
      belt_joint.SetVelocity(_ecm, {0.0});
      operating_status = OperatingStates::MALFUNCTIONING;
      current_status.operating_status = operating_status;
      return;
    }
  }

  current_status.speed = abs(speed);
  current_status.direction = direction;
  current_status.operating_status = operating_status;

  if (direction == ConveyorStatus::FORWARD) {
    belt_joint.SetVelocity(_ecm, {speed});
  } else if (direction == ConveyorStatus::BACKWARD) {
    belt_joint.SetVelocity(_ecm, {-speed});
  }

  std::optional<std::vector<double>> position_vector = belt_joint.Position(_ecm);

  if (belt_joint.Position(_ecm).has_value() && belt_joint.Position(_ecm).value().size() > 0) {
    position = belt_joint.Position(_ecm).value()[0];
  }

  if (abs(position) >= travel) {
    belt_joint.ResetPosition(_ecm, {0.0});
  }
}

void LinearConveyorPlugin::publish_state_cb() const
{
  state_publisher->publish(current_status);
}

void LinearConveyorPlugin::control_cb(const ConveyorControlRequestPtr request, ConveyorControlResposePtr response)
{
  if ((request->speed < 0 || request->speed > max_speed)) {
    gzmsg << "Conveyor speed out of bounds. Range [0, " << max_speed << "]\n";
    response->success = false;
    return;
  }
 
  speed = request->speed;

  response->success = true;
}

void LinearConveyorPlugin::bi_control_cb(const BiConveyorControlRequestPtr request, BiConveyorControlResposePtr response)
{
  if ((request->speed < 0 || request->speed > max_speed)) {
    gzmsg << "Conveyor speed out of bounds. Range [0, " << max_speed << "]\n";
    response->success = false;
    return;
  }

  if (request->direction != ConveyorStatus::FORWARD && request->direction != ConveyorStatus::BACKWARD) {
    gzmsg << "Unknown direction\n";
    response->success = false;
    return;
  }
 
  speed = request->speed;
  direction = request->direction;

  response->success = true;
}

void LinearConveyorPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg)
{
  competition_state = msg->competition_state;
  competiton_start_time = rclcpp::Time(msg->time.start, RCL_ROS_TIME);
}