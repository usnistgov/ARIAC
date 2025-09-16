#include "ariac_plugins/voltage_tester_plugin.hpp"


GZ_ADD_PLUGIN(
  ariac_plugins::VoltageTesterPlugin,
  gz::sim::System,
  ariac_plugins::VoltageTesterPlugin::ISystemConfigure,
  ariac_plugins::VoltageTesterPlugin::ISystemUpdate
)

using namespace ariac_plugins;

VoltageTesterPlugin::~VoltageTesterPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void VoltageTesterPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  auto model = gz::sim::Model(_entity);

  voltage_tester_name = model.Name(_ecm);
  tester_number = voltage_tester_name.back() - '0';

  gz_contact_topic = "/world/ariac/model/" + voltage_tester_name + "/link/voltage_tester/sensor/voltage_tester_collision_detector/contact";

  update_rate = _sdf->Get<double>("update_rate");

  double noise_std_dev = _sdf->Get<double>("noise_std_dev");

  // Read trial component
  if (!_ecm.EntityHasComponentType(world_entity, gz::sim::components::Trial::typeId)){
    throw(std::runtime_error("Could not find trial component"));
  }

  auto trial_component = _ecm.Component<gz::sim::components::Trial>(world_entity);
  if (trial_component == nullptr) {
    throw(std::runtime_error("Could not find trial component"));
  }

  // Get challenges
  for (auto const &m: trial_component->Data().voltage_tester_malfunctions) {
    if (m.tester == tester_number) {
      malfunctions.push_back(std::make_pair(m , false));
    }
  }

  // Noise Distribution
  rng.seed(trial_component->Data().seed);
  noise_distribution = std::normal_distribution<double>(0.0 ,noise_std_dev);

  // GZ setup
  gz_node = std::make_shared<gz::transport::Node>();

  gz_node->Subscribe(gz_contact_topic, &VoltageTesterPlugin::contact_msg_cb, this);

  // ROS setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }
  
  ros_node = rclcpp::Node::make_shared(voltage_tester_name+"_plugin");

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok(node_context)){
      executor->spin_once();
    }
    };
  
  thread_executor_spin = std::thread(spin);

  // Publisher
  voltage_tester_publisher = ros_node->create_publisher<VoltageReading>(
    voltage_tester_name+"/voltage", 
    10
  );
  publish_timer = ros_node->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&VoltageTesterPlugin::publish_voltage_cb, this));

  // Subscribe to the compeition status topic
  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 
    10,
    std::bind(&VoltageTesterPlugin::competition_status_cb, this, std::placeholders::_1)
  );

}

void VoltageTesterPlugin::Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{ 
  if (_info.paused) { return; }
  
  if (competition_state != CompetitionStates::STARTED && competition_state != CompetitionStates::ORDERS_COMPLETE) { return; }

  // Check if malfunction should be initiated
  if (!malfunctions.empty() && competiton_start_time.has_value()) {
    for (auto &malfunction: malfunctions) {

      if (malfunction.second) { continue; } // Malfunction has already been handled

      if (ros_node->get_clock()->now() - competiton_start_time.value() > rclcpp::Duration::from_seconds(malfunction.first.start_time)) {
        malfunction_end_time = ros_node->get_clock()->now() + rclcpp::Duration::from_seconds(malfunction.first.duration);
        malfunction.second = true;
        gzwarn << voltage_tester_name + " is malfunctioning\n"; 
        return;
      } 
    }
  }

  if (malfunction_end_time.has_value()) {
    // Check if malfunction duration has passed 
    if (ros_node->get_clock()->now() > malfunction_end_time.value()) {
      malfunction_end_time = std::nullopt;
      gzwarn << voltage_tester_name + " is working properly again\n"; 
    } else {
      voltage_reading.voltage = -1;
      voltage_reading.operation_status = OperationStates::MALFUNCTIONING;
      // voltage_tester_publisher->publish(voltage_reading);
      return;
    }
  }

  double current_time = _info.simTime.count();

  // Check if system should publish based on update rate
  if (current_time - last_publish_time < 1E9/update_rate) {
    return;
  }

  last_publish_time = current_time;

  double noise = noise_distribution(rng);

  voltage_reading.voltage = 0.0 + noise;
  voltage_reading.operation_status = OperationStates::OPERATIONAL;

  // Check if cell is no longer present
  if (current_time - last_contact_time > 1E8){ // 100 milliseconds
    cell_present = false;
    // voltage_tester_publisher->publish(voltage_reading);
    cell_voltage = std::nullopt;
    return;
  }

  // Read current cell voltage from component
  if (!cell_voltage.has_value()) {
    if (!_ecm.EntityByName(cell_name).has_value()) {
      gzerr << "Cell " + cell_name + " not found";
      return;
    }

    gz::sim::Entity cell_entity = _ecm.EntityByName(cell_name).value();

    if (!_ecm.EntityHasComponentType(cell_entity, gz::sim::components::Cell::typeId)){
      gzerr << "Cell " + cell_name + " does not have a cell component";
      return;
    }

    cell_voltage = _ecm.Component<gz::sim::components::Cell>(cell_entity)->Data().voltage;
  }
  
  voltage_reading.voltage = cell_voltage.value() + noise;
  // voltage_tester_publisher->publish(voltage_reading);    
}

void VoltageTesterPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg)
{
  competition_state = msg->competition_state;
  competiton_start_time = rclcpp::Time(msg->time.start, RCL_ROS_TIME);
}

void VoltageTesterPlugin::publish_voltage_cb() const
{
  voltage_tester_publisher->publish(voltage_reading);
}

void VoltageTesterPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){

  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();

    if (collision.find("cell") != std::string::npos){

      cell_name = collision.substr(0, collision.find("::"));

      cell_present = true;

      auto gz_time =_gz_contacts_msg.header().stamp();
      auto rcl_time = rclcpp::Time(gz_time.sec(), gz_time.nsec());
      last_contact_time = rcl_time.nanoseconds();
      break;
    }
  }    
}
