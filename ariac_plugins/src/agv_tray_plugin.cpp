#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <ariac_plugins/agv_tray_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::AgvTrayPlugin,
  gz::sim::System,
  ariac_plugins::AgvTrayPlugin::ISystemPreUpdate,
  ariac_plugins::AgvTrayPlugin::ISystemConfigure)

namespace ariac_plugins{

  AgvTrayPlugin::~AgvTrayPlugin()
  {
    executor->cancel();
    thread_executor_spin.join();
  }

  void AgvTrayPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    model = gz::sim::Model(_entity);

    agv_name = model.Name(_ecm);

    std::string ros_namespace = agv_name;

    tray_link = model.LinkByName(_ecm, "tray_link");

    gz_node = std::make_shared<gz::transport::Node>();

    // Subscribe to slot gz contact topics
    std::string left_contact_topic = "/world/ariac/model/" + agv_name + "/link/slot_{n}_link/sensor/left_contact_sensor/contact";
    std::string right_contact_topic = "/world/ariac/model/" + agv_name + "/link/slot_{n}_link/sensor/right_contact_sensor/contact";

    std::string center_contact_topic = "/world/ariac/model/" + agv_name + "/link/center_slot_link/sensor/contact_sensor/contact";

    // Create ROS node
    ros_node = rclcpp::Node::make_shared("tray_plugin", ros_namespace);

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

    // Add subscriber for location
    location_subscription = ros_node->create_subscription<ariac_interfaces::msg::AgvStatus>(
      "info", 
      10,
      std::bind(&AgvTrayPlugin::agv_station_check, this, std::placeholders::_1)
    );

    // Add service for removing cells at recycling

    recycle_cells_srv = ros_node->create_service<ariac_interfaces::srv::Trigger>(
      "recycle_cells",
      std::bind(&AgvTrayPlugin::recycle_cells_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create publisher and timer
    agv_slot_info_pub = ros_node->create_publisher<ariac_interfaces::msg::AgvTrayStatus>("tray_status", 10);

    pub_timer = ros_node->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AgvTrayPlugin::pub_timer_cb, this));

    for (int i=1; i<=4; i++) {
      std::string name = left_contact_topic;
      topic_names["left"][i] = name.replace(left_contact_topic.find("{n}"), 3, std::to_string(i));
      name = right_contact_topic;
      topic_names["right"][i] = name.replace(right_contact_topic.find("{n}"), 3, std::to_string(i));
    }

    gz_node->Subscribe(center_contact_topic, &AgvTrayPlugin::center_slot_contact_msg_cb, this);

    for (const auto& [side, slot_topic_names] : topic_names) {
      for (const auto& [slot, topic] : slot_topic_names) {
        std::function func = [this, slot, side](const gz::msgs::Contacts &msg) {contact_msg_cb(slot, side, msg); };
        gz_node->Subscribe(topic, func);
      }
    }	
  }
  
  void AgvTrayPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    switch(agv_station)
    {
    // In either of these situations, we want to lock any detected cells in place with detachable joints
    case AGVStations::IN_TRANSIT:
    case AGVStations::INSPECTION:

      for (const auto& [slot, slot_info] : cell_in_slot){
        if (slot_locked[slot]){
          continue;
        }

        if (!slot_info.left.in_contact || !slot_info.right.in_contact){ // Ensure contact on both sides of a slot
          continue;
        }
  
        if (slot_info.left.model_name != slot_info.right.model_name){ // Ensure object in contact is the same object
          continue;
        }
  
        std::optional<gz::sim::v8::Entity> cell_entity = _ecm.EntityByName(slot_info.left.model_name);

        if(!cell_entity.has_value()){
          cell_entity = _ecm.EntityByName(slot_info.right.model_name);
        }

        if (!cell_entity.has_value()) {
          cell_in_slot[slot].left.in_contact = false;
          cell_in_slot[slot].right.in_contact = false;
          cell_in_slot[slot].left.model_name = "";
          cell_in_slot[slot].right.model_name = "";
          slot_locked[slot] = false;
          continue;
        }
  
        auto cell_link = gz::sim::Model(cell_entity.value()).LinkByName(_ecm, "base_link");
  
        if (cell_link == 0) {
          gzerr << "Unable to locate cell link" << std::endl;
          continue;
        }
  
        lock_joints[slot] = _ecm.CreateEntity();
  
        // After finding the cell base link, attach the cell to the tray via a fixed detachable joint, creating a lock
        _ecm.CreateComponent(lock_joints[slot], gz::sim::components::DetachableJoint({tray_link, cell_link, "fixed"}));
  
        gzmsg << slot_info.left.model_name + " locked in slot " + std::to_string(slot) << std::endl;
  
        slot_locked[slot] = true;
        
      }

      break;
    // In either of these cases, we want to unlock the cells so that necessary operations can be performed using them
    case AGVStations::SHIPPING:
      for (const auto& [slot, slot_info] : cell_in_slot){
        if (!_ecm.HasEntity(lock_joints[slot])){
          lock_joints[slot] = gz::sim::kNullEntity;
          cell_in_slot[slot].left.in_contact = false;
          cell_in_slot[slot].right.in_contact = false;
          cell_in_slot[slot].left.model_name = "";
          cell_in_slot[slot].right.model_name = "";
          slot_locked[slot] = false;
        }
      }

      break;
    case AGVStations::ASSEMBLY:
      
      for (const auto& [slot, slot_info] : cell_in_slot){
        if (!slot_locked[slot]){
          continue;
        }
        
        // Remove the entity for the saved lock joint
        _ecm.RequestRemoveEntity(lock_joints[slot]);
        gzmsg << agv_name << " slot_" << slot << " unlocked" << std::endl;
        slot_locked[slot] = false;
        lock_joints[slot] = gz::sim::kNullEntity;
      }

      // Checking if cell is still in contact, remove cell from slot data if contact not recieved for 100 ms
      for (const auto& [slot, slot_info] : cell_in_slot) {
        if (_info.simTime.count() - slot_info.left.last_contact_time > 1E8) {
          cell_in_slot[slot].left.in_contact = false;
          cell_in_slot[slot].left.model_name = "";
        }

        if (_info.simTime.count() - slot_info.right.last_contact_time > 1E8) {
          cell_in_slot[slot].right.in_contact = false;
          cell_in_slot[slot].right.model_name = "";
        }
      }

      break;
    
    // At this station, we want to have a service which can be called to delete all the cells currently on the AGV
    case AGVStations::RECYCLING:

      if (recycle_requested){
        recycle_request_iteration = _info.iterations;
        recycle_requested = false;
      }

      // On the first iteration, remove the lock joint on the cell in each slot
      if (_info.iterations == recycle_request_iteration){
        for (const auto& [slot, slot_info] : cell_in_slot) { 
          if (!slot_locked[slot]){
            continue;
          }
          gzwarn << "about to remove joint for slot " << slot << std::endl;
          _ecm.RequestRemoveEntity(lock_joints[slot]);
          lock_joints[slot] = gz::sim::kNullEntity;
        }
      } else if(_info.iterations == recycle_request_iteration + 1) { // On the second iteration, we remove the cell from each slot
        for (const auto& [slot, slot_info] : cell_in_slot) { 
        
          if (!slot_locked[slot]){
            continue;
          }
  
          auto cell_to_remove = slot_info.left.model_name;
  
          auto cell_entity = _ecm.EntityByName(cell_to_remove);
  
          if (!cell_entity.has_value()){
            gzerr << "Unable to find cell to remove." << std::endl;
            break;
          }
          gzwarn << "about to remove cell for slot " << slot << std::endl;
          _ecm.RequestRemoveEntity(cell_entity.value());
  
          gzmsg << "Removed " + cell_to_remove + " from the world scene" << std::endl;
        }
      } else if (_info.iterations == recycle_request_iteration + 2){ // On the third iteration, reset our variables.

        for (const auto& [slot, slot_info] : cell_in_slot) {
          cell_in_slot[slot].left.in_contact = false;
          cell_in_slot[slot].right.in_contact = false;
          cell_in_slot[slot].left.model_name = "";
          cell_in_slot[slot].right.model_name = "";
          slot_locked[slot] = false;
        }
      }

      break;
    }

    switch (center_slot_state)
    {
    case CenterSlotState::IDLE:
      break;

    case CenterSlotState::TELEPORT_REQUESTED:

      if (!_ecm.EntityByName(cell_to_teleport).has_value()) {
        center_slot_state = CenterSlotState::IDLE;
        break;
      }
      
      auto cell_model = gz::sim::Model(_ecm.EntityByName(cell_to_teleport).value());
      auto cell_link = gz::sim::Link(cell_model.LinkByName(_ecm, "base_link"));

      if(!gz::sim::Link(tray_link).WorldPose(_ecm).has_value()){
        center_slot_state = CenterSlotState::IDLE;
        break;
      }

      if(!cell_link.WorldPose(_ecm).has_value()){
        center_slot_state = CenterSlotState::IDLE;
        break;
      }

      auto tray_pose = gz::sim::Link(tray_link).WorldPose(_ecm);

      auto cell_pose = cell_link.WorldPose(_ecm).value();

      float cell_z_offset = 0.0;
      if(cell_pose.Rot().RotateVector(gz::math::Vector3d::UnitZ).Dot(gz::math::Vector3d::UnitZ) < 0){
        cell_z_offset = 0.07;
      }

      gz::math::Pose3d teleport_cell_pose = gz::math::Pose3d(
        tray_pose.value().X(),
        tray_pose.value().Y(),
        tray_pose.value().Z() + 0.0101 + cell_z_offset,
        0.0,
        cell_z_offset > 0.0 ? M_PI : 0.0,
        cell_pose.Yaw()
      );
      

      cell_model.SetWorldPoseCmd(_ecm, teleport_cell_pose);

      center_slot_state = CenterSlotState::IDLE;

      break;
    }
  }

  void AgvTrayPlugin::contact_msg_cb(int slot, std::string side, const gz::msgs::Contacts &_gz_contacts_msg){
    auto cell = get_cell_in_contact(_gz_contacts_msg);

    if (cell.has_value()) {

      auto gz_time =_gz_contacts_msg.header().stamp();
      auto rcl_time = rclcpp::Time(gz_time.sec(), gz_time.nsec());
      double last_contact_time = rcl_time.nanoseconds(); // Save time to check in shipping and assembly case, to detect when contact is no longer recieved

      if (side == "left") {
        cell_in_slot[slot].left = {true, cell.value(), last_contact_time};
      }

      if (side == "right") {
        cell_in_slot[slot].right = {true, cell.value(), last_contact_time};
      }
      
    }
  }

  void AgvTrayPlugin::center_slot_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
    auto cell = get_cell_in_contact(_gz_contacts_msg);

    if(cell.has_value()){
      if(std::find(teleported_cells.begin(), teleported_cells.end(), cell.value()) == teleported_cells.end()){
        gzwarn << "Requesting teleport of cell: " << cell.value() << "\n";
        cell_to_teleport = cell.value();
        center_slot_state = CenterSlotState::TELEPORT_REQUESTED;
        teleported_cells.push_back(cell_to_teleport);
      }
    }
  }

  std::optional<std::string> AgvTrayPlugin::get_cell_in_contact(const gz::msgs::Contacts &_gz_contacts_msg)
  {
    for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();
      if (collision.find("cell") != std::string::npos){
        return collision.substr(0, collision.find("::"));
      }
    }

    return std::nullopt;
  }

  void AgvTrayPlugin::agv_station_check(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
    agv_station = msg->station_id;
  }

  void AgvTrayPlugin::recycle_cells_cb(const ariac_interfaces::srv::Trigger::Request::SharedPtr, ariac_interfaces::srv::Trigger::Response::SharedPtr rep){
    
    if (agv_station != ariac_interfaces::msg::AgvStations::RECYCLING){
      rep->success = false;
      rep->message = "Unable to run service unless AGV is at recycling station";
      return;
    }

    recycle_requested = true;
    rep->success = true;
    rep->message = "Removal of cells requested";
    return;
  }

  void AgvTrayPlugin::pub_timer_cb(){
    tray_status.slot_1_occupied = cell_in_slot[1].left.in_contact && cell_in_slot[1].right.in_contact;
    tray_status.slot_2_occupied = cell_in_slot[2].left.in_contact && cell_in_slot[2].right.in_contact;
    tray_status.slot_3_occupied = cell_in_slot[3].left.in_contact && cell_in_slot[3].right.in_contact;
    tray_status.slot_4_occupied = cell_in_slot[4].left.in_contact && cell_in_slot[4].right.in_contact;

    agv_slot_info_pub->publish(tray_status);
  }
}