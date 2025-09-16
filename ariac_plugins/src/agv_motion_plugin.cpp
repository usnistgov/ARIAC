#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <ariac_plugins/agv_motion_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::AgvMotionPlugin,
  gz::sim::System,
  ariac_plugins::AgvMotionPlugin::ISystemPreUpdate,
  ariac_plugins::AgvMotionPlugin::ISystemUpdate,
  ariac_plugins::AgvMotionPlugin::ISystemConfigure)

namespace ariac_plugins{
  AgvMotionPlugin::AgvMotionPlugin() : velocity_planner(v_max, acc) {

    start_locations["agv1"] = { 2.15, 1.5 };
    start_locations["agv2"] = { 2.5, 1.5 };
    start_locations["agv3"] = { 2.85, 1.5 };

    goal_locations[AGVStations::ASSEMBLY] = { 5, 4.5 };
    goal_locations[AGVStations::SHIPPING] = { 6.5, 2.55 };
    goal_locations[AGVStations::RECYCLING] = { 1.0, 6.6 };

    location_rotations[AGVStations::INSPECTION] = M_PI_2;
    location_rotations[AGVStations::ASSEMBLY] = 0;
    location_rotations[AGVStations::SHIPPING] = -M_PI_2;
    location_rotations[AGVStations::RECYCLING] = M_PI_2;


    waypoints["agv1"][AGVPath::INSPECTION_TO_ASSEMBLY] = {
      start_locations["agv1"],
      { 2.15, 2.0 },
      { 2.325, 2.5 },
      { 2.5, 3.0 },
      { 2.5, 3.5 },
      { 3.5, 4.5 },
      goal_locations[AGVStations::ASSEMBLY]
    };

    waypoints["agv2"][AGVPath::INSPECTION_TO_ASSEMBLY] = {
      start_locations["agv2"],
      { 2.5, 3.5 },
      { 3.5, 4.5 },
      goal_locations[AGVStations::ASSEMBLY]
    };

    waypoints["agv3"][AGVPath::INSPECTION_TO_ASSEMBLY] = {
      start_locations["agv3"],
      { 2.85, 2.0 },
      { 2.675, 2.5 },
      { 2.5, 3.0 },
      { 2.5, 3.5 },
      { 3.5, 4.5 },
      goal_locations[AGVStations::ASSEMBLY]
    };

    waypoints["agv1"][AGVPath::INSPECTION_TO_SHIPPING] = {
      start_locations["agv1"],
      { 2.15, 2.0 },
      { 2.325, 2.5 },
      { 2.5, 3.0 },
      { 3.0, 3.5 },
      { 6.0, 3.5 },
      { 6.5, 3.0 },
      goal_locations[AGVStations::SHIPPING]
    };

    waypoints["agv2"][AGVPath::INSPECTION_TO_SHIPPING] = {
      start_locations["agv2"],
      { 2.5, 3.0 },
      { 3.0, 3.5 },
      { 6.0, 3.5 },
      { 6.5, 3.0 },
      goal_locations[AGVStations::SHIPPING]
    };

    waypoints["agv3"][AGVPath::INSPECTION_TO_SHIPPING] = {
      start_locations["agv3"],
      { 2.85, 2.0 },
      { 2.675, 2.5 },
      { 2.5, 3.0 },
      { 3.0, 3.5 },
      { 6.0, 3.5 },
      { 6.5, 3.0 },
      goal_locations[AGVStations::SHIPPING]
    };

    waypoints["agv1"][AGVPath::INSPECTION_TO_RECYCLING] = {
      start_locations["agv1"],
      { 2.15, 2.0 },
      { 2.325, 2.5 },
      { 2.5, 3.0 },
      { 2.0, 3.5 },
      { 1.5, 3.5 },
      { 1.0, 4.0 },
      goal_locations[AGVStations::RECYCLING]
    };

    waypoints["agv2"][AGVPath::INSPECTION_TO_RECYCLING] = {
      start_locations["agv2"],
      { 2.5, 3.0 },
      { 2.0, 3.5 },
      { 1.5, 3.5 },
      { 1.0, 4.0 },
      goal_locations[AGVStations::RECYCLING]
    };

    waypoints["agv3"][AGVPath::INSPECTION_TO_RECYCLING] = {
      start_locations["agv3"],
      { 2.85, 2.0 },
      { 2.675, 2.5 },
      { 2.5, 3.0 },
      { 2.0, 3.5 },
      { 1.5, 3.5 },
      { 1.0, 4.0 },
      goal_locations[AGVStations::RECYCLING]
    };

    waypoints["agv1"][AGVPath::SHIPPING_TO_RECYCLING] = {
      goal_locations[AGVStations::SHIPPING],
      { 6.5, 2.1 },
      { 5.5, 2.1 },
      { 5.5, 2.9 },
      { 4.9, 3.5 },
      { 1.5, 3.5 },
      { 1.0, 4.0 },
      goal_locations[AGVStations::RECYCLING]
    };
    waypoints["agv2"][AGVPath::SHIPPING_TO_RECYCLING] = waypoints["agv1"][AGVPath::SHIPPING_TO_RECYCLING];
    waypoints["agv3"][AGVPath::SHIPPING_TO_RECYCLING] = waypoints["agv1"][AGVPath::SHIPPING_TO_RECYCLING];

    

    for (auto& agv_entry : waypoints) {
      const std::vector<path_velocity_planner::Point>& to_assem = agv_entry.second[AGVPath::INSPECTION_TO_ASSEMBLY];
      std::vector<path_velocity_planner::Point> assem_to_inspection = to_assem;
      std::reverse(assem_to_inspection.begin(), assem_to_inspection.end());
      agv_entry.second[AGVPath::ASSEMBLY_TO_INSPECTION] = assem_to_inspection;

      const std::vector<path_velocity_planner::Point>& to_shipping = agv_entry.second[AGVPath::INSPECTION_TO_SHIPPING];
      std::vector<path_velocity_planner::Point> ship_to_inspection = to_shipping;
      std::reverse(ship_to_inspection.begin(), ship_to_inspection.end());
      agv_entry.second[AGVPath::SHIPPING_TO_INSPECTION] = ship_to_inspection;

      const std::vector<path_velocity_planner::Point>& to_recycling = agv_entry.second[AGVPath::INSPECTION_TO_RECYCLING];
      std::vector<path_velocity_planner::Point> recycling_to_inspection = to_recycling;
      std::reverse(recycling_to_inspection.begin(), recycling_to_inspection.end());
      agv_entry.second[AGVPath::RECYCLING_TO_INSPECTION] = recycling_to_inspection;
    }
  }

  AgvMotionPlugin::~AgvMotionPlugin()
  {
    executor->cancel();
    thread_executor_spin.join();
  }

  void AgvMotionPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    // Create model from entity
    model = gz::sim::Model(_entity);

    agv_name = model.Name(_ecm);

    std::string ros_namespace = agv_name;

    agv_base_link_entity = model.LinkByName(_ecm, link_name);

    // Create link
    agv_base_link = gz::sim::Link(agv_base_link_entity);

    if (!rclcpp::ok()){
      rclcpp::init(0, nullptr);
    }

    // Create ROS node
    ros_node = rclcpp::Node::make_shared("motion_plugin", ros_namespace);

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

    // Create GZ node
    gz_node = std::make_shared<gz::transport::Node>();

    // Subscribe to gz contact topic 
    std::string gz_contact_topic = "/world/ariac/model/" + agv_name + "/link/agv/sensor/collision_detector/contact";
    gz_node->Subscribe(gz_contact_topic, &AgvMotionPlugin::contact_msg_cb, this);

    // Create action server
    action_server = rclcpp_action::create_server<MoveAGVAction>(
      ros_node, 
      "move",
      std::bind(&AgvMotionPlugin::goal_recieved_cb, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&AgvMotionPlugin::goal_cancel_cb, this, std::placeholders::_1),
      std::bind(&AgvMotionPlugin::goal_accepted_cb, this, std::placeholders::_1)
    );

    // Create publisher and timer
    agv_info_pub = ros_node->create_publisher<AGVStatus>("info", 10);

    pub_timer = ros_node->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AgvMotionPlugin::pub_timer_cb, this));
  }
  
  void AgvMotionPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    switch (lock_state)
    {
    case AGVLockState::LOCK_REQUESTED:
      // Lock joint
      lock_joint = _ecm.CreateEntity();

      _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({floor_link_entity, agv_base_link_entity, "fixed"}));

      lock_state = AGVLockState::LOCKED;
      break;
    
    case AGVLockState::UNLOCK_REQUESTED:
      // Unlock joint
      _ecm.RequestRemoveEntity(lock_joint);
      lock_joint = gz::sim::kNullEntity;

      lock_state = AGVLockState::UNLOCKED;
      break;
    }
  }

  void AgvMotionPlugin::Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    if(_info.paused){
      return;
    }

    info_msg.pose = gz_to_ros_pose(agv_base_link.WorldPose(_ecm).value());

    switch (motion_state)
    {
    case AGVMotionStatus::CONFIGURE: {

      std::optional<gz::sim::v8::Entity> track = _ecm.EntityByName(floor_model_name);
      
      if (!track.has_value()) {
        RCLCPP_ERROR(ros_node->get_logger(), "Unable to locate agv track model");
        throw std::runtime_error("Unable to locate agv track model");
      }

      floor_link_entity = gz::sim::Model(track.value()).LinkByName(_ecm, floor_link_name);

      if (floor_link_entity == 0) {
        RCLCPP_ERROR(ros_node->get_logger(), "Unable to locate agv track link");
        throw std::runtime_error("Unable to locate agv track link");
      }

      lock_state = AGVLockState::LOCK_REQUESTED;

      motion_state = AGVMotionStatus::IDLE;

      info_msg.station_id = AGVStations::INSPECTION;

      break;
    }

    case AGVMotionStatus::IDLE:
      // Do nothing
      break;

    case AGVMotionStatus::PROCESSING:
      // Wait for agv to be unlocked
      if (lock_state == AGVLockState::LOCKED) {
        lock_state = AGVLockState::UNLOCK_REQUESTED;
        break;
      } 

      start_time = _info.simTime.count() / 1e9;
      last_feedback_time = 0.0;

      // Set waypoints for path
      velocity_planner.set_waypoints(
        waypoints[agv_name][current_path], 
        agv_base_link.WorldPose(_ecm).value().Yaw(),
        direction);

      motion_state = AGVMotionStatus::MOVING;

      info_msg.station_id = AGVStations::IN_TRANSIT;

      break;

    case AGVMotionStatus::MOVING: {
      double current_time = _info.simTime.count()/1e9 - start_time; // current time in seconds
      
      // Check if finished
      if (velocity_planner.is_finished(current_time)) {
        motion_state = AGVMotionStatus::MOTION_FINISHED;
        linear_velocity_vector.Set(0.0, 0.0, 0.0);
        angular_velocity_vector.Set(0.0, 0.0, 0.0);
      } else {
        // Pass current time to waypoint planner
        path_velocity_planner::PathVelocity vel = velocity_planner.get_velocity_at_time(current_time);

        linear_velocity_vector.Set(vel.linear, 0.0, 0.0);
        angular_velocity_vector.Set(0.0, 0.0, vel.angular);
      }

      // Publish feedback
      if (current_time - last_feedback_time > 1/feedback_rate){
        auto feedback = std::make_shared<MoveAGVAction::Feedback>();
        feedback->status.station_id = AGVStations::IN_TRANSIT;
        feedback->status.pose = info_msg.pose;
        current_goal_handle->publish_feedback(feedback);
        last_feedback_time = current_time;
      }

      agv_base_link.SetLinearVelocity(_ecm, linear_velocity_vector);
      agv_base_link.SetAngularVelocity(_ecm, angular_velocity_vector);

      break;
    }

    case AGVMotionStatus::MOTION_FINISHED: {
      
      if (!is_at_target_pose(agv_base_link.WorldPose(_ecm).value())){
        motion_state = AGVMotionStatus::TELEPORTING;
        break;
      }
      
      lock_state = AGVLockState::LOCK_REQUESTED;

      motion_state = AGVMotionStatus::IDLE;

      info_msg.station_id = destination_station;

      if (collision_occurred) {
        // Create penalty component
        ariac_components::Penalty penalty = ariac_components::Penalty{
          ariac_components::PenaltyType::AGV_COLLISION,
          static_cast<double>(_info.simTime.count()),
          agv_name + " in collision"
        };

        gz::sim::Entity penalty_entity = _ecm.CreateEntity();
        _ecm.CreateComponent(penalty_entity, gz::sim::components::Penalty(penalty));
      }

      if (!current_goal_handle->is_active()) { // agv was stationary when collision occurred 
        collision_occurred = false;
        break;
      }
      
      auto result = std::make_shared<MoveAGVAction::Result>();
      
      result->status.station_id = info_msg.station_id;
      result->status.pose = info_msg.pose;
      
      if (collision_occurred) {
        current_goal_handle->abort(result);
        collision_occurred = false;
      } else {
        current_goal_handle->succeed(result);
      }

      break;
    }

    case AGVMotionStatus::TELEPORTING: {
      // Wait for agv to be unlocked
      if (lock_state == AGVLockState::LOCKED) {
        lock_state = AGVLockState::UNLOCK_REQUESTED;
        break;
      }

      // Teleport to destination pose
      auto destination = get_destination();
      goal_pose.Set(
        gz::math::Vector3d(destination.first.x, destination.first.y, 0.005),
        gz::math::Vector3d(0, 0, destination.second)
      );

      model.SetWorldPoseCmd(_ecm, goal_pose);

      motion_state = AGVMotionStatus::MOTION_FINISHED;
    }
    }
  }

  rclcpp_action::GoalResponse AgvMotionPlugin::goal_recieved_cb(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveAGVAction::Goal> goal)
  {
    if (goal->station_id == info_msg.station_id || motion_state != AGVMotionStatus::IDLE) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (info_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::ASSEMBLY){
      current_path = AGVPath::INSPECTION_TO_ASSEMBLY;
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (info_msg.station_id == AGVStations::ASSEMBLY && goal->station_id == AGVStations::INSPECTION){
      current_path = AGVPath::ASSEMBLY_TO_INSPECTION;
      direction = path_velocity_planner::Direction::BACKWARD;
    } else if (info_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::SHIPPING){
      current_path = AGVPath::INSPECTION_TO_SHIPPING;
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (info_msg.station_id == AGVStations::SHIPPING && goal->station_id == AGVStations::INSPECTION){
      current_path = AGVPath::SHIPPING_TO_INSPECTION;
      direction = path_velocity_planner::Direction::BACKWARD;
    } else if (info_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::RECYCLING){
      current_path = AGVPath::INSPECTION_TO_RECYCLING;
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (info_msg.station_id == AGVStations::RECYCLING && goal->station_id == AGVStations::INSPECTION){
      current_path = AGVPath::RECYCLING_TO_INSPECTION;
      direction = path_velocity_planner::Direction::BACKWARD;
    }else if (info_msg.station_id == AGVStations::SHIPPING && goal->station_id == AGVStations::RECYCLING){
      current_path = AGVPath::SHIPPING_TO_RECYCLING;
      direction = path_velocity_planner::Direction::FORWARD;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse AgvMotionPlugin::goal_cancel_cb(const GoalHandlePtr)
  {
    // Don't allow goal cancellation 
    return rclcpp_action::CancelResponse::REJECT;
  }
  
  void AgvMotionPlugin::goal_accepted_cb(GoalHandlePtr goal_handle)
  {
    current_goal_handle = goal_handle;

    destination_station = current_goal_handle->get_goal()->station_id;

    motion_state = AGVMotionStatus::PROCESSING;
  }

  void AgvMotionPlugin::pub_timer_cb()
  {
    agv_info_pub->publish(info_msg);
  }

  void AgvMotionPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
  {
    for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();

      if (collision.find("agv") != std::string::npos){

        collision_occurred = true;

        if (lock_state == AGVLockState::LOCKED) {
          lock_state = AGVLockState::UNLOCK_REQUESTED;
        }
        
        destination_station = AGVStations::INSPECTION;
        motion_state = AGVMotionStatus::TELEPORTING;
      }
    }
  }

  bool AgvMotionPlugin::is_at_target_pose(gz::math::Pose3d current_pose)
  {
    auto destination = get_destination();

    bool location_correct = std::hypot(destination.first.x - current_pose.X(), destination.first.y - current_pose.Y()) < goal_distance_threshold;
    bool rotation_correct = abs(angles::shortest_angular_distance(current_pose.Yaw(), destination.second) < goal_angle_threshold);

    return location_correct && rotation_correct;
  }

  std::pair<path_velocity_planner::Point, double> AgvMotionPlugin::get_destination()
  {
    path_velocity_planner::Point destination_point;

    if (destination_station == AGVStations::INSPECTION) {
      destination_point = start_locations[agv_name];
    } else {
      destination_point = goal_locations[destination_station];
    }

    double destination_rotation = location_rotations[destination_station];

    return std::make_pair(destination_point, destination_rotation);
  }

  geometry_msgs::msg::Pose AgvMotionPlugin::gz_to_ros_pose(const gz::math::Pose3d &gz_pose)
  {
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = gz_pose.Pos().X();
    ros_pose.position.y = gz_pose.Pos().Y();
    ros_pose.position.z = gz_pose.Pos().Z();
    ros_pose.orientation.x = gz_pose.Rot().X();
    ros_pose.orientation.y = gz_pose.Rot().Y();
    ros_pose.orientation.z = gz_pose.Rot().Z();
    ros_pose.orientation.w = gz_pose.Rot().W();
    return ros_pose;
  }
}