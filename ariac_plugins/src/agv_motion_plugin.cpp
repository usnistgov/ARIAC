#include <ariac_plugins/agv_motion_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::AgvMotionPlugin,
  gz::sim::System,
  ariac_plugins::AgvMotionPlugin::ISystemPreUpdate,
  ariac_plugins::AgvMotionPlugin::ISystemConfigure)

namespace ariac_plugins{
  AgvMotionPlugin::AgvMotionPlugin() : velocity_planner(v_max, acc) {}

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
    agv_model = gz::sim::Model(_entity);

    std::string agv_name = agv_model.Name(_ecm);

    if(agv_name == "agv1"){
      start_location = { 2.15, 1.5 };
    } else if (agv_name == "agv2"){
      start_location = { 2.5, 1.5 };
    } else {
      start_location = { 2.85, 1.5 };
    }

    agv_base_link_entity = agv_model.LinkByName(_ecm, link_name);

    // Create link
    agv_base_link = gz::sim::Link(agv_base_link_entity);

    if (!rclcpp::ok()){
      rclcpp::init(0, nullptr);
    }

    // Create ROS node
    ros_node = rclcpp::Node::make_shared("motion_plugin", agv_name);

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

    status_msg.station_id = AGVStations::INSPECTION;

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
    if(wait_until_iteration.has_value() && _info.iterations < wait_until_iteration){
      // gzwarn << "AGV " << agv_model.Name(_ecm) << " waiting until iteration " << wait_until_iteration.value() << "\n";
      return;
    }
    wait_until_iteration = std::nullopt;

    auto pose_opt = agv_base_link.WorldPose(_ecm);
    if(!pose_opt.has_value()){
      throw std::runtime_error("Could not get pose for " + agv_model.Name(_ecm) + "\n");
    }        
    auto agv_pose = pose_opt.value();

    status_msg.pose = gz_to_ros_pose(agv_pose);
    
    switch(motion_state){
      case AGVMotionStatus::IDLE:
        break;
      case AGVMotionStatus::UNLOCK:
        if(lock_joint == gz::sim::kNullEntity){
          gzerr << "Unable to unlock " << agv_model.Name(_ecm) << ". Lock joint does not exist\n";
        }

        _ecm.RequestRemoveEntity(lock_joint);

        lock_joint = gz::sim::kNullEntity;

        motion_state = AGVMotionStatus::MOVING;
        gzwarn << "AGV " << agv_model.Name(_ecm) << " unlocked and starting motion\n";

        wait_until_iteration = _info.iterations + 50;
        break;
      case AGVMotionStatus::MOVING:
      {
        if(!current_goal_handle.has_value()){
          throw std::runtime_error(agv_model.Name(_ecm) + " in motion without a valid goal handle\n");
        }

        if(!motion_start_time.has_value()){
          velocity_planner.set_waypoints(current_waypoints, agv_pose.Yaw(), direction);
          motion_start_time = _info.simTime.count() / 1e9;
        }
        double current_time = _info.simTime.count()/1e9 - motion_start_time.value(); // current time in seconds
        
        status_msg.station_id = AGVStations::IN_TRANSIT;
        // Check if finished
        if (velocity_planner.is_finished(current_time)) {
          motion_state = AGVMotionStatus::TELEPORT;
          gzwarn << "AGV " << agv_model.Name(_ecm) << " reached end of path, teleporting to goal\n";
          wait_until_iteration = _info.iterations + 50;
          motion_start_time = std::nullopt;
          linear_velocity_vector.Set(0.0, 0.0, 0.0);
          angular_velocity_vector.Set(0.0, 0.0, 0.0);
        } else {
          // Pass current time to waypoint planner
          path_velocity_planner::PathVelocity vel = velocity_planner.get_velocity_at_time(current_time);
          
          
          linear_velocity_vector.Set(vel.linear, 0.0, (agv_pose.Z() < 0.005) ? 0.015 : 0.0);
          angular_velocity_vector.Set(0.0, 0.0, vel.angular);
        }

        agv_base_link.SetLinearVelocity(_ecm, linear_velocity_vector);
        agv_base_link.SetAngularVelocity(_ecm, angular_velocity_vector);

        // Publish feedback
        if (_info.iterations % (1000 / feedback_rate) == 0){
          auto feedback = std::make_shared<MoveAGVAction::Feedback>();
          feedback->status.station_id = AGVStations::IN_TRANSIT;
          feedback->status.pose = status_msg.pose;
          current_goal_handle.value()->publish_feedback(feedback);
        }

        break;
      }
      case AGVMotionStatus::TELEPORT:
      { 
        if(!current_goal_handle.has_value()){
          throw std::runtime_error(agv_model.Name(_ecm) + " in teleport without a valid goal handle\n");
        }
        // Teleport to destination pose
        auto destination = current_waypoints[current_waypoints.size() - 1];
        gz::math::Pose3d goal_pose;
        goal_pose.Set(
          gz::math::Vector3d(destination.x, destination.y, 0.005),
          gz::math::Vector3d(0, 0, station_yaw[current_goal_handle.value()->get_goal()->station_id])
        );

        agv_model.SetWorldPoseCmd(_ecm, goal_pose);
        
        status_msg.station_id = current_goal_handle.value()->get_goal()->station_id;
        wait_until_iteration = _info.iterations + 50;
        motion_state = AGVMotionStatus::LOCK;
        gzwarn << "AGV " << agv_model.Name(_ecm) << " teleported to station " << std::to_string(status_msg.station_id) << ". Locking\n";

        break;
      }
      
      case AGVMotionStatus::LOCK:
      {
        if(lock_joint != gz::sim::kNullEntity){
          throw std::runtime_error("Can't lock " + agv_model.Name(_ecm) + " since lock joint is not null");
        }
        lock_joint = _ecm.CreateEntity();

        std::optional<gz::sim::v8::Entity> floor_entity_opt = _ecm.EntityByName(floor_model_name);
      
        if (!floor_entity_opt.has_value()) {
          throw std::runtime_error("Unable to locate floor entity");
        }

        auto floor_link_entity = gz::sim::Model(floor_entity_opt.value()).LinkByName(_ecm, floor_link_name);

        if (floor_link_entity == gz::sim::kNullEntity) {
          throw std::runtime_error("Unable to locate floor link");
        }
        
        _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({floor_link_entity, agv_base_link_entity, "fixed"}));
        
        if(current_goal_handle.has_value()){
          wait_until_iteration = _info.iterations + 100;
          motion_state = AGVMotionStatus::COMPLETE_GOAL;
          gzwarn << "AGV " << agv_model.Name(_ecm) << " locked. Completing goal\n";
        } else{
          motion_state = AGVMotionStatus::IDLE;
          gzwarn << "AGV " << agv_model.Name(_ecm) << " locked with no current goal. Switching to IDLE\n";
        }
        break;
      }
      case AGVMotionStatus::COMPLETE_GOAL:
      {
        if(!current_goal_handle.has_value()){
          gzerr << "Current goal handle has no value";
        } else {
          try {
            auto gh = current_goal_handle.value();
            if (gh && gh->is_active()) {
              auto result = std::make_shared<MoveAGVAction::Result>();
              result->status.station_id = status_msg.station_id;
              result->status.pose = status_msg.pose;
              gzwarn << "Succeeding action\n";
              gh->succeed(result);
            } else {
              gzerr << "Goal handle not active; cannot succeed\n";
            }
          } catch (const std::exception &e) {
            gzerr << "Exception while completing goal: " << e.what() << "\n";
          }
          current_goal_handle = std::nullopt;
        }
        motion_state = AGVMotionStatus::IDLE;
        // gzwarn << "AGV " << agv_model.Name(_ecm) << " completed goal and is now IDLE\n";
        break;
      }
      default:
        throw std::runtime_error("BAD");
    }
  }

  rclcpp_action::GoalResponse AgvMotionPlugin::goal_recieved_cb(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveAGVAction::Goal> goal)
  {
    if (goal->station_id == status_msg.station_id || motion_state != AGVMotionStatus::IDLE) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (status_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::ASSEMBLY){
      current_waypoints = get_waypoints(AGVPath::INSPECTION_TO_ASSEMBLY);
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (status_msg.station_id == AGVStations::ASSEMBLY && goal->station_id == AGVStations::INSPECTION){
      current_waypoints = get_waypoints(AGVPath::ASSEMBLY_TO_INSPECTION);
      direction = path_velocity_planner::Direction::BACKWARD;
    } else if (status_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::SHIPPING){
      current_waypoints = get_waypoints(AGVPath::INSPECTION_TO_SHIPPING);
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (status_msg.station_id == AGVStations::SHIPPING && goal->station_id == AGVStations::INSPECTION){
      current_waypoints = get_waypoints(AGVPath::SHIPPING_TO_INSPECTION);
      direction = path_velocity_planner::Direction::BACKWARD;
    } else if (status_msg.station_id == AGVStations::INSPECTION && goal->station_id == AGVStations::RECYCLING){
      current_waypoints = get_waypoints(AGVPath::INSPECTION_TO_RECYCLING);
      direction = path_velocity_planner::Direction::FORWARD;
    } else if (status_msg.station_id == AGVStations::RECYCLING && goal->station_id == AGVStations::INSPECTION){
      current_waypoints = get_waypoints(AGVPath::RECYCLING_TO_INSPECTION);
      direction = path_velocity_planner::Direction::BACKWARD;
    } else if (status_msg.station_id == AGVStations::SHIPPING && goal->station_id == AGVStations::RECYCLING){
      current_waypoints = get_waypoints(AGVPath::SHIPPING_TO_RECYCLING);
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

    motion_state = AGVMotionStatus::UNLOCK;
  }

  void AgvMotionPlugin::pub_timer_cb()
  {
    agv_info_pub->publish(status_msg);
  }

  void AgvMotionPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
  {
    for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();

      if (collision.find("agv") != std::string::npos){
        
        collision_occurred = true;
      }
    }
  }

  std::vector<path_velocity_planner::Point> AgvMotionPlugin::get_waypoints(AGVPath path){    
    if(!start_location.has_value()){
      throw std::runtime_error("Start location not defined");
    }

    path_velocity_planner::Point start = start_location.value();
    
    std::vector<path_velocity_planner::Point> waypoints;

    switch(path){
      case AGVPath::INSPECTION_TO_ASSEMBLY:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 2.5, 3.5 },
          { 3.5, 4.5 },
          goal_locations[AGVStations::ASSEMBLY]
        };
        break;
      case AGVPath::ASSEMBLY_TO_INSPECTION:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 2.5, 3.5 },
          { 3.5, 4.5 },
          goal_locations[AGVStations::ASSEMBLY]
        };
        std::reverse(waypoints.begin(), waypoints.end());
        break;
      case AGVPath::INSPECTION_TO_SHIPPING:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 3.0, 3.5 },
          { 6.0, 3.5 },
          { 6.5, 3.0 },
          goal_locations[AGVStations::SHIPPING]
        };
        break;
      case AGVPath::SHIPPING_TO_INSPECTION:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 3.0, 3.5 },
          { 6.0, 3.5 },
          { 6.5, 3.0 },
          goal_locations[AGVStations::SHIPPING]
        };
        std::reverse(waypoints.begin(), waypoints.end());
        break;
      case AGVPath::INSPECTION_TO_RECYCLING:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 2.0, 3.5 },
          { 1.5, 3.5 },
          { 1.0, 4.0 },
          goal_locations[AGVStations::RECYCLING]
        };
        break;
      case AGVPath::RECYCLING_TO_INSPECTION:
        waypoints = {
          start,
          { start.x, 2.0},
          { (start.x + 2.5) / 2, 2.5},
          { 2.5, 3.0 },
          { 2.0, 3.5 },
          { 1.5, 3.5 },
          { 1.0, 4.0 },
          goal_locations[AGVStations::RECYCLING]
        };
        std::reverse(waypoints.begin(), waypoints.end());
        break;
      case AGVPath::SHIPPING_TO_RECYCLING:
        waypoints = {
          goal_locations[AGVStations::SHIPPING],
          { 6.5, 2.1 },
          { 5.5, 2.1 },
          { 5.5, 2.9 },
          { 4.9, 3.5 },
          { 1.5, 3.5 },
          { 1.0, 4.0 },
          goal_locations[AGVStations::RECYCLING]
        };
        break;
      default:
        throw std::runtime_error("Invalid path requested");
    }
    
    return waypoints;
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