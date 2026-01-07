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
    std::string gz_contact_topic = "/world/ariac/model/" + agv_name + "/link/base_link/sensor/collision_detector/contact";
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
        if (status_msg.station_id == AGVStations::INSPECTION) {
          motion_state == AGVMotionStatus::HOLD_POSITION;
        }

        break;
      
      case AGVMotionStatus::HOLD_POSITION:
      {
        if (status_msg.station_id == AGVStations::IN_TRANSIT) {
          gzerr << "Cannot hold position while in transit\n";
          motion_state = AGVMotionStatus::IDLE;
          break;
        }

        auto velocities = compute_hold_position_velocity(status_msg.station_id, agv_pose);

        agv_base_link.SetLinearVelocity(_ecm, velocities.first);
        agv_base_link.SetAngularVelocity(_ecm, velocities.second);

        break;
      }

      case AGVMotionStatus::MOVING:
      {
        if(!current_goal_handle.has_value()){
          throw std::runtime_error(agv_model.Name(_ecm) + " in motion without a valid goal handle\n");
        }

        if (collision_occurred) {
          gzerr << "AGV Collision. Teleporting back to inspection]\n";

          // Create penalty component
          ariac_components::Penalty penalty = ariac_components::Penalty{
            ariac_components::PenaltyType::AGV_COLLISION,
            static_cast<double>(_info.simTime.count()),
            agv_model.Name(_ecm) + " in collision"
          };

          gz::sim::Entity penalty_entity = _ecm.CreateEntity();
          _ecm.CreateComponent(penalty_entity, gz::sim::components::Penalty(penalty));

          motion_start_time = std::nullopt;

          motion_state = AGVMotionStatus::TELEPORT;
          break;
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
          wait_until_iteration = _info.iterations + teleport_wait_iterations;
          motion_start_time = std::nullopt;
          linear_velocity_vector.Set(0.0, 0.0, 0.0);
          angular_velocity_vector.Set(0.0, 0.0, 0.0);
        } else {
          // Pass current time to waypoint planner
          path_velocity_planner::PathVelocity vel = velocity_planner.get_velocity_at_time(current_time);
          
          linear_velocity_vector.Set(vel.linear, 0.0, (agv_pose.Z() < z_threshold) ? z_lift_velocity : 0.0);
          angular_velocity_vector.Set(0.0, 0.0, vel.angular);
        }

        agv_base_link.SetLinearVelocity(_ecm, linear_velocity_vector);
        agv_base_link.SetAngularVelocity(_ecm, angular_velocity_vector);

        // Publish feedback
        if (_info.iterations % (feedback_publish_interval / feedback_rate) == 0){
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
        
        path_velocity_planner::Point teleport_location;
        
        if (collision_occurred) {
          teleport_location = start_location.value();
          status_msg.station_id = station_yaw[AGVStations::INSPECTION];
        } else {
          teleport_location = current_waypoints[current_waypoints.size() - 1];
          status_msg.station_id = current_goal_handle.value()->get_goal()->station_id;
        }

        gz::math::Pose3d goal_pose;
        goal_pose.Set(
          gz::math::Vector3d(teleport_location.x, teleport_location.y, z_threshold),
          gz::math::Vector3d(0, 0,  station_yaw[status_msg.station_id])
        );

        agv_model.SetWorldPoseCmd(_ecm, goal_pose);

        wait_until_iteration = _info.iterations + complete_goal_wait_iterations;
        motion_state = AGVMotionStatus::COMPLETE_GOAL;
        
        break;
      }
      
      case AGVMotionStatus::COMPLETE_GOAL:
      {
        if(current_goal_handle.has_value()){
          auto result = std::make_shared<MoveAGVAction::Result>();
          result->status.station_id = status_msg.station_id;
          result->status.pose = status_msg.pose;
          
          if (collision_occurred) {
            current_goal_handle.value()->abort(result);
            collision_occurred = false;
          } else {
            current_goal_handle.value()->succeed(result);
          }

          current_goal_handle = std::nullopt;
        } else {
          gzerr << "Current goal handle has no value";
        }
        motion_state = AGVMotionStatus::IDLE;

        break;
      }
      default:
        throw std::runtime_error("motion state not valid");
    }
  }

  rclcpp_action::GoalResponse AgvMotionPlugin::goal_recieved_cb(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveAGVAction::Goal> goal)
  {
    bool valid_state = motion_state == AGVMotionStatus::IDLE || motion_state == AGVMotionStatus::HOLD_POSITION;
    if (goal->station_id == status_msg.station_id || !valid_state) {
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

    motion_state = AGVMotionStatus::MOVING;
  }

  void AgvMotionPlugin::pub_timer_cb()
  {
    agv_info_pub->publish(status_msg);
  }

  void AgvMotionPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
  {
    for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();

      if (collision.find("agv") != std::string::npos && collision.find("tray") == std::string::npos && motion_state == AGVMotionStatus::MOVING){
        
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

  std::pair<gz::math::Vector3d, gz::math::Vector3d> AgvMotionPlugin::compute_hold_position_velocity(
    int station_id, const gz::math::Pose3d &current_pose)
  {
    // Determine target position
    double target_x, target_y;

    if (station_id == AGVStations::INSPECTION) {
      if (!start_location.has_value()) {
        throw std::runtime_error("Start location not set, unable to hold position");
      }
      target_x = start_location.value().x;
      target_y = start_location.value().y;
    } else {
      target_x = goal_locations[station_id].x;
      target_y = goal_locations[station_id].y;
    }

    double target_yaw = station_yaw[station_id];

    // Helper to compute simple proportional control velocity
    auto compute_vel = [this](double current, double target) -> double {
      if (current == target) return 0.0;
      return (current < target) ? hold_position_velocity : -hold_position_velocity;
    };

    // Compute desired velocities in world frame
    double world_vx = compute_vel(current_pose.X(), target_x);
    double world_vy = compute_vel(current_pose.Y(), target_y);
    double rot_vel = compute_vel(current_pose.Yaw(), target_yaw);

    // Transform world velocities to AGV body frame using rotation matrix
    double cos_yaw = std::cos(target_yaw);
    double sin_yaw = std::sin(target_yaw);

    double x_vel = cos_yaw * world_vx + sin_yaw * world_vy;
    double y_vel = -sin_yaw * world_vx + cos_yaw * world_vy;

    gz::math::Vector3d linear_vel(x_vel, y_vel, (current_pose.Z() < z_threshold) ? z_lift_velocity : 0.0);
    gz::math::Vector3d angular_vel(0.0, 0.0, rot_vel);

    return {linear_vel, angular_vel};
  }
}