#include <test_competitor/test_competitor.hpp>

TestCompetitor::TestCompetitor()
 : Node("test_competitor"),
  floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
  ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
  planning_scene_()
{
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);

  // Subscribe to topics
  rclcpp::SubscriptionOptions options;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  options.callback_group = topic_cb_group_;

  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1, 
    std::bind(&TestCompetitor::orders_cb, this, std::placeholders::_1), options);

  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1, 
    std::bind(&TestCompetitor::competition_state_cb, this, std::placeholders::_1), options);

  kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::kts1_camera_cb, this, std::placeholders::_1), options);

  kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::kts2_camera_cb, this, std::placeholders::_1), options);

  left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::left_bins_camera_cb, this, std::placeholders::_1), options);

  right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
    "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::right_bins_camera_cb, this, std::placeholders::_1), options);

  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(), 
    std::bind(&TestCompetitor::floor_gripper_state_cb, this, std::placeholders::_1), options);

  // Initialize service clients 
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");

  AddModelsToPlanningScene();

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

TestCompetitor::~TestCompetitor() 
{
  floor_robot_.~MoveGroupInterface();
  ceiling_robot_.~MoveGroupInterface();
}

void TestCompetitor::orders_cb(
  const ariac_msgs::msg::Order::ConstSharedPtr msg) 
{
  orders_.push_back(*msg);
}

void TestCompetitor::competition_state_cb(
  const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) 
{
  competition_state_ = msg->competition_state;
}

void TestCompetitor::kts1_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!kts1_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }
  
  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::kts2_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!kts2_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::left_bins_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!left_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::right_bins_camera_cb(
  const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!right_bins_camera_recieved_data) {
    RCLCPP_INFO(get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::floor_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}

geometry_msgs::msg::Pose TestCompetitor::MultiplyPose(
  geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  KDL::Frame f3 = f1*f2;
  
  return tf2::toMsg(f3);
}

geometry_msgs::msg::Pose TestCompetitor::BuildPose(
  double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

geometry_msgs::msg::Pose TestCompetitor::FrameWorldPose(std::string frame_id){
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

double TestCompetitor::GetYaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

geometry_msgs::msg::Quaternion TestCompetitor::QuaternionFromRPY(double r, double p, double y){
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(r, p, y);

  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

void TestCompetitor::AddModelToPlanningScene(
  std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}

void TestCompetitor::AddModelsToPlanningScene()
{
  std::map<std::string, std::pair<double, double>> bin_positions = {
    {"bin1", std::pair<double, double>(-1.9, 3.375)},
    {"bin2", std::pair<double, double>(-1.9, 2.625)},
    {"bin3", std::pair<double, double>(-2.65, 2.625)},
    {"bin4", std::pair<double, double>(-2.65, 3.375)},
    {"bin5", std::pair<double, double>(-1.9, -3.375)},
    {"bin6", std::pair<double, double>(-1.9, -2.625)},
    {"bin7", std::pair<double, double>(-2.65, -2.625)},
    {"bin8", std::pair<double, double>(-2.65, -3.375)}
  };

  geometry_msgs::msg::Pose bin_pose;
  for (auto const& bin : bin_positions) {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

  AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

bool TestCompetitor::FloorRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }

}

bool TestCompetitor::FloorRobotMoveCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

geometry_msgs::msg::Quaternion TestCompetitor::FloorRobotSetOrientation(double rotation)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);
  
  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q; 
}

void TestCompetitor::FloorRobotWaitForAttach(double timeout){
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

void TestCompetitor::FloorRobotSendHome()
{
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  FloorRobotMovetoTarget();
}

bool TestCompetitor::FloorRobotSetGripperState(bool enable)
{
  if (floor_gripper_state_.enabled == enable) {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

bool TestCompetitor::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
  // Move gripper into tool changer
  auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, FloorRobotSetOrientation(0.0)));
  
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z, FloorRobotSetOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  
  if (gripper_type == "trays") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto result =floor_robot_tool_changer_->async_send_request(request);
  result.wait();
  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, FloorRobotSetOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
    return false;

  return true;

}

bool TestCompetitor::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
  // Check if kit tray is on one of the two tables
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool found_tray = false;

  // Check table 1
  for (auto tray: kts1_trays_) {
    if (tray.id == tray_id) {
      station = "kts1";
      tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray) {
    for (auto tray: kts2_trays_) {
      if (tray.id == tray_id) {
        station = "kts2";
        tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
        found_tray = true;
        break;
      }
    }
  }
  if (!found_tray)
    return false;

  double tray_rotation = GetYaw(tray_pose);

  // Move floor robot to the corresponding kit tray table
  if (station == "kts1") {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  } else {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper") {
    FloorRobotChangeGripper(station, "trays");
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
    tray_pose.position.z + 0.2, FloorRobotSetOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
    tray_pose.position.z + pick_offset_, FloorRobotSetOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_.attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
    tray_pose.position.z + 0.2, FloorRobotSetOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  FloorRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = GetYaw(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
    agv_tray_pose.position.z + 0.3, FloorRobotSetOrientation(agv_rotation)));
 
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
    agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, FloorRobotSetOrientation(agv_rotation)));
  
  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  FloorRobotSetGripperState(false);

  floor_robot_.detachObject(tray_name);

  LockAGVTray(agv_num);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
    agv_tray_pose.position.z + 0.3, FloorRobotSetOrientation(0)));
  
  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;
}

bool TestCompetitor::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " <<
    part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part: left_bins_parts_) {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color) {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part) {
    for (auto part: right_bins_parts_) {
      if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color) {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    } 
  }
  if (!found_part) {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = GetYaw(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper") {
    std::string station;
    if (part_pose.position.y < 0) {
      station = "kts1";
    } else {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    FloorRobotChangeGripper(station, "parts");
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
    part_pose.position.z + 0.5, FloorRobotSetOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
    part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, FloorRobotSetOrientation(part_rotation)));
  
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  FloorRobotSetGripperState(true);

  FloorRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
    part_pose.position.z + 0.3, FloorRobotSetOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  return true;
}

bool TestCompetitor::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
  if (!floor_gripper_state_.attached) {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }

  // Move to agv
  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  FloorRobotMovetoTarget();

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0, 
    geometry_msgs::msg::Quaternion());

  auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
    part_drop_pose.position.z + 0.3, FloorRobotSetOrientation(0)));

  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
    part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
    FloorRobotSetOrientation(0)));
  
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  // Drop part in quadrant
  FloorRobotSetGripperState(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] + 
    "_" + part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);

  waypoints.clear();
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
    part_drop_pose.position.z + 0.3,
    FloorRobotSetOrientation(0)));
  
  
  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;

}

void TestCompetitor::CeilingRobotSendHome()
{
  // Move floor robot to home joint state
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}

bool TestCompetitor::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }

}


bool TestCompetitor::CompleteOrders(){
  // Wait for first order to be published
  while (orders_.size() == 0) {}

  bool success;
  while (true) {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false;
      break;
    }

    if (orders_.size() == 0){
      if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
        // wait for more orders
        RCLCPP_INFO(get_logger(), "Waiting for orders...");
        while (orders_.size() == 0) {}
      } else {
        RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        break;
      }
    }
    
    current_order_ = orders_.front();
    orders_.erase(orders_.begin());

    if (current_order_.type == ariac_msgs::msg::Order::KITTING) {
      TestCompetitor::CompleteKittingTask(current_order_.kitting_task);
      // Submit order
      TestCompetitor::SubmitOrder(current_order_.id);
    } else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY) {
      TestCompetitor::CompleteAssemblyTask(current_order_.assembly_task);
    } else if (current_order_.type == ariac_msgs::msg::Order::COMBINED) {
      TestCompetitor::CompleteCombinedTask(current_order_.combined_task);
    }
  }

  return success;
}

bool TestCompetitor::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{
  FloorRobotSendHome();

  FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

  for (auto kit_part: task.parts) {
    FloorRobotPickBinPart(kit_part.part);
    FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
  }

  // Check quality
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = current_order_.id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

  if (!result.get()->all_passed) {
    RCLCPP_ERROR(get_logger(), "Issue with shipment");
  }

  MoveAGV(task.agv_number, task.destination);

  return true;
}

bool TestCompetitor::CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task)
{
  if (task.agv_numbers.front() == 1){

  }

  RCLCPP_WARN(get_logger(), "Assembly tasks not yet implemented");
  return false;
}

bool TestCompetitor::CompleteCombinedTask(ariac_msgs::msg::CombinedTask task)
{
  if (task.station == 1){

  }
  RCLCPP_WARN(get_logger(), "Combined tasks not yet implemented");
  return false;
}

bool TestCompetitor::StartCompetition()
{
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {}

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool TestCompetitor::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool TestCompetitor::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool TestCompetitor::LockAGVTray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool TestCompetitor::MoveAGV(int agv_num, int destination)
{
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}