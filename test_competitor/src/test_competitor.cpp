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

  ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
      "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
      std::bind(&TestCompetitor::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

  as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&TestCompetitor::as1_state_cb, this, std::placeholders::_1), options);

  as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&TestCompetitor::as2_state_cb, this, std::placeholders::_1), options);

  as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&TestCompetitor::as3_state_cb, this, std::placeholders::_1), options);

  as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
      "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(),
      std::bind(&TestCompetitor::as4_state_cb, this, std::placeholders::_1), options);

  agv1_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv1_status", 10,
      std::bind(&TestCompetitor::agv1_status_cb, this, std::placeholders::_1), options);

  agv2_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&TestCompetitor::agv2_status_cb, this, std::placeholders::_1), options);

  agv3_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&TestCompetitor::agv3_status_cb, this, std::placeholders::_1), options);

  agv4_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&TestCompetitor::agv4_status_cb, this, std::placeholders::_1), options);

  // Initialize service clients
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
  pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
  ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");

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
  if (!kts1_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_recieved_data = true;
  }

  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!kts2_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_recieved_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!left_bins_camera_recieved_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void TestCompetitor::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!right_bins_camera_recieved_data)
  {
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

void TestCompetitor::ceiling_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
  ceiling_gripper_state_ = *msg;
}

void TestCompetitor::as1_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void TestCompetitor::as2_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void TestCompetitor::as3_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}

void TestCompetitor::as4_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

void TestCompetitor::agv1_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[1] = msg->location;
}

void TestCompetitor::agv2_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[2] = msg->location;
}

void TestCompetitor::agv3_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[3] = msg->location;
}

void TestCompetitor::agv4_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[4] = msg->location;
}

geometry_msgs::msg::Pose TestCompetitor::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
  KDL::Frame f1;
  KDL::Frame f2;

  tf2::fromMsg(p1, f1);
  tf2::fromMsg(p2, f2);

  KDL::Frame f3 = f1 * f2;

  return tf2::toMsg(f3);
}

void TestCompetitor::LogPose(geometry_msgs::msg::Pose p)
{
  tf2::Quaternion q(
      p.orientation.x,
      p.orientation.y,
      p.orientation.z,
      p.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  roll *= 180 / M_PI;
  pitch *= 180 / M_PI;
  yaw *= 180 / M_PI;

  RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
              p.position.x, p.position.y, p.position.z,
              roll, pitch, yaw);
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

geometry_msgs::msg::Pose TestCompetitor::FrameWorldPose(std::string frame_id)
{
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try
  {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
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

geometry_msgs::msg::Quaternion TestCompetitor::QuaternionFromRPY(double r, double p, double y)
{
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
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
      {"bin1", std::pair<double, double>(-1.9, 3.375)},
      {"bin2", std::pair<double, double>(-1.9, 2.625)},
      {"bin3", std::pair<double, double>(-2.65, 2.625)},
      {"bin4", std::pair<double, double>(-2.65, 3.375)},
      {"bin5", std::pair<double, double>(-1.9, -3.375)},
      {"bin6", std::pair<double, double>(-1.9, -2.625)},
      {"bin7", std::pair<double, double>(-2.65, -2.625)},
      {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions)
  {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
      {"as1", std::pair<double, double>(-7.3, 3)},
      {"as2", std::pair<double, double>(-12.3, 3)},
      {"as3", std::pair<double, double>(-7.3, -3)},
      {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions)
  {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions)
  {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
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

geometry_msgs::msg::Quaternion TestCompetitor::SetRobotOrientation(double rotation)
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

bool TestCompetitor::FloorRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(floor_robot_.execute(plan));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool TestCompetitor::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9)
  {
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

void TestCompetitor::FloorRobotWaitForAttach(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout))
    {
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
  if (floor_gripper_state_.enabled == enable)
  {
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

  if (!result.get()->success)
  {
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
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  }
  else if (gripper_type == "parts")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto result = floor_robot_tool_changer_->async_send_request(request);
  result.wait();
  if (!result.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

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
  for (auto tray : kts1_trays_)
  {
    if (tray.id == tray_id)
    {
      station = "kts1";
      tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray)
  {
    for (auto tray : kts2_trays_)
    {
      if (tray.id == tray_id)
      {
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
  if (station == "kts1")
  {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  }
  else
  {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  FloorRobotMovetoTarget();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper")
  {
    FloorRobotChangeGripper(station, "trays");
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
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
                                tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  FloorRobotMovetoTarget();

  auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = GetYaw(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  FloorRobotSetGripperState(false);

  floor_robot_.detachObject(tray_name);

  LockAGVTray(agv_num);

  waypoints.clear();
  waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;
}

bool TestCompetitor::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part : left_bins_parts_)
  {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
    {
      part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part)
  {
    for (auto part : right_bins_parts_)
    {
      if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
      {
        part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    }
  }
  if (!found_part)
  {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = GetYaw(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station;
    if (part_pose.position.y < 0)
    {
      station = "kts1";
    }
    else
    {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
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
                                part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

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
                                part_pose.position.z + 0.3, SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  return true;
}

bool TestCompetitor::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
  if (!floor_gripper_state_.attached)
  {
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
                                part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

  // Drop part in quadrant
  FloorRobotSetGripperState(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                          "_" + part_types_[floor_robot_attached_part_.type];
  floor_robot_.detachObject(part_name);

  waypoints.clear();
  waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                part_drop_pose.position.z + 0.3,
                                SetRobotOrientation(0)));

  FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  return true;
}

void TestCompetitor::CeilingRobotSendHome()
{
  // Move ceiling robot to home joint state
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}

bool TestCompetitor::CeilingRobotSetGripperState(bool enable)
{
  if (ceiling_gripper_state_.enabled == enable)
  {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = ceiling_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

void TestCompetitor::CeilingRobotWaitForAttach(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout))
    {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

bool TestCompetitor::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  bool assembled = false;
  while (!assembled)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type)
    {
    case ariac_msgs::msg::Part::BATTERY:
      assembled = assembly_station_states_[station].battery_attached;
      break;
    case ariac_msgs::msg::Part::PUMP:
      assembled = assembly_station_states_[station].pump_attached;
      break;
    case ariac_msgs::msg::Part::SENSOR:
      assembled = assembly_station_states_[station].sensor_attached;
      break;
    case ariac_msgs::msg::Part::REGULATOR:
      assembled = assembly_station_states_[station].regulator_attached;
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid part type");
      return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(5))
    {
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceiling_robot_.stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");

  return true;
}

bool TestCompetitor::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool TestCompetitor::CeilingRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

bool TestCompetitor::CeilingRobotMoveToAssemblyStation(int station)
{
  switch (station)
  {
  case 1:
    ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
    break;
  case 2:
    ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
    break;
  case 3:
    ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
    break;
  case 4:
    ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
    break;
  default:
    RCLCPP_WARN(get_logger(), "Not a valid assembly station");
    return false;
  }

  return CeilingRobotMovetoTarget();
}

bool TestCompetitor::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY)
  {
    dx = battery_grip_offset_ * cos(part_rotation);
    dy = battery_grip_offset_ * sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;
  current_pose.position.z += 0.2;

  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;
}

bool TestCompetitor::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Check that part is attached and matches part to assemble
  if (!ceiling_gripper_state_.attached)
  {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }

  if (part.part != ceiling_robot_attached_part_)
  {
    RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
    return false;
  }

  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station)
  {
  case 1:
    insert_frame_name = "as1_insert_frame";
    break;
  case 2:
    insert_frame_name = "as2_insert_frame";
    break;
  case 3:
    insert_frame_name = "as3_insert_frame";
    break;
  case 4:
    insert_frame_name = "as4_insert_frame";
    break;
  default:
    RCLCPP_WARN(get_logger(), "Not a valid assembly station");
    return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part.part.type == ariac_msgs::msg::Part::BATTERY)
  {
    tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
  }
  else
  {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }

  // Move to approach position
  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilingRobotWaitForAssemble(station, part);

  CeilingRobotSetGripperState(false);

  std::string part_name = part_colors_[ceiling_robot_attached_part_.color] +
                          "_" + part_types_[ceiling_robot_attached_part_.type];
  ceiling_robot_.detachObject(part_name);

  // Move away slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;

  if (part.part.type == ariac_msgs::msg::Part::REGULATOR)
  {
    current_pose.position.x -= 0.05;
  }
  else
  {
    current_pose.position.z += 0.1;
  }

  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  return true;
}

bool TestCompetitor::CompleteOrders()
{
  // Wait for first order to be published
  while (orders_.size() == 0)
  {
  }

  bool success;
  while (true)
  {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
    {
      success = false;
      break;
    }

    if (orders_.size() == 0)
    {
      if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
      {
        // wait for more orders
        RCLCPP_INFO(get_logger(), "Waiting for orders...");
        while (orders_.size() == 0)
        {
        }
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        break;
      }
    }

    current_order_ = orders_.front();
    orders_.erase(orders_.begin());
    int kitting_agv_num = -1;

    if (current_order_.type == ariac_msgs::msg::Order::KITTING)
    {
      TestCompetitor::CompleteKittingTask(current_order_.kitting_task);
      kitting_agv_num = current_order_.kitting_task.agv_number;
    }
    else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY)
    {
      TestCompetitor::CompleteAssemblyTask(current_order_.assembly_task);
    }
    else if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
    {
      TestCompetitor::CompleteCombinedTask(current_order_.combined_task);
      // write your own code here to set the kitting_agv_num value
    }

    // loop until the AGV is at the warehouse
    auto agv_location = -1;
    while (agv_location != ariac_msgs::msg::AGVStatus::WAREHOUSE)
    {
      if (kitting_agv_num == 1)
        agv_location = agv_locations_[1];
      else if (kitting_agv_num == 2)
        agv_location = agv_locations_[2];
      else if (kitting_agv_num == 3)
        agv_location = agv_locations_[3];
      else if (kitting_agv_num == 4)
        agv_location = agv_locations_[4];
    }

    TestCompetitor::SubmitOrder(current_order_.id);
  }
  return success;
}


bool TestCompetitor::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{
  FloorRobotSendHome();

  FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

  for (auto kit_part : task.parts)
  {
    FloorRobotPickBinPart(kit_part.part);
    FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
  }

  // Check quality
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = current_order_.id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

  if (!result.get()->all_passed)
  {
    RCLCPP_ERROR(get_logger(), "Issue with shipment");
  }

  MoveAGV(task.agv_number, task.destination);

  return true;
}

bool TestCompetitor::CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task)
{
  // Send AGVs to assembly station
  for (auto const &agv : task.agv_numbers)
  {
    int destination;
    if (task.station == ariac_msgs::msg::AssemblyTask::AS1 || task.station == ariac_msgs::msg::AssemblyTask::AS3)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
    }
    else if (task.station == ariac_msgs::msg::AssemblyTask::AS2 || task.station == ariac_msgs::msg::AssemblyTask::AS4)
    {
      destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
    }

    LockAGVTray(agv);
    MoveAGV(agv, destination);
  }

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
  auto result = pre_assembly_poses_getter_->async_send_request(request);

  result.wait();

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses;
  if (result.get()->valid_id)
  {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts)
  {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part : agv_part_poses)
    {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
      {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);

    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
  }

  return true;
}

bool TestCompetitor::CompleteCombinedTask(ariac_msgs::msg::CombinedTask task)
{
  // Decide on a tray to use
  int id;
  if (kts1_trays_.size() != 0)
  {
    id = kts1_trays_[0].id;
  }
  else if (kts2_trays_.size() != 0)
  {
    id = kts2_trays_[0].id;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "No trays available.");
    return false;
  }

  // Decide which AGV to use
  int agv_number;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2)
  {
    agv_number = 1;
  }
  else
  {
    agv_number = 4;
  }

  MoveAGV(agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING);

  FloorRobotPickandPlaceTray(id, agv_number);

  int count = 1;
  for (auto assembly_part : task.parts)
  {
    FloorRobotPickBinPart(assembly_part.part);
    FloorRobotPlacePartOnKitTray(agv_number, count);
    count++;
  }

  int destination;
  if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS3)
  {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
  }
  else
  {
    destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
  }

  MoveAGV(agv_number, destination);

  CeilingRobotMoveToAssemblyStation(task.station);

  // Get Assembly Poses
  auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
  request->order_id = current_order_.id;
  auto result = pre_assembly_poses_getter_->async_send_request(request);

  result.wait();

  std::vector<ariac_msgs::msg::PartPose> agv_part_poses;
  if (result.get()->valid_id)
  {
    agv_part_poses = result.get()->parts;

    if (agv_part_poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No part poses recieved");
      return false;
    }
  }
  else
  {
    RCLCPP_WARN(get_logger(), "Not a valid order ID");
    return false;
  }

  for (auto const &part_to_assemble : task.parts)
  {
    // Check if matching part exists in agv_parts
    bool part_exists = false;
    ariac_msgs::msg::PartPose part_to_pick;
    part_to_pick.part = part_to_assemble.part;
    for (auto const &agv_part : agv_part_poses)
    {
      if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
      {
        part_exists = true;
        part_to_pick.pose = agv_part.pose;
        break;
      }
    }

    if (!part_exists)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
      continue;
    }

    // Pick up part
    CeilingRobotPickAGVPart(part_to_pick);

    CeilingRobotMoveToAssemblyStation(task.station);

    // Assemble Part to insert
    CeilingRobotAssemblePart(task.station, part_to_assemble);

    CeilingRobotMoveToAssemblyStation(task.station);
  }

  return true;
}

bool TestCompetitor::StartCompetition()
{
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
  {
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

bool TestCompetitor::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
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

  auto result = client->async_send_request(request);
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

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}