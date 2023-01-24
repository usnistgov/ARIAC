#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>

#include <geometry_msgs/msg/pose.hpp>

class TestCompetitor : public rclcpp::Node
{
public:
  /// Constructor
  TestCompetitor();

  ~TestCompetitor();

  // Floor Robot Public Functions
  void FloorRobotSendHome();
  bool FloorRobotSetGripperState(bool enable);
  bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
  bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);
  bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick);
  bool FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick);
  bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);

  // Ceiling Robot Public Functions
  void CeilingRobotSendHome();

  // ARIAC Functions
  bool StartCompetition();
  bool EndCompetition();
  bool LockAGVTray(int agv_num);
  bool MoveAGV(int agv_num, int destination);
  bool SubmitOrder(std::string order_id);

  bool CompleteOrders();
  bool CompleteKittingTask(ariac_msgs::msg::KittingTask task);
  bool CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task);
  bool CompleteCombinedTask(ariac_msgs::msg::CombinedTask task);

private:
  // Robot Move Functions
  bool FloorRobotMovetoTarget();
  bool CeilingRobotMovetoTarget();
  bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
  geometry_msgs::msg::Quaternion FloorRobotSetOrientation(double rotation);
  void FloorRobotWaitForAttach(double timeout);

  // Helper Functions
  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
  geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
  geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
  double GetYaw(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

  void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
  void AddModelsToPlanningScene();

  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface floor_robot_;
  moveit::planning_interface::MoveGroupInterface ceiling_robot_;
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Subscriptions
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

  // Orders List
  ariac_msgs::msg::Order current_order_;
  std::vector<ariac_msgs::msg::Order> orders_;

  unsigned int competition_state_;

  // Gripper State
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
  ariac_msgs::msg::Part floor_robot_attached_part_;

  // Sensor poses
  geometry_msgs::msg::Pose kts1_camera_pose_;
  geometry_msgs::msg::Pose kts2_camera_pose_;
  geometry_msgs::msg::Pose left_bins_camera_pose_;
  geometry_msgs::msg::Pose right_bins_camera_pose_;

  // Trays
  std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
  std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

  // Bins
  std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
  std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

  // Sensor Callbacks
  bool kts1_camera_recieved_data = false;
  bool kts2_camera_recieved_data = false;
  bool left_bins_camera_recieved_data = false;
  bool right_bins_camera_recieved_data = false;

  void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

  // Gripper State Callback
  void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

  // Orders Callback
  void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);

  // Competition state callback
  void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

  // ARIAC Services
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;

  // Constants
  double kit_tray_thickness_ = 0.01;
  double drop_height_ = 0.002;
  double pick_offset_ = 0.003;

  std::map<int, std::string> part_types_ = {
    {ariac_msgs::msg::Part::BATTERY, "battery"},
    {ariac_msgs::msg::Part::PUMP, "pump"},
    {ariac_msgs::msg::Part::REGULATOR, "regulator"},
    {ariac_msgs::msg::Part::SENSOR, "sensor"}
  };

  std::map<int, std::string> part_colors_ = {
    {ariac_msgs::msg::Part::RED, "red"},
    {ariac_msgs::msg::Part::BLUE, "blue"},
    {ariac_msgs::msg::Part::GREEN, "green"},
    {ariac_msgs::msg::Part::ORANGE, "orange"},
    {ariac_msgs::msg::Part::PURPLE, "purple"},
  };

  // Part heights
  std::map<int, double> part_heights_ = {
    {ariac_msgs::msg::Part::BATTERY, 0.04},
    {ariac_msgs::msg::Part::PUMP, 0.12},
    {ariac_msgs::msg::Part::REGULATOR, 0.07},
    {ariac_msgs::msg::Part::SENSOR, 0.07}
  };

  // Quadrant Offsets
  std::map<int, std::pair<double, double>> quad_offsets_ = {
    {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
  };

  std::map<std::string, double> rail_positions_ = {
    {"agv1", -4.5},
    {"agv2", -1.2},
    {"agv3", 1.2},
    {"agv4", 4.5},
    {"left_bins", 3}, 
    {"right_bins", -3}
  };

  // Joint value targets for kitting stations
  std::map<std::string, double> floor_kts1_js_ = {
    {"linear_actuator_joint", 4.0},
    {"floor_shoulder_pan_joint", 1.57},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> floor_kts2_js_ = {
    {"linear_actuator_joint", -4.0},
    {"floor_shoulder_pan_joint", -1.57},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };
};