#ifndef ARIAC_PLUGINS__AGV_MOTION_PLUGIN_HPP_
#define ARIAC_PLUGINS__AGV_MOTION_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/create_server.hpp"

#include "angles/angles.h"

#include <geometry_msgs/msg/pose.hpp>

#include <path_velocity_planner/velocity_planner.hpp>

#include <ariac_components/penalty.hpp>

#include <ariac_interfaces/msg/agv_stations.hpp>
#include <ariac_interfaces/msg/agv_status.hpp>
#include <ariac_interfaces/action/move_agv.hpp>

#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

using AGVStations = ariac_interfaces::msg::AgvStations;

using AGVStatus = ariac_interfaces::msg::AgvStatus;

using MoveAGVAction = ariac_interfaces::action::MoveAgv;
using ActionServer = rclcpp_action::Server<MoveAGVAction>;
using ActionServerPtr = ActionServer::SharedPtr;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveAGVAction>;
using GoalHandlePtr = std::shared_ptr<GoalHandle>;

namespace ariac_plugins
{
  enum class AGVMotionStatus {
    IDLE,
    HOLD_POSITION,
    MOVING,
    TELEPORT,
    COMPLETE_GOAL
  };

  enum class AGVPath {
    INSPECTION_TO_ASSEMBLY,
    ASSEMBLY_TO_INSPECTION,
    INSPECTION_TO_SHIPPING,
    SHIPPING_TO_INSPECTION,
    INSPECTION_TO_RECYCLING,
    RECYCLING_TO_INSPECTION,
    SHIPPING_TO_RECYCLING
  };

  class AgvMotionPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public:
        AgvMotionPlugin();
        ~AgvMotionPlugin() override;
      
      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_manager) override;
      
      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

    private: 
      // GZ Callbacks
      void contact_msg_cb(const gz::msgs::Contacts &_msg);

      // ROS Callbacks
      rclcpp_action::GoalResponse goal_recieved_cb(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveAGVAction::Goal> goal);
      rclcpp_action::CancelResponse goal_cancel_cb(const GoalHandlePtr goal_handle);
      void goal_accepted_cb(GoalHandlePtr goal_handle);
      void pub_timer_cb();

      // Functions
      std::vector<path_velocity_planner::Point> get_waypoints(AGVPath path);
      geometry_msgs::msg::Pose gz_to_ros_pose(const gz::math::Pose3d &gz_pose);
      std::pair<gz::math::Vector3d, gz::math::Vector3d> compute_hold_position_velocity(
        int station_id, const gz::math::Pose3d &current_pose);

      // GZ 
      gz::sim::Model agv_model;
      gz::sim::Link agv_base_link;
      gz::sim::Entity agv_base_link_entity = gz::sim::kNullEntity;
      gz::sim::Entity lock_joint = gz::sim::kNullEntity;
      std::shared_ptr<gz::transport::Node> gz_node;

      // ROS 
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
      ActionServerPtr action_server;
      std::optional<GoalHandlePtr> current_goal_handle = std::nullopt;
      rclcpp::Publisher<AGVStatus>::SharedPtr agv_info_pub;
      rclcpp::TimerBase::SharedPtr pub_timer;
      AGVStatus status_msg;

      // Parameters
      const double v_max = 0.8;
      const double acc = 1;
      const double goal_distance_threshold = 0.001;
      const double goal_angle_threshold = (M_PI / 180) * 2; // 2 degrees
      const double timeout = 15;
      const int feedback_rate = 10;
      const std::string link_name = "base_link";
      const std::string floor_model_name = "floor";
      const std::string floor_link_name = "floor";

      // Motion control parameters
      const double hold_position_velocity = 0.005;
      const double z_threshold = 0.001;
      const double z_lift_velocity = 0.015;
      const int teleport_wait_iterations = 5;
      const int complete_goal_wait_iterations = 150;
      const int feedback_publish_interval = 1000;

      std::optional<double> motion_start_time = std::nullopt;

      // Class variables
      std::optional<int> wait_until_iteration;
      bool collision_occurred = false;
      std::optional<path_velocity_planner::Point> start_location = std::nullopt;

      std::map<int, path_velocity_planner::Point> goal_locations = {
        {AGVStations::ASSEMBLY, { 5, 4.5 }},
        {AGVStations::SHIPPING, { 6.5, 2.55 }},
        {AGVStations::RECYCLING, { 1.0, 6.3}}
      };
      std::map<int, double> station_yaw = {
        {AGVStations::INSPECTION, M_PI_2},
        {AGVStations::ASSEMBLY, 0},
        {AGVStations::SHIPPING, -M_PI_2},
        {AGVStations::RECYCLING, M_PI_2}
      };

      gz::math::Vector3d linear_velocity_vector; 
      gz::math::Vector3d angular_velocity_vector; 
      
      path_velocity_planner::VelocityPlanner velocity_planner;
      path_velocity_planner::Direction direction;

      AGVMotionStatus motion_state = AGVMotionStatus::IDLE;

      std::vector<path_velocity_planner::Point> current_waypoints;
  };
}

#endif // ARIAC_PLUGINS__AGV_MOTION_PLUGIN_HPP