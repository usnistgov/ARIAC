#ifndef ARIAC_PLUGINS__LINEAR_CONVEYOR_PLUGIN_HPP_
#define ARIAC_PLUGINS__LINEAR_CONVEYOR_PLUGIN_HPP_

// GZ
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>

// GZ msgs
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/Utility.hh>

#include <ariac_components/trial.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// MSGS
#include <ariac_interfaces/msg/conveyor_status.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>
#include <ariac_interfaces/msg/operation_states.hpp>

// SRVS
#include <ariac_interfaces/srv/conveyor_control.hpp>
#include <ariac_interfaces/srv/bidirectional_conveyor_control.hpp>

// General
#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

using ConveyorControl = ariac_interfaces::srv::ConveyorControl;
using ConveyorControlRequestPtr = ConveyorControl::Request::SharedPtr;
using ConveyorControlResposePtr = ConveyorControl::Response::SharedPtr;

using BiConveyorControl = ariac_interfaces::srv::BidirectionalConveyorControl;
using BiConveyorControlRequestPtr = BiConveyorControl::Request::SharedPtr;
using BiConveyorControlResposePtr = BiConveyorControl::Response::SharedPtr;

using ConveyorStatus = ariac_interfaces::msg::ConveyorStatus;
using OperatingStates = ariac_interfaces::msg::OperationStates;

using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;

namespace ariac_plugins
{
  class LinearConveyorPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: ~LinearConveyorPlugin() override;

    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_manager) override;

    void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

    private: 
      // Callbacks
      void publish_state_cb() const;
      void control_cb(const ConveyorControlRequestPtr request, ConveyorControlResposePtr response);
      void bi_control_cb(const BiConveyorControlRequestPtr request, BiConveyorControlResposePtr response);
      void competition_status_cb(const CompetitionStatus::SharedPtr);

      // GZ
      gz::sim::Model model;
      gz::sim::Joint belt_joint;
      gz::sim::Entity world_entity = 1;

      // SDF Params
      bool controllable;
      bool bidirectional;
      double initial_speed;
      double max_speed;
      double travel;

      // Variables
      bool configured = false;
      int direction = ConveyorStatus::FORWARD;
      int operating_status = OperatingStates::OPERATIONAL;
      int competition_state = CompetitionStates::PREPARING;

      double position = 0.0;
      double speed = 0.0;

      std::vector<std::pair<ariac_components::ConveyorMalfunction, bool>> malfunctions; 

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
      std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
      std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle;

      std::optional<rclcpp::Time> competiton_start_time = std::nullopt;
      std::optional<rclcpp::Time> malfunction_end_time = std::nullopt;

      rclcpp::Publisher<ConveyorStatus>::SharedPtr state_publisher;
      rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;
      rclcpp::Service<ConveyorControl>::SharedPtr control_srv;
      rclcpp::Service<BiConveyorControl>::SharedPtr bi_control_srv;
      rclcpp::TimerBase::SharedPtr publish_timer;

      ConveyorStatus current_status;
  };
}

#endif