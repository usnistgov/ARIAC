#ifndef ARIAC_PLUGINS__VOLTAGE_TESTER_PLUGIN_HPP
#define ARIAC_PLUGINS__VOLTAGE_TESTER_PLUGIN_HPP

// Gazebo 
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>

#include <ariac_components/trial.hpp>
#include <ariac_components/cell.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Msg
#include <ariac_interfaces/msg/voltage_reading.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>
#include <ariac_interfaces/msg/operation_states.hpp>

// General
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <random>

using VoltageReading = ariac_interfaces::msg::VoltageReading;
using OperationStates = ariac_interfaces::msg::OperationStates;
using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;

namespace ariac_plugins{
  class VoltageTesterPlugin: 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate
  {
    public:
      ~VoltageTesterPlugin();

      void Configure (const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

    private:
      // ROS callbacks
      void publish_voltage_cb() const;
      void competition_status_cb(const CompetitionStatus::SharedPtr);    

      // GZ callbacks
      void contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::Context::SharedPtr node_context;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;

      std::optional<rclcpp::Time> competiton_start_time = std::nullopt;
      std::optional<rclcpp::Time> malfunction_end_time = std::nullopt;

      rclcpp::Publisher<VoltageReading>::SharedPtr voltage_tester_publisher;
      rclcpp::TimerBase::SharedPtr publish_timer;
      rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;

      // GZ
      std::shared_ptr<gz::transport::Node> gz_node;
      gz::sim::Entity world_entity = 1;
      std::string gz_contact_topic;

      // Other
      bool cell_present = false;

      int competition_state = CompetitionStates::PREPARING;
      int tester_number;
    
      double update_rate;
      double last_contact_time = 0.0;
      double last_publish_time = 0.0;

      std::string voltage_tester_name;
      std::string cell_name;

      std::optional<double> cell_voltage = std::nullopt;

      std::mt19937 rng;
      std::normal_distribution<double> noise_distribution;

      std::vector<std::pair<ariac_components::VoltageTesterMalfunction, bool>> malfunctions; 
          
      VoltageReading voltage_reading;
      int operation_state;
  };
}

#endif // ARIAC_PLUGINS__VOLTAGE_TESTER_PLUGIN_HPP