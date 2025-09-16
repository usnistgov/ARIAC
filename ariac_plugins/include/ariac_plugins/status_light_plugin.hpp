#ifndef ARIAC_PLUGINS__STATUS_LIGHT_PLUGIN_HPP
#define ARIAC_PLUGINS__STATUS_LIGHT_PLUGIN_HPP

// Gazebo 
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>
#include <ariac_interfaces/srv/trigger.hpp>

using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;
using Trigger = ariac_interfaces::srv::Trigger;

using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;

namespace ariac_plugins{
  class StatusLightPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~StatusLightPlugin();

      void Configure (const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
    
    private:
      void remove_light(gz::sim::EntityComponentManager &);
      void change_visual_status(int, bool);
      void update_lights();

      void competition_status_cb(const CompetitionStatus::SharedPtr);
      
      std::shared_ptr<gz::transport::Node> gz_node;

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
  
      // Subscribers
      rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;
    
      CompetitionStatus current_status;

      bool light_on = false;

      int lit_light = -1;

      std::map<int, gz::math::Color> status_colors{
        {CompetitionStates::PREPARING, gz::math::Color::Yellow},
        {CompetitionStates::READY, gz::math::Color::Blue},
        {CompetitionStates::STARTED, gz::math::Color::Green},
        {CompetitionStates::ORDERS_COMPLETE, gz::math::Color::Magenta},
        {CompetitionStates::ENDED, gz::math::Color::Red}
        // {CompetitionStates::PREPARING, gz::math::Color(0.8, 0.8, 0, 1)}
      };

      std::map<int, gz::sim::Entity> light_visuals;

      gz::sim::Link light_link;
  };
}

#endif // ARIAC_PLUGINS__STATUS_LIGHT_PLUGIN_HPP