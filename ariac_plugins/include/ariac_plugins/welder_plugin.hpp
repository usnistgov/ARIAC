#ifndef ARIAC_PLUGINS__WELDER_PLUGIN_HPP
#define ARIAC_PLUGINS__WELDER_PLUGIN_HPP

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
#include <gz/sim/components/Component.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/msgs/material.pb.h>
#include <gz/math/Rand.hh>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <random>
#include <ariac_components/module.hpp>
#include <ariac_interfaces/srv/trigger.hpp>

using gz::sim::components::Module;
using Trigger = ariac_interfaces::srv::Trigger;
using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;

namespace ariac_plugins{
  struct Contact{
    bool in_contact;
    std::string model;
    int bead_id;

    bool operator==(const Contact& other) const {
        return (model == other.model && bead_id == other.bead_id);
    }
  };

  struct BeadContact{
    bool in_contact = false;
    std::string shell_model_name = "";
    int bead_id = -1;
    double last_contact_time = 0.0;
  };

  class WelderPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~WelderPlugin();
      
      void Configure (const gz::sim::Entity &,
        const std::shared_ptr<const sdf::Element> &,
        gz::sim::EntityComponentManager &,
        gz::sim::EventManager &) override;

      void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) final;
    
    private:
      void contact_sensor_1_cb(const gz::msgs::Contacts &);
      void contact_sensor_2_cb(const gz::msgs::Contacts &);
      void check_object_in_contact(const gz::msgs::Contacts &, int);
      bool change_visual_status(const gz::sim::Entity, bool);
      void weld_cb(const TriggerReqPtr, TriggerResPtr);
      void create_welder_light(gz::sim::EntityComponentManager &, const gz::math::Pose3d &);
      void remove_welder_light(gz::sim::EntityComponentManager &);
      BeadContact check_bead_in_contact(const gz::msgs::Contacts &);
      ariac_components::Module get_module_component(const gz::sim::EntityComponentManager &, const std::string);

      std::shared_ptr<gz::transport::Node> gz_node;
      gz::transport::Node::Publisher pub;
      
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::Context::SharedPtr node_context;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
      rclcpp::Service<Trigger>::SharedPtr weld_srv;

      gz::sim::Model model;
      gz::sim::Link electrodes_link;

      std::map<int, BeadContact> electrode_in_contact;

      std::map<std::string, int> bead_ids = {
        {"bead_1_link", 1},
        {"bead_2_link", 2},
        {"bead_3_link", 3},
        {"bead_4_link", 4}
      };

      std::map<std::string, std::map<int, bool>> welded;

      BeadContact bead_to_weld;

      bool weld_requested = false;

      int light_on_step = 0;
      bool light_on = false;

      bool light_has_visual = false;
  };
}

#endif // ARIAC_PLUGINS__TOOL_STAND_PLUGIN_HPP