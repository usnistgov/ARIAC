#ifndef ARIAC_PLUGINS__ENTITY_DISPOSAL_PLUGIN_HPP
#define ARIAC_PLUGINS__ENTITY_DISPOSAL_PLUGIN_HPP

// Gazebo 
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// Msg
#include <ariac_components/penalty.hpp>
#include <ariac_components/cell.hpp>

// General
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <random>
#include <iostream>

namespace ariac_plugins{

  enum class ModelType {
    InspectionBin,
    InspectionConveyorBin,
    RecyclingBin,
    Other
  };

  struct ModelContact {
    bool has_contact;
    std::string name;
  };

  struct DetachJointInfo {
    gz::sim::Entity joint;
    gz::sim::Entity parent;
    gz::sim::Entity child;
  };

  class EntityDisposalPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~EntityDisposalPlugin();

      void Configure (const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override;
    
    private:      
      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;
      
      // GZ
      std::shared_ptr<gz::transport::Node> gz_node;
      std::string gz_contact_topic;
      
      // SDF Tags
      std::string object_name;
      std::string contact_link;
      
      // Variables
      ModelContact model_in_contact = {false, ""};

      ModelType model_type = ModelType::Other;
      
      std::map<std::string, ModelType> name_to_model_type = {
        {"inspection_bin", ModelType::InspectionBin},
        {"inspection_conveyor_bin", ModelType::InspectionConveyorBin},
        {"recycling_bin", ModelType::RecyclingBin}
      };

      // Functions
      void contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
      std::optional<ariac_components::Penalty> handle_penalty(std::optional<ariac_components::Cell>, double time);
  };
}

#endif // ARIAC_PLUGINS__ENTITY_DISPOSAL_PLUGIN_HPP