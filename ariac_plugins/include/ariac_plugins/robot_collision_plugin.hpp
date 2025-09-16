#ifndef ARIAC_PLUGINS__ROBOT_COLLISION_PLUGIN_HPP_
#define ARIAC_PLUGINS__ROBOT_COLLISION_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/create_server.hpp"

#include <ariac_components/penalty.hpp>

#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace ariac_plugins {

struct ContactInfo {
  bool in_contact = false;
  std::string model_name;
  double last_contact_time = 0.0;
  double last_penalty_time = 0.0;
};

class RobotCollisionPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:         
    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_manager) override;
    
    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

  private: 
    // GZ 
    std::shared_ptr<gz::transport::Node> gz_node;
    gz::sim::Model model;
    gz::sim::Entity robot_link = gz::sim::kNullEntity;

    // Variables
    ContactInfo contact_info;
    std::string robot_name;

    int seconds_to_reissue_penalty = 5;

    // GZ CBs
    void contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg); 

};
} 

#endif // ARIAC_PLUGINS__ROBOT_COLLISION_PLUGIN_HPP_