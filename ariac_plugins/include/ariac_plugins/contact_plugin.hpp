#ifndef ARIAC_PLUGINS__CONTACT_PLUGIN_HPP_
#define ARIAC_PLUGINS__CONTACT_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

namespace ariac_plugins
{
  class ContactPlugin:
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
    gz::sim::Model model;
    gz::sim::Link link_obj;
    gz::sim::Sensor sensor;
    std::shared_ptr<gz::transport::Node> gz_node;
    gz::transport::Node::Publisher pub;
    gz::msgs::StringMsg_V current_contact_msg;

    // General
    std::string topic_to_publish_to;
    std::string search_term;
    double last_set_time = -INFINITY;
    int update_rate = 30;
    int iteration_update_mod = 100/30;

    // GZ Callbacks
    void contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
  };
}

#endif // ARIAC_PLUGINS__CONTACT_PLUGIN_HPP_