#ifndef ARIAC_PLUGINS__TOOL_STAND_PLUGIN_HPP
#define ARIAC_PLUGINS__TOOL_STAND_PLUGIN_HPP

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

// ROS
#include <ariac_interfaces/msg/vacuum_tools.hpp>

using VacuumTools = ariac_interfaces::msg::VacuumTools;

namespace ariac_plugins{
  enum class ToolStandLockState {
    LOCKED,
    UNLOCKED,
    LOCK_REQUESTED,
    UNLOCK_REQUESTED
  };

  class ToolStandPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      void Configure (const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
    
    private:
      bool lock_cb(const gz::msgs::Entity &, gz::msgs::Boolean &);
      bool unlock_cb(const gz::msgs::Entity &, gz::msgs::Boolean &);

      std::shared_ptr<gz::transport::Node> gz_node;

      gz::sim::Model tool_stand_model;
      gz::sim::Entity tool_stand_link;
      
      std::map<int, ToolStandLockState> lock_states = {
        {VacuumTools::VG_2, ToolStandLockState::UNLOCKED},
        {VacuumTools::VG_4, ToolStandLockState::UNLOCKED} 
      };

      std::map<int, gz::sim::Entity> detachable_joints = {
        {VacuumTools::VG_2, gz::sim::kNullEntity},
        {VacuumTools::VG_4, gz::sim::kNullEntity} 
      };

      std::map<int, gz::sim::Entity> tool_link_entity = {
        {VacuumTools::VG_2, gz::sim::kNullEntity},
        {VacuumTools::VG_4, gz::sim::kNullEntity} 
      };
  };
}

#endif // ARIAC_PLUGINS__TOOL_STAND_PLUGIN_HPP