#ifndef ARIAC_PLUGINS__TOOL_CHANGER_PLUGIN_HPP_
#define ARIAC_PLUGINS__TOOL_CHANGER_PLUGIN_HPP_

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
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/math.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/math/Quaternion.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ariac_interfaces/msg/tool_changer_status.hpp>
#include <ariac_interfaces/msg/vacuum_tools.hpp>
#include <ariac_interfaces/srv/attach_tool.hpp>
#include <ariac_interfaces/srv/trigger.hpp>

using ToolChangerStatus = ariac_interfaces::msg::ToolChangerStatus;
using VacuumTools = ariac_interfaces::msg::VacuumTools;
using AttachToolSrv = ariac_interfaces::srv::AttachTool;
using Trigger = ariac_interfaces::srv::Trigger;

using AttachToolReqPtr = AttachToolSrv::Request::SharedPtr;
using AttachToolResPtr = AttachToolSrv::Response::SharedPtr;

using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;

namespace ariac_plugins
{
enum class ToolLockState {
  LOCKED,
  UNLOCKED,
  LOCK_REQUESTED,
  UNLOCK_REQUESTED
};

class ToolChangerPlugin: 
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate
{
  public:
    ~ToolChangerPlugin();
    
    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;
    
    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
    void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;
    
  private:
    bool lock_tool_to_stand(int);
    bool unlock_tool_from_stand(int);
    
    // ROS Callbacks
    void attach_tool_srv_cb(const AttachToolReqPtr request, AttachToolResPtr response);
    void detach_tool_srv_cb(const TriggerReqPtr request, TriggerResPtr response);
    void publish_status_cb();

    // GZ Callbacks
    void tool_changer_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
    
    // ROS
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;

    // Publishers
    rclcpp::Publisher<ToolChangerStatus>::SharedPtr status_pub;

    // Timers
    rclcpp::TimerBase::SharedPtr pub_timer;
    
    // Services
    rclcpp::Service<AttachToolSrv>::SharedPtr attach_tool_srv;
    rclcpp::Service<Trigger>::SharedPtr detach_tool_srv;
    
    ToolChangerStatus current_status;

    // GZ
    std::shared_ptr<gz::transport::Node> gz_node;
    
    gz::sim::Entity lock_joint;
    gz::sim::Entity tool_changer_link;

    std::map<int, gz::sim::Entity> tool_link_entities = {
      {VacuumTools::VG_2, gz::sim::kNullEntity},
      {VacuumTools::VG_4, gz::sim::kNullEntity}
    };

    std::map<int, bool> tool_in_contact = {
      {VacuumTools::VG_2, false},
      {VacuumTools::VG_4, false}
    };

    std::map<int, gz::sim::Joint> contact_link_joints = {
      {VacuumTools::VG_2, gz::sim::Joint()},
      {VacuumTools::VG_4, gz::sim::Joint()}
    };

    std::map<int, gz::sim::Link> tool_links;

    std::map<int, gz::math::Pose3d> initial_poses;
    std::map<int, gz::math::Pose3d> current_poses;
    
    ToolLockState lock_state = ToolLockState::UNLOCKED;
};
}

#endif