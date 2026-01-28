#ifndef ARIAC_PLUGINS__VACUUM_TOOL_PLUGIN_HPP_
#define ARIAC_PLUGINS__VACUUM_TOOL_PLUGIN_HPP_

#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <ariac_components/trial.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ariac_interfaces/msg/vacuum_tools.hpp>
#include <ariac_interfaces/srv/trigger.hpp>

using VacuumTools = ariac_interfaces::msg::VacuumTools;
using Trigger = ariac_interfaces::srv::Trigger;
using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;

namespace ariac_plugins{

  struct PadContact {
    bool in_contact;
    std::string model_name;
  };

  enum class VacuumToolLockState {
    LOCKED,
    UNLOCKED,
    LOCK_REQUESTED,
    UNLOCK_REQUESTED
  };

  class VacuumToolPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
  {
    public:

    ~VacuumToolPlugin();

    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;
    
    void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) final;
    
    private:
    // ROS callbacks
    void vg_2_attach_cb(const TriggerReqPtr request, TriggerResPtr response);
    void vg_4_attach_cb(const TriggerReqPtr request, TriggerResPtr response);
    void detach_object_cb(const TriggerReqPtr request, TriggerResPtr response);

    // GZ callbacks
    void contact_sensor_1_cb(const gz::msgs::StringMsg_V &);
    void contact_sensor_2_cb(const gz::msgs::StringMsg_V &);
    void contact_sensor_3_cb(const gz::msgs::StringMsg_V &);
    void contact_sensor_4_cb(const gz::msgs::StringMsg_V &);

    bool wait_for_state(VacuumToolLockState);
    bool should_malfunction();
    void clear_malfunction();

    // Functions
    std::optional<std::string> shell_in_contact(const gz::msgs::StringMsg_V &);
    bool lock_tool_to_stand();
      
    // GZ
    std::shared_ptr<gz::transport::Node> gz_node;
    gz::sim::Entity lock_joint;
    gz::sim::Entity gripper_base_link;
    gz::sim::Entity world_entity = 1;
    std::vector<gz::sim::Joint> suction_cup_joints = {};
    
    // ROS
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;
    rclcpp::Service<Trigger>::SharedPtr attach_srv;
    rclcpp::Service<Trigger>::SharedPtr detach_srv;

    // Variables
    int tool_type = VacuumTools::NONE;
    int grasp_occurrence = 1;

    bool malfunction_active = false;

    std::string tool_holder_contact_topic;
    std::string attach_shell_name = "";

    std::map<int, PadContact> pad_contacts;

    std::vector<std::pair<int, bool>> malfunctions; 

    VacuumToolLockState lock_state = VacuumToolLockState::UNLOCKED;

  };
} // namespace ariac_plugins

#endif // ARIAC_PLUGINS__VACUUM_TOOL_PLUGIN_HPP_