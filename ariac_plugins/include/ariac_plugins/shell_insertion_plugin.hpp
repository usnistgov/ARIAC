#ifndef ARIAC_PLUGINS__SHELL_INSERTION_PLUGIN_HPP
#define ARIAC_PLUGINS__SHELL_INSERTION_PLUGIN_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>
#include <random>

// Gazebo 
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <ariac_components/trial.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Msgs
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>

// Srvs
#include <ariac_interfaces/srv/trigger.hpp>

using Trigger = ariac_interfaces::srv::Trigger;
using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;

using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;

namespace ariac_plugins{

enum class ShellTypes {
  TOP,
  BOTTOM
};

class ShellInsertionPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:
    ~ShellInsertionPlugin();

    void Configure (const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
  
  private:
    bool shell_in_contact(const gz::msgs::Contacts &);
    std::optional<std::string> generate_shell_sdf(ShellTypes shell);
    void spawn_shell(ShellTypes shell);

    // ROS callbacks
    void competition_status_cb(const CompetitionStatus::SharedPtr);
    void insert_bottom_shell_cb(const TriggerReqPtr, TriggerResPtr);
    void insert_top_shell_cb(const TriggerReqPtr, TriggerResPtr);
  
    // GZ callbacks
    void table_contact_msg_cb(const gz::msgs::Contacts &);
    void conveyor_contact_msg_cb(const gz::msgs::Contacts &);

    // ROS
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;

    rclcpp::Service<Trigger>::SharedPtr insert_bottom_shell_srv;
    rclcpp::Service<Trigger>::SharedPtr insert_top_shell_srv;
    
    rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;
    

    // GZ
    std::shared_ptr<gz::transport::Node> gz_node;
    gz::sim::Entity world_entity = 1;

    std::map<ShellTypes, gz::math::Vector3d> shell_positions = {
      {ShellTypes::BOTTOM, gz::math::Vector3d(4.6, 5.0, 0.43)},
      {ShellTypes::TOP, gz::math::Vector3d(3.85, 5.2, 0.455)},
    };

    std::map<ShellTypes, double> last_contact_time = {
      {ShellTypes::BOTTOM, 0.0},
      {ShellTypes::TOP, 0.0},
    };

    std::map<ShellTypes, bool> shell_present = {
      {ShellTypes::BOTTOM, false},
      {ShellTypes::TOP, false},
    };

    std::map<ShellTypes, int> shell_counts = {
      {ShellTypes::BOTTOM, 0},
      {ShellTypes::TOP, 0},
    };

    std::map<ShellTypes, std::string> model_paths = {
      {ShellTypes::BOTTOM, ""},
      {ShellTypes::TOP, ""},
    };

    std::map<ShellTypes, std::string> shell_names = {
      {ShellTypes::BOTTOM, "bottom_shell"},
      {ShellTypes::TOP, "top_shell"},
    };

    bool advertise_services = true;
    bool reset_services = false;

    int competition_state;

    std::mt19937 rng;
    std::uniform_real_distribution<double> x_offset_distrubution = std::uniform_real_distribution<double>(-0.05, 0.05);
    std::uniform_real_distribution<double> y_offset_distrubution = std::uniform_real_distribution<double>(-0.1, 0.1);
    std::uniform_real_distribution<double> angle_distribution = std::uniform_real_distribution<double>(-M_PI, M_PI);
  };
}

#endif // ARIAC_PLUGINS__SHELL_INSERTION_PLUGIN_HPP