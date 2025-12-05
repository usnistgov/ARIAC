#ifndef ARIAC_PLUGINS__AGV_TRAY_PLUGIN_HPP_
#define ARIAC_PLUGINS__AGV_TRAY_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/create_server.hpp"

#include <ariac_components/cell.hpp>

#include <ariac_interfaces/msg/agv_stations.hpp>
#include <ariac_interfaces/msg/agv_status.hpp>
#include <ariac_interfaces/msg/agv_tray_status.hpp>
#include <ariac_interfaces/msg/cell_types.hpp>
#include <ariac_interfaces/srv/check_kit_quality.hpp>
#include <ariac_interfaces/srv/trigger.hpp>

#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

using AGVStations = ariac_interfaces::msg::AgvStations;
using AGVStatus = ariac_interfaces::msg::AgvStatus;
using CellTypes = ariac_interfaces::msg::CellTypes;

namespace ariac_plugins
{
  enum class AGVTrayLockState {
    LOCKED,
    UNLOCKED,
    LOCK_REQUESTED,
    UNLOCK_REQUESTED,
    REMOVAL_REQUESTED
    };

  enum class CenterSlotState {
    IDLE,
    TELEPORT_REQUESTED
  };

  struct ContactInfo {
    bool in_contact;
    std::string model_name;
    double last_contact_time;
  };

  struct SlotContactInfo {
    ContactInfo left;
    ContactInfo right;
  };

  class AgvTrayPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public:   
      ~AgvTrayPlugin() override;
      
      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_manager) override;
      
      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

    private: 

    // GZ 
    gz::sim::Model model;
    std::shared_ptr<gz::transport::Node> gz_node;

    // ROS
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;
    rclcpp::Publisher<ariac_interfaces::msg::AgvTrayStatus>::SharedPtr agv_slot_info_pub;
    rclcpp::TimerBase::SharedPtr pub_timer;
    rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr location_subscription;
    rclcpp::Service<ariac_interfaces::srv::CheckKitQuality>::SharedPtr check_kit_quality_srv;
    rclcpp::Service<ariac_interfaces::srv::Trigger>::SharedPtr recycle_cells_srv;

    // SDF Tags
    gz::sim::Entity tray_link = gz::sim::kNullEntity;

    // Variables
    std::string agv_name;
    ariac_interfaces::msg::AgvTrayStatus tray_status;
    int recycle_request_iteration;
    AGVTrayLockState lock_state = AGVTrayLockState::LOCK_REQUESTED;
    bool recycle_requested = false;
    int agv_station = AGVStations::INSPECTION;

    // Center teleporter
    std::vector<std::string> teleported_cells;
    CenterSlotState center_slot_state = CenterSlotState::IDLE;
    std::string cell_to_teleport;

    // Mappings
    std::map<std::string, std::map<int, std::string>> topic_names;
    std::map<int, gz::sim::Entity> lock_joints = {
        { 1, gz::sim::kNullEntity },
        { 2, gz::sim::kNullEntity },
        { 3, gz::sim::kNullEntity },
        { 4, gz::sim::kNullEntity },
    };

    std::map<int, bool> slot_locked = {
        { 1, false },
        { 2, false },
        { 3, false },
        { 4, false },
    };

    std::map<int, SlotContactInfo> cell_in_slot = {
      { 1, {{ false, "", 0.0 }, { false, "", 0.0 }}},
      { 2, {{ false, "", 0.0 }, { false, "", 0.0 }}},
      { 3, {{ false, "", 0.0 }, { false, "", 0.0 }}},
      { 4, {{ false, "", 0.0 }, { false, "", 0.0 }}},
    };

    std::map<int, std::optional<ariac_components::Cell>> cell_components = {
      { 1, {}},
      { 2, {}},
      { 3, {}},
      { 4, {}}
    };

    std::map<int, double> nominal_voltages = {
      {CellTypes::LI_ION, CellTypes::LI_ION_NOMINAL_VOLTAGE},
      {CellTypes::NIMH, CellTypes::NIMH_NOMINAL_VOLTAGE},
    };

    // Functions
    std::optional<std::string> get_cell_in_contact(const gz::msgs::Contacts &_gz_contacts_msg);
    void agv_station_check(ariac_interfaces::msg::AgvStatus::SharedPtr msg);
    void check_kit_quality_cb(const ariac_interfaces::srv::CheckKitQuality::Request::SharedPtr, ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr);
    void recycle_cells_cb(const ariac_interfaces::srv::Trigger::Request::SharedPtr, ariac_interfaces::srv::Trigger::Response::SharedPtr);
    void pub_timer_cb();

    // GZ CBs
    void contact_msg_cb(int slot, std::string side, const gz::msgs::Contacts &_gz_contacts_msg);
    void center_slot_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg); 
  };
}

#endif // ARIAC_PLUGINS__AGV_TRAY_PLUGIN_HPP