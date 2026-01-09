#ifndef ARIAC_PLUGINS__TRAY_PLUGIN_HPP_
#define ARIAC_PLUGINS__TRAY_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/create_server.hpp"

#include <ariac_components/cell.hpp>
#include <ariac_components/kit.hpp>

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
  struct SlotCell {
    gz::sim::Entity entity;
    bool defective;
    int type;
    float voltage;
  };

  struct SlotContact {
    std::string left;
    std::string right;
  };

  enum class TrayState {
    READY_TO_LOCK_TO_AGV,
    LOCKED_TO_AGV,
    UNLOCKING_FROM_AGV,
    TELEPORTING,
    LOCKING_TO_SHELF,
    LOCKED_TO_SHELF
  };

  class KitTrayPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public:   
      ~KitTrayPlugin() override;
      
      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_manager) override;
      
      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

    private:

      // General variables
      std::string tray_name;
      std::string agv_name;
      int agv_station = AGVStations::INSPECTION;
      TrayState tray_status = TrayState::READY_TO_LOCK_TO_AGV;
      std::map<int, std::optional<SlotCell>> collision_slot_cells = {
        {1, std::nullopt},
        {2, std::nullopt},
        {3, std::nullopt},
        {4, std::nullopt}
      };
      std::map<int, std::optional<SlotCell>> locked_cells = {
        {1, std::nullopt},
        {2, std::nullopt},
        {3, std::nullopt},
        {4, std::nullopt}
      };
      std::map<int, SlotContact> slot_contacts = {
        { 1, {"", ""}},
        { 2, {"", ""}},
        { 3, {"", ""}},
        { 4, {"", ""}},
      };

      // General functions
      void update_slot_cells(gz::sim::EntityComponentManager &);
    
      // General Info
      std::map<int, double> nominal_voltages = {
        {CellTypes::LI_ION, CellTypes::LI_ION_NOMINAL_VOLTAGE},
        {CellTypes::NIMH, CellTypes::NIMH_NOMINAL_VOLTAGE},
      };

      // GZ Node
      std::shared_ptr<gz::transport::Node> gz_node;

      // GZ Models/Links/Entities/Joints/Components
      gz::sim::Model model;
      gz::sim::Link base_link;
      std::map<int, gz::sim::Entity> lock_joints = {
          { 1, gz::sim::kNullEntity },
          { 2, gz::sim::kNullEntity },
          { 3, gz::sim::kNullEntity },
          { 4, gz::sim::kNullEntity },
      };
      std::vector<gz::sim::Entity> cells_to_ensure_deleted = {};
      ariac_components::Kit kit_component;

      // GZ Topic Callbacks
      void cell_contacts_msg_cb(int, std::string, const gz::msgs::StringMsg_V &);

      // GZ Service Callbacks
      bool handle_kitting_submission(const gz::msgs::Empty &, gz::msgs::Boolean &);

      // GZ Slot Locking/Unlocking
      void lock_slot(int, gz::sim::EntityComponentManager &);
      void unlock_slot(int, gz::sim::EntityComponentManager &);

      // GZ Other Methods
      void clear_tray(gz::sim::EntityComponentManager &);

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;

      // ROS Subscriptions
      std::optional<rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr> location_subscription;

      // ROS Topic CBs
      void agv_info_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg);

      // ROS Services
      rclcpp::Service<ariac_interfaces::srv::Trigger>::SharedPtr recycle_srv;
      rclcpp::Service<ariac_interfaces::srv::CheckKitQuality>::SharedPtr check_kit_quality_srv;

      // Ros Service CBs
      void recycle_cells_cb(const ariac_interfaces::srv::Trigger::Request::SharedPtr, ariac_interfaces::srv::Trigger::Response::SharedPtr);
      void check_kit_quality_cb(const ariac_interfaces::srv::CheckKitQuality::Request::SharedPtr, ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr);
  };
}

#endif // ARIAC_PLUGINS__TRAY_PLUGIN_HPP_