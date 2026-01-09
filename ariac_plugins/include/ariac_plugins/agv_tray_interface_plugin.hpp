#ifndef ARIAC_PLUGINS__AGV_TRAY_INTERFACE_PLUGIN_HPP_
#define ARIAC_PLUGINS__AGV_TRAY_INTERFACE_PLUGIN_HPP_

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
#include <gz/msgs/entity_factory.pb.h>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/create_server.hpp"

#include <ariac_components/cell.hpp>
#include <ariac_components/kit.hpp>
#include <ariac_components/shelf_slot.hpp>

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
#include "angles/angles.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>

using AGVStations = ariac_interfaces::msg::AgvStations;
using AGVStatus = ariac_interfaces::msg::AgvStatus;
using CellTypes = ariac_interfaces::msg::CellTypes;

namespace ariac_plugins
{
  enum class TrayState {
    INSERTING,
    LOCKING,
    LOCKED,
    UNLOCKING,
    MOVING_TO_SHELF,
    LOCKING_TO_SHELF,    
  };

  class AgvTrayInterfacePlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public:   
      ~AgvTrayInterfacePlugin() override;
      
      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_manager) override;
      
      void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;

    private:

      // General variables
      std::optional<std::string> current_tray_name = std::nullopt;
      
      std::string agv_name;
      
      int agv_station = AGVStations::INSPECTION;
      int tray_index = 0;
      
      TrayState tray_status = TrayState::INSERTING;

      std::optional<int> wait_until_iteration;

      // General Information
      const std::string floor_link_name = "floor";

      std::map<int, double> nominal_voltages = {
        {CellTypes::LI_ION, CellTypes::LI_ION_NOMINAL_VOLTAGE},
        {CellTypes::NIMH, CellTypes::NIMH_NOMINAL_VOLTAGE},
      };

      // GZ Node
      std::shared_ptr<gz::transport::Node> gz_node;

      // GZ Models/Links/Entities/Joints/Components
      gz::sim::Model agv_model;
      gz::sim::Link agv_base_link;

      gz::sim::Entity lock_joint;
      gz::sim::Entity agv_base_link_entity;

      std::optional<ariac_components::Kit> kit_component = std::nullopt; 

      gz::math::Pose3d tray_transform = gz::math::Pose3d(0.0, 0.0, 0.35, 0.0, 0.0, 1.57);
      gz::math::Pose3d tray_spawn_transform = gz::math::Pose3d(0.0, 0.0, 0.3501, 0.0, 0.0, 1.57);

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;

      // ROS Subscriptions
      std::optional<rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr> location_subscription;

      // ROS Publishers
      rclcpp::Publisher<ariac_interfaces::msg::AgvTrayStatus>::SharedPtr agv_tray_info_pub;

      // ROS Services
      rclcpp::Service<ariac_interfaces::srv::Trigger>::SharedPtr recycle_srv;
      rclcpp::Service<ariac_interfaces::srv::CheckKitQuality>::SharedPtr check_kit_quality_srv;

      // ROS Topic CBs
      void agv_info_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg);

      // Ros Service CBs
      void recycle_cells_cb(const ariac_interfaces::srv::Trigger::Request::SharedPtr, ariac_interfaces::srv::Trigger::Response::SharedPtr);
      void check_kit_quality_cb(const ariac_interfaces::srv::CheckKitQuality::Request::SharedPtr, ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr);
      
      // GZ Service Callbacks
      bool handle_kitting_submission(const gz::msgs::Int32 &, gz::msgs::Boolean &);
      ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr validate_kit(int cell_type);
      void spawn_tray(gz::math::Pose3d agv_pose, std::string tray_name);
  };
}

#endif // ARIAC_PLUGINS__AGV_TRAY_INTERFACE_PLUGIN_HPP