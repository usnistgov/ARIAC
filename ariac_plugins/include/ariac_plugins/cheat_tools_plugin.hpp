#ifndef ARIAC_PLUGINS__CHEAT_TOOLS_PLUGIN_HPP_
#define ARIAC_PLUGINS__CHEAT_TOOLS_PLUGIN_HPP_

// GZ
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>

// GZ msgs
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/Utility.hh>

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS MSGS
#include <ariac_interfaces/msg/cell_defect.hpp>
#include <ariac_interfaces/msg/cell_types.hpp>
#include <ariac_interfaces/msg/cell_feeder_status.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>

// Custom ARIAC Components
#include <ariac_components/cell.hpp>
#include <ariac_components/module.hpp>

// ROS SRVS
#include <ariac_interfaces/srv/trigger.hpp>
#include <ariac_interfaces/srv/control_cell_feeder.hpp>

// OTHER
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
#include <random>
#include <ament_index_cpp/get_package_share_directory.hpp>


using CellTypes = ariac_interfaces::msg::CellTypes;
using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;
using FeederStatusMsg = ariac_interfaces::msg::CellFeederStatus;
using Trigger = ariac_interfaces::srv::Trigger;

using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;
using ControlSrv = ariac_interfaces::srv::ControlCellFeeder;
using ControlSrvReqPtr = ControlSrv::Request::SharedPtr;
using ControlSrvResPtr = ControlSrv::Response::SharedPtr;

namespace ariac_plugins{

enum class ShellTypes {
  TOP,
  BOTTOM
};

enum class TeleportStatus {
  NOT_NEEDED,
  REQUESTED,
  TELEPORTED
};

class CheatToolsPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:

  void Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_event_mgr) override;
  
  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) final;
  
  private:

  std::shared_ptr<gz::transport::Node> gz_node;

  TeleportStatus teleport_bottom_shell = TeleportStatus::NOT_NEEDED;

  rclcpp::Node::SharedPtr ros_node;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  std::thread thread_executor_spin;
  
  rclcpp::Service<Trigger>::SharedPtr agv1_spawn_kit_srv;
  rclcpp::Service<Trigger>::SharedPtr agv2_spawn_kit_srv;
  rclcpp::Service<Trigger>::SharedPtr agv3_spawn_kit_srv;
  
  void agv1_spawn_kit_cb_(const TriggerReqPtr, TriggerResPtr);
  void agv2_spawn_kit_cb_(const TriggerReqPtr, TriggerResPtr);
  void agv3_spawn_kit_cb_(const TriggerReqPtr, TriggerResPtr);

  // Other methods
  void spawn_cell(ariac_components::Cell cell, gz::math::Pose3d);
  ariac_components::Cell get_next_cell(bool);
  std::optional<std::string> generate_cell_sdf(ariac_components::Cell);

  void complete_kit(int, bool);
  void complete_module();
  void partial_module();
  void flipped_module();
  void spawn_cells_in_voltage_testers();
  bool change_visual_status(const gz::sim::Entity, bool);
  bool spawn_shell(ShellTypes, gz::math::Pose3d);
  std::optional<std::string> get_shell_xml(ShellTypes);

  std::map<ShellTypes, std::string> shell_paths;

  std::map<ShellTypes, std::string> shell_names = {
    {ShellTypes::TOP, "top"},
    {ShellTypes::BOTTOM, "bottom"}
  };

  std::string sdf_path;
  std::shared_ptr<const sdf::Element> sdf;

  int cell_count = 0;
  int request_step = -1;
  bool welds_requested = false;
  bool log_cell_info = false;
  std::string bottom_shell_name;
  std::vector<std::string> logged_cells = {};
  std::map<int, std::vector<ariac_interfaces::msg::CellDefect>> defect_info;

  std::map<int, std::string> cell_colors =
  {
    {CellTypes::LI_ION, "0.62 0.42 0.72 1.0"},
    {CellTypes::NIMH, "0.92 0.66 0.2 1.0"}
  };

  std::map<int, gz::math::Pose3d> agv_slot_offsets = {
    {1, gz::math::Pose3d(-0.1, 0.05, 0.0, -1.57, 0.0, -1.57)},
    {2, gz::math::Pose3d(0.03, 0.05, 0.0, -1.57, 0.0, -1.57)},
    {3, gz::math::Pose3d(-0.1, -0.05, 0.0, -1.57, 0.0, -1.57)},
    {4, gz::math::Pose3d(0.03, -0.05, 0.0, -1.57, 0.0, -1.57)}
  };

  std::map<int, gz::math::Pose3d> agv_poses = {
    {1, gz::math::Pose3d(2.15, 1.50, 0.39, 0.0, 0.0, 0.0)},
    {2, gz::math::Pose3d(2.50, 1.50, 0.39, 0.0, 0.0, 0.0)},
    {3, gz::math::Pose3d(2.85, 1.50, 0.39, 0.0, 0.0, 0.0)}
  };

  std::map<ShellTypes, gz::math::Pose3d> shell_poses = {
    {ShellTypes::TOP, gz::math::Pose3d(4.2, 5.55, 0.519, M_PI, 0.0, 0.0)},
    {ShellTypes::BOTTOM, gz::math::Pose3d(4.2, 5.55, 0.43, 0.0, 0.0, 0.0)}
  };

  gz::math::Pose3d partial_module_bottom_shell_pose = gz::math::Pose3d(4.6, 5.0, 0.43, 0.0, 0.0, 0.0);

  gz::math::Pose3d flipped_bottom_shell_pose = gz::math::Pose3d(4.2, 5.55, 0.519, M_PI, 0.0, M_PI);

  std::map<ShellTypes, gz::math::Pose3d> flipped_shell_poses = {
    {ShellTypes::TOP, gz::math::Pose3d(4.2, 5.55, 0.089, M_PI, 0.0, 0.0)},
    {ShellTypes::BOTTOM, gz::math::Pose3d(4.2, 5.55, 0.0, 0.0, 0.0, 0.0)}
  };

  std::map<int, gz::math::Pose3d> vt_cell_poses = {
    {1, gz::math::Pose3d(2.0, 0.91, 0.43, -M_PI_2, 0.0, -M_PI)},
    {2, gz::math::Pose3d(2.0, 0.76, 0.43, -M_PI_2, 0.0, -M_PI)}
  };

  std::map<int, gz::math::Pose3d> module_slot_offsets = {
    {1, gz::math::Pose3d(-0.036, 0.0, 0.004, 0.0, 0.0, 0.0)},
    {2, gz::math::Pose3d(-0.012, 0.0, 0.074, M_PI, 0.0, 0.0)},
    {3, gz::math::Pose3d(0.012, 0.0, 0.004, 0.0, 0.0, 0.0)},
    {4, gz::math::Pose3d(0.036, 0.0, 0.074, M_PI, 0.0, 0.0)}
  };

  std::map<int, std::string> cell_names =
  {
    {CellTypes::LI_ION, "li-ion"},
    {CellTypes::NIMH, "nimh"},  
  };

  std::vector<std::pair<std::string, ariac_components::Cell>> components_to_add;
};
}

#endif // ARIAC_PLUGINS__CHEAT_TOOLS_PLUGIN_HPP_