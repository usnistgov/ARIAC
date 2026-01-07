#ifndef ARIAC_PLUGINS__PHYSICAL_INSPECTION_PLUGIN_HPP_
#define ARIAC_PLUGINS__PHYSICAL_INSPECTION_PLUGIN_HPP_

#include <angles/angles.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// GZ
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/sim/components/Pose.hh> 

#include <ariac_components/cell.hpp>
#include <ariac_components/inspection_results.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// MSGS
#include <ariac_interfaces/msg/cell_defect.hpp>
#include <ariac_interfaces/msg/inspection_report.hpp>

// SRVS
#include <ariac_interfaces/srv/submit_inspection_report.hpp>

using Defect = ariac_interfaces::msg::CellDefect;
using InspectionReport = ariac_interfaces::msg::InspectionReport;

using SubmissionSrv = ariac_interfaces::srv::SubmitInspectionReport;
using SubmissionSrvReqPtr = SubmissionSrv::Request::SharedPtr;
using SubmissionSrvResPtr = SubmissionSrv::Response::SharedPtr;

namespace ariac_plugins
{
enum class InspectionStatus {
  DOOR_OPEN,
  DOOR_CLOSED,
  DOOR_OPENING,
  DOOR_CLOSING,
  PROCESSING,
  FINISHED_PROCESSING,
  WAITING_FOR_CELL,
};

class PhysicalInspectionPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:
  ~PhysicalInspectionPlugin() override;

  void Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_event_manager) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  private:
  // ROS callbacks
  void submission_cb(const SubmissionSrvReqPtr request, SubmissionSrvResPtr response);

  // GZ CBs
  void conveyor_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);

  void validate_report(const ariac_interfaces::msg::InspectionReport&);

  // GZ
  gz::sim::Model model;
  gz::sim::Entity model_entity;
  gz::sim::Joint door_joint;
  gz::sim::Entity current_cell = gz::sim::kNullEntity;

  std::shared_ptr<gz::transport::Node> gz_node;

  // SDF Params
  double max_speed;

  // ROS
  rclcpp::Node::SharedPtr ros_node;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  std::thread thread_executor_spin;

  rclcpp::Service<SubmissionSrv>::SharedPtr submission_srv;
  
  // Variables
  double speed = 0.0;

  const double door_x = 1.1;
  const double closed_position = 0.0;
  const double opened_position = 0.785;

  const double report_height_threshold = 0.005; // ±5mm
  const double report_angle_threshold = 0.26; // ±15°
  
  std::map<std::string, double> cell_positions = {
    {"open_door", 1.0},
    {"close_door", 1.25},
  };

  std::map<int, std::vector<ariac_interfaces::msg::CellDefect>> defect_info;

  std::vector<std::string> cells_on_conveyor;

  std::optional<ariac_components::Cell> current_cell_data = std::nullopt;
  ariac_components::InspectionResults inspection_results;

  InspectionStatus status = InspectionStatus::DOOR_CLOSED;
};
}

#endif // ARIAC_PLUGINS__PHYSICAL_INSPECTION_PLUGIN_HPP_