#ifndef ARIAC_PLUGINS__CELL_FEED_PLUGIN_HPP
#define ARIAC_PLUGINS__CELL_FEED_PLUGIN_HPP

// GZ
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>

// GZ msgs
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/Utility.hh>

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS MSGS
#include <ariac_interfaces/msg/cell_types.hpp>
#include <ariac_interfaces/msg/cell_feeder_status.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>
#include <ariac_interfaces/msg/conveyor_status.hpp>

// Custom ARIAC Components
#include <ariac_components/trial.hpp>
#include <ariac_components/cell.hpp>
#include <ariac_components/feed_results.hpp>

// ROS SRVS
#include <ariac_interfaces/srv/control_cell_feeder.hpp>

// OTHER
#include <yaml-cpp/yaml.h>
#include <tinyxml2.h>
#include <random>
#include <ament_index_cpp/get_package_share_directory.hpp>


using CellTypes = ariac_interfaces::msg::CellTypes;
using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;
using ConveyorStatus = ariac_interfaces::msg::ConveyorStatus;
using FeederStatusMsg = ariac_interfaces::msg::CellFeederStatus;

using ControlSrv = ariac_interfaces::srv::ControlCellFeeder;
using ControlSrvReqPtr = ControlSrv::Request::SharedPtr;
using ControlSrvResPtr = ControlSrv::Response::SharedPtr;

namespace ariac_plugins{

enum class FeedStatus {
  WAIT_FOR_CELL,
  CREATE_CELL,
  ADD_CELL_COMPONENT,
};

class CellFeedPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:
  ~CellFeedPlugin() override;

  void Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_event_mgr) override;
  
  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) final;
  
  private:

  rclcpp::Node::SharedPtr ros_node;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
  std::thread thread_executor_spin;
  
  std::shared_ptr<gz::transport::Node> gz_node;

  // Services
  rclcpp::Service<ControlSrv>::SharedPtr control_feed_srv;

  // Service Callbacks
  void control_feed_cb(const ControlSrvReqPtr, ControlSrvResPtr);

  // Topic Callbacks
  void competition_status_cb(const CompetitionStatus::SharedPtr);

  // Subscribers
  rclcpp::Subscription<CompetitionStatus>::SharedPtr competition_status_sub;

  // Publishers
  rclcpp::Publisher<FeederStatusMsg>::SharedPtr status_pub;

  // Timers
  rclcpp::TimerBase::SharedPtr pub_timer;

  // Parameters
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle;

  // Timer Callback
  void publish_status_cb();

  // Other methods
  void spawn_cell(ariac_components::Cell cell);
  ariac_components::Cell get_next_cell();
  std::optional<std::string> generate_cell_sdf(ariac_components::Cell);

  // Constants
  const double voltage_std_dev = 0.1;

  // Variables
  bool configured = false;

  int seed;
  int competition_state = CompetitionStates::PREPARING;

  double defect_rate;
  double last_spawn_time = -INFINITY;
  double conveyor_speed = 0.1;

  std::string sdf_path;

  // RNG
  std::mt19937 rng;
  std::uniform_real_distribution<double> defect_distribution;
  std::uniform_int_distribution<> defect_type_distribution;
  std::uniform_real_distribution<double> rotation_distribution;
  std::normal_distribution<double> voltage_offset;

  // GZ
  gz::sim::Entity model_entity;
  gz::sim::Entity world_entity = 1;
  gz::math::Vector3d battery_spawn_location = {0.55, 1, 0.422};

  // Custom 
  FeederStatusMsg status_msg;
  FeedStatus feed_status = FeedStatus::WAIT_FOR_CELL;

  ariac_components::Cell current_cell;
  ariac_components::FeedResults feed_results;

  std::vector<int> defect_types;
  std::vector<int> possible_defects;
  std::map<int, std::string> defect_type_to_visual;

  std::map<int, double> nominal_voltages = 
  {
    {CellTypes::LI_ION, CellTypes::LI_ION_NOMINAL_VOLTAGE},
    {CellTypes::NIMH, CellTypes::NIMH_NOMINAL_VOLTAGE}
  };

  std::map<int, std::string> cell_names =
  {
    {CellTypes::LI_ION, "li-ion"},
    {CellTypes::NIMH, "nimh"},  
  };
};
}

#endif // ARIAC_PLUGINS__CELL_FEED_PLUGIN_HPP