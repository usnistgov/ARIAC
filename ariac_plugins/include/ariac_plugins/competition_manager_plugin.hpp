#ifndef ARIAC_PLUGINS__COMPETITION_MANAGER_PLUGIN_HPP
#define ARIAC_PLUGINS__COMPETITION_MANAGER_PLUGIN_HPP

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>
#include <unistd.h>    // for mkstemp
#include <fcntl.h>     // for file flags
#include <iostream>
#include <fstream>

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
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/server_control.pb.h>

#include <ariac_components/trial.hpp>
#include <ariac_components/cell.hpp>
#include <ariac_components/module.hpp>
#include <ariac_components/penalty.hpp>
#include <ariac_components/inspection_results.hpp>
#include <ariac_components/feed_results.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ariac_interfaces/msg/competition_states.hpp>
#include <ariac_interfaces/msg/competition_status.hpp>
#include <ariac_interfaces/msg/competition_time.hpp>
#include <ariac_interfaces/msg/high_priority_order.hpp>
#include <ariac_interfaces/msg/agv_status.hpp>
#include <ariac_interfaces/msg/agv_stations.hpp>
#include <ariac_interfaces/msg/cell_types.hpp>
#include <ariac_interfaces/msg/vacuum_tools.hpp>
#include <ariac_interfaces/srv/submit_high_priority_order.hpp>
#include <ariac_interfaces/srv/trigger.hpp>
#include <ariac_interfaces/srv/end_competition.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <db_manager/db_manager.hpp>

using CompetitionStates = ariac_interfaces::msg::CompetitionStates;
using CompetitionStatus = ariac_interfaces::msg::CompetitionStatus;
using CompetitionTime = ariac_interfaces::msg::CompetitionTime;
using HighPriorityOrderMsg = ariac_interfaces::msg::HighPriorityOrder;
using SubmitHighPriorityOrder = ariac_interfaces::srv::SubmitHighPriorityOrder;
using AGVStatus = ariac_interfaces::msg::AgvStatus;
using AGVStations = ariac_interfaces::msg::AgvStations;
using CellTypes = ariac_interfaces::msg::CellTypes;
using VacuumTools = ariac_interfaces::msg::VacuumTools;
using Trigger = ariac_interfaces::srv::Trigger;
using EndCompetition = ariac_interfaces::srv::EndCompetition;

using TriggerReqPtr = Trigger::Request::SharedPtr;
using TriggerResPtr = Trigger::Response::SharedPtr;
using EndCompetitionReqPtr = EndCompetition::Request::SharedPtr;
using EndCompetitionResPtr = EndCompetition::Response::SharedPtr;
using SubmitHighPriorityOrderReqPtr = SubmitHighPriorityOrder::Request::SharedPtr;
using SubmitHighPriorityOrderResPtr = SubmitHighPriorityOrder::Response::SharedPtr;

namespace ariac_plugins{

enum class SubmissionStatus {
  NOT_REQUESTED,
  REQUESTED,
  SUCCESSFUL,
  FAIL
};

struct SubmissionResponse {
  SubmissionStatus status;
  std::string message;
};

struct HighPriorityOrder {
  std::string id;
  bool submitted;
  bool published;
  double announcement_time;
};

class CompetitionManagerPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public:
    ~CompetitionManagerPlugin() override;

    void Configure (const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) final;
  
  private:
    void read_trial(std::string filepath);
    void publish_status();

    void update_competition_time();
    bool orders_complete();


    SubmissionResponse check_module(gz::sim::EntityComponentManager &_ecm, ariac_components::Module module);

    bool connect_to_database(std::string trial_config);

    void handle_module_order_submission(gz::sim::EntityComponentManager &_ecm);

    void handle_competition_end(gz::sim::EntityComponentManager &_ecm);
    void shutdown_gazebo();

    std::string create_temp_file();
    int get_agv_at_shipping();

    std::vector<ariac_components::Module> get_modules_in_bbox(
      gz::sim::EntityComponentManager &_ecm,
      gz::math::AxisAlignedBox bbox);

    // ROS Service Callbacks
    void start_competition_cb(const TriggerReqPtr, TriggerResPtr);
    void end_competition_cb(const EndCompetitionReqPtr, EndCompetitionResPtr);
    void submit_kitting_cb(const TriggerReqPtr, TriggerResPtr);
    void submit_high_priority_cb(const SubmitHighPriorityOrderReqPtr, SubmitHighPriorityOrderResPtr);
    void submit_module_cb(const TriggerReqPtr, TriggerResPtr);

    // ROS Subscriber Callbacks
    void agv1_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg);
    void agv2_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg);
    void agv3_station_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg);

    // ROS
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

    rclcpp::CallbackGroup::SharedPtr pub_cb_group;
    rclcpp::TimerBase::SharedPtr status_pub_timer;
    rclcpp::Publisher<CompetitionStatus>::SharedPtr competition_status_pub;
    rclcpp::Publisher<HighPriorityOrderMsg>::SharedPtr high_priority_pub;

    rclcpp::Service<Trigger>::SharedPtr start_competition_srv;
    rclcpp::Service<EndCompetition>::SharedPtr end_competition_srv;

    rclcpp::Service<Trigger>::SharedPtr submit_kitting_srv;
    rclcpp::Service<SubmitHighPriorityOrder>::SharedPtr submit_high_priority_srv;
    rclcpp::Service<Trigger>::SharedPtr submit_module_srv;

    rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr agv1_info_sub;
    rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr agv2_info_sub;
    rclcpp::Subscription<ariac_interfaces::msg::AgvStatus>::SharedPtr agv3_info_sub;

    rclcpp::Time end_time;

    // GZ
    std::shared_ptr<gz::transport::Node> gz_node;
    gz::math::AxisAlignedBox shipping_bbox{gz::math::Vector3d(6.2, 2.25, 0.0), gz::math::Vector3d(6.8, 2.85, 1.0)};
    gz::math::AxisAlignedBox module_submission_bbox{gz::math::Vector3d(4.1, 6.75, 0.41), gz::math::Vector3d(4.3, 6.88, 0.57)};
  
    // Other
    bool log_generated = false;
    bool connected_to_db = false;
    bool shutdown = false;
    bool end_handled = false;

    std::unique_ptr<ariac_db::DatabaseManager> db_manager;

    std::string competitor_name;

    int sensor_cost;
    int competitor_id;
    int trial_id;
    int run_id = -1;
    int competition_state = CompetitionStates::PREPARING;
    // double cell_voltage_tolerance = 0.2;
    // double kit_voltage_tolerance = 0.15;

    double competition_ended_time;

    ariac_components::Trial trial;

    CompetitionTime competition_time;

    std::vector<HighPriorityOrder> high_priority_orders;
    std::vector<gz::sim::Entity> detachable_joints_to_delete;
    std::vector<gz::sim::Entity> cells_to_delete;

    std::map<int, double> nominal_voltages = {
      {CellTypes::LI_ION, CellTypes::LI_ION_NOMINAL_VOLTAGE},
      {CellTypes::NIMH, CellTypes::NIMH_NOMINAL_VOLTAGE},
    };

    std::vector<ariac_db::OrderSubmissionData> order_submissions;

    std::map<ariac_db::OrderType, int> num_submitted_orders = {
      {ariac_db::OrderType::KIT, 0},
      {ariac_db::OrderType::MODULE, 0},
      {ariac_db::OrderType::HIGH_PRIORITY, 0}
    };

    std::map<int, int> agv_locations = {
      {1, AGVStations::INSPECTION},
      {2, AGVStations::INSPECTION},
      {3, AGVStations::INSPECTION},
    };

    SubmissionResponse module_submission_response;
  };
}

#endif // ARIAC_PLUGINS__COMPETITION_MANAGER_PLUGIN_HPP