/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// standard library
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>
// gazebo
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/LogRecord.hh>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/ModelStates.h"
// ros
#include "geometry_msgs/Pose.h"
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
// gear
#include "nist_gear/ARIAC.hh"
#include "nist_gear/TaskManagerPlugin.hh"
#include "nist_gear/AriacScorer.h"
#include "nist_gear/ConveyorBeltControl.h"
#include "nist_gear/DetectKittingShipment.h"
#include "nist_gear/DetectMovableTray.h"
#include "nist_gear/DetectAssemblyShipment.h"
#include "nist_gear/ExpectedKittingShipment.h"
#include "nist_gear/DetectedKittingShipment.h"
#include "nist_gear/AssemblyShipment.h"
#include "nist_gear/Product.h"
#include "nist_gear/MovableTray.h"
#include "nist_gear/Order.h"
#include "nist_gear/RobotHealth.h"
#include "nist_gear/VacuumGripperState.h"
#include "nist_gear/DropProducts.h"
#include "nist_gear/SubmitKittingShipment.h"
#include "nist_gear/MoveToStation.h"
#include "nist_gear/MovableTrayInOrder.h"
#include "nist_gear/TrayContents.h"
#include "nist_gear/AgvInOrder.h"
#include "nist_gear/KittingInOrder.h"
#include "nist_gear/ProductInOrder.h"
#include "nist_gear/Orders.h"
#include "nist_gear/AssemblyInOrder.h"
#include <nist_gear/DetectKitTrayContent.h>
// #include "nist_gear/DetectConnectedPartsToBriefcase.h"

namespace gazebo
{
/**
 * @internal
 * @struct TaskManagerPluginPrivate
 * @brief Private data for the TaskManagerPlugin class.
 */
struct TaskManagerPluginPrivate
{
public:
  //!@brief World pointer
  physics::WorldPtr world;
  //!@brief SDF pointer
  sdf::ElementPtr sdf;
  //!@brief Collection of orders to announce
  std::vector<ariac::Order> orders_to_announce;
  //!@brief Current order being processed
  ariac::Order currentOrder;
  //!@brief Collection of orders which have been announced but are not yet complete
  std::stack<ariac::Order> ordersInProgress;
  //!@brief Mapping between material types and their locations
  std::map<std::string, std::vector<std::string>> materialLocations;
  //!@brief Data structure to store kitting shipments
  std::map<std::string, nist_gear::TrayContents::ConstPtr> kitting_shipment_contents;
  //!@brief Data structure to store assembly shipments
  std::map<std::string, nist_gear::DetectedAssemblyShipment::ConstPtr> assembly_shipment_contents;
  //!@brief A scorer to manage the trial score
  AriacScorer ariac_scorer;
  /*!< The current game score. */
  ariac::GameScore current_trial_score;
  /*!< ROS node handle. */
  std::unique_ptr<ros::NodeHandle> rosNode;
  /*!< Publishes an order. */
  ros::Publisher orderPub;
  /*!< Subscription for tray content */
  ros::Subscriber kittingShipmentContentSubscriber;

  ros::Subscriber gantryPositionSubscriber;
  /*!< Subscription for briefcase content */
  ros::Subscriber assemblyShipmentContentSubscriber;
  /*!< Subscriber to retrieve the health status of both robots */
  ros::Subscriber robotHealthSubscriber;
  /*!< Subscription to get the current station of AGV1*/
  ros::Subscriber stationForAGV1Sub;
  /*!< Subscription to get the current station of AGV2*/
  ros::Subscriber stationForAGV2Sub;
  /*!< Subscription to get the current station of AGV3*/
  ros::Subscriber stationForAGV3Sub;
  /*!< Subscription to get the current station of AGV4*/
  ros::Subscriber stationForAGV4Sub;
  //!@brief Publisher for storing each model to drop on each agv
  ros::Publisher drop_object_publisher;
  ros::Publisher taskStatePub;
  /*!< Publishes the game score total. */
  ros::Publisher taskScorePub;
  /*!< Publishes the health of the robots. */
  ros::Publisher robot_health_pub;
  /*!< Publishes the current location of an AGV. */
  ros::Publisher agv1_current_station_publisher;
  /*!< Publishes the current location of an AGV. */
  ros::Publisher agv2CurrentStationPub;
  /*!< Publishes the current location of an AGV. */
  ros::Publisher agv3CurrentStationPub;
  /*!< Publishes the current location of an AGV. */
  ros::Publisher agv4CurrentStationPub;
  /*!< Name of service that allows the user to start the competition. */
  std::string compStartServiceName;
  /*!< Service that allows the user to start the competition. */
  ros::ServiceServer compStartServiceServer;
  /*!< Service that allows the user to end the competition. */
  ros::ServiceServer compEndServiceServer;
  /*!< Service that retrieves the pose of the gantry in Gazebo. */
  ros::ServiceServer gazeboRobotPoseServiceServer;
  /*!< Service that allows the user to change the gantry's gripper. */
  ros::ServiceServer gripperChangeServiceServer;
  /*!< Service that allows users to query the location of materials. */
  ros::ServiceServer getMaterialLocationsServiceServer;
  /*!< Service that allows a tray to be submitted for inspection. */
  ros::ServiceServer submitTrayServiceServer;
  /*!< Map of agv id to server that handles requests to deliver shipment. */
  std::map<int, ros::ServiceServer> agvDeliverService;

  /*!< To ship the content of a briefcase. */
  std::map<int, ros::ServiceServer> station_ship_content_service;
  // std::map<int, ros::ServiceServer> connected_parts_to_briefcase_service;
  /*!< Map of agv id to client that can get its content. */
  std::map<int, ros::ServiceClient> kit_tray_content_client;
  std::map<int, ros::ServiceClient> get_kit_tray_content_client;
  /*!< Map of assembly station id to client that can get its content. */
  std::map<int, ros::ServiceClient> stationGetContentClient;
  //  std::map<int, ros::ServiceClient> stationGetConnectedPartClient;

  std::map<int, ros::ServiceServer> ks_to_as1_ServiceServer;
  std::map<int, ros::ServiceServer> ks_to_as2_ServiceServer;
  std::map<int, ros::ServiceServer> ks_to_as3_ServiceServer;
  std::map<int, ros::ServiceServer> ks_to_as4_ServiceServer;
  std::map<int, ros::ServiceServer> as1_to_as2_ServiceServer;
  std::map<int, ros::ServiceServer> as2_to_as1_ServiceServer;
  std::map<int, ros::ServiceServer> as3_to_as4_ServiceServer;
  std::map<int, ros::ServiceServer> as4_to_as3_ServiceServer;
  std::map<int, ros::ServiceServer> as1_to_ks_ServiceServer;
  std::map<int, ros::ServiceServer> as2_to_ks_ServiceServer;
  std::map<int, ros::ServiceServer> as3_to_ks_ServiceServer;
  std::map<int, ros::ServiceServer> as4_to_ks_ServiceServer;
  std::map<int, ros::ServiceServer> submit_kitting_shipment_server;
  std::map<int, ros::ServiceServer> get_kitting_shipment_server;
  std::map<int, ros::ServiceServer> move_to_station_server;

  std::map<int, ros::ServiceClient> ks_to_as1_AnimateClient;
  std::map<int, ros::ServiceClient> ks_to_as2_AnimateClient;
  std::map<int, ros::ServiceClient> ks_to_as3_AnimateClient;
  std::map<int, ros::ServiceClient> ks_to_as4_AnimateClient;
  std::map<int, ros::ServiceClient> as1_to_as2_AnimateClient;
  std::map<int, ros::ServiceClient> as2_to_as1_AnimateClient;
  std::map<int, ros::ServiceClient> as3_to_as4_AnimateClient;
  std::map<int, ros::ServiceClient> as4_to_as3_AnimateClient;
  std::map<int, ros::ServiceClient> as1_to_ks_AnimateClient;
  std::map<int, ros::ServiceClient> as2_to_ks_AnimateClient;
  std::map<int, ros::ServiceClient> as3_to_ks_AnimateClient;
  std::map<int, ros::ServiceClient> as4_to_ks_AnimateClient;

  /*!< Map of agv id to client that can ask AGV to move to as2 */
  std::map<int, ros::ServiceClient> agvToAS2AnimateClient;
  /*!< Map of agv id to client that can ask AGV to move to as3 */
  std::map<int, ros::ServiceClient> agvToAS3AnimateClient;
  /*!< Map of agv id to client that can ask AGV to move to as4 */
  std::map<int, ros::ServiceClient> agvToAS4AnimateClient;
  /*!< Client that turns on conveyor belt. */
  ros::ServiceClient conveyorControlClient;
  /*!< Transportation node. */
  transport::NodePtr gz_node;
  /*!< Publisher for enabling the product population on the conveyor. */
  transport::PublisherPtr populatePub;
  /*!< Publisher for controlling the blackout of sensors. */
  transport::PublisherPtr sensor_blackout_control_publisher;

  /*!< Duration at which to blackout sensors. */
  double sensor_blackout_duration;
  /*!< Product count at which to blackout sensors. */
  int sensor_blackout_product_count = 0;
  /*!< If sensor blackout is currently in progress. */
  bool is_sensor_blackout_in_progress = false;
  /*!< flag to activate the belt. */
  bool isBeltNeeded = false;
  //!@brief Health status of the kitting robot
  std::string kitting_robot_health;
  //!@brief Health status of the gantry robot
  std::string assembly_robot_health;
  /*!< The start time of the sensor blackout. */
  common::Time sensor_blackout_start_time;
  /*!< Timer for regularly publishing state/score. */
  ros::Timer statusPubTimer;
  /*!< Connection event. */
  event::ConnectionPtr connection;
  /*!< Publish Gazebo server control messages. */
  transport::PublisherPtr serverControlPub;
  /*!< The time the last update was called. */
  common::Time lastUpdateTime;
  /*!< The time the sim time was last published. */
  common::Time lastSimTimePublish;
  common::Time gameStartTime;               /*!< The time specified in the product is relative to this time. */
  double timeLimit;                         /*!< The time in seconds permitted to complete the trial. */
  double timeSpentOnCurrentOrder;           /*!< The time in seconds that has been spent on the current order. */
  std::string currentState = "init";        /*!< Pointer to the current state. */
  std::mutex mutex;                         /*!< The A mutex to protect currentState. */
  bool competitionMode = false;             /*!< During the competition, this environment variable will be set. */
  transport::SubscriberPtr contactSub;      /*!< Subscriber for the contact topic. */
  transport::SubscriberPtr floorPenaltySub; /*!< Subscriber for the floor penalty topic. */
  int floorPenalty;                         /*!< Total number of parts dropped on floor. */
  /*!< The assembly station to which the agv is sent when submitting a kitting shipment. */
  std::string submitted_kitting_shipment_station;
  //!@brief Assembly station used for an assembly submission
  std::string submitted_assembly_shipment_station;
  /*!< AGV id used when submitting a kitting shipment. */
  int submitted_shipment_agv;
  /*!< Name of the assembly shipment station. */
  std::string assemblyShipmentStation;
  std::string agv1_current_station;
  std::string agv2_current_station;
  std::string agv3_current_station;
  std::string agv4_current_station;
  /*!< Controller manager service to switch controllers for the gantry. */
  // ros::ServiceClient gantry_controller_manager_client;
  // /*!< Controller manager service to switch controllers for the kitting robot. */
  // ros::ServiceClient kitting_controller_manager_client;
  bool kitting_robot_running;
  bool gantry_robot_running;
  std::vector<std::string> gantry_consistent_controllers_;
  std::vector<std::string> gantry_stopped_controllers_;
  std::string gripper_tray_type;
  std::string gripper_part_type;
  double gripper_changing_station_lower_x;
  double gripper_changing_station_lower_y;
  double gripper_changing_station_higher_x;
  double gripper_changing_station_higher_y;
  std::string current_gripper_type;
  double gantry_world_x;
  double gantry_world_y;
  // std::string gripper_param;

  /**
   * @brief Publishes the status of as2
   *
   * This is used by the ROSPersonByStation plugin.
   * The person will move away from as2 when there is an agv at as2
   */
  ros::Publisher as2StatusPublisher;

  /**
   * @brief Publishes the status of as4
   *
   * This is used by the ROSPersonByStation plugin.
   * The person will move away from as4 when there is an agv at as4
   */
  ros::Publisher as4StatusPublisher;

  //!@brief Publisher to publish the current gripper mounted on the gantry robot
  ros::Publisher gantry_gripper_type_publisher;
  //!@brief Publisher to publish the current gripper mounted on the kitting robot
  ros::Publisher kitting_gripper_type_publisher;
  //!@brief Topic name to publish the current gripper for the gantry robot
  std::string gantry_gripper_type_topic;
  //!@brief Topic name to publish the current gripper for the kitting robot
  std::string kitting_gripper_type_topic;

  //!@brief Name of the config yaml storing trial information
  std::string config_yaml_file;

  const std::vector<std::string> gantry_collision_filter_vec{
    "base_link_collision",    "shoulder_link_collision", "upper_arm_link_collision", "forearm_link_collision",
    "wrist_1_link_collision", "wrist_2_link_collision",  "wrist_3_link_collision",   "vacuum_gripper_link_collision"
  };

  // std::vector<std::string> gantry_static_controllers;
  // std::vector<std::string> kitting_static_controllers;
};

}  // namespace gazebo

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(TaskManagerPlugin)

/**
 * @brief Build an order ROS Message using the order from ariac.world
 *
 * @param txt_order Order information retrieved from ariac.world
 * @param msg_order Order message to be published on /ariac/orders
 */
static void fill_order_message(const ariac::Order& yaml_order, nist_gear::Order& msg_order)
{
  msg_order.order_id = yaml_order.order_id;
  // if the order consists of a kitting task
  if (yaml_order.has_kitting_task)
  {
    // parse each kitting shipment
    for (const auto& yaml_shipment : yaml_order.kitting_shipments)
    {
      // compact the shipment in nist_gear::KittingShipment
      nist_gear::ExpectedKittingShipment expected_shipment_msg;
      expected_shipment_msg.shipment_type = yaml_shipment.shipment_type;
      expected_shipment_msg.assembly_station = yaml_shipment.assembly_station;

      nist_gear::TrayContents tray_content_msg;
      tray_content_msg.kit_tray = yaml_shipment.agv_id;
      tray_content_msg.movable_tray.movable_tray_type = yaml_shipment.movable_tray.type;
      // pose of the movable tray
      tray_content_msg.movable_tray.movable_tray_pose.position.x = yaml_shipment.movable_tray.pose.Pos().X();
      tray_content_msg.movable_tray.movable_tray_pose.position.y = yaml_shipment.movable_tray.pose.Pos().Y();
      tray_content_msg.movable_tray.movable_tray_pose.position.z = yaml_shipment.movable_tray.pose.Pos().Z();
      tray_content_msg.movable_tray.movable_tray_pose.orientation.x = yaml_shipment.movable_tray.pose.Rot().X();
      tray_content_msg.movable_tray.movable_tray_pose.orientation.y = yaml_shipment.movable_tray.pose.Rot().Y();
      tray_content_msg.movable_tray.movable_tray_pose.orientation.z = yaml_shipment.movable_tray.pose.Rot().Z();
      tray_content_msg.movable_tray.movable_tray_pose.orientation.w = yaml_shipment.movable_tray.pose.Rot().W();
      // pose of parts in the movable tray
      for (const auto& product : yaml_shipment.products)
      {
        nist_gear::DetectedProduct product_msg;
        product_msg.type = product.type;
        product_msg.is_faulty = false;
        product_msg.pose.position.x = product.pose.Pos().X();
        product_msg.pose.position.y = product.pose.Pos().Y();
        product_msg.pose.position.z = product.pose.Pos().Z();
        product_msg.pose.orientation.x = product.pose.Rot().X();
        product_msg.pose.orientation.y = product.pose.Rot().Y();
        product_msg.pose.orientation.z = product.pose.Rot().Z();
        product_msg.pose.orientation.w = product.pose.Rot().W();
        // Add the product to the shipment.
        tray_content_msg.products.push_back(product_msg);
      }
      expected_shipment_msg.tray_content = tray_content_msg;

      msg_order.kitting_shipments.push_back(expected_shipment_msg);
    }
  }

  // if the order has an assembly task
  if (yaml_order.has_assembly_task)
  {
    for (const auto& shipment : yaml_order.assembly_shipments)
    {
      nist_gear::AssemblyShipment assembly_shipment_msg;
      assembly_shipment_msg.shipment_type = shipment.shipmentType;
      assembly_shipment_msg.station_id = shipment.assembly_station;
      // get information for each product (type and pose)
      for (const auto& product : shipment.products)
      {
        nist_gear::Product product_msg;
        product_msg.type = product.type;
        product_msg.pose.position.x = product.pose.Pos().X();
        product_msg.pose.position.y = product.pose.Pos().Y();
        product_msg.pose.position.z = product.pose.Pos().Z();
        product_msg.pose.orientation.x = product.pose.Rot().X();
        product_msg.pose.orientation.y = product.pose.Rot().Y();
        product_msg.pose.orientation.z = product.pose.Rot().Z();
        product_msg.pose.orientation.w = product.pose.Rot().W();

        // Add the product to the shipment.
        assembly_shipment_msg.products.push_back(product_msg);
      }
      msg_order.assembly_shipments.push_back(assembly_shipment_msg);
    }
  }
}

/////////////////////////////////////////////////
TaskManagerPlugin::TaskManagerPlugin() : data_ptr(new TaskManagerPluginPrivate)
{
}

/////////////////////////////////////////////////
TaskManagerPlugin::~TaskManagerPlugin()
{
  // ROS_ERROR_STREAM("[TaskManagerPlugin] Destructor");
  this->data_ptr->rosNode->shutdown();
}

/**
 * @brief The Load function is called by Gazebo when the plugin is inserted in simulation.
 *
 *  It parses ariac.world and fill out data structures.
 *
 * @param world A pointer to the model that this plugin is attached to.
 * @param sdf  pointer to the plugin's SDF element.
 */
void TaskManagerPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  gzdbg << "ARIAC VERSION: v.05.30.2022\n";

  // ROS_WARN_STREAM("[TaskManagerPlugin::Load]");

  auto competitionEnv = std::getenv("ARIAC_COMPETITION");
  this->data_ptr->competitionMode = competitionEnv != NULL;
  gzdbg << "ARIAC COMPETITION MODE: " << (this->data_ptr->competitionMode ? competitionEnv : "false") << std::endl;

  GZ_ASSERT(world, "TaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(sdf, "TaskManagerPlugin sdf pointer is NULL");
  this->data_ptr->world = world;
  this->data_ptr->sdf = sdf;

  // Initialize Gazebo transport.
  this->data_ptr->gz_node = transport::NodePtr(new transport::Node());
  this->data_ptr->gz_node->Init();

  std::string robotNamespace = "";
  if (sdf->HasElement("robot_namespace"))
  {
    robotNamespace = sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
  }

  /*
  Avoid the slowdown that is present with contact manager filter in gazebo 9.12 by
  subscribing to gazebo's main contact topic.
  */
  this->data_ptr->contactSub =
      this->data_ptr->gz_node->Subscribe("~/physics/contacts", &TaskManagerPlugin::OnContactsReceived, this);

  // initialize ROS
  this->data_ptr->rosNode.reset(new ros::NodeHandle(robotNamespace));

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Process data from ariac.world

  this->data_ptr->gantry_gripper_type_topic = "";
  if (sdf->HasElement("gantry_gripper_type_topic"))
    this->data_ptr->gantry_gripper_type_topic = sdf->Get<std::string>("gantry_gripper_type_topic");

  this->data_ptr->kitting_gripper_type_topic = "";
  if (sdf->HasElement("kitting_gripper_type_topic"))
    this->data_ptr->kitting_gripper_type_topic = sdf->Get<std::string>("kitting_gripper_type_topic");

  this->data_ptr->config_yaml_file = "";
  if (sdf->HasElement("config_yaml_file"))
    this->data_ptr->config_yaml_file = sdf->Get<std::string>("config_yaml_file");

  this->data_ptr->current_gripper_type = "";
  if (sdf->HasElement("current_gripper_type"))
    this->data_ptr->current_gripper_type = sdf->Get<std::string>("current_gripper_type");

  this->data_ptr->gripper_tray_type = "gripper_tray_type";
  if (sdf->HasElement("gripper_tray_type"))
    this->data_ptr->gripper_tray_type = sdf->Get<std::string>("gripper_tray_type");

  this->data_ptr->gripper_part_type = "gripper_part_type";
  if (sdf->HasElement("gripper_part_type"))
    this->data_ptr->gripper_part_type = sdf->Get<std::string>("gripper_part_type");

  this->data_ptr->gripper_changing_station_lower_x = -4.25;
  if (sdf->HasElement("gripper_changing_station_lower_x"))
    this->data_ptr->gripper_changing_station_lower_x = sdf->Get<double>("gripper_changing_station_lower_x");

  this->data_ptr->gripper_changing_station_higher_x = -3.60;
  if (sdf->HasElement("gripper_changing_station_higher_x"))
    this->data_ptr->gripper_changing_station_higher_x = sdf->Get<double>("gripper_changing_station_higher_x");

  this->data_ptr->gripper_changing_station_lower_y = 5.50;
  if (sdf->HasElement("gripper_changing_station_lower_y"))
    this->data_ptr->gripper_changing_station_lower_y = sdf->Get<double>("gripper_changing_station_lower_y");

  this->data_ptr->gripper_changing_station_higher_y = 6.90;
  if (sdf->HasElement("gripper_changing_station_higher_y"))
    this->data_ptr->gripper_changing_station_higher_y = sdf->Get<double>("gripper_changing_station_higher_y");

  // this->data_ptr->gripper_param = "/ariac/gripper_type";
  // if (sdf->HasElement("gripper_param"))
  //   this->data_ptr->gripper_param = sdf->Get<std::string>("gripper_param");
  // set current gripper on the parameter server
  // this->data_ptr->rosNode->setParam(this->data_ptr->gripper_param, this->data_ptr->current_gripper_type);

  this->data_ptr->timeLimit = -1.0;
  if (sdf->HasElement("competition_time_limit"))
    this->data_ptr->timeLimit = sdf->Get<double>("competition_time_limit");

  std::string compEndServiceName = "end_competition";
  if (sdf->HasElement("end_competition_service_name"))
    compEndServiceName = sdf->Get<std::string>("end_competition_service_name");

  this->data_ptr->compStartServiceName = "start_competition";
  if (sdf->HasElement("start_competition_service_name"))
    this->data_ptr->compStartServiceName = sdf->Get<std::string>("start_competition_service_name");

  std::string taskStateTopic = "competition_state";
  if (sdf->HasElement("task_state_topic"))
    taskStateTopic = sdf->Get<std::string>("task_state_topic");

  std::string taskScoreTopic = "current_score";
  if (sdf->HasElement("task_score_topic"))
    taskScoreTopic = sdf->Get<std::string>("task_score_topic");

  std::string conveyorEnableTopic = "conveyor/enable";
  if (sdf->HasElement("conveyor_enable_topic"))
    conveyorEnableTopic = sdf->Get<std::string>("conveyor_enable_topic");

  std::string conveyorControlService = "conveyor/control";
  if (sdf->HasElement("conveyor_control_service"))
    conveyorControlService = sdf->Get<std::string>("conveyor_control_service");

  std::string populationActivateTopic = "populate_belt";
  if (sdf->HasElement("population_activate_topic"))
  {
    populationActivateTopic = sdf->Get<std::string>("population_activate_topic");
    // Gazebo topic publisher for belt population
    this->data_ptr->populatePub = this->data_ptr->gz_node->Advertise<msgs::GzString>(populationActivateTopic);
  }

  // gzdbg << "population active topic: " << populationActivateTopic << "\n";

  std::string ordersTopic = "orders";
  if (sdf->HasElement("orders_topic"))
    ordersTopic = sdf->Get<std::string>("orders_topic");

  std::string kittingContentTopic = "kitting_shipment_content";
  if (sdf->HasElement("kitting_shipment_content_topic_name"))
    kittingContentTopic = sdf->Get<std::string>("kitting_shipment_content_topic_name");

  std::string assemblyContentTopic = "assembly_shipment_content";
  if (sdf->HasElement("assembly_shipment_content_topic_name"))
    assemblyContentTopic = sdf->Get<std::string>("assembly_shipment_content_topic_name");

  std::string getMaterialLocationsServiceName = "material_locations";
  if (sdf->HasElement("material_locations_service_name"))
    getMaterialLocationsServiceName = sdf->Get<std::string>("material_locations_service_name");

  std::string floorPenaltyTopic = "/ariac/floor_penalty";
  if (sdf->HasElement("floor_penalty_topic"))
  {
    floorPenaltyTopic = sdf->Get<std::string>("floor_penalty_topic");
  }
  // gazebo subscriber for floor penalty info
  this->data_ptr->floorPenaltySub =
      this->data_ptr->gz_node->Subscribe(floorPenaltyTopic, &TaskManagerPlugin::OnPenaltyReceived, this);
  this->data_ptr->floorPenalty = 0;

  std::string robotHealthTopic = "robot_health";
  if (sdf->HasElement("robot_health_topic"))
    robotHealthTopic = sdf->Get<std::string>("robot_health_topic");

  std::string gripperChangeService{};
  if (sdf->HasElement("gripper_change_service_name"))
    gripperChangeService = sdf->Get<std::string>("gripper_change_service_name");

  // Checking for belt population cycles
  if (sdf->HasElement("belt_population_cycles"))
  {
    int belt_cycles = sdf->Get<int>("belt_population_cycles");

    if (belt_cycles > 0)
    {
      this->data_ptr->isBeltNeeded = true;
    }
  }

  // Publishers
  //^^^^^^^^^^^

  // Publisher for the gripper for the gantry robot
  this->data_ptr->gantry_gripper_type_publisher =
      this->data_ptr->rosNode->advertise<std_msgs::String>(this->data_ptr->gantry_gripper_type_topic, 10, true);
  // Publisher for the gripper for the kitting robot
  this->data_ptr->kitting_gripper_type_publisher =
      this->data_ptr->rosNode->advertise<std_msgs::String>(this->data_ptr->kitting_gripper_type_topic, 10, true);

  // set the kitting robot gripper to be a part gripper
  std_msgs::String gripper_msg;
  gripper_msg.data = this->data_ptr->gripper_part_type;
  this->data_ptr->kitting_gripper_type_publisher.publish(gripper_msg);

  // set the gantry robot gripper using current_gripper_type from the yaml file
  gripper_msg.data = this->data_ptr->current_gripper_type;
  this->data_ptr->gantry_gripper_type_publisher.publish(gripper_msg);

  /////////////////////////////////////////////////////////////////////////
  /*
  <plugin filename="libTaskManagerPlugin.so" name="task_manager">
  <station index="@(station_id)">
          <station_shipment_service_name>/ariac/as@(station_id)/submit_shipment</station_shipment_service_name>
          <get_content_service_name>/ariac/briefcase_@(station_id)/get_content</get_content_service_name>
        </station>
  </plugin>
  */
  /////////////////////////////////////////////////////////////////////////

  // get the content of a briefcase
  std::map<int, std::string> station_get_content_service_name;
  // std::map<int, std::string> station_get_connected_part_service_name;
  // submit an assembly shipment at a station
  std::map<int, std::string> station_submit_shipment_service_name;

  if (sdf->HasElement("station"))
  {
    sdf::ElementPtr stationElem = sdf->GetElement("station");
    while (stationElem)
    {
      int index = stationElem->Get<int>("index");
      station_get_content_service_name[index] = "get_content";
      station_submit_shipment_service_name[index] = "submit_shipment";
      // station_get_connected_part_service_name[index] = "";

      // <get_content_service_name>
      if (stationElem->HasElement("get_content_service_name"))
      {
        station_get_content_service_name[index] = stationElem->Get<std::string>("get_content_service_name");
      }

      // <get_connected_part_service_name>
      // service name to get the parts touching a briefcase
      // if (stationElem->HasElement("get_connected_part_service_name")) {
      //   station_get_connected_part_service_name[index] =
      //   stationElem->Get<std::string>("get_connected_part_service_name");
      // }

      // <station_shipment_service_name>
      if (stationElem->HasElement("station_shipment_service_name"))
      {
        station_submit_shipment_service_name[index] = stationElem->Get<std::string>("station_shipment_service_name");
      }
      stationElem = stationElem->GetNextElement("station");
    }
  }

  /////////////////////////////////////////////////////////////////////////
  /*
  <plugin filename="libTaskManagerPlugin.so" name="task_manager">
    <agv index="@(agv_id)">
    ...
    </agv>
  </plugin>
  */
  /////////////////////////////////////////////////////////////////////////

  std::map<int, std::string> ks_to_as1_AnimationServiceName{};
  std::map<int, std::string> as1_to_ks_AnimationServiceName{};
  std::map<int, std::string> ks_to_as2_AnimationServiceName{};
  std::map<int, std::string> as2_to_ks_AnimationServiceName{};
  std::map<int, std::string> as1_to_as2_AnimationServiceName{};
  std::map<int, std::string> as2_to_as1_AnimationServiceName{};
  std::map<int, std::string> ks_to_as3_AnimationServiceName{};
  std::map<int, std::string> as3_to_ks_AnimationServiceName{};
  std::map<int, std::string> ks_to_as4_AnimationServiceName{};
  std::map<int, std::string> as4_to_ks_AnimationServiceName{};
  std::map<int, std::string> as3_to_as4_AnimationServiceName{};
  std::map<int, std::string> as4_to_as3_AnimationServiceName{};

  // std::map<int, std::string> agvDeliverServiceName;
  // get the content of the AGV
  std::map<int, std::string> kit_tray_content_name;
  // service name for submitting a kitting shipment
  std::map<int, std::string> submit_kitting_shipment_name;
  std::map<int, std::string> get_kit_tray_content_name;
  // service name for just moving an AGV between stations
  std::map<int, std::string> move_to_station_name;
  std::map<int, std::string> agv_to_as_service_name;
  std::map<int, std::string> agv_to_ks_service_name;

  // a service used to task an AGV to go to a specific station
  // e.g., /ariac/agv1/to_assembly_station AS1 order_0_kitting_shipment_0
  std::map<int, std::string> agvLocationTopic;
  std::map<int, std::string> agvStartLocation;

  // information on the location of human beings
  this->data_ptr->as2StatusPublisher =
      this->data_ptr->rosNode->advertise<std_msgs::Bool>("/ariac/as2/occupied", 10, true);
  // information on the location of human beings
  this->data_ptr->as4StatusPublisher =
      this->data_ptr->rosNode->advertise<std_msgs::Bool>("/ariac/as4/occupied", 10, true);

  if (sdf->HasElement("agv"))
  {
    sdf::ElementPtr agvElem = sdf->GetElement("agv");
    while (agvElem)
    {
      int index = agvElem->Get<int>("index");

      submit_kitting_shipment_name[index] = "";
      get_kit_tray_content_name[index] = "";
      move_to_station_name[index] = "";
      kit_tray_content_name[index] = "get_content";
      agvLocationTopic[index] = "";
      agvStartLocation[index] = "";
      agv_to_as_service_name[index] = "";
      agv_to_ks_service_name[index] = "";

      if (agvElem->HasElement("agv_location_topic"))
      {
        agvLocationTopic[index] = agvElem->Get<std::string>("agv_location_topic");
        // publisher for setting the location (station) of AGVs in the environment
        // gzdbg << index << ":" << agvLocationTopic[index] << "\n";
        if (index == 1)
        {
          this->data_ptr->agv1_current_station_publisher =
              this->data_ptr->rosNode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);

          this->data_ptr->stationForAGV1Sub = this->data_ptr->rosNode->subscribe(
              agvLocationTopic[index], 1000, &TaskManagerPlugin::OnAGV1Location, this);
        }
        if (index == 2)
        {
          this->data_ptr->agv2CurrentStationPub =
              this->data_ptr->rosNode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);

          this->data_ptr->stationForAGV2Sub = this->data_ptr->rosNode->subscribe(
              agvLocationTopic[index], 1000, &TaskManagerPlugin::OnAGV2Location, this);
        }
        if (index == 3)
        {
          this->data_ptr->agv3CurrentStationPub =
              this->data_ptr->rosNode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);

          this->data_ptr->stationForAGV3Sub = this->data_ptr->rosNode->subscribe(
              agvLocationTopic[index], 1000, &TaskManagerPlugin::OnAGV3Location, this);
        }
        if (index == 4)
        {
          this->data_ptr->agv4CurrentStationPub =
              this->data_ptr->rosNode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);

          this->data_ptr->stationForAGV4Sub = this->data_ptr->rosNode->subscribe(
              agvLocationTopic[index], 1000, &TaskManagerPlugin::OnAGV4Location, this);
        }
      }
      if (agvElem->HasElement("agv_start_location_name"))
      {
        agvStartLocation[index] = agvElem->Get<std::string>("agv_start_location_name");
        std_msgs::String msg;
        msg.data = agvStartLocation[index];

        // this section is used for PersonByStationPlugin
        // first we set messages published to these topics to false
        // we will set those values to true when an agv is located at as2 or as4
        std_msgs::Bool tmp_msg;
        tmp_msg.data = false;
        this->data_ptr->as2StatusPublisher.publish(tmp_msg);
        this->data_ptr->as4StatusPublisher.publish(tmp_msg);

        // agv1
        if (index == 1)
        {
          this->data_ptr->agv1_current_station = agvStartLocation[index];
          this->data_ptr->agv1_current_station_publisher.publish(msg);
          // this parameter will be used in ROSAGVPlugin.cc
          this->data_ptr->rosNode->setParam("/ariac/agv1/current_station", agvStartLocation[index]);
        }

        // agv2
        if (index == 2)
        {
          this->data_ptr->agv2_current_station = agvStartLocation[index];
          this->data_ptr->agv2CurrentStationPub.publish(msg);
          // this parameter will be used in ROSAGVPlugin.cc
          this->data_ptr->rosNode->setParam("/ariac/agv2/current_station", agvStartLocation[index]);
        }

        // agv3
        if (index == 3)
        {
          this->data_ptr->agv3_current_station = agvStartLocation[index];
          this->data_ptr->agv3CurrentStationPub.publish(msg);
          // this parameter will be used in ROSAGVPlugin.cc
          this->data_ptr->rosNode->setParam("/ariac/agv3/current_station", agvStartLocation[index]);
        }

        // agv4
        if (index == 4)
        {
          this->data_ptr->agv4_current_station = agvStartLocation[index];
          this->data_ptr->agv4CurrentStationPub.publish(msg);
          // this parameter will be used in ROSAGVPlugin.cc
          this->data_ptr->rosNode->setParam("/ariac/agv4/current_station", agvStartLocation[index]);
        }
      }

      // if (agvElem->HasElement("agv_control_service_name")) {
      //   agvDeliverServiceName[index] = agvElem->Get<std::string>("agv_control_service_name");
      // }

      // used to call the service in ROSAriacKitTrayPlugin
      if (agvElem->HasElement("get_content_service_name"))
      {
        kit_tray_content_name[index] = agvElem->Get<std::string>("get_content_service_name");
      }
      if (agvElem->HasElement("submit_kitting_shipment_name"))
      {
        submit_kitting_shipment_name[index] = agvElem->Get<std::string>("submit_kitting_shipment_name");
      }
      if (agvElem->HasElement("get_kit_tray_content_name"))
      {
        get_kit_tray_content_name[index] = agvElem->Get<std::string>("get_kit_tray_content_name");
      }

      if (agvElem->HasElement("move_to_station_name"))
      {
        move_to_station_name[index] = agvElem->Get<std::string>("move_to_station_name");
      }

      // animation service
      /////////////////////////////////////////////////////////////////////////
      /*
      <plugin filename="libTaskManagerPlugin.so" name="task_manager">
        <agv index="@(agv_id)">
          <ks_to_as1_name>/ariac/agv@(agv_id)/ks_to_as1</ks_to_as1_name>
          <ks_to_as2_name>/ariac/agv@(agv_id)/ks_to_as2</ks_to_as2_name>
          <as1_to_as2_name>/ariac/agv@(agv_id)/as1_to_as2</as1_to_as2_name>
          <as2_to_as1_name>/ariac/agv@(agv_id)/as2_to_as1</as2_to_as1_name>
          <as1_to_ks_name>/ariac/agv@(agv_id)/as1_to_ks</as1_to_ks_name>
          <as2_to_ks_name>/ariac/agv@(agv_id)/as2_to_ks</as2_to_ks_name>
        </agv>
      </plugin>
      */
      /////////////////////////////////////////////////////////////////////////
      if (agvElem->HasElement("ks_to_as1_name"))
      {
        ks_to_as1_AnimationServiceName[index] = agvElem->Get<std::string>("ks_to_as1_name");
      }
      if (agvElem->HasElement("as1_to_ks_name"))
      {
        as1_to_ks_AnimationServiceName[index] = agvElem->Get<std::string>("as1_to_ks_name");
      }
      if (agvElem->HasElement("ks_to_as2_name"))
      {
        ks_to_as2_AnimationServiceName[index] = agvElem->Get<std::string>("ks_to_as2_name");
      }
      if (agvElem->HasElement("as2_to_ks_name"))
      {
        as2_to_ks_AnimationServiceName[index] = agvElem->Get<std::string>("as2_to_ks_name");
      }
      if (agvElem->HasElement("as1_to_as2_name"))
      {
        as1_to_as2_AnimationServiceName[index] = agvElem->Get<std::string>("as1_to_as2_name");
      }
      if (agvElem->HasElement("as2_to_as1_name"))
      {
        as2_to_as1_AnimationServiceName[index] = agvElem->Get<std::string>("as2_to_as1_name");
      }
      /////////////////////////////////////////////////////////////////////////
      /*
      <plugin filename="libTaskManagerPlugin.so" name="task_manager">
        <agv index="@(agv_id)">
          <ks_to_as3_name>/ariac/agv@(agv_id)/ks_to_as3</ks_to_as3_name>
          <ks_to_as4_name>/ariac/agv@(agv_id)/ks_to_as4</ks_to_as4_name>
          <as3_to_as4_name>/ariac/agv@(agv_id)/as3_to_as4</as3_to_as4_name>
          <as4_to_as3_name>/ariac/agv@(agv_id)/as4_to_as3</as4_to_as3_name>
          <as3_to_ks_name>/ariac/agv@(agv_id)/as3_to_ks</as3_to_ks_name>
          <as4_to_ks_name>/ariac/agv@(agv_id)/as4_to_ks</as4_to_ks_name>
        </agv>
      </plugin>
      */
      /////////////////////////////////////////////////////////////////////////
      if (agvElem->HasElement("ks_to_as3_name"))
      {
        ks_to_as3_AnimationServiceName[index] = agvElem->Get<std::string>("ks_to_as3_name");
      }
      if (agvElem->HasElement("as3_to_ks_name"))
      {
        as3_to_ks_AnimationServiceName[index] = agvElem->Get<std::string>("as3_to_ks_name");
      }
      if (agvElem->HasElement("ks_to_as4_name"))
      {
        ks_to_as4_AnimationServiceName[index] = agvElem->Get<std::string>("ks_to_as4_name");
      }
      if (agvElem->HasElement("as4_to_ks_name"))
      {
        as4_to_ks_AnimationServiceName[index] = agvElem->Get<std::string>("as4_to_ks_name");
      }
      if (agvElem->HasElement("as3_to_as4_name"))
      {
        as3_to_as4_AnimationServiceName[index] = agvElem->Get<std::string>("as3_to_as4_name");
      }
      if (agvElem->HasElement("as4_to_as3_name"))
      {
        as4_to_as3_AnimationServiceName[index] = agvElem->Get<std::string>("as4_to_as3_name");
      }

      agvElem = agvElem->GetNextElement("agv");
    }
  }

  /////////////////////////////////////////////////////////////////////////
  /*
  <plugin filename="libTaskManagerPlugin.so" name="task_manager">
    <order>
    ...
    </order>
  </plugin>
  */
  /////////////////////////////////////////////////////////////////////////
  sdf::ElementPtr orderElem = NULL;
  if (sdf->HasElement("order"))
  {
    orderElem = sdf->GetElement("order");
  }

  while (orderElem)
  {
    // Parse the order name.
    ariac::OrderID_t orderID = orderElem->Get<std::string>("name");
    // ROS_WARN_STREAM("Order ID: "<< orderID);

    int order_priority = 1;
    if (orderElem->HasElement("priority"))
    {
      sdf::ElementPtr order_priorityElem = orderElem->GetElement("priority");
      order_priority = order_priorityElem->Get<int>();
    }

    // Check if the order has a kitting field
    bool has_kitting_task = false;
    if (orderElem->HasElement("kitting_shipment"))
    {
      has_kitting_task = true;
    }

    // Check if the order has a kitting field
    bool has_assembly_task = false;
    if (orderElem->HasElement("assembly_shipment"))
    {
      has_assembly_task = true;
    }

    // parse the start time.
    double startTime = std::numeric_limits<double>::infinity();
    // time at which this order is announced
    if (orderElem->HasElement("start_time"))
    {
      sdf::ElementPtr startTimeElement = orderElem->GetElement("start_time");
      startTime = startTimeElement->Get<double>();
    }

    ariac::NewOrderAnnouncement new_order_announcement{};
    // Trigger a new order (priority 3) when unwanted products are placed on the tray
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    int interruptOnUnwantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_unwanted_products"))
    {
      sdf::ElementPtr interruptOnUnwantedProductsElem = orderElem->GetElement("interrupt_on_unwanted_products");
      interruptOnUnwantedProducts = interruptOnUnwantedProductsElem->Get<int>();
      new_order_announcement.set_type("unwanted_products");
      new_order_announcement.set_part_number(interruptOnUnwantedProducts);
    }

    // Trigger a new order (priority 3) when wanted products are placed on the tray
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    int interruptOnWantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_wanted_products"))
    {
      sdf::ElementPtr interruptOnWantedProductsElem = orderElem->GetElement("interrupt_on_wanted_products");
      interruptOnWantedProducts = interruptOnWantedProductsElem->Get<int>();
      new_order_announcement.set_type("wanted_products");
      new_order_announcement.set_part_number(interruptOnWantedProducts);
    }

    // Trigger a new order (priority 1) when a kitting shipment is submitted
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    std::string on_kitting_submission_agv = "";
    std::string on_kitting_submission_station = "";
    if (orderElem->HasElement("on_kitting_submission"))
    {
      sdf::ElementPtr on_kitting_submission_ptr = orderElem->GetElement("on_kitting_submission");
      sdf::ElementPtr agv_elem_ptr = on_kitting_submission_ptr->GetElement("agv");
      on_kitting_submission_agv = agv_elem_ptr->Get<std::string>();
      sdf::ElementPtr station_ptr = on_kitting_submission_ptr->GetElement("station");
      on_kitting_submission_station = station_ptr->Get<std::string>();
      new_order_announcement.set_type("on_kitting_submission");
      new_order_announcement.set_agv(on_kitting_submission_agv);
      new_order_announcement.set_station(on_kitting_submission_station);
    }

    // Trigger a new order (priority 1) when an assembly shipment is submitted
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    std::string on_assembly_submission_station = "";
    if (orderElem->HasElement("on_assembly_submission"))
    {
      sdf::ElementPtr on_assembly_submission_ptr = orderElem->GetElement("on_assembly_submission");
      sdf::ElementPtr station_ptr = on_assembly_submission_ptr->GetElement("station");
      on_assembly_submission_station = station_ptr->Get<std::string>();
      new_order_announcement.set_type("on_assembly_submission");
      new_order_announcement.set_station(on_assembly_submission_station);
    }

    // information on when to disable robots
    // Check if a robot needs to be disabled during this order
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ariac::RobotDisableCondition robot_disable_condition;
    robot_disable_condition.is_order_with_disabled_robot = false;
    robot_disable_condition.robot_type = "";
    robot_disable_condition.disable_condition = 0;
    robot_disable_condition.disable_value = 0;

    if (orderElem->HasElement("robot_to_disable"))
    {
      sdf::ElementPtr robot_to_disable_elem = orderElem->GetElement("robot_to_disable");
      // robot type
      sdf::ElementPtr robot_type_elem_ptr = robot_to_disable_elem->GetElement("robot_type");
      robot_disable_condition.robot_type = robot_type_elem_ptr->Get<std::string>();
      // location
      sdf::ElementPtr disable_condition_elem_ptr = robot_to_disable_elem->GetElement("disable_condition");
      robot_disable_condition.disable_condition = disable_condition_elem_ptr->Get<int>();
      // nb_of_parts
      sdf::ElementPtr nb_of_parts_elem_ptr = robot_to_disable_elem->GetElement("disable_value");
      robot_disable_condition.disable_value = nb_of_parts_elem_ptr->Get<int>();
      // set this attribute to true
      robot_disable_condition.is_order_with_disabled_robot = true;
    }

    // the start health status for each robot
    std::string kitting_robot_health{};
    std::string assembly_robot_health{};
    if (orderElem->HasElement("kitting_robot_health"))
    {
      sdf::ElementPtr kitting_robot_healthElem = orderElem->GetElement("kitting_robot_health");
      auto tmp = kitting_robot_healthElem->Get<unsigned int>();
      if (tmp == 0)
      {
        kitting_robot_health = "inactive";
      }
      else if (tmp == 1)
      {
        kitting_robot_health = "active";
      }
    }

    if (orderElem->HasElement("assembly_robot_health"))
    {
      sdf::ElementPtr assembly_robot_healthElem = orderElem->GetElement("assembly_robot_health");
      // assembly_robot_health = assembly_robot_healthElem->Get<int>();
      auto tmp = assembly_robot_healthElem->Get<unsigned int>();
      if (tmp == 0)
      {
        assembly_robot_health = "inactive";
      }
      else if (tmp == 1)
      {
        assembly_robot_health = "active";
      }
    }

    // Parse the allowed completion time.
    double allowedTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("allowed_time"))
    {
      sdf::ElementPtr allowedTimeElement = orderElem->GetElement("allowed_time");
      allowedTime = allowedTimeElement->Get<double>();
    }

    // an order needs at least one kitting shipment OR one assembly shipment
    if (!orderElem->HasElement("kitting_shipment") && !orderElem->HasElement("assembly_shipment"))
    {
      gzerr << "Unable to find <kitting_shipment> or <assembly_element> element in <order>. Ignoring" << std::endl;
      orderElem = orderElem->GetNextElement("order");
      continue;
    }

    // Store all kitting shipments for an order.
    std::vector<ariac::KittingShipment> kitting_shipments;

    ////////////////////////////////////////////
    //-- check if kitting is part of the order
    ////////////////////////////////////////////
    if (orderElem->HasElement("kitting_shipment"))
    {
      sdf::ElementPtr shipmentElem = orderElem->GetElement("kitting_shipment");
      while (shipmentElem)
      {
        // Check the validity of the shipment.
        if (!shipmentElem->HasElement("product"))
        {
          gzerr << "Unable to find <product> element in <kitting_shipment>. Ignoring" << std::endl;
          shipmentElem = shipmentElem->GetNextElement("kitting_shipment");
          continue;
        }

        // kitting:
        //  shipment_count: 1
        //  trays: [movable_tray_dark_wood]
        //  agvs : [agv1]
        //  destinations : [as1]
        //  products :
        //    part_0 :
        //      type : assembly_pump_green
        //      pose :
        //        xyz: [-0.1, -0.1, 0]
        //        rpy : [0, 0, 0]
        //    part_1 :
        //      type : assembly_pump_green
        //      pose :
        //        xyz: [0.0, -0.1, 0]
        //        rpy : [0, 0, 0]
        ariac::KittingShipment kitting_shipment;

        //--which agv to use to build the kit
        //--default is 'any'
        kitting_shipment.agv_id = "any";
        if (shipmentElem->HasElement("agv"))
        {
          kitting_shipment.agv_id = shipmentElem->Get<std::string>("agv");
        }
        else
        {
          std::string warnStr = "In YAML file: No AGV provided, 'any' is used.";
          gzerr << warnStr << std::endl;
        }

        //--where to send the AGV once the kit is built
        //--Error if this field is missing from the yaml file
        kitting_shipment.assembly_station = "";
        if (shipmentElem->HasElement("destination"))
        {
          kitting_shipment.assembly_station = shipmentElem->Get<std::string>("destination");
        }
        if (kitting_shipment.assembly_station.empty())
        {
          std::string errStr = "In YAML file: The field 'destinations' for kitting can not be empty.";
          gzerr << errStr << std::endl;
          ROS_ERROR_STREAM(errStr);
        }

        // Parse the shipment type.
        ariac::KittingShipmentType_t kittingShipmentType;
        if (shipmentElem->HasElement("shipment_type"))
        {
          kittingShipmentType = shipmentElem->Get<std::string>("shipment_type");
        }
        kitting_shipment.shipment_type = kittingShipmentType;

        // Parse the trays inside the shipment
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        sdf::ElementPtr movable_tray_elem_ptr = shipmentElem->GetElement("movable_tray");
        sdf::ElementPtr type_elem_ptr = movable_tray_elem_ptr->GetElement("type");
        std::string movable_tray_type = type_elem_ptr->Get<std::string>();
        sdf::ElementPtr pose_elem_ptr = movable_tray_elem_ptr->GetElement("pose");
        ignition::math::Pose3d movable_tray_pose = pose_elem_ptr->Get<ignition::math::Pose3d>();
        ariac::MovableTray movable_tray = { movable_tray_type, "", movable_tray_pose };
        kitting_shipment.movable_tray = movable_tray;

        // Parse the products inside the shipment.
        sdf::ElementPtr productElem = shipmentElem->GetElement("product");
        while (productElem)
        {
          // Parse the product type.
          if (!productElem->HasElement("type"))
          {
            gzerr << "Unable to find <type> in product.\n";
            productElem = productElem->GetNextElement("product");
            continue;
          }
          sdf::ElementPtr typeElement = productElem->GetElement("type");
          std::string type = typeElement->Get<std::string>();

          // Parse the product pose
          if (!productElem->HasElement("pose"))
          {
            gzerr << "Unable to find <pose> in product.\n";
            productElem = productElem->GetNextElement("product");
            continue;
          }
          sdf::ElementPtr poseElement = productElem->GetElement("pose");
          ignition::math::Pose3d pose = poseElement->Get<ignition::math::Pose3d>();
          // ROS_WARN_STREAM("Pose: " << pose);

          // Add the product to the shipment.
          bool isFaulty = false;  // we never want to request faulty products.
          ariac::Product product = { type, isFaulty, pose };
          kitting_shipment.products.push_back(product);

          productElem = productElem->GetNextElement("product");
        }

        // Add a new shipment to the collection of shipments.
        kitting_shipments.push_back(kitting_shipment);

        shipmentElem = shipmentElem->GetNextElement("kitting_shipment");
      }
    }

    ////////////////////////////////////////////
    //-- check if assembly is part of the order
    ////////////////////////////////////////////
    //--std::vector that will be used to store information on <assembly_shipment>
    //--we extract information from the following xml snippet
    // <assembly_shipment>
    //   <shipment_type>order_0_assembly_shipment_0</shipment_type>
    //   <station>AS2</station>
    //   <product>
    //     <type>assembly_battery_blue</type>
    //     <pose>-0.032465 0.174845 0.15 0.0 0.0 0.0</pose>
    //   </product>
    // </assembly_shipment>
    std::vector<ariac::AssemblyShipment> assembly_shipments;
    if (orderElem->HasElement("assembly_shipment"))
    {
      sdf::ElementPtr shipmentElem = orderElem->GetElement("assembly_shipment");
      while (shipmentElem)
      {
        // Check the validity of the shipment.
        if (!shipmentElem->HasElement("product"))
        {
          gzerr << "Unable to find <product> element in <assembly_shipment>. Ignoring" << std::endl;
          shipmentElem = shipmentElem->GetNextElement("assembly_shipment");
          continue;
        }

        ariac::AssemblyShipment assembly_shipment;

        // <station> tag: At which station to do assembly
        assembly_shipment.assembly_station = "";
        if (shipmentElem->HasElement("station"))
        {
          assembly_shipment.assembly_station = shipmentElem->Get<std::string>("station");
        }
        if (assembly_shipment.assembly_station.empty())
        {
          std::string errStr = "In trial config (YAML) file: The field 'station' for assembly can not be empty.";
          gzerr << errStr << std::endl;
          ROS_ERROR_STREAM(errStr);
        }
        // Parse the shipment type.
        ariac::AssemblyShipmentType_t assembly_shipment_t;
        if (shipmentElem->HasElement("shipment_type"))
        {
          assembly_shipment_t = shipmentElem->Get<std::string>("shipment_type");
        }
        assembly_shipment.shipmentType = assembly_shipment_t;

        // Parse the products inside the shipment.
        sdf::ElementPtr productElem = shipmentElem->GetElement("product");
        while (productElem)
        {
          // Parse the product type.
          if (!productElem->HasElement("type"))
          {
            gzerr << "Unable to find <type> in product.\n";
            productElem = productElem->GetNextElement("product");
            continue;
          }
          sdf::ElementPtr typeElement = productElem->GetElement("type");
          std::string type = typeElement->Get<std::string>();

          // Parse the product pose (optional).
          if (!productElem->HasElement("pose"))
          {
            gzerr << "Unable to find <pose> in product.\n";
            productElem = productElem->GetNextElement("product");
            continue;
          }
          sdf::ElementPtr poseElement = productElem->GetElement("pose");
          ignition::math::Pose3d pose = poseElement->Get<ignition::math::Pose3d>();

          // Add the product to the shipment.
          bool isFaulty = false;  // We never want to request faulty products.
          ariac::Product obj = { type, isFaulty, pose };
          assembly_shipment.products.push_back(obj);

          productElem = productElem->GetNextElement("product");
        }

        // Add a new shipment to the collection of shipments.
        assembly_shipments.push_back(assembly_shipment);

        shipmentElem = shipmentElem->GetNextElement("assembly_shipment");
      }
    }

    // ariac::Order constructor to initialize all the attributes
    // Has to follow the same order the attributes are listed in ariac::Order (ARIAC.hh)
    ariac::Order order = { orderID,
                           order_priority,
                           startTime,
                           new_order_announcement,
                           allowedTime,
                           kitting_shipments,
                           assembly_shipments,
                           0.0,  // time taken
                           has_kitting_task,
                           has_assembly_task,
                           robot_disable_condition,
                           kitting_robot_health,
                           assembly_robot_health,
                           this->data_ptr->config_yaml_file };

    this->data_ptr->orders_to_announce.push_back(order);

    orderElem = orderElem->GetNextElement("order");
  }

  // Sort the orders by their start times.
  std::sort(this->data_ptr->orders_to_announce.begin(), this->data_ptr->orders_to_announce.end());

  // Debug output.
  // gzdbg << "Orders:" << std::endl;
  // for (auto order : this->data_ptr->ordersToAnnounce)
  //   gzdbg << order << std::endl;

  // Parse the material storage locations.

  ////////////////////////////////
  /// Material storage locations
  ////////////////////////////////
  if (sdf->HasElement("material_locations"))
  {
    sdf::ElementPtr materialLocationsElem = sdf->GetElement("material_locations");
    sdf::ElementPtr materialElem = NULL;
    if (materialLocationsElem->HasElement("material"))
    {
      materialElem = materialLocationsElem->GetElement("material");
    }
    while (materialElem)
    {
      std::string materialType = materialElem->Get<std::string>("type");
      std::vector<std::string> locations;

      // Parse locations of this material.
      sdf::ElementPtr locationElem = NULL;
      if (materialElem->HasElement("location"))
      {
        locationElem = materialElem->GetElement("location");
      }
      while (locationElem)
      {
        std::string location = locationElem->Get<std::string>("storage_unit");
        locations.push_back(location);
        locationElem = locationElem->GetNextElement("location");
      }
      this->data_ptr->materialLocations[materialType] = locations;
      materialElem = materialElem->GetNextElement("material");
    }
  }

  /////////////////////
  /// Sensor Blackout
  /////////////////////
  if (sdf->HasElement("sensor_blackout"))
  {
    // ROS_ERROR_STREAM("SENSOR BLACKOUT" << "\n");
    auto sensorBlackoutElem = sdf->GetElement("sensor_blackout");
    std::string sensorEnableTopic = sensorBlackoutElem->Get<std::string>("topic");
    this->data_ptr->sensor_blackout_product_count = sensorBlackoutElem->Get<int>("product_count");
    this->data_ptr->sensor_blackout_duration = sensorBlackoutElem->Get<double>("duration");
    this->data_ptr->sensor_blackout_control_publisher =
        this->data_ptr->gz_node->Advertise<msgs::GzString>(sensorEnableTopic);
    gzdbg << "topic /ariac/sensor_enable advertised" << std::endl;
  }

  // Publisher for announcing new orders.
  this->data_ptr->orderPub =
      this->data_ptr->rosNode->advertise<nist_gear::Orders>(ordersTopic, 1000, true);  // latched=true

  // Publisher for announcing new state of the competition.
  this->data_ptr->taskStatePub = this->data_ptr->rosNode->advertise<std_msgs::String>(taskStateTopic, 1000);

  // publisher for announcing the score of the game.
  if (!this->data_ptr->competitionMode)
  {
    this->data_ptr->taskScorePub = this->data_ptr->rosNode->advertise<std_msgs::Float32>(taskScoreTopic, 1000);
  }

  /////////////////////////////
  // Publishers
  /////////////////////////////

  // Publisher for announcing the health of the robots
  this->data_ptr->robot_health_pub =
      this->data_ptr->rosNode->advertise<nist_gear::RobotHealth>(robotHealthTopic, 1000, true);  // latched=true

  // Publisher for storing each model to drop on each agv
  this->data_ptr->drop_object_publisher =
      this->data_ptr->rosNode->advertise<nist_gear::DropProducts>("/ariac/drop_products", 1000, true);  // latched=true

  // Timer for regularly publishing state/score.
  this->data_ptr->statusPubTimer =
      this->data_ptr->rosNode->createTimer(ros::Duration(0.1), &TaskManagerPlugin::PublishStatus, this);

  /////////////////////////////
  // Subscribers
  /////////////////////////////

  // subscriber to robotHealthTopic
  this->data_ptr->robotHealthSubscriber =
      this->data_ptr->rosNode->subscribe(robotHealthTopic, 1000, &TaskManagerPlugin::OnRobotHealthContent, this);

  // subscriber for tray content
  this->data_ptr->kittingShipmentContentSubscriber =
      this->data_ptr->rosNode->subscribe(kittingContentTopic, 1000, &TaskManagerPlugin::OnKittingShipmentContent, this);

  this->data_ptr->gantryPositionSubscriber = this->data_ptr->rosNode->subscribe(
      "/ariac/gantry/world_position", 1000, &TaskManagerPlugin::OnGantryPosition, this);

  // subscriber for briefcase content
  this->data_ptr->assemblyShipmentContentSubscriber = this->data_ptr->rosNode->subscribe(
      assemblyContentTopic, 1000, &TaskManagerPlugin::OnAssemblyShipmentContent, this);

  /////////////////////////////
  // Service Servers
  /////////////////////////////

  // service for ending the competition.
  this->data_ptr->compEndServiceServer = this->data_ptr->rosNode->advertiseService(
      compEndServiceName, &TaskManagerPlugin::EndCompetitionServiceCallback, this);

  // service for changing gripper
  this->data_ptr->gripperChangeServiceServer = this->data_ptr->rosNode->advertiseService(
      gripperChangeService, &TaskManagerPlugin::GripperChangeServiceCallback, this);

  // service for submitting AGV trays for inspection without moving the AGVs
  // this->data_ptr->submitTrayServiceServer =
  //   this->data_ptr->rosNode->advertiseService(submitTrayServiceName,
  //     &TaskManagerPlugin::InspectKittingShipmentServiceCallback, this);

  // service for querying material storage locations.
  if (!this->data_ptr->competitionMode)
  {
    this->data_ptr->getMaterialLocationsServiceServer = this->data_ptr->rosNode->advertiseService(
        getMaterialLocationsServiceName, &TaskManagerPlugin::GetMaterialLocationsServiceCallback, this);
  }

  /////////////////////////////
  // Service Clients
  /////////////////////////////

  for (auto& pair : kit_tray_content_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->data_ptr->kit_tray_content_client[index] =
        this->data_ptr->rosNode->serviceClient<nist_gear::DetectMovableTray>(serviceName);
  }

  // for (auto& pair : get_kit_tray_content_name)
  // {
  //   int index = pair.first;
  //   std::string serviceName = pair.second;
  //   this->data_ptr->get_kit_tray_content_client[index] =
  //       this->data_ptr->rosNode->serviceClient<nist_gear::DetectKitTrayContent>(serviceName);
  // }

  for (auto& pair : station_get_content_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->data_ptr->stationGetContentClient[index] =
        this->data_ptr->rosNode->serviceClient<nist_gear::DetectAssemblyShipment>(serviceName);
  }

  // for (auto& pair : station_get_connected_part_service_name) {
  //   int index = pair.first;
  //   std::string serviceName = pair.second;
  //   this->data_ptr->stationGetConnectedPartClient[index] =
  //     this->data_ptr->rosNode->serviceClient<nist_gear::DetectConnectedPartsToBriefcase>(serviceName);
  // }

  // take care of the services within the <station> tags

  // for (auto& pair : station_get_connected_part_service_name) {
  //   int index = pair.first;
  //   std::string serviceName = pair.second;
  //   this->data_ptr->connected_parts_to_briefcase_service[index] =
  //     this->data_ptr->rosNode->advertiseService<nist_gear::DetectConnectedPartsToBriefcase::Request,
  //     nist_gear::DetectConnectedPartsToBriefcase::Response>(
  //       serviceName,
  //       boost::bind(&TaskManagerPlugin::HandleDetectConnectedParts, this,
  //         _1, _2, index));
  // }

  for (auto& pair : station_submit_shipment_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->data_ptr->station_ship_content_service[index] =
        this->data_ptr->rosNode->advertiseService<nist_gear::AssemblyStationSubmitShipment::Request,
                                                  nist_gear::AssemblyStationSubmitShipment::Response>(
            serviceName, boost::bind(&TaskManagerPlugin::HandleAssemblySubmission, this, _1, _2, index));
  }

  for (auto& pair : submit_kitting_shipment_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->data_ptr->submit_kitting_shipment_server[index] =
        this->data_ptr->rosNode
            ->advertiseService<nist_gear::SubmitKittingShipment::Request, nist_gear::SubmitKittingShipment::Response>(
                serviceName, boost::bind(&TaskManagerPlugin::HandleKittingSubmission, this, _1, _2, index));
  }

  if (!this->data_ptr->competitionMode)
  {
    for (auto& pair : get_kit_tray_content_name)
    {
      int index = pair.first;
      std::string serviceName = pair.second;
      this->data_ptr->get_kitting_shipment_server[index] =
          this->data_ptr->rosNode
              ->advertiseService<nist_gear::DetectKitTrayContent::Request, nist_gear::DetectKitTrayContent::Response>(
                  serviceName, boost::bind(&TaskManagerPlugin::HandleGetKitTrayContent, this, _1, _2, index));
    }
  }

  for (auto& pair : move_to_station_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->data_ptr->move_to_station_server[index] =
        this->data_ptr->rosNode
            ->advertiseService<nist_gear::MoveToStation::Request, nist_gear::MoveToStation::Response>(
                serviceName, boost::bind(&TaskManagerPlugin::HandleMoveToStationCallback, this, _1, _2, index));
  }

  /// AS1
  for (auto& pair : ks_to_as1_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->ks_to_as1_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as1_to_ks_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as1_to_ks_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as1_to_as2_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as1_to_as2_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  /// AS2
  for (auto& pair : ks_to_as2_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->ks_to_as2_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as2_to_ks_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as2_to_ks_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as2_to_as1_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as2_to_as1_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  /// AS3
  for (auto& pair : ks_to_as3_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->ks_to_as3_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as3_to_ks_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as3_to_ks_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as3_to_as4_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as3_to_as4_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  /// AS4
  for (auto& pair : ks_to_as4_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->ks_to_as4_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as4_to_ks_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as4_to_ks_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto& pair : as4_to_as3_AnimationServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->data_ptr->as4_to_as3_AnimateClient[index] =
          this->data_ptr->rosNode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  this->data_ptr->conveyorControlClient =
      this->data_ptr->rosNode->serviceClient<nist_gear::ConveyorBeltControl>(conveyorControlService);

  // Controller manager service to switch controllers for the gantry robot.
  // this->data_ptr->gantry_controller_manager_client =
  //     this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(conveyorControlService);
  // // Controller manager service to switch controllers for the kitting robot.
  // this->data_ptr->kitting_controller_manager_client =
  //     this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(conveyorControlService);

  /////////////////////////////////

  this->data_ptr->serverControlPub = this->data_ptr->gz_node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  this->data_ptr->connection = event::Events::ConnectWorldUpdateEnd(boost::bind(&TaskManagerPlugin::OnUpdate, this));

  // // controller manager

  // // gantry
  // std::string gantry_controller_service = "/ariac/gantry/controller_manager/switch_controller";
  // this->data_ptr->gantry_controller_manager_client =
  //     this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(gantry_controller_service);

  // // kitting
  // std::string kitting_controller_service = "/ariac/kitting/controller_manager/switch_controller";
  // this->data_ptr->kitting_controller_manager_client =
  //     this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(kitting_controller_service);

}  // end Load()

void TaskManagerPlugin::OnAGV1Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->agv1_current_station = _msg->data;
}

void TaskManagerPlugin::OnAGV2Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->agv2_current_station = _msg->data;
}

void TaskManagerPlugin::OnAGV3Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->agv3_current_station = _msg->data;
}

void TaskManagerPlugin::OnAGV4Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->agv4_current_station = _msg->data;
}


/////////////////////////////////////////////////
void TaskManagerPlugin::OnUpdate()
{
  // gzerr << "onupdate" << std::endl;
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  auto current_sime_time = this->data_ptr->world->SimTime();

  if (this->data_ptr->agv1_current_station == "as2" || this->data_ptr->agv2_current_station == "as2")
  {
    std_msgs::Bool msg_as;
    msg_as.data = true;
    // gzwarn << "Publishing station for agv" << std::endl;
    this->data_ptr->as2StatusPublisher.publish(msg_as);
  }

  if (this->data_ptr->agv4_current_station == "as4" || this->data_ptr->agv3_current_station == "as4")
  {
    std_msgs::Bool msg_as;
    msg_as.data = true;

    this->data_ptr->as4StatusPublisher.publish(msg_as);
  }

  // Delay advertising the competition start service to avoid a crash.
  // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
  // See https://bitbucket.org/osrf/ariac/issues/91
  if (!this->data_ptr->compStartServiceServer && current_sime_time.Double() >= 5.0)
  {
    // Service for starting the competition.
    this->data_ptr->compStartServiceServer = this->data_ptr->rosNode->advertiseService(
        this->data_ptr->compStartServiceName, &TaskManagerPlugin::StartCompetitionServiceCallback, this);
  }

  if ((current_sime_time - this->data_ptr->lastSimTimePublish).Double() >= 1.0)
  {
    gzdbg << "Sim time: " << current_sime_time.Double() << std::endl;
    this->data_ptr->lastSimTimePublish = current_sime_time;
  }

  double elapsedTime = (current_sime_time - this->data_ptr->lastUpdateTime).Double();
  if (this->data_ptr->timeLimit >= 0 && this->data_ptr->currentState == "go" &&
      (current_sime_time - this->data_ptr->gameStartTime) > this->data_ptr->timeLimit)
  {
    this->data_ptr->currentState = "end_game";
  }

  if (this->data_ptr->currentState == "ready")
  {
    // gzdbg << "ready\n";
    this->data_ptr->gameStartTime = current_sime_time;
    this->data_ptr->currentState = "go";

    // start the conveyor belt
    if (this->data_ptr->isBeltNeeded)
    {
      // gzwarn << "Enabling belt" << std::endl;
      this->EnableConveyorBeltControl();
      this->PopulateConveyorBelt();
    }
  }

  /////////////////////////////////////////
  // GO
  if (this->data_ptr->currentState == "go")
  {
    // gzwarn << "GO" << std::endl;
    // Update the order manager.
    this->ProcessOrdersToAnnounce(current_sime_time);
    
    // gzwarn << "Done processing order" << std::endl;
    // Check if we need to disable any robot
    // NOTE: This function must be placed exactly after ProcessOrdersToAnnounce
    this->ProcessRobotStatus();
    // gzwarn << "Done processing robot status" << std::endl;

    // Update the sensors if appropriate.
    this->ProcessSensorBlackout();
    // gzwarn << "Done processing sensor blackout" << std::endl;

    // Update the score.
    // TODO(sloretz) only publish this when an event that could change the score happens
    ariac::GameScore gameScore = this->data_ptr->ariac_scorer.GetGameScore(this->data_ptr->floorPenalty);
    // gzwarn << "Done getting game score" << std::endl;
    // gzmsg << "gameScore.total(): " << gameScore.total() << std::endl;
    // gzmsg << "current_trial_score.total(): " << this->data_ptr->current_trial_score.total() << std::endl;

    if (gameScore.total() != this->data_ptr->current_trial_score.total())
    {
      std::ostringstream logMessage;
      logMessage << "Current game score: " << gameScore.total();
      ROS_DEBUG_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      this->data_ptr->current_trial_score = gameScore;
    }
    
    

    if (!this->data_ptr->ordersInProgress.empty())
    {
      // gzwarn << "-- Order in progress" << std::endl;
      this->data_ptr->ordersInProgress.top().time_taken += elapsedTime;
      auto orderID = this->data_ptr->ordersInProgress.top().order_id;
      // gzwarn << "Order: " << orderID << std::endl;
      // TODO: timing should probably be managed by the scorer but we want to use sim time
      this->data_ptr->timeSpentOnCurrentOrder = this->data_ptr->ordersInProgress.top().time_taken;

      auto has_kitting_shipments = this->data_ptr->ordersInProgress.top().has_kitting_task;
      auto has_assembly_shipments = this->data_ptr->ordersInProgress.top().has_assembly_task;

      // if the order has kitting shipments, check if all kits have been submitted
      if (has_kitting_shipments && !has_assembly_shipments)
      {
        // gzerr << "kitting only" << std::endl;
        bool all_kitting_shipments_submitted = gameScore.order_scores_map.at(orderID).isKittingComplete();
        // gzerr << "checked kitting submission" << std::endl;
        if (all_kitting_shipments_submitted)
        {
          // gzerr << "kitting complete" << std::endl;
          if (this->data_ptr->gameStartTime != common::Time())
          {
            this->data_ptr->current_trial_score.total_process_time =
                (current_sime_time - this->data_ptr->gameStartTime).Double();
          }

          std::ostringstream logMessage;
          logMessage << "All kitting shipments submitted for order: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
      if (!has_kitting_shipments && has_assembly_shipments)
      {
        // gzerr << "assembly only" << std::endl;
        // gzdbg << "Only Assembly\n";
        bool all_assembly_shipments_submitted = gameScore.order_scores_map.at(orderID).isAssemblyComplete();
        if (all_assembly_shipments_submitted)
        {
          std::ostringstream logMessage;
          logMessage << "All assembly shipments submitted for order: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
      if (has_kitting_shipments && has_assembly_shipments)
      {
        // gzerr << "Assembly and Kitting" << std::endl;
        bool all_kitting_shipments_submitted = gameScore.order_scores_map.at(orderID).isKittingComplete();
        // if (all_kitting_shipments_submitted)
        // {
        //   gzerr << "All kitting shipments submitted" << std::endl;
        // }

        bool all_assembly_shipments_submitted = gameScore.order_scores_map.at(orderID).isAssemblyComplete();
        // if (all_assembly_shipments_submitted)
        // {
        //   gzerr << "All assembly shipments submitted" << std::endl;
        // }
        // if (all_kitting_shipments_submitted)
        // {
        //   std::ostringstream logMessageKitting;
        //   logMessageKitting << "All kitting shipments submitted for current order: " <<
        //   this->data_ptr->current_trial_score.total() << "\nScore breakdown:\n"
        //                     << this->data_ptr->current_trial_score;
        //   ROS_INFO_STREAM(logMessageKitting.str().c_str());
        //   gzdbg << logMessageKitting.str() << std::endl;
        // }

        if (all_assembly_shipments_submitted && all_kitting_shipments_submitted)
        {
          std::ostringstream logMessage;
          logMessage << "All kitting and assembly shipments submitted for order: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
      // else
      // {
        // Check if the time limit for the current order has been exceeded.
      // allowedTime
      // gzerr << "Time spent on current order: " << this->data_ptr->timeSpentOnCurrentOrder << std::endl;
      // gzerr << "Allowed time: " << this->data_ptr->ordersInProgress.top().allowed_time << std::endl;
      // if (this->data_ptr->timeSpentOnCurrentOrder > this->data_ptr->ordersInProgress.top().allowed_time)
      //   {
      //     std::ostringstream logMessage;
      //     logMessage << "Order timed out: " << orderID;
      //     ROS_INFO_STREAM(logMessage.str().c_str());
      //     gzdbg << logMessage.str() << std::endl;
      //     this->StopCurrentOrder();
      //   }
      // }
    }

    // gzwarn << "2" << std::endl;

    if (this->data_ptr->ordersInProgress.empty() && this->data_ptr->orders_to_announce.empty())
    {
      gzdbg << "No more orders to process." << std::endl;
      this->data_ptr->currentState = "end_game";
    }
  }

  if (this->data_ptr->currentState == "end_game")
  {
    // gzwarn << "END" << std::endl;
    // gzdbg << "END" << std::endl;
    // todo(zeid): Apply this-dataPtr->floorPenalty to GameScore calculation
    this->data_ptr->current_trial_score = this->data_ptr->ariac_scorer.GetGameScore(this->data_ptr->floorPenalty);
    if (this->data_ptr->gameStartTime != common::Time())
    {
      this->data_ptr->current_trial_score.total_process_time =
          (current_sime_time - this->data_ptr->gameStartTime).Double();
    }
    std::ostringstream logMessage;
    logMessage << "\n------------------------" << std::endl;
    logMessage << "Final Score: " << this->data_ptr->current_trial_score.total() << std::endl;
    logMessage << "Configuration File: " << this->data_ptr->config_yaml_file << std::endl;
    logMessage << "------------------------" << std::endl;
    logMessage << this->data_ptr->current_trial_score;
    // logMessage << "End of trial. Final score: " << this->data_ptr->current_trial_score.total() << "\nScore
    // breakdown:\n"
    //   << this->data_ptr->current_trial_score;
    ROS_INFO_STREAM(logMessage.str().c_str());
    gzdbg << logMessage.str() << std::endl;
    this->data_ptr->currentState = "done";

    bool is_first = true;
    std::stringstream sstr;
    for (const auto order_tuple : this->data_ptr->current_trial_score.order_scores_map)
    {
      sstr << order_tuple.second.csv_kitting(is_first).c_str();
      sstr << order_tuple.second.csv_assembly(is_first).c_str();
      is_first = false;
    }
    ROS_INFO_STREAM(sstr.str().c_str());
  }

  if (this->data_ptr->currentState == "done") {
    // gzwarn << "status DONE" << std::endl;
    auto v = std::getenv("ARIAC_EXIT_ON_COMPLETION");
    
    if (v)
    {
      // Gazebo will accumulate a number of state loggings before writing them to file.
      // Request that the accumulated states are written now before we shutdown,
      // otherwise the end of the state log may be cut off.
      util::LogRecord::Instance()->Notify();
      std::this_thread::sleep_for(std::chrono::seconds(2));

      std::string logMessage = "Requesting that Gazebo shut down";
      gzmsg << logMessage << std::endl;
      ROS_INFO("%s", logMessage.c_str());
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->data_ptr->serverControlPub->Publish(msg);
    }
  }

  this->data_ptr->lastUpdateTime = current_sime_time;
}

/////////////////////////////////////////////////
void TaskManagerPlugin::PublishStatus(const ros::TimerEvent&)
{
  std_msgs::Float32 scoreMsg;
  scoreMsg.data = this->data_ptr->current_trial_score.total();
  if (!this->data_ptr->competitionMode)
  {
    this->data_ptr->taskScorePub.publish(scoreMsg);
  }

  std_msgs::String stateMsg;
  stateMsg.data = this->data_ptr->currentState;
  this->data_ptr->taskStatePub.publish(stateMsg);
}

// helper function to extract the ID of an AGV
int GetAgvID(std::string agv_name)
{
  auto agv_id = agv_name.back();
  return int(agv_id);
}

///////////////////////////////////////////////////////
void TaskManagerPlugin::OnGantryPosition(nist_gear::GantryPosition::ConstPtr gantry_position)
{
  this->data_ptr->gantry_world_x = gantry_position->gantry_world_x;
  this->data_ptr->gantry_world_y = gantry_position->gantry_world_y;
}

/////////////////////////////////////////////////
// void TaskManagerPlugin::GetStaticControllers()
// {
//   // staic controllers for the kitting robot
//   std::string kitting_srv_name = "/ariac/kitting/controller_manager/list_controllers";
//   ros::ServiceClient client =
//       this->data_ptr->rosNode->serviceClient<controller_manager_msgs::ListControllers>(kitting_srv_name);
//   controller_manager_msgs::ListControllers kitting_srv;

//   if (client.call(kitting_srv))
//   {
//     for (auto state : kitting_srv.response.controller)
//     {
//       if (state.name.rfind("static", 0) == 0)
//       {
//         this->data_ptr->kitting_static_controllers.push_back(state.name);
//         // ROS_INFO_STREAM("Retrieved static controllers for the kitting robot");
//         // gzerr << "Static: " << state.name << std::endl;;
//       }
//     }
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Unable to call " << kitting_srv_name);
//   }

//   // static controllers for the gantry robot
//   std::string gantry_srv_name = "/ariac/gantry/controller_manager/list_controllers";
//   client = this->data_ptr->rosNode->serviceClient<controller_manager_msgs::ListControllers>(gantry_srv_name);
//   controller_manager_msgs::ListControllers gantry_srv;

//   if (client.call(gantry_srv))
//   {
//     for (auto state : gantry_srv.response.controller)
//     {
//       if (state.name.rfind("static", 0) == 0)
//       {
//         this->data_ptr->gantry_static_controllers.push_back(state.name);
//         // ROS_INFO_STREAM("Retrieved static controllers for the gantry robot");
//       }
//     }
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Unable to call " << gantry_srv_name);
//   }
// }

/////////////////////////////////////////////////
// void TaskManagerPlugin::StopRobot(std::string robot_name)
// {
//   controller_manager_msgs::SwitchController srv;
//   std::vector<std::string> static_controllers;
//   if (robot_name.compare("kitting") == 0)
//   {
//     ROS_INFO_STREAM("Disabling kitting robot");
//     static_controllers = this->data_ptr->kitting_static_controllers;
//   }
//   else if (robot_name.compare("gantry") == 0)
//   {
//     ROS_INFO_STREAM("Disabling gantry robot");
//     static_controllers = this->data_ptr->gantry_static_controllers;
//   }
//   srv.request.start_controllers = static_controllers;
//   srv.request.stop_controllers.push_back(robot_name + "_arm_controller");

//   std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
//   ros::ServiceClient client =
//       this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(srv_name);

//   srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;

//   if (client.call(srv))
//   {
//     if (srv.response.ok)
//     {
//       ROS_INFO_STREAM("Stopped " << robot_name << " controller");
//     }
//     else
//     {
//       ROS_ERROR_STREAM("Unable to stop controller");
//     }
//   }
// }

// /////////////////////////////////////////////////
// void TaskManagerPlugin::StartRobot(std::string robot_name)
// {
//   controller_manager_msgs::SwitchController srv;
//   std::vector<std::string> static_controllers;
//   if (robot_name.compare("kitting") == 0)
//   {
//     static_controllers = this->data_ptr->kitting_static_controllers;
//   }
//   else if (robot_name.compare("gantry") == 0)
//   {
//     static_controllers = this->data_ptr->gantry_static_controllers;
//   }
//   srv.request.start_controllers.push_back(robot_name + "_arm_controller");

//   std::string srv_name = "/ariac/" + robot_name + "/controller_manager/switch_controller";
//   ros::ServiceClient client =
//       this->data_ptr->rosNode->serviceClient<controller_manager_msgs::SwitchController>(srv_name);

//   srv.request.strictness = 1;

//   if (client.call(srv))
//   {
//     if (srv.response.ok)
//     {
//       ROS_INFO_STREAM("Started " << robot_name << " controller");
//     }
//     else
//     {
//       ROS_ERROR_STREAM("Unable to start controller");
//     }
//   }
// }

/////////////////////////////////////////////////
void TaskManagerPlugin::ProcessRobotStatus()
{
  // std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  // auto order_in_progress = this->data_ptr->ordersInProgress.top();
  // get the disable condition for the current order
  // auto robot_disable_condition = order_in_progress.robot_disable_condition;
  // check whether this order requires a robot to be disabled
  if (!this->data_ptr->ordersInProgress.top().robot_disable_condition.is_order_with_disabled_robot)
  {
    return;
  }
  auto robot_disable_condition = this->data_ptr->ordersInProgress.top().robot_disable_condition;

  // compare the number of parts located in the kit
  // the content of the kit is published on /ariac/trays
  for (auto& cpair : this->data_ptr->kitting_shipment_contents)
  {
    // get the id of the kit tray, e.g., grab 1 from kit_tray_1
    unsigned int kit_tray_id = cpair.second->kit_tray.back() - '0';
    // get the correct kit tray as in disable_condition
    if (kit_tray_id == robot_disable_condition.disable_condition)
    {
      // get the number of products placed in the kit
      unsigned int product_count = cpair.second->products.size();
      // if the number of products in the kit is the same as disable_value
      // then disable a robot
      if (product_count > 0)
      {
        if (robot_disable_condition.disable_value == product_count)
        {
          nist_gear::RobotHealth robot_health_msg;
          if (robot_disable_condition.robot_type.compare("kitting_robot") == 0)
          {
            // ROS_ERROR_STREAM("Disable Kitting");
            // set this flag to false so this function is not processed in the next iteration
            // robot_disable_condition.is_order_with_disabled_robot = false;
            this->data_ptr->ordersInProgress.top().robot_disable_condition.is_order_with_disabled_robot = false;
            // publish the robots' health status
            robot_health_msg.kitting_robot_health = "inactive";
            robot_health_msg.assembly_robot_health = "active";
            this->data_ptr->robot_health_pub.publish(robot_health_msg);
          }
          else if (robot_disable_condition.robot_type.compare("assembly_robot") == 0)
          {
            // ROS_ERROR_STREAM("Disable Gantry");
            // set this flag to false so this function is not processed in the next iteration
            this->data_ptr->ordersInProgress.top().robot_disable_condition.is_order_with_disabled_robot = false;
            // publish the robots' health status
            robot_health_msg.kitting_robot_health = "active";
            robot_health_msg.assembly_robot_health = "inactive";
            this->data_ptr->robot_health_pub.publish(robot_health_msg);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::ProcessOrdersToAnnounce(gazebo::common::Time simTime)
{
  if (this->data_ptr->orders_to_announce.empty())
    return;

  // get the first order to announce
  auto order_to_announce = this->data_ptr->orders_to_announce.front();
  // this is used to process the robots' health status
  this->data_ptr->currentOrder = this->data_ptr->orders_to_announce.front();

  auto new_order = order_to_announce.new_order_announcement;
  bool has_unwanted_products = false;
  bool has_wanted_products = false;
  bool has_on_kitting_submission = false;
  bool has_on_assembly_submission = false;
  int nb_of_parts_unwanted = -1;
  int nb_of_parts_wanted = -1;

  if (new_order.get_type().compare("unwanted_products") == 0)
  {
    // gzerr << "Has Unwanted Products" << std::endl;
    has_unwanted_products = true;
    nb_of_parts_unwanted = new_order.get_part_number();
  }

  else if (new_order.get_type().compare("wanted_products") == 0)
  {
    has_wanted_products = true;
    nb_of_parts_wanted = new_order.get_part_number();
  }

  else if (new_order.get_type().compare("on_kitting_submission") == 0)
    has_on_kitting_submission = true;
  else if (new_order.get_type().compare("on_assembly_submission") == 0)
    has_on_assembly_submission = true;

  // bool has_wanted_products = order_to_announce.interrupt_on_wanted_products > 0;
  // bool interruptOnStationReached = !order_to_announce.interrupt_on_station_reached.empty();
  bool no_active_order = this->data_ptr->ordersInProgress.empty();
  auto elapsed = this->data_ptr->world->SimTime() - this->data_ptr->gameStartTime;
  bool announce_next_order = false;

  // Check whether announce a new order from the list.
  // Announce an order if the appropriate amount of time has elapsed
  announce_next_order |= elapsed.Double() >= order_to_announce.start_time;
  // gzdbg << "announceNextOrder time: " << announceNextOrder << std::endl;
  // Announce next order if there is no active order and we are waiting to interrupt
  announce_next_order |= no_active_order && (has_wanted_products || has_unwanted_products);
  // gzdbg << "announceNextOrder condition: " << announceNextOrder << std::endl;
  // Check if it's time to interrupt (skip if we're already interrupting anyway)
  if (!announce_next_order &&
      (has_wanted_products || has_unwanted_products || has_on_kitting_submission || has_on_assembly_submission))
  {
    // if (has_wanted_products) {
    //   gzdbg << "has_wanted_products\n";
    // }
    // if (has_unwanted_products) {
    //   // gzdbg << "has_unwanted_products\n";
    // }
    // if (has_on_kitting_submission) {
    //   gzdbg << "has_on_kitting_submission\n";
    // }
    // if (has_on_assembly_submission) {
    //   gzdbg << "has_on_assembly_submission\n";
    // }
    /**
     * ******************************************
     * Dealing with has_on_kitting_submission
     * announcement_condition: kitting_submission
     * announcement_condition_value: [agv1, as1]
     * ******************************************
     */
    if (has_on_kitting_submission)
    {
      // let's check if a kitting shipment has been submitted
      // we know if a kitting shipment was submitted by checking the following variables
      std::string actual_station_name = this->data_ptr->submitted_kitting_shipment_station;
      // TODO: reset this->data_ptr->submitted_kitting_shipment_station
      // this->data_ptr->submitted_kitting_shipment_station = "";
      int actual_agv_id = this->data_ptr->submitted_shipment_agv;
      // TODO: reset this->data_ptr->submitted_shipment_agv
      // this->data_ptr->submitted_shipment_agv = 0;
      std::string actual_agv_name = "agv" + std::to_string(actual_agv_id);

      if (!actual_station_name.empty() && actual_agv_id > 0)
      {
        auto expected_agv_name = order_to_announce.new_order_announcement.get_agv();
        auto expected_station_name = order_to_announce.new_order_announcement.get_station();
        // if correct agv sent to correct station then announce the next order
        if (actual_agv_name.compare(expected_agv_name) == 0)
        {
          if (actual_station_name.compare(expected_station_name) == 0)
          {
            announce_next_order = true;
          }
        }
      }
    }

    /**
     * ******************************************
     * Dealing with has_on_assembly_submission
     * announcement_condition: assembly_submission
     * announcement_condition_value: [as1]
     * ******************************************
     * Announce a non-priority order when an assembly is submitted
     */
    if (has_on_assembly_submission)
    {
      // check an assembly was submitted by checking the following variable
      std::string actual_station_name = this->data_ptr->submitted_assembly_shipment_station;
      if (actual_station_name != "")
      {
        auto expected_station_name = order_to_announce.new_order_announcement.get_station();
        // if correct AS submitted
        if (actual_station_name.compare(expected_station_name) == 0)
        {
          announce_next_order = true;
        }
      }
    }
    /**
     * *****************************************************************************************
     * Dealing with wanted and unwanted products only if the second order has kitting shipments
     * *****************************************************************************************
     */
    // Check if the products in the shipping boxes are enough to interrupt the current order
    // What to interrupt:
    // kitting -> kitting
    // assembly -> kitting
    // kitting -> assembly: Make sure there is an AGV with parts at the AS
    // assembly -> assembly: Make sure there is an AGV with parts at the AS

    // kitting -> kitting
    std::vector<std::string> products_in_order_to_announce;
    int max_num_wanted_products = 0;
    int max_num_unwanted_products = 0;

    // next order has kitting
    if (order_to_announce.has_kitting_task)
    {
      products_in_order_to_announce.clear();
      for (const auto& shipment : order_to_announce.kitting_shipments)
      {
        for (const auto& product : shipment.products)
        {
          products_in_order_to_announce.push_back(product.type);
        }
      }
    }

    // next order has assembly
    if (order_to_announce.has_assembly_task)
    {
      products_in_order_to_announce.clear();
      for (const auto& shipment : order_to_announce.assembly_shipments)
      {
        for (const auto& product : shipment.products)
        {
          products_in_order_to_announce.push_back(product.type);
        }
      }
    }

    // Check the number of parts in the current briefcase
    for (auto& cpair : this->data_ptr->assembly_shipment_contents)
    {
      std::vector<std::string> products_in_order_to_announce_copy(products_in_order_to_announce);
      int num_wanted_products = 0;
      int num_unwanted_products = 0;
      if (cpair.second->products.size() > 0)
      {
        for (const auto& product : cpair.second->products)
        {
          // Don't count faulty products, because they have to be removed anyway.
          if (!product.is_faulty)
          {
            auto it = std::find(products_in_order_to_announce_copy.begin(), products_in_order_to_announce_copy.end(),
                                product.type);
            if (it == products_in_order_to_announce_copy.end())
              ++num_unwanted_products;
            else
            {
              ++num_wanted_products;
              products_in_order_to_announce_copy.erase(it);
            }
          }
        }
      }
      if (num_wanted_products > max_num_wanted_products)
        max_num_wanted_products = num_wanted_products;

      if (num_unwanted_products > max_num_unwanted_products)
        max_num_unwanted_products = num_unwanted_products;
    }

    // Check the number of parts in the current kit
    // get the content of a movable tray, which is constently published
    for (auto& cpair : this->data_ptr->kitting_shipment_contents)
    {
      std::vector<std::string> products_in_order_to_announce_copy(products_in_order_to_announce);
      int num_wanted_products = 0;
      int num_unwanted_products = 0;
      // gzerr << "Checking Current Tray" << std::endl;
      if (cpair.second->products.size() > 0)
      {
        for (const auto& product : cpair.second->products)
        {
          // gzerr << "PRODUCT In TRAY: " << product.type << std::endl;
          // Don't count faulty products, because they have to be removed anyway.
          if (!product.is_faulty)
          {
            auto it = std::find(products_in_order_to_announce_copy.begin(), products_in_order_to_announce_copy.end(),
                                product.type);
            if (it == products_in_order_to_announce_copy.end())
              ++num_unwanted_products;
            else
            {
              // gzerr << "FOUND NEEDED PRODUCT: " << product.type << std::endl;
              ++num_wanted_products;
              products_in_order_to_announce_copy.erase(it);
            }
          }
        }
      }
      if (num_wanted_products > max_num_wanted_products)
        max_num_wanted_products = num_wanted_products;

      if (num_unwanted_products > max_num_unwanted_products)
        max_num_unwanted_products = num_unwanted_products;
    }
    // Announce next order if a tray has more than enough wanted or unwanted products
    announce_next_order |= has_wanted_products && (max_num_wanted_products >= nb_of_parts_wanted);
    announce_next_order |= has_unwanted_products && (max_num_unwanted_products >= nb_of_parts_unwanted);
  }

  if (announce_next_order)
  {
    // gzdbg << "announceNextOrder\n";
    auto updateLocn = order_to_announce.order_id.find("_update");
    if (updateLocn != std::string::npos)
    {
      gzdbg << "Order to update: " << order_to_announce.order_id << std::endl;
      this->AnnounceOrder(order_to_announce);

      // Update the order the scorer's monitoring
      // gzdbg << "Updating order: " << order_to_announce << std::endl;
      auto nextOrderID = order_to_announce.order_id.substr(0, updateLocn);
      nist_gear::Order orderMsg;
      fill_order_message(order_to_announce, orderMsg);
      this->data_ptr->ariac_scorer.NotifyOrderUpdated(simTime, nextOrderID, orderMsg);
      this->data_ptr->orders_to_announce.erase(this->data_ptr->orders_to_announce.begin());
      return;
    }

    // gzdbg << "New order to announce: " << order_to_announce.order_id << std::endl;

    // Move order to the 'in process' stack
    this->data_ptr->ordersInProgress.push(ariac::Order(order_to_announce));
    this->data_ptr->orders_to_announce.erase(this->data_ptr->orders_to_announce.begin());

    this->AnnounceOrder(order_to_announce);
    // Assign the scorer the order to monitor
    // gzdbg << "Assigning order: \n"
    //   << order_to_announce << std::endl;
    nist_gear::Order orderMsg;
    fill_order_message(order_to_announce, orderMsg);

    this->data_ptr->ariac_scorer.NotifyOrderStarted(simTime, orderMsg, order_to_announce.priority);

    // reset those attributes
    this->data_ptr->submitted_kitting_shipment_station = "";
    this->data_ptr->submitted_shipment_agv = 0;
    this->data_ptr->submitted_assembly_shipment_station = "";
  }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::ProcessSensorBlackout()
{
  auto current_sime_time = this->data_ptr->world->SimTime();
  if (this->data_ptr->sensor_blackout_product_count > 0)
  {
    // count total products in all boxes.
    int total_products = 0;
    // get the number of products detected in kitting
    for (auto& cpair : this->data_ptr->kitting_shipment_contents)
    {
      total_products += cpair.second->products.size();
    }
    // get the number of products detected in assembly
    for (auto& cpair : this->data_ptr->assembly_shipment_contents)
    {
      total_products += cpair.second->products.size();
    }

    if (total_products >= this->data_ptr->sensor_blackout_product_count)
    {
      std::ostringstream logMessage;
      logMessage << std::endl;
      logMessage << "----------------------------" << std::endl;
      logMessage << " Sensor Blackout Triggered" << std::endl;
      logMessage << "----------------------------" << std::endl;
      ROS_INFO_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;

      gazebo::msgs::GzString message;
      message.set_data("deactivate");
      this->data_ptr->sensor_blackout_control_publisher->Publish(message);
      this->data_ptr->sensor_blackout_product_count = -1;
      this->data_ptr->sensor_blackout_start_time = current_sime_time;
      this->data_ptr->is_sensor_blackout_in_progress = true;
    }
  }
  if (this->data_ptr->is_sensor_blackout_in_progress)
  {
    auto elapsedTime = (current_sime_time - this->data_ptr->sensor_blackout_start_time).Double();
    if (elapsedTime > this->data_ptr->sensor_blackout_duration)
    {
      std::ostringstream logMessage;
      logMessage << std::endl;
      logMessage << "------------------------" << std::endl;
      logMessage << " Sensor Blackout Ended" << std::endl;
      logMessage << "------------------------" << std::endl;
      ROS_INFO_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      gazebo::msgs::GzString message;
      message.set_data("activate");
      this->data_ptr->sensor_blackout_control_publisher->Publish(message);
      this->data_ptr->is_sensor_blackout_in_progress = false;
    }
  }
}

/////////////////////////////////////////////////
bool TaskManagerPlugin::StartCompetitionServiceCallback(std_srvs::Trigger::Request& req,
                                                        std_srvs::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

  gzdbg << "\n";
  gzdbg << "[Service Call] Start Competition\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";

  // ROS_INFO_STREAM("Get static controllers");
  // GetStaticControllers();

  (void)req;

  if (this->data_ptr->currentState == "init")
  {
    this->data_ptr->currentState = "ready";
    res.success = true;
    res.message = " -- Competition started successfully! --";
    return true;
  }
  res.success = false;
  res.message = "-- ERROR: Cannot start competition if current state is not 'init'";
  return true;
}

bool TaskManagerPlugin::GripperChangeServiceCallback(nist_gear::ChangeGripper::Request& req,
                                                     nist_gear::ChangeGripper::Response& res)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  gzdbg << "\n";
  gzdbg << "[Service Call] Gripper Change\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";

  // ROS_INFO_STREAM(req.gripper_type);

  // first make sure the requested gripper type is valid
  if (req.gripper_type != this->data_ptr->gripper_tray_type && req.gripper_type != this->data_ptr->gripper_part_type)
  {
    res.success = false;
    res.message = "[GripperChangeServiceCallback] ERROR: Wrong gripper type provided";
    return true;
  }

  // auto gantry_position_msg =
  // ros::topic::waitForMessage<nist_gear::GantryPosition>("/ariac/gantry/world_position");

  // check the robot pose first to make sure it is above
  // the gripper changing station
  // gazebo_msgs::GetLinkState linkState;
  // std::string linkName = (std::string) "gantry::torso_base";
  // std::string referenceFrame = (std::string) "world";

  // ros::ServiceClient client =
  //     this->data_ptr->rosNode->serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  // linkState.request.link_name = linkName;
  // linkState.request.reference_frame = referenceFrame;
  // client.call(linkState);

  // auto gantry_world_x = gantry_position_msg->gantry_world_x;
  // auto gantry_world_y = gantry_position_msg->gantry_world_y;
  // geometry_msgs::Point pp = linkState.response.link_state.pose.position;
  // gzerr << "robot position: " << pp.x << "," << pp.y << "\n";

  if ((this->data_ptr->gripper_changing_station_lower_x <= this->data_ptr->gantry_world_x) &&
      (this->data_ptr->gantry_world_x <= this->data_ptr->gripper_changing_station_higher_x) &&
      (this->data_ptr->gantry_world_y <= this->data_ptr->gripper_changing_station_higher_y) &&
      (this->data_ptr->gripper_changing_station_lower_y <= this->data_ptr->gantry_world_y))
  {
    ROS_INFO_STREAM("Gantry is at gripper changing station.");
  }
  else
  {
    res.success = false;
    res.message = "[GripperChangeServiceCallback] ERROR: Gantry is not properly located above gripper station";
    return true;
  }

  // finally, set the parameter using the Request
  std_msgs::String gripper_msg;
  gripper_msg.data = req.gripper_type;
  this->data_ptr->gantry_gripper_type_publisher.publish(gripper_msg);
  res.success = true;
  res.message = "New gripper attached to gantry: " + req.gripper_type;
  return true;
}

/////////////////////////////////////////////////
bool TaskManagerPlugin::EndCompetitionServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  gzdbg << "\n";
  gzdbg << "Handle end service called\n";
  gzdbg << "[Service Call] End Competition\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";
  (void)req;

  this->data_ptr->currentState = "end_game";
  res.success = true;
  res.message = "-- Competition ended successfully! --";
  return true;
}

/////////////////////////////////////////////////
bool TaskManagerPlugin::GetMaterialLocationsServiceCallback(nist_gear::GetMaterialLocations::Request& req,
                                                            nist_gear::GetMaterialLocations::Response& res)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  gzdbg << "\n";
  gzdbg << "[Service Call] Get Material Location\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";
  auto it = this->data_ptr->materialLocations.find(req.material_type);
  if (it == this->data_ptr->materialLocations.end())
  {
    gzdbg << "No known locations for material type: " << req.material_type << std::endl;
  }
  else
  {
    auto locations = it->second;
    for (auto storage_unit : locations)
    {
      nist_gear::StorageUnit storageUnitMsg;
      storageUnitMsg.unit_id = storage_unit;
      res.storage_units.push_back(storageUnitMsg);
    }
  }
  return true;
}

////////////////////////////////////////////////
bool TaskManagerPlugin::HandleMoveToStationCallback(nist_gear::MoveToStation::Request& req,
                                                    nist_gear::MoveToStation::Response& res, int agv_id)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

  if (this->data_ptr->currentState != "go")
  {
    std::string errStr = "Competition is not running so this service will not work.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  // concatenate string and agv id
  std::string agv_name = "agv";
  agv_name = agv_name + std::to_string(agv_id);

  std::string parameter = "/ariac/" + agv_name + "/current_station";
  std::string current_station{};
  if (this->data_ptr->rosNode->getParam(parameter, current_station))
  {
    gzdbg << "[Current Station] " << current_station << std::endl;
  }

  gzdbg << "\n";
  gzdbg << "[Service Call] Move To Station\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";

  // get the correct service client based on req.assembly_station_name
  ros::ServiceClient animate_client;

  // ks1 -> as1
  // ks1 -> as2
  // ks2 -> as1
  // ks2 -> as2
  if (current_station.compare("ks1") == 0 || current_station.compare("ks2") == 0)
  {
    if (req.station_name.compare("as1") == 0)
    {
      animate_client = this->data_ptr->ks_to_as1_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("as2") == 0)
    {
      animate_client = this->data_ptr->ks_to_as2_AnimateClient.at(agv_id);
    }
  }
  // as1 -> as2
  // as1 -> ks1
  // as1 -> ks2
  else if (current_station.compare("as1") == 0)
  {
    if (req.station_name.compare("as2") == 0)
    {
      animate_client = this->data_ptr->as1_to_as2_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("ks") == 0)
    {
      animate_client = this->data_ptr->as1_to_ks_AnimateClient.at(agv_id);
    }
  }
  // as2 -> as1
  // as2 -> ks1
  // as2 -> ks2
  else if (current_station.compare("as2") == 0)
  {
    if (req.station_name.compare("as1") == 0)
    {
      animate_client = this->data_ptr->as2_to_as1_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("ks") == 0)
    {
      animate_client = this->data_ptr->as2_to_ks_AnimateClient.at(agv_id);
    }
  }
  // ks3 -> as3
  // ks3 -> as4
  // ks4 -> as3
  // ks4 -> as4
  if (current_station.compare("ks3") == 0 || current_station.compare("ks4") == 0)
  {
    if (req.station_name.compare("as3") == 0)
    {
      animate_client = this->data_ptr->ks_to_as3_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("as4") == 0)
    {
      animate_client = this->data_ptr->ks_to_as4_AnimateClient.at(agv_id);
    }
  }
  // as3 -> as4
  // as3 -> ks3
  // as3 -> ks4
  else if (current_station.compare("as3") == 0)
  {
    if (req.station_name.compare("as4") == 0)
    {
      animate_client = this->data_ptr->as3_to_as4_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("ks") == 0)
    {
      animate_client = this->data_ptr->as3_to_ks_AnimateClient.at(agv_id);
    }
  }
  // as4 -> as3
  // as4 -> ks3
  // as4 -> ks4
  else if (current_station.compare("as4") == 0)
  {
    if (req.station_name.compare("as3") == 0)
    {
      animate_client = this->data_ptr->as4_to_as3_AnimateClient.at(agv_id);
    }
    else if (req.station_name.compare("ks") == 0)
    {
      animate_client = this->data_ptr->as4_to_ks_AnimateClient.at(agv_id);
    }
  }

  std_srvs::Trigger animate;
  if (!animate_client.call(animate))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to task agv" << agv_id << " to move");
    return false;
  }

  if (animate.response.success)
  {
    auto current_sim_time = this->data_ptr->world->SimTime();
    res.success = true;
  }
  else
  {
    res.success = false;
    res.message = animate.response.message;
  }
  return true;
}

bool TaskManagerPlugin::HandleGetKitTrayContent(nist_gear::DetectKitTrayContent::Request& req,
                                                nist_gear::DetectKitTrayContent::Response& res, int agv_id)
{
  gzdbg << "\n";
  gzdbg << "[Service Call] Get Kit Tray Content\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";

  if (this->data_ptr->currentState != "go")
  {
    std::string errStr = "Competition is not running so this service will not work.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  // First, get the movable tray located on this AGV
  auto& getKitTrayContentClient = this->data_ptr->kit_tray_content_client.at(agv_id);
  if (!getKitTrayContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] Kit content service does not exist for agv" << agv_id);
    return false;
  }

  // get the content inside this movable tray
  nist_gear::DetectMovableTray agv_content;
  if (!getKitTrayContentClient.call(agv_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content for agv" << agv_id);
    return false;
  }

  std::string movable_tray_name = "";

  movable_tray_name = agv_content.response.movable_tray.movable_tray_name;

  if (movable_tray_name == "")
  {
    ROS_ERROR_STREAM("This AGV has no movable tray.");
    return false;
  }

  // now that we have the name of the movable tray, call the service to
  // get the products on this movable tray
  auto movable_tray_content_service = "/ariac/" + movable_tray_name + "/get_content";
  auto get_movable_tray_content_client =
      this->data_ptr->rosNode->serviceClient<nist_gear::DetectKittingShipment>(movable_tray_content_service);
  if (!get_movable_tray_content_client.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] Service to get the content of a movable tray does not exist");
    return false;
  }

  // get the parts located on the movable tray
  nist_gear::DetectKittingShipment movable_tray_content;
  if (!get_movable_tray_content_client.call(movable_tray_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content for movable tray" << agv_id);
    return false;
  }

  nist_gear::TrayContents tray_content;
  res.kit_tray_content = movable_tray_content.response.shipment;

  return true;
}

///////////////////////////////////////////////////////////////////
bool TaskManagerPlugin::HandleKittingSubmission(nist_gear::SubmitKittingShipment::Request& req,
                                                nist_gear::SubmitKittingShipment::Response& res, int agv_id)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

  if (this->data_ptr->currentState != "go")
  {
    std::string errStr = "Competition is not running so this service will not work.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  // First, get the movable tray located on this AGV
  auto& getKitTrayContentClient = this->data_ptr->kit_tray_content_client.at(agv_id);
  if (!getKitTrayContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] Kit content service does not exist for agv" << agv_id);
    return false;
  }

  // get the content inside this movable tray
  nist_gear::DetectMovableTray agv_content;
  if (!getKitTrayContentClient.call(agv_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to find the movable tray for agv" << agv_id);
    return false;
  }

  std::string movable_tray_name = "";
  movable_tray_name = agv_content.response.movable_tray.movable_tray_name;

  if (movable_tray_name == "")
  {
    ROS_ERROR_STREAM("Trying to submit an AGV without a movable tray.");
    return false;
  }

  // now that we have the name of the movable tray, call the service to
  // get the products on this movable tray
  auto movable_tray_content_service = "/ariac/" + movable_tray_name + "/get_content";
  auto get_movable_tray_content_client =
      this->data_ptr->rosNode->serviceClient<nist_gear::DetectKittingShipment>(movable_tray_content_service);
  if (!get_movable_tray_content_client.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] Service to get the content of a movable tray does not exist");
    return false;
  }

  // get the parts located on the movable tray
  nist_gear::DetectKittingShipment movable_tray_content;
  if (!get_movable_tray_content_client.call(movable_tray_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get parts inside the movable tray" << agv_id);
    return false;
  }

  // concatenate string and agv id
  std::string agv_name = "agv";
  agv_name = agv_name + std::to_string(agv_id);

  std::string parameter = "/ariac/" + agv_name + "/current_station";
  std::string current_station{};
  if (this->data_ptr->rosNode->getParam(parameter, current_station))
  {
    gzdbg << "[Current Station] " << current_station << std::endl;
  }

  this->data_ptr->submitted_kitting_shipment_station = req.assembly_station_name;
  this->data_ptr->submitted_shipment_agv = agv_id;

  // get the correct service client based on req.assembly_station_name
  ros::ServiceClient animate_client;

  // ks1 -> as1
  // ks1 -> as2
  // ks2 -> as1
  // ks2 -> as2
  if (current_station == "ks1" || current_station == "ks2")
  {
    // ROS_INFO_STREAM("current station: " << current_station);
    if (req.assembly_station_name.compare("as1") == 0)
    {
      // ROS_INFO_STREAM("assembly station: " << req.assembly_station_name);
      animate_client = this->data_ptr->ks_to_as1_AnimateClient.at(agv_id);
    }
    else if (req.assembly_station_name.compare("as2") == 0)
    {
      animate_client = this->data_ptr->ks_to_as2_AnimateClient.at(agv_id);
    }
  }
  else if (current_station.compare("ks3") == 0 || current_station.compare("ks4") == 0)
  {
    if (req.assembly_station_name.compare("as3") == 0)
    {
      animate_client = this->data_ptr->ks_to_as3_AnimateClient.at(agv_id);
    }
    else if (req.assembly_station_name.compare("as4") == 0)
    {
      animate_client = this->data_ptr->ks_to_as4_AnimateClient.at(agv_id);
    }
  }
  else
  {
    ROS_ERROR_STREAM("AGV has to be at its kitting station to submit a shipment");
    return false;
  }

  std_srvs::Trigger animate;
  if (!animate_client.call(animate))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to task agv" << agv_id << " to move");
    return false;
  }

  if (animate.response.success)
  {
    auto current_sim_time = this->data_ptr->world->SimTime();
    res.success = true;
  }
  else
  {
    res.success = false;
    res.message = animate.response.message;
  }

  // If AGV says it's moving, then notify scorer about shipment
  if (animate.response.success)
  {
    auto current_sim_time = this->data_ptr->world->SimTime();
    res.success = true;
    nist_gear::DetectedKittingShipment detected_shipment;
    detected_shipment.shipment_type = req.shipment_type;
    detected_shipment.assembly_station = req.assembly_station_name;
    nist_gear::TrayContents tray_content;
    detected_shipment.tray_content = movable_tray_content.response.shipment;
    // gzdbg << "NOTIFY KITTING SHIPMENT" << std::endl;
    this->data_ptr->ariac_scorer.NotifyKittingShipmentReceived(current_sim_time, req.shipment_type, detected_shipment);
  }
  else
  {
    res.success = false;
    res.message = animate.response.message;
  }

  return true;
}

/////////////////////////////////////////////////
bool TaskManagerPlugin::HandleAssemblySubmission(nist_gear::AssemblyStationSubmitShipment::Request& req,
                                                 nist_gear::AssemblyStationSubmitShipment::Response& res,
                                                 int station_id)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

  gzdbg << "\n";
  gzdbg << "[Service Call] Submit Assembly Shipment\n";
  gzdbg << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  gzdbg << "\n";

  if (this->data_ptr->currentState != "go")
  {
    std::string errStr = "Competition is not running so shipments cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  this->data_ptr->submitted_assembly_shipment_station = "as" + std::to_string(station_id);

  if (this->data_ptr->stationGetContentClient.end() == this->data_ptr->stationGetContentClient.find(station_id))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] no get_content client for station " << station_id);
    return false;
  }

  auto& getContentClient = this->data_ptr->stationGetContentClient.at(station_id);

  if (!getContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] get_content service does not exist for station" << station_id);
    return false;
  }

  nist_gear::DetectAssemblyShipment shipment_content;
  if (!getContentClient.call(shipment_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content at station" << station_id);
    return false;
  }

  auto current_sime_time = this->data_ptr->world->SimTime();
  res.success = true;
  std::string station_name = "as" + std::to_string(station_id);

  // gzdbg << "Before NotifyAssemblyShipmentReceived" << std::endl;
  this->data_ptr->ariac_scorer.NotifyAssemblyShipmentReceived(current_sime_time, req.shipment_type,
                                                              shipment_content.response.shipment, station_name);
  // gzdbg << "After NotifyAssemblyShipmentReceived" << std::endl;
  // Figure out what the score of that shipment was
  res.inspection_result = 0;
  this->data_ptr->current_trial_score = this->data_ptr->ariac_scorer.GetGameScore(this->data_ptr->floorPenalty);

  gzdbg << "Game score: " << this->data_ptr->current_trial_score << std::endl;

  for (auto& order_tuple : this->data_ptr->current_trial_score.order_scores_map)
  {
    for (const auto& shipmentScorePair : order_tuple.second.assembly_shipment_scores)
    {
      if (shipmentScorePair.first == req.shipment_type)
      {
        res.inspection_result = shipmentScorePair.second.total();
        break;
      }
    }
  }

  gzdbg << "Inspection result: " << res.inspection_result << std::endl;
  return true;
}

/////////////////////////////////////////////////////////////////////////
void TaskManagerPlugin::SetAGVLocation(std::string agv_frame, std::string assembly_station)
{
  std::string agv = agv_frame.substr(0, 4);
  std_msgs::String msg;
  msg.data = assembly_station;
  if (agv == "agv1")
  {
    this->data_ptr->agv1_current_station_publisher.publish(msg);
  }
  if (agv == "agv2")
  {
    this->data_ptr->agv2CurrentStationPub.publish(msg);
  }
  if (agv == "agv3")
  {
    this->data_ptr->agv3CurrentStationPub.publish(msg);
  }
  if (agv == "agv4")
  {
    this->data_ptr->agv4CurrentStationPub.publish(msg);
  }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::EnableConveyorBeltControl()
{
  gzdbg << "Enabling conveyor belt"
        << "\n";
  if (!this->data_ptr->conveyorControlClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] conveyor belt control service does not exist");
    return;
  }
  nist_gear::ConveyorBeltControl req;
  req.request.power = 100.0;
  if (!this->data_ptr->conveyorControlClient.call(req))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to enable conveyor belt");
  }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::PopulateConveyorBelt()
{
  gzdbg << "Populate conveyor belt called.\n";
  // Publish a message on the activation_plugin of the PopulationPlugin.
  gazebo::msgs::GzString msg;
  msg.set_data("restart");
  this->data_ptr->populatePub->Publish(msg);
}

/////////////////////////////////////////////////
void TaskManagerPlugin::AnnounceOrder(const ariac::Order& order)
{
  std::string priority = "";
  if (order.priority == 1)
  {
    priority = "\033[1;33mREGULAR\033[0m";
  }
  else if (order.priority == 3)
  {
    priority = "\033[1;31mHIGH\033[0m";
  }
  // Publish the order to ROS topic
  std::ostringstream logMessage;
  logMessage << std::endl;
  logMessage << "------------------------" << std::endl;
  logMessage << "    Announcing Order" << std::endl;
  logMessage << "------------------------" << std::endl;
  logMessage << "-Order ID: " << order.order_id << std::endl;
  logMessage << "-Priority: " << priority << std::endl;
  logMessage << "-File: " << this->data_ptr->config_yaml_file << std::endl;
  logMessage << "------------------------" << std::endl;
  logMessage << "-Kitting Shipment: x" << order.kitting_shipments.size() << std::endl;
  if (order.kitting_shipments.size() > 0)
  {
    int counter{ 1 };
    for (auto shipment : order.kitting_shipments)
    {
      logMessage << std::string(2, ' ') << "-Shipment #" << counter << ":\n";
      logMessage << std::string(4, ' ') << "-Tray Model: " << shipment.movable_tray.type << std::endl;
      logMessage << std::string(4, ' ') << "-AGV: " << shipment.agv_id << std::endl;
      logMessage << std::string(4, ' ') << "-Assembly Station: " << shipment.assembly_station << std::endl;
      logMessage << std::string(4, ' ') << "-Products:" << std::endl;
      for (auto product : shipment.products)
      {
        logMessage << std::string(6, ' ') << product.type << ":\n"
                   << std::string(8, ' ') << "xyz:" << product.pose.Pos().X() << "," << product.pose.Pos().Y() << ","
                   << product.pose.Pos().Z() << "\n"
                   << std::string(8, ' ') << "rpy:" << product.pose.Rot().Roll() << "," << product.pose.Rot().Pitch()
                   << "," << product.pose.Rot().Yaw() << "\n";
      }
      counter++;
    }
  }
  logMessage << "\n-Assembly Shipment: x" << order.assembly_shipments.size() << std::endl;
  if (order.assembly_shipments.size() > 0)
  {
    int counter{ 1 };
    for (auto shipment : order.assembly_shipments)
    {
      logMessage << std::string(2, ' ') << "-Shipment #" << counter << ":\n";
      logMessage << std::string(4, ' ') << "-Assembly Station: " << shipment.assembly_station << std::endl;
      logMessage << std::string(4, ' ') << "-Products:" << std::endl;
      for (auto product : shipment.products)
      {
        logMessage << std::string(6, ' ') << product.type << ":\n"
                   << std::string(8, ' ') << "xyz:" << product.pose.Pos().X() << "," << product.pose.Pos().Y() << ","
                   << product.pose.Pos().Z() << "\n"
                   << std::string(8, ' ') << "rpy:" << product.pose.Rot().Roll() << "," << product.pose.Rot().Pitch()
                   << "," << product.pose.Rot().Yaw() << "\n";
      }
      counter++;
    }
  }
  logMessage << "------------------------" << std::endl;

  nist_gear::RobotHealth robot_health_msg;
  robot_health_msg.kitting_robot_health = order.kitting_robot_health;
  robot_health_msg.assembly_robot_health = order.assembly_robot_health;
  this->data_ptr->robot_health_pub.publish(robot_health_msg);

  ROS_INFO_STREAM(logMessage.str().c_str());
  gzdbg << logMessage.str() << std::endl;
  nist_gear::Order orderMsg;
  fill_order_message(order, orderMsg);

  // building the order to be published on /ariac/orders
  nist_gear::Orders published_order_msg;
  published_order_msg.order_id = order.order_id;

  for (auto kitting_shipment : order.kitting_shipments)
  {
    nist_gear::KittingInOrder kitting_in_order_msg;
    kitting_in_order_msg.shipment_type = kitting_shipment.shipment_type;
    kitting_in_order_msg.agv = kitting_shipment.agv_id;
    kitting_in_order_msg.assembly_station = kitting_shipment.assembly_station;
    nist_gear::MovableTrayInOrder movable_tray_msg;
    movable_tray_msg.gripper = "gripper_tray";
    movable_tray_msg.movable_tray_type = kitting_shipment.movable_tray.type;
    movable_tray_msg.pose.position.x = kitting_shipment.movable_tray.pose.Pos().X();
    movable_tray_msg.pose.position.y = kitting_shipment.movable_tray.pose.Pos().Y();
    movable_tray_msg.pose.position.z = kitting_shipment.movable_tray.pose.Pos().Z();
    movable_tray_msg.pose.orientation.x = kitting_shipment.movable_tray.pose.Rot().X();
    movable_tray_msg.pose.orientation.y = kitting_shipment.movable_tray.pose.Rot().Y();
    movable_tray_msg.pose.orientation.z = kitting_shipment.movable_tray.pose.Rot().Z();
    movable_tray_msg.pose.orientation.w = kitting_shipment.movable_tray.pose.Rot().W();
    kitting_in_order_msg.movable_tray = movable_tray_msg;

    for (auto product : kitting_shipment.products)
    {
      nist_gear::ProductInOrder product_msg;
      product_msg.gripper = "gripper_part";
      product_msg.type = product.type;
      product_msg.pose.position.x = product.pose.Pos().X();
      product_msg.pose.position.y = product.pose.Pos().Y();
      product_msg.pose.position.z = product.pose.Pos().Z();
      product_msg.pose.orientation.x = product.pose.Rot().X();
      product_msg.pose.orientation.y = product.pose.Rot().Y();
      product_msg.pose.orientation.z = product.pose.Rot().Z();
      product_msg.pose.orientation.w = product.pose.Rot().W();
      kitting_in_order_msg.products.push_back(product_msg);
    }
    published_order_msg.kitting_shipments.push_back(kitting_in_order_msg);
  }

  for (auto assembly_shipment : order.assembly_shipments)
  {
    nist_gear::AssemblyInOrder assembly_in_order_msg;
    assembly_in_order_msg.shipment_type = assembly_shipment.shipmentType;
    assembly_in_order_msg.station_id = assembly_shipment.assembly_station;
    for (auto product : assembly_shipment.products)
    {
      nist_gear::ProductInOrder product_msg;
      product_msg.gripper = "gripper_part";
      product_msg.type = product.type;
      product_msg.pose.position.x = product.pose.Pos().X();
      product_msg.pose.position.y = product.pose.Pos().Y();
      product_msg.pose.position.z = product.pose.Pos().Z();
      product_msg.pose.orientation.x = product.pose.Rot().X();
      product_msg.pose.orientation.y = product.pose.Rot().Y();
      product_msg.pose.orientation.z = product.pose.Rot().Z();
      product_msg.pose.orientation.w = product.pose.Rot().W();
      assembly_in_order_msg.products.push_back(product_msg);
    }
    published_order_msg.assembly_shipments.push_back(assembly_in_order_msg);
  }

  this->data_ptr->orderPub.publish(published_order_msg);
}

/////////////////////////////////////////////////
void TaskManagerPlugin::StopCurrentOrder()
{
  // Stop the current order; any previous orders that are incomplete will automatically be resumed
  if (this->data_ptr->ordersInProgress.size())
  {
    auto orderID = this->data_ptr->ordersInProgress.top().order_id;
    gzdbg << "Stopping order: " << orderID << std::endl;
    this->data_ptr->ordersInProgress.pop();
  }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::OnKittingShipmentContent(nist_gear::TrayContents::ConstPtr shipment)
{
  // store the shipment content to be used for deciding when to interrupt orders
  // and when to disable a robot
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->kitting_shipment_contents[shipment->kit_tray] = shipment;
  // for (auto& cpair : this->data_ptr->kitting_shipment_contents) {
  //   for (const auto& product : cpair.second->products) {
  //     gzerr << product.type << std::endl;
  //   }
  // }
}

/////////////////////////////////////////////////
void TaskManagerPlugin::OnRobotHealthContent(nist_gear::RobotHealth msg)
{
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->kitting_robot_health = msg.kitting_robot_health;
  this->data_ptr->assembly_robot_health = msg.assembly_robot_health;
  // gzerr << msg.kitting_robot_health << std::endl;
}

/////////////////////////////////////////////////
void TaskManagerPlugin::OnAssemblyShipmentContent(nist_gear::DetectedAssemblyShipment::ConstPtr shipment)
{
  // store the shipment content to be used for deciding when to interrupt orders
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->assembly_shipment_contents[shipment->briefcase_id] = shipment;
}
//////////////////////////////////////////////////
void TaskManagerPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    const auto& contact = _msg->contact(i);
    /*
    bool col_1_is_arm = false;
    bool col_2_is_arm = false;
    for (const auto & collision_name : this->data_ptr->collisionFilter)
    {
      if (!col_1_is_arm && std::string::npos != contact.collision1().rfind(collision_name))
      {
        col_1_is_arm = true;
      }
      if (!col_2_is_arm && std::string::npos != contact.collision2().rfind(collision_name))
      {
        col_2_is_arm = true;
      }
    }
    if (col_1_is_arm && col_2_is_arm)
    {
      ROS_ERROR_STREAM("arm/arm contact detected: " << contact.collision1() << " and " << contact.collision2());
      std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
      common::Time time(contact.time().sec(), contact.time().nsec());
      this->data_ptr->ariac_scorer.NotifyArmArmCollision(time);
    }
    */

    // Simplified arm-arm and arm-torso collision, as all arm and torso links are prefaced with 'gantry::'
    // e.g. gantry::left_forearm_link::left_forearm_link_collision and gantry::torso_main::torso_main_collision
    // Also - only check if competition has started, as arm is in collision when first spawned
    if (this->data_ptr->currentState == "go" && contact.collision1().rfind("gantry", 0) == 0 &&
        contact.collision2().rfind("gantry", 0) == 0)
    {
      ROS_ERROR_STREAM("arm/arm contact detected: " << contact.collision1() << " and " << contact.collision2());
      std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
      common::Time time(contact.time().sec(), contact.time().nsec());
      this->data_ptr->ariac_scorer.NotifyArmArmCollision(time);
    }
  }
}

//////////////////////////////////////////////////
void TaskManagerPlugin::OnPenaltyReceived(ConstModelPtr& msg)
{
  auto message = msg;
  std::lock_guard<std::mutex> lock(this->data_ptr->mutex);
  this->data_ptr->floorPenalty += 1;
  gzdbg << "Penalty Applied: " << this->data_ptr->floorPenalty << std::endl;
}
