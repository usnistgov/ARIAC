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

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>
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
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "nist_gear/ARIAC.hh"
#include "nist_gear/ROSAriacTaskManagerPlugin.hh"
#include "nist_gear/AriacScorer.h"
#include "nist_gear/ConveyorBeltControl.h"
#include "nist_gear/DetectKittingShipment.h"
#include "nist_gear/DetectAssemblyShipment.h"
#include "nist_gear/KittingShipment.h"
#include "nist_gear/AssemblyShipment.h"
#include "nist_gear/Product.h"
#include "nist_gear/Order.h"
#include "nist_gear/RobotHealth.h"
#include "nist_gear/VacuumGripperState.h"

namespace gazebo {
/// \internal
/// \brief Private data for the ROSAriacTaskManagerPlugin class.
struct ROSAriacTaskManagerPluginPrivate
{
  public:
    physics::WorldPtr world;                    /*!< World pointer. */
    sdf::ElementPtr sdf;                        /*!< SDF pointer. */
    std::vector<ariac::Order> ordersToAnnounce; /*!< Collection of orders to announce. */
    ariac::Order currentOrder;                  /*!< Current order being processed */
    std::stack<ariac::Order> ordersInProgress;  //!< Collection of orders which have been announced but are not yet complete.
                                                //!< The order at the top of the stack is the active order.
    std::map<std::string, std::vector<std::string>> materialLocations;/*!< Mapping between material types and their locations. */
    std::map<std::string, nist_gear::DetectedKittingShipment::ConstPtr> kittingShipmentContents;/*!< Stored tray contents */
    std::map<std::string, nist_gear::DetectedAssemblyShipment::ConstPtr> assemblyShipmentContents;/*!< Stored briefcase contents */
    AriacScorer ariacScorer;                    /*!< A scorer to manage the game score. */
    ariac::GameScore currentGameScore;          /*!< The current game score. */
    std::unique_ptr<ros::NodeHandle> rosnode;   /*!< ROS node handle. */
    ros::Publisher orderPub;                    /*!< Publishes an order. */
    ros::Subscriber kittingShipmentContentSubscriber;/*!< Subscription for tray content */
    ros::Subscriber assemblyShipmentContentSubscriber;/*!< Subscription for briefcase content */
    ros::Subscriber stationForAGV1Sub;          /*!< Subscription to get the current station of AGV1*/
    ros::Subscriber stationForAGV2Sub;          /*!< Subscription to get the current station of AGV2*/
    ros::Subscriber stationForAGV3Sub;          /*!< Subscription to get the current station of AGV3*/
    ros::Subscriber stationForAGV4Sub;          /*!< Subscription to get the current station of AGV4*/
    ros::Publisher taskStatePub;                /*!< Publishes the Gazebo task state. */
    ros::Publisher taskScorePub;                /*!< Publishes the game score total. */
    ros::Publisher robot_health_pub;            /*!< Publishes the health of the robots. */
    ros::Publisher agv1CurrentStationPub;        /*!< Publishes the current location of an AGV. */
    ros::Publisher agv2CurrentStationPub;        /*!< Publishes the current location of an AGV. */
    ros::Publisher agv3CurrentStationPub;        /*!< Publishes the current location of an AGV. */
    ros::Publisher agv4CurrentStationPub;        /*!< Publishes the current location of an AGV. */
    
    std::string compStartServiceName;           /*!< Name of service that allows the user to start the competition. */
    ros::ServiceServer compStartServiceServer;  /*!< Service that allows the user to start the competition. */
    ros::ServiceServer compEndServiceServer;    /*!< Service that allows the user to end the competition. */
    ros::ServiceServer getMaterialLocationsServiceServer;/*!< Service that allows users to query the location of materials. */
    ros::ServiceServer submitTrayServiceServer; /*!< Service that allows a tray to be submitted for inspection. */
    std::map<int, ros::ServiceServer> agvDeliverService;/*!< Map of agv id to server that handles requests to deliver shipment. */
    std::map<int, ros::ServiceServer> agvToAssemblyStationService;/*!< Map of agv id to server that handles requests to go to assembly stations. */
    std::map<int, ros::ServiceServer> station_ship_content_service;/*!< The current game score. */
    std::map<int, ros::ServiceClient> agvGetContentClient;/*!< Map of agv id to client that can get its content. */
    std::map<int, ros::ServiceClient> stationGetContentClient;/*!< Map of assembly station id to client that can get its content. */
    std::map<int, ros::ServiceClient> agvToAS1AnimateClient;/*!< Map of agv id to client that can ask AGV to move to as1 */
    std::map<int, ros::ServiceClient> agvToAS2AnimateClient;/*!< Map of agv id to client that can ask AGV to move to as2 */
    std::map<int, ros::ServiceClient> agvToAS3AnimateClient;/*!< Map of agv id to client that can ask AGV to move to as3 */
    std::map<int, ros::ServiceClient> agvToAS4AnimateClient;/*!< Map of agv id to client that can ask AGV to move to as4 */
    ros::ServiceClient conveyorControlClient;   /*!< Client that turns on conveyor belt. */
    transport::NodePtr node;                    /*!< Transportation node. */
    transport::PublisherPtr populatePub;        /*!< Publisher for enabling the product population on the conveyor. */
    transport::PublisherPtr sensorBlackoutControlPub;/*!< Publisher for controlling the blackout of sensors. */
    double sensorBlackoutDuration;              /*!< Duration at which to blackout sensors. */
    int sensorBlackoutProductCount = 0;         /*!< Product count at which to blackout sensors. */
    bool isSensorBlackoutInProgress = false;    /*!< If sensor blackout is currently in progress. */
    bool isBeltNeeded = false;                  /*!< flag to activate the belt. */
    common::Time sensorBlackoutStartTime;       /*!< The start time of the sensor blackout. */
    ros::Timer statusPubTimer;                  /*!< Timer for regularly publishing state/score. */
    event::ConnectionPtr connection;            /*!< Connection event. */
    transport::PublisherPtr serverControlPub;   /*!< Publish Gazebo server control messages. */
    common::Time lastUpdateTime;                /*!< The time the last update was called. */
   common::Time lastSimTimePublish;             /*!< The time the sim time was last published. */
    common::Time gameStartTime;                 /*!< The time specified in the product is relative to this time. */
    double timeLimit;                           /*!< The time in seconds permitted to complete the trial. */
    double timeSpentOnCurrentOrder;             /*!< The time in seconds that has been spent on the current order. */
    std::string currentState = "init";          /*!< Pointer to the current state. */
    std::mutex mutex;                           /*!< The A mutex to protect currentState. */
    bool competitionMode = false;               /*!< During the competition, this environment variable will be set. */
    transport::SubscriberPtr contactSub;        /*!< Subscriber for the contact topic. */
    transport::SubscriberPtr floorPenaltySub;   /*!< Subscriber for the floor penalty topic. */
    int floorPenalty;                           /*!< Total number of parts dropped on floor. */
    std::string actualStationForKittingShipment;/*!< The assembly station to which the agv is sent when submitting a kitting shipmen. */
    int actualAGVUsedForKittingShipment;        /*!< AGV id used when submitting a kitting shipment. */
    std::string assemblyShipmentStation;        /*!< Name of the assembly shipment station. */
    std::string agv1_current_station;
    std::string agv2_current_station;
    std::string agv3_current_station;
    std::string agv4_current_station;

    const std::vector<std::string> gantry_collision_filter_vec{
        "base_link_collision", "shoulder_link_collision", "upper_arm_link_collision",
        "forearm_link_collision", "wrist_1_link_collision", "wrist_2_link_collision",
        "wrist_3_link_collision", "vacuum_gripper_link_collision"};
  };
} // namespace gazebo

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ROSAriacTaskManagerPlugin)

/** 
 * @brief Use an object from ariac::Order to fill out the nist_gear::Order message.
 */
static void fillOrderMsg(const ariac::Order &_objOrder,
                         nist_gear::Order &_msgOrder)
{
  _msgOrder.order_id = _objOrder.order_id;
  //--if the order consists of kitting tasks
  if (_objOrder.has_kitting_task)
  {
    //--parse each kitting shipment
    for (const auto &shipment : _objOrder.kitting_shipments)
    {
      nist_gear::KittingShipment msgShipment; //--compact the shipment in nist_gear::KittingShipment
      msgShipment.station_id = shipment.assembly_station;
      msgShipment.shipment_type = shipment.shipment_type;
      msgShipment.agv_id = shipment.agv_id;
      // gzdbg << "Shipment type: " << msgShipment.shipment_type << std::endl;
      // for (auto order : this->dataPtr->ordersToAnnounce)
      //   gzdbg << order << std::endl;

      //--get information for each product (type and pose)
      for (const auto &objProduct : shipment.products)
      {
        nist_gear::Product msgProduct;
        msgProduct.type = objProduct.type;
        msgProduct.pose.position.x = objProduct.pose.Pos().X();
        msgProduct.pose.position.y = objProduct.pose.Pos().Y();
        msgProduct.pose.position.z = objProduct.pose.Pos().Z();
        msgProduct.pose.orientation.x = objProduct.pose.Rot().X();
        msgProduct.pose.orientation.y = objProduct.pose.Rot().Y();
        msgProduct.pose.orientation.z = objProduct.pose.Rot().Z();
        msgProduct.pose.orientation.w = objProduct.pose.Rot().W();

        // Add the product to the shipment.
        msgShipment.products.push_back(msgProduct);
      }
      _msgOrder.kitting_shipments.push_back(msgShipment);
    }
  }

  //--if the order requires to do assembly
  if (_objOrder.has_assembly_task)
  {
    for (const auto &shipment : _objOrder.assembly_shipments)
    {
      nist_gear::AssemblyShipment msgShipment;
      msgShipment.shipment_type = shipment.shipmentType;
      msgShipment.station_id = shipment.assembly_station;
      for (const auto &objProduct : shipment.products)
      {
        nist_gear::Product msgProduct;
        msgProduct.type = objProduct.type;
        msgProduct.pose.position.x = objProduct.pose.Pos().X();
        msgProduct.pose.position.y = objProduct.pose.Pos().Y();
        msgProduct.pose.position.z = objProduct.pose.Pos().Z();
        msgProduct.pose.orientation.x = objProduct.pose.Rot().X();
        msgProduct.pose.orientation.y = objProduct.pose.Rot().Y();
        msgProduct.pose.orientation.z = objProduct.pose.Rot().Z();
        msgProduct.pose.orientation.w = objProduct.pose.Rot().W();

        // Add the product to the shipment.
        msgShipment.products.push_back(msgProduct);
      }
      _msgOrder.assembly_shipments.push_back(msgShipment);
    }
  }
  //--we now have built the Order message from the order as specified in ariac.world
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::ROSAriacTaskManagerPlugin()
    : dataPtr(new ROSAriacTaskManagerPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::~ROSAriacTaskManagerPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/********************************************************************************
 * @brief The @param _sdf element is used to retrieve elements from ariac.world
 *******************************************************************************/
void ROSAriacTaskManagerPlugin::Load(physics::WorldPtr _world,
                                     sdf::ElementPtr _sdf)
{
  gzdbg << "ARIAC VERSION: v.5.2\n";
  auto competitionEnv = std::getenv("ARIAC_COMPETITION");
  this->dataPtr->competitionMode = competitionEnv != NULL;
  gzdbg << "ARIAC COMPETITION MODE: " << (this->dataPtr->competitionMode ? competitionEnv : "false") << std::endl;

  GZ_ASSERT(_world, "ROSAriacTaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "ROSAriacTaskManagerPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  // Initialize Gazebo transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
                             "robot_namespace")
                         ->Get<std::string>() +
                     "/";
  }

  // Avoid the slowdown that is present with contact manager filter in gazebo 9.12 by subscribing to gazebo's main contact topic
  // Note: Moved this out of the order parsing loop as it didn't need to be in there,
  this->dataPtr->contactSub = this->dataPtr->node->Subscribe("~/physics/contacts", &ROSAriacTaskManagerPlugin::OnContactsReceived, this);

  // Initialize ROS
  this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  this->dataPtr->timeLimit = -1.0;
  if (_sdf->HasElement("competition_time_limit"))
    this->dataPtr->timeLimit = _sdf->Get<double>("competition_time_limit");

  std::string compEndServiceName = "end_competition";
  if (_sdf->HasElement("end_competition_service_name"))
    compEndServiceName = _sdf->Get<std::string>("end_competition_service_name");

  this->dataPtr->compStartServiceName = "start_competition";
  if (_sdf->HasElement("start_competition_service_name"))
    this->dataPtr->compStartServiceName = _sdf->Get<std::string>("start_competition_service_name");

  std::string taskStateTopic = "competition_state";
  if (_sdf->HasElement("task_state_topic"))
    taskStateTopic = _sdf->Get<std::string>("task_state_topic");

  std::string taskScoreTopic = "current_score";
  if (_sdf->HasElement("task_score_topic"))
    taskScoreTopic = _sdf->Get<std::string>("task_score_topic");

  std::string conveyorEnableTopic = "conveyor/enable";
  if (_sdf->HasElement("conveyor_enable_topic"))
    conveyorEnableTopic = _sdf->Get<std::string>("conveyor_enable_topic");

  

  std::string conveyorControlService = "conveyor/control";
  if (_sdf->HasElement("conveyor_control_service"))
    conveyorControlService = _sdf->Get<std::string>("conveyor_control_service");

  std::string populationActivateTopic = "populate_belt";
  if (_sdf->HasElement("population_activate_topic"))
    populationActivateTopic = _sdf->Get<std::string>("population_activate_topic");

  std::string ordersTopic = "orders";
  if (_sdf->HasElement("orders_topic"))
    ordersTopic = _sdf->Get<std::string>("orders_topic");

  std::string submitTrayServiceName = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayServiceName = _sdf->Get<std::string>("submit_tray_service_name");

  std::string kittingContentTopic = "kitting_shipment_content";
  if (_sdf->HasElement("kitting_shipment_content_topic_name"))
    kittingContentTopic = _sdf->Get<std::string>("kitting_shipment_content_topic_name");

  std::string assemblyContentTopic = "assembly_shipment_content";
  if (_sdf->HasElement("assembly_shipment_content_topic_name"))
    assemblyContentTopic = _sdf->Get<std::string>("assembly_shipment_content_topic_name");

  std::string getMaterialLocationsServiceName = "material_locations";
  if (_sdf->HasElement("material_locations_service_name"))
    getMaterialLocationsServiceName = _sdf->Get<std::string>("material_locations_service_name");

  std::string floorPenaltyTopic = "/ariac/floor_penalty";
  if (_sdf->HasElement("floor_penalty_topic"))
    floorPenaltyTopic = _sdf->Get<std::string>("floor_penalty_topic");

  std::string robotHealthTopic = "robot_health";
  if (_sdf->HasElement("robot_health_topic"))
    robotHealthTopic = _sdf->Get<std::string>("robot_health_topic");

  std::map<int, std::string> agvDeliverServiceName;
  std::map<int, std::string> agvAnimateServiceName;
  //--this will call /to_as1, defined in ROSAGVPlugin.cc
  std::map<int, std::string> agvToAS1AnimateServiceName;
  //--this will call /to_as2, defined in ROSAGVPlugin.cc
  std::map<int, std::string> agvToAS2AnimateServiceName;
  //--this will call /to_as3, defined in ROSAGVPlugin.cc
  std::map<int, std::string> agvToAS3AnimateServiceName;
  //--this will call /to_as4, defined in ROSAGVPlugin.cc
  std::map<int, std::string> agvToAS4AnimateServiceName;
  //--get the content of the AGV
  std::map<int, std::string> agv_get_content_service_name;
  //--get the content of a station
  std::map<int, std::string> station_get_content_service_name;
  //--submit an assembly shipment at a station
  std::map<int, std::string> station_submit_shipment_service_name;
  //--a service used to task an AGV to go to a specific station
  //-- e.g., /ariac/agv1/to_assembly_station AS1 order_0_kitting_shipment_0
  std::map<int, std::string> agv_to_assembly_station_service_name;
  std::map<int, std::string> agvLocationTopic;
  std::map<int, std::string> agvStartLocation;
  //<agv>...</agv>

  //--Checking for belt population cycles

  if (_sdf->HasElement("belt_population_cycles"))
  {
    // sdf::ElementPtr belt_cycles_element = _sdf->GetElement("belt_population_cycles");
    int belt_cycles = _sdf->Get<int>("belt_population_cycles");

    if (belt_cycles > 0)
    {
      this->dataPtr->isBeltNeeded = true;
    }
  }

  //--take care of <station> tags
  if (_sdf->HasElement("station"))
  {
    sdf::ElementPtr stationElem = _sdf->GetElement("station");
    while (stationElem)
    {
      int index = stationElem->Get<int>("index");
      station_get_content_service_name[index] = "get_content";
      station_submit_shipment_service_name[index] = "submit_shipment";

      //--<get_content_service_name>
      if (stationElem->HasElement("get_content_service_name"))
      {
        station_get_content_service_name[index] = stationElem->Get<std::string>("get_content_service_name");
      }
      //--<station_shipment_service_name>
      if (stationElem->HasElement("station_shipment_service_name"))
      {
        station_submit_shipment_service_name[index] = stationElem->Get<std::string>("station_shipment_service_name");
      }
      stationElem = stationElem->GetNextElement("station");
    }
  }

  //--take care of <agv> tags
  if (_sdf->HasElement("agv"))
  {
    sdf::ElementPtr agvElem = _sdf->GetElement("agv");
    while (agvElem)
    {
      int index = agvElem->Get<int>("index");
      agvDeliverServiceName[index] = "deliver";
      //@todo: removed in ARIAC2021
      // agvAnimateServiceName[index] = "animate";

      agvToAS1AnimateServiceName[index] = "";
      agvToAS2AnimateServiceName[index] = "";
      agvToAS3AnimateServiceName[index] = "";
      agvToAS4AnimateServiceName[index] = "";

      agv_get_content_service_name[index] = "get_content";
      agv_to_assembly_station_service_name[index] = "submit_shipment";
      agvLocationTopic[index]="";
      agvStartLocation[index]="";

      if (agvElem->HasElement("agv_location_topic"))
      {
        agvLocationTopic[index] = agvElem->Get<std::string>("agv_location_topic");
        //--Publisher for setting the location (station) of AGVs in the environment
        gzdbg << index << ":" << agvLocationTopic[index] << "\n";
        if (index == 1)
        {
          this->dataPtr->agv1CurrentStationPub = this->dataPtr->rosnode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);
          this->dataPtr->stationForAGV1Sub =
              this->dataPtr->rosnode->subscribe(agvLocationTopic[index], 1000, &ROSAriacTaskManagerPlugin::OnAGV1Location, this);
        }
        if (index == 2)
        {
          this->dataPtr->agv2CurrentStationPub = this->dataPtr->rosnode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);
          this->dataPtr->stationForAGV2Sub =
              this->dataPtr->rosnode->subscribe(agvLocationTopic[index], 1000, &ROSAriacTaskManagerPlugin::OnAGV2Location, this);
        }
        if (index == 3)
        {
          this->dataPtr->agv3CurrentStationPub = this->dataPtr->rosnode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);
          this->dataPtr->stationForAGV3Sub =
              this->dataPtr->rosnode->subscribe(agvLocationTopic[index], 1000, &ROSAriacTaskManagerPlugin::OnAGV3Location, this);
        }
        if (index == 4)
        {
          this->dataPtr->agv4CurrentStationPub = this->dataPtr->rosnode->advertise<std_msgs::String>(agvLocationTopic[index], 1000, true);
          this->dataPtr->stationForAGV4Sub =
              this->dataPtr->rosnode->subscribe(agvLocationTopic[index], 1000, &ROSAriacTaskManagerPlugin::OnAGV4Location, this);
        }
      }
      if (agvElem->HasElement("agv_start_location_name"))
      {
        agvStartLocation[index] = agvElem->Get<std::string>("agv_start_location_name");
        std_msgs::String msg;
        msg.data = agvStartLocation[index];

        if (index == 1){
          this->dataPtr->agv1_current_station = agvStartLocation[index];
          this->dataPtr->agv1CurrentStationPub.publish(msg);
        }

        if (index == 2)
        {
          this->dataPtr->agv2_current_station = agvStartLocation[index];
          this->dataPtr->agv2CurrentStationPub.publish(msg);
        }

        if (index == 3)
        {
          this->dataPtr->agv3_current_station = agvStartLocation[index];
          this->dataPtr->agv3CurrentStationPub.publish(msg);
        }

        if (index == 4)
        {
          this->dataPtr->agv4_current_station = agvStartLocation[index];
          this->dataPtr->agv4CurrentStationPub.publish(msg);
        }
      }
      if (agvElem->HasElement("agv_control_service_name"))
      {
        agvDeliverServiceName[index] = agvElem->Get<std::string>("agv_control_service_name");
      }
      if (agvElem->HasElement("get_content_service_name"))
      {
        agv_get_content_service_name[index] = agvElem->Get<std::string>("get_content_service_name");
      }
      if (agvElem->HasElement("agv_to_as_service_name"))
      {
        agv_to_assembly_station_service_name[index] = agvElem->Get<std::string>("agv_to_as_service_name");
      }
      if (agvElem->HasElement("to_as1_name"))
      {
        agvToAS1AnimateServiceName[index] = agvElem->Get<std::string>("to_as1_name");
      }
      if (agvElem->HasElement("to_as2_name"))
      {
        agvToAS2AnimateServiceName[index] = agvElem->Get<std::string>("to_as2_name");
      }
      if (agvElem->HasElement("to_as3_name"))
      {
        agvToAS3AnimateServiceName[index] = agvElem->Get<std::string>("to_as3_name");
      }
      if (agvElem->HasElement("to_as4_name"))
      {
        agvToAS4AnimateServiceName[index] = agvElem->Get<std::string>("to_as4_name");
      }

      

      agvElem = agvElem->GetNextElement("agv");
    }
  }

  // Parse the orders.
  sdf::ElementPtr orderElem = NULL;
  if (_sdf->HasElement("order"))
  {
    orderElem = _sdf->GetElement("order");
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

    // Parse the start time.
    double startTime = std::numeric_limits<double>::infinity();
    //--time at which this order is announced
    if (orderElem->HasElement("start_time"))
    {
      sdf::ElementPtr startTimeElement = orderElem->GetElement("start_time");
      startTime = startTimeElement->Get<double>();
    }

    // Parse the interruption criteria.
    int interruptOnUnwantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_unwanted_products"))
    {
      sdf::ElementPtr interruptOnUnwantedProductsElem = orderElem->GetElement("interrupt_on_unwanted_products");
      interruptOnUnwantedProducts = interruptOnUnwantedProductsElem->Get<int>();
    }
    int interruptOnWantedProducts = -1;
    if (orderElem->HasElement("interrupt_on_wanted_products"))
    {
      sdf::ElementPtr interruptOnWantedProductsElem = orderElem->GetElement("interrupt_on_wanted_products");
      interruptOnWantedProducts = interruptOnWantedProductsElem->Get<int>();
    }
    //--we need to add a new one for ARIAC2021
    //--interruptOnStationReached
    //--trigger an assembly order after the AGV is sent to the correct station (after completing <kitting_shipment>)
    // Parse the interruption criteria.
    std::string interruptOnStationReached = "";
    if (orderElem->HasElement("interrupt_on_station_reached"))
    {
      sdf::ElementPtr interruptOnStationReachedElem = orderElem->GetElement("interrupt_on_station_reached");
      interruptOnStationReached = interruptOnStationReachedElem->Get<std::string>();
    }

    //--If the kitting robot has to be disabled at some point
    ariac::RobotDisableCondition robot_disable_condition;
    robot_disable_condition.robot_type = "";
    robot_disable_condition.location = "";
    robot_disable_condition.number_of_parts = 0;

    if (orderElem->HasElement("robot_to_disable"))
    {
      sdf::ElementPtr robot_to_disable_elem = orderElem->GetElement("robot_to_disable");
      robot_disable_condition.robot_type = robot_to_disable_elem->Get<std::string>();
    }
    if (orderElem->HasElement("location_to_disable"))
    {
      sdf::ElementPtr location_to_disable_elem = orderElem->GetElement("location_to_disable");
      robot_disable_condition.location = location_to_disable_elem->Get<std::string>();
    }
    if (orderElem->HasElement("part_quantity_to_disable"))
    {
      sdf::ElementPtr part_quantity_elem = orderElem->GetElement("part_quantity_to_disable");
      robot_disable_condition.number_of_parts = part_quantity_elem->Get<int>();
    }

    //--The original health status for each robot
    int kitting_robot_health{};
    int assembly_robot_health{};
    if (orderElem->HasElement("kitting_robot_health"))
    {
      sdf::ElementPtr kitting_robot_healthElem = orderElem->GetElement("kitting_robot_health");
      kitting_robot_health = kitting_robot_healthElem->Get<int>();
    }

    if (orderElem->HasElement("assembly_robot_health"))
    {
      sdf::ElementPtr assembly_robot_healthElem = orderElem->GetElement("assembly_robot_health");
      assembly_robot_health = assembly_robot_healthElem->Get<int>();
    }

    // Parse the allowed completion time.
    double allowedTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("allowed_time"))
    {
      sdf::ElementPtr allowedTimeElement = orderElem->GetElement("allowed_time");
      allowedTime = allowedTimeElement->Get<double>();
    }

    //An order needs at least one kitting shipment OR one assembly shipment
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
          gzerr << "Unable to find <product> element in <kitting_shipment>. Ignoring"
                << std::endl;
          shipmentElem = shipmentElem->GetNextElement("kitting_shipment");
          continue;
        }

        //--std::vector that will be used to store information on <kitting_shipment>
        //--we extract information from the following xml snippet
        //<kitting_shipment>
        // <shipment_type>order_0_kitting_shipment_0</shipment_type>
        // <agv>agv2</agv>
        // <destination>AS1</destination>
        // <product>
        //   <type>assembly_battery_blue</type>
        //   <pose>0.2 -0.15 0.0 0.0 0.0 0.785398163397</pose>
        // </product>
        //</kitting_shipment>
        ariac::KittingShipment kitting_shipment;

        //--which agv to use to build the kit
        //--default is 'any'
        kitting_shipment.agv_id = "any";
        if (shipmentElem->HasElement("agv"))
        {
          kitting_shipment.agv_id = shipmentElem->Get<std::string>("agv");
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

        // ROS_WARN_STREAM("destination: "<< kitting_shipment.assembly_station);

        // Parse the shipment type.
        ariac::KittingShipmentType_t kittingShipmentType;
        if (shipmentElem->HasElement("shipment_type"))
        {
          kittingShipmentType = shipmentElem->Get<std::string>("shipment_type");
        }
        kitting_shipment.shipment_type = kittingShipmentType;
        // ROS_WARN_STREAM("shipment_type: "<< kitting_shipment.shipment_type);

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
          bool isFaulty = false; // We never want to request faulty products.
          ariac::Product product = {type, isFaulty, pose};
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
          gzerr << "Unable to find <product> element in <assembly_shipment>. Ignoring"
                << std::endl;
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
          bool isFaulty = false; // We never want to request faulty products.
          ariac::Product obj = {type, isFaulty, pose};
          assembly_shipment.products.push_back(obj);

          productElem = productElem->GetNextElement("product");
        }

        // Add a new shipment to the collection of shipments.
        assembly_shipments.push_back(assembly_shipment);

        shipmentElem = shipmentElem->GetNextElement("assembly_shipment");
      }
    }

    // Add a new order.
    //-- Has to follow the same order the attributes are listed in ariac::Order (ARIAC.hh)
    ariac::Order order = {
        orderID,
        order_priority,
        startTime,
        interruptOnUnwantedProducts,
        interruptOnWantedProducts,
        interruptOnStationReached,
        allowedTime,
        kitting_shipments,
        assembly_shipments,
        0.0, //--time taken
        has_kitting_task,
        has_assembly_task,
        robot_disable_condition,
        kitting_robot_health,
        assembly_robot_health};
    this->dataPtr->ordersToAnnounce.push_back(order);

    orderElem = orderElem->GetNextElement("order");
  }

  // Sort the orders by their start times.
  std::sort(this->dataPtr->ordersToAnnounce.begin(), this->dataPtr->ordersToAnnounce.end());

  // Debug output.
  // gzdbg << "Orders:" << std::endl;
  // for (auto order : this->dataPtr->ordersToAnnounce)
  //   gzdbg << order << std::endl;

  // Parse the material storage locations.

  ////////////////////////////////
  /// Material storage locations
  ////////////////////////////////
  if (_sdf->HasElement("material_locations"))
  {
    sdf::ElementPtr materialLocationsElem = _sdf->GetElement("material_locations");
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
      this->dataPtr->materialLocations[materialType] = locations;
      materialElem = materialElem->GetNextElement("material");
    }
  }

  /////////////////////
  /// Sensor Blackout
  /////////////////////
  if (_sdf->HasElement("sensor_blackout"))
  {
    auto sensorBlackoutElem = _sdf->GetElement("sensor_blackout");
    std::string sensorEnableTopic = sensorBlackoutElem->Get<std::string>("topic");
    this->dataPtr->sensorBlackoutProductCount = sensorBlackoutElem->Get<int>("product_count");
    this->dataPtr->sensorBlackoutDuration = sensorBlackoutElem->Get<double>("duration");
    this->dataPtr->sensorBlackoutControlPub =
        this->dataPtr->node->Advertise<msgs::GzString>(sensorEnableTopic);
  }

  // Publisher for announcing new orders.
  this->dataPtr->orderPub = this->dataPtr->rosnode->advertise<
      nist_gear::Order>(ordersTopic, 1000, true); // latched=true

  // Publisher for announcing new state of the competition.
  this->dataPtr->taskStatePub = this->dataPtr->rosnode->advertise<
      std_msgs::String>(taskStateTopic, 1000);

  // Publisher for announcing the score of the game.
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->taskScorePub = this->dataPtr->rosnode->advertise<
        std_msgs::Float32>(taskScoreTopic, 1000);
  }

  //--Publisher for announcing the health of the robots
  this->dataPtr->robot_health_pub = this->dataPtr->rosnode->advertise<
      nist_gear::RobotHealth>(robotHealthTopic, 1000, true);


  // Service for ending the competition.
  this->dataPtr->compEndServiceServer =
      this->dataPtr->rosnode->advertiseService(compEndServiceName,
                                               &ROSAriacTaskManagerPlugin::HandleEndService, this);

  // Service for submitting AGV trays for inspection without moving the AGVs
  this->dataPtr->submitTrayServiceServer =
      this->dataPtr->rosnode->advertiseService(submitTrayServiceName,
                                               &ROSAriacTaskManagerPlugin::HandleSubmitKittingShipmentService, this);

  // Service for querying material storage locations.
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->getMaterialLocationsServiceServer =
        this->dataPtr->rosnode->advertiseService(getMaterialLocationsServiceName,
                                                 &ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService, this);
  }

  // Subscriber for tray content
  this->dataPtr->kittingShipmentContentSubscriber =
      this->dataPtr->rosnode->subscribe(kittingContentTopic, 1000,
                                        &ROSAriacTaskManagerPlugin::OnKittingShipmentContent, this);

  // Subscriber for briefcase content
  this->dataPtr->assemblyShipmentContentSubscriber =
      this->dataPtr->rosnode->subscribe(assemblyContentTopic, 1000,
                                        &ROSAriacTaskManagerPlugin::OnAssemblyShipmentContent, this);

  // Gz Subscriber for floor penalty info
  this->dataPtr->floorPenaltySub =
  this->dataPtr->node->Subscribe(floorPenaltyTopic,
  &ROSAriacTaskManagerPlugin::OnPenaltyReceived, this);
  this->dataPtr->floorPenalty = 0;

  // Timer for regularly publishing state/score.
  this->dataPtr->statusPubTimer =
      this->dataPtr->rosnode->createTimer(ros::Duration(0.1),
                                          &ROSAriacTaskManagerPlugin::PublishStatus, this);

  this->dataPtr->populatePub =
      this->dataPtr->node->Advertise<msgs::GzString>(populationActivateTopic);

  //@todo: remove
  // for (auto &pair : agvDeliverServiceName)
  // {
  //   int index = pair.first;
  //   std::string serviceName = pair.second;
  //   this->dataPtr->agvDeliverService[index] =
  //       this->dataPtr->rosnode->advertiseService<nist_gear::AGVControl::Request, nist_gear::AGVControl::Response>(
  //           serviceName,
  //           boost::bind(&ROSAriacTaskManagerPlugin::HandleAGVDeliverService, this,
  //                       _1, _2, index));
  // }

  for (auto &pair : agv_to_assembly_station_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->agvToAssemblyStationService[index] =
        this->dataPtr->rosnode->advertiseService<nist_gear::AGVToAssemblyStation::Request, nist_gear::AGVToAssemblyStation::Response>(
            serviceName,
            boost::bind(&ROSAriacTaskManagerPlugin::HandleSendAgvToASService, this,
                        _1, _2, index));
  }

  for (auto &pair : agv_get_content_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->agvGetContentClient[index] =
        this->dataPtr->rosnode->serviceClient<nist_gear::DetectKittingShipment>(serviceName);
  }

  //--take care of the services within the <station> tags
  for (auto &pair : station_submit_shipment_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->station_ship_content_service[index] =
        this->dataPtr->rosnode->advertiseService<nist_gear::AssemblyStationSubmitShipment::Request, nist_gear::AssemblyStationSubmitShipment::Response>(
            serviceName,
            boost::bind(&ROSAriacTaskManagerPlugin::HandleSubmitAssemblyShipmentService, this,
                        _1, _2, index));
  }

  for (auto &pair : station_get_content_service_name)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    this->dataPtr->stationGetContentClient[index] =
        this->dataPtr->rosnode->serviceClient<nist_gear::DetectAssemblyShipment>(serviceName);
  }

  //@todo: remove
  // for (auto &pair : agvAnimateServiceName)
  // {
  //   int index = pair.first;
  //   std::string serviceName = pair.second;
  //   this->dataPtr->agvAnimateClient[index] =
  //       this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
  // }

  for (auto &pair : agvToAS1AnimateServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->dataPtr->agvToAS1AnimateClient[index] =
          this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto &pair : agvToAS2AnimateServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      // ROS_WARN_STREAM("agvToAS2AnimateServiceName");
      this->dataPtr->agvToAS2AnimateClient[index] =
          this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto &pair : agvToAS3AnimateServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->dataPtr->agvToAS3AnimateClient[index] =
          this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  for (auto &pair : agvToAS4AnimateServiceName)
  {
    int index = pair.first;
    std::string serviceName = pair.second;
    if (!serviceName.empty())
    {
      this->dataPtr->agvToAS4AnimateClient[index] =
          this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(serviceName);
    }
  }

  this->dataPtr->conveyorControlClient =
      this->dataPtr->rosnode->serviceClient<nist_gear::ConveyorBeltControl>(conveyorControlService);

  this->dataPtr->serverControlPub =
      this->dataPtr->node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&ROSAriacTaskManagerPlugin::OnUpdate, this));
}


void ROSAriacTaskManagerPlugin::OnAGV1Location(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->agv1_current_station = _msg->data;
}

void ROSAriacTaskManagerPlugin::OnAGV2Location(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->agv2_current_station = _msg->data;
}

void ROSAriacTaskManagerPlugin::OnAGV3Location(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->agv3_current_station = _msg->data;
}

void ROSAriacTaskManagerPlugin::OnAGV4Location(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->agv4_current_station = _msg->data;
}
/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto currentSimTime = this->dataPtr->world->SimTime();

  // Delay advertising the competition start service to avoid a crash.
  // Sometimes if the competition is started before the world is fully loaded, it causes a crash.
  // See https://bitbucket.org/osrf/ariac/issues/91
  if (!this->dataPtr->compStartServiceServer && currentSimTime.Double() >= 5.0)
  {
    // Service for starting the competition.
    this->dataPtr->compStartServiceServer =
        this->dataPtr->rosnode->advertiseService(this->dataPtr->compStartServiceName,
                                                 &ROSAriacTaskManagerPlugin::HandleStartService, this);
  }

  if ((currentSimTime - this->dataPtr->lastSimTimePublish).Double() >= 1.0)
  {
    gzdbg << "Sim time: " << currentSimTime.Double() << std::endl;
    this->dataPtr->lastSimTimePublish = currentSimTime;
  }

  double elapsedTime = (currentSimTime - this->dataPtr->lastUpdateTime).Double();
  if (this->dataPtr->timeLimit >= 0 && this->dataPtr->currentState == "go" &&
      (currentSimTime - this->dataPtr->gameStartTime) > this->dataPtr->timeLimit)
  {
    this->dataPtr->currentState = "end_game";
  }

  if (this->dataPtr->currentState == "ready")
  {
    // gzdbg << "ready\n";
    this->dataPtr->gameStartTime = currentSimTime;
    this->dataPtr->currentState = "go";

    if (this->dataPtr->isBeltNeeded)
    {
      this->EnableConveyorBeltControl();
      this->PopulateConveyorBelt();
    }
  }
  else if (this->dataPtr->currentState == "go")
  {
    // Check if we need to disable any robot
      this->ProcessRobotStatus();
    // Update the order manager.
    this->ProcessOrdersToAnnounce(currentSimTime);

    // Update the sensors if appropriate.
    this->ProcessSensorBlackout();

    // Update the score.
    // TODO(sloretz) only publish this when an event that could change the score happens
    auto gameScore = this->dataPtr->ariacScorer.GetGameScore(this->dataPtr->floorPenalty);

    if (gameScore.total() != this->dataPtr->currentGameScore.total())
    {
      std::ostringstream logMessage;
      logMessage << "Current game score: " << gameScore.total();
      ROS_DEBUG_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      this->dataPtr->currentGameScore = gameScore;
    }

    if (!this->dataPtr->ordersInProgress.empty())
    {
      this->dataPtr->ordersInProgress.top().time_taken += elapsedTime;
      auto orderID = this->dataPtr->ordersInProgress.top().order_id;
      // TODO: timing should probably be managed by the scorer but we want to use sim time
      this->dataPtr->timeSpentOnCurrentOrder = this->dataPtr->ordersInProgress.top().time_taken;

      auto has_kitting_shipments = this->dataPtr->ordersInProgress.top().has_kitting_task;
      auto has_assembly_shipments = this->dataPtr->ordersInProgress.top().has_assembly_task;

      //--if the order has kitting shipments, check if all kits have been submitted
      if (has_kitting_shipments && !has_assembly_shipments)
      {
        // gzdbg << "Only Kitting\n";
        bool all_kitting_shipments_submitted = gameScore.order_scores_map.at(orderID).isKittingComplete();
        if (all_kitting_shipments_submitted)
        {

          if (this->dataPtr->gameStartTime != common::Time())
          {
            this->dataPtr->currentGameScore.total_process_time =
                (currentSimTime - this->dataPtr->gameStartTime).Double();
          }

          std::ostringstream logMessage;
          logMessage << "All kitting shipments submitted for order: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
      else if (!has_kitting_shipments && has_assembly_shipments)
      {
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
      else if (has_kitting_shipments && has_assembly_shipments)
      {
        // gzdbg << "Assembly and Kitting\n";
        bool all_kitting_shipments_submitted = gameScore.order_scores_map.at(orderID).isKittingComplete();
        bool all_assembly_shipments_submitted = gameScore.order_scores_map.at(orderID).isAssemblyComplete();

        // if (all_kitting_shipments_submitted)
        // {
        //   std::ostringstream logMessageKitting;
        //   logMessageKitting << "All kitting shipments submitted for current order: " << this->dataPtr->currentGameScore.total() << "\nScore breakdown:\n"
        //                     << this->dataPtr->currentGameScore;
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
      else
      {
        // Check if the time limit for the current order has been exceeded.
        if (this->dataPtr->timeSpentOnCurrentOrder > this->dataPtr->ordersInProgress.top().allowed_time)
        {
          std::ostringstream logMessage;
          logMessage << "Order timed out: " << orderID;
          ROS_INFO_STREAM(logMessage.str().c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
    }

    if (this->dataPtr->ordersInProgress.empty() && this->dataPtr->ordersToAnnounce.empty())
    {
      gzdbg << "No more orders to process." << std::endl;
      this->dataPtr->currentState = "end_game";
    }
  }
  else if (this->dataPtr->currentState == "end_game")
  {
    //TODO: Apply this-dataPtr->floorPenalty to GameScore calculation
    this->dataPtr->currentGameScore = this->dataPtr->ariacScorer.GetGameScore(this->dataPtr->floorPenalty);
    if (this->dataPtr->gameStartTime != common::Time())
    {
      this->dataPtr->currentGameScore.total_process_time =
          (currentSimTime - this->dataPtr->gameStartTime).Double();
    }
    std::ostringstream logMessage;
    logMessage << "End of trial. Final score: " << this->dataPtr->currentGameScore.total() << "\nScore breakdown:\n"
               << this->dataPtr->currentGameScore;
    ROS_INFO_STREAM(logMessage.str().c_str());
    gzdbg << logMessage.str() << std::endl;
    this->dataPtr->currentState = "done";

    bool is_first = true;
    std::stringstream sstr;
    for (const auto order_tuple : this->dataPtr->currentGameScore.order_scores_map)
    {

      sstr << order_tuple.second.csv_kitting(is_first).c_str();
      sstr << order_tuple.second.csv_assembly(is_first).c_str();
      is_first = false;
    }
    ROS_INFO_STREAM(sstr.str().c_str());

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
      this->dataPtr->serverControlPub->Publish(msg);
    }
  }

  this->dataPtr->lastUpdateTime = currentSimTime;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PublishStatus(const ros::TimerEvent &)
{
  std_msgs::Float32 scoreMsg;
  scoreMsg.data = this->dataPtr->currentGameScore.total();
  if (!this->dataPtr->competitionMode)
  {
    this->dataPtr->taskScorePub.publish(scoreMsg);
  }

  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->taskStatePub.publish(stateMsg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessRobotStatus()
{
  /**
  Check the field Order::robot_disable_condition against what we have in simulation
  if location and number_of_parts match then disable the apropriate robot based (field robot_type)
   */

  // gzdbg << "ProcessRobotStatus: \n";
  
  auto robotDisableCondition = this->dataPtr->currentOrder.robot_disable_condition;
  // gzdbg << "robotType: " << robotDisableCondition.robot_type << "\n";
  if (robotDisableCondition.robot_type.empty())
    return;
  else{
    auto robotType = robotDisableCondition.robot_type;
    auto location = robotDisableCondition.location;
    unsigned int nbOfParts = robotDisableCondition.number_of_parts;
    // gzdbg << "robotType: " << robotType << "\n";
    // gzdbg << "location: " << location << "\n";

    //--check how many parts we have in location
    //--call the service /ariac/kit_tray_x/get_content
    std::string location_topic = "";
    if (location == "agv1" || location == "agv2" || location == "agv3" || location == "agv4")
    {
      location_topic = "/ariac/kit_tray_";
      location_topic += +location.back();
      location_topic += "/get_content";
      // gzdbg << "LOCATION TOPIC: " << location_topic<< "\n";
      ros::ServiceClient client = this->dataPtr->rosnode->serviceClient<nist_gear::DetectKittingShipment>(location_topic);
      nist_gear::DetectKittingShipment srv;
      if (client.call(srv))
      {
        // gzdbg << "Service called" << std::endl;
        auto resp_products = srv.response.shipment.products;
        //--if the number of parts matches, then update the robot health status
        if (resp_products.size() == nbOfParts)
        {
          nist_gear::RobotHealth robot_health_msg;
          if (robotType == "kitting_robot")
          {
            gzdbg << "Disabling kitting_robot\n";
            robot_health_msg.kitting_robot_enabled = 0;
            robot_health_msg.assembly_robot_enabled = 1;
            this->dataPtr->robot_health_pub.publish(robot_health_msg);
          }
          else if (robotType == "assembly_robot")
          {
            gzdbg << "Disabling assembly_robot\n";
            robot_health_msg.kitting_robot_enabled = 1;
            robot_health_msg.assembly_robot_enabled = 0;
            this->dataPtr->robot_health_pub.publish(robot_health_msg);
          }
          //--update the conditions so this function is not called
          this->dataPtr->currentOrder.robot_disable_condition.robot_type = "";
          this->dataPtr->currentOrder.robot_disable_condition.location = "";
          this->dataPtr->currentOrder.robot_disable_condition.number_of_parts = 0;
        }
        // gzdbg << "Nb of products: \n" << resp_products.size();
        // for (auto product: resp_products){
        //   gzdbg << product.type << std::endl;
        // }
      }
    }

    if (location == "as1" ||location == "as2" ||location == "as3" || location == "as4"){
      location_topic = "/ariac/briefcase_";
      location_topic += +location.back();
      location_topic += "/get_content";
      // gzdbg << "LOCATION TOPIC: " << location_topic<< "\n";
      ros::ServiceClient client = this->dataPtr->rosnode->serviceClient<nist_gear::DetectAssemblyShipment>(location_topic);
      nist_gear::DetectAssemblyShipment srv;
      if (client.call(srv))
      {
        // gzdbg << "Service called" << std::endl;
        auto resp_products = srv.response.shipment.products;
        //--if the number of parts matches, then update the robot health status
        if (resp_products.size() == nbOfParts)
        {
          nist_gear::RobotHealth robot_health_msg;
          if (robotType == "assembly_robot")
          {
            gzdbg << "Disabling assembly_robot\n";
            robot_health_msg.kitting_robot_enabled = 1;
            robot_health_msg.assembly_robot_enabled = 0;
            this->dataPtr->robot_health_pub.publish(robot_health_msg);
          }
          else if (robotType == "kitting_robot")
          {
            gzdbg << "Disabling kitting_robot\n";
            robot_health_msg.kitting_robot_enabled = 0;
            robot_health_msg.assembly_robot_enabled = 1;
            this->dataPtr->robot_health_pub.publish(robot_health_msg);
          }
          //--update the conditions so this function is not called
          this->dataPtr->currentOrder.robot_disable_condition.robot_type = "";
          this->dataPtr->currentOrder.robot_disable_condition.location = "";
          this->dataPtr->currentOrder.robot_disable_condition.number_of_parts = 0;
        }
        // gzdbg << "Nb of products: \n" << resp_products.size();
        // for (auto product: resp_products){
        //   gzdbg << product.type << std::endl;
        // }
      }
    }
  }
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessOrdersToAnnounce(gazebo::common::Time simTime)
{
  
  if (this->dataPtr->ordersToAnnounce.empty())
    return;

  auto nextOrder = this->dataPtr->ordersToAnnounce.front();

  //--this is used to process the robots' health status
  this->dataPtr->currentOrder = this->dataPtr->ordersToAnnounce.front();

  // gzdbg << "Orders:" << std::endl;
  // for (auto order : this->dataPtr->ordersToAnnounce)
  // {
  //   gzdbg << order.order_id << std::endl;
  //   gzdbg << order.has_assembly_task << std::endl;
  //   gzdbg << order.has_kitting_task << std::endl;
  // }

  //--publish the health of each robot when announcing an order
  nist_gear::RobotHealth robot_health_msg;
  robot_health_msg.kitting_robot_enabled = nextOrder.kitting_robot_health;
  robot_health_msg.assembly_robot_enabled = nextOrder.assembly_robot_health;
  this->dataPtr->robot_health_pub.publish(robot_health_msg);

  // gzwarn << "-------------PUBLISHING to robot_health\n";

  bool interruptOnUnwantedProducts = nextOrder.interrupt_on_unwanted_products > 0;
  bool interruptOnWantedProducts = nextOrder.interrupt_on_wanted_products > 0;
  bool interruptOnStationReached = !nextOrder.interrupt_on_station_reached.empty();
  bool noActiveOrder = this->dataPtr->ordersInProgress.empty();
  auto elapsed = this->dataPtr->world->SimTime() - this->dataPtr->gameStartTime;
  bool announceNextOrder = false;

  // Check whether announce a new order from the list.
  // Announce next order if the appropriate amount of time has elapsed
  announceNextOrder |= elapsed.Double() >= nextOrder.start_time;
  gzdbg << "announceNextOrder time: " << announceNextOrder << std::endl;
  // Announce next order if there is no active order and we are waiting to interrupt
  announceNextOrder |= noActiveOrder && (interruptOnWantedProducts || interruptOnUnwantedProducts);
  gzdbg << "announceNextOrder condition: " << announceNextOrder << std::endl;
  // Check if it's time to interrupt (skip if we're already interrupting anyway)
  if (!announceNextOrder && (interruptOnWantedProducts || interruptOnUnwantedProducts || interruptOnStationReached))
  {
    /**
     * ******************************************
     * Dealing with interruptOnStationReached 
     * ******************************************
     */
    if (nextOrder.has_assembly_task && !nextOrder.has_kitting_task)
    {
      //--let's check if a kitting shipment has been submitted
      //--we know if a kitting shipment was submitted by checking the following variables
      std::string actual_station_name = this->dataPtr->actualStationForKittingShipment;
      int actual_agv_id = this->dataPtr->actualAGVUsedForKittingShipment;
      std::string actual_agv_name = "agv" + std::to_string(actual_agv_id);

      if (!actual_station_name.empty() && actual_agv_id > 0)
      {
        // gzerr << "+++kitting shipment submitted" << std::endl;
        // gzerr << "+++ actual station: " << actual_station_name << std::endl;
        // gzerr << "+++ actual agv: " << actual_agv_name << std::endl;
        //--get information from the second order in the list
        // auto second_order = this->dataPtr->ordersToAnnounce.at(0);
        //--this will return something like: agv2_at_as2
        std::string interrupt_condition = nextOrder.interrupt_on_station_reached;
        //--let's grab the first 4 characters of interrupt_condition to get the agv name
        auto expected_agv_name = interrupt_condition.substr(0, 4);
        //--let's grab the last 3 characters of interrupt_condition to get the station name
        auto expected_station_name = interrupt_condition.substr(interrupt_condition.size() - 3);
        // gzerr << "+++ expected station: " << expected_station_name << std::endl;
        // gzerr << "+++ expected agv: " << expected_agv_name << std::endl;

        //if correct agv sent to correct station then announce the next order
        if (actual_agv_name == expected_agv_name)
        {
          if (actual_station_name.compare(expected_station_name) == 0)
          {
            announceNextOrder = true;
          }
        }
      }
    }
    gzdbg << "announceNextOrder reached: " << announceNextOrder << std::endl;
    /**
     * *****************************************************************************************
     * Dealing with wanted and unwanted products only if the second order has kitting shipments
     * *****************************************************************************************
     */
    // Check if the products in the shipping boxes are enough to interrupt the current order
    //Task interruption will only work if the next order has a kitting task (not assembly)
    if (!nextOrder.has_assembly_task && nextOrder.has_kitting_task)
    {
      std::vector<std::string> productsInNextOrder;
      for (const auto &shipment : nextOrder.kitting_shipments)
      {
        for (const auto &product : shipment.products)
        {
          productsInNextOrder.push_back(product.type);
        }
      }
      //@todo Add assembly shipment

      // Check whether the trays have products for the next order or not
      // This is used to trigger the announcement of the next order at convenient or inconvenient times
      int max_num_wanted_products = 0;
      int max_num_unwanted_products = 0;
      for (auto &cpair : this->dataPtr->kittingShipmentContents)
      {
        std::vector<std::string> productsInNextOrder_copy(productsInNextOrder);
        int num_wanted_products = 0;
        int num_unwanted_products = 0;
        for (const auto &product : cpair.second->products)
        {
          // Don't count faulty products, because they have to be removed anyway.
          if (!product.is_faulty)
          {
            auto it = std::find(productsInNextOrder_copy.begin(), productsInNextOrder_copy.end(), product.type);
            if (it == productsInNextOrder_copy.end())
            {
              ++num_unwanted_products;
            }
            else
            {
              ++num_wanted_products;
              productsInNextOrder_copy.erase(it);
            }
          }
        }
        if (num_wanted_products > max_num_wanted_products)
        {
          max_num_wanted_products = num_wanted_products;
        }
        if (num_unwanted_products > max_num_unwanted_products)
        {
          max_num_unwanted_products = num_unwanted_products;
        }
      }

      // Announce next order if a tray has more than enough wanted or unwanted products
      announceNextOrder |= interruptOnWantedProducts && (max_num_wanted_products >= nextOrder.interrupt_on_wanted_products);
      gzdbg << "announceNextOrder interruptOnWantedProducts: " << announceNextOrder << std::endl;
      announceNextOrder |= interruptOnUnwantedProducts && (max_num_unwanted_products >= nextOrder.interrupt_on_unwanted_products);
      gzdbg << "announceNextOrder interruptOnUnwantedProducts: " << announceNextOrder << std::endl;
    }
  }

  if (announceNextOrder)
  {
    // gzdbg << "announceNextOrder\n";
    auto updateLocn = nextOrder.order_id.find("_update");
    if (updateLocn != std::string::npos)
    {
      gzdbg << "Order to update: " << nextOrder.order_id << std::endl;
      this->AnnounceOrder(nextOrder);

      // Update the order the scorer's monitoring
      gzdbg << "Updating order: " << nextOrder << std::endl;
      auto nextOrderID = nextOrder.order_id.substr(0, updateLocn);
      nist_gear::Order orderMsg;
      fillOrderMsg(nextOrder, orderMsg);
      this->dataPtr->ariacScorer.NotifyOrderUpdated(simTime, nextOrderID, orderMsg);
      this->dataPtr->ordersToAnnounce.erase(this->dataPtr->ordersToAnnounce.begin());
      return;
    }

    gzdbg << "New order to announce: " << nextOrder.order_id << std::endl;

    // Move order to the 'in process' stack
    this->dataPtr->ordersInProgress.push(ariac::Order(nextOrder));
    this->dataPtr->ordersToAnnounce.erase(this->dataPtr->ordersToAnnounce.begin());

    this->AnnounceOrder(nextOrder);
    // Assign the scorer the order to monitor
    gzdbg << "Assigning order: \n"
          << nextOrder << std::endl;
    nist_gear::Order orderMsg;
    fillOrderMsg(nextOrder, orderMsg);

    this->dataPtr->ariacScorer.NotifyOrderStarted(simTime, orderMsg, nextOrder.priority);
  }
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessSensorBlackout()
{
  // gzdbg << "ProcessSensorBlackout\n";
  auto currentSimTime = this->dataPtr->world->SimTime();
  if (this->dataPtr->sensorBlackoutProductCount > 0)
  {
    // Count total products in all boxes.
    int totalProducts = 0;
    for (auto &cpair : this->dataPtr->kittingShipmentContents)
    {
      totalProducts += cpair.second->products.size();
    }
    if (totalProducts >= this->dataPtr->sensorBlackoutProductCount)
    {
      ROS_INFO_STREAM("Triggering sensor blackout because " << totalProducts << " products detected.");
      gazebo::msgs::GzString activateMsg;
      activateMsg.set_data("deactivate");
      this->dataPtr->sensorBlackoutControlPub->Publish(activateMsg);
      this->dataPtr->sensorBlackoutProductCount = -1;
      this->dataPtr->sensorBlackoutStartTime = currentSimTime;
      this->dataPtr->isSensorBlackoutInProgress = true;
    }
  }
  if (this->dataPtr->isSensorBlackoutInProgress)
  {
    auto elapsedTime = (currentSimTime - this->dataPtr->sensorBlackoutStartTime).Double();
    if (elapsedTime > this->dataPtr->sensorBlackoutDuration)
    {
      gzdbg << "Ending sensor blackout." << std::endl;
      gazebo::msgs::GzString activateMsg;
      activateMsg.set_data("activate");
      this->dataPtr->sensorBlackoutControlPub->Publish(activateMsg);
      this->dataPtr->isSensorBlackoutInProgress = false;
    }
  }
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleStartService(
    std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res)
{
  gzdbg << "Handle start service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->currentState == "init")
  {
    this->dataPtr->currentState = "ready";
    res.success = true;
    res.message = "competition started successfully! GOOD LUCK!";
    return true;
  }
  res.success = false;
  res.message = "cannot start if not in 'init' state";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleEndService(
    std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res)
{
  gzdbg << "Handle end service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->currentState = "end_game";
  res.success = true;
  res.message = "competition ended successfully!";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleSubmitKittingShipmentService(
    ros::ServiceEvent<nist_gear::SubmitShipment::Request, nist_gear::SubmitShipment::Response> &event)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const nist_gear::SubmitShipment::Request &req = event.getRequest();
  nist_gear::SubmitShipment::Response &res = event.getResponse();

  const std::string &callerName = event.getCallerName();
  gzdbg << "Submit shipment service called by: " << callerName << std::endl;

  if (this->dataPtr->competitionMode && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition mode is enabled so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  if (this->dataPtr->currentState != "go")
  {
    std::string errStr = "Competition is not running so shipments cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  // Figure out which AGV is being submitted
  int agv_id = 0;
  std::string destination_id(req.destination_id);
  if (destination_id.size() > 1)
  {
    // this is probably a tray name, reduce it to just the AGV id
    size_t kit_tray_pos = destination_id.find("kit_tray_");
    if (kit_tray_pos != std::string::npos)
    {
      size_t id_pos = kit_tray_pos + std::string("kit_tray_").size();
      if (destination_id.size() > id_pos)
      {
        // throw away all but 1 character
        destination_id = destination_id[id_pos];
      }
    }
  }

  std::string current_station{};

  if (1 == destination_id.size() && '1' == destination_id[0])
  {
    agv_id = 1;
    current_station = this->dataPtr->agv1_current_station;
    // this->dataPtr->rosnode->getParam("/ariac/agv1_station", current_station);
  }
  else if (1 == destination_id.size() && '2' == destination_id[0])
  {
    agv_id = 2;
    current_station = this->dataPtr->agv2_current_station;
    // this->dataPtr->rosnode->getParam("/ariac/agv2_station", current_station);
  }
  else if (1 == destination_id.size() && '3' == destination_id[0])
  {
    agv_id = 3;
    current_station = this->dataPtr->agv3_current_station;
    // this->dataPtr->rosnode->getParam("/ariac/agv3_station", current_station);
  }
  else if (1 == destination_id.size() && '4' == destination_id[0])
  {
    agv_id = 4;
    current_station = this->dataPtr->agv4_current_station;
    // this->dataPtr->rosnode->getParam("/ariac/agv4_station", current_station);
  }

  if (0 == agv_id)
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] Could not determine AGV from: " << req.destination_id);
    res.success = false;
    res.inspection_result = 0;
    return true;
  }

  if (this->dataPtr->agvGetContentClient.end() == this->dataPtr->agvGetContentClient.find(agv_id))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] no content client for agv " << agv_id);
    return false;
  }

  auto &getContentClient = this->dataPtr->agvGetContentClient.at(agv_id);

  if (!getContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] content service does not exist for " << agv_id);
    return false;
  }

  nist_gear::DetectKittingShipment shipment_content;
  if (!getContentClient.call(shipment_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content" << agv_id);
    return false;
  }

  auto currentSimTime = this->dataPtr->world->SimTime();
  res.success = true;
  this->dataPtr->ariacScorer.NotifyKittingShipmentReceived(currentSimTime,
                                                           req.shipment_type,
                                                           shipment_content.response.shipment,
                                                           req.station_id);
  // SetAGVLocation(shipment_content.response.shipment.destination_id, req.station_id);

  // Figure out what the score of that shipment was
  res.inspection_result = 0;
  this->dataPtr->currentGameScore = this->dataPtr->ariacScorer.GetGameScore(this->dataPtr->floorPenalty);
  for (auto &orderScorePair : this->dataPtr->currentGameScore.order_scores_map)
  {
    for (const auto &shipmentScorePair : orderScorePair.second.kitting_shipment_scores)
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

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService(
    nist_gear::GetMaterialLocations::Request &req,
    nist_gear::GetMaterialLocations::Response &res)
{
  gzdbg << "Get material locations service called\n";
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->materialLocations.find(req.material_type);
  if (it == this->dataPtr->materialLocations.end())
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

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleSubmitAssemblyShipmentService(
    nist_gear::AssemblyStationSubmitShipment::Request &req, nist_gear::AssemblyStationSubmitShipment::Response &res, int station_id)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  gzdbg << "Submit assembly shipment service called" << std::endl;

  if (this->dataPtr->currentState != "go")
  {
    std::string errStr = "Competition is not running so shipments cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  if (this->dataPtr->stationGetContentClient.end() == this->dataPtr->stationGetContentClient.find(station_id))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] no get_content client for station " << station_id);
    return false;
  }

  auto &getContentClient = this->dataPtr->stationGetContentClient.at(station_id);

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

  auto currentSimTime = this->dataPtr->world->SimTime();
  res.success = true;
  std::string station_name = "as" + std::to_string(station_id);
  this->dataPtr->ariacScorer.NotifyAssemblyShipmentReceived(currentSimTime, req.shipment_type, shipment_content.response.shipment, station_name);

  // Figure out what the score of that shipment was
  res.inspection_result = 0;
  this->dataPtr->currentGameScore = this->dataPtr->ariacScorer.GetGameScore(this->dataPtr->floorPenalty);
  for (auto &order_tuple : this->dataPtr->currentGameScore.order_scores_map)
  {
    for (const auto &shipmentScorePair : order_tuple.second.assembly_shipment_scores)
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

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleSendAgvToASService(
    nist_gear::AGVToAssemblyStation::Request &req, nist_gear::AGVToAssemblyStation::Response &res, int agv_id)
{
  this->dataPtr->actualStationForKittingShipment = req.assembly_station_name;
  this->dataPtr->actualAGVUsedForKittingShipment = agv_id;
  std::string shipment_type = req.shipment_type;

  gzdbg << "AGV go to station service called for agv" << agv_id << "\n";
  if (agv_id == 1 || agv_id == 2) //--for AGV1 and AGV2
  {
    if (this->dataPtr->agvToAS1AnimateClient.end() == this->dataPtr->agvToAS1AnimateClient.find(agv_id))
    {
      ROS_ERROR_STREAM("[ARIAC TaskManager] NO \"to_as1\" animate client for agv " << agv_id);
      return false;
    }
    if (this->dataPtr->agvToAS2AnimateClient.end() == this->dataPtr->agvToAS2AnimateClient.find(agv_id))
    {
      ROS_ERROR_STREAM("[ARIAC TaskManager] NO \"to_as2\" animate client for agv " << agv_id);
      return false;
    }
    // if (this->dataPtr->agvToAS3AnimateClient.end() == this->dataPtr->agvToAS3AnimateClient.find(agv_id))
    // {
    //   ROS_ERROR_STREAM("[ARIAC TaskManager] no \"to_as3\" animate client for agv " << agv_id);
    //   return false;
    // }
  }
  else if (agv_id == 3 || agv_id == 4) //--for AGV3 and AGV4
  {
    if (this->dataPtr->agvToAS3AnimateClient.end() == this->dataPtr->agvToAS3AnimateClient.find(agv_id))
    {
      ROS_ERROR_STREAM("[ARIAC TaskManager] NO \"to_as3\" animate client for agv " << agv_id);
      return false;
    }
    if (this->dataPtr->agvToAS4AnimateClient.end() == this->dataPtr->agvToAS4AnimateClient.find(agv_id))
    {
      ROS_ERROR_STREAM("[ARIAC TaskManager] NO \"to_as4\" animate client for agv " << agv_id);
      return false;
    }
    // if (this->dataPtr->agvToAS6AnimateClient.end() == this->dataPtr->agvToAS6AnimateClient.find(agv_id))
    // {
    //   ROS_ERROR_STREAM("[ARIAC TaskManager] no \"to_as6\" animate client for agv " << agv_id);
    //   return false;
    // }
  }

  ros::ServiceClient agv_to_as_animate_client;
  if (this->dataPtr->actualStationForKittingShipment == "as1")
  {
    agv_to_as_animate_client = this->dataPtr->agvToAS1AnimateClient.at(agv_id);
  }
  else if (this->dataPtr->actualStationForKittingShipment == "as2")
  {
    // gzdbg << "-----as2-----"<< agv_id << std::endl;
    agv_to_as_animate_client = this->dataPtr->agvToAS2AnimateClient.at(agv_id);
  }
  else if (this->dataPtr->actualStationForKittingShipment == "as3")
  {
    agv_to_as_animate_client = this->dataPtr->agvToAS3AnimateClient.at(agv_id);
  }
  else if (this->dataPtr->actualStationForKittingShipment == "as4")
  {
    agv_to_as_animate_client = this->dataPtr->agvToAS4AnimateClient.at(agv_id);
  }

  if (!agv_to_as_animate_client.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] animate service does not exist for agv" << agv_id);
    return false;
  }

  //--get kit content
  auto &getContentClient = this->dataPtr->agvGetContentClient.at(agv_id);
  if (!getContentClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] content service does not exist for " << agv_id);
    return false;
  }

  //--detect shipment
  nist_gear::DetectKittingShipment shipment_content;
  if (!getContentClient.call(shipment_content))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to get content" << agv_id);
    return false;
  }

  std_srvs::Trigger animate;
  if (!agv_to_as_animate_client.call(animate))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to ask agv to animate" << agv_id);
    return false;
  }

  // If AGV says it's moving, then notify scorer about shipment
  if (animate.response.success)
  {
    auto currentSimTime = this->dataPtr->world->SimTime();
    res.success = true;
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    //--we cannot get the assembly station the kitting shipment was sent to
    //--so, we grab this information from the service call, e.g:
    //--rosservice call /ariac/agv1/to_assembly station "as1" order_0_kitting_shipment_0
    //--in this example this->dataPtr->actualStationForKittingShipment = "as1"
    this->dataPtr->ariacScorer.NotifyKittingShipmentReceived(currentSimTime, req.shipment_type, shipment_content.response.shipment, this->dataPtr->actualStationForKittingShipment);
    //  SetAGVLocation(shipment_content.response.shipment.destination_id, this->dataPtr->actualStationForKittingShipment);
     
  }
  else
  {
    res.success = false;
    res.message = animate.response.message;
  }
  return true;
}

void ROSAriacTaskManagerPlugin::SetAGVLocation(std::string agv_frame, std::string assembly_station)
{
  std::string agv = agv_frame.substr(0, 4);
  std::string parameter = "/ariac/"+agv+"_station";
  std_msgs::String msg;
  msg.data = assembly_station;
  if (agv == "agv1")
  {
    this->dataPtr->agv1CurrentStationPub.publish(msg);
  }
  if (agv == "agv2")
  {
    this->dataPtr->agv2CurrentStationPub.publish(msg);
  }
  if (agv == "agv3")
  {
    this->dataPtr->agv3CurrentStationPub.publish(msg);
  }
  if (agv == "agv4")
  {
    this->dataPtr->agv4CurrentStationPub.publish(msg);
  }
}


/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::EnableConveyorBeltControl()
{
  if (!this->dataPtr->conveyorControlClient.exists())
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] conveyor belt control service does not exist");
    return;
  }
  nist_gear::ConveyorBeltControl req;
  req.request.power = 100.0;
  if (!this->dataPtr->conveyorControlClient.call(req))
  {
    ROS_ERROR_STREAM("[ARIAC TaskManager] failed to enable conveyor belt");
  }
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PopulateConveyorBelt()
{
  gzdbg << "Populate conveyor belt called.\n";
  // Publish a message on the activation_plugin of the PopulationPlugin.
  gazebo::msgs::GzString msg;
  msg.set_data("restart");
  this->dataPtr->populatePub->Publish(msg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::AnnounceOrder(const ariac::Order &order)
{
  // Publish the order to ROS topic
  std::ostringstream logMessage;
  logMessage << "Announcing order: " << order.order_id << std::endl;
  ROS_INFO_STREAM(logMessage.str().c_str());
  gzdbg << logMessage.str() << std::endl;
  nist_gear::Order orderMsg;
  fillOrderMsg(order, orderMsg);
  this->dataPtr->orderPub.publish(orderMsg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::StopCurrentOrder()
{
  // Stop the current order; any previous orders that are incomplete will automatically be resumed
  if (this->dataPtr->ordersInProgress.size())
  {
    auto orderID = this->dataPtr->ordersInProgress.top().order_id;
    gzdbg << "Stopping order: " << orderID << std::endl;
    this->dataPtr->ordersInProgress.pop();
  }
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnKittingShipmentContent(nist_gear::DetectedKittingShipment::ConstPtr shipment)
{
  // store the shipment content to be used for deciding when to interrupt orders
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->kittingShipmentContents[shipment->destination_id] = shipment;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnAssemblyShipmentContent(nist_gear::DetectedAssemblyShipment::ConstPtr shipment)
{
  // store the shipment content to be used for deciding when to interrupt orders
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->assemblyShipmentContents[shipment->briefcase_id] = shipment;
}
//////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnContactsReceived(ConstContactsPtr &_msg)
{
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    const auto &contact = _msg->contact(i);
    /*
    bool col_1_is_arm = false;
    bool col_2_is_arm = false;
    for (const auto & collision_name : this->dataPtr->collisionFilter)
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
      std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
      common::Time time(contact.time().sec(), contact.time().nsec());
      this->dataPtr->ariacScorer.NotifyArmArmCollision(time);
    }
    */

    // Simplified arm-arm and arm-torso collision, as all arm and torso links are prefaced with 'gantry::'
    // e.g. gantry::left_forearm_link::left_forearm_link_collision and gantry::torso_main::torso_main_collision
    // Also - only check if competition has started, as arm is in collision when first spawned
    if (this->dataPtr->currentState == "go" &&
        contact.collision1().rfind("gantry", 0) == 0 &&
        contact.collision2().rfind("gantry", 0) == 0)
    {
      ROS_ERROR_STREAM("arm/arm contact detected: " << contact.collision1() << " and " << contact.collision2());
      std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
      common::Time time(contact.time().sec(), contact.time().nsec());
      this->dataPtr->ariacScorer.NotifyArmArmCollision(time);
    }
  }
}

//////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnPenaltyReceived(ConstModelPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->floorPenalty += 1;
  gzdbg << "Dropped Parts: " << this->dataPtr->floorPenalty << std::endl;
}
