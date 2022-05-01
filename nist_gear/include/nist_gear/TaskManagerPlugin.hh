/*! \file ROSAriacTaskManagerPlugin.hh
    \brief File which contains functionalities to orchestrate ARIAC.
*/

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

#ifndef GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_
#define GAZEBO_ROS_ARIAC_TASK_MANAGER_PLUGIN_HH_

//standard library
#include <memory>
//gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
//gear
#include <nist_gear/AGVControl.h>
#include <nist_gear/AgvGoFromASToKS.h>
#include <nist_gear/AgvGoFromKSToAS.h>
#include <nist_gear/AgvGoFromASToAS.h>
#include <nist_gear/DetectedKittingShipment.h>
#include <nist_gear/DetectedAssemblyShipment.h>
#include <nist_gear/GetMaterialLocations.h>
#include <nist_gear/SubmitShipment.h>
#include <nist_gear/AssemblyStationSubmitShipment.h>
#include <nist_gear/SubmitKittingShipment.h>
#include <nist_gear/MoveToStation.h>
#include <nist_gear/RobotHealth.h>
#include <nist_gear/ChangeGripper.h>
#include <nist_gear/TrayContents.h>
#include <nist_gear/DetectKitTrayContent.h>
#include <nist_gear/GantryPosition.h>
//ros
#include <sdf/sdf.hh>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>


namespace gazebo {
  // Forward declare private data class
  class TaskManagerPluginPrivate;

  /// \brief A plugin that orchestrates an ARIAC task. First of all, it loads a
  /// description of the orders from ariac.world. Here's an example:
  ///
  ///  <order>
  ///    <time>5.0</time>
  ///    <!-- 1st shipment -->
  ///    <kitting>
  ///      <!-- 1st shipment -->
  ///      <shipment>
  ///        <product>
  ///          <type>coke_can</type>
  ///          <pose>-1 2.5 0.2 0 0 0</pose>
  ///        </product>
  ///        <product>
  ///          <type>cordless_drill/</type>
  ///          <pose>-1 2.5 0.2 0 0 0</pose>
  ///        </product>
  ///      </shipment>
  ///      <!-- 2nd shipment -->
  ///      <shipment>
  ///        <product>
  ///          <type>coke_can</type>
  ///          <pose>-1 2.5 0.2 0 0 0</pose>
  ///        </product>
  ///        <product>
  ///          <type>beer</type>
  ///          <pose>-1 2.5 0.2 0 0 0</pose>
  ///        </product>
  ///      </shipment>
  ///   </kitting>
  ///  </order>
  ///
  /// A task can have multiple orders. Each order has a time element. At that
  /// point in simulation time, the order will be notified to the team.
  /// An order is composed by a positive number of shipments. A shipment is composed by
  /// a positive number of products. An product contains a type (e.g.: coke_can)
  /// and a pose. The pose is the target pose where the product should be placed
  /// on a destination shipping box.
  ///
  /// After loading the orders, the plugin will use a simple finite state machine
  /// to handle the different tasks to do.
  ///
  /// The initial state is called "init" and there's not much to do when the
  /// plugin is in this state. The state of the Gazebo task manager is
  /// periodically published on a ROS topic "gazebo_task/state". This topic can
  /// be changed using the SDF element <gazebo_task_state_topic>. The plugin is
  /// waiting for a ROS message on topic "team_task/state". This topic shows
  /// the state of the team performing the task. When the value of a message
  /// received is "ready", the plugin will transition its internal state towards
  /// "ready" too.
  ///
  /// The "ready" state is considered simulation time zero for notifying the
  /// orders to the teams. A <time>1.0</time> inside a <order> element will be
  /// notified 1 second after entering in the "ready" state. The order will be
  /// published using a ROS topic on the topic ${robot_namespace}/order or
  /// ${robot_namespace}/${orders_topic} if the parameter <orders_topic> is passed
  /// in to the plugin. Also, when the plugin is in this state, it will use the
  /// conveyor activation topic to communicate with the Population plugin and
  /// enable the population of the conveyor belt. The element
  /// <conveyor_activation_topic> should match the topic used in the
  /// <activation_topic> of the Population plugin. After enabling the conveyor
  /// belt, the state changes to "go".
  ///
  /// In "go" state, the plugin will be updating the function that processes
  /// the orders. This is essentially checking if it's time to announce a new
  /// order.
  ///
  /// In "finish" state there's nothing to do.
  class GAZEBO_VISIBLE TaskManagerPlugin : public WorldPlugin
  {
    public:
    /**
     * @brief Construct a new ROSAriacTaskManagerPlugin object
     *
     */
    TaskManagerPlugin();

    /**
     * @brief Destroy the ROSAriacTaskManagerPlugin object
     *
     */
    virtual ~TaskManagerPlugin();

    /**
     * @brief Callback for start competition service
     *
     *Set the current state to "ready"
     *
     * @param req Service request (empty)
     * @param res Service response
     * @return true If the competition is started or the competition is not started
     * @return false Never set
     */
    bool StartCompetitionServiceCallback(
      std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    /**
     * @brief Callback for end competition service
     *
     *Set the current state to "end_game"
     *
     * @param req Service request (empty)
     * @param res Service response
     * @return true
     * @return false Never set
     */
    bool EndCompetitionServiceCallback(
      std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    /**
     * @brief Callback for when a kitting shipment is submitted for inspection.
     *
     *This service only works in development mode.
     *
     * @param event
     * @return true
     * @return false Never set
     */
    // bool InspectKittingShipmentServiceCallback(
    //   ros::ServiceEvent<nist_gear::SubmitShipment::Request,
    //   nist_gear::SubmitShipment::Response>& event);

  /**
   * @brief Process a submitted kitting shipment
   * 
   * @param req The request consists of the shipment type and the assembly station
   * @param res Success status
   * @param agv_id AGV to submit
   * @return true 
   * @return false 
   */
    bool HandleKittingSubmission(nist_gear::SubmitKittingShipment::Request& req,
      nist_gear::SubmitKittingShipment::Response& res, int agv_id);

    bool HandleGetKitTrayContent(nist_gear::DetectKitTrayContent::Request& req,
      nist_gear::DetectKitTrayContent::Response& res, int agv_id);

    /**
      * @brief Process a call to move an AGV to a station without shipment
      *
      * @param req The request consists of the name of the station
      * @param res Success status
      * @param agv_id AGV to submit
      * @return true
      * @return false
      */
    bool HandleMoveToStationCallback(nist_gear::MoveToStation::Request& req,
      nist_gear::MoveToStation::Response& res, int agv_id);
    

    /**
     * @brief Callback function to process service call to change gripper
     *
     * This function performs the following:
     * - Check the gripper type argument is correct
     * - Check the gantry is at the gripper changing station
     * - Update the value for the gripper parameter on the parameter server
     * @param req request with type of gripper
     * @param res response message from server to client
     * @return Always return true
     */
    bool GripperChangeServiceCallback(
      nist_gear::ChangeGripper::Request& req, nist_gear::ChangeGripper::Response& res);

    /**
   * @brief Inherited method
   *
   * @param _world
   * @param _sdf
   */
    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /**
     * @brief
     *
     * @param req
     * @param res
     * @param station_id
     * @return true
     * @return false
     */
    bool HandleAssemblySubmission(
      nist_gear::AssemblyStationSubmitShipment::Request& req,
      nist_gear::AssemblyStationSubmitShipment::Response& res, int station_id);

    // bool HandleDetectConnectedParts(
    //   nist_gear::DetectConnectedPartsToBriefcase::Request& req,
    //   nist_gear::DetectConnectedPartsToBriefcase::Response& res, int station_id);

    /**
     * @brief Get the Material Locations Service Callback object
     *
     * @param req
     * @param res
     * @return true
     * @return false
     */
    bool GetMaterialLocationsServiceCallback(
      nist_gear::GetMaterialLocations::Request& req,
      nist_gear::GetMaterialLocations::Response& res);

    /**
     * @brief
     *
     * @param req
     * @param res
     * @param agv_id
     * @return true
     * @return false
     */
    bool HandleAGVDeliverService(
      nist_gear::AGVControl::Request& req,
      nist_gear::AGVControl::Response& res, int agv_id);


    /**
     * @brief Callback when a tray publishes its content
     *
     * @param shipment
     */
    void OnKittingShipmentContent(nist_gear::TrayContents::ConstPtr shipment);

    void OnGantryPosition(nist_gear::GantryPosition::ConstPtr gantry_position);

    /**
     * @brief
     *
     * @param shipment
     */
    void OnAssemblyShipmentContent(nist_gear::DetectedAssemblyShipment::ConstPtr shipment);

    std::vector<std::string> GetStaticControllers(std::string robot_name);
    bool StopRobot(std::string robot_name);
    void StartRobot(std::vector<std::string> static_controllers, std::string robot_name);
  protected:
    /// \brief Update the plugin
    void OnUpdate();
    /// \brief Decide whether to announce a new order.
    void ProcessOrdersToAnnounce(gazebo::common::Time simTime);
    /// \brief Checks if the conditions are met to disable a robot
    void ProcessRobotStatus();
    /// \brief Set the assembly station of an AGV on the parameter server
    void SetAGVLocation(std::string agv_frame, std::string assembly_station);
    /// \brief Enable control of the conveyor belt.
    void EnableConveyorBeltControl();
    /// \brief Trigger/end the sensor blackout if appropriate.
    void ProcessSensorBlackout();
    /// \brief Publish competition status.
    void PublishStatus(const ros::TimerEvent&);
    /// \brief Start populating the conveyor belt.
    void PopulateConveyorBelt();

    // HandleSubmitShipmentService(
    // ros::ServiceEvent<nist_gear::SubmitShipment::Request, nist_gear::SubmitShipment::Response> & event)

    /// \brief Callback that receives the contact sensor's messages.
    void OnContactsReceived(ConstContactsPtr& msg);

    /// \brief Callback that receives the models from the floor deletion plugin
    void OnPenaltyReceived(ConstModelPtr& msg);

    /// \brief Announce an order to participants.
    void AnnounceOrder(const ariac::Order& order);

    /// \brief Stop scoring the current order and assign the next order on stack.
    void StopCurrentOrder();
    void OnAGV1Location(std_msgs::String::ConstPtr msg);
    void OnAGV2Location(std_msgs::String::ConstPtr msg);
    void OnAGV3Location(std_msgs::String::ConstPtr msg);
    void OnAGV4Location(std_msgs::String::ConstPtr msg);
    /// \brief Callback that receives the status of the robots
    void OnRobotHealthContent(nist_gear::RobotHealth msg);

    /// \brief Private data pointer.
    private:
    std::unique_ptr<TaskManagerPluginPrivate> data_ptr;
  };
} // namespace gazebo
#endif
