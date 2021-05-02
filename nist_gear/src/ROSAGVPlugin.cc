/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "ROSAGVPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <nist_gear/SubmitTray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include <string>

namespace gazebo
{

    /// \internal
    /// \brief Private data for the ROSAGVPlugin class.
    struct ROSAGVPluginPrivate
    {
        /// \brief Name of the AGV
    public:
        std::string agvName;
        std::mutex mutex;
    
    

    public:
        std::string agv1Param;
        std::string agv2Param;
        std::string agv3Param;
        std::string agv4Param;

        /// \brief Name of the assembly station
    public:
        std::string assemblyStationName;

        /// \brief Scoped name of the link of the tray on the AGV
    public:
        std::string trayLinkName;

        /// \brief World pointer
    public:
        physics::WorldPtr world;

        /// \brief Pointer to the update event connection
    public:
        event::ConnectionPtr updateConnection;

        /// \brief for setting ROS name space
    public:
        std::string robotNamespace;

        /// \brief ros node handle
    public:
        ros::NodeHandle *rosnode;

        /// \brief Receives service calls for controlling the AGV
    public:
        ros::ServiceServer rosService;
        ros::ServiceServer rosServiceAGVToAS1;
        ros::ServiceServer rosServiceAGVToAS2;
        ros::ServiceServer rosServiceAGVToAS3;
        ros::ServiceServer rosServiceAGVToAS4;
    public:
        ros::ServiceServer rosServiceAssembly;
        // ros::ServiceServer rosServiceKitting;

        /// \brief Transportation node.
    public:
        transport::NodePtr gzNode;

        /// \brief Gazebo publish for locking/unlocking parts to this AGV's tray
    public:
        transport::PublisherPtr lockUnlockTrayModelsPub;

        /// \brief Client for clearing this AGV's tray
    public:
        // ros::ServiceClient rosClearTrayClient;
        ros::ServiceClient rosAGVToAssemblyClient;

        /// \brief Robot animation for delivering the tray
    public:
        gazebo::common::PoseAnimationPtr deliverTrayAnimation;
        /// \brief Animation for AGVs going from a kitting station to assembly station 1 or 4
        gazebo::common::PoseAnimationPtr KS_to_AS1AS3_animation;
        /// \brief Animation for AGVs going from a kitting station to assembly station 2 or 5
        gazebo::common::PoseAnimationPtr KS_to_AS2AS4_animation;
        gazebo::common::PoseAnimationPtr AS1AS3_to_KS_animation;
        gazebo::common::PoseAnimationPtr AS2AS4_to_KS_animation;
        /// \brief Robot animation for the AGV returning to its base
    public:
        gazebo::common::PoseAnimationPtr returnAnimation;

        /// \brief Pointer to the model
    public:
        gazebo::physics::ModelPtr model;

        /// \brief The state of the AGV
    public:
        std::string currentState;

        /// \brief The time the last tray delivery was triggered
    public:
        common::Time deliveryTriggerTime;

    public:
        common::Time goToAssemblyStationTriggerTime;
        common::Time goToKittingStationTriggerTime;

        /// \brief Whether or not gravity of the AGV has been disabled
    public:
        bool gravityDisabled;

        /// \brief Flag for triggering tray delivery from the service callback
    public:
        bool deliveryTriggered = false;

    public:
        bool goToAssemblyStationTriggered = false;
        bool goToKittingStationTriggered = false;

        /// \brief Publishes the AGV state.
    public:
        ros::Publisher statePub;
        /// \brief Publishes the station where the AGV is located
        ros::Publisher stationPub;
        // Subscriber for agv location
        ros::Subscriber agvLocationSubscriber;
        std::string current_station;
  
    };
} // namespace gazebo

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSAGVPlugin);

/*
 * @brief Constructor
 */
ROSAGVPlugin::ROSAGVPlugin()
    : dataPtr(new ROSAGVPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAGVPlugin::~ROSAGVPlugin()
{
    this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAGVPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    gzdbg << "ROSAGVPlugin\n";
    std::string index;

    if (_sdf->HasElement("index"))
        index = _sdf->Get<std::string>("index");
    else
        gzerr << "AGV is missing an index. The AGV will not work.\n";

    this->dataPtr->world = _parent->GetWorld();

    // load parameters
    this->dataPtr->robotNamespace = "";
    if (_sdf->HasElement("robotNamespace"))
    {
        this->dataPtr->robotNamespace = _sdf->GetElement(
                                                "robotNamespace")
                                            ->Get<std::string>() +
                                        "/";
        ROS_WARN_STREAM("robotNamespace: " << this->dataPtr->robotNamespace << "\n");
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
                         << "unable to load plugin. Load the Gazebo system plugin "
                         << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    this->dataPtr->agvName = std::string("agv") + index;

    this->dataPtr->trayLinkName =
        this->dataPtr->agvName + "::kit_tray_" + index + "::kit_tray_" + index + "::tray";

    // Topics to store the current location of AGVs
    std::string AGVStationTopicName = "";
    if (_sdf->HasElement("agv_location_topic_name")){
        AGVStationTopicName = _sdf->Get<std::string>("agv_location_topic_name");
    }

    std::string AGVToAS1Service = "to_as1";
    if (_sdf->HasElement("to_as1_name"))
        AGVToAS1Service = _sdf->Get<std::string>("to_as1_name");
    ROS_DEBUG_STREAM("Using AGVToAS1Service service: " << AGVToAS1Service);

    std::string AGVToAS2Service = "to_as2";
    if (_sdf->HasElement("to_as2_name"))
        AGVToAS2Service = _sdf->Get<std::string>("to_as2_name");
    ROS_DEBUG_STREAM("Using AGVToAS2Service service: " << AGVToAS2Service);

    std::string AGVToAS3Service = "to_as3";
    if (_sdf->HasElement("to_as3_name"))
        AGVToAS3Service = _sdf->Get<std::string>("to_as3_name");
    ROS_DEBUG_STREAM("Using AGVToAS3Service service: " << AGVToAS3Service);

    std::string AGVToAS4Service = "to_as4";
    if (_sdf->HasElement("to_as4_name"))
        AGVToAS4Service = _sdf->Get<std::string>("to_as4_name");
    ROS_DEBUG_STREAM("Using AGVToAS4Service service: " << AGVToAS4Service);

    std::string lockUnlockTrayServiceName = "lock_tray_models";
    if (_sdf->HasElement("lock_unlock_tray_service_name"))
        lockUnlockTrayServiceName = _sdf->Get<std::string>("lock_unlock_tray_service_name");

    std::string toAssemblyServiceName = "to_assembly_station";
    if (_sdf->HasElement("agv_to_as_service_name"))
        toAssemblyServiceName = _sdf->Get<std::string>("agv_to_as_service_name");

    this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->robotNamespace);

    // Initialize Gazebo transport
    this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
    this->dataPtr->gzNode->Init();

    this->dataPtr->agvLocationSubscriber =
        this->dataPtr->rosnode->subscribe(AGVStationTopicName, 1000, &ROSAGVPlugin::OnAGVLocation, this);

    //--ROS publisher to send "lock" or "unlock" messages to topic
    this->dataPtr->lockUnlockTrayModelsPub =
        this->dataPtr->gzNode->Advertise<msgs::GzString>(lockUnlockTrayServiceName);

    double speedFactor = 1.2;

    
    

    // //--All 4 AGVs have the same y position and the same rpy during animation
    float ypos = _parent->WorldPose().Pos().Y();
    // //--the xpos of each AGV tells us at which station the AGV is
    // //--we will then use this information to set the parameter on the parameter server
    // float xpos = _parent->WorldPose().Pos().X();

    // //--for agv1
    // if (ypos > 4.65 && ypos < 4.68)
    // {
    //     if (-2.25 > xpos && xpos > -2.27)
    //     {
    //         agv_station_msg.set_data("ks1");
    //     }
    //     else if (-5.59 > xpos && xpos > -5.61)
    //     {
    //         agv_station_msg.set_data("as1");
    //     }
    //     else if (-10.58 > xpos && xpos > -10.60)
    //     {
    //         agv_station_msg.set_data("as2");
    //     }
    // }
    // //--for agv2
    // if (ypos > 1.35 && ypos < 1.37)
    // {
    //     if (-2.25 > xpos && xpos > -2.27)
    //     {
    //         agv_station_msg.set_data("ks2");
    //     }
    //     else if (-5.59 > xpos && xpos > -5.61)
    //     {
    //         agv_station_msg.set_data("as1");
    //     }
    //     else if (-10.58 > xpos && xpos > -10.60)
    //     {
    //         agv_station_msg.set_data("as2");
    //     }
    // }
    // //--for agv3: y=-1.333917
    // if (-1.32 > ypos && ypos > -1.34)
    // {
    //     // ROS_WARN_STREAM("=== AGV3 ===");
    //     if (-2.25 > xpos && xpos > -2.27)
    //     {
    //         agv_station_msg.set_data("ks3");
    //     }
    //     else if (-5.59 > xpos && xpos > -5.61)
    //     {
    //         agv_station_msg.set_data("as3");
    //     }
    //     else if (-10.58 > xpos && xpos > -10.60)
    //     {
    //         agv_station_msg.set_data("as4");
    //     }
    // }
    // //--for agv4: y=-4.696062
    // if (-4.68 > ypos && ypos > -4.70)
    // {
    //     if (-2.25 > xpos && xpos > -2.27)
    //     {
    //         agv_station_msg.set_data("ks4");
    //     }
    //     else if (-5.59 > xpos && xpos > -5.61)
    //     {
    //         agv_station_msg.set_data("as3");
    //     }
    //     else if (-10.58 > xpos && xpos > -10.60)
    //     {
    //         agv_station_msg.set_data("as4");
    //     }
    // }

    // //--publish the current station for the current AGV
    // this->dataPtr->AGVStationPublisher->Publish(agv_station_msg);

    float yaw = -1.570796;
    float height = -0.02f;

    /**
    * ======================================
    * KS1/KS2/KS3/KS4 to AS1/AS3
    * ======================================
    */
    speedFactor = 2;

    this->dataPtr->KS_to_AS1AS3_animation.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agvName, 6 / speedFactor, false));

    gazebo::common::PoseKeyFrame *key = this->dataPtr->KS_to_AS1AS3_animation->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(-2.265685, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS1AS3_animation->CreateKeyFrame(2 / speedFactor);
    key->Translation(ignition::math::Vector3d(-3.361776, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS1AS3_animation->CreateKeyFrame(4 / speedFactor);
    key->Translation(ignition::math::Vector3d(-4.457867, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS1AS3_animation->CreateKeyFrame(6 / speedFactor);
    key->Translation(ignition::math::Vector3d(-5.60, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    /**
    * ======================================
    * KS1/KS2/KS3/KS4 to AS2/AS5
    * ======================================
    */
    speedFactor = 2.5;

    this->dataPtr->KS_to_AS2AS4_animation.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->agvName, 12 / speedFactor, false));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(-2.265685, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(2 / speedFactor);
    key->Translation(ignition::math::Vector3d(-3.6531165, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(4 / speedFactor);
    key->Translation(ignition::math::Vector3d(-5.040548, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(6 / speedFactor);
    key->Translation(ignition::math::Vector3d(-6.4279795, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(8 / speedFactor);
    key->Translation(ignition::math::Vector3d(-7.815411, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(10 / speedFactor);
    key->Translation(ignition::math::Vector3d(-9.2028425, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    key = this->dataPtr->KS_to_AS2AS4_animation->CreateKeyFrame(12 / speedFactor);
    key->Translation(ignition::math::Vector3d(-10.590274, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    this->dataPtr->model = _parent;
    
    /*
    * =====================================================
    * Going from Assembly Station  to Kitting Station
    * =====================================================
    */
    /**
    * ===========================
    * AS1/AS3 to KS1/KS2/KS3/KS4
    * ===========================
    */
    // speedFactor = 2;

    // this->dataPtr->AS1AS3_to_KS_animation.reset(
    //     new gazebo::common::PoseAnimation(this->dataPtr->agvName, 6 / speedFactor, false));

    // key = this->dataPtr->AS1AS3_to_KS_animation->CreateKeyFrame(0);
    // key->Translation(ignition::math::Vector3d(-5.60, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS1AS3_to_KS_animation->CreateKeyFrame(2 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-4.457867, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS1AS3_to_KS_animation->CreateKeyFrame(4 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-3.361776, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS1AS3_to_KS_animation->CreateKeyFrame(6 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-2.265685, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    /**
    * ===========================
    * AS2/AS5 to KS1/KS2/KS3/KS4
    * ===========================
    */

    // speedFactor = 2.5;

    // this->dataPtr->AS2AS4_to_KS_animation.reset(
    //     new gazebo::common::PoseAnimation(this->dataPtr->agvName, 12 / speedFactor, false));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(0);
    // key->Translation(ignition::math::Vector3d(-10.590274, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(2 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-9.2028425, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(4 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-7.815411, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(6 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-6.4279795, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(8 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-5.040548, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(10 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-3.6531165, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS2AS4_to_KS_animation->CreateKeyFrame(12 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-2.265685, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    /**
    * ===========================
    * AS3/AS6 to KS1/KS2/KS3/KS4
    * ===========================
    */

    // speedFactor = 2.5;

    // this->dataPtr->AS3AS6_to_KS_animation.reset(
    //     new gazebo::common::PoseAnimation(this->dataPtr->agvName, 12 / speedFactor, false));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(0);
    // key->Translation(ignition::math::Vector3d(-15.580548, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(2 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-13.3614, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(4 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-11.142257, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(6 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-8.923114, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(8 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-6.703971, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(10 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-4.484828, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    // key = this->dataPtr->AS3AS6_to_KS_animation->CreateKeyFrame(12 / speedFactor);
    // key->Translation(ignition::math::Vector3d(-2.265685, ypos, height));
    // key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    /**
 * =========================================
 * Advertise services
 * =========================================
 */
    if (this->dataPtr->model->GetName() == "agv1" || this->dataPtr->model->GetName() == "agv2")
    {
        this->dataPtr->rosServiceAGVToAS1 = this->dataPtr->rosnode->advertiseService(AGVToAS1Service,
                                                                                      &ROSAGVPlugin::OnCommandAGVToAS1, this);
        this->dataPtr->rosServiceAGVToAS2 = this->dataPtr->rosnode->advertiseService(AGVToAS2Service,
                                                                                      &ROSAGVPlugin::OnCommandAGVToAS2, this);
        // this->dataPtr->rosServiceAGVToAS3 = this->dataPtr->rosnode->advertiseService(AGVToAS3Service,
        //                                                                               &ROSAGVPlugin::OnCommandAGVToAS3, this);
    }

       if (this->dataPtr->model->GetName() == "agv3" || this->dataPtr->model->GetName() == "agv4")
    {
        this->dataPtr->rosServiceAGVToAS3 = this->dataPtr->rosnode->advertiseService(AGVToAS3Service,
                                                                                      &ROSAGVPlugin::OnCommandAGVToAS3, this);
        this->dataPtr->rosServiceAGVToAS4 = this->dataPtr->rosnode->advertiseService(AGVToAS4Service,
                                                                                      &ROSAGVPlugin::OnCommandAGVToAS4, this);
        // this->dataPtr->rosServiceAGVToAS6 = this->dataPtr->rosnode->advertiseService(AGVToAS6Service,
        //                                                                               &ROSAGVPlugin::OnCommandAGVToAS6, this);
    }

    // }
    // this->dataPtr->rosServiceAssembly = this->dataPtr->rosnode->advertiseService(toAssemblyServiceName,
    //  &ROSAGVPlugin::OnCommandToAssemblyStation, this);

    // this->dataPtr->rosServiceKitting =
    //     this->dataPtr->rosnode->advertiseService(toKittingServiceName,
    //                                              &ROSAGVPlugin::OnCommandToKittingStation, this);

    // Client for clearing trays.
    //@todo: removing for ARIAC2021
    // this->dataPtr->rosClearTrayClient =
    //     this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(clearTrayServiceName);

    // this->dataPtr->rosAGVToAssemblyClient = this->dataPtr->rosnode->serviceClient<nist_gear::AGVToAssemblyStation>(assemblyStationService1);

    // Publisher for the status of the AGV.
    std::string stateTopic = "/ariac/" + this->dataPtr->agvName + "/state";
    this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
        std_msgs::String>(stateTopic, 1000);

    std::string stationTopic = "/ariac/" + this->dataPtr->agvName + "/station";
    this->dataPtr->stationPub = this->dataPtr->rosnode->advertise<
        std_msgs::String>(stationTopic, 1000);

    // std::string stateTopic = "/ariac/" + this->dataPtr->agvName + "/state";

    this->dataPtr->currentState = "ready_to_deliver";

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ROSAGVPlugin::OnUpdate, this, _1));
}


void ROSAGVPlugin::OnAGVLocation(std_msgs::String::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->current_station = _msg->data;
    gzdbg << "current_station: "<< this->dataPtr->current_station << "\n";
}

/////////////////////////////////////////////////
void ROSAGVPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    auto currentSimTime = _info.simTime;
    if (this->dataPtr->currentState == "ready_to_deliver")
    {
        /**
       * =============================================
       * Tasking an AGV to go to an assembly station
       * =============================================
       */
        if (this->dataPtr->goToAssemblyStationTriggered)
        {
            this->dataPtr->currentState = "go_to_assembly_station";
            this->dataPtr->goToAssemblyStationTriggerTime = currentSimTime;
            gzdbg << "goToAssemblyStationTriggerTime: " << this->dataPtr->goToAssemblyStationTriggerTime << "\n";
        }
        this->dataPtr->goToAssemblyStationTriggered = false;

        /**
       * =================================================
       * Tasking an AGV to go back to its kitting station
       * =================================================
       */
        // if (this->dataPtr->goToKittingStationTriggered)
        // {
        //     this->dataPtr->currentState = "go_to_kitting_station";
        //     this->dataPtr->goToKittingStationTriggerTime = currentSimTime;
        // }
        // this->dataPtr->goToKittingStationTriggered = false;
    }
    /**
     * ======================================================
     * Logic to choose the path to go to an assembly station
     * ======================================================
     */
    // if (this->dataPtr->currentState == "go_to_assembly_station")
    // {
    //     gzdbg << "go_to_assembly_station\n";

    //     // if (currentSimTime - this->dataPtr->goToAssemblyStationTriggerTime > 0.75)
    //     // {
            
    //         // Make a request to lock the models to the tray
    //         gazebo::msgs::GzString lock_msg;
    //         lock_msg.set_data("lock");
    //         this->dataPtr->lockUnlockTrayModelsPub->Publish(lock_msg);

    //         //--select animation based on the name of the station
    //         std::string current_station{};
    //         if (this->dataPtr->agvName == "agv1")
    //         {
    //             // ROS_INFO_STREAM("AGV1");
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv1Param, current_station);
    //         }
    //         else if (this->dataPtr->agvName == "agv2")
    //         {
    //             // gzdbg << "Go to assembly station\n";
                
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv2Param, current_station);
    //             gzdbg << "agv2::"<<current_station<<"\n";
    //         }
    //         else if (this->dataPtr->agvName == "agv3")
    //         {
    //             // ROS_INFO_STREAM("AGV3");
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv3Param, current_station);
    //         }
    //         else if (this->dataPtr->agvName == "agv4")
    //         {
    //             // ROS_INFO_STREAM("AGV4");
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv4Param, current_station);
    //         }

    //         //--Going from KS to AS
    //         if (current_station == "ks1" || current_station == "ks2" || current_station == "ks3" || current_station == "ks4")
    //         {
    //             if (this->dataPtr->assemblyStationName == "as1" || this->dataPtr->assemblyStationName == "as3")
    //             {
    //                 gzdbg << "Set animation KS_AS1AS3\n";
    //                 this->dataPtr->KS_to_AS1AS3_animation->SetTime(0);
    //                 this->dataPtr->model->SetAnimation(this->dataPtr->KS_to_AS1AS3_animation);
    //                 this->dataPtr->currentState = "KS_AS1AS3";
    //             }
    //             else if (this->dataPtr->assemblyStationName == "as2" || this->dataPtr->assemblyStationName == "as4")
    //             {
    //                 gzdbg << "Set animation KS_AS2AS4\n";
    //                 // ROS_INFO_STREAM("Current station: " << current_station);
    //                 this->dataPtr->KS_to_AS2AS4_animation->SetTime(0);
    //                 this->dataPtr->model->SetAnimation(this->dataPtr->KS_to_AS2AS4_animation);
    //                 this->dataPtr->currentState = "KS_AS2AS4";
    //             }
    //         }
    //         else{
    //             gzerr << "Current kitting station could not be retrieved\n";
    //         }

    //         //--Going from AS1/AS4 to other stations
    //         // if (current_station == "AS1" || current_station == "AS4")
    //         // {
    //         //     if (this->dataPtr->assemblyStationName == "AS2" || this->dataPtr->assemblyStationName == "AS5")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS1AS4_to_AS2AS5_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS1AS4_to_AS2AS5_animation);
    //         //         this->dataPtr->currentState = "AS1AS4_AS2AS5";
    //         //     }
    //         //     else if (this->dataPtr->assemblyStationName == "AS3" || this->dataPtr->assemblyStationName == "AS6")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS1AS4_to_AS3AS6_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS1AS4_to_AS3AS6_animation);
    //         //         this->dataPtr->currentState = "AS1AS4_AS3AS6";
    //         //     }
    //         // }

    //         //--Going from AS2/AS5 to other stations
    //         // if (current_station == "AS2" || current_station == "AS5")
    //         // {
    //         //     if (this->dataPtr->assemblyStationName == "AS1" || this->dataPtr->assemblyStationName == "AS4")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS2AS5_to_AS1AS4_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS2AS5_to_AS1AS4_animation);
    //         //         this->dataPtr->currentState = "AS2AS5_AS1AS4";
    //         //     }
    //         //     else if (this->dataPtr->assemblyStationName == "AS3" || this->dataPtr->assemblyStationName == "AS6")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS2AS5_to_AS3AS6_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS2AS5_to_AS3AS6_animation);
    //         //         this->dataPtr->currentState = "AS2AS5_AS3AS6";
    //         //     }
    //         // }
    //         //--Going from AS3/AS6 to other stations
    //         // if (current_station == "AS3" || current_station == "AS6")
    //         // {
    //         //     if (this->dataPtr->assemblyStationName == "AS1" || this->dataPtr->assemblyStationName == "AS4")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS3AS6_to_AS1AS4_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS3AS6_to_AS1AS4_animation);
    //         //         this->dataPtr->currentState = "AS3AS6_AS1AS4";
    //         //     }
    //         //     else if (this->dataPtr->assemblyStationName == "AS2" || this->dataPtr->assemblyStationName == "AS5")
    //         //     {
    //         //         // ROS_INFO_STREAM("Current station: " << current_station);
    //         //         this->dataPtr->AS3AS6_to_AS2AS5_animation->SetTime(0);
    //         //         this->dataPtr->model->SetAnimation(this->dataPtr->AS3AS6_to_AS2AS5_animation);
    //         //         this->dataPtr->currentState = "AS3AS6_AS2AS5";
    //         //     }
    //         // }
    //     // }
    // }

    /**
     * ======================================================
     * Logic to choose the path to go to a kitting station
     * ======================================================
     * Each AGV has its associated kitting station.
     * AGV1 can only go to kitting station: KS1
     * AGV2 can only go to kitting station: KS2
     * AGV3 can only go to kitting station: KS3
     * AGV4 can only go to kitting station: KS4
     */
    // if (this->dataPtr->currentState == "go_to_kitting_station")
    // {
    //     if (currentSimTime - this->dataPtr->goToKittingStationTriggerTime > 0.75)
    //     {
    //         // Make a request to lock the models to the tray.
    //         // There may be some cases where the AGVs return back to
    //         // the kitting stations with non-empty trays
    //         gazebo::msgs::GzString lock_msg;
    //         lock_msg.set_data("lock");
    //         this->dataPtr->lockUnlockTrayModelsPub->Publish(lock_msg);

    //         //--Get the current location of the AGV from the parameter server
    //         //--This is necessary to choose from predefined paths
    //         std::string current_station{};
    //         if (this->dataPtr->agvName == "agv1")
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv1Param, current_station);

    //         if (this->dataPtr->agvName == "agv2")
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv2Param, current_station);

    //         if (this->dataPtr->agvName == "agv3")
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv3Param, current_station);

    //         if (this->dataPtr->agvName == "agv4")
    //             this->dataPtr->rosnode->getParam(this->dataPtr->agv4Param, current_station);

    //         if (current_station == "as1" || current_station == "as3")
    //         {
    //             this->dataPtr->AS1AS3_to_KS_animation->SetTime(0);
    //             this->dataPtr->model->SetAnimation(this->dataPtr->AS1AS3_to_KS_animation);
    //             this->dataPtr->currentState = "AS1AS3_KS";
    //         }
    //         else if (current_station == "as2" || current_station == "as4")
    //         {
    //             this->dataPtr->AS2AS4_to_KS_animation->SetTime(0);
    //             this->dataPtr->model->SetAnimation(this->dataPtr->AS2AS4_to_KS_animation);
    //             this->dataPtr->currentState = "AS2AS4_KS";
    //         }
    //         gzdbg << "AGV is en route to Kitting Station.\n";
    //     }
    // }

    if (this->dataPtr->currentState == "go_to_assembly_station")
    {
        // gzdbg << "go_to_assembly_station\n";
        if (currentSimTime - this->dataPtr->goToAssemblyStationTriggerTime > 0.75)
        {
            // Make a request to lock the models to the tray
            gazebo::msgs::GzString lock_msg;
            lock_msg.set_data("lock");
            this->dataPtr->lockUnlockTrayModelsPub->Publish(lock_msg);

            //--select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;
            // gzdbg << "CURRENT STATION: " << current_station << "\n";

            //--Going from KS to AS
            if (current_station == "ks1" || current_station == "ks2" || current_station == "ks3" || current_station == "ks4")
            {
                if (this->dataPtr->assemblyStationName == "as1" || this->dataPtr->assemblyStationName == "as3")
                {
                    // gzdbg << "Set animation KS_AS1AS3\n";
                    this->dataPtr->KS_to_AS1AS3_animation->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->KS_to_AS1AS3_animation);
                    this->dataPtr->currentState = "KS_AS1AS3";
                }
                else if (this->dataPtr->assemblyStationName == "as2" || this->dataPtr->assemblyStationName == "as4")
                {
                    // gzdbg << "Set animation KS_AS2AS4\n";
                    // ROS_INFO_STREAM("Current station: " << current_station);
                    this->dataPtr->KS_to_AS2AS4_animation->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->KS_to_AS2AS4_animation);
                    this->dataPtr->currentState = "KS_AS2AS4";
                }
            }
            else
            {
                gzerr << "Current kitting station could not be retrieved\n";
            }
        }
    }
    //--Current state: KS_AS1AS4
    if (this->dataPtr->currentState == "KS_AS1AS3")
    {
        bool goToASAnimationDone = this->dataPtr->KS_to_AS1AS3_animation->GetTime() >=
                                    this->dataPtr->KS_to_AS1AS3_animation->GetLength();
        if (goToASAnimationDone)
        {
            gzdbg << "Docking animation finished." << std::endl;
            this->dataPtr->currentState = "docked_to_assembly_station";
            // gazebo::msgs::GzString unlock_msg;
            // unlock_msg.set_data("unlock");
            // this->dataPtr->lockUnlockTrayModelsPub->Publish(unlock_msg);
        }
    }

    //--Current state: KS_AS2AS5
    if (this->dataPtr->currentState == "KS_AS2AS4")
    {
        // Wait until AGV is away from potential user interference
        // if (!this->dataPtr->gravityDisabled && this->dataPtr->KS_to_AS2AS4_animation->GetTime() >= 0.2)
        // {
        //     // Parts will fall through the tray during the animation unless gravity is disabled on the AGV
        //     gzdbg << "Disabling gravity on model: " << this->dataPtr->agvName << std::endl;
        //     ROS_INFO_STREAM("Disabling gravity on model: " << this->dataPtr->agvName);
        //     // this->dataPtr->model->SetGravityMode(false);
        //     // this->dataPtr->gravityDisabled = true;
        // }
        bool goToASAnimationDone = this->dataPtr->KS_to_AS2AS4_animation->GetTime() >=
                                    this->dataPtr->KS_to_AS2AS4_animation->GetLength();
        if (goToASAnimationDone)
        {
            gzdbg << "Docking animation finished." << std::endl;
            this->dataPtr->currentState = "docked_to_assembly_station";
            // gazebo::msgs::GzString unlock_msg;
            // unlock_msg.set_data("unlock");
            // this->dataPtr->lockUnlockTrayModelsPub->Publish(unlock_msg);
        }
    }

    if (this->dataPtr->currentState == "docked_to_assembly_station")
    {

        // this->dataPtr->model->SetGravityMode(true);
        
        gzdbg << "Docked to station." << std::endl;

        //--publish the new station on the appropriate topic
        std_msgs::String msg;
        msg.data=this->dataPtr->assemblyStationName;
        this->dataPtr->stationPub.publish(msg);

        //--wait for 2 s and then issue idle state for lockunlock topic
        // ros::Duration(1.0).sleep();
        gazebo::msgs::GzString unlock_msg;
        unlock_msg.set_data("unlock");
        this->dataPtr->lockUnlockTrayModelsPub->Publish(unlock_msg);
        // gazebo::msgs::GzString idle_msg;
        // unlock_msg.set_data("idle");
        // this->dataPtr->lockUnlockTrayModelsPub->Publish(idle_msg);

        this->dataPtr->currentState = "ready_to_deliver";

        
    }
    std_msgs::String stateMsg;
    stateMsg.data = this->dataPtr->currentState;
    this->dataPtr->statePub.publish(stateMsg);
}


bool ROSAGVPlugin::OnCommandAGVToAS1(
    std_srvs::Trigger::Request &,
    std_srvs::Trigger::Response &_res)
{
    std::string current_station;
    std::string parameter = "/ariac/" + this->dataPtr->agvName + "_station";
    this->dataPtr->rosnode->getParam(parameter, current_station);

   gzdbg << "[INFO] Triggered: '" << this->dataPtr->agvName << "': "<< current_station<<"--> as1";

    if (current_station == "as1")
    {
        _res.message = "[" + this->dataPtr->agvName + " already at as1] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    if (this->dataPtr->currentState != "ready_to_deliver")
    {
        _res.message = "[" + this->dataPtr->agvName + "-> as1] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    _res.success = true;
    _res.message = "[" + this->dataPtr->agvName + "-> as1] SUCCESS: AGV successfully triggered.";
    ROS_INFO_STREAM(_res.message);
    this->dataPtr->goToAssemblyStationTriggered = true;

    this->dataPtr->assemblyStationName = "as1";

    return true;
}


bool ROSAGVPlugin::OnCommandAGVToAS2(
    std_srvs::Trigger::Request &,
    std_srvs::Trigger::Response &_res)
{
    std::string current_station;
    std::string parameter = "/ariac/" + this->dataPtr->agvName + "_station";
    this->dataPtr->rosnode->getParam(parameter, current_station);

    gzdbg << "[INFO] Triggered: '" << this->dataPtr->agvName << "': "<< current_station<<"--> as2\n";

    if (current_station == "as2")
    {
        _res.message = "[" + this->dataPtr->agvName + " already at as2] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    if (this->dataPtr->currentState != "ready_to_deliver")
    {
        _res.message = "[" + this->dataPtr->agvName + "-> as2] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    _res.success = true;
    _res.message = "[" + this->dataPtr->agvName + "-> as2] SUCCESS: AGV successfully triggered.";
    gzdbg << _res.message << "\n";
    ROS_INFO_STREAM(_res.message);
    this->dataPtr->goToAssemblyStationTriggered = true;

    this->dataPtr->assemblyStationName = "as2";

    return true;
}





bool ROSAGVPlugin::OnCommandAGVToAS3(
    std_srvs::Trigger::Request &,
    std_srvs::Trigger::Response &_res)
{
    std::string current_station;
    std::string parameter = "/ariac/" + this->dataPtr->agvName + "_station";
    this->dataPtr->rosnode->getParam(parameter, current_station);

    gzdbg << "[INFO] Triggered: '" << this->dataPtr->agvName << "': "<< current_station<<"--> as3";

    if (current_station == "as3")
    {
        _res.message = "[" + this->dataPtr->agvName + " already at as3] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    if (this->dataPtr->currentState != "ready_to_deliver")
    {
        _res.message = "[" + this->dataPtr->agvName + "-> as3] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    _res.success = true;
    _res.message = "[" + this->dataPtr->agvName + "-> as3] SUCCESS: AGV successfully triggered.";
    ROS_INFO_STREAM(_res.message);
    this->dataPtr->goToAssemblyStationTriggered = true;

    this->dataPtr->assemblyStationName = "as3";

    return true;
}

bool ROSAGVPlugin::OnCommandAGVToAS4(
    std_srvs::Trigger::Request &,
    std_srvs::Trigger::Response &_res)
{
    std::string current_station;
    std::string parameter = "/ariac/" + this->dataPtr->agvName + "_station";
    this->dataPtr->rosnode->getParam(parameter, current_station);

    gzdbg << "[INFO] Triggered: '" << this->dataPtr->agvName << "': "<< current_station<<"--> as4";

    if (current_station == "as4")
    {
        _res.message = "[" + this->dataPtr->agvName + " already at as4] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    if (this->dataPtr->currentState != "ready_to_deliver")
    {
        _res.message = "[" + this->dataPtr->agvName + "-> as4] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(_res.message);
        _res.success = false;
        return true;
    }

    _res.success = true;
    _res.message = "[" + this->dataPtr->agvName + "-> as4] SUCCESS: AGV successfully triggered.";
    ROS_INFO_STREAM(_res.message);
    this->dataPtr->goToAssemblyStationTriggered = true;

    this->dataPtr->assemblyStationName = "as4";

    return true;
}



bool ROSAGVPlugin::OnCommandToAssemblyStation(
    nist_gear::AGVToAssemblyStation::Request &req,
    nist_gear::AGVToAssemblyStation::Response &res)
{

    std::string current_station;
    std::string parameter = "/ariac/" + this->dataPtr->agvName + "_station";
    this->dataPtr->rosnode->getParam(parameter, current_station);

    ROS_WARN_STREAM("[INFO] AGV '" << this->dataPtr->agvName << "' tasked to go to assembly station " << req.assembly_station_name);

    if (current_station == req.assembly_station_name)
    {
        res.message = "[" + this->dataPtr->agvName + " already at station: " + req.assembly_station_name + "] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(res.message);
        res.success = false;
        return true;
    }

    if (this->dataPtr->agvName == "agv1" || this->dataPtr->agvName == "agv2")
        if (req.assembly_station_name.compare("as1") != 0 &&
            req.assembly_station_name.compare("as2") != 0)
            // req.assembly_station_name.compare("AS3") != 0)
        {
            res.message = "[" + this->dataPtr->agvName + " can not reach station: " + req.assembly_station_name + "] FAILURE: AGV not successfully triggered.";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

    if (this->dataPtr->agvName == "agv3" || this->dataPtr->agvName == "agv4")
        if (req.assembly_station_name.compare("as3") != 0 &&
            req.assembly_station_name.compare("as4") != 0)
            // req.assembly_station_name.compare("AS6") != 0)
        {
            res.message = "[" + this->dataPtr->agvName + " can not reach station: " + req.assembly_station_name + "] FAILURE: AGV not successfully triggered.";
            ROS_ERROR_STREAM(res.message);
            res.success = false;
            return true;
        }

    if (this->dataPtr->currentState != "ready_to_deliver")
    {
        res.message = "[" + this->dataPtr->agvName + "->" + req.assembly_station_name + "] FAILURE: AGV not successfully triggered.";
        ROS_ERROR_STREAM(res.message);
        res.success = false;
        return true;
    }

    res.success = true;
    res.message = "[" + this->dataPtr->agvName + "->" + req.assembly_station_name + "] SUCCESS: AGV successfully triggered.";
    ROS_INFO_STREAM(res.message);
    this->dataPtr->goToAssemblyStationTriggered = true;

    this->dataPtr->assemblyStationName = req.assembly_station_name;

    return true;
}

