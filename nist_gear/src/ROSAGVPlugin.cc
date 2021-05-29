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
}  // namespace gazebo

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
    if (_sdf->HasElement("agv_location_topic_name"))
    {
        AGVStationTopicName = _sdf->Get<std::string>("agv_location_topic_name");
    }

    std::string AGVToAS1Service = "to_as1";
    if (_sdf->HasElement("to_as1_name"))
        AGVToAS1Service = _sdf->Get<std::string>("to_as1_name");
    // ROS_DEBUG_STREAM("Using AGVToAS1Service service: " << AGVToAS1Service);

    std::string AGVToAS2Service = "to_as2";
    if (_sdf->HasElement("to_as2_name"))
        AGVToAS2Service = _sdf->Get<std::string>("to_as2_name");
    // ROS_DEBUG_STREAM("Using AGVToAS2Service service: " << AGVToAS2Service);

    std::string AGVToAS3Service = "to_as3";
    if (_sdf->HasElement("to_as3_name"))
        AGVToAS3Service = _sdf->Get<std::string>("to_as3_name");
    // ROS_DEBUG_STREAM("Using AGVToAS3Service service: " << AGVToAS3Service);

    std::string AGVToAS4Service = "to_as4";
    if (_sdf->HasElement("to_as4_name"))
        AGVToAS4Service = _sdf->Get<std::string>("to_as4_name");
    // ROS_DEBUG_STREAM("Using AGVToAS4Service service: " << AGVToAS4Service);

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

    // ROS publisher to send "lock" or "unlock" messages to topic
    this->dataPtr->lockUnlockTrayModelsPub =
        this->dataPtr->gzNode->Advertise<msgs::GzString>(lockUnlockTrayServiceName);

    double speedFactor = 1.2;

    // All 4 AGVs have the same y position and the same rpy during animation
    float ypos = _parent->WorldPose().Pos().Y();
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
    }

    if (this->dataPtr->model->GetName() == "agv3" || this->dataPtr->model->GetName() == "agv4")
    {
        this->dataPtr->rosServiceAGVToAS3 = this->dataPtr->rosnode->advertiseService(AGVToAS3Service,
                                                                                     &ROSAGVPlugin::OnCommandAGVToAS3, this);
        this->dataPtr->rosServiceAGVToAS4 = this->dataPtr->rosnode->advertiseService(AGVToAS4Service,
                                                                                     &ROSAGVPlugin::OnCommandAGVToAS4, this);
    }

    // Publisher for the status of the AGV.
    std::string stateTopic = "/ariac/" + this->dataPtr->agvName + "/state";
    this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
        std_msgs::String>(stateTopic, 1000);

    std::string stationTopic = "/ariac/" + this->dataPtr->agvName + "/station";
    this->dataPtr->stationPub = this->dataPtr->rosnode->advertise<
        std_msgs::String>(stationTopic, 1000);

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
        }
        this->dataPtr->goToAssemblyStationTriggered = false;
    }

    if (this->dataPtr->currentState == "go_to_assembly_station")
    {
        if (currentSimTime - this->dataPtr->goToAssemblyStationTriggerTime > 0.75)
        {
            // make a request to lock the models to the tray
            gazebo::msgs::GzString lock_msg;
            lock_msg.set_data("lock");
            this->dataPtr->lockUnlockTrayModelsPub->Publish(lock_msg);

            // select animation based on the name of the station
            std::string current_station = this->dataPtr->current_station;

            // going from KS to AS
            if (current_station == "ks1" || current_station == "ks2" || current_station == "ks3" || current_station == "ks4")
            {
                if (this->dataPtr->assemblyStationName == "as1" || this->dataPtr->assemblyStationName == "as3")
                {
                    this->dataPtr->KS_to_AS1AS3_animation->SetTime(0);
                    this->dataPtr->model->SetAnimation(this->dataPtr->KS_to_AS1AS3_animation);
                    this->dataPtr->currentState = "KS_AS1AS3";
                }
                else if (this->dataPtr->assemblyStationName == "as2" || this->dataPtr->assemblyStationName == "as4")
                {
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
    // current state: KS_AS1AS4
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

    // Current state: KS_AS2AS5
    if (this->dataPtr->currentState == "KS_AS2AS4")
    {
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
        gzdbg << "Docked to station." << std::endl;

        // publish the new station on the appropriate topic
        std_msgs::String msg;
        msg.data = this->dataPtr->assemblyStationName;
        this->dataPtr->stationPub.publish(msg);

        // wait for 2 s and then issue idle state for lockunlock topic
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

    // gzdbg << "[INFO] Triggered: '" << this->dataPtr->agvName << "': "<< current_station<<"--> as2\n";

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
