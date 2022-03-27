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
#include "ROSPersonByStationPlugin.hh"
#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <string>

namespace gazebo {
    /**
     * @internal
     * @brief Private data for the ROSPersonByStation Plugin class
     */
    class ROSPersonByStationPluginPrivate {
    public:
        //! \brief Mutex for subscribers
        std::mutex mutex;
         //! \brief Name of the person model
        std::string personName;
         //! \brief World pointer
        physics::WorldPtr world;
         //! \brief Pointer to the update event connection
        event::ConnectionPtr updateConnection;
         //! \brief ROS namespace
        std::string robotNamespace;
         //! \brief ROS node handle
        ros::NodeHandle* rosNode;
         //! \brief Transportation Node
        transport::NodePtr gzNode;
         //! \brief Person model animation on the floor
        gazebo::common::PoseAnimationPtr personAnimation;
         //! \brief Pointer to the model
        gazebo::physics::ModelPtr model;
         //! \brief Publisher for the model's state
        ros::Publisher modelStatePub;
         //! \brief Time at which the person starts moving
        float startTime;
         //! \brief How long the person waits before moving out of the way
        float waitTime;
         //! \brief Subscriber for /ariac/as2/occupied
        ros::Subscriber as2StatusSubscriber;
         //! \brief Subscriber for /ariac/as4/occupied
        ros::Subscriber as4StatusSubscriber;
        //! \brief Name of the model (retrieved from the world file)
        std::string modelName;
        /**
         * @brief State of a person
         *The possibilities are:
         * - "at_station"
         * - "go_away"
         */ 
        std::string currentState;
         //! \brief Simulation time for when the person was triggered to move away from station 
        common::Time goAwayTriggerTime;
    };//--end of class
}//--end of namespace

using namespace gazebo;


/////////////////////////////////////////////////
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(ROSPersonByStationPlugin);

/////////////////////////////////////////////////
ROSPersonByStationPlugin::ROSPersonByStationPlugin()
    : dataPtr(new ROSPersonByStationPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSPersonByStationPlugin::~ROSPersonByStationPlugin()
{
    this->dataPtr->rosNode->shutdown();
}

/////////////////////////////////////////////////
void ROSPersonByStationPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

    this->dataPtr->world = _parent->GetWorld();

    // Load parameters
    this->dataPtr->robotNamespace = "";
    if (_sdf->HasElement("robotNamespace")) {
        this->dataPtr->robotNamespace = _sdf->GetElement(
            "robotNamespace")->Get<std::string>() + "/";
        // ROS_WARN_STREAM("robotNamespace: " << this->dataPtr->robotNamespace << "\n");
    }

    this->dataPtr->modelName = "";
    if (_sdf->HasElement("model_name")) {
        this->dataPtr->modelName = _sdf->GetElement(
            "model_name")->Get<std::string>();
    }

    this->dataPtr->waitTime = 0;
    if (_sdf->HasElement("wait_time")) {
        this->dataPtr->waitTime = _sdf->GetElement(
            "wait_time")->Get<float>();
    }

    //Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
            << "unable to load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    //Initialize a ROS Node
    this->dataPtr->rosNode = new ros::NodeHandle(this->dataPtr->robotNamespace);
    
    //Current state when the plugin starts
    this->dataPtr->currentState = "at_station";

    //Initialize Gazebo transport
    this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
    this->dataPtr->gzNode->Init();

    // float waitTime = this->dataPtr->waitTime;
    float moveTime = 5.;
    float aisleLength = 2.695137;

    double speedFactor = 1.; //1.2;
    this->dataPtr->personAnimation.reset(
        new gazebo::common::PoseAnimation(this->dataPtr->modelName, moveTime / speedFactor, false));

    
    float yaw = -1.57;
    float height = -0.02f;
    float start = _parent->WorldPose().Pos().X();
    float ypos = _parent->WorldPose().Pos().Y();


    gazebo::common::PoseKeyFrame* key = this->dataPtr->personAnimation->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(start, ypos, height));
    key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

    for (int i = 1; i < 11; i++) {
        key = this->dataPtr->personAnimation->CreateKeyFrame(i / 10. * moveTime / speedFactor);
        key->Translation(ignition::math::Vector3d(start + i / 10. * aisleLength, ypos, height));
        key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
    }


    this->dataPtr->model = _parent;

    // Publisher for the status of the person.
    std::string stateTopic = "/ariac/" + this->dataPtr->modelName + "/state";
    this->dataPtr->modelStatePub = this->dataPtr->rosNode->advertise<
        std_msgs::String>(stateTopic, 1000);

    this->dataPtr->as2StatusSubscriber =
        this->dataPtr->rosNode->subscribe("/ariac/as2/occupied",
            10,
            &ROSPersonByStationPlugin::AS2Callback, this);

    this->dataPtr->as4StatusSubscriber =
        this->dataPtr->rosNode->subscribe("/ariac/as4/occupied",
            10,
            &ROSPersonByStationPlugin::AS4Callback, this);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ROSPersonByStationPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ROSPersonByStationPlugin::AS2Callback(std_msgs::Bool::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (this->dataPtr->currentState == "at_station") { 
        if (this->dataPtr->modelName == "person_1_ariac") {
            if ((bool)_msg->data == true) {
                this->dataPtr->currentState = "agv_has_arrived";
            }
        }
    }
}

/////////////////////////////////////////////////
void ROSPersonByStationPlugin::AS4Callback(std_msgs::Bool::ConstPtr _msg)
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (this->dataPtr->currentState == "at_station") {
        if (this->dataPtr->modelName == "person_2_ariac") {
            if ((bool)_msg->data == true) {
                this->dataPtr->currentState = "agv_has_arrived";
            }
        }
    }
}

/////////////////////////////////////////////////
void ROSPersonByStationPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    auto currentSimTime = _info.simTime;

    if (this->dataPtr->currentState == "agv_has_arrived") {
        // ROS_FATAL_STREAM("agv_has_arrived");
        this->dataPtr->currentState = "tasked_to_move_away";
        this->dataPtr->goAwayTriggerTime = currentSimTime;
    }

    if (this->dataPtr->currentState == "tasked_to_move_away") {
        if (currentSimTime - this->dataPtr->goAwayTriggerTime >= this->dataPtr->waitTime) {
            this->dataPtr->currentState = "moved_away";
            this->dataPtr->model->SetAnimation(this->dataPtr->personAnimation);
            std_msgs::String stateMsg;
            stateMsg.data = this->dataPtr->currentState;
            this->dataPtr->modelStatePub.publish(stateMsg);
        }
    }
}
