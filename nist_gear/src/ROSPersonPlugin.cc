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
#include "ROSPersonPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>
#include <std_msgs/String.h>

#include <string>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSPersonPlugin class.
  class ROSPersonPluginPrivate{
  public:
    /// \brief Name of the Person
    std::string personName;

    /// \brief World pointer
    physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    /// \brief for setting ROS name space
    std::string robotNamespace;

    /// \brief ros node handle
    ros::NodeHandle *rosnode;

    /// \brief Transportation node.
    transport::NodePtr gzNode;

    /// \brief Person animation on the floor
    gazebo::common::PoseAnimationPtr backAndForthAnimation;
    //public: gazebo::common::PoseAnimationPtr deliverTrayAnimation;

    /// \brief Pointer to the model
    gazebo::physics::ModelPtr model;

    /// \brief The state of the person
    std::string currentState;

    /// \brief Whether or not gravity of the Model has been disabled
    bool gravityDisabled;

    /// \brief Publishes the Model state.
    ros::Publisher statePub;

    // \brief When the person starts moving
    float startTime;

    // \brief How long the person waits at either end
    float waitTime;

    // \brief How long the person takes to move end-to-end
    float moveTime;
  };//--end of class
}//--end of namespace

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSPersonPlugin);

/////////////////////////////////////////////////
ROSPersonPlugin::ROSPersonPlugin()
  : dataPtr(new ROSPersonPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSPersonPlugin::~ROSPersonPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSPersonPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  this->dataPtr->world = _parent->GetWorld();

  // load parameters
  this->dataPtr->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace")){
    this->dataPtr->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
        ROS_WARN_STREAM("robotNamespace: " << this->dataPtr->robotNamespace << "\n");
  }

  this->dataPtr->startTime = 0;
  if (_sdf->HasElement("start_time")){
    this->dataPtr->startTime = _sdf->GetElement(
        "start_time")->Get<float>();
  }

  this->dataPtr->waitTime = 0;
  if (_sdf->HasElement("wait_time")){
    this->dataPtr->waitTime = _sdf->GetElement(
        "wait_time")->Get<float>();
  }

  this->dataPtr->moveTime = 0;
  if (_sdf->HasElement("move_time")){
    this->dataPtr->moveTime = _sdf->GetElement(
        "move_time")->Get<float>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->dataPtr->personName = std::string("person");

  this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->robotNamespace);

  // Initialize Gazebo transport
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();

  float waitTime = this->dataPtr->waitTime;
  float moveTime = this->dataPtr->moveTime;
  float totalTime = 2 * (waitTime + moveTime);
  float aisleLength = 15.;

  double speedFactor = 1.; //1.2;
  this->dataPtr->backAndForthAnimation.reset(
    new gazebo::common::PoseAnimation(this->dataPtr->personName,totalTime/speedFactor, true));
  //
  // float sign = index == "1" ? 1.0 : -1.0;
  // float yaw = index == "1" ? 3.1415 : 0.0;
  float yaw = -1.57;
  float height = -0.02f;
  float start = _parent->WorldPose().Pos().X();
  float ypos = _parent->WorldPose().Pos().Y();


  //old num 1.495831
  gazebo::common::PoseKeyFrame *key = this->dataPtr->backAndForthAnimation->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(start, ypos, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

  for (int i = 1; i < 11; i++)
  {
      key = this->dataPtr->backAndForthAnimation->CreateKeyFrame(i/10. * moveTime/speedFactor);
      key->Translation(ignition::math::Vector3d(start - i/10. * aisleLength, ypos, height));
      key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
  }

  for (int i = 1; i < 11; i++)
  {
      key = this->dataPtr->backAndForthAnimation->CreateKeyFrame((moveTime + i/10. * waitTime)/speedFactor);
      key->Translation(ignition::math::Vector3d(start - aisleLength, ypos, height));
      key->Rotation(ignition::math::Quaterniond(0, 0, -yaw));
  }

  for (int i = 1; i < 11; i++)
  {
      key = this->dataPtr->backAndForthAnimation->CreateKeyFrame((moveTime + waitTime + i/10. * moveTime)/speedFactor);
      key->Translation(ignition::math::Vector3d(start - aisleLength + i/10. * aisleLength, ypos, height));
      key->Rotation(ignition::math::Quaterniond(0, 0, -yaw));
  }

  for (int i = 1; i < 11; i++)
  {
      key = this->dataPtr->backAndForthAnimation->CreateKeyFrame((2 * moveTime + waitTime + i/10. * waitTime)/speedFactor);
      key->Translation(ignition::math::Vector3d(start, ypos, height));
      key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
  }

  this->dataPtr->model = _parent;

  // Publisher for the status of the person.
  std::string stateTopic = "/ariac/" + this->dataPtr->personName + "/state";
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(stateTopic, 1000);

  this->dataPtr->currentState = "person_ready";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ROSPersonPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ROSPersonPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  auto currentSimTime = _info.simTime;

  if (this->dataPtr->currentState == "person_ready")
  {
    if (common::Time(this->dataPtr->startTime) - currentSimTime < 0
	 && this->dataPtr->currentState != "person_moving")
    {
      this->dataPtr->model->SetAnimation(this->dataPtr->backAndForthAnimation);
      this->dataPtr->currentState = "person_moving";
      // ROS_WARN_STREAM("Person movement successfully triggered.");
      // ROS_WARN_STREAM("Person moving on the floor: " << this->dataPtr->personName << "\n");
      // gzdbg << "Person moving on the floor: " << this->dataPtr->personName << std::endl;
      std_msgs::String stateMsg;
      stateMsg.data = this->dataPtr->currentState;
      this->dataPtr->statePub.publish(stateMsg);
    }

  }
}
