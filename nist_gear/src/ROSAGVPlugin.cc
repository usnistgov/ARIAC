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

#include <string>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSAGVPlugin class.
  struct ROSAGVPluginPrivate
  {
    /// \brief Name of the AGV
    public: std::string agvName;

    /// \brief Scoped name of the link of the tray on the AGV
    public: std::string trayLinkName;

    /// \brief World pointer
    public: physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief for setting ROS name space
    public: std::string robotNamespace;

    /// \brief ros node handle
    public: ros::NodeHandle *rosnode;

    /// \brief Receives service calls for controlling the AGV
    public: ros::ServiceServer rosService;

    /// \brief Transportation node.
    public: transport::NodePtr gzNode;

    /// \brief Gazebo publish for locking parts to this AGV's tray
    public: transport::PublisherPtr lockTrayModelsPub;

    /// \brief Client for clearing this AGV's tray
    public: ros::ServiceClient rosClearTrayClient;

    /// \brief Robot animation for delivering the tray
    public: gazebo::common::PoseAnimationPtr deliverTrayAnimation;

    /// \brief Robot animation for the AGV returning to its base
    public: gazebo::common::PoseAnimationPtr returnAnimation;

    /// \brief Pointer to the model
    public: gazebo::physics::ModelPtr model;

    /// \brief The state of the AGV
    public: std::string currentState;

    /// \brief The time the last tray delivery was triggered
    public: common::Time deliveryTriggerTime;

    /// \brief Whether or not gravity of the AGV has been disabled
    public: bool gravityDisabled;

    /// \brief Flag for triggering tray delivery from the service callback
    public: bool deliveryTriggered = false;

    /// \brief Publishes the AGV state.
    public: ros::Publisher statePub;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSAGVPlugin);

/////////////////////////////////////////////////
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
  {
    index = _sdf->Get<std::string>("index");
  }
  else
  {
    gzerr << "AGV is missing an index. The AGV will not work.\n";
  }

  this->dataPtr->world = _parent->GetWorld();

  // load parameters
  this->dataPtr->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
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

  // Topic used to ask AGV to move
  std::string agvControlService = "animate";
  if (_sdf->HasElement("agv_control_service_name"))
    agvControlService = _sdf->Get<std::string>("agv_control_service_name");
  ROS_DEBUG_STREAM("Using AGV control service: " << agvControlService);

  std::string lockTrayServiceName = "lock_tray_models";
  if (_sdf->HasElement("lock_tray_service_name"))
    lockTrayServiceName = _sdf->Get<std::string>("lock_tray_service_name");
  ROS_DEBUG_STREAM("Using lock tray service topic: " << lockTrayServiceName);

  std::string clearTrayServiceName = "clear_tray";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearTrayServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  ROS_DEBUG_STREAM("Using clear tray service topic: " << clearTrayServiceName);

  this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->robotNamespace);

  // Initialize Gazebo transport
  this->dataPtr->gzNode = transport::NodePtr(new transport::Node());
  this->dataPtr->gzNode->Init();
  this->dataPtr->lockTrayModelsPub =
    this->dataPtr->gzNode->Advertise<msgs::GzString>(lockTrayServiceName);

  double speedFactor = 1.2;
  this->dataPtr->deliverTrayAnimation.reset(
    new gazebo::common::PoseAnimation(this->dataPtr->agvName,12/speedFactor, false));

  float sign = index == "1" ? 1.0 : -1.0;
  float yaw = index == "1" ? 3.1415 : 0.0;
  float height = -0.02f;
  gazebo::common::PoseKeyFrame *key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(2/speedFactor);
  key->Translation(ignition::math::Vector3d(-0.5, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(4/speedFactor);
  key->Translation(ignition::math::Vector3d(1., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(6/speedFactor);
  key->Translation(ignition::math::Vector3d(3., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(8/speedFactor);
  key->Translation(ignition::math::Vector3d(5., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(10/speedFactor);
  key->Translation(ignition::math::Vector3d(7., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->deliverTrayAnimation->CreateKeyFrame(12/speedFactor);
  key->Translation(ignition::math::Vector3d(9., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));


  this->dataPtr->returnAnimation.reset(
    new gazebo::common::PoseAnimation(this->dataPtr->agvName, 16/speedFactor, false));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(9., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw+sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(2/speedFactor);
  key->Translation(ignition::math::Vector3d(8.5, 7.764603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(4/speedFactor);
  key->Translation(ignition::math::Vector3d(7.5, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(6/speedFactor);
  key->Translation(ignition::math::Vector3d(6, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(8/speedFactor);
  key->Translation(ignition::math::Vector3d(4., 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(10/speedFactor);
  key->Translation(ignition::math::Vector3d(2, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(12/speedFactor);
  key->Translation(ignition::math::Vector3d(1, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(14/speedFactor);
  key->Translation(ignition::math::Vector3d(-0.5, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw-sign*1.571));

  key = this->dataPtr->returnAnimation->CreateKeyFrame(16/speedFactor);
  key->Translation(ignition::math::Vector3d(0, 7.264603*sign, height));
  key->Rotation(ignition::math::Quaterniond(0, 0, yaw));

  this->dataPtr->model = _parent;

  this->dataPtr->rosService = this->dataPtr->rosnode->advertiseService(agvControlService,
      &ROSAGVPlugin::OnCommand, this);

  // Client for clearing trays.
  this->dataPtr->rosClearTrayClient =
    this->dataPtr->rosnode->serviceClient<std_srvs::Trigger>(clearTrayServiceName);

  // Publisher for the status of the AGV.
  std::string stateTopic = "/ariac/" + this->dataPtr->agvName + "/state";
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(stateTopic, 1000);

  this->dataPtr->currentState = "ready_to_deliver";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ROSAGVPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ROSAGVPlugin::OnUpdate(const common::UpdateInfo & _info)
{
  auto currentSimTime = _info.simTime;
  if (this->dataPtr->currentState == "ready_to_deliver")
  {
    if (this->dataPtr->deliveryTriggered)
    {
      this->dataPtr->currentState = "preparing_to_deliver";
      this->dataPtr->deliveryTriggerTime = currentSimTime;
    }
    this->dataPtr->deliveryTriggered = false;
  }
  if (this->dataPtr->currentState == "preparing_to_deliver")
  {
    // Wait a bit to ensure the models have been detected by the kit tray's plugin
    if (currentSimTime - this->dataPtr->deliveryTriggerTime > 0.75)
    {
      // Make a request to lock the models to the tray
      gazebo::msgs::GzString lock_msg;
      lock_msg.set_data("lock");
      this->dataPtr->lockTrayModelsPub->Publish(lock_msg);

      // Trigger the tray delivery animation
      this->dataPtr->deliverTrayAnimation->SetTime(0);
      this->dataPtr->model->SetAnimation(this->dataPtr->deliverTrayAnimation);
      ROS_INFO_STREAM("AGV successfully triggered.");
      this->dataPtr->currentState = "delivering";
    }
  }
  if (this->dataPtr->currentState == "delivering")
  {
    // Wait until AGV is away from potential user interference
    if (!this->dataPtr->gravityDisabled && this->dataPtr->deliverTrayAnimation->GetTime() >= 0.5)
    {
      // Parts will fall through the tray during the animation unless gravity is disabled on the AGV
      gzdbg << "Disabling gravity on model: " << this->dataPtr->agvName << std::endl;
      this->dataPtr->model->SetGravityMode(false);
      this->dataPtr->gravityDisabled = true;
    }
    bool deliverTrayAnimationDone = this->dataPtr->deliverTrayAnimation->GetTime() >= \
      this->dataPtr->deliverTrayAnimation->GetLength();
    if (deliverTrayAnimationDone)
    {
      gzdbg << "Delivery animation finished." << std::endl;
      this->dataPtr->currentState = "delivered";
    }
  }
  if (this->dataPtr->currentState == "delivered")
  {
    // Make a service call to clear the tray.
    if (!this->dataPtr->rosClearTrayClient.exists())
    {
      this->dataPtr->rosClearTrayClient.waitForExistence();
    }
    std_srvs::Trigger clear_srv;
    this->dataPtr->rosClearTrayClient.call(clear_srv);
    if (!clear_srv.response.success)
    {
      ROS_ERROR_STREAM("Failed to clear tray.");
    }
    else
    {
      ROS_DEBUG_STREAM("Tray successfully cleared.");
    }
    this->dataPtr->model->SetGravityMode(true);
    this->dataPtr->gravityDisabled = false;

    // Trigger the return animation.
    this->dataPtr->returnAnimation->SetTime(0);
    this->dataPtr->model->SetAnimation(this->dataPtr->returnAnimation);
    this->dataPtr->currentState = "returning";
  }
  if (this->dataPtr->currentState == "returning")
  {
    bool returnAnimationDone = this->dataPtr->returnAnimation->GetTime() >= \
      this->dataPtr->returnAnimation->GetLength();
    if (returnAnimationDone)
    {
      gzdbg << "Return animation finished." << std::endl;
      this->dataPtr->currentState = "ready_to_deliver";
    }
  }
  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->statePub.publish(stateMsg);

}

/////////////////////////////////////////////////
bool ROSAGVPlugin::OnCommand(
  std_srvs::Trigger::Request &,
  std_srvs::Trigger::Response &_resp)
{
  if (this->dataPtr->currentState != "ready_to_deliver")
  {
    _resp.message = "AGV not successfully triggered as it was not ready to deliver trays.";
    ROS_ERROR_STREAM(_resp.message);
    _resp.success = false;
    return true;
  }
  ROS_INFO_STREAM("[INFO] AGV '" << this->dataPtr->agvName << "' delivery triggered");
  this->dataPtr->deliveryTriggered = true;
  _resp.success = true;
  return true;
}
