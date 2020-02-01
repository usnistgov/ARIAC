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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <gazebo/common/Console.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include "nist_gear/ROSPopulationPlugin.hh"
#include "nist_gear/PopulationControl.h"
#include "nist_gear/PopulationState.h"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSPopulationPlugin class.
  struct ROSPopulationPluginPrivate
  {
    /// \brief ROS node handle.
    public: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Publishes the state of the plugin.
    public: ros::Publisher statePub;

    /// \brief Receives service calls to control the plugin.
    public: ros::ServiceServer controlService;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ROSPopulationPlugin);

/////////////////////////////////////////////////
ROSPopulationPlugin::ROSPopulationPlugin()
  : PopulationPlugin(),
    dataPtr(new ROSPopulationPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSPopulationPlugin::~ROSPopulationPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSPopulationPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load SDF parameters.
  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
      "robot_namespace")->Get<std::string>() + "/";
  }

  std::string controlTopic = "population/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "population/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  PopulationPlugin::Load(_world, _sdf);

  this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  // During competition, this environment variable will be set and the
  // population control service won't be available.
  auto v = std::getenv("ARIAC_COMPETITION");
  if (!v)
  {
    // Service for controlling the plugin.
    this->dataPtr->controlService =
      this->dataPtr->rosnode->advertiseService(controlTopic,
        &ROSPopulationPlugin::OnPopulationControl, this);
  }

  // Message used for publishing the state of the gripper.
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    nist_gear::PopulationState>(stateTopic, 1000);
}

/////////////////////////////////////////////////
void ROSPopulationPlugin::Reset()
{
  PopulationPlugin::Reset();
}

bool ROSPopulationPlugin::OnPopulationControl(
  nist_gear::PopulationControl::Request &_req,
  nist_gear::PopulationControl::Response &_res)
{
  gzdbg << "ROSPopulationPlugin::OnPopulationControl() called with: " << _req.action << std::endl;
  _res.success = true;

  if (_req.action == "pause")
    this->Pause();
  else if (_req.action == "resume")
    this->Resume();
  else if (_req.action == "restart")
    this->Restart();
  else
  {
    gzerr << "ROSPopulationPlugin::OnPopulationControl() error: Illegal "
          << "action received: [" << _req.action << "]" << std::endl;
    _res.success = false;
  }

  return _res.success;
}

/////////////////////////////////////////////////
void ROSPopulationPlugin::Publish() const
{
  nist_gear::PopulationState msg;
  msg.enabled = this->Enabled();
  this->dataPtr->statePub.publish(msg);
}
