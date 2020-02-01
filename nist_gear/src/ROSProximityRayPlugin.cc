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

#include "ROSProximityRayPlugin.hh"

#include <string>

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ROSProximityRayPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
ROSProximityRayPlugin::ROSProximityRayPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ROSProximityRayPlugin::~ROSProximityRayPlugin()
{
  this->rosnode->shutdown();
}

void ROSProximityRayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ProximityRayPlugin::Load(_parent, _sdf);

  // Read ROS-specific sdf tags
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Over-ride topics from the ProximityRayPlugin which may contain ~s
  this->stateTopic = _parent->Name();
  if (_sdf->HasElement("output_state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("output_state_topic");
  }

  this->stateChangeTopic = _parent->Name() + "_change";
  if (_sdf->HasElement("output_change_topic"))
  {
    this->stateChangeTopic = _sdf->Get<std::string>("output_change_topic");
  }

  this->frameId = _parent->Name() + "_frame";
  if (_sdf->HasElement("frame_id"))
  {
    this->frameId = _sdf->Get<std::string>("frame_id");
  }
  this->state_msg.header.frame_id = this->frameId;

  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  // Initialize the publishers
  this->statePub = this->rosnode->advertise<nist_gear::Proximity>(
    this->stateTopic, 1, false);
  this->stateChangePub = this->rosnode->advertise<nist_gear::Proximity>(
    this->stateChangeTopic, 1, true);

  // Callback for laser scans
  this->newLaserScansConnection =
      this->parentSensor->LaserShape()->ConnectNewLaserScans(
          std::bind(&ROSProximityRayPlugin::OnNewLaserScans, this));

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void ROSProximityRayPlugin::OnNewLaserScans()
{
  auto now = this->world->SimTime();
  bool stateChanged = this->ProcessScan();
  this->state_msg.header.stamp.sec = now.sec;
  this->state_msg.header.stamp.nsec = now.nsec;
  this->state_msg.object_detected = this->objectDetected;
  this->state_msg.min_range = this->sensingRangeMin;
  this->state_msg.max_range = this->sensingRangeMax;
  this->statePub.publish(this->state_msg);
  if (stateChanged)
  {
    gzdbg << this->parentSensor->Name() << ": change in sensor state\n";
    this->stateChangePub.publish(this->state_msg);
  }
}

