/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <cstdio>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include "SensorBlackoutPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SensorBlackoutPlugin)

/////////////////////////////////////////////////
SensorBlackoutPlugin::SensorBlackoutPlugin()
{
}

/////////////////////////////////////////////////
SensorBlackoutPlugin::~SensorBlackoutPlugin()
{
    this->parentSensor.reset();
}

/////////////////////////////////////////////////
void SensorBlackoutPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    // Get the name of the parent sensor
    this->parentSensor = _parent;

    std::string worldName = this->parentSensor->WorldName();
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->parentSensor->WorldName());
    this->sensor_update_rate = this->parentSensor->UpdateRate();

    if (!_sdf->HasElement("activation_topic"))
    {
      gzerr << "Activation topic required." << std::endl;
      return;
    }
    std::string activationTopic = _sdf->Get<std::string>("activation_topic");
    this->activationSub = this->node->Subscribe(activationTopic,
            &SensorBlackoutPlugin::OnActivationMsg, this);
}

/////////////////////////////////////////////////
void SensorBlackoutPlugin::OnActivationMsg(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "activate")
  {
    this->parentSensor->SetActive(true);
    this->parentSensor->SetUpdateRate(this->sensor_update_rate); //Hack for rgbd camera
  }
  else if (_msg->data() == "deactivate")
  {
    this->parentSensor->SetActive(false);
    this->parentSensor->SetUpdateRate(1e-7); //Hack for rgbd camera
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
