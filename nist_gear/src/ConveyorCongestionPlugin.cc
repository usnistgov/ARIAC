/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <string>

namespace gazebo
{
class ConveyorCongestionPlugin : public WorldPlugin
{
  private: transport::NodePtr gzNode;

  private: physics::WorldPtr world;
  private: event::ConnectionPtr updateConnection;

  private: transport::SubscriberPtr gzBreakBeamSub;
  private: transport::SubscriberPtr gzWaitingBoxSub;
  private: transport::PublisherPtr gzConveyorEnablePub;
  private: bool congestionSensorState;
  private: bool boxWaiting;
  private: bool beltEnabled = true;

  public: ~ConveyorCongestionPlugin()
  {
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->world = _parent;

    // Initialize Gazebo transport
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();

    // Create a subscriber for the break beam output
    std::string breakBeamStateTopic = "~/congestion_sensor";
    if (_sdf->HasElement("congestion_sensor_topic"))
    {
      breakBeamStateTopic = _sdf->Get<std::string>("congestion_sensor_topic");
    }
    this->gzBreakBeamSub = this->gzNode->Subscribe(
        breakBeamStateTopic, &ConveyorCongestionPlugin::OnSensorState, this);

    // Create a publisher for the conveyor enable topic
    std::string conveyorControlTopic = "~/conveyor_enable";
    if (_sdf->HasElement("conveyor_control_topic"))
    {
      conveyorControlTopic = _sdf->Get<std::string>("conveyor_control_topic");
    }
    this->gzConveyorEnablePub =
      this->gzNode->Advertise<msgs::GzString>(conveyorControlTopic);

    std::string waitingBoxTopic = "~/waiting_shipping_box";
    if (_sdf->HasElement("waiting_box_topic"))
    {
      waitingBoxTopic = _sdf->Get<std::string>("waiting_box_topic");
    }
    this->gzWaitingBoxSub = this->gzNode->Subscribe(
      waitingBoxTopic, &ConveyorCongestionPlugin::OnWaitingBox, this);

    // Listen to the update event that is broadcasted every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ConveyorCongestionPlugin::OnUpdate, this));
  }

  private: void OnWaitingBox(ConstGzStringPtr &_msg)
  {
    if (_msg->data() == "")
    {
      this->boxWaiting = false;
    } else
    {
      this->boxWaiting = true;
    }
  }

  private: void OnSensorState(ConstHeaderPtr &_msg)
  {
    this->congestionSensorState = _msg->index();
  }

  private: void OnUpdate()
  {
    if (this->congestionSensorState && this->boxWaiting)
    {
      if (this->beltEnabled)
      {
        std::string logMessage;
        logMessage = "Disabling belt due to congestion";
        gzdbg << logMessage << std::endl;
        gazebo::msgs::GzString msg;
        msg.set_data("disabled");
        this->gzConveyorEnablePub->Publish(msg);
        this->beltEnabled = false;
      }
    } else
    {
      if (!this->beltEnabled)
      {
        std::string logMessage;
        logMessage = "Re-enabling belt because it's no longer congested";
        gzdbg << logMessage << std::endl;
        gazebo::msgs::GzString msg;
        msg.set_data("enabled");
        this->gzConveyorEnablePub->Publish(msg);
        this->beltEnabled = true;
      }
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ConveyorCongestionPlugin)
}
