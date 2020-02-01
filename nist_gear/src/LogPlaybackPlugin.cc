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

#include "LogPlaybackPlugin.hh"
#include <iostream>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(LogPlaybackPlugin)

//////////////////////////////////////////////////
LogPlaybackPlugin::~LogPlaybackPlugin()
{
}

//////////////////////////////////////////////////
void LogPlaybackPlugin::Load(int _argc, char **_argv)
{
  (void) _argc;
  (void) _argv;

  this->worldCreatedConn = event::Events::ConnectWorldCreated(
      std::bind(&LogPlaybackPlugin::OnWorldCreated, this));

  this->updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LogPlaybackPlugin::Update, this));

}

//////////////////////////////////////////////////
void LogPlaybackPlugin::Init()
{
}

//////////////////////////////////////////////////
void LogPlaybackPlugin::OnWorldCreated()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->world = gazebo::physics::get_world();

  std::cout << "World created. Model count: " << this->world->ModelCount()
            << std::endl;

  gazebo::physics::Model_V models = this->world->Models();

  for (gazebo::physics::ModelPtr model : models)
  {
    // Get the shipping boxes.
    if (model->GetName().find("shipping_box") != std::string::npos)
    {
      this->boxes.push_back(model);
      this->toggled[model->GetName()] = false;
      this->droneToggled[model->GetName()] = false;
      std::string topicName = "/ariac/" +
        model->GetName() + "_visual_toggle";
      this->pubs[model->GetName()] = this->node->Advertise<msgs::GzString>(
          topicName);
      std::cout << "Shipping box model: " << model->GetName() << std::endl;
    }

    // Get the drone.
    if (model->GetName().find("drone") != std::string::npos)
    {
      this->drone = model;
      this->droneBoxEnabled = false;
      std::cout << "Drone model: " << model->GetName() << std::endl;
    }
  }

  this->droneTogglePub =
    this->node->Advertise<msgs::GzString>(
        "/gazebo/default/drone_box_visual_toggle");
}

//////////////////////////////////////////////////
void LogPlaybackPlugin::Update()
{
  for (gazebo::physics::ModelPtr box : this->boxes)
  {
    double xPos = box->WorldPose().Pos().X();
    double yPos = box->WorldPose().Pos().Y();
    double zPos = box->WorldPose().Pos().Z();
    // Toggle the box's visuals when it reaches the end of the belt.
    if ((yPos < -2.8 && !this->toggled[box->GetName()]) ||
        (yPos > -2.8 && this->toggled[box->GetName()]))
    {
      gazebo::msgs::GzString toggleMsg;
      toggleMsg.set_data("toggle");
      this->toggled[box->GetName()] = !this->toggled[box->GetName()];
      this->pubs[box->GetName()]->Publish(toggleMsg);
    }

    // Enable the drone's box visual when the waiting box gets "collected".
    // At that point the waiting box gets teleported out-of-sight.
    if (yPos < -9 && zPos < 0 && xPos < -7 &&
        !this->droneToggled[box->GetName()])
    {
      this->droneToggled[box->GetName()] = true;
      this->droneBoxEnabled = true;
      gazebo::msgs::GzString toggleMsg;
      toggleMsg.set_data("on");
      this->droneTogglePub->Publish(toggleMsg);
    }
  }

  // A drone may collect multiple boxes in a single trial.
  // Disable the drone's box visual at the start of repeat animations.
  if (this->droneBoxEnabled && this->drone->WorldPose().Pos().X() < -1.0)
  {
    this->droneBoxEnabled = false;
    gazebo::msgs::GzString toggleMsg;
    toggleMsg.set_data("off");
    this->droneTogglePub->Publish(toggleMsg);
  }
}
