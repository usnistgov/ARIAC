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

#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "SideContactPlugin.hh"
#include <ignition/math/Vector3.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SideContactPlugin)

/////////////////////////////////////////////////
SideContactPlugin::SideContactPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
SideContactPlugin::~SideContactPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SideContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  // gzerr << _model->GetName() << std::endl;

  if (!_sdf->HasElement("contact_sensor_name"))
  {
    gzerr << "'contact_sensor_name' not specified in SDF\n";
  }
  this->model = _model;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->Name());

  this->contactSensorName = _sdf->Get<std::string>("contact_sensor_name");
  bool sensorFound = this->FindContactSensor();
  if (!sensorFound || !this->parentSensor)
  {
    gzerr << "Contact sensor not found: " << this->contactSensorName << "\n";
  }
  //  this->parentSensor->SetActive(true);

  std::string parentLinkName = this->parentLink->GetScopedName();

    // gzdbg << "---------parentLinkName: " << parentLinkName << "\n";
  std::string defaultCollisionName = parentLinkName + "::__default__";
  // gzdbg << "---------defaultCollisionName: " << defaultCollisionName << "\n";
  if (this->parentSensor->GetCollisionCount() != 1)
  {
    gzerr << "SideContactPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->collisionName = this->parentSensor->GetCollisionName(0);
  // gzdbg << "---------collisionName: " << this->collisionName << "\n";
  if (this->collisionName == defaultCollisionName)
  {
    // Use the first collision of the parent link by default
    if (this->parentLink->GetCollisions().empty())
    {
      gzerr << "Couldn't find any collisions for the contact sensor.";
      return;
    }
    unsigned int index = 0;
    this->collisionName = this->parentLink->GetCollision(index)->GetScopedName();
  }
  gzdbg << "[" << this->model->GetName() << "] Watching collisions on: " << this->collisionName << "\n";


  if (_sdf->HasElement("update_rate"))
  {
    std::string ur = _sdf->Get<std::string>("update_rate");
    try
    {
      double v = std::stod(ur);
      if (v <= 0)
      {
        gzerr << "Illegal update_rate value [" << v << "]" << std::endl;
      }
      this->updateRate = v;
    } catch (const std::exception& e)
    {
      gzerr << "Unable to parse update_rate [" << ur << "]" << std::endl;
    }
  }

  this->lastUpdateTime = this->world->SimTime();

  // FIXME: how to not hard-code this gazebo prefix?
  std::string contactTopic = "/gazebo/" + this->scopedContactSensorName;
  boost::replace_all(contactTopic, "::", "/");

  gzdbg << "Gazebo contact topic: "<< contactTopic << "\n";
  this->contactSub =
    this->node->Subscribe(contactTopic, &SideContactPlugin::OnContactsReceived, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SideContactPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
bool SideContactPlugin::FindContactSensor()
{
  auto sensorManager = sensors::SensorManager::Instance();
  auto links = this->model->GetLinks();
  for (const auto &link : links)
  {
    std::string scopedContactSensorName =
      this->world->Name() + "::" + link->GetScopedName() + "::" + this->contactSensorName;
      // gzwarn << "this->world->Name(): " << this->world->Name() << "\n";
      // gzwarn << "link->GetScopedName(): " << link->GetScopedName() << "\n";
      // gzwarn << "this->contactSensorName: " << this->contactSensorName << "\n";
      // gzwarn << "scopedContactSensorName: " << scopedContactSensorName << "\n";
      //  gzwarn << "link->GetName(): " << link->GetName() << "\n";
      // gzwarn << "link->GetSensorCount(): " << link->GetSensorCount() << "\n";
      
      
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      // gzwarn << "link->GetSensorName(i): " << link->GetSensorName(i) << "\n";
      if (link->GetSensorName(i) == scopedContactSensorName)
      {
        this->parentLink = link;
        this->scopedContactSensorName = scopedContactSensorName;
        // gzdbg << scopedContactSensorName << "\n";
        this->parentSensor =
          std::static_pointer_cast<sensors::ContactSensor>(
            sensorManager->GetSensor(scopedContactSensorName));
        
       

        // gzdbg << "Parent sensor name: " << this->parentSensor->Name() << "\n";
        return this->parentSensor != 0;
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->newestContactsMsg = *_msg;
  this->newMsg = true;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingLinks()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->newMsg)
  {
    return;
  }

  this->contactingLinks.clear();

  // Get all the contacts
  for (int i = 0; i < this->newestContactsMsg.contact_size(); ++i)
  {
    // Get the collision that's not the parent link
    const auto &contact = this->newestContactsMsg.contact(i);
    const std::string *collision = &(contact.collision1());
    if (this->collisionName == *collision) {
      collision = &(contact.collision2());
    }

    physics::CollisionPtr collisionPtr =
      boost::static_pointer_cast<physics::Collision>(this->world->EntityByName(*collision));
    if (collisionPtr) { // ensure the collision hasn't been deleted
      this->contactingLinks.insert(collisionPtr->GetLink());
    }
  }
  this->newMsg = false;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}

/////////////////////////////////////////////////
void SideContactPlugin::  ClearContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Static because multiple plugin instances may exist, and we want them to share the state
  static int index = 0;

  for (auto model : this->contactingModels)
  {
    gzdbg << "Teleporting model: " << model->GetScopedName() << std::endl;
    model->SetAutoDisable(true);
    model->SetGravityMode(true);
    model->SetStatic(true);
    model->SetWorldPose(ignition::math::Pose3d(0 + 0.25*index++, 0, -5, 0, 0, 0));
  }
}

/////////////////////////////////////////////////
bool SideContactPlugin::TimeToExecute()
{
  // We're using a custom update rate.
  if (this->updateRate <= 0)
    return true;

  gazebo::common::Time curTime = this->world->SimTime();
  auto dt = (curTime - this->lastUpdateTime).Double();
  if (dt < 0)
  {
    // Probably we had a reset.
    this->lastUpdateTime = curTime;
    return false;
  }

  // Update based on sensorsUpdateRate.
  if (dt < (1.0 / this->updateRate))
    return false;

  this->lastUpdateTime = curTime;
  return true;
}
