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
#include <ros/ros.h>
#include "ModelContactPlugin.hh"
#include <ignition/math/Vector3.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ModelContactPlugin)

/////////////////////////////////////////////////
ModelContactPlugin::ModelContactPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
ModelContactPlugin::~ModelContactPlugin()
{
  // ROS_ERROR_STREAM("~ModelContactPlugin");
  this->updateConnection.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ModelContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");
  // gzerr << _model->GetName() << std::endl;

  this->model = _model;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->Name());

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

  // Find all collision elements of the model
  auto modelLinks = this->model->GetLinks();
  for (auto currentLink : modelLinks)
  {
    for (auto j = 0u; j < currentLink->GetChildCount(); ++j)
    {
      physics::CollisionPtr collision = currentLink->GetCollision(j);
  
      //skip if collision already listed
      std::map<std::string, physics::CollisionPtr>::iterator collIter =
  	this->collisions.find(collision->GetScopedName());
      if (collIter != this->collisions.end())
        continue;
  
      this->collisions[collision->GetScopedName()] = collision;
      gzdbg << collision->GetScopedName() << std::endl;
    }
  }

  // Create a filter to receive collision information
  auto mgr = this->world->Physics()->GetContactManager();
  auto filter_name = this->model->GetName();
  auto topic = mgr->CreateFilter(filter_name, this->collisions);
  if (!this->contactSub)
  {
    this->contactSub = this->node->Subscribe(topic,
                           &ModelContactPlugin::OnContactsReceived, this);
  }

  this->Reset();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ModelContactPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void ModelContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->newestContactsMsg = *_msg;
  this->newMsg = true;
  // gzerr << _msg->GetTypeName() << "\n";
}

/////////////////////////////////////////////////
void ModelContactPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void ModelContactPlugin::CalculateContactingLinks()
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
    if (collision->find(this->model->GetName()) != std::string::npos) {
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
void ModelContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}
