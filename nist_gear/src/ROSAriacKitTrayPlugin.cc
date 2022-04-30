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

#include <cstdlib>
#include <string>

#include <nist_gear/DetectedKittingShipment.h>
#include "ROSAriacKitTrayPlugin.hh"
#include <nist_gear/DetectedMovableTray.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KitTrayPlugin)

/////////////////////////////////////////////////
KitTrayPlugin::KitTrayPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
KitTrayPlugin::~KitTrayPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void KitTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");

  // Initialize Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  if (this->updateRate > 0)
    gzdbg << "KitTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "KitTrayPlugin running at the default update rate\n";

  this->tray_id = this->parentLink->GetScopedName();

  this->agv1LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv1/station", 1000, &KitTrayPlugin::OnAGV1Location, this);
  this->agv2LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv2/station", 1000, &KitTrayPlugin::OnAGV2Location, this);
  this->agv3LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv3/station", 1000, &KitTrayPlugin::OnAGV3Location, this);
  this->agv4LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv4/station", 1000, &KitTrayPlugin::OnAGV4Location, this);

  this->tf_frame_name = "kit_tray_frame";
  if (_sdf->HasElement("tf_frame_name"))
    this->tf_frame_name = _sdf->Get<std::string>("tf_frame_name");

  // ROS service for getting the content of the tray
  std::string kit_tray_content_service_name = "get_content";
  if (_sdf->HasElement("get_content_service_name"))
    kit_tray_content_service_name = _sdf->Get<std::string>("get_content_service_name");
  this->tray_contents_server =
      this->rosNode->advertiseService(kit_tray_content_service_name, &KitTrayPlugin::HandleGetMovableTrayService, this);

  std::string manual_lock_tray_service_name = "";
  if (_sdf->HasElement("manual_lock_tray_service_name"))
    manual_lock_tray_service_name = _sdf->Get<std::string>("manual_lock_tray_service_name");
  // ROS_ERROR_STREAM("Advertising Lock: " << manual_lock_tray_service_name);
  this->manual_lock_tray_server =
      this->rosNode->advertiseService(manual_lock_tray_service_name, &KitTrayPlugin::HandleManualLockTrayService, this);

  std::string manual_unlock_tray_service_name = "";
  if (_sdf->HasElement("manual_unlock_tray_service_name"))
    manual_unlock_tray_service_name = _sdf->Get<std::string>("manual_unlock_tray_service_name");
  // ROS_ERROR_STREAM("Advertising Unlock: " << manual_unlock_tray_service_name);
  this->manual_unlock_tray_server = this->rosNode->advertiseService(
      manual_unlock_tray_service_name, &KitTrayPlugin::HandleManualUnlockTrayService, this);

  std::string tray_locked_status_param = "";
  if (_sdf->HasElement("tray_locked_status_param"))
  {
    tray_locked_status_param = _sdf->Get<std::string>("tray_locked_status_param");
    this->kittray_lock_status_param = tray_locked_status_param;
    // start all kit trays in an unlocked state
    this->rosNode->setParam(tray_locked_status_param, "unlocked");
  }

  // Gazebo subscription for the lock trays topic
  std::string lockUnlockModelsServiceName{};
  if (_sdf->HasElement("lock_unlock_movable_tray_on_agv"))
  {
    lockUnlockModelsServiceName = _sdf->Get<std::string>("lock_unlock_movable_tray_on_agv");
    this->lock_movable_tray_on_agv_subscriber =
        this->gzNode->Subscribe(lockUnlockModelsServiceName, &KitTrayPlugin::HandleLockUnlockModelsRequest, this);
  }
  else
  {
    gzerr << "lock_unlock_movable_tray_on_agv missing in ariac.world\n";
  }

  if (_sdf->HasElement("grippable_model_types"))
  {
    this->grippable_model_types.clear();
    sdf::ElementPtr grippableModelTypesElem = _sdf->GetElement("grippable_model_types");
    if (!grippableModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <grippable_model_types> section\n";
      return;
    }
    sdf::ElementPtr grippableModelTypeElem = grippableModelTypesElem->GetElement("type");
    while (grippableModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = grippableModelTypeElem->Get<std::string>();

      // gzdbg << "New grippable model type: " << type << "\n";
      this->grippable_model_types.push_back(type);
      grippableModelTypeElem = grippableModelTypeElem->GetNextElement("type");
    }
  }
}

///////////////////////////////////
// MANUAL CALL to /ariac/kit_tray_x/lock and /ariac/kit_tray_x/unlock
void KitTrayPlugin::HandleLockUnlockModelsRequest(ConstGzStringPtr& _msg)
{
  std::string kit_tray_locked_status;
  this->rosNode->getParam(this->kittray_lock_status_param, kit_tray_locked_status);

  // gzerr << "<<<<< status " << kit_tray_locked_status << "\n";
  if (_msg->data() == "lock")
  {
    gzerr << "<<<<< status " << kit_tray_locked_status << "\n";
    gzerr << "<<<<< message " << _msg->data() << "\n";
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (kit_tray_locked_status == "locked")
    {
      // std::this_thread::sleep_for(std::chrono::milliseconds(150));
      this->UnlockContactingModels();
      this->LockContactingModels();
    }
    else if (kit_tray_locked_status == "unlocked")
    {
      this->LockContactingModels();
    }
    this->rosNode->setParam(this->kittray_lock_status_param, "locked");
  }
  else if (_msg->data() == "unlock")
  {
    gzerr << "<<<<< status " << kit_tray_locked_status << "\n";
    gzerr << "<<<<< message " << _msg->data() << "\n";
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));
    if (kit_tray_locked_status == "locked")
    {
      // std::this_thread::sleep_for(std::chrono::milliseconds(150));
      this->UnlockContactingModels();
      // this->UnlockContactingModels();
      this->rosNode->setParam(this->kittray_lock_status_param, "unlocked");
    }
  }
}

//////////////////////////////////////////////
void KitTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;

  for (auto movable_tray : this->contactingModels)
  {
    auto model_name = movable_tray->GetName();
    auto model_type = ariac::DetermineModelType(movable_tray->GetName());

    // make sure this model is an ariac part
    auto is_grippable_it =
        std::find(this->grippable_model_types.begin(), this->grippable_model_types.end(), model_type);
    bool grippable_model = is_grippable_it != this->grippable_model_types.end();
    if (grippable_model)
    {
      // create a fixed joint on the kit tray (e.g., kit_tray_1)
      fixedJoint = this->world->Physics()->CreateJoint("fixed", this->model);
      // create a name for this joint
      auto jointName = this->model->GetName() + "_" + movable_tray->GetName() + "__joint__";
      gzdbg << "Creating fixed joint: " << jointName << std::endl;
      fixedJoint->SetName(jointName);

      movable_tray->SetGravityMode(false);
      // Lift the part slightly because it will fall through the tray if the tray is animated
      movable_tray->SetWorldPose(movable_tray->WorldPose() + ignition::math::Pose3d(0, 0, 0.01, 0, 0, 0));

      auto modelName = movable_tray->GetName();
      std::string linkName = modelName + "::link";
      auto link = movable_tray->GetLink(linkName);
      if (link == NULL)
      {
        // If the model was inserted into the world using the "population" SDF tag,
        // the link will have an additional namespace of the model type.
        linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
        link = movable_tray->GetLink(linkName);
        if (link == NULL)
        {
          linkName = modelName + "::" + modelName + "::link";
          link = movable_tray->GetLink(linkName);
          if (link == NULL)
          {
            gzwarn << "Couldn't find link to make joint with: " << linkName;
            continue;
          }
        }
      }

      link->SetGravityMode(false);
      fixedJoint->SetModel(this->model);
      fixedJoint->Attach(link, this->parentLink);
      fixedJoint->Load(this->parentLink, link, ignition::math::Pose3d());
      fixedJoint->Init();
      this->fixedJoints.push_back(fixedJoint);
      movable_tray->SetAutoDisable(false);
    }
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;

  for (auto fixedJoint : this->fixedJoints)
  {
    fixedJoint->Detach();
  }
  this->fixedJoints.clear();
  for (auto model : this->contactingModels)
  {
    model->SetGravityMode(true);
    model->SetAutoDisable(false);
  }
}

void KitTrayPlugin::OnAGV1Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->agv1CurrentStation = _msg->data;
  // gzdbg << "current_station: "<< this->dataPtr->current_station << "\n";
}

void KitTrayPlugin::OnAGV2Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->agv2CurrentStation = _msg->data;
  // gzdbg << "current_station: "<< this->dataPtr->current_station << "\n";
}

void KitTrayPlugin::OnAGV3Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->agv3CurrentStation = _msg->data;
  // gzdbg << "current_station: "<< this->dataPtr->current_station << "\n";
}

void KitTrayPlugin::OnAGV4Location(std_msgs::String::ConstPtr _msg)
{
  // std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->agv4CurrentStation = _msg->data;
  // gzdbg << "current_station: "<< this->dataPtr->current_station << "\n";
}
/////////////////////////////////////////////////
void KitTrayPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
  {
    return;
  }

  if (!this->newMsg)
  {
    return;
  }

  std::set<physics::ModelPtr> prevContactingModels(this->contactingModels);
  this->CalculateContactingModels();
  if (prevContactingModels.size() != this->contactingModels.size())
  {
    ROS_DEBUG_STREAM(this->parentLink->GetScopedName()
                     << ": number of contacting models: " << this->contactingModels.size() << std::endl);
  }

  // Look for models that were contacting tray but now aren't.
  // Either they've been fixed to tray or have been removed from tray
  std::vector<physics::ModelPtr> removedContactingModels;
  std::set_difference(prevContactingModels.begin(), prevContactingModels.end(), this->contactingModels.begin(),
                      this->contactingModels.end(),
                      std::inserter(removedContactingModels, removedContactingModels.begin()));
  for (auto model : removedContactingModels)
  {
    if (model)
    {
      gzdbg << "removed contact " << model->GetName() << std::endl;
      model->SetAutoDisable(true);
    }
  }

  this->ProcessContactingModels();
  // if (this->publishingEnabled) {
  //   this->PublishKitMsg();
  // }
  this->PublishTFTransform(_info.simTime);
}

/////////////////////////////////////////////////
void KitTrayPlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto fixedJoint : this->fixedJoints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }

  auto trayPose = this->parentLink->WorldPose();
  for (auto model : this->contactingModels)
  {
    if (model)
    {
      model->SetAutoDisable(false);
      ariac::MovableTray object;

      // Determine the object type
      object.type = ariac::DetermineModelType(model->GetName());
      object.name = ariac::TrimNamespace(model->GetName());

      // gzerr << "Object Type: " << object.type << std::endl;
      // gzerr << "Object Name: " << object.name << std::endl;

      // Determine the pose of the object in the frame of the tray
      ignition::math::Pose3d objectPose = model->WorldPose();
      ignition::math::Matrix4d transMat(trayPose);
      ignition::math::Matrix4d objectPoseMat(objectPose);
      object.pose = (transMat.Inverse() * objectPoseMat).Pose();
      object.pose.Rot().Normalize();

      this->current_movable_tray = object;
    }
  }
}

/////////////////////////////////////////////////
// void KitTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
// {
//   auto subscriberName = pub.getSubscriberName();
//   gzwarn << this->tray_id << ": New subscription from node: " << subscriberName << std::endl;

//   // During the competition, this environment variable will be set.
//   auto compRunning = std::getenv("ARIAC_COMPETITION");
//   if (compRunning && subscriberName.compare("/gazebo") != 0) {
//     std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
//     gzerr << errStr << std::endl;
//     ROS_ERROR_STREAM(errStr);
//     // Disable publishing of kit messages.
//     // This will break the scoring but ensure competitors can't cheat.
//     this->publishingEnabled = false;
//   }
// }

/////////////////////////////////////////////////
// void KitTrayPlugin::PublishKitMsg()
// {
//   auto agv = this->tray_id.substr(0, 4);
//   nist_gear::DetectedMovableTray detected_movable_tray_msg;
//   detected_movable_tray_msg.agv_name = agv;
//   detected_movable_tray_msg.movable_tray_name = this->current_movable_tray.name;
//   detected_movable_tray_msg.movable_tray_type = this->current_movable_tray.type;
//   detected_movable_tray_msg.pose.position.x = this->current_movable_tray.pose.Pos().X();
//   detected_movable_tray_msg.pose.position.y = this->current_movable_tray.pose.Pos().Y();
//   detected_movable_tray_msg.pose.position.z = this->current_movable_tray.pose.Pos().Z();
//   detected_movable_tray_msg.pose.orientation.x = this->current_movable_tray.pose.Rot().X();
//   detected_movable_tray_msg.pose.orientation.y = this->current_movable_tray.pose.Rot().Y();
//   detected_movable_tray_msg.pose.orientation.z = this->current_movable_tray.pose.Rot().Z();
//   detected_movable_tray_msg.pose.orientation.w = this->current_movable_tray.pose.Rot().W();

//   // get the products inside this->current_movable_tray

//   // response.movable_tray = detected_movable_tray_msg;
//   this->currentKitPub.publish(detected_movable_tray_msg);
// }

// ros service call
bool KitTrayPlugin::HandleManualLockTrayService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::string kit_tray_locked_status;
  this->rosNode->getParam(this->kittray_lock_status_param, kit_tray_locked_status);

  if (kit_tray_locked_status == "unlocked")
  {
    this->LockContactingModels();

    this->rosNode->setParam(this->kittray_lock_status_param, "locked");
    res.message = "Movable tray is now locked on the kit tray";
    res.success = true;
  }
  else if (kit_tray_locked_status == "locked")
  {
    this->UnlockContactingModels();
    this->LockContactingModels();
    this->rosNode->setParam(this->kittray_lock_status_param, "locked");
    res.message = "Movable tray is now locked on the kit tray";
    res.success = true;
  }

  return true;
}

// ros service call
bool KitTrayPlugin::HandleManualUnlockTrayService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::string kit_tray_locked_status;
  this->rosNode->getParam(this->kittray_lock_status_param, kit_tray_locked_status);

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  if (kit_tray_locked_status == "locked")
  {
    this->UnlockContactingModels();

    this->rosNode->setParam(this->kittray_lock_status_param, "unlocked");
    res.message = "Movable tray is now unlocked on the kit tray";
    res.success = true;
  }
  else
  {
    res.message = "Movable tray is already unlocked on the kit tray";
    res.success = false;
  }

  return true;
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleGetMovableTrayService(
    ros::ServiceEvent<nist_gear::DetectMovableTray::Request, nist_gear::DetectMovableTray::Response>& event)
{
  const std::string& callerName = event.getCallerName();

  auto agv = this->tray_id.substr(0, 4);

  // gzdbg << "Handle get content service called by: " << callerName << std::endl;
  // ROS_ERROR_STREAM(this->trayID << ": Handle get content service called by: " << callerName);

  auto& response = event.getResponse();

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return true;
  }

  // int agv_id = std::stoi(this->tray_id.substr(3, 1)); //--get the id of the agv

  nist_gear::DetectedMovableTray detected_movable_tray_msg;
  detected_movable_tray_msg.agv_name = agv;
  detected_movable_tray_msg.movable_tray_name = this->current_movable_tray.name;
  detected_movable_tray_msg.movable_tray_type = this->current_movable_tray.type;
  detected_movable_tray_msg.pose.position.x = this->current_movable_tray.pose.Pos().X();
  detected_movable_tray_msg.pose.position.y = this->current_movable_tray.pose.Pos().Y();
  detected_movable_tray_msg.pose.position.z = this->current_movable_tray.pose.Pos().Z();
  detected_movable_tray_msg.pose.orientation.x = this->current_movable_tray.pose.Rot().X();
  detected_movable_tray_msg.pose.orientation.y = this->current_movable_tray.pose.Rot().Y();
  detected_movable_tray_msg.pose.orientation.z = this->current_movable_tray.pose.Rot().Z();
  detected_movable_tray_msg.pose.orientation.w = this->current_movable_tray.pose.Rot().W();

  response.movable_tray = detected_movable_tray_msg;
  return true;
}

void KitTrayPlugin::PublishTFTransform(const common::Time sim_time)
{
  this->tray_pose = this->model->WorldPose();
  ignition::math::Pose3d objectPose = this->tray_pose;
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  tfStamped.header.frame_id = "world";
  tfStamped.child_frame_id = this->tf_frame_name;
  tfStamped.transform.translation.x = objectPose.Pos().X();
  tfStamped.transform.translation.y = objectPose.Pos().Y();
  tfStamped.transform.translation.z = objectPose.Pos().Z();

  tfStamped.transform.rotation.x = objectPose.Rot().X();
  tfStamped.transform.rotation.y = objectPose.Rot().Y();
  tfStamped.transform.rotation.z = objectPose.Rot().Z();
  tfStamped.transform.rotation.w = objectPose.Rot().W();

  tf_broadcaster.sendTransform(tfStamped);
}
