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

  if (_sdf->HasElement("faulty_parts"))
  {
    this->faultyPartNames.clear();
    sdf::ElementPtr faultyPartNamesElem = _sdf->GetElement("faulty_parts");
    if (faultyPartNamesElem->HasElement("name"))
    {
      sdf::ElementPtr faultyPartElem = faultyPartNamesElem->GetElement("name");
      while (faultyPartElem)
      {
        std::string faultyPartName = faultyPartElem->Get<std::string>();

        ROS_DEBUG_STREAM("Ignoring part: " << faultyPartName);
        this->faultyPartNames.push_back(faultyPartName);
        faultyPartElem = faultyPartElem->GetNextElement("name");
      }
    }
  }

  if (this->updateRate > 0)
    gzdbg << "KitTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "KitTrayPlugin running at the default update rate\n";

  this->trayID = this->parentLink->GetScopedName();
  
  

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");

  this->currentKitPub = this->rosNode->advertise<nist_gear::DetectedKittingShipment>(
    "/ariac/trays", 1000, boost::bind(&KitTrayPlugin::OnSubscriberConnect, this, _1));
  this->publishingEnabled = true;

  this->agv1LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv1/station",
                               1000, &KitTrayPlugin::OnAGV1Location, this);
  this->agv2LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv2/station",
                               1000, &KitTrayPlugin::OnAGV2Location, this);
  this->agv3LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv3/station",
                               1000, &KitTrayPlugin::OnAGV3Location, this);
  this->agv4LocationSubscriber =
      this->rosNode->subscribe("/ariac/agv4/station",
                               1000, &KitTrayPlugin::OnAGV4Location, this);

  this->tf_frame_name = "kit_tray_frame";
  if (_sdf->HasElement("tf_frame_name"))
    this->tf_frame_name = _sdf->Get<std::string>("tf_frame_name");

  // ROS service for clearing the tray
  std::string clearServiceName = "clear";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  this->clearTrayServer =
    this->rosNode->advertiseService(clearServiceName, &KitTrayPlugin::HandleClearService, this);

  // ROS service for getting the content of the tray
  std::string contentServiceName = "get_content";
  if (_sdf->HasElement("get_content_service_name"))
    contentServiceName = _sdf->Get<std::string>("get_content_service_name");
  this->trayContentsServer =
    this->rosNode->advertiseService(contentServiceName, &KitTrayPlugin::HandleGetContentService, this);

  // Initialize Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  // Gazebo subscription for the lock trays topic
  std::string lockUnlockModelsServiceName = "lock_models";
  if (_sdf->HasElement("lock_unlock_models_service_name"))
    lockUnlockModelsServiceName = _sdf->Get<std::string>("lock_unlock_models_service_name");
  this->lockModelsSub = this->gzNode->Subscribe(
    lockUnlockModelsServiceName, &KitTrayPlugin::HandleLockModelsRequest, this);

  // cache tray pose
  this->tray_pose = this->model->WorldPose();
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
void KitTrayPlugin::OnUpdate(const common::UpdateInfo & _info)
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
  if (prevContactingModels.size() != this->contactingModels.size()) {
    ROS_DEBUG_STREAM(this->parentLink->GetScopedName() << ": number of contacting models: "
      << this->contactingModels.size()<< std::endl);
  }

  // Look for models that were contacting tray but now aren't.
  // Either they've been fixed to tray or have been removed from tray
  std::vector<physics::ModelPtr> removedContactingModels;
  std::set_difference(prevContactingModels.begin(), prevContactingModels.end(),
    this->contactingModels.begin(), this->contactingModels.end(),
    std::inserter(removedContactingModels, removedContactingModels.begin()));
  for (auto model : removedContactingModels)
  {
    if (model) {
      gzdbg << "removed contact " << model->GetName() << std::endl;
      model->SetAutoDisable(true);
    }
  }

  this->ProcessContactingModels();
  if (this->publishingEnabled)
  {
    this->PublishKitMsg();
  }
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
  this->currentKit.objects.clear();
  auto trayPose = this->parentLink->WorldPose();
  for (auto model : this->contactingModels) {
    if (model) {
      model->SetAutoDisable(false);
      ariac::KitObject object;

      // Determine the object type
      object.type = ariac::DetermineModelType(model->GetName());

      // Determine if the object is faulty
      auto modelName = ariac::TrimNamespace(model->GetName());
      auto it = std::find(this->faultyPartNames.begin(), this->faultyPartNames.end(), modelName);
      object.isFaulty = it != this->faultyPartNames.end();

      // Determine the pose of the object in the frame of the tray
      ignition::math::Pose3d objectPose = model->WorldPose();
      ignition::math::Matrix4d transMat(trayPose);
      ignition::math::Matrix4d objectPoseMat(objectPose);
      object.pose = (transMat.Inverse() * objectPoseMat).Pose();
      object.pose.Rot().Normalize();

      this->currentKit.objects.push_back(object);
    }
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzwarn << this->trayID << ": New subscription from node: " << subscriberName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && subscriberName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    // Disable publishing of kit messages.
    // This will break the scoring but ensure competitors can't cheat.
    this->publishingEnabled = false;
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::PublishKitMsg()
{
  int agv_id = std::stoi(this->trayID.substr(3,1)); //--get the id of the agv
  // std::string agv_name = this->trayID.substr(3,1);

  std::string current_station{};
  if (agv_id == 1)
  {
    current_station = this->agv1CurrentStation;
  }
  if (agv_id == 2)
  {
    current_station = this->agv2CurrentStation;
  }
  if (agv_id == 3)
  {
    current_station = this->agv3CurrentStation;
  }
  if (agv_id == 4)
  {
    current_station = this->agv4CurrentStation;
  }
  this->station_name = current_station;
  
  // ROS_WARN_STREAM("this->trayID " << this->trayID);
  // ROS_WARN_STREAM("agv_id " << agv_id);
  // ROS_WARN_STREAM("station " << this->station_name);

  // Publish current kit
  nist_gear::DetectedKittingShipment kitTrayMsg;
  kitTrayMsg.destination_id = this->trayID;
  kitTrayMsg.station_id = this->station_name;
  for (const auto &obj : this->currentKit.objects)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = obj.type;
    msgObj.is_faulty = obj.isFaulty;
    msgObj.pose.position.x = obj.pose.Pos().X();
    msgObj.pose.position.y = obj.pose.Pos().Y();
    msgObj.pose.position.z = obj.pose.Pos().Z();
    msgObj.pose.orientation.x = obj.pose.Rot().X();
    msgObj.pose.orientation.y = obj.pose.Rot().Y();
    msgObj.pose.orientation.z = obj.pose.Rot().Z();
    msgObj.pose.orientation.w = obj.pose.Rot().W();

    // Add the object to the kit.
    kitTrayMsg.products.push_back(msgObj);
  }
  this->currentKitPub.publish(kitTrayMsg);
}



// /////////////////////////////////////////////////
// void KitTrayPlugin::LockContactingModels()
// {
//   ROS_WARN_STREAM("LockContactingModels");
//   boost::mutex::scoped_lock lock(this->mutex);
//   physics::JointPtr fixedJoint;
//   gzdbg << "Number of models in contact with the tray: " << this->contactingModels.size() << std::endl;
//   for (auto model : this->contactingModels)
//   {
//   // Create the joint that will attach the models
//   fixedJoint = this->world->Physics()->CreateJoint(
//         "fixed", this->model);
//   auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
//   gzdbg << "Creating fixed joint: " << jointName << std::endl;
//   fixedJoint->SetName(jointName);

//   model->SetGravityMode(false);

//   // Lift the part slightly because it will fall through the tray if the tray is animated
//   model->SetWorldPose(model->WorldPose() + ignition::math::Pose3d(0,0,0.01,0,0,0));

//   auto modelName = model->GetName();
//   auto linkName = modelName + "::link";
//   auto link = model->GetLink(linkName);
//   if (link == NULL)
//   {
//     // If the model was inserted into the world using the "population" SDF tag,
//     // the link will have an additional namespace of the model type.
//     linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
//     link = model->GetLink(linkName);
//     if (link == NULL)
//     {
//       gzwarn << "Couldn't find link to make joint with: " << linkName;
//       continue;
//     }
//   }
//   link->SetGravityMode(false);
//   fixedJoint->Load(link, this->parentLink, ignition::math::Pose3d());
//   fixedJoint->Attach(this->parentLink, link);
//   fixedJoint->Init();
//   this->fixedJoints.push_back(fixedJoint);
//   model->SetAutoDisable(true);
//   }
// }


// void KitTrayPlugin::UnlockContactingModels()
// {
//   boost::mutex::scoped_lock lock(this->mutex);
//   physics::JointPtr fixedJoint;
//   for (auto fixedJoint : this->fixedJoints)
//   {
//     fixedJoint->Detach();
//   }
//   this->fixedJoints.clear();
// }

///////////////////////////////////////////////////////////
void KitTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  gzdbg << "Number of models in contact with the tray: " << this->contactingModels.size() << "\n";
  for (auto model : this->contactingModels)
  {
  // Create the joint that will attach the models
  fixedJoint = this->world->Physics()->CreateJoint(
        "fixed", this->model);
  auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
  gzdbg << "Creating fixed joint: " << jointName << std::endl;
  fixedJoint->SetName(jointName);

  model->SetGravityMode(false);

  // Lift the part slightly because it will fall through the tray if the tray is animated
  model->SetWorldPose(model->WorldPose() + ignition::math::Pose3d(0,0,0.0001,0,0,0));

  auto modelName = model->GetName();
  auto linkName = modelName + "::link";
  auto link = model->GetLink(linkName);
  if (link == NULL)
  {
    // If the model was inserted into the world using the "population" SDF tag,
    // the link will have an additional namespace of the model type.
    linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
    link = model->GetLink(linkName);
    if (link == NULL)
    {
      gzwarn << "Couldn't find link to make joint with: " << linkName;
      continue;
    }
  }
  link->SetGravityMode(false);
  fixedJoint->Load(link, this->parentLink, ignition::math::Pose3d());
  fixedJoint->Attach(this->parentLink, link);
  fixedJoint->Init();
  this->fixedJoints.push_back(fixedJoint);
  model->SetAutoDisable(true);
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;

  for (auto fixedJoint : this->fixedJoints)
  {
    // gzdbg << fixedJoint->GetParent()->GetName() << "\n";
    // gzdbg << fixedJoint->GetChild()->GetName() << "\n";
    fixedJoint->Detach();
  }
  this->fixedJoints.clear();
  for (auto model : this->contactingModels)
  {
    gzdbg << model->GetName() << "\n";
    model->SetGravityMode(true);
    model->SetAutoDisable(false);
  }
  //   auto modelName = model->GetName();
  //   auto linkName = modelName + "::link";
  //   auto link = model->GetLink(linkName);
  //   if (link == NULL)
  //   {
  //     // If the model was inserted into the world using the "population" SDF tag,
  //     // the link will have an additional namespace of the model type.
  //     linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
  //     link = model->GetLink(linkName);
  //     if (link == NULL)
  //     {
  //       gzwarn << "Couldn't find link to remove joint with: " << linkName;
  //       continue;
  //     }
  //   }
  //   link->SetGravityMode(true);
  //   model->SetAutoDisable(true);
  // }
}

/////////////////////////////////////////////////
void KitTrayPlugin::HandleLockModelsRequest(ConstGzStringPtr &_msg)
{
  gzdbg << this->trayID << ": Handle lock/unlock models service called.\n";

  // gzdbg << "lockunlock---------- " << _msg->data() << "\n";

  if (_msg->data() == "lock")
  {
    this->LockContactingModels();
  }
  else if (_msg->data() == "unlock")
  {
    this->UnlockContactingModels();
  }

  // (void)_msg;
  
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleClearService(
  ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << this->trayID << ": Handle clear tray service called by: " << callerName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  this->UnlockContactingModels();
  this->ClearContactingModels();
  res.success = true;
  return true;
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleGetContentService(
  ros::ServiceEvent<nist_gear::DetectKittingShipment::Request, nist_gear::DetectKittingShipment::Response> & event)
{
  const std::string& callerName = event.getCallerName();
  gzdbg << this->trayID << ": Handle get content service called by: " << callerName << std::endl;
  // ROS_ERROR_STREAM(this->trayID << ": Handle get content service called by: " << callerName);

  auto & response = event.getResponse();

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return true;
  }

  int agv_id = std::stoi(this->trayID.substr(3,1)); //--get the id of the agv
  // std::string agv_name = this->trayID.substr(3,1);

  std::string current_station{};
  if (agv_id == 1)
  {
    current_station = this->agv1CurrentStation;
  }
  if (agv_id == 2)
  {
    current_station = this->agv2CurrentStation;
  }
  if (agv_id == 3)
  {
    current_station = this->agv3CurrentStation;
  }
  if (agv_id == 4)
  {
    current_station = this->agv4CurrentStation;
  }
  this->station_name = current_station;
  // ROS_WARN_STREAM("station " << this->station_name);

  nist_gear::DetectedKittingShipment kitTrayMsg;
  kitTrayMsg.destination_id = this->trayID;
  kitTrayMsg.station_id = this->station_name;
  
  for (const auto &obj : this->currentKit.objects)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = obj.type;
    msgObj.is_faulty = obj.isFaulty;
    msgObj.pose.position.x = obj.pose.Pos().X();
    msgObj.pose.position.y = obj.pose.Pos().Y();
    msgObj.pose.position.z = obj.pose.Pos().Z();
    msgObj.pose.orientation.x = obj.pose.Rot().X();
    msgObj.pose.orientation.y = obj.pose.Rot().Y();
    msgObj.pose.orientation.z = obj.pose.Rot().Z();
    msgObj.pose.orientation.w = obj.pose.Rot().W();

    // Add the object to the kit.
    kitTrayMsg.products.push_back(msgObj);
  }

  response.shipment = kitTrayMsg;
  return true;
}

void KitTrayPlugin::PublishTFTransform(const common::Time sim_time)
{
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
