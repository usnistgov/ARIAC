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

#include "ROSAriacGantryTrayPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GantryTrayPlugin)

/////////////////////////////////////////////////
GantryTrayPlugin::GantryTrayPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
GantryTrayPlugin::~GantryTrayPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void GantryTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);


  if (this->updateRate > 0)
    gzdbg << "GantryTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "GantryTrayPlugin running at the default update rate\n";

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");

    this->currentTrayPub = this->rosNode->advertise<nist_gear::DetectedKittingShipment>(
    "/ariac/gantry_tray", 1000, boost::bind(&GantryTrayPlugin::OnSubscriberConnect, this, _1));
 

  this->tf_frame_name = "gantry_tray_frame";;
  if (_sdf->HasElement("frameName"))
    this->tf_frame_name = _sdf->Get<std::string>("frameName");

  // ROS service for clearing the tray
  std::string clearServiceName{};
  if (_sdf->HasElement("clear_tray_service_name"))
    clearServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  this->clearTrayServer =
    this->rosNode->advertiseService(clearServiceName, &GantryTrayPlugin::HandleClearService, this);

  // Gazebo subscription for the lock trays topic
  std::string lockModelsServiceName{};
  if (_sdf->HasElement("lock_models_service_name"))
    lockModelsServiceName = _sdf->Get<std::string>("lock_models_service_name");
  this->lockModelsServer =
    this->rosNode->advertiseService(lockModelsServiceName, &GantryTrayPlugin::HandleLockModelsRequest, this);

  // this->lockModelsSub = this->gzNode->Subscribe(
  //   lockModelsServiceName, &GantryTrayPlugin::HandleLockModelsRequest, this);

  std::string unlockModelsServiceName{};
  if (_sdf->HasElement("unlock_models_service_name"))
    unlockModelsServiceName = _sdf->Get<std::string>("unlock_models_service_name");
  this->unlockModelsServer =
    this->rosNode->advertiseService(unlockModelsServiceName, &GantryTrayPlugin::HandleUnlockModelsRequest, this);
  // this->unlockModelsSub = this->gzNode->Subscribe(
  //   unlockModelsServiceName, &GantryTrayPlugin::HandleUnlockModelsRequest, this);

    // Initialize Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  // cache tray pose
  //FIXME
  this->tray_pose = this->model->WorldPose();
  // gzdbg << "Gantry tray pose: " << this->tray_pose << "\n";
}

/////////////////////////////////////////////////
void GantryTrayPlugin::OnUpdate(const common::UpdateInfo & _info)
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
      << this->contactingModels.size());
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
  this->PublishTFTransform(_info.simTime);
}

/////////////////////////////////////////////////
void GantryTrayPlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto fixedJoint : this->fixedJoints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
//   this->currentKit.objects.clear();
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

    //   this->currentKit.objects.push_back(object);
    }
  }
}

/////////////////////////////////////////////////
void GantryTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzwarn << "New subscription from node: " << subscriberName << std::endl;
}


/////////////////////////////////////////////////
void GantryTrayPlugin::UnlockContactingModels()
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
    model->SetGravityMode(false);
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
        gzwarn << "Couldn't find link to remove joint with: " << linkName;
        continue;
      }
    }
    link->SetGravityMode(true);
    model->SetAutoDisable(true);
  }
}
// void GantryTrayPlugin::UnlockContactingModels()
// {
//   boost::mutex::scoped_lock lock(this->mutex);
//   physics::JointPtr fixedJoint;
//   for (auto fixedJoint : this->fixedJoints)
//   {
//     fixedJoint->Detach();
//   }
//   this->fixedJoints.clear();
// }

/////////////////////////////////////////////////
void GantryTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  gzdbg << "Number of models in contact with the tray: " << this->contactingModels.size() << std::endl;
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
  model->SetWorldPose(model->WorldPose() + ignition::math::Pose3d(0,0,0.01,0,0,0));

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
// void GantryTrayPlugin::HandleLockModelsRequest(ConstGzStringPtr &_msg)
// {
//   gzdbg << "Handle Lock Models service called.\n";
//   (void)_msg;
//   this->LockContactingModels();
// }





bool GantryTrayPlugin::HandleLockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, 
std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << "Handle lock models service called by: " << callerName << std::endl;

  // (void)_msg;
  this->LockContactingModels();
  res.success = true;
  return true;
}

// void GantryTrayPlugin::HandleUnlockModelsRequest(ConstGzStringPtr &_msg)
// {
//   gzdbg << "Handle Unlock Models service called.\n";
//   (void)_msg;
//   this->UnlockContactingModels();
// }

bool GantryTrayPlugin::HandleUnlockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, 
std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << "Handle unlock tray service called by: " << callerName << std::endl;

  this->UnlockContactingModels();
  res.success = true;
  return true;
}


/////////////////////////////////////////////////
bool GantryTrayPlugin::HandleClearService(
  ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << "Handle gantry clear tray service called by: " << callerName << std::endl;

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


void GantryTrayPlugin::PublishTFTransform(const common::Time sim_time)
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
