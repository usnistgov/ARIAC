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

#include <nist_gear/DetectedAssemblyShipment.h>

#include "ROSAriacBriefcasePlugin.hh"
#include "nist_gear/DetectedConnectedProducts.h"
#include "nist_gear/DetectedConnectedProduct.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(BriefcasePlugin)

/////////////////////////////////////////////////
BriefcasePlugin::BriefcasePlugin() : ModelContactPlugin()
{
  // gzdbg << "BriefcasePlugin Plugin\n";
}

/////////////////////////////////////////////////
BriefcasePlugin::~BriefcasePlugin()
{
  this->updateConnection.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void BriefcasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ModelContactPlugin::Load(_model, _sdf);

  if (_sdf->HasElement("faulty_parts"))
  {
    this->faulty_part_names.clear();
    sdf::ElementPtr faultyPartNamesElem = _sdf->GetElement("faulty_parts");
    if (faultyPartNamesElem->HasElement("name"))
    {
      sdf::ElementPtr faultyPartElem = faultyPartNamesElem->GetElement("name");
      while (faultyPartElem)
      {
        std::string faultyPartName = faultyPartElem->Get<std::string>();

        ROS_DEBUG_STREAM("Ignoring part: " << faultyPartName);
        this->faulty_part_names.push_back(faultyPartName);
        faultyPartElem = faultyPartElem->GetNextElement("name");
      }
    }
  }

  if (this->updateRate > 0)
    gzdbg << "BriefcasePlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "BriefcasePlugin running at the default update rate\n";

  this->briefcase_id = this->model->GetScopedName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->node_handle = new ros::NodeHandle("");

  this->assembly_state_publisher = this->node_handle->advertise<nist_gear::DetectedAssemblyShipment>(
    "/ariac/briefcases", 1000, boost::bind(&BriefcasePlugin::OnSubscriberConnect, this, _1));
  this->is_publishing_enabled = true;

  this->tf_frame_name = "briefcase_frame";
  if (_sdf->HasElement("tf_frame_name"))
    this->tf_frame_name = _sdf->Get<std::string>("tf_frame_name");

  // ROS service for getting the content of the tray
  std::string contentServiceName = "get_content";
  if (_sdf->HasElement("get_content_service_name"))
    contentServiceName = _sdf->Get<std::string>("get_content_service_name");
  this->briefcase_contents_server =
    this->node_handle->advertiseService(contentServiceName, &BriefcasePlugin::HandleGetContentService, this);

  std::string connectedPartsServiceName = "";
  if (_sdf->HasElement("get_connected_part_service_name"))
    connectedPartsServiceName = _sdf->Get<std::string>("get_connected_part_service_name");
  this->briefcase_connected_parts_server =
    this->node_handle->advertiseService(connectedPartsServiceName, &BriefcasePlugin::HandleGetConnectedPartsService, this);

  

  // Initialize Gazebo transport
  this->gz_node = transport::NodePtr(new transport::Node());
  this->gz_node->Init();

  // cache tray pose
  this->briefcase_pose = this->model->WorldPose();
}

/////////////////////////////////////////////////
void BriefcasePlugin::OnUpdate(const common::UpdateInfo & _info)
{
  if (!this->newMsg)
  {
    return;
  }

  std::set<physics::ModelPtr> prevContactingModels(this->contactingModels);
  this->CalculateContactingModels();
  if (prevContactingModels.size() != this->contactingModels.size()) {
    ROS_DEBUG_STREAM(this->model->GetScopedName() << ": number of contacting models: "
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
      //gzdbg << "removed contact " << model->GetName() << std::endl;
      model->SetAutoDisable(true);
    }
  }

  this->ProcessContactingModels();
  if (this->is_publishing_enabled)
  {
    this->PublishAssemblyMsg();
  }
  this->PublishTFTransform(_info.simTime);
}

/////////////////////////////////////////////////
void BriefcasePlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto fixedJoint : this->fixed_joints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
  this->current_assembly.objects.clear();
  auto trayPose = this->model->WorldPose();
  for (auto model : this->contactingModels) {
    if (model) {
      model->SetAutoDisable(false);
      ariac::BriefcaseProduct object;

      // Determine the object type
      object.productType = ariac::DetermineModelType(model->GetName());

      // Determine if the object is faulty
      auto modelName = ariac::TrimNamespace(model->GetName());
      auto it = std::find(this->faulty_part_names.begin(), this->faulty_part_names.end(), modelName);
      object.isProductFaulty = it != this->faulty_part_names.end();

      // Determine the pose of the object in the frame of the tray
      ignition::math::Pose3d objectPose = model->WorldPose();
      ignition::math::Matrix4d transMat(trayPose);
      ignition::math::Matrix4d objectPoseMat(objectPose);
      object.productPose = (transMat.Inverse() * objectPoseMat).Pose();
      object.productPose.Rot().Normalize();

      this->current_assembly.objects.push_back(object);
    }
  }
}

/////////////////////////////////////////////////
void BriefcasePlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzwarn << this->briefcase_id << ": New subscription from node: " << subscriberName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && subscriberName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    // Disable publishing of kit messages.
    // This will break the scoring but ensure competitors can't cheat.
    this->is_publishing_enabled = false;
  }
}

/////////////////////////////////////////////////
void BriefcasePlugin::PublishAssemblyMsg()
{
  // Publish current kit
  nist_gear::DetectedAssemblyShipment assembly_msg;
  assembly_msg.briefcase_id = this->briefcase_id;
  // ROS_INFO_STREAM(kitTrayMsg.destination_id);
  for (const auto &obj : this->current_assembly.objects)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = obj.productType;
    // ROS_INFO_STREAM(obj.type);
    msgObj.is_faulty = obj.isProductFaulty;
    msgObj.pose.position.x = obj.productPose.Pos().X();
    msgObj.pose.position.y = obj.productPose.Pos().Y();
    msgObj.pose.position.z = obj.productPose.Pos().Z();
    msgObj.pose.orientation.x = obj.productPose.Rot().X();
    msgObj.pose.orientation.y = obj.productPose.Rot().Y();
    msgObj.pose.orientation.z = obj.productPose.Rot().Z();
    msgObj.pose.orientation.w = obj.productPose.Rot().W();

    // Add the object to the kit.
    assembly_msg.products.push_back(msgObj);
  }
  this->assembly_state_publisher.publish(assembly_msg);
}

///////////////////////////////////////
bool BriefcasePlugin::HandleGetConnectedPartsService(ros::ServiceEvent<nist_gear::DetectConnectedPartsToBriefcase::Request, 
nist_gear::DetectConnectedPartsToBriefcase::Response> & event)
{
  auto & response = event.getResponse();
  nist_gear::DetectedConnectedProducts connected_products_msg;
  connected_products_msg.briefcase_id = this->briefcase_id;
  for (const auto &obj : this->current_assembly.objects)
  {
    nist_gear::DetectedConnectedProduct msgObj;
    msgObj.type = obj.productType;
    msgObj.is_faulty = obj.isProductFaulty;
    connected_products_msg.products.push_back(msgObj);
  }

  response.connected_products = connected_products_msg;
  return true;
  
}

/////////////////////////////////////////////////
bool BriefcasePlugin::HandleGetContentService(
  ros::ServiceEvent<nist_gear::DetectAssemblyShipment::Request, nist_gear::DetectAssemblyShipment::Response> & event)
{
  const std::string& callerName = event.getCallerName();
  gzdbg << this->briefcase_id << ": Handle get content service called by: " << callerName << std::endl;

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

  nist_gear::DetectedAssemblyShipment assembly_msg;
  assembly_msg.briefcase_id = this->briefcase_id;
  for (const auto &obj : this->current_assembly.objects)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = obj.productType;
    msgObj.is_faulty = obj.isProductFaulty;
    msgObj.pose.position.x = obj.productPose.Pos().X();
    msgObj.pose.position.y = obj.productPose.Pos().Y();
    msgObj.pose.position.z = obj.productPose.Pos().Z();
    msgObj.pose.orientation.x = obj.productPose.Rot().X();
    msgObj.pose.orientation.y = obj.productPose.Rot().Y();
    msgObj.pose.orientation.z = obj.productPose.Rot().Z();
    msgObj.pose.orientation.w = obj.productPose.Rot().W();

    // Add the object to the kit.
    assembly_msg.products.push_back(msgObj);
  }

  response.shipment = assembly_msg;
  return true;
}

void BriefcasePlugin::PublishTFTransform(const common::Time sim_time)
{
  ignition::math::Pose3d objectPose = this->briefcase_pose;
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
