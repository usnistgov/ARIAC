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

#include "ROSLogicalCameraPlugin.hh"

#include "nist_gear/ARIAC.hh"
#include "nist_gear/LogicalCameraImage.h"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo/sensors/Noise.hh"
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <algorithm>
#include <sstream>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSLogicalCameraPlugin);

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::ROSLogicalCameraPlugin()
{
}

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::~ROSLogicalCameraPlugin()
{
  this->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace = "logical_camera";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement(
        "robotNamespace")->Get<std::string>() + "/";
  }

  this->world = _parent->GetWorld();
  this->name = _parent->GetName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->onlyPublishKnownModels = false;
  if (_sdf->HasElement("known_model_types"))
  {
    ROS_DEBUG("Only publishing known model types");
    this->onlyPublishKnownModels = true;
    this->knownModelTypes.clear();
    sdf::ElementPtr knownModelTypesElem = _sdf->GetElement("known_model_types");
    if (!knownModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <known_model_types> section\n";
      return;
    }
    sdf::ElementPtr knownModelTypeElem = knownModelTypesElem->GetElement("type");
    while (knownModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = knownModelTypeElem->Get<std::string>();

      ROS_DEBUG_STREAM("New known model type: " << type);
      this->knownModelTypes.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }

  if (_sdf->HasElement("known_model_names"))
  {
    ROS_DEBUG("Only publishing known model names");
    this->onlyPublishKnownModels = true;
    this->knownModelNames.clear();
    sdf::ElementPtr knownModelNamesElem = _sdf->GetElement("known_model_names");
    if (knownModelNamesElem->HasElement("name"))
    {
      sdf::ElementPtr knownModelNameElem = knownModelNamesElem->GetElement("name");
      while (knownModelNameElem)
      {
        std::string knownModelName = knownModelNameElem->Get<std::string>();

        ROS_DEBUG_STREAM("New known model name: " << knownModelName);
        this->knownModelNames.push_back(knownModelName);
        knownModelNameElem = knownModelNameElem->GetNextElement("name");
      }
    }
  }

  this->anonymizeModels = false;
  if (_sdf->HasElement("anonymize_models"))
  {
    this->anonymizeModels = _sdf->Get<bool>("anonymize_models");
  }
  if (this->anonymizeModels)
  {
    ROS_DEBUG("Anonymizing model types");
  } else {
    ROS_DEBUG("Not anonymizing model types");
  }

  this->modelFramePrefix = this->name + "_";
  if (_sdf->HasElement("model_frame_prefix"))
  {
    this->modelFramePrefix = _sdf->GetElement("model_frame_prefix")->Get<std::string>();
  }
  gzdbg << "Using model frame prefix of: " << this->modelFramePrefix << std::endl;

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());
  this->rosnode = new ros::NodeHandle(this->robotNamespace);

  this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }
  ignition::math::Vector3d kitTrayPosition = ignition::math::Vector3d(0, 0.15, 0.75);
  ignition::math::Quaterniond kitTrayOrientation = ignition::math::Quaterniond(1, 0, 0, 0);
  this->kitTrayToAgv = ignition::math::Pose3d(kitTrayPosition, kitTrayOrientation);

  // Handle noise model settings.
  if (_sdf->HasElement("position_noise"))
  {
    this->noiseModels["POSITION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("position_noise")->GetElement("noise"),
      "logical_camera");
  }
  if (_sdf->HasElement("orientation_noise"))
  {
    this->noiseModels["ORIENTATION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("orientation_noise")->GetElement("noise"),
      "logical_camera");
  }

  std::string imageTopic_ros = this->name;
  if (_sdf->HasElement("image_topic_ros")) {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
          &ROSLogicalCameraPlugin::OnImage, this);
  gzdbg << "Subscribing to gazebo topic: " << this->sensor->Topic() << "\n";

  this->imagePub = this->rosnode->advertise<nist_gear::LogicalCameraImage>(imageTopic_ros, 1, false);
  gzdbg << "Publishing to ROS topic: " << imagePub.getTopic() << "\n";

  if (_sdf->HasElement("activation_topic"))
  {
    std::string activationTopic = _sdf->Get<std::string>("activation_topic");
    this->activationSub = this->node->Subscribe(activationTopic,
            &ROSLogicalCameraPlugin::OnActivationMsg, this);
  }

 transformBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
}

void ROSLogicalCameraPlugin::FindLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
        break;
      }
    }
    if (this->sensor)
    {
      this->cameraLink = link;
      break;
    }
  }
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  if (!this->publishing)
  {
    return;
  }
  nist_gear::LogicalCameraImage imageMsg;
  ignition::math::Vector3d cameraPosition = msgs::ConvertIgn(_msg->pose().position());
  ignition::math::Quaterniond cameraOrientation =
    msgs::ConvertIgn(_msg->pose().orientation());
  auto cameraPose = ignition::math::Pose3d(cameraPosition, cameraOrientation);
  this->PublishTF(cameraPose, "world", this->name + "_frame");

  imageMsg.pose.position.x = cameraPosition.X();
  imageMsg.pose.position.y = cameraPosition.Y();
  imageMsg.pose.position.z = cameraPosition.Z();
  imageMsg.pose.orientation.x = cameraOrientation.X();
  imageMsg.pose.orientation.y = cameraOrientation.Y();
  imageMsg.pose.orientation.z = cameraOrientation.Z();
  imageMsg.pose.orientation.w = cameraOrientation.W();

  std::ostringstream logStream;
  ignition::math::Pose3d modelPose;
  for (int i = 0; i < _msg->model_size(); ++i)
  {
    std::string modelName = _msg->model(i).name();
    std::string modelType = ariac::DetermineModelType(modelName);

    if (!this->ModelToPublish(modelName, modelType))
    {
      logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
    }
    else
    {
      logStream << "Publishing model: " << modelName << " of type: " << modelType << std::endl;
      ignition::math::Vector3d modelPosition =
        msgs::ConvertIgn(_msg->model(i).pose().position());
      ignition::math::Quaterniond modelOrientation =
        msgs::ConvertIgn(_msg->model(i).pose().orientation());
      modelPose = ignition::math::Pose3d(modelPosition, modelOrientation);

      std::string modelNameToUse;
      std::string modelTypeToUse;
      if (this->anonymizeModels)
      {
        modelNameToUse = "model_" + ariac::DetermineModelId(modelName);
        modelTypeToUse = "model";
      }
      else
      {
        modelNameToUse = ariac::TrimNamespace(modelName);
        modelTypeToUse = modelType;
      }
      std::string modelFrameId = this->modelFramePrefix + modelNameToUse + "_frame";

      bool isAgv = modelType == "agv1" || modelType == "agv2";
      if (isAgv)
      {
        // If AGVs are detected, also publish the pose to the respective kit tray.
        // Add noise to the kit tray pose, not the AGV base (it is too much noise by the time the tray pose is extrapolated)
        auto noisyKitTrayPose = ignition::math::Pose3d(this->kitTrayToAgv);
        this->AddNoise(noisyKitTrayPose);
        if (modelType == "agv1")
        {
          this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix + "kit_tray_1_frame");
        }
        else if (modelType == "agv2")
        {
          this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix + "kit_tray_2_frame");
        }
      }
      else
      {
        this->AddNoise(modelPose);
      }
      this->AddModelToMsg(modelTypeToUse, modelPose, imageMsg);
      this->PublishTF(modelPose, this->name + "_frame", modelFrameId);

    }

    // Check any children models
    auto modelPtr = this->world->ModelByName(modelName);
    auto nestedModels = modelPtr->NestedModels();
    for (auto nestedModel : nestedModels)
    {
      modelName = nestedModel->GetName();
      modelType = ariac::DetermineModelType(modelName);
      if (!this->ModelToPublish(modelName, modelType))
      {
        logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
        continue;
      }
      logStream << "Publishing model: " << modelName << " of type: " << modelType  << std::endl;
      // Convert the world pose of the model into the camera frame
      modelPose = nestedModel->WorldPose() - cameraPose;
      this->AddNoise(modelPose);
      this->AddModelToMsg(modelType, modelPose, imageMsg);
      // Do not publish TF information for nested models (kit_tray) because it's not accurate.
      // See https://bitbucket.org/osrf/ariac/issues/54.
      // this->PublishTF(modelPose, this->name + "_frame", this->modelFramePrefix + ariac::TrimNamespace(modelName) + "_frame");
    }
  }

  if (!logStream.str().empty())
  {
    ROS_DEBUG_THROTTLE(1, "%s", logStream.str().c_str());
  }
  this->imagePub.publish(imageMsg);
}

bool ROSLogicalCameraPlugin::ModelToPublish(
  const std::string & modelName, const std::string & modelType)
{
  bool publishModel = true;

  // Check if there are restrictions on which models to publish
  if (this->onlyPublishKnownModels)
  {
    // Only publish the model if its type is known
    auto it = std::find(this->knownModelTypes.begin(), this->knownModelTypes.end(), modelType);
    bool knownModel = it != this->knownModelTypes.end();
    it = std::find(this->knownModelNames.begin(), this->knownModelNames.end(), ariac::TrimNamespace(modelName));
    knownModel |= it != this->knownModelNames.end();
    publishModel = knownModel;
  }
  return publishModel;
}

void ROSLogicalCameraPlugin::AddNoise(ignition::math::Pose3d & pose)
{
  if (this->noiseModels.find("POSITION_NOISE") != this->noiseModels.end())
  {
    // Apply additive noise to the model position
    pose.Pos().X() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().X());
    pose.Pos().Y() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Y());
    pose.Pos().Z() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Z());
  }

  if (this->noiseModels.find("ORIENTATION_NOISE") != this->noiseModels.end())
  {
    // Create a perturbation quaternion and apply it to the model orientation
    double r = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    double p = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    double y = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    auto pert = ignition::math::Quaterniond(r, p, y);
    pose.Rot() *= pert;
  }
}

void ROSLogicalCameraPlugin::AddModelToMsg(
  const std::string & modelType, const ignition::math::Pose3d & modelPose,
  nist_gear::LogicalCameraImage & imageMsg)
{
  nist_gear::Model modelMsg;
  modelMsg.pose.position.x = modelPose.Pos().X();
  modelMsg.pose.position.y = modelPose.Pos().Y();
  modelMsg.pose.position.z = modelPose.Pos().Z();

  modelMsg.pose.orientation.x = modelPose.Rot().X();
  modelMsg.pose.orientation.y = modelPose.Rot().Y();
  modelMsg.pose.orientation.z = modelPose.Rot().Z();
  modelMsg.pose.orientation.w = modelPose.Rot().W();
  modelMsg.type = modelType;
  imageMsg.models.push_back(modelMsg);
}

void ROSLogicalCameraPlugin::PublishTF(
  const ignition::math::Pose3d & pose, const std::string & parentFrame, const std::string & frame)
{
  ros::Time currentTime = ros::Time::now();

  tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

  tf::Transform transform (qt, vt);
  transformBroadcaster->sendTransform(tf::StampedTransform(transform, currentTime, parentFrame, frame));

}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnActivationMsg(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "activate")
  {
    this->publishing = true;
  }
  else if (_msg->data() == "deactivate")
  {
    this->publishing = false;
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
