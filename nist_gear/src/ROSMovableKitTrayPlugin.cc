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
#include "ROSMovableKitTrayPlugin.hh"
#include "nist_gear/TrayContents.h"
#include "nist_gear/DetectKitTrayContent.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MovableKitTrayPlugin)

/////////////////////////////////////////////////
MovableKitTrayPlugin::MovableKitTrayPlugin()
{
}

/////////////////////////////////////////////////
MovableKitTrayPlugin::~MovableKitTrayPlugin()
{
  this->updateConnection.reset();
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "MovableKitTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "MovableKitTrayPlugin running at the default update rate\n";

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

  // ROS Topic to publish the content of a tray including
  // the movable kit tray and products in the movable kit tray
  std::string tray_content = "";
  if (_sdf->HasElement("all_tray_contents_topic"))
  {
    tray_content = _sdf->Get<std::string>("all_tray_contents_topic");
    this->currentKitPub = this->rosNode->advertise<nist_gear::TrayContents>(
        tray_content, 1000, boost::bind(&MovableKitTrayPlugin::OnSubscriberConnect, this, _1));
  }
  else
  {
    gzerr << "Missing all_tray_contents_topic in ariac.world\n";
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

        // ROS_DEBUG_STREAM("Ignoring part: " << faultyPartName);
        this->faultyPartNames.push_back(faultyPartName);
        faultyPartElem = faultyPartElem->GetNextElement("name");
      }
    }
  }

  // Services to get movable tray content
  auto tray_name = this->model->GetName();

  std::string lock_unlock_service_str{};
  if (_sdf->HasElement("lock_unlock_kt1_topic"))
  {
    lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt1_topic");

    this->lock_unlock_kt1_gz_sub =
        this->gzNode->Subscribe(lock_unlock_service_str, &MovableKitTrayPlugin::LockUnlockServiceCallback, this);
  }
  else
  {
    gzerr << "Missing lock_unlock_kt1_topic in ariac.world\n";
  }
  if (_sdf->HasElement("lock_unlock_kt2_topic"))
  {
    lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt2_topic");

    this->lock_unlock_kt2_gz_sub =
        this->gzNode->Subscribe(lock_unlock_service_str, &MovableKitTrayPlugin::LockUnlockServiceCallback, this);
  }
  else
  {
    gzerr << "Missing lock_unlock_kt2_topic in ariac.world\n";
  }
  if (_sdf->HasElement("lock_unlock_kt3_topic"))
  {
    lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt3_topic");

    this->lock_unlock_kt3_gz_sub =
        this->gzNode->Subscribe(lock_unlock_service_str, &MovableKitTrayPlugin::LockUnlockServiceCallback, this);
  }
  else
  {
    gzerr << "Missing lock_unlock_kt3_topic in ariac.world\n";
  }
  if (_sdf->HasElement("lock_unlock_kt4_topic"))
  {
    lock_unlock_service_str = _sdf->Get<std::string>("lock_unlock_kt4_topic");

    this->lock_unlock_kt4_gz_sub =
        this->gzNode->Subscribe(lock_unlock_service_str, &MovableKitTrayPlugin::LockUnlockServiceCallback, this);
  }
  else
  {
    gzerr << "Missing lock_unlock_kt4_topic in ariac.world\n";
  }

  if (_sdf->HasElement("model_name"))
  {
    std::string model_name_str = _sdf->Get<std::string>("model_name");

    this->movable_tray_name = model_name_str;
  }

  // only advertise the service for the current movable tray
  if (tray_name.compare("movable_tray_dark_wood_1") == 0)
  {
    if (_sdf->HasElement("get_content_dwood_1"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_dwood_1");
      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_dark_wood_2") == 0)
  {
    if (_sdf->HasElement("get_content_dwood_2"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_dwood_2");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_light_wood_1") == 0)
  {
    if (_sdf->HasElement("get_content_lwood_1"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_lwood_1");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_light_wood_2") == 0)
  {
    if (_sdf->HasElement("get_content_lwood_2"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_lwood_2");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_metal_rusty_1") == 0)
  {
    if (_sdf->HasElement("get_content_mrusty_1"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_mrusty_1");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_metal_rusty_2") == 0)
  {
    if (_sdf->HasElement("get_content_mrusty_2"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_mrusty_2");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_metal_shiny_1") == 0)
  {
    if (_sdf->HasElement("get_content_mshiny_1"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_mshiny_1");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }
  else if (tray_name.compare("movable_tray_metal_shiny_2") == 0)
  {
    if (_sdf->HasElement("get_content_mshiny_2"))
    {
      std::string content_service = _sdf->Get<std::string>("get_content_mshiny_2");

      this->movable_tray_content_srv =
          this->rosNode->advertiseService(content_service, &MovableKitTrayPlugin::HandleMovableTrayContent, this);
    }
  }

  // hardcoded
//   this->kit_tray_contents_server = this->rosNode->advertiseService(
//       "/ariac/kit_tray/get_content", &MovableKitTrayPlugin::HandleGetKitTrayContent, this);

  this->publishingEnabled = true;

  tray_name.erase(tray_name.length() - 2);  // removes last 2 characters
  this->tray_model = tray_name;

  // gzerr << "[TRAY MODEL]" << tray_model << std::endl;
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::LockUnlockServiceCallback(ConstGzStringPtr& _msg)
{
  if (_msg->data() == "lock")
  {
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));
    this->LockContactingModels();
  }
  else if (_msg->data() == "unlock")
  {
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));
    this->UnlockContactingModels();
  }
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // check the movable tray has been placed on a kit tray before updating anything here

  // If we are using a custom update rate value we have to check if it's time to
  // update the plugin or not.

  if (!this->TimeToExecute())
  {
    return;
  }

  if (!this->newMsg)
  {
    return;
  }

  // gzerr << "EXECUTING" << std::endl;
  // Do the following only when the tray is placed on the AGV
  // If the y position of a tray is > 4.9 then the tray is on the table
  // if (this->parentLink->WorldPose().Pos().Y() > 4.9) {
  //     return;
  // }

  std::set<physics::ModelPtr> prevContactingModels(this->contactingModels);
  this->CalculateContactingModels();
  // if (prevContactingModels.size() != this->contactingModels.size()) {
  //     gzdbg << this->parentLink->GetScopedName() << ": number of contacting models: "
  //         << this->contactingModels.size() << std::endl;
  //     // ROS_DEBUG_STREAM(this->parentLink->GetScopedName() << ": number of contacting models: "
  //     //     << this->contactingModels.size() << std::endl);
  // }

  // Look for models that were contacting tray but now aren't.
  // Either they've been fixed to tray or have been removed from tray
  std::vector<physics::ModelPtr> removedContactingModels;
  std::set_difference(prevContactingModels.begin(), prevContactingModels.end(), this->contactingModels.begin(),
                      this->contactingModels.end(),
                      std::inserter(removedContactingModels, removedContactingModels.begin()));
  for (auto model : removedContactingModels)
  {
    // gzerr << "Removing contacting model: " << model->GetName() << std::endl;
    if (model->GetName() != "tray_table1" and
      model->GetName() != "tray_table2" and
      model->GetName() != "bin1" and
      model->GetName() != "bin2" and
      model->GetName() != "bin3" and
      model->GetName() != "bin4" and
      model->GetName() != "bin5" and
      model->GetName() != "bin6" and
      model->GetName() != "bin7" and
      model->GetName() != "bin8")

      // if (model->GetName().compare("agv1") != 0 &&
      //     model->GetName().compare("agv2") != 0 &&
      //     model->GetName().compare("agv3") != 0 &&
      //     model->GetName().compare("agv4") != 0) {

      // gzdbg << "<<<<<<<< removed contact " << model->GetName() << std::endl;

      model->SetAutoDisable(true);
    // }
  }

  this->ProcessContactingModels();
  if (this->publishingEnabled)
  {
    this->PublishKitMsg();
  }
  // this->PublishTFTransform(_info.simTime);
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::ProcessContactingModels()
{
  auto trayPose = this->parentLink->WorldPose();
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto trayPartJoint : this->trayPartJoints)
  {
    auto link = trayPartJoint->GetChild();
    // gzerr << "<<<<<<<<< LINK: " << link->GetParentModel()->GetName() << std::endl;
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
  this->currentKit.products.clear();

  for (auto model : this->contactingModels)
  {
    // gzerr << "CONTACTING MODEL NAME: " << model->GetName() << std::endl;
    if (model->GetName() != "tray_table1" and
      model->GetName() != "tray_table2" and
      model->GetName() != "bin1" and
      model->GetName() != "bin2" and
      model->GetName() != "bin3" and
      model->GetName() != "bin4" and
      model->GetName() != "bin5" and
      model->GetName() != "bin6" and
      model->GetName() != "bin7" and
      model->GetName() != "bin8")
    {
      // gzerr << "GET NAME: " << model->GetName() << std::endl;
      model->SetAutoDisable(false);
    }

    ariac::KitObject object;

    // Determine the object type
    object.type = ariac::DetermineModelType(model->GetName());
    // gzwarn << "[OBJECT TYPE] -> " << object.type << std::endl;

    // Determine if the object is faulty
    auto modelName = ariac::TrimNamespace(model->GetName());
    auto it = std::find(this->faultyPartNames.begin(), this->faultyPartNames.end(), modelName);
    object.isFaulty = it != this->faultyPartNames.end();

    // Determine the pose of the object in the frame of the movable tray
    ignition::math::Pose3d objectPose = model->WorldPose();
    ignition::math::Matrix4d transMat(trayPose);
    ignition::math::Matrix4d objectPoseMat(objectPose);
    object.pose = (transMat.Inverse() * objectPoseMat).Pose();
    object.pose.Rot().Normalize();

    std::string object_type = object.type;
    // make sure this model is an ariac part
    auto model_it = std::find(this->grippable_model_types.begin(), this->grippable_model_types.end(), object_type);
    bool grippable_model = model_it != this->grippable_model_types.end();
    if (grippable_model)
    {
      this->currentKit.products.push_back(object);
    }
    this->currentKit.movable_tray.type = ariac::DetermineModelType(this->model->GetName());
    this->currentKit.movable_tray.name = this->model->GetName();
    if (object.type.compare("kit_tray") == 0)
    {
      this->currentKit.kit_tray_name = model->GetName();
      // Determine the pose of the movable tray in the frame of the kit tray
      ignition::math::Pose3d movable_tray_pose = this->model->WorldPose();
      ignition::math::Matrix4d transform_mat(model->WorldPose());
      ignition::math::Matrix4d movable_tray_pose_mat(movable_tray_pose);
      movable_tray_pose = (transform_mat.Inverse() * movable_tray_pose_mat).Pose();
      movable_tray_pose.Rot().Normalize();
      this->currentKit.movable_tray.pose = movable_tray_pose;
    }
  }
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  // gzwarn << this->trayID << ": New subscription from node: " << subscriberName << std::endl;

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

// std::string MovableKitTrayPlugin::GetAgv(double movable_tray_y_position) {
//     if (inRange(4.675404, movable_tray_y_position))
//         return "agv1";
//     else if (inRange(1.367643, movable_tray_y_position))
//         return "agv2";
//     else if (inRange(-1.333917, movable_tray_y_position))
//         return "agv3";
//     else if (inRange(-4.696062, movable_tray_y_position))
//         return "agv3";
//     else return "null";
// }

// std::string MovableKitTrayPlugin::GetStation(double movable_tray_x_position, double movable_tray_y_position) {
//     if (inRange(-2.265685, movable_tray_x_position)) {
//         if (inRange(4.675404, movable_tray_y_position))
//             return "ks1";
//         else if (inRange(1.367643, movable_tray_y_position))
//             return "ks2";
//         else if (inRange(-1.333917, movable_tray_y_position))
//             return "ks3";
//         else if (inRange(-4.696062, movable_tray_y_position))
//             return "ks4";
//         else return "null";
//     }
//     else if (inRange(-5.600000, movable_tray_x_position)) {
//         if (inRange(4.675404, movable_tray_y_position))
//             return "as1";
//         else if (inRange(1.367643, movable_tray_y_position))
//             return "as1";
//         else if (inRange(-1.333917, movable_tray_y_position))
//             return "as3";
//         else if (inRange(-4.696062, movable_tray_y_position))
//             return "as3";
//         else return "null";
//     }
//     else if (inRange(-10.590274, movable_tray_x_position)) {
//         if (inRange(4.675404, movable_tray_y_position))
//             return "as2";
//         else if (inRange(1.367643, movable_tray_y_position))
//             return "as2";
//         else if (inRange(-1.333917, movable_tray_y_position))
//             return "as4";
//         else if (inRange(-4.696062, movable_tray_y_position))
//             return "as4";
//         else return "null";
//     }
//     else return "null";
// }

/////////////////////////////////////////////////
void MovableKitTrayPlugin::PublishKitMsg()
{
  // agv1 @ ks1: [-2.265685, 4.675404]
  // agv1 @ as1: [-5.600000, 4.675404]
  // agv1 @ as2: [-10.590274, 4.675404]

  // agv2 @ ks2: [-2.265685, 1.367643]
  // agv2 @ as1: [-5.600000, 1.367643]
  // agv2 @ as2: [-10.590274, 1.367643]

  // agv3 @ ks3: [-2.265685, -1.333917]
  // agv3 @ as3: [-5.600000, -1.333917]
  // agv3 @ as4: [-10.590274, -1.333917]

  // agv4 @ ks4: [-2.265685, -4.696062]
  // agv4 @ as3: [-5.600000, -4.696062]
  // agv4 @ as4: [-10.590274, -4.696062]

  // Publish current kit
  nist_gear::TrayContents tray_content;

  auto movable_tray_name = this->model->GetName();
  auto movable_tray_model = this->model->GetName();
  movable_tray_model.erase(movable_tray_model.length() - 2);  // removes last 2 characters, e.g. _1
  tray_content.movable_tray.movable_tray_type = movable_tray_model;

  tray_content.movable_tray.movable_tray_name = movable_tray_name;

  tray_content.movable_tray.movable_tray_pose.position.x = this->currentKit.movable_tray.pose.Pos().X();
  tray_content.movable_tray.movable_tray_pose.position.y = this->currentKit.movable_tray.pose.Pos().Y();
  tray_content.movable_tray.movable_tray_pose.position.z = this->currentKit.movable_tray.pose.Pos().Z();
  tray_content.movable_tray.movable_tray_pose.orientation.x = this->currentKit.movable_tray.pose.Rot().X();
  tray_content.movable_tray.movable_tray_pose.orientation.y = this->currentKit.movable_tray.pose.Rot().Y();
  tray_content.movable_tray.movable_tray_pose.orientation.z = this->currentKit.movable_tray.pose.Rot().Z();
  tray_content.movable_tray.movable_tray_pose.orientation.w = this->currentKit.movable_tray.pose.Rot().W();

  if (this->currentKit.kit_tray_name == "kit_tray_1")
    tray_content.kit_tray = "agv1";
  else if (this->currentKit.kit_tray_name == "kit_tray_2")
    tray_content.kit_tray = "agv2";
  else if (this->currentKit.kit_tray_name == "kit_tray_3")
    tray_content.kit_tray = "agv3";
  else if (this->currentKit.kit_tray_name == "kit_tray_4")
    tray_content.kit_tray = "agv4";

  for (const auto& product : this->currentKit.products)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = product.type;
    msgObj.is_faulty = product.isFaulty;
    msgObj.pose.position.x = product.pose.Pos().X();
    msgObj.pose.position.y = product.pose.Pos().Y();
    msgObj.pose.position.z = product.pose.Pos().Z();
    msgObj.pose.orientation.x = product.pose.Rot().X();
    msgObj.pose.orientation.y = product.pose.Rot().Y();
    msgObj.pose.orientation.z = product.pose.Rot().Z();
    msgObj.pose.orientation.w = product.pose.Rot().W();
    tray_content.products.push_back(msgObj);
  }
  this->currentKitPub.publish(tray_content);
}

///////////////////////////////////////////////////////////
void MovableKitTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  

  for (auto part : this->contactingModels)
  {
    physics::JointPtr trayPartJoint;
    auto model_name = part->GetName();
    auto model_type = ariac::DetermineModelType(part->GetName());

    // make sure this model is an ariac part
    auto it = std::find(this->grippable_model_types.begin(), this->grippable_model_types.end(), model_type);
    bool grippable_model = it != this->grippable_model_types.end();
    if (grippable_model)
    {
      trayPartJoint = this->world->Physics()->CreateJoint("fixed", this->model);
      auto jointName = this->model->GetName() + "_" + part->GetName() + "__joint__";
      gzerr << "Creating fixed joint: " << jointName << std::endl;
      trayPartJoint->SetName(jointName);

      part->SetGravityMode(false);
      // Lift the part slightly because it will fall through the tray if the tray is animated
      part->SetWorldPose(part->WorldPose() + ignition::math::Pose3d(0, 0, 0.0015, 0, 0, 0));

      auto modelName = part->GetName();
      auto linkName = modelName + "::link";
      auto link = part->GetLink(linkName);
      if (link == NULL)
      {
        // If the model was inserted into the world using the "population" SDF tag,
        // the link will have an additional namespace of the model type.
        auto linkName = modelName + "::" + model_type + "::link";
        link = part->GetLink(linkName);
        if (link == NULL)
        {
          gzwarn << "Couldn't find link to make joint with: " << linkName;
          continue;
        }
      }
      link->SetGravityMode(false);
      trayPartJoint->Load(link, this->parentLink, ignition::math::Pose3d());
      trayPartJoint->SetModel(this->model);
      trayPartJoint->Attach(this->parentLink, link);
      trayPartJoint->Init();
      this->trayPartJoints.push_back(trayPartJoint);
      part->SetAutoDisable(false);
    }
  }
}

/////////////////////////////////////////////////
void MovableKitTrayPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);

  for (auto trayPartJoint : this->trayPartJoints)
  {
    if (trayPartJoint->GetChild()->GetName().compare("link") == 0)
    {
      gzwarn << "[DETACHING]->" << trayPartJoint->GetChild()->GetName() << std::endl;
      trayPartJoint->Detach();
    }
  }

  trayPartJoints.clear();

  // put gravity back on for parts on the tray
  for (auto model : this->contactingModels)
  {
    auto model_name = model->GetName();

    auto model_type = ariac::DetermineModelType(model->GetName());
    gzwarn << "Model type: " << model_type << std::endl;
    // gzerr << "UNLOCK CONTACTING MODEL: " << model_name << std::endl;

    // make sure this model is an ariac part
    auto it = std::find(this->grippable_model_types.begin(), this->grippable_model_types.end(), model_type);
    bool grippable_model = it != this->grippable_model_types.end();
    if (grippable_model)
    {
      gzwarn << "GAVITY ON" << std::endl;
      model->SetGravityMode(true);
      model->SetAutoDisable(true);
    }
  }

  // gzwarn << "[FIXED JOINTS SIZE3]->" << this->trayPartJoints.size() << std::endl;
}

// bool MovableKitTrayPlugin::HandleGetKitTrayContent(
//     ros::ServiceEvent<nist_gear::DetectKitTrayContent::Request, nist_gear::DetectKitTrayContent::Response>& event)
// {
//   const std::string& callerName = event.getCallerName();
//   auto& request = event.getRequest();
//   ROS_INFO_STREAM("REQUEST: " << request);
//   auto& response = event.getResponse();

//   // During the competition, this environment variable will be set.
//   auto compRunning = std::getenv("ARIAC_COMPETITION");
//   if (compRunning && callerName.compare("/gazebo") != 0)
//   {
//     std::string errStr = "Competition is running so this service is not enabled.";
//     gzerr << errStr << std::endl;
//     ROS_ERROR_STREAM(errStr);
//     return true;
//   }

//   nist_gear::TrayContents tray_content;

//   auto movable_tray_name = this->model->GetName();
//   auto movable_tray_model = this->model->GetName();
//   movable_tray_model.erase(movable_tray_model.length() - 2);  // removes last 2 characters, e.g. _1
//   tray_content.movable_tray.movable_tray_type = movable_tray_model;

//   tray_content.movable_tray.movable_tray_name = movable_tray_name;

//   // pose of the movable tray, relative to the kit_tray
//   tray_content.movable_tray.movable_tray_pose.position.x = this->currentKit.movable_tray.pose.Pos().X();
//   tray_content.movable_tray.movable_tray_pose.position.y = this->currentKit.movable_tray.pose.Pos().Y();
//   tray_content.movable_tray.movable_tray_pose.position.z = this->currentKit.movable_tray.pose.Pos().Z();
//   tray_content.movable_tray.movable_tray_pose.orientation.x = this->currentKit.movable_tray.pose.Rot().X();
//   tray_content.movable_tray.movable_tray_pose.orientation.y = this->currentKit.movable_tray.pose.Rot().Y();
//   tray_content.movable_tray.movable_tray_pose.orientation.z = this->currentKit.movable_tray.pose.Rot().Z();
//   tray_content.movable_tray.movable_tray_pose.orientation.w = this->currentKit.movable_tray.pose.Rot().W();

//   if (this->currentKit.kit_tray_name == request.kit_tray)
//   {
//     for (const auto& product : this->currentKit.products)
//     {
//       nist_gear::DetectedProduct msgObj;
//       msgObj.type = product.type;
//       msgObj.is_faulty = product.isFaulty;
//       msgObj.pose.position.x = product.pose.Pos().X();
//       msgObj.pose.position.y = product.pose.Pos().Y();
//       msgObj.pose.position.z = product.pose.Pos().Z();
//       msgObj.pose.orientation.x = product.pose.Rot().X();
//       msgObj.pose.orientation.y = product.pose.Rot().Y();
//       msgObj.pose.orientation.z = product.pose.Rot().Z();
//       msgObj.pose.orientation.w = product.pose.Rot().W();
//       tray_content.products.push_back(msgObj);
//     }

//     response.kit_tray_content = tray_content;
//   }
//   else{
//       gzerr << "No movable tray found on this kit tray\n";
//   }

//   return true;
// }

bool MovableKitTrayPlugin::HandleMovableTrayContent(
    ros::ServiceEvent<nist_gear::DetectKittingShipment::Request, nist_gear::DetectKittingShipment::Response>& event)
{
  const std::string& callerName = event.getCallerName();

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

  nist_gear::TrayContents tray_content;

  auto movable_tray_name = this->model->GetName();
  auto movable_tray_model = this->model->GetName();
  movable_tray_model.erase(movable_tray_model.length() - 2);  // removes last 2 characters, e.g. _1
  tray_content.movable_tray.movable_tray_type = movable_tray_model;

  tray_content.movable_tray.movable_tray_name = movable_tray_name;

  // pose of the movable tray, relative to the kit_tray
  tray_content.movable_tray.movable_tray_pose.position.x = this->currentKit.movable_tray.pose.Pos().X();
  tray_content.movable_tray.movable_tray_pose.position.y = this->currentKit.movable_tray.pose.Pos().Y();
  tray_content.movable_tray.movable_tray_pose.position.z = this->currentKit.movable_tray.pose.Pos().Z();
  tray_content.movable_tray.movable_tray_pose.orientation.x = this->currentKit.movable_tray.pose.Rot().X();
  tray_content.movable_tray.movable_tray_pose.orientation.y = this->currentKit.movable_tray.pose.Rot().Y();
  tray_content.movable_tray.movable_tray_pose.orientation.z = this->currentKit.movable_tray.pose.Rot().Z();
  tray_content.movable_tray.movable_tray_pose.orientation.w = this->currentKit.movable_tray.pose.Rot().W();

  if (this->currentKit.kit_tray_name == "kit_tray_1")
    tray_content.kit_tray = "agv1";
  else if (this->currentKit.kit_tray_name == "kit_tray_2")
    tray_content.kit_tray = "agv2";
  else if (this->currentKit.kit_tray_name == "kit_tray_3")
    tray_content.kit_tray = "agv3";
  else if (this->currentKit.kit_tray_name == "kit_tray_4")
    tray_content.kit_tray = "agv4";

  for (const auto& product : this->currentKit.products)
  {
    nist_gear::DetectedProduct msgObj;
    msgObj.type = product.type;
    msgObj.is_faulty = product.isFaulty;
    msgObj.pose.position.x = product.pose.Pos().X();
    msgObj.pose.position.y = product.pose.Pos().Y();
    msgObj.pose.position.z = product.pose.Pos().Z();
    msgObj.pose.orientation.x = product.pose.Rot().X();
    msgObj.pose.orientation.y = product.pose.Rot().Y();
    msgObj.pose.orientation.z = product.pose.Rot().Z();
    msgObj.pose.orientation.w = product.pose.Rot().W();
    tray_content.products.push_back(msgObj);
  }

  response.shipment = tray_content;
  return true;
}
