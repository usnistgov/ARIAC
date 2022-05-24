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
/*
 * Desc: Kit tray plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_BRIEFCASE_PLUGIN_HH_
#define _GAZEBO_BRIEFCASE_PLUGIN_HH_

#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <nist_gear/ARIAC.hh>
#include <nist_gear/DetectAssemblyShipment.h>
#include <nist_gear/DetectedAssemblyShipment.h>
#include <nist_gear/DetectConnectedPartsToBriefcase.h>
#include <nist_gear/AttachedAssemblyProduct.h>
#include "ModelContactPlugin.hh"

namespace gazebo
{
/// \brief A plugin for a contact sensor on a briefcase.
class GAZEBO_VISIBLE BriefcasePlugin : public ModelContactPlugin
{
  /// \brief Constructor.
public:
  BriefcasePlugin();

  /// \brief Destructor.
public:
  virtual ~BriefcasePlugin();

  /// \brief Load the model plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Callback that receives the world update event
protected:
  void OnUpdate(const common::UpdateInfo& _info);
  // protected: void OnContactsReceived(ConstContactsPtr& _msg);

  /// \brief Update the briefcase based on which models are in contact
protected:
  void ProcessContactingModels();

  /// \brief Update the kit based on which models are in contact
public:
  std::string DetermineModelType(const std::string& modelName);

  /// \brief Callback for when a new subscriber connects to the Kit ROS publisher
  /// This will check that only the /gazebo node is subscribed during the competition
protected:
  void OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub);

  /// \brief Publish the Assembly ROS message
protected:
  void PublishAssemblyMsg();

  /// \brief Service for getting the content of a briefcase
protected:
  bool HandleGetContentService(ros::ServiceEvent<nist_gear::DetectAssemblyShipment::Request,
                                                 nist_gear::DetectAssemblyShipment::Response>& event);

  /// \brief Service for getting parts connected to a briefcase
protected:
  bool HandleGetConnectedPartsService(ros::ServiceEvent<nist_gear::DetectConnectedPartsToBriefcase::Request,
                                                        nist_gear::DetectConnectedPartsToBriefcase::Response>& event);

protected:
  void BriefcaseContentCB(nist_gear::DetectedAssemblyShipment::ConstPtr msg);
  void AssemblySensorCB(nist_gear::AttachedAssemblyProduct::ConstPtr msg);
  void AssemblyRegulatorCB(nist_gear::AttachedAssemblyProduct::ConstPtr msg);
  void AssemblyPumpCB(nist_gear::AttachedAssemblyProduct::ConstPtr msg);
  void AssemblyBatteryCB(nist_gear::AttachedAssemblyProduct::ConstPtr msg);

protected:
  void PublishTFTransform(const common::Time sim_time);

  /// \brief Assembly which is currently in the briefcase
protected:
  ariac::Assembly current_assembly;

  /// \brief ID of briefcase
protected:
std::string briefcase_id;
std::string briefcase_name;

  /// \brief ID of the station
protected:
  std::string station_id;

  /// \brief Fixed joints to lock contacting models
protected:
  std::vector<physics::JointPtr> fixed_joints;

  /// \brief ROS node handle
protected:
  ros::NodeHandle* node_handle;

  /// \brief Gazebo node for communication
protected:
  transport::NodePtr gz_node;

  /// \brief Publisher for the assembly state
  //@todo
protected:
  ros::Publisher assembly_state_publisher;

  /// \brief Whether or not the Kit ROS topic is enabled
  /// If unpermitted subscribers connect during the competition, publishing is disabled
protected:
  bool is_publishing_enabled;

  /// \brief Service that locks models to the briefcase
public:
  ros::ServiceServer lock_models_server;

  /// \brief ROS service that clears the briefcase
public:
  ros::ServiceServer clear_briefcase_server;

  /// \brief ROS service to get the contents of the briefcase
public:
  ros::ServiceServer briefcase_contents_server;

  /// \brief ROS service to get parts connected to the briefcase
public:
  ros::ServiceServer briefcase_connected_parts_server;

  /// \brief Broadcaster for the tf frame of the briefcase
public:
  tf2_ros::TransformBroadcaster tf_broadcaster;

  /// \brief Name of the tf transform
public:
  std::string tf_frame_name;

  /// \brief cache briefcae pose at start to work around bug
public:
  ignition::math::Pose3d briefcase_pose;

  /// \brief Parts to ignore (will be published as faulty in briefcase msgs)
  /// The namespace of the part (e.g. bin7) is ignored.
  /// e.g. if model_name1 is faulty, either bin7|model_name1 or bin6|model_name1 will be considered faulty
protected:
  std::vector<std::string> faulty_part_names;

  /// \brief Gazebo subscriber to the lock models topic
protected:
  transport::SubscriberPtr lock_models_sub;

protected:
  ros::Subscriber briefcaseContentSubscriber;

protected:
  ros::Subscriber sensor_part_subscriber;
  ros::Subscriber regulator_part_subscriber;
  ros::Subscriber battery_part_subscriber;
  ros::Subscriber pump_part_subscriber;
  
  ariac::BriefcaseProduct sensor_product;
  ariac::BriefcaseProduct regulator_product;
  ariac::BriefcaseProduct battery_product;
  ariac::BriefcaseProduct pump_product;
};
}  // namespace gazebo
#endif
