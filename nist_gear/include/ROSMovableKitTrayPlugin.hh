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
#ifndef _GAZEBO_MOVABLE_KIT_TRAY_PLUGIN_HH_
#define _GAZEBO_MOVABLE_KIT_TRAY_PLUGIN_HH_

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
#include <nist_gear/DetectedKittingShipment.h>
#include <nist_gear/DetectKittingShipment.h>
#include <nist_gear/LockUnlockMovableTrayOnAgv.h>
#include <nist_gear/DetectKitTrayContent.h>
#include "SideContactPlugin.hh"
#include <std_msgs/String.h>

namespace gazebo
{
/// \brief A plugin for a contact sensor on a movable kit tray.
class GAZEBO_VISIBLE MovableKitTrayPlugin : public SideContactPlugin
{
  /// \brief Constructor.
public:
  MovableKitTrayPlugin();

  /// \brief Destructor.
public:
  virtual ~MovableKitTrayPlugin();

  /**
   * @brief Load the model plugin.
   * @param _model Pointer to the model that loaded this plugin.
   * @param _sdf SDF element that describes the plugin
   */
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Update the kit based on which models are in contact
public:
  std::string DetermineModelType(const std::string& modelName);

  /// \brief Service that locks models to the tray
  ros::ServiceServer lockModelsServer;

  /// \brief ROS service that clears the tray
  ros::ServiceServer clearTrayServer;

  /// \brief ROS service to get the contents of the tray
  ros::ServiceServer movable_tray_content_srv;
  /// \brief ROS service to manually lock a movable tray on an AGV
  ros::ServiceServer lockMovableTrayOnAGVServer;

  ros::ServiceServer kit_tray_contents_server;
  //   ros::ServiceServer kit_tray_2_contents_server;
  //   ros::ServiceServer kit_tray_3_contents_server;
  //   ros::ServiceServer kit_tray_4_contents_server;

  /// \brief Broadcaster for the tf frame of the tray
  tf2_ros::TransformBroadcaster tf_broadcaster;

  /// \brief Name of the tf transform
  std::string tf_frame_name;

  /// \brief cache tray pose at start to work around bug where tray pose drops during AGV animation
  ignition::math::Pose3d tray_pose;

  std::string kit_tray_lock_unlock_request;

  /// \brief Callback that receives the world update event
  // void OnUpdate(const common::UpdateInfo& _info);
protected:
  void OnUpdate(const common::UpdateInfo& _info);
  /// \brief Update the kit based on which models are in contact
protected:
  void ProcessContactingModels();
  /// \brief Create a fixed joint to all contacting models
protected:
  virtual void LockContactingModels();
  /// \brief Remove any fixed joints to contacting models
protected:
  virtual void UnlockContactingModels();

  /// \brief Callback for when a new subscriber connects to the Kit ROS publisher
  /// This will check that only the /gazebo node is subscribed during the competition
  void OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub);

  /// \brief Publish the Kit ROS message
  void PublishKitMsg();

  /// \brief Service for locking the models to the tray and disabling updates
  void HandleLockModelsRequestOnAGV1(ConstGzStringPtr& _msg);
  void HandleLockModelsRequestOnAGV2(ConstGzStringPtr& _msg);
  void HandleLockModelsRequestOnAGV3(ConstGzStringPtr& _msg);
  void HandleLockModelsRequestOnAGV4(ConstGzStringPtr& _msg);

  // /// \brief Service for clearing the tray
  // bool HandleClearService(
  //     ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event);

  /// \brief Service for getting the content of a tray
  bool HandleMovableTrayContent(
      ros::ServiceEvent<nist_gear::DetectKittingShipment::Request, nist_gear::DetectKittingShipment::Response>& event);

  bool HandleGetKitTrayContent(
      ros::ServiceEvent<nist_gear::DetectKitTrayContent::Request, nist_gear::DetectKitTrayContent::Response>& event);

      /// \brief Callback function for when the service to lock/unlock a movable tray on an AGV is called
      // bool  HandleLockUnlockMovableTrayOnAgvService(
      //     nist_gear::LockUnlockMovableTrayOnAgv::Request& req,
      //     nist_gear::LockUnlockMovableTrayOnAgv::Response& res);

      void PublishTFTransform(const common::Time sim_time);

  /// \brief Kit which is currently on the tray
  ariac::Kit currentKit;

  /// \brief ID of tray
  std::string tray_model;

  /// \brief ID of the station
  std::string station_name;

  /// \brief Fixed joints to lock contacting models
  std::vector<physics::JointPtr> fixedJoints;

  std::vector<physics::JointPtr> trayPartJoints;

  /// \brief ROS node handle
  ros::NodeHandle* rosNode;

  /// \brief Gazebo node for communication
  transport::NodePtr gzNode;

  /// \brief Publisher for the kit state
  ros::Publisher currentKitPub;

  /// \brief Whether or not the Kit ROS topic is enabled
  /// If unpermitted subscribers connect during the competition, publishing is disabled
  bool publishingEnabled;

  /// \brief Parts to ignore (will be published as faulty in tray msgs)
  /// The namespace of the part (e.g. bin7) is ignored.
  /// e.g. if model_name1 is faulty, either bin7|model_name1 or bin6|model_name1 will be considered faulty
  std::vector<std::string> faultyPartNames;

  /// \brief Gazebo subscriber to the lock models topic
  transport::SubscriberPtr lockModelsOnAGV1Sub;
  transport::SubscriberPtr lockModelsOnAGV2Sub;
  transport::SubscriberPtr lockModelsOnAGV3Sub;
  transport::SubscriberPtr lockModelsOnAGV4Sub;

  std::string GetAgv(double movable_tray_y_position);
  std::string GetStation(double movable_tray_x_position, double movable_tray_y_position);
  bool inRange(double agv_pos, double tray_world_pos)
  {
    double epsilon{ 0.2 };
    double low = std::abs(agv_pos) - epsilon;
    double high = std::abs(agv_pos) + epsilon;
    return ((std::abs(tray_world_pos) - low) <= (high - low));
  }

  ros::Subscriber agv1LocationSubscriber;
  ros::Subscriber agv2LocationSubscriber;
  ros::Subscriber agv3LocationSubscriber;
  ros::Subscriber agv4LocationSubscriber;
  std::string agv1CurrentStation;
  std::string agv2CurrentStation;
  std::string agv3CurrentStation;
  std::string agv4CurrentStation;
  void OnAGV1Location(std_msgs::String::ConstPtr msg);
  void OnAGV2Location(std_msgs::String::ConstPtr msg);
  void OnAGV3Location(std_msgs::String::ConstPtr msg);
  void OnAGV4Location(std_msgs::String::ConstPtr msg);
  void LockUnlockServiceCallback(ConstGzStringPtr& msg);

protected:
  transport::SubscriberPtr lock_unlock_kt1_gz_sub;

protected:
  transport::SubscriberPtr lock_unlock_kt2_gz_sub;

protected:
  transport::SubscriberPtr lock_unlock_kt3_gz_sub;

protected:
  transport::SubscriberPtr lock_unlock_kt4_gz_sub;

protected:
  std::vector<physics::JointPtr> fixed_joints;

protected:
  std::string movable_tray_name;

protected:
  ariac::Kit kit;
  /// \brief List of parts that a movable tray should care about.
  /// Anything not in this list will not be reported by the plugin.
public:
  std::vector<std::string> grippable_model_types;
};
}  // namespace gazebo
#endif
