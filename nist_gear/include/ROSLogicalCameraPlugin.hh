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

#ifndef _ROS_LOGICAL_CAMERA_PLUGIN_HH_
#define _ROS_LOGICAL_CAMERA_PLUGIN_HH_

#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/msgs/logical_camera_image.pb.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math/Pose3.hh>

// ROS
#include "nist_gear/LogicalCameraImage.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  /// \brief ROS publisher for the logical camera
  class ROSLogicalCameraPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ROSLogicalCameraPlugin();

    /// \brief Destructor
    public: virtual ~ROSLogicalCameraPlugin();

    /// \brief Model that contains the logical camera
    protected: physics::ModelPtr model;

    /// \brief Gazebo world pointer.
    protected: physics::WorldPtr world;

    /// \brief Link that holds the logical camera
    protected: physics::LinkPtr cameraLink;

    /// \brief The logical camera sensor
    protected: sensors::SensorPtr sensor;

    /// \brief The model name
    protected: std::string name;

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Searches the model links for a logical camera sensor
    protected: void FindLogicalCamera();

    /// \brief Callback for when logical camera images are received
    /// \param[in] _msg The logical camera image
    public: void OnImage(ConstLogicalCameraImagePtr &_msg);

    /// \brief Determine if the model is one that should be published
    protected: bool ModelToPublish(const std::string & modelName, const std::string & modelType);

    /// \brief Add noise to a model pose
    protected: void AddNoise(ignition::math::Pose3d & pose);

    /// \brief Add model info to the message to be published
    protected: void AddModelToMsg(
      const std::string & modelType, const ignition::math::Pose3d & modelPose,
      nist_gear::LogicalCameraImage & imageMsg);

    /// \brief Publish the TF frame of a model
    protected: void PublishTF(
      const ignition::math::Pose3d & pose, const std::string & parentFrame, const std::string & frame);

    /// \brief Called when an activation/deactivation message received
    public: void OnActivationMsg(ConstGzStringPtr &_msg);

    /// \brief Subscriber to the activation topic.
    protected: transport::SubscriberPtr activationSub;

    /// \brief If true, publish to the ROS topic
    protected: bool publishing = true;

    /// \brief Node for communication with gazebo
    protected: transport::NodePtr node;

    /// \brief Subscription to logical camera image messages from gazebo
    protected: transport::SubscriberPtr imageSub;

    /// \brief for setting ROS name space
    protected: std::string robotNamespace;

    /// \brief ros node handle
    protected: ros::NodeHandle *rosnode;

    /// \brief ROS publisher for the logical camera image
    protected: ros::Publisher imagePub;

    /// \brief Prefix for the model TF frames published
    protected: std::string modelFramePrefix;

    /// \brief If true, only publish the models if their type is known; otherwise publish all
    protected: bool onlyPublishKnownModels;

    /// \brief Whitelist of the known model types to detect
    protected: std::vector<std::string> knownModelTypes;

    /// \brief Whitelist of known models by name (independent of the namespace (e.g. bin7)).
    /// e.g. if model_name1 is whitelisted, both bin7|model_name1 and bin6|model_name1 will be published
    protected: std::vector<std::string> knownModelNames;

    /// \brief If true, detected model type will be anonymized
    protected: bool anonymizeModels;

    /// \brief Map of noise IDs to noise models
    protected: std::map<std::string, sensors::NoisePtr> noiseModels;

    /// \brief Pose of kit trays w.r.t. their parent AGV
    protected: ignition::math::Pose3d kitTrayToAgv;

    /// \brief TF broadcaster for model frames
    protected: boost::shared_ptr<tf::TransformBroadcaster> transformBroadcaster;
  };
}
#endif
