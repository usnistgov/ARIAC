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
 * Desc: Tray located on the gantry
 * Author: Zeid Kootbally
 */
#ifndef _GAZEBO_GANTRY_TRAY_PLUGIN_HH_
#define _GAZEBO_GANTRY_TRAY_PLUGIN_HH_

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
#include "SideContactPlugin.hh"

namespace gazebo
{
    /// \brief A plugin for a contact sensor on a kit tray.
    class GAZEBO_VISIBLE GantryTrayPlugin : public SideContactPlugin
    {
        /// \brief Constructor.
    public:
        GantryTrayPlugin();

        /// \brief Destructor.
    public:
        virtual ~GantryTrayPlugin();

        /// \brief Load the model plugin.
        /// \param[in] _model Pointer to the model that loaded this plugin.
        /// \param[in] _sdf SDF element that describes the plugin.
    public:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback that receives the world update event
    protected:
        void OnUpdate(const common::UpdateInfo &_info);

        /// \brief Update the kit based on which models are in contact
    protected:
        void ProcessContactingModels();

        /// \brief Create a fixed joint to all contacting models
    protected:
        virtual void LockContactingModels();

        /// \brief Remove any fixed joints to contacting models
    protected:
        virtual void UnlockContactingModels();

        /// \brief Update the kit based on which models are in contact
    public:
        std::string DetermineModelType(const std::string &modelName);

        /// \brief Callback for when a new subscriber connects to the Kit ROS publisher
        /// This will check that only the /gazebo node is subscribed during the competition
    protected:
        void OnSubscriberConnect(const ros::SingleSubscriberPublisher &pub);

        /// \brief Publish the Kit ROS message
        // protected: void PublishKitMsg();

        /// \brief Service for locking the models to the tray and disabling updates
    protected:
        bool HandleLockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response> &event);

    protected:
        bool HandleUnlockModelsRequest(ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response> &event);

   protected: ros::Publisher currentTrayPub;
        /// \brief Service for clearing the tray
    protected:
        bool HandleClearService(
            ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response> &event);

    protected:
        void PublishTFTransform(const common::Time sim_time);

        /// \brief ID of the station
        // protected: std::string station_name;

        /// \brief Fixed joints to lock contacting models
    protected:
        std::vector<physics::JointPtr> fixedJoints;

        /// \brief ROS node handle
    protected:
        ros::NodeHandle *rosNode;

        /// \brief Gazebo node for communication
    protected:
        transport::NodePtr gzNode;

        /// \brief Publisher for the kit state
        // protected: ros::Publisher currentKitPub;

        /// \brief ROS service that clears the tray
    public:
        ros::ServiceServer clearTrayServer;

        /// \brief ROS service that locks parts in the gantry tray
    public:
        ros::ServiceServer lockModelsServer;

        /// \brief ROS service that unlocks parts in the gantry tray
    public:
        ros::ServiceServer unlockModelsServer;

        /// \brief ROS service to get the contents of the tray
    public:
        ros::ServiceServer trayContentsServer;

        /// \brief Broadcaster for the tf frame of the tray
    public:
        tf2_ros::TransformBroadcaster tf_broadcaster;

        /// \brief Name of the tf transform
    public:
        std::string tf_frame_name;

        /// \brief cache tray pose at start to work around bug where tray pose drops during AGV animation
    public:
        ignition::math::Pose3d tray_pose;

        /// \brief Parts to ignore (will be published as faulty in tray msgs)
        /// The namespace of the part (e.g. bin7) is ignored.
        /// e.g. if model_name1 is faulty, either bin7|model_name1 or bin6|model_name1 will be considered faulty
    protected:
        std::vector<std::string> faultyPartNames;

        /// \brief Gazebo subscriber to the lock models topic
    // protected:
    //     transport::SubscriberPtr lockModelsSub;

    // protected:
    //     transport::SubscriberPtr unlockModelsSub;
    };
}
#endif
