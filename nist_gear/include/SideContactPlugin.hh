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
 * Desc: Plugin for monitoring the side of a ContactSensor
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_SIDE_CONTACT_PLUGIN_HH_
#define _GAZEBO_SIDE_CONTACT_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// \brief A plugin for a model with a contact sensor that only monitors
  /// collisions on one of its sides.
  class GAZEBO_VISIBLE SideContactPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: SideContactPlugin();

    /// \brief Destructor.
    public: virtual ~SideContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's messages.
    protected: virtual void OnContactsReceived(ConstContactsPtr& _msg);

    /// \brief Called when world update events are received
    /// \param[in] _info Update information provided by the server.
    protected: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Name of the contact sensor
    protected: std::string contactSensorName;

    /// \brief Scoped name of the contact sensor
    protected: std::string scopedContactSensorName;

    /// \brief Pointer to the contact sensor
    protected: sensors::ContactSensorPtr parentSensor;

    /// \brief The normal, in local frame, to the side that is to have contacts monitored
    /// (default (0, 0, 1))
    protected: ignition::math::Vector3d sideNormal;

    /// \brief Pointer to the world
    protected: physics::WorldPtr world;

    /// \brief Pointer to the model
    protected: physics::ModelPtr model;

    /// \brief Pointer to this node for publishing/subscribing
    protected: transport::NodePtr node;

    /// \brief Subscriber for the contact topic
    protected: transport::SubscriberPtr contactSub;

    /// \brief Contacts msg received
    protected: msgs::Contacts newestContactsMsg;

    /// \brief Mutex for protecting contacts msg
    protected: mutable boost::mutex mutex;

    /// \brief Flag for new contacts message
    protected: bool newMsg;

    /// \brief Name of the collision of the parent's link
    protected: std::string collisionName;

    /// \brief Pointer to the sensor's parent's link
    protected: physics::LinkPtr parentLink;

    /// \brief Set of pointers to links that have collisions with the parent link's side
    protected: std::set<physics::LinkPtr> contactingLinks;

    /// \brief Set of pointers to models that have collisions with the parent link's side
    protected: std::set<physics::ModelPtr> contactingModels;

    /// \brief Iterate through links of model to find sensor with the specified name
    /// \returns true if the sensor was successfully found and is a contact sensor
    protected: bool FindContactSensor();

    /// \brief Determine which links are in contact with the side of the parent link
    protected: virtual void CalculateContactingLinks();

    /// \brief Determine which models are in contact with the side of the parent link
    protected: virtual void CalculateContactingModels();

    /// \brief Teleport all contacting models to a predetermined location
    protected: virtual void ClearContactingModels();

    /// \brief Determine whether is time to give the plugin an update based on
    /// the plugin's update rate.
    protected: bool TimeToExecute();

    /// \brief Plugin update rate (Hz). A negative value means that the custom
    /// update rate is disabled. The plugin will execute at the physics rate.
    protected: double updateRate = -1;

    /// \brief Last time (sim time) that the plugin was updated.
    protected: gazebo::common::Time lastUpdateTime;

  };
}
#endif
