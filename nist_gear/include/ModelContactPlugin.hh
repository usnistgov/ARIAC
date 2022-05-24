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
 * Desc: Plugin for monitoring contacts with a given model
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_MODEL_CONTACT_PLUGIN_HH_
#define _GAZEBO_MODEL_CONTACT_PLUGIN_HH_

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
  class GAZEBO_VISIBLE ModelContactPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ModelContactPlugin();

    /// \brief Destructor.
    public: virtual ~ModelContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's messages.
    protected: void OnContactsReceived(ConstContactsPtr& _msg);

    /// \brief Called when world update events are received
    /// \param[in] _info Update information provided by the server.
    protected: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Pointer to the update event connection.
    protected: event::ConnectionPtr updateConnection;

    /// \brief The normal, in local frame, to the side that is to have contacts monitored
    /// (default (0, 0, 1))
    protected: ignition::math::Vector3d sideNormal;

    /// \brief Pointer to the world
    protected: physics::WorldPtr world;

    /// \brief Pointer to the model
    protected: physics::ModelPtr model;

    /// \brief Pointer to this node for publishing/subscribing
    protected: transport::NodePtr node;

    /// \brief collision names
    protected: std::map<std::string, physics::CollisionPtr> collisions;

    /// \brief Subscriber for the contact topic
    protected: transport::SubscriberPtr contactSub;

    /// \brief Contacts msg received
    protected: msgs::Contacts newestContactsMsg;

    /// \brief Mutex for protecting contacts msg
    protected: mutable boost::mutex mutex;

    /// \brief Flag for new contacts message
    protected: bool newMsg;

    /// \brief Set of pointers to links that have collisions with the parent link's side
    protected: std::set<physics::LinkPtr> contactingLinks;

    /// \brief Set of pointers to models that have collisions with the parent link's side
    protected: std::set<physics::ModelPtr> contactingModels;

    /// \brief Determine which links are in contact with the side of the parent link
    protected: virtual void CalculateContactingLinks();

    /// \brief Determine which models are in contact with the side of the parent link
    protected: virtual void CalculateContactingModels();

    /// \brief Plugin update rate (Hz). A negative value means that the custom
    /// update rate is disabled. The plugin will execute at the physics rate.
    protected: double updateRate = -1;

  };
}
#endif
