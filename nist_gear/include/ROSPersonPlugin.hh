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
#ifndef ROS_PERSON_PLUGIN_HH_
#define ROS_PERSON_PLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>


namespace gazebo
{
  // Forward declare private data class
  class ROSPersonPluginPrivate;

  /// \brief ROS implementation of the ROSPersonPlugin plugin
  class ROSPersonPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ROSPersonPlugin();

    /// \brief Destructor
    public: virtual ~ROSPersonPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called when world update events are received
    /// \param[in] _info Update information provided by the server.
    protected: virtual void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Private data pointer.
    private: std::unique_ptr<ROSPersonPluginPrivate> dataPtr;
  };
}
#endif
