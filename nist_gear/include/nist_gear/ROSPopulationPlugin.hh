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

#ifndef _ROS_POPULATION_PLUGIN_HH_
#define _ROS_POPULATION_PLUGIN_HH_

#include <memory>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>
#include "nist_gear/PopulationControl.h"
#include "nist_gear/PopulationPlugin.hh"

namespace gazebo
{
  /// \brief Forward declaration of the private data class.
  class ROSPopulationPluginPrivate;

  /// \brief ROS interface for the Population plugin.
  class ROSPopulationPlugin : public PopulationPlugin
  {
    /// \brief Constructor.
    public: ROSPopulationPlugin();

    /// \brief Destructor.
    public: virtual ~ROSPopulationPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Documentation inherited.
    public: virtual void Reset();

    /// \brief Receives messages for controlling the object population plugin
    /// on the object population's topic.
    /// \param[in] _req The message to control the object population plugin.
    /// \param[out] _rep If the service succeed or not.
    public: bool OnPopulationControl(
      nist_gear::PopulationControl::Request &_req,
      nist_gear::PopulationControl::Response &_res);

    // Documentation inherited.
    private: virtual void Publish() const;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ROSPopulationPluginPrivate> dataPtr;
  };
}
#endif
