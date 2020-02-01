/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
 * Desc: ROS wrapper for Proximity Ray Plugin
 * Author: Deanna Hood
 * Date: 18 July 2016
 */

#ifndef _ROS_PROXIMITY_RAY_PLUGIN_HH_
#define _ROS_PROXIMITY_RAY_PLUGIN_HH_

// Gazebo
#include "ProximityRayPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

// ROS
#include <ros/ros.h>
#include <nist_gear/Proximity.h>

namespace gazebo
{
  /// \brief ROS interface for the ProximityRayPlugin plugin
  class ROSProximityRayPlugin : public ProximityRayPlugin
  {
    /// \brief Constructor
    public: ROSProximityRayPlugin();

    /// \brief Destructor
    public: virtual ~ROSProximityRayPlugin();

    /// \brief Load the plugin
    /// \param _parent The parent entity must be a Ray sensor
    public: void Load( sensors::SensorPtr _parent, sdf::ElementPtr _sdf );

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief The connection tied to ROSProximityRayPlugin::OnNewLaserScans()
    protected: event::ConnectionPtr newLaserScansConnection;

    /// \brief A pointer to the ROS node. A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode;

    /// \brief ROS publisher for the sensor state
    protected: ros::Publisher statePub;

    /// \brief ROS publisher for the sensor state changes
    protected: ros::Publisher stateChangePub;

    /// \brief ROS message for the sensor state
    protected: nist_gear::Proximity state_msg;

    /// \brief for setting ROS namespace
    protected: std::string robotNamespace;

    /// \brief for setting ROS frame id
    protected: std::string frameId;

  };
}

#endif

