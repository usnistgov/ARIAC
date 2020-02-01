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
 * Desc: Proximity Ray Plugin
 * Author: Nate Koenig mod by John Hsu, Deanna Hood
 */

#ifndef _GAZEBO_SENSOR_BLACKOUT_PLUGIN_HH_
#define _GAZEBO_SENSOR_BLACKOUT_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief A Ray Sensor Plugin which makes it act as a proximity sensor
  class GAZEBO_VISIBLE SensorBlackoutPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: SensorBlackoutPlugin();

    /// \brief Destructor
    public: virtual ~SensorBlackoutPlugin();

    /// \brief Called when an activation/deactivation message received
    public: void OnActivationMsg(ConstGzStringPtr &_msg);

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to this node for publishing
    protected: transport::NodePtr node;

    /// \brief The parent sensor
    protected: sensors::SensorPtr parentSensor;

    /// \brief Subscriber to the activation topic.
    protected: transport::SubscriberPtr activationSub;

  };
}
#endif
