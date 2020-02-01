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

#ifndef _GAZEBO_LIGHT_CURTAIN_PLUGIN_HH_
#define _GAZEBO_LIGHT_CURTAIN_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief A Ray Sensor Plugin which makes it act as a proximity sensor
  class GAZEBO_VISIBLE ProximityRayPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: ProximityRayPlugin();

    /// \brief Destructor
    public: virtual ~ProximityRayPlugin();

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief Process the scan data and update state
    /// \returns true if the state has changed since processing the last scan
    public: virtual bool ProcessScan();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Generate a scoped topic name from a local one
    /// \param local local topic name
    protected: std::string Topic(std::string topicName) const;

    /// \brief Publisher for the sensor state
    protected: transport::PublisherPtr statePub;

    /// \brief Publisher for the sensor state change
    protected: transport::PublisherPtr stateChangePub;

    /// \brief State message
    protected: msgs::Header stateMsg;

    /// \brief Mutex to protect interruptionMsg
    protected: std::mutex mutex;

    /// \brief Topic name for state message
    protected: std::string stateTopic;

    /// \brief Topic name for state change message
    protected: std::string stateChangeTopic;

    /// \brief Sensor detection state
    protected: bool objectDetected;

    /// \brief Convert sensor ranges to parent link frame?
    protected: bool useLinkFrame;

    /// \brief Minimum sensing range in meters
    protected: double sensingRangeMin;

    /// \brief Maximum sensing range in meters
    protected: double sensingRangeMax;

    /// \brief Whether or not the output function is normally open (default) or normally closed
    protected: bool normallyOpen;

    /// \brief Pointer to parent link
    protected: physics::LinkPtr link;

    /// \brief Pointer to world
    protected: physics::WorldPtr world;

    /// \brief Pointer to this node for publishing
    protected: transport::NodePtr node;

    /// \brief The parent sensor
    protected: sensors::RaySensorPtr parentSensor;

    /// \brief The connection tied to ProximityRayPlugin::OnNewLaserScans()
    protected: event::ConnectionPtr newLaserScansConnection;

  };
}
#endif
