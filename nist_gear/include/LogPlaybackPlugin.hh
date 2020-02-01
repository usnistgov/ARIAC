/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef ARIAC_LOGPLAYBACKPLUGIN_HH_
#define ARIAC_LOGPLAYBACKPLUGIN_HH_
#include <vector>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class LogPlaybackPlugin : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~LogPlaybackPlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief Update the plugin.
    private: void Update();

    /// \brief On world created callback.
    private: void OnWorldCreated();

    /// \brief The update connection.
    private: gazebo::event::ConnectionPtr updateConn;

    private: gazebo::event::ConnectionPtr worldCreatedConn;

    /// \brief Node for communication.
    private: gazebo::transport::NodePtr node;

    /// \brief Publisher used to control the drone's box visual.
    private: transport::PublisherPtr droneTogglePub;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief The list of shipping box models.
    private: std::vector<gazebo::physics::ModelPtr> boxes;

    /// \brief Map between shipping box names and their state.
    private: std::map<std::string, bool> toggled;
    private: std::map<std::string, bool> droneToggled;

    /// \brief Publishers used to control the shipping boxes' visuals.
    private: std::map<std::string, transport::PublisherPtr> pubs;

    /// \brief The drone model.
    private: gazebo::physics::ModelPtr drone;

    /// \brief True if the drone's box visual is enabled.
    private: bool droneBoxEnabled;
  };
}
#endif
