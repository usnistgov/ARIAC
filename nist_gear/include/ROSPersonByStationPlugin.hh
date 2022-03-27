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
#ifndef _ROS_PERSON_BY_STATION_PLUGIN_HH_
#define _ROS_PERSON_BY_STATION_PLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>


namespace gazebo {
    // Forward declare private data class
    class ROSPersonByStationPluginPrivate;

    /**
     * @brief ROS implementation of the ROSPersonByStationPlugin plugin
     *
     * This plugin is used to move human models in Gazebo
     *
     */
    class ROSPersonByStationPlugin : public ModelPlugin
    {
    public:

        /**
         * @brief Construct a new ROSPersonByStationPlugin object
         *
         */
        ROSPersonByStationPlugin();

        /**
         * @brief Destroy the ROSPersonByStationPlugin object
         *
         */
        virtual ~ROSPersonByStationPlugin();

        /**
         * @brief Load the plugin
         *
         * @param _parent Pointer to the parent model
         * @param _sdf Pointer to the SDF element of the plugin
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
        /**
         * @brief Called when world update events are received
         *
         * @param _info Update information provided by the server
         */
        virtual void OnUpdate(const common::UpdateInfo& _info);

    private:
        /**
         * @brief Private data pointer
         *
         */
        std::unique_ptr<ROSPersonByStationPluginPrivate> dataPtr;

        void AS2Callback(std_msgs::Bool::ConstPtr msg);
        void AS4Callback(std_msgs::Bool::ConstPtr msg);
    };
}
#endif
