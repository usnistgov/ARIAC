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

#ifndef LIGHTINGPLUGIN_HPP_
#define LIGHTINGPLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Color.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/srv/light_malfunction_challenge.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace ariac_plugins
{
    class Block {
        public:
            Block(float duration, float interval, ignition::math::Color color)
                : duration(duration), color(color), interval(interval) {}
        
        /// \brief The duration time to flash (in seconds).
            float duration;

        /// \brief The interval time between flashing (in seconds). When it is zero, the light is constant.
            float interval;

        /// \brief The color of light.
            ignition::math::Color color;
    };

    // forward declaration
    // class LightSettingPrivate;
    class LightSettingPrivate {
    
    /// Constructor
    public:
        LightSettingPrivate(bool is_chlng_lyt, int num_intensities);

        gazebo::physics::LinkPtr FindLinkForLight(const gazebo::physics::ModelPtr &_model, const std::string &_lightName,
                                          const std::string &_linkName);

    /// \brief The name of flash light.
        std::string name;

    /// \brief Link which holds this flash light.
        gazebo::physics::LinkPtr link;

    /// \brief The time at which the current phase started.
        gazebo::common::Time startTime;

    /// \brief The current switch state (the light itself is active or not).
        // bool switchOn;

    /// \brief The current flasshing state (flash or dim).
        bool flashing;

    /// \brief The length of the ray (in meters).
        float range;

    /// \brief The pointer to publisher to send a command to a light.
        gazebo::transport::PublisherPtr pubLight;

    /// \brief A message holding a flashlight command.
        gazebo::msgs::Light msg;

    /// \brief True if <light> element exists.
        // bool lightExists;

    /// \brief The list of blocks of light having the Range of intensity values the light has to shine.
        std::vector<std::shared_ptr<Block>> blocks;

    /// \brief the index of the current block.
        int currentBlockIndex;
    };


    // forward declaration
    class LightPluginPrivate;


    class GZ_PLUGIN_VISIBLE LightSetting {
        public:
            LightSetting(std::string &lightId, bool is_chlng, const LightPluginPrivate *lpp, 
                              const gazebo::physics::ModelPtr &_model, const gazebo::common::Time &_currentTime);
            virtual ~LightSetting();
            virtual void InitPubLight(const gazebo::transport::PublisherPtr &_pubLight) final;
            virtual void UpdateLightInEnv(const gazebo::common::Time &_currentTime, bool challenge_is_active, float nominal_brightness) final;

        protected:
            virtual void Flash(float brightness = -1);
            virtual void Dim();

    
        private:
            std::unique_ptr<LightSettingPrivate> dataPtr;
    };


    class LightPluginPrivate {
    public:
        LightPluginPrivate() : enabled_(false) {}

        void EnableChallenge(
            ariac_msgs::srv::LightMalfunctionChallenge::Request::SharedPtr req, 
            ariac_msgs::srv::LightMalfunctionChallenge::Response::SharedPtr res
        );

    /// \brief pointer to the model.
        gazebo::physics::ModelPtr model;

    /// \brief pointer to the world.
        gazebo::physics::WorldPtr world;

    /// \brief The pointer to node for communication.
        gazebo::transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the light.
        gazebo::transport::PublisherPtr pubLight;

    /// \brief The list of light settings to control.
        std::vector< std::shared_ptr<LightSetting> > listLight;

        /// \brief pointer to the update even connection.
        gazebo::event::ConnectionPtr updateConnection;

        /// Node for ROS communication.
        gazebo_ros::Node::SharedPtr ros_node_;
        rclcpp::Service<ariac_msgs::srv::LightMalfunctionChallenge>::SharedPtr LMC_sevice;

        // Whether challenge is active
        bool enabled_;

          // Brightness of the lights when there is no challenge being implemented.
        float nominal_brightness;

        // Challenge scenario: dip, flicker, fluctuate
        std::string scenario;

        // duration (in seconds) the lighting challenge has to be run for
        float challenge_duration;
        
        // All Lights
        std::vector<std::string> all_light_ids;
        
        // Lights to be controlled
        std::vector<std::string> chlg_light_ids;
        
        // Range of intensity values the light has to go through
        std::vector<float> intensities;

        // Duration of each intensity value in the range
        float intensity_duration;
        
        // range (in meters) of each light, i.e length of the light rays originating from a light source
        float light_range;
    };


    class GZ_PLUGIN_VISIBLE LightPlugin : public gazebo::ModelPlugin {
        public:
            LightPlugin();
            virtual ~LightPlugin();
            void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

        protected:
            virtual void OnUpdate();
            
        private:
            std::unique_ptr<LightPluginPrivate> dataPtr;
    };

}

#endif
