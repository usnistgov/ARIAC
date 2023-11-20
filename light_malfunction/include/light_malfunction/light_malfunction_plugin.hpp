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

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{
    class Block {
        public:
            Block(double duration, double interval, ignition::math::Color color)
                : duration(duration), color(color), interval(interval) {}
        
        /// \brief The duration time to flash (in seconds).
            double duration;

        /// \brief The interval time between flashing (in seconds). When it is zero, the light is constant.
            double interval;

        /// \brief The color of light.
            ignition::math::Color color;
    };

    // forward declaration
    // class LightSettingPrivate;
    class LightSettingPrivate {
    
    /// Constructor
    public:
        LightSettingPrivate(int num_intensities);

        physics::LinkPtr FindLinkForLight(const physics::ModelPtr &_model, const std::string &_lightName,
                                          const std::string &_linkName);

    /// \brief The name of flash light.
        std::string name;

    /// \brief Link which holds this flash light.
        physics::LinkPtr link;

    /// \brief The time at which the current phase started.
        common::Time startTime;

    /// \brief The current switch state (the light itself is active or not).
        // bool switchOn;

    /// \brief The current flasshing state (flash or dim).
        bool flashing;

    /// \brief The length of the ray (in meters).
        double range;

    /// \brief The pointer to publisher to send a command to a light.
        transport::PublisherPtr pubLight;

    /// \brief A message holding a flashlight command.
        msgs::Light msg;

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
                              const physics::ModelPtr &_model, const common::Time &_currentTime);
            virtual ~LightSetting();
            virtual void InitPubLight(const transport::PublisherPtr &_pubLight) final;
            virtual void UpdateLightInEnv(const common::Time &_currentTime) final;
            // virtual const std::string Name() const final;
            // virtual const physics::LinkPtr Link() const final;
            // virtual void SwitchOn() final;
            // virtual void SwitchOff() final;
            // virtual unsigned int BlockCount() final;

        protected:
            virtual void Flash();
            virtual void Dim();
            // virtual ignition::math::Color CurrentColor() final;

    
        private:
            std::unique_ptr<LightSettingPrivate> dataPtr;
    };


    class LightPluginPrivate {
    public:
        // std::shared_ptr<LightSetting> SettingByLightNameAndLinkName( const std::string &_lightName,
        //                                                              const std::string &_linkName) const {
        //     for (auto &setting: this->listLight) {
        //         if (setting->Name() == _lightName) {
        //             if (_linkName.length() == 0 || setting->Link()->GetName() == _linkName) {
        //                 return setting;
        //             }
        //         }
        //     }
        //     return nullptr;
        // }

    /// \brief pointer to the model.
        physics::ModelPtr model;

    /// \brief pointer to the world.
        physics::WorldPtr world;

    /// \brief The pointer to node for communication.
        transport::NodePtr node;

    /// \brief The pointer to publisher to send a command to the light.
        transport::PublisherPtr pubLight;

    /// \brief The list of light settings to control.
        std::vector< std::shared_ptr<LightSetting> > listLight;

        /// \brief pointer to the update even connection.
        event::ConnectionPtr updateConnection;
        
        // Brightness of the lights when there is no challenge being implemented.
        double nominal_brightness;

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
        double light_range;
    };


    class GZ_PLUGIN_VISIBLE LightPlugin : public ModelPlugin {
        public:
            LightPlugin();
            virtual ~LightPlugin();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

        protected:
            virtual void OnUpdate();
            // virtual bool TurnOn(const std::string &_lightName, const std::string &_linkName) final;
            // virtual bool TurnOnAll() final;
            // virtual bool TurnOff(const std::string &_lightName, const std::string &_linkName) final;
            // virtual bool TurnOffAll() final;
            // virtual std::shared_ptr<LightSetting> CreateSetting(const sdf::ElementPtr &_sdf,
            //     const physics::ModelPtr &_model,
            //     const common::Time &_currentTime);
            // virtual void InitSettingBySpecificData(std::shared_ptr<LightSetting> &_setting);

        private:
            std::unique_ptr<LightPluginPrivate> dataPtr;
    };

}

#endif
