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
#ifndef ROS_AGV_PLUGIN_HH_
#define ROS_AGV_PLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include "nist_gear/AgvGoFromASToAS.h" //custom service
#include "nist_gear/AgvGoFromASToKS.h"  //custom service
#include "nist_gear/AgvGoFromKSToAS.h"  //custom service
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace gazebo {
  // Forward declare private data class
  class ROSAGVPluginPrivate;

  /**
   * @brief ROS implementation of the ROSAGVPlugin plugin
   *
   */
  class ROSAGVPlugin : public ModelPlugin {
    public:
    /**
     * @brief Construct a new ROSAGVPlugin object
     *
     */
    ROSAGVPlugin();
    /**
     * @brief Destroy the ROSAGVPlugin object
     *
     */
    virtual ~ROSAGVPlugin();
    /**
     * @brief Load the plugin
     *
     * @param _parent Pointer to the parent model
     * @param _sdf Pointer to the SDF element of the plugin
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    /**
     * @brief Provides the service for controlling a AGV
     *
     * @param _req
     * @param _res
     * @return true
     * @return false
     */
    bool OnCommand(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res);
    /**
     * @brief Provides the service for tasking an AGV to go to an assembly station
     *
     */
     // public: bool OnCommandToStation(std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_res);

    void OnAGVLocation(std_msgs::String::ConstPtr msg);

    void CreateAgv1FromAs1ToKs();


    bool ProcessKsToAs1ServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool ProcessKsToAs2ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool ProcessKsToAs3ServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool ProcessKsToAs4ServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);

    bool ProcessAs1ToAs2ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool ProcessAs2ToAs1ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool ProcessAs3ToAs4ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool ProcessAs4ToAs3ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    bool ProcessAs1ToKsServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool ProcessAs2ToKsServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool ProcessAs3ToKsServiceCallback(std_srvs::Trigger::Request& req,std_srvs::Trigger::Response& res);
    bool ProcessAs4ToKsServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);



    /// \brief Called when world update events are received
    /// \param[in] _info Update information provided by the server.
    protected:
    virtual void OnUpdate(const common::UpdateInfo& _info);

    /// \brief Private data pointer.
    private:
    std::unique_ptr<ROSAGVPluginPrivate> dataPtr;
  };
} // namespace gazebo
#endif
