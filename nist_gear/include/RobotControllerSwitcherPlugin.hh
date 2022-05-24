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
#ifndef ARIAC_ROBOTCONTROLLERSWITCHERPLUGIN_HH_
#define ARIAC_ROBOTCONTROLLERSWITCHERPLUGIN_HH_
#include <memory>
//gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <nist_gear/RobotHealth.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
//ros
#include <sdf/sdf.hh>

namespace gazebo
{
  class RobotControllerSwitcherPluginPrivate;
  class GAZEBO_VISIBLE RobotControllerSwitcherPlugin : public WorldPlugin
{
  /// \brief Destructor
public:
  RobotControllerSwitcherPlugin();
  virtual ~RobotControllerSwitcherPlugin();
  void OnRobotHealthContent(nist_gear::RobotHealth msg);
  void GetStaticControllers(std::string robot_name);
  void StartRobot(std::string robot_name);
  void StopKittingRobot();
  void StopGantryRobot();

  /// \brief Load the plugin.
public:
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief Initialize the plugin.
private:
  void Init();

  /// \brief Update the plugin.
private:
  void OnUpdate();

private:
    std::unique_ptr<RobotControllerSwitcherPluginPrivate> data_ptr;
  };
}  // namespace gazebo
#endif
