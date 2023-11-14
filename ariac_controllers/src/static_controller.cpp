// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ariac_controllers/static_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace ariac_controllers {

controller_interface::InterfaceConfiguration
StaticController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::string name : joint_names_) {
    config.names.push_back(name + "/position");
  }

  return config;
}

controller_interface::InterfaceConfiguration
StaticController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::string name : joint_names_) {
    config.names.push_back(name + "/position");
  }

  return config;
}

controller_interface::return_type StaticController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& /*period*/) 
{

  for (int i = 0; i < int(command_interfaces_.size()); ++i) {
    command_interfaces_[i].set_value(joint_states_[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn StaticController::on_init() {
  try
  {
    auto_declare<std::vector<std::string>>("joints", {});
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn StaticController::on_configure(
  const rclcpp_lifecycle::State&) 
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "joints parameter not set");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn StaticController::on_activate(
    const rclcpp_lifecycle::State&)
{
  joint_states_.clear();

  for (int i = 0; i < int(state_interfaces_.size()); ++i) {
    const auto& position_interface = state_interfaces_.at(i);

    joint_states_.push_back(position_interface.get_value());
  }
  
  return CallbackReturn::SUCCESS;
}

}  // namespace ariac_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(ariac_controllers::StaticController,
                       controller_interface::ControllerInterface)