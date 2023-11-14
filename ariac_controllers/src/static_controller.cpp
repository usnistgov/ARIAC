/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#include <ariac_controllers/static_controller.hpp>

#include <exception>

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