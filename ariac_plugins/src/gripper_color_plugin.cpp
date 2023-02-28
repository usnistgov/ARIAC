/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/
#include <ariac_plugins/gripper_color_plugin.hpp>

#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GripperColorPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::rendering::VisualPtr visual_;

  std::map<std::string, ignition::math::Color> colors_;

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr gripper_state_sub_;

  void OnUpdate();

  void GripperStateCallback(const ariac_msgs::msg::VacuumGripperState::SharedPtr msg_);

};

GripperColorPlugin::GripperColorPlugin()
: impl_(std::make_unique<GripperColorPluginPrivate>())
{
}

GripperColorPlugin::~GripperColorPlugin()
{
}

void GripperColorPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  std::string name = sdf->GetElement("robot_name")->Get<std::string>();

  impl_->visual_ = visual;

  ignition::math::Color blue;
  blue.R(0.0); blue.G(0.18); blue.B(0.7); blue.A(1.0);
  impl_->colors_.insert({"part_gripper", blue});

  ignition::math::Color purple;
  purple.R(0.4); purple.G(0.0); purple.B(0.4); purple.A(1.0);
  impl_->colors_.insert({"tray_gripper", purple});

  impl_->gripper_state_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/" + name + "_gripper_state", 10, 
    std::bind(&GripperColorPluginPrivate::GripperStateCallback, impl_.get(), std::placeholders::_1));

  // impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
  //   std::bind(&GripperColorPluginPrivate::OnUpdate, impl_.get()));
}

void GripperColorPluginPrivate::GripperStateCallback(
  const ariac_msgs::msg::VacuumGripperState::SharedPtr msg_) 
{
  ignition::math::Color color = colors_[msg_->type];
  
  visual_->SetDiffuse(color);
  visual_->SetAmbient(color);
  visual_->SetTransparency(0);
}

void GripperColorPluginPrivate::OnUpdate()
{
}


// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GripperColorPlugin)
}  // namespace ariac_plugins
