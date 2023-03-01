/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/human_teleport_plugin.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class HumanTeleportPluginPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::ModelPtr model_;  
  gazebo::physics::WorldPtr world_;

  ignition::math::Pose3d home_pose;

  // Teleport service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr teleport_service_;

  void TeleportHuman(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

};

HumanTeleportPlugin::HumanTeleportPlugin()
: impl_(std::make_unique<HumanTeleportPluginPrivate>())
{
}

HumanTeleportPlugin::~HumanTeleportPlugin()
{
}

void HumanTeleportPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->model_ = model;

  impl_->home_pose = model->WorldPose();

  impl_->teleport_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
    "/ariac_human/teleport",
    std::bind(
      &HumanTeleportPluginPrivate::TeleportHuman, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

void HumanTeleportPluginPrivate::TeleportHuman(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res) 
{
  res->success = false;
  
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Teleporting human to orign");

  model_->SetWorldPose(home_pose);

  res->success = true;
  return;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HumanTeleportPlugin)
}  // namespace ariac_plugins
