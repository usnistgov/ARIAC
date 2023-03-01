/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <ariac_plugins/disable_shadows_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class DisableShadowsPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::rendering::VisualPtr visual_;
  gazebo::rendering::ScenePtr scene_;

  bool first_update_;

};

DisableShadowsPlugin::DisableShadowsPlugin()
: impl_(std::make_unique<DisableShadowsPluginPrivate>())
{
}

DisableShadowsPlugin::~DisableShadowsPlugin()
{
}

void DisableShadowsPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  // impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->visual_ = visual;

  impl_->scene_ = impl_->visual_->GetScene();
  impl_->first_update_ = true;

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
    std::bind(&DisableShadowsPlugin::OnUpdate, this));
}

void DisableShadowsPlugin::OnUpdate()
{
  // Do something every simulation iteration
  if (impl_->first_update_) {
    impl_->scene_->SetShadowsEnabled(true);
    impl_->scene_->SetShadowsEnabled(false);
    impl_->first_update_ = false;
  } 
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(DisableShadowsPlugin)
}  // namespace ariac_plugins
