// Copyright 2018 Open Source Robotics Foundation, Inc.
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
