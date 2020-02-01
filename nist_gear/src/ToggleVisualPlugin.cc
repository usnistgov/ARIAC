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

#include <mutex>

#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include "ToggleVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class ToggleVisualPlugin ToggleVisualPlugin.hh
  /// \brief Private data for the ToggleVisualPlugin class.
  class ToggleVisualPluginPrivate
  {
    /// \brief Visual whose visibility will be toggled.
    public: rendering::VisualPtr visual;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief Current visibilty visible.
    public: bool visible = true;

    /// \brief Node used for communication.
    public: transport::NodePtr node;

    /// \brief Mutex protecting visible.
    public: std::mutex mutex;

    /// \brief Subscriber to the toggle topic.
    public: transport::SubscriberPtr toggleSub;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ToggleVisualPlugin)

/////////////////////////////////////////////////
ToggleVisualPlugin::ToggleVisualPlugin() : dataPtr(new ToggleVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
ToggleVisualPlugin::~ToggleVisualPlugin()
{
  this->dataPtr->toggleSub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
}

/////////////////////////////////////////////////
void ToggleVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  if (_sdf->HasElement("initially_visible"))
  {
    this->dataPtr->visible = _sdf->Get<bool>("initially_visible");
  }

  // Get the topic
  if (!_sdf->HasElement("topic"))
  {
    gzerr << "Topic of ToggleVisualPlugin must be specified." << std::endl;
    return;
  }
  std::string toggle_topic = _sdf->Get<std::string>("topic");

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->toggleSub = this->dataPtr->node->Subscribe(
        toggle_topic, &ToggleVisualPlugin::OnToggle, this);

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&ToggleVisualPlugin::Update, this));
}

/////////////////////////////////////////////////
void ToggleVisualPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->visual)
  {
    gzerr << "The visual is null." << std::endl;
    return;
  }

  if (this->dataPtr->visual->GetVisible() != this->dataPtr->visible)
  {
    this->dataPtr->visual->SetVisible(this->dataPtr->visible, true);
    gzdbg << "Toggling visual: " << this->dataPtr->visual->Name() << std::endl;
  }
}

/////////////////////////////////////////////////
void ToggleVisualPlugin::OnToggle(ConstGzStringPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (_msg->data() == "toggle")
  {
    this->dataPtr->visible = !this->dataPtr->visible;
  }
  else if (_msg->data() == "on")
  {
    this->dataPtr->visible = true;
  }
  else if (_msg->data() == "off")
  {
    this->dataPtr->visible = false;
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
