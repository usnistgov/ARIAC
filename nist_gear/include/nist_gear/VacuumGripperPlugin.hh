/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_VACUUM_GRIPPER_PLUGIN_HH_
#define _GAZEBO_VACUUM_GRIPPER_PLUGIN_HH_

#include <memory>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/contacts.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Forward declaration of the private data class.
  class VacuumGripperPluginPrivate;

  /// \brief
  class GAZEBO_VISIBLE VacuumGripperPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: explicit VacuumGripperPlugin();

    /// \brief Destructor.
    public: virtual ~VacuumGripperPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Documentation inherited.
    public: virtual void Reset();

    /// \brief Return the name of the gripper.
    public: std::string Name() const;

    /// \brief Whether the suction of the gripper has been enabled.
    public: bool Enabled() const;

    /// \brief True if the gripper is attached to another model.
    /// \return True if the gripper is active and a joint has been
    /// created between the gripper and another model.
    public: bool Attached() const;

    /// \brief Enable the suction.
    public: void Enable();

    /// \brief Disable the suction.
    public: void Disable();

    /// \brief Update the gripper.
    private: void OnUpdate();

    /// \brief Callback used when the gripper contacts an object.
    /// \param[in] _msg Message that contains contact information.
    private: void OnContacts(ConstContactsPtr &_msg);

    /// \brief Determine if the colliding model is sufficiently in contact with the gripper.
    /// \return True if the colliding model is sufficiently in contact with the gripper.
    private: bool CheckModelContact();

    /// \brief Determine the normal of the contact with the gripper.
    /// \return True if the normal was successfully calculated.
    private: bool GetContactNormal();

    /// \brief Attach an object to the gripper.
    private: void HandleAttach();

    /// \brief Detach an object from the gripper.
    private: void HandleDetach();

    /// \brief Overwrite this method for sending periodic updates with the
    /// gripper state.
    private: virtual void Publish() const;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<VacuumGripperPluginPrivate> dataPtr;
  };
}
#endif
