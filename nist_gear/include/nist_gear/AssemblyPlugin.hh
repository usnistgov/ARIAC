#ifndef _ASSEMBLY_PLUGIN_HH_
#define _ASSEMBLY_PLUGIN_HH_

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
  class AssemblyPluginPrivate;

  /// \brief
  class GAZEBO_VISIBLE AssemblyPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: explicit AssemblyPlugin();

    /// \brief Destructor.
    public: virtual ~AssemblyPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Documentation inherited.
    public: virtual void Reset();

    /// \brief Update the assembly
    private: void OnUpdate();

    /// \brief Callback used when the assembly surface contacts an object.
    /// \param[in] _msg Message that contains contact information.
    private: void OnContacts(ConstContactsPtr &_msg);

    /// \brief Determine if the colliding model is sufficiently in contact with the assembly.
    /// \return True if the colliding model is sufficiently in contact with the assembly.
    private: bool CheckModelContact();

    /// \brief Determine the normal of the contact with the .
    /// \return True if the normal was successfully calculated.
    private: bool GetContactNormal();

    /// \brief Attach an object to the assembly.
    private: void HandleAttach();

    /// \brief Send periodic updates with the completed assembly info
    private: virtual void Publish() const;

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<AssemblyPluginPrivate> dataPtr;
  };
}
#endif
