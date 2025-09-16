#ifndef ARIAC_PLUGINS__TOP_SHELL_PLUGIN_HPP_
#define ARIAC_PLUGINS__TOP_SHELL_PLUGIN_HPP_

// GZ
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>

#include <ariac_components/module.hpp>

using gz::sim::components::Module;

namespace ariac_plugins
{
  enum class TopShellLockState {
    LOCKED,
    UNLOCKED
  };

  class TopShellPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_manager) override;

    void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

    private:
      std::optional<std::string> get_cell_in_contact(const gz::msgs::Contacts &);
      gz::sim::Entity get_bottom_shell_entity(gz::sim::EntityComponentManager &, std::string);
      gz::sim::Entity get_base_module_shell(const gz::sim::EntityComponentManager &, const gz::sim::Entity);

      // GZ CBs
      void slot_1_contact_msg_cb(const gz::msgs::Contacts &);
      void slot_4_contact_msg_cb(const gz::msgs::Contacts &);

      // GZ
      std::shared_ptr<gz::transport::Node> gz_node;

      gz::sim::Model top_shell_model;

      gz::sim::Entity shell_base_link = gz::sim::kNullEntity;

      gz::sim::Entity lock_joint;

      // Other
      std::map<int, bool> slot_locked = {
        { 1, false },
        { 4, false },
      };
    
      std::map<int, std::pair<bool, std::string>> cell_in_slot = {
        { 1, { false, "" } },
        { 4, { false, "" } },
      };

      std::map<int, std::string> topic_names;

      TopShellLockState lock_state = TopShellLockState::UNLOCKED;
      
  };
}

#endif // ARIAC_PLUGINS__TOP_SHELL_PLUGIN_HPP_