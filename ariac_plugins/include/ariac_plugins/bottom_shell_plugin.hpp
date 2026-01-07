#ifndef ARIAC_PLUGINS__BOTTOM_SHELL_PLUGIN_HPP_
#define ARIAC_PLUGINS__BOTTOM_SHELL_PLUGIN_HPP_

#include <queue>

// GZ
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <ariac_components/module.hpp>


using gz::sim::components::Module;

namespace ariac_plugins
{
  enum class BottomShellTeleportState {
    IDLE,
    READY,
    JOINT_NEEDED,
    JOINT_REMOVAL,
    FINISHED
  };

  struct CellToLock {
    gz::sim::Model cell_model;
    std::string direction;
    int slot;
  };

  class BottomShellPlugin:
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
      gz::sim::Model bottom_shell_model;

      gz::sim::Entity bottom_shell_entity;

      gz::sim::Entity shell_base_link = gz::sim::kNullEntity;

      std::map<int, gz::sim::Entity> lock_joints = {
        { 1, gz::sim::kNullEntity },
        { 2, gz::sim::kNullEntity },
        { 3, gz::sim::kNullEntity },
        { 4, gz::sim::kNullEntity },
      };

      std::map<int, bool> slot_teleported = {
        { 1, false },
        { 2, false },
        { 3, false },
        { 4, false },
      };

      std::map<int, bool> slot_locked = {
        { 1, false },
        { 2, false },
        { 3, false },
        { 4, false },
      };
    
      std::map<int, std::pair<bool, std::string>> cell_in_slot = {
        { 1, { false, "" } },
        { 2, { false, "" } },
        { 3, { false, "" } },
        { 4, { false, "" } },
      };

      std::map<int, float> slot_offsets = {
        {1, -0.036},
        {2, -0.012},
        {3, 0.012},
        {4, 0.036}
      };

      std::map<std::string, gz::math::Pose3d> up_down_poses = {
        {"up", gz::math::Pose3d(0.0, 0.0, 0.0435, 0.0, 0.0, 0.0)},
        {"down", gz::math::Pose3d(0.0, 0.0, 0.0435, 0.0, M_PI, 0.0)}
      };

      std::shared_ptr<gz::transport::Node> gz_node;
      std::map<int, std::string> topic_names;

      gz::sim::Entity lock_joint = gz::sim::kNullEntity;
      gz::sim::Entity section_3_link_entity;
      
      std::optional<std::string> get_cell_in_contact(const gz::msgs::Contacts &_gz_contacts_msg);

      bool set_visuals = false;
      bool component_set = false;

      int teleport_step;

      gz::math::Pose3d section_3_pose = gz::math::Pose3d(4.2, 5.55, 0.422, 0.0, 0.0, 0.0);

      BottomShellTeleportState teleport_state = BottomShellTeleportState::IDLE;

      std::queue<std::pair<int, gz::sim::Model>> cells_to_teleport;
      std::queue<CellToLock> cells_to_lock;

      // GZ CBs
      void slot_1_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
      void slot_2_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
      void slot_3_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
      void slot_4_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);

      void base_contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg);
  };
}

#endif