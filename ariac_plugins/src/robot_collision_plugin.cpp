#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

# include <ariac_plugins/robot_collision_plugin.hpp>

GZ_ADD_PLUGIN(
    ariac_plugins::RobotCollisionPlugin,
    gz::sim::System,
    ariac_plugins::RobotCollisionPlugin::ISystemConfigure,
    ariac_plugins::RobotCollisionPlugin::ISystemPreUpdate)

using namespace ariac_plugins;

void RobotCollisionPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  model = gz::sim::Model(_entity);

  robot_name = model.Name(_ecm);

  std::vector<std::pair<std::string, std::string>> contact_names;

  // This each function finds all links in the robot as well as finds all sensor names for use in subscribing to gz topics
  _ecm.Each<gz::sim::components::Link>(
    [&](const gz::sim::Entity &link_entity, const gz::sim::components::Link *link) -> bool
    {
      if (_ecm.ParentEntity(link_entity) == _entity){
        auto link_name = _ecm.Component<gz::sim::components::Name>(link_entity);
        auto descendants = _ecm.Descendants(link_entity);
        for (auto descend : descendants){
          auto descendant_name = _ecm.Component<gz::sim::components::Name>(descend)->Data();
          if (descendant_name.find("sensor") != std::string::npos){
            contact_names.push_back({link_name->Data(), descendant_name});
          }
        }
      }
      return true;
    }
  );

  // Create GZ Node
  gz_node = std::make_shared<gz::transport::Node>();

  // Subscribe to contact sensors
  for (auto contact_name : contact_names) {
    std::string contact_topic = "/world/ariac/model/" + robot_name + "/link/" + contact_name.first + "/sensor/" + contact_name.second + "/contact";
    gz_node->Subscribe(contact_topic, &RobotCollisionPlugin::contact_msg_cb, this);
  }

  // Set Penalty Count to Zero
  contact_info.in_contact = false;
}

void RobotCollisionPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !contact_info.in_contact)
    return;

  double current_time = static_cast<double>(_info.simTime.count());

  // Clear contact if no update in 100ms
  if (current_time - contact_info.last_contact_time > 1e8) {
    contact_info.in_contact = false;
    contact_info.model_name = "";
    contact_info.last_contact_time = 0.0;
    contact_info.last_penalty_time = 0.0;
    return;
  }

  // Issue penalty every 5 seconds (5e9 ns)
  if (current_time - contact_info.last_penalty_time >= seconds_to_reissue_penalty*1e9) {
    ariac_components::Penalty penalty = {
      ariac_components::PenaltyType::ROBOT_COLLISION,
      current_time,
      robot_name + " in contact with " + contact_info.model_name
    };

    gz::sim::Entity penalty_entity = _ecm.CreateEntity();
    _ecm.CreateComponent(penalty_entity, gz::sim::components::Penalty(penalty));

    contact_info.last_penalty_time = current_time;
  }
}

void RobotCollisionPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg)
{
  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i) {
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();

    if (
      collision.find("cell") != std::string::npos || 
      collision.find("shell") != std::string::npos || 
      (collision.find("vg") != std::string::npos) && robot_name == "assembly_robot_2")
      break;

    std::string model_in_contact = collision.substr(0, collision.find("::"));

    const auto &gz_time = _gz_contacts_msg.header().stamp();
    double contact_time = static_cast<double>(gz_time.sec()) * 1e9 + static_cast<double>(gz_time.nsec());

    if (!contact_info.in_contact) {
      contact_info.in_contact = true;
      contact_info.model_name = model_in_contact;
      contact_info.last_penalty_time = -std::numeric_limits<double>::infinity();;  // Issue penalty immediately
    }

    contact_info.last_contact_time = contact_time;
  }
}

