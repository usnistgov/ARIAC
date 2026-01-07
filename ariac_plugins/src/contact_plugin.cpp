#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <ariac_plugins/contact_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::ContactPlugin,
  gz::sim::System,
  ariac_plugins::ContactPlugin::ISystemPreUpdate,
  ariac_plugins::ContactPlugin::ISystemConfigure)

namespace ariac_plugins{


  void ContactPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    sensor = gz::sim::Sensor(_entity);
    std::optional<gz::sim::Entity> link_entity = sensor.Parent(_ecm);
    if(!link_entity.has_value()){
      throw std::logic_error("Could not find the parent entity of sensor");
    }
    link_obj = gz::sim::Link(link_entity.value());
    std::optional<gz::sim::Model> model_temp = link_obj.ParentModel(_ecm);
    if(!model_temp.has_value()){
      throw std::logic_error("Could not find the parent entity of sensor");
    }
    model = model_temp.value();

    std::string topic = "/world/ariac/model/" + model.Name(_ecm) + 
                        "/link/" + link_obj.Name(_ecm).value() + 
                        "/sensor/" + sensor.Name(_ecm).value() + "/contact";

    topic_to_publish_to = "/" + model.Name(_ecm) + "/" + link_obj.Name(_ecm).value() + "/" + sensor.Name(_ecm).value();

    search_term = _sdf->Get<std::string>("search_term");
    update_rate = _sdf->Get<int>("update_rate");
    iteration_update_mod = 1000/update_rate;

    // GZ Node
    gz_node = std::make_shared<gz::transport::Node>();

    gz_node->Subscribe(topic, &ContactPlugin::contact_msg_cb, this);

    pub = gz_node->Advertise<gz::msgs::StringMsg_V>(topic_to_publish_to);
    if(!pub){
      gzmsg << "Unable to publish topic " << topic_to_publish_to;
    }
  }
  
  void ContactPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    if(_info.paused){
      return;
    }
    if(_info.iterations % iteration_update_mod == 0){
      pub.Publish(current_contact_msg);
    }
    {
      if(_info.simTime.count() - last_set_time > 1E8){
        current_contact_msg = gz::msgs::StringMsg_V();
      }
    }
  }

  void ContactPlugin::contact_msg_cb(const gz::msgs::Contacts &_gz_contacts_msg){
    gz::msgs::StringMsg_V string_vector;
    for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
      std::string collision = _gz_contacts_msg.contact(i).collision2().name();
      if (collision.find(search_term) != std::string::npos){
        string_vector.add_data(collision.substr(0, collision.find("::")));
      }
    }

    auto gz_time = _gz_contacts_msg.header().stamp();
    {
      last_set_time = rclcpp::Time(gz_time.sec(), gz_time.nsec()).nanoseconds();
      current_contact_msg = string_vector;
    }
  }
}