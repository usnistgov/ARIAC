#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <ariac_plugins/kit_tray_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::KitTrayPlugin,
  gz::sim::System,
  ariac_plugins::KitTrayPlugin::ISystemPreUpdate,
  ariac_plugins::KitTrayPlugin::ISystemConfigure)

namespace ariac_plugins{

  KitTrayPlugin::~KitTrayPlugin()
  {
    executor->cancel();
    thread_executor_spin.join();
  }

  void KitTrayPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    model = gz::sim::Model(_entity);
    base_link = gz::sim::Link(model.LinkByName(_ecm, "base_link"));

    tray_name = model.Name(_ecm);

    // GZ Node
    gz_node = std::make_shared<gz::transport::Node>();

    std::string slot_left_contact_topic = tray_name + "/slot_{n}_link/left_contact_sensor";
    std::string slot_right_contact_topic = tray_name + "/slot_{n}_link/right_contact_sensor";

    agv_name = tray_name.substr(0, 4);
    
    ros_node = rclcpp::Node::make_shared("kit_tray_plugin", tray_name);

    rclcpp::Parameter sim_time("use_sim_time", true);
    ros_node->set_parameter(sim_time);

    // Spin up executor thread
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(ros_node);

    auto spin = [this](){
      while(rclcpp::ok()){
        executor->spin_once();
      }
    };

    thread_executor_spin = std::thread(spin);

    location_subscription = ros_node->create_subscription<ariac_interfaces::msg::AgvStatus>(
      "/"+agv_name+"/info", 
      10,
      std::bind(&KitTrayPlugin::agv_info_cb, this, std::placeholders::_1)
    );

    // Hold GZ topic names
    std::map<std::string, std::map<int, std::string>> topic_names;

    // Build correct topic names
    for (int i=1; i<=4; i++) {
      std::string name = slot_left_contact_topic;
      topic_names["left"][i] = name.replace(slot_left_contact_topic.find("{n}"), 3, std::to_string(i));
      name = slot_right_contact_topic;
      topic_names["right"][i] = name.replace(slot_right_contact_topic.find("{n}"), 3, std::to_string(i));
    }

    // Subscribe to gz topics
    for (const auto& [side, slot_topic_names] : topic_names) {
      for (const auto& [slot, topic] : slot_topic_names) {
        std::function callback_function = [this, slot, side](const gz::msgs::StringMsg_V &msg) {
          this->cell_contacts_msg_cb(slot, side, msg);
        };
        gz_node->Subscribe(topic, callback_function);
      }
    }
  }
  
  void KitTrayPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    update_slot_cells(_ecm);
    
    switch(agv_station){
      case AGVStations::IN_TRANSIT:
      case AGVStations::INSPECTION:
      {
        for(const auto& [slot, cell_info] : collision_slot_cells){
          if(lock_joints[slot] != gz::sim::kNullEntity || !cell_info.has_value()){
            continue;
          }

          lock_slot(slot, _ecm);
          ariac_components::SlotCellInfo slot_info;
          slot_info.cell_type = cell_info.value().type;
          slot_info.defective = cell_info.value().defective;
          slot_info.voltage = cell_info.value().voltage;
          kit_component.slots.at(slot) = slot_info;
          locked_cells[slot] = cell_info;
        }
        break;
      }
      case AGVStations::SHIPPING:
      case AGVStations::RECYCLING:
        break;
      case AGVStations::ASSEMBLY:
        for(const auto& [slot, ent] : lock_joints){
          if (ent == gz::sim::kNullEntity){
            continue;
          }
          unlock_slot(slot, _ecm);
          kit_component.slots.at(slot) = std::nullopt;
          locked_cells[slot] = std::nullopt;
        }
        break;
    }

    if(_info.iterations%10==0){
      if(!_ecm.EntityHasComponentType(model.Entity(), gz::sim::components::Kit::typeId)){
        _ecm.CreateComponent(model.Entity(), gz::sim::components::Kit(kit_component));
      } else {
        auto component = _ecm.Component<gz::sim::components::Kit>(model.Entity());
        if (component)
        {
          component->SetData(kit_component, ariac_components::Kit::equal);
        }
      }
    }
  }

  void KitTrayPlugin::update_slot_cells(gz::sim::EntityComponentManager &_ecm){
    std::map<int, SlotContact> contacts_copy;

    contacts_copy = slot_contacts;

    for(auto& [slot, info] : contacts_copy){
      if(info.right == "" || info.left != info.right){
        collision_slot_cells[slot] = std::nullopt;
        continue;
      }
      
      std::optional<gz::sim::Entity> cell_entity = _ecm.EntityByName(info.left);

      if(!cell_entity.has_value()){
        collision_slot_cells[slot] = std::nullopt;
        continue;
      }

      SlotCell slot_cell;
      slot_cell.entity = cell_entity.value();

      auto component = _ecm.Component<gz::sim::components::Cell>(cell_entity.value());

      if(component == nullptr){
        collision_slot_cells[slot] = std::nullopt;
        continue;
      }

      slot_cell.defective = component->Data().defective;
      slot_cell.voltage = component->Data().voltage;
      slot_cell.type = component->Data().cell_type;
      
      collision_slot_cells[slot] = slot_cell;
    }
  }

  void KitTrayPlugin::lock_slot(int slot, gz::sim::EntityComponentManager &_ecm){
    if(lock_joints[slot] != gz::sim::kNullEntity || !collision_slot_cells[slot].has_value()){
      return;
    }

    lock_joints[slot] = _ecm.CreateEntity();
    
    auto cell_link = gz::sim::Model(collision_slot_cells[slot].value().entity).LinkByName(_ecm, "base_link");

    _ecm.CreateComponent(lock_joints[slot], gz::sim::components::DetachableJoint({base_link.Entity(), cell_link, "fixed"}));
  }

  void KitTrayPlugin::unlock_slot(int slot, gz::sim::EntityComponentManager &_ecm){
    if(lock_joints[slot] == gz::sim::kNullEntity){
      return;
    }

    _ecm.RequestRemoveEntity(lock_joints[slot]);
    lock_joints[slot] = gz::sim::kNullEntity;
  }

  void KitTrayPlugin::cell_contacts_msg_cb(int slot, std::string side, const gz::msgs::StringMsg_V &msg){
    auto data = msg.data();
    
    if(data.empty()){

      if (side == "left") {
        slot_contacts[slot].left = "";
      } else if (side == "right") {
        slot_contacts[slot].right = "";
      }
    } else {

      if (side == "left") {
        slot_contacts[slot].left = data.at(0);
      } else if (side == "right") {
        slot_contacts[slot].right = data.at(0);
      }
    }
  }

  void KitTrayPlugin::agv_info_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
    agv_station = msg->station_id;
  }
}