#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <ariac_plugins/agv_tray_interface_plugin.hpp>

GZ_ADD_PLUGIN(
  ariac_plugins::AgvTrayInterfacePlugin,
  gz::sim::System,
  ariac_plugins::AgvTrayInterfacePlugin::ISystemPreUpdate,
  ariac_plugins::AgvTrayInterfacePlugin::ISystemConfigure)

namespace ariac_plugins{

  AgvTrayInterfacePlugin::~AgvTrayInterfacePlugin()
  {
    executor->cancel();
    thread_executor_spin.join();
  }

  void AgvTrayInterfacePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    agv_model = gz::sim::Model(_entity);
    agv_base_link_entity = agv_model.LinkByName(_ecm, "base_link");

    if (agv_base_link_entity == gz::sim::kNullEntity) {
      throw std::runtime_error("Unable to get agv base link");
    }

    agv_base_link = gz::sim::Link(agv_base_link_entity);

    agv_name = agv_model.Name(_ecm);

    wait_until_iteration = 1000;

    // GZ Node
    gz_node = std::make_shared<gz::transport::Node>();
    
    ros_node = rclcpp::Node::make_shared("agv_interface_plugin", agv_name);

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

    recycle_srv = ros_node->create_service<ariac_interfaces::srv::Trigger>(
      "recycle_cells",
      std::bind(&AgvTrayInterfacePlugin::recycle_cells_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    check_kit_quality_srv = ros_node->create_service<ariac_interfaces::srv::CheckKitQuality>(
      "check_kit_quality",
      std::bind(&AgvTrayInterfacePlugin::check_kit_quality_cb, this, std::placeholders::_1, std::placeholders::_2)
    );

    agv_tray_info_pub = ros_node->create_publisher<ariac_interfaces::msg::AgvTrayStatus>("tray_status", 10);

    location_subscription = ros_node->create_subscription<ariac_interfaces::msg::AgvStatus>(
      "info", 
      10,
      std::bind(&AgvTrayInterfacePlugin::agv_info_cb, this, std::placeholders::_1)
    );

    if (!gz_node->Advertise("/"+agv_name+"/handle_kit", &AgvTrayInterfacePlugin::handle_kitting_submission, this)){
      gzmsg << "Error advertising /"+agv_name+"/handle_kit";
    }
  }
  
  void AgvTrayInterfacePlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    if(wait_until_iteration.has_value() && _info.iterations < wait_until_iteration){
      return;
    }
    wait_until_iteration = std::nullopt;
    
    switch(tray_status){
      case TrayState::INSERTING:
      {
        current_tray_name = agv_name + "_tray_" + std::to_string(tray_index);
        tray_index++;

        auto pose_opt = agv_base_link.WorldPose(_ecm);

        if (!pose_opt.has_value()) {
          throw std::runtime_error("Cannot add new tray, Unable to get AGV pose.");
        }

        if (!current_tray_name.has_value()) {
          throw std::runtime_error("No tray name set");
        }
        
        spawn_tray(pose_opt.value(), current_tray_name.value());
        tray_status = TrayState::LOCKING;
        wait_until_iteration = _info.iterations + 5;
        break;
      }

      case TrayState::LOCKING:
      {
        if (!current_tray_name.has_value()) {
          throw std::runtime_error("No tray name set");
        }

        auto tray_entity = _ecm.EntityByName(current_tray_name.value());

        if (!tray_entity.has_value()) {
          throw std::runtime_error("Unable to get entity for tray");
        }

        auto tray_link = gz::sim::Model(tray_entity.value()).LinkByName(_ecm, "base_link");

        if (tray_link == gz::sim::kNullEntity) {
          throw std::runtime_error("Unable to get base link for tray");
        }

        lock_joint = _ecm.CreateEntity();

        _ecm.CreateComponent(lock_joint, gz::sim::components::DetachableJoint({agv_base_link_entity, tray_link, "fixed"}));

        tray_status = TrayState::LOCKED;

        wait_until_iteration = _info.iterations + 10;

        break;
      }

      case TrayState::LOCKED:
      {
        if (!current_tray_name.has_value()) {
          throw std::runtime_error("No tray name set");
        }

        auto tray_entity = _ecm.EntityByName(current_tray_name.value());

        if (!tray_entity.has_value()) {
          throw std::runtime_error("Unable to get entity for tray");
        }
        
        auto component = _ecm.Component<gz::sim::components::Kit>(tray_entity.value());

        if (component == nullptr) {
          throw std::runtime_error("Unable to get kit component for tray");
        }

        kit_component = component->Data();

        if(_info.iterations % 100 == 0){
          ariac_interfaces::msg::AgvTrayStatus status;
          status.slot_1_occupied = kit_component.value().slots[1].has_value();
          status.slot_2_occupied = kit_component.value().slots[2].has_value();
          status.slot_3_occupied = kit_component.value().slots[3].has_value();
          status.slot_4_occupied = kit_component.value().slots[4].has_value();
          
          agv_tray_info_pub->publish(status);
        }
        break;
      }

      case TrayState::UNLOCKING:
      {
        if(lock_joint == gz::sim::kNullEntity){
          throw std::runtime_error("Tray lock joint does not exist in unlocking tray\n");
        }

        _ecm.RequestRemoveEntity(lock_joint);
        lock_joint = gz::sim::kNullEntity;
        kit_component = std::nullopt;
        tray_status = TrayState::MOVING_TO_SHELF;

        wait_until_iteration = _info.iterations + 5;

        break;
      }
      
      case TrayState::MOVING_TO_SHELF:
      {
        auto shelf_entity_opt = _ecm.EntityByName(shelf_model_names[agv_station]);
        
        if(!shelf_entity_opt.has_value()){
          throw std::runtime_error("Could not find shelf entity");
        }

        ariac_components::ShelfSlot shelf_slot;

        if(!_ecm.EntityHasComponentType(shelf_entity_opt.value(), gz::sim::components::ShelfSlot::typeId)){
          _ecm.CreateComponent<gz::sim::components::ShelfSlot>(shelf_entity_opt.value(), gz::sim::components::ShelfSlot(shelf_slot));
        } else {
          auto component = _ecm.Component<gz::sim::components::ShelfSlot>(shelf_entity_opt.value());
          if (component == nullptr) {
            throw std::runtime_error("Could not find shelf slot component");
          }
          shelf_slot = component->Data();
        }
        
        auto shelf_base_link_entity = gz::sim::Model(shelf_entity_opt.value()).LinkByName(_ecm, "base_link");
        if (shelf_base_link_entity == gz::sim::kNullEntity) {
          throw std::runtime_error("Unable to find shelf base link");
        }

        gz::sim::Link shelf_base_link = gz::sim::Link(shelf_base_link_entity);
        auto shelf_world_pose_opt = shelf_base_link.WorldPose(_ecm);
        if (!shelf_world_pose_opt.has_value()){
          throw std::runtime_error("Could not find world pose for shelf_base_link");
        }
        
        if (!current_tray_name.has_value()) {
          throw std::runtime_error("No tray name set");
        }

        auto tray_entity = _ecm.EntityByName(current_tray_name.value());

        if (!tray_entity.has_value()) {
          throw std::runtime_error("Unable to get entity for tray");
        }

        auto tray_model = gz::sim::Model(tray_entity.value());

        tray_model.SetWorldPoseCmd(_ecm, shelf_world_pose_opt.value() * shelf_slot.SLOT_TRANSFORMS[shelf_slot.index]);
        shelf_slot.index++;
        
        _ecm.SetComponentData<gz::sim::components::ShelfSlot>(shelf_entity_opt.value(), shelf_slot);
        
        tray_status = TrayState::LOCKING_TO_SHELF;
        
        break;
      }
      case TrayState::LOCKING_TO_SHELF:
      {
        // RCLCPP_INFO_STREAM(ros_node->get_logger(), "LOCKING TRAY TO SHELF");
        gz::sim::Entity floor_link = _ecm.EntityByComponents(gz::sim::components::Name(floor_link_name), gz::sim::components::Link());
      
        if (floor_link == gz::sim::kNullEntity) {
          gzerr << "Unable to locate floor link";
          throw std::runtime_error("Unable to locate floor link");
        }

        if (!current_tray_name.has_value()) {
          throw std::runtime_error("No tray name set");
        }

        auto tray_entity = _ecm.EntityByName(current_tray_name.value());

        if (!tray_entity.has_value()) {
          throw std::runtime_error("Unable to get entity for tray");
        }

        auto tray_link = gz::sim::Model(tray_entity.value()).LinkByName(_ecm, "base_link");

        if (tray_link == gz::sim::kNullEntity) {
          throw std::runtime_error("Unable to get base link for tray");
        }
    
        _ecm.CreateComponent(_ecm.CreateEntity(), gz::sim::components::DetachableJoint({floor_link, tray_link, "fixed"}));
        
        tray_status = TrayState::INSERTING;
        
        break;
      }
      
      default:
        break;
      
      
    }

  }


  void AgvTrayInterfacePlugin::agv_info_cb(ariac_interfaces::msg::AgvStatus::SharedPtr msg){
    agv_station = msg->station_id;
  }

  void AgvTrayInterfacePlugin::recycle_cells_cb(const ariac_interfaces::srv::Trigger::Request::SharedPtr, ariac_interfaces::srv::Trigger::Response::SharedPtr res){
    if(agv_station != AGVStations::RECYCLING){
      res->message = agv_name + " not at the recycling station";
      res->success = false;
      return;
    }
    tray_status = TrayState::UNLOCKING;

    rclcpp::Time start_time = ros_node->now();
    while (rclcpp::ok()) {
      if (tray_status != TrayState::LOCKED) {
        break;
      } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(5.0)) {
        res->success = false;
        res->message = "Timed out while processing recycling request";
        return;
      }
    };

    res->message = "Recycling cells on " + agv_name;
    res->success = true;
  }

  void AgvTrayInterfacePlugin::check_kit_quality_cb(const ariac_interfaces::srv::CheckKitQuality::Request::SharedPtr req, ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr res){
    if (agv_station != AGVStations::INSPECTION){
      res->is_good = false;
      res->message = "AGV must be at inspection to check kit quality";
      return;
    }

    auto kit_validation = validate_kit(req->cell_type);
    res->is_good = kit_validation->is_good;
    res->message = kit_validation->message;
  }

  bool AgvTrayInterfacePlugin::handle_kitting_submission(
    const gz::msgs::Int32 &req, 
    gz::msgs::Boolean &res
  ){
    int cell_type = req.data();
    if(cell_type != CellTypes::LI_ION && cell_type != CellTypes::NIMH){
      gzerr << "Entered cell type (" << cell_type << ") is not valid\n";
      res.set_data(false);
      return false;
    }

    if (agv_station != AGVStations::SHIPPING){
      gzerr << "AGV not at shipping";
      res.set_data(false);
      return false;
    }

    ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr kit_quality = validate_kit(cell_type);

    if(!kit_quality->is_good){
      gzerr << kit_quality->message << "\n";
      res.set_data(false);
      return false;
    }

    tray_status = TrayState::UNLOCKING;

    rclcpp::Time start_time = ros_node->now();
    while (rclcpp::ok()) {
      if (tray_status != TrayState::LOCKED) {
        break;
      } else if (ros_node->now() - start_time > rclcpp::Duration::from_seconds(5.0)) {
        gzerr << "Timed out while processing kit submission tray tp";
        res.set_data(false);
        return false;
      }
    };

    res.set_data(true);
    return true;
  }

  ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr AgvTrayInterfacePlugin::validate_kit(int cell_type){
    
    ariac_interfaces::srv::CheckKitQuality::Response::SharedPtr res = std::make_shared<ariac_interfaces::srv::CheckKitQuality::Response>();
    if(cell_type != CellTypes::LI_ION && cell_type != CellTypes::NIMH){
      res->is_good = false;
      res->message = "Entered cell type (" + std::to_string(cell_type) + ") is not valid";
      return res;
    }

    if(kit_component == std::nullopt){
      res->is_good = false;
      res->message = "Could not find kit component on tray model";
      return res;
    }

    res->message = "";

    float total_voltage = 0;

    for(const auto& [slot, cell_info] : kit_component->slots){
      if(!cell_info.has_value()){
        res->message += "Cell in slot" + std::to_string(slot) + ") is not valid. ";
      }

      ariac_components::SlotCellInfo cell_being_checked = cell_info.value();
      
      total_voltage += cell_being_checked.voltage;

      if(cell_being_checked.cell_type != cell_type){
        res->message += "Cell in slot " + std::to_string(slot) + " is not the correct type. ";
        continue;
      }
      if(cell_being_checked.defective){
        res->message += "Cell in slot " + std::to_string(slot) + " is defective. ";
        continue;
      }
      if(abs(cell_being_checked.voltage - nominal_voltages[cell_type]) > CellTypes::CELL_VOLTAGE_TOLERANCE){
        res->message += "Cell in slot " + std::to_string(slot) + " is not within the cell voltage tolerance. ";
      }
    }

    if (res->message != ""){
      res->is_good = false;
      return res;
    }


    if (abs(total_voltage - (nominal_voltages[cell_type] * 4)) > CellTypes::KIT_VOLTAGE_TOLERANCE) {
      res->message = "Total voltage of good cells is not within allowed tolerance.";
      res->is_good = false;
      return res;
    }

    res->message = "Kit is good";
    res->is_good = true;

    return res;
  }

  void AgvTrayInterfacePlugin::spawn_tray(gz::math::Pose3d agv_pose, std::string name){
    

    tinyxml2::XMLDocument doc;

    if (doc.LoadFile((ament_index_cpp::get_package_share_directory("ariac_gz")+"/models/kit_tray/model.sdf").c_str()) != tinyxml2::XML_SUCCESS) {
      return;
    }

    // Convert from XML to string
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    
    auto sdf = printer.CStr();

    gz::msgs::EntityFactory req;

    req.set_name(name);
    req.set_sdf(sdf);
    gz::msgs::Set(req.mutable_pose(), agv_pose * tray_spawn_transform);

    gz::msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    bool executed = gz_node->Request("/world/ariac/create", req, timeout, res, result);

    if (executed) {
      if (!result && res.data()) {
        gzerr << "Failed request to create entity.";
        return;
      }
    } else {
      gzerr << "Request to create entity from create service timed out.";
      return;
    }

  }

}