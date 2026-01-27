#include "ariac_plugins/cheat_tools_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::CheatToolsPlugin,
  gz::sim::System,
  ariac_plugins::CheatToolsPlugin::ISystemConfigure,
  ariac_plugins::CheatToolsPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;
  
void CheatToolsPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_event_mgr)
{
  // Create GZ Node
  gz_node = std::make_shared<gz::transport::Node>();

  shell_paths[ShellTypes::TOP] = ament_index_cpp::get_package_share_directory("ariac_gz") + "/models/battery_module/top_shell/model.sdf";
  shell_paths[ShellTypes::BOTTOM] = ament_index_cpp::get_package_share_directory("ariac_gz") + "/models/battery_module/bottom_shell/model.sdf";

  sdf_path = ament_index_cpp::get_package_share_directory("ariac_gz") + "/models/battery_cell/model.sdf";

  sdf = _sdf;

  // Read defect config file
  std::string share_dir = ament_index_cpp::get_package_share_directory("ariac_setup");
  YAML::Node config = YAML::LoadFile(share_dir + "/config/defects.yaml"); 
  
  YAML::Node defect_types_node = config["DEFECT_TYPES"];

  if (!defect_types_node.IsDefined() || !defect_types_node.IsMap()){
    throw std::runtime_error("Defect Types not found in config");
  }

  for (const auto& defect_type_node : defect_types_node){
    int defect_type = defect_type_node.first.as<int>();
    
    YAML::Node defects_list_node = defect_type_node.second["DEFECTS"];

    if (!defects_list_node.IsDefined() || !defects_list_node.IsSequence()){
      throw std::runtime_error("Error reading defects in config");
    }

    std::vector<ariac_interfaces::msg::CellDefect> defects_vector;

    for (const auto& defect : defects_list_node){
      ariac_interfaces::msg::CellDefect d;

      if (!defect["TYPE"].IsDefined() || !defect["THETA"].IsDefined() || !defect["Z"].IsDefined()) {
        throw std::runtime_error("Defect not properly structured");
      }

      d.defect_type = defect["TYPE"].as<int>();
      d.theta = defect["THETA"].as<double>();
      d.z = defect["Z"].as<double>(); 

      defects_vector.push_back(d);
    }

    defect_info[defect_type] = defects_vector;
  }
}

void CheatToolsPlugin::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
  if (_info.iterations < 1000) {
    return;
  } else if (_info.iterations == 1000) {

    if(sdf->HasElement("kit_on_agv1") && sdf->Get<bool>("kit_on_agv1")){
      if(sdf->HasElement("agv1_high_priority")){
        complete_kit(1, sdf->Get<bool>("agv1_high_priority"));
      } else {
        complete_kit(1, false);
      }
    }

    if(sdf->HasElement("kit_on_agv2") && sdf->Get<bool>("kit_on_agv2")){
      if(sdf->HasElement("agv2_high_priority")){
        complete_kit(2, sdf->Get<bool>("agv2_high_priority"));
      } else {
        complete_kit(2, false);
      }
    }

    if(sdf->HasElement("kit_on_agv3") && sdf->Get<bool>("kit_on_agv3")){
      if(sdf->HasElement("agv3_high_priority")){
        complete_kit(3, sdf->Get<bool>("agv3_high_priority"));
      } else {
        complete_kit(3, false);
      }
    }

    if(sdf->HasElement("module") && sdf->Get<bool>("module")){
      complete_module();
      if(sdf->HasElement("module_has_welds") && sdf->Get<bool>("module_has_welds")){
        welds_requested = true;
      }
    }

    if(sdf->HasElement("partial_module") && sdf->Get<bool>("partial_module")){
      partial_module();
    }

    if(sdf->HasElement("flipped_module") && sdf->Get<bool>("flipped_module")){
      complete_module();
      if(sdf->HasElement("module_has_welds") && sdf->Get<bool>("module_has_welds")){
        welds_requested = true;
      }
      teleport_bottom_shell = TeleportStatus::REQUESTED;
    }

    if(sdf->HasElement("cells_in_voltage_testers") && sdf->Get<bool>("cells_in_voltage_testers")){
      spawn_cells_in_voltage_testers();
    }
    
    if(sdf->HasElement("log_cell_info") && sdf->Get<bool>("log_cell_info")){
      log_cell_info = true;
    }

  }

  if(log_cell_info){
    _ecm.Each<gz::sim::components::Cell>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Cell *cell) -> bool {
            auto c_data = cell->Data();
            if(std::find(logged_cells.begin(), logged_cells.end(), c_data.cell_name) == logged_cells.end()){
              gzmsg << "Cell " << c_data.cell_name << ":\n";
              gzmsg << "\tType: " << (c_data.cell_type==1 ? "Lithium Ion" : "NIMH") << "\n";
              gzmsg << "\tVoltage: " << c_data.voltage << "\n";
              gzmsg << "\tRotation: " << c_data.rotation << "\n";
              if (c_data.defective){
                gzmsg << "\tDefective: True\n";
                gzmsg << "\tDefect type: " << c_data.defect_type << "\n";
                gzmsg << "\tDefects:\n";
                for (const auto& defect : defect_info[static_cast<int>(c_data.defect_type)]){
                  switch (defect.defect_type)
                  {
                  case 1:
                    gzmsg << "\t - Type: Dent\n";
                    break;
                  case 2:
                    gzmsg << "\t - Type: Bulge\n";
                    break;
                  case 3:
                    gzmsg << "\t - Type: Scratch\n";
                    break;
                  default:
                    gzmsg << "\tCould not find defect type\n";
                    break;
                  }
                  gzmsg << "\t   Relative theta: " << defect.theta << "\n";
                  gzmsg << "\t   Absolute theta: " << defect.theta + c_data.rotation << "\n";
                  gzmsg << "\t   Z: " << defect.z << "\n";
                }
              } else {
                gzmsg << "\tDefective: False\n";
              }
              
              logged_cells.push_back(c_data.cell_name);
            }
            return true;
        }
    );
  }
  
  if(!welds_requested && components_to_add.size() == 0 && 
    (teleport_bottom_shell == TeleportStatus::NOT_NEEDED || 
    teleport_bottom_shell == TeleportStatus::TELEPORTED))
  {
    return;
  }
  
  for(const auto& pair : components_to_add){
    if(!_ecm.EntityByName(pair.first).has_value()) {return;}
  }

  for(auto& pair : components_to_add){
    pair.second.cell_entity = _ecm.EntityByName(pair.first).value();
    _ecm.CreateComponent<gz::sim::components::Cell>(
      pair.second.cell_entity,
      gz::sim::components::Cell(pair.second)
    );
  }

  components_to_add.clear();

  std::optional<gz::sim::Entity> bottom_shell_entity = _ecm.EntityByName(bottom_shell_name);
  if(welds_requested){
    if(!bottom_shell_entity.has_value()) {return;}

    auto module = _ecm.Component<gz::sim::components::Module>(bottom_shell_entity.value());
    if(module==nullptr){
      return;
    }
    auto currentState = module->Data();
    if(currentState.top_shell_entity == gz::sim::kNullEntity){
      return;
    }

    for(int i = 1; i <= 4; i++){
      if(currentState.cell_orientation[i] == ariac_components::CellOrientation::NOT_PRESENT){return;}
    }

    for(int i = 1; i <= 4; i++){
      currentState.top_welds[i] = true;
    }

    currentState.bottom_welds[1] = true;
    currentState.bottom_welds[2] = true;

    auto bottom_shell_model = gz::sim::Model(bottom_shell_entity.value());

    for(int i = 1; i < 3; i++){
      auto bead_link = gz::sim::Link(bottom_shell_model.LinkByName(_ecm, "bead_" + std::to_string(i) + "_link"));

      gz::sim::Entity bead_visual_entity = bead_link.VisualByName(_ecm, "bead_visual_"+std::to_string(i));
      
      bool success = change_visual_status(bead_visual_entity, true);
    }

    auto top_shell_entity = gz::sim::Model(currentState.top_shell_entity);

    for(int i = 1; i < 5; i++){
      auto bead_link = gz::sim::Link(top_shell_entity.LinkByName(_ecm, "bead_" + std::to_string(i) + "_link"));

      gz::sim::Entity bead_visual_entity = bead_link.VisualByName(_ecm, "bead_visual_"+std::to_string(i));
      
      bool success = change_visual_status(bead_visual_entity, true);
    }

    module->SetData(
      currentState,
      ariac_components::Module::equal
    );
    welds_requested = false;
  }

  switch(teleport_bottom_shell){
    case TeleportStatus::NOT_NEEDED:
      break;
    case TeleportStatus::REQUESTED:
    {
      if(!bottom_shell_entity.has_value()){break;}
      if(request_step < 0){
        request_step = _info.iterations;
      }

      if(_info.iterations - request_step < 1000){
        break;
      }

      auto module = _ecm.Component<gz::sim::components::Module>(bottom_shell_entity.value());
      if(module==nullptr){break;}
      
      auto currentState = module->Data();
      if(currentState.top_shell_entity == gz::sim::kNullEntity){break;}

      gz::sim::Model(bottom_shell_entity.value()).SetWorldPoseCmd(_ecm, flipped_bottom_shell_pose);
      gzwarn << "\n\n\n\n\n\nFlipped module teleported\n\n\n\n\n\n";
      teleport_bottom_shell = TeleportStatus::TELEPORTED;
      break;
    }
    case TeleportStatus::TELEPORTED:
      break;
    default:
      break;
  }
}

void CheatToolsPlugin::complete_module(){
  spawn_shell(ShellTypes::BOTTOM, shell_poses[ShellTypes::BOTTOM]);
  for(int i = 1; i <= 4; i++){
    spawn_cell(get_next_cell(false), shell_poses[ShellTypes::BOTTOM] * module_slot_offsets[i]);
  }
  spawn_shell(ShellTypes::TOP, shell_poses[ShellTypes::TOP]);
}

void CheatToolsPlugin::partial_module(){
  spawn_shell(ShellTypes::BOTTOM, partial_module_bottom_shell_pose);
  for(int i = 1; i <= 4; i++){
    spawn_cell(get_next_cell(false), partial_module_bottom_shell_pose * module_slot_offsets[i]);
  }
}

void CheatToolsPlugin::flipped_module(){
  spawn_shell(ShellTypes::TOP, flipped_shell_poses[ShellTypes::TOP]);
  for(int i = 1; i <= 4; i++){
    spawn_cell(get_next_cell(false), flipped_shell_poses[ShellTypes::TOP] * module_slot_offsets[i]);
  }
  spawn_shell(ShellTypes::BOTTOM, flipped_shell_poses[ShellTypes::BOTTOM]);
}

void CheatToolsPlugin::complete_kit(int agv, bool high_priority){
  for(int i = 1; i <= 4; i++){
    spawn_cell(get_next_cell(high_priority), agv_poses[agv] * (agv_slot_offsets[i]));
  }
}

void CheatToolsPlugin::spawn_cells_in_voltage_testers(){
  spawn_cell(get_next_cell(false), vt_cell_poses[1]);
  spawn_cell(get_next_cell(false), vt_cell_poses[2]);
}


ariac_components::Cell CheatToolsPlugin::get_next_cell(bool high_priority)
{
  ariac_components::Cell cell;

  cell.cell_type = high_priority ? CellTypes::NIMH : CellTypes::LI_ION;

  cell.cell_name = std::string(high_priority ? "NIMH" : "LI-ION") + "_cheat_cell_" + std::to_string(++cell_count);
  
  cell.defective = false;
  
  cell.defect_type = 0;

  cell.rotation = 0;

  cell.voltage = high_priority ? CellTypes::NIMH_NOMINAL_VOLTAGE : CellTypes::LI_ION_NOMINAL_VOLTAGE;

  return cell;
}

void CheatToolsPlugin::spawn_cell(ariac_components::Cell cell, gz::math::Pose3d pose){

  gz::msgs::EntityFactory req;

  req.set_name(cell.cell_name); // Names the cell

  auto xml = generate_cell_sdf(cell);
  
  if (!xml.has_value()){
    return;
  }

  req.set_sdf(xml.value());

  gz::msgs::Set(req.mutable_pose(), pose);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  bool executed = gz_node->Request("/world/ariac/create", req, timeout, res, result);

  if (executed) {
    if (!result && res.data()) {
      gzerr << "Failed request to create entity.\n";
    } else {
      components_to_add.push_back(std::make_pair(cell.cell_name, cell));
    }
  } else {
    gzerr << "Request to create entity from create service timed out.\n";
  }

}

std::optional<std::string> CheatToolsPlugin::generate_cell_sdf(ariac_components::Cell cell){ 
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::nullopt;
  }

  auto root = doc.RootElement();

  if (!root) {
    return std::nullopt;
  }

  std::string visual_path = "model://battery_cell/meshes/" + cell_names[cell.cell_type];

  // Change visual to correct model for defect type

  if (!cell.defective){
    visual_path += "/base.glb";
  } else {
    visual_path += "/defect_" + std::to_string(cell.defect_type) + ".glb";
  } 

  // Set color based on type  
  auto current_element = root;
  for (std::string tag : {"model", "link", "visual", "geometry", "mesh", "uri"}){
    current_element = current_element->FirstChildElement(tag.c_str());
    
    if(!current_element){
      return std::nullopt;
    }

    if (tag == "uri") {
      current_element->SetText(visual_path.c_str());
    }
  }
  
  // Convert from XML to string
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  
  return printer.CStr();
}

bool CheatToolsPlugin::spawn_shell(ShellTypes shell_type, gz::math::Pose3d pose){
  gz::msgs::EntityFactory req;

  req.set_name("cheat_" + shell_names[shell_type] + "_shell_0"); // Names the cell

  if(shell_type == ShellTypes::BOTTOM){
    bottom_shell_name = req.name();
  }

  auto xml = get_shell_xml(shell_type);
  if (!xml.has_value()){
    return false;
  }
  req.set_sdf(xml.value());

  gz::msgs::Set(req.mutable_pose(), pose);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  bool executed = gz_node->Request("/world/ariac/create", req, timeout, res, result);

  if (executed) {
    if (!result && res.data()) {
      gzerr << "Failed request to create entity.";
      return false;
    }
  } else {
    gzerr << "Request to create entity from create service timed out.";
    return false;
  }
  
  return true;
}

std::optional<std::string> CheatToolsPlugin::get_shell_xml(ShellTypes shell_type){ 
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(shell_paths[shell_type].c_str()) != tinyxml2::XML_SUCCESS) {
    gzerr << "Failed to load file: " << shell_paths[shell_type];
    return std::nullopt;
  }

  tinyxml2::XMLElement* root = doc.RootElement();

  if (!root) {
    gzerr << "No root element in SDF.";
    return std::nullopt;
  }
  
  // Convert from XML to string
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}

bool CheatToolsPlugin::change_visual_status(const gz::sim::Entity visual_entity, bool visible){
  gz::msgs::Visual req;

  req.set_visible(visible);
  req.set_type(gz::msgs::Visual::VISUAL);
  req.set_id(visual_entity);

  auto mat = req.mutable_material();
  mat->mutable_diffuse()->set_r(0.4);
  mat->mutable_diffuse()->set_g(0.4);
  mat->mutable_diffuse()->set_b(0.4);
  mat->mutable_diffuse()->set_a(visible? 1.0 : 0.0);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;

  bool executed = gz_node->Request("/world/ariac/visual_config", req, timeout, res, result);

  if (executed){
    if(result && res.data()){
      gzlog << visual_entity << " successfully changed visibility";
    } else {
      gzerr << visual_entity << " could not  change visibility";
    }
  } else {
    gzerr << "Service change visibility for " << visual_entity << " timed out";
  }

  return result;
}