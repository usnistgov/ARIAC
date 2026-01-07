#include "ariac_plugins/welder_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::WelderPlugin,
  gz::sim::System,
  ariac_plugins::WelderPlugin::ISystemConfigure,
  ariac_plugins::WelderPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

WelderPlugin::~WelderPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void WelderPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  model = gz::sim::Model(_entity);

  electrodes_link = gz::sim::Link(model.LinkByName(_ecm, "welder_electrodes"));
  std::string ros_namespace = "";
  if (_sdf->HasElement("ros")){
    if (_sdf->GetElementImpl("ros")->HasElement("namespace")){
      ros_namespace = _sdf->GetElementImpl("ros")->Get<std::string>("namespace");
    }
  }

  // ROS setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared("welder_plugin_node", ros_namespace);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);
  
  std::vector<std::string> topic_names;
  std::string topic = "/world/ariac/model/" + model.Name(_ecm) + "/link/welder_electrodes/sensor/welder_contact_sensor_{n}/contact";

  for(int i = 1; i <= 2; i++){
    std::string name = topic;
    topic_names.push_back(name.replace(topic.find("{n}"), 3, std::to_string(i)));
  }

  electrode_in_contact = {
    {1, BeadContact()},
    {2, BeadContact()}
  };
  
  gz_node = std::make_shared<gz::transport::Node>();
  gz_node->Subscribe(topic_names[0], &WelderPlugin::contact_sensor_1_cb, this);
  gz_node->Subscribe(topic_names[1], &WelderPlugin::contact_sensor_2_cb, this);

  weld_srv = ros_node->create_service<Trigger>(
    "weld",
    std::bind(&WelderPlugin::weld_cb, this, std::placeholders::_1, std::placeholders::_2)
  );

  welded.clear();
}

void WelderPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused){
    return;
  }

  if(light_on && _info.iterations >= light_on_step + 100){
    remove_welder_light(_ecm);
    light_on = false;
  }

  double current_time = _info.simTime.count();

  for(auto &bead: electrode_in_contact){
    if (current_time - bead.second.last_contact_time > 1E8){ // 100 milliseconds
      bead.second = BeadContact();
    }
  }
  
  if(weld_requested){
    std::optional<gz::sim::v8::Entity> shell_entity = _ecm.EntityByName(bead_to_weld.shell_model_name);
      
    if (!shell_entity.has_value()) {
      throw std::runtime_error("Unable to locate model");
    }

    auto shell_model = gz::sim::Model(shell_entity.value());

    auto bead_link = gz::sim::Link(shell_model.LinkByName(_ecm, "bead_"+std::to_string(bead_to_weld.bead_id)+"_link"));

    gz::sim::Entity bead_visual_entity = bead_link.VisualByName(_ecm, "bead_visual_"+std::to_string(bead_to_weld.bead_id));

    bool success = change_visual_status(bead_visual_entity, true);

    gz::math::Pose3d pose(electrodes_link.WorldPose(_ecm).value().Pos(), gz::math::Quaternion(0.0, 0.0, 0.0));
    create_welder_light(_ecm, pose);

    light_on_step = _info.iterations;

    auto module = get_module_component(_ecm, bead_to_weld.shell_model_name);

    if(bead_to_weld.shell_model_name.find("bottom") != std::string::npos){
      module.bottom_welds[bead_to_weld.bead_id] = true;
    } else {
      module.top_welds[bead_to_weld.bead_id] = true;
    }

    _ecm.SetComponentData<gz::sim::components::Module>(module.bottom_shell_entity, module);
    
    weld_requested = false;
  }
}

ariac_components::Module WelderPlugin::get_module_component(
  const gz::sim::EntityComponentManager &_ecm, 
  const std::string shell_in_contact
){
  std::optional<gz::sim::Entity> optional_contact_shell_entity = _ecm.EntityByName(shell_in_contact);
  if (!optional_contact_shell_entity.has_value()){
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Could not find entity for model: " << shell_in_contact);
  }

  gz::sim::Entity contact_shell_entity = optional_contact_shell_entity.value();


  ariac_components::Module module_component;
  _ecm.Each<gz::sim::components::Module>(
    [&](
      const gz::sim::Entity &entity,
      const gz::sim::components::Module *module_state_comp
    ) -> bool{
      if(
        module_state_comp->Data().top_shell_entity == contact_shell_entity || 
        module_state_comp->Data().bottom_shell_entity == contact_shell_entity
      ){
        module_component = module_state_comp->Data();
        return false;
      } 
      return true;
    }
  );

  return module_component;
}

void WelderPlugin::create_welder_light(gz::sim::EntityComponentManager &_ecm, const gz::math::Pose3d &original_pose){
  sdf::Link link;
  link.SetName("emitter_link");

  gz::math::Pose3d pose = original_pose;
  pose.SetZ(pose.Z()+ 0.003);

  gz::math::Color orange(1.0, 0.65, 0.0, 1.0);

  sdf::Light lightSdf;
  lightSdf.SetType(sdf::LightType::POINT);
  lightSdf.SetName("welder_light");
  lightSdf.SetDiffuse({1, 0.9, 0.7, 1});
  lightSdf.SetSpecular({1, 1, 1, 1});
  lightSdf.SetAttenuationRange(1.0);
  lightSdf.SetConstantAttenuationFactor(0.5);
  lightSdf.SetLinearAttenuationFactor(0.01);
  lightSdf.SetQuadraticAttenuationFactor(0.001);
  lightSdf.SetCastShadows(false);

  link.AddLight(lightSdf);
  for (int i = 0; i < 10; ++i)
  {
    sdf::ParticleEmitter emitter;
    emitter.SetName("spark_" + std::to_string(i));
    emitter.SetType(sdf::ParticleEmitterType::POINT);
    emitter.SetSize(gz::math::Vector3d(0.0, 0.0, 0.0));
    emitter.SetRate(300);
    emitter.SetDuration(0.2);
    emitter.SetLifetime(0.05);

    sdf::Material material;
    material.SetAmbient(i%2==0 ? gz::math::Color::Yellow : orange);
    material.SetDiffuse(i%2==0 ? gz::math::Color::Yellow : orange);
    emitter.SetMaterial(material);
    emitter.SetMinVelocity(0.5);
    emitter.SetMaxVelocity(1);
    emitter.SetEmitting(true);
    emitter.SetParticleSize(gz::math::Vector3d(0.001, 0.001, 0.001));

    // Random orientation to simulate spread
    double rx = gz::math::Rand::DblUniform(-0.4, 0.0);
    double ry = gz::math::Rand::DblUniform(-0.4, 0.0);
    double rz = gz::math::Rand::DblUniform(-M_PI, M_PI);
    gz::math::Quaterniond rot(rx, ry, rz);
    emitter.SetRawPose(gz::math::Pose3d(0, 0, 0, rot.Roll(), rot.Pitch(), rot.Yaw()));

    link.AddParticleEmitter(emitter);
  }

  sdf::Model model;
  model.SetName("weld_spark_emitter");
  model.AddLink(link);
  model.SetStatic(true);
  model.SetRawPose(pose);

  sdf::SDF emitterSDF;
  emitterSDF.SetRoot(model.ToElement());

  auto root = emitterSDF.Root();

  std::vector<std::string> vals = {"link", "light"};

  emitterSDF.Root()->GetElementImpl("link")->GetElementImpl("light")->AddElement("visualize")->Set<bool>(false);
  std::string sdfString = emitterSDF.ToString();

  gz::msgs::EntityFactory msg;

  msg.set_sdf(sdfString);

  gz::msgs::Boolean rep;
  bool result;
  unsigned int timeout = 5000;
  bool executed = gz_node->Request("/world/ariac/create", msg, timeout, rep, result);

  if (executed && result && rep.data()) {
    gzdbg << "Requested creation of welder light entity.\n";
  } else {
    gzwarn << "Failed to create welder light entity.\n";
  }

  light_has_visual = true;
  light_on = true;
}

void WelderPlugin::remove_welder_light(gz::sim::EntityComponentManager &_ecm){
  gz::sim::Entity lightEntity = _ecm.EntityByComponents(gz::sim::components::Name("weld_spark_emitter"));
  if (lightEntity != gz::sim::kNullEntity)
  {
    _ecm.RequestRemoveEntity(lightEntity);
  }
}

bool WelderPlugin::change_visual_status(const gz::sim::Entity visual_entity, bool visible){
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
      RCLCPP_INFO_STREAM(ros_node->get_logger(), visual_entity << " successfully changed visibility");
    } else {
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), visual_entity << " could not  change visibility");
    }
  } else {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Service change visibility for " << visual_entity << " timed out");
  }

  return result;
}

void WelderPlugin::weld_cb(const TriggerReqPtr request, TriggerResPtr response){
  if(!electrode_in_contact[1].in_contact && !electrode_in_contact[2].in_contact){
    response->message = "Welder electrodes are not in contact with weld plates";
    response->success = false;
    return;
  }

  bead_to_weld = electrode_in_contact[1].in_contact ? electrode_in_contact[1] : electrode_in_contact[2];

  if(welded[bead_to_weld.shell_model_name][bead_to_weld.bead_id]){
    response->message = "Bead " + std::to_string(bead_to_weld.bead_id) + " has already been completed for this shell";
    response->success = false;
    return;
  }

  welded[bead_to_weld.shell_model_name][bead_to_weld.bead_id] = true;

  weld_requested = true;

  response->message = "Weld successful";
  response->success = true;
}

void WelderPlugin::contact_sensor_1_cb(const gz::msgs::Contacts &_gz_contacts_msg)
{
  electrode_in_contact[1] = check_bead_in_contact(_gz_contacts_msg);
}

void WelderPlugin::contact_sensor_2_cb(const gz::msgs::Contacts &_gz_contacts_msg)
{
  electrode_in_contact[2] = check_bead_in_contact(_gz_contacts_msg);
}

BeadContact WelderPlugin::check_bead_in_contact(const gz::msgs::Contacts &msg){
  BeadContact bead_contact;
  for (int i = 0; i < msg.contact_size(); ++i){
    std::string collision = msg.contact(i).collision2().name();
    if (collision.find("bead") != std::string::npos){
      bead_contact.in_contact = true;
      bead_contact.shell_model_name = collision.substr(0, collision.find("::"));
      bead_contact.bead_id = collision.back() - '0';

      auto gz_time =msg.header().stamp();
      bead_contact.last_contact_time = rclcpp::Time(gz_time.sec(), gz_time.nsec()).nanoseconds();
      return bead_contact;
    }
  }

  return bead_contact;
}