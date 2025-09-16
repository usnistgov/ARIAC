#include "ariac_plugins/status_light_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::StatusLightPlugin,
  gz::sim::System,
  ariac_plugins::StatusLightPlugin::ISystemConfigure,
  ariac_plugins::StatusLightPlugin::ISystemPreUpdate
)

using namespace ariac_plugins;

StatusLightPlugin::~StatusLightPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void StatusLightPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  light_link = gz::sim::Link(gz::sim::Model(_entity).LinkByName(_ecm, "base_link"));
  // ROS Setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  // Create ROS node
  ros_node = rclcpp::Node::make_shared(gz::sim::Model(_entity).Name(_ecm)+"_status_light_plugin");

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  current_status.competition_state = CompetitionStates::PREPARING;

   // Spin up executor thread
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);
  // executor->add_callback_group(pub_cb_group, ros_node->get_node_base_interface());

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };

  thread_executor_spin = std::thread(spin);

  competition_status_sub = ros_node->create_subscription<CompetitionStatus>(
    "/competition_status", 10, std::bind(&StatusLightPlugin::competition_status_cb, this, std::placeholders::_1)
  );

  gz_node = std::make_shared<gz::transport::Node>();
}

void StatusLightPlugin::PreUpdate(const gz::sim::UpdateInfo &update_info,
      gz::sim::EntityComponentManager &_ecm)
{
  if(update_info.paused){
    return;
  }

  if (light_visuals.size() ==0){
    light_visuals = {
      {CompetitionStates::ENDED, light_link.VisualByName(_ecm, "red_light_visual")},
      {CompetitionStates::ORDERS_COMPLETE, light_link.VisualByName(_ecm, "purple_light_visual")},
      {CompetitionStates::STARTED, light_link.VisualByName(_ecm, "green_light_visual")},
      {CompetitionStates::READY, light_link.VisualByName(_ecm, "blue_light_visual")},
      {CompetitionStates::PREPARING, light_link.VisualByName(_ecm, "yellow_light_visual")}
    };
  }
  if(lit_light != current_status.competition_state){
    update_lights();
  }
}

void StatusLightPlugin::update_lights(){
  if(lit_light != -1){
    change_visual_status(lit_light, false);
  }
  change_visual_status(current_status.competition_state, true);
}

void StatusLightPlugin::change_visual_status(int status, bool visible){
  gz::msgs::Visual req;

  req.set_type(gz::msgs::Visual::VISUAL);
  req.set_id(light_visuals[status]);

  auto mat = req.mutable_material();
  mat->mutable_emissive()->set_r(visible? status_colors[status].R() : 0.0);
  mat->mutable_emissive()->set_g(visible? status_colors[status].G() : 0.0);
  mat->mutable_emissive()->set_b(visible? status_colors[status].B() : 0.0);
  mat->mutable_emissive()->set_a(visible? 1.0 : 0.0);

  mat->mutable_diffuse()->set_r(status_colors[status].R());
  mat->mutable_diffuse()->set_g(status_colors[status].G());
  mat->mutable_diffuse()->set_b(status_colors[status].B());
  mat->mutable_diffuse()->set_a(visible? 1.0 : 0.9);

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;

  bool executed = gz_node->Request("/world/ariac/visual_config", req, timeout, res, result);

  if (executed){
    if(result && res.data()){
      if(visible){
        light_on = true;
        lit_light = status;
      }
    } else {
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), light_visuals[status] << " could not  change visibility");
    }
  } else {
    RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Service change visibility for " << light_visuals[status] << " timed out");
  }
}

void StatusLightPlugin::remove_light(gz::sim::EntityComponentManager &_ecm){
  gz::sim::Entity lightEntity = _ecm.EntityByComponents(gz::sim::components::Name("environment_status_light"));
  if (lightEntity != gz::sim::kNullEntity)
  {
    _ecm.RequestRemoveEntity(lightEntity);
  }
}

void StatusLightPlugin::competition_status_cb(const CompetitionStatus::SharedPtr msg){
  current_status.competition_state = msg->competition_state;
}
