#include "ariac_plugins/tf_publisher_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::TFPublisherPlugin,
  gz::sim::System,
  ariac_plugins::TFPublisherPlugin::ISystemConfigure,
  ariac_plugins::TFPublisherPlugin::ISystemPostUpdate
)

using namespace ariac_plugins;

TFPublisherPlugin::~TFPublisherPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}


void TFPublisherPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{

  if (_sdf->HasElement("sensor")){
    sensor_status = SensorStatus::WAIT_NEEDED;
  }

  // GZ Setup
  model = gz::sim::Model(_entity);
  base_link = gz::sim::Link(model.LinkByName(_ecm, _sdf->Get<std::string>("base_link")));

  // ROS Setup
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }

  // Create ROS node
  ros_node = rclcpp::Node::make_shared(model.Name(_ecm) + "_tf_plugin");

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

  // Construct broadcasters
  static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(ros_node);
  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node);

  is_static = (_sdf->HasElement("static")) ? _sdf->Get<bool>("static") : false;

  // Build Base Transform
  base_frame_name = model.Name(_ecm) + "_frame";

  base_link_transform = gz_pose_to_transform(base_link.WorldPose(_ecm).value(), "world", base_frame_name);

  // Build child transforms
  sdf::ElementConstPtr childElem = _sdf->FindElement("child");

  while(childElem){
    std::string link = childElem->GetAttribute("link")->GetAsString();
    std::string child_frame = model.Name(_ecm) + "_" + childElem->GetAttribute("frame")->GetAsString();

    std::string parent_frame;
    gz::math::Pose3d pose;
    if (childElem->GetAttribute("parent_frame")) {
      parent_frame = model.Name(_ecm) + "_" + childElem->GetAttribute("parent_frame")->GetAsString();

      auto child_link = gz::sim::Link(model.LinkByName(_ecm, link));
      auto parent_link = gz::sim::Link(model.LinkByName(_ecm, childElem->GetAttribute("parent_link")->GetAsString()));

      if(parent_link.WorldPose(_ecm).has_value() && child_link.WorldPose(_ecm).has_value()) {
        auto parent_pose = parent_link.WorldPose(_ecm).value();
        auto child_pose = child_link.WorldPose(_ecm).value();

        pose = parent_pose.Inverse() * child_pose;
      }

    } else {
      parent_frame = base_frame_name;
      pose = _ecm.Component<gz::sim::components::Pose>(model.LinkByName(_ecm, link))->Data();
    }

    child_transforms.push_back(gz_pose_to_transform(pose, parent_frame, child_frame));

    childElem = childElem->GetNextElement("child");
  }
  
  if(is_static && sensor_status == SensorStatus::NOT_SENSOR){
    static_tf_broadcaster->sendTransform(base_link_transform);
  } else {
    tf_broadcaster->sendTransform(base_link_transform);
  }

  for(auto t : child_transforms){
    static_tf_broadcaster->sendTransform(t);
  }
}

void TFPublisherPlugin::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused){
    return;
  }

  switch(sensor_status){
    case SensorStatus::NOT_SENSOR:
      if(is_static){ return; }
      base_link_transform = gz_pose_to_transform(base_link.WorldPose(_ecm).value(), "world", base_frame_name);
      tf_broadcaster->sendTransform(base_link_transform);
      return;
    case SensorStatus::WAIT_NEEDED:
      sensor_status = SensorStatus::READY_TO_SET;
      return;
    case SensorStatus::READY_TO_SET:
      base_link_transform = gz_pose_to_transform(base_link.WorldPose(_ecm).value(), "world", base_frame_name);
      static_tf_broadcaster->sendTransform(base_link_transform);
      sensor_status = SensorStatus::SET;
      return;
    case SensorStatus::SET:
      return;
    default:
      RCLCPP_ERROR_STREAM(ros_node->get_logger(), "Invalid sensor status passed into tf publisher plugin post-update.");
      return;
  }
  
}

TransformStamped TFPublisherPlugin::gz_pose_to_transform(
  gz::math::Pose3d gz_pose, 
  std::string parent_frame,
  std::string child_frame)
{
  TransformStamped t;
  
  t.header.frame_id = parent_frame;
  t.header.stamp = ros_node->get_clock()->now();

  t.child_frame_id = child_frame;

  t.transform.translation.x = gz_pose.Pos().X();
  t.transform.translation.y = gz_pose.Pos().Y();
  t.transform.translation.z = gz_pose.Pos().Z();

  t.transform.rotation.w = gz_pose.Rot().W();
  t.transform.rotation.x = gz_pose.Rot().X();
  t.transform.rotation.y = gz_pose.Rot().Y();
  t.transform.rotation.z = gz_pose.Rot().Z();

  return t;
}
