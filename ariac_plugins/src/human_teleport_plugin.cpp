#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/human_teleport_plugin.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class HumanTeleportPluginPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::ModelPtr model_;  
  gazebo::physics::WorldPtr world_;

  ignition::math::Pose3d home_pose;

  // Teleport service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr teleport_service_;

  void TeleportHuman(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

};

HumanTeleportPlugin::HumanTeleportPlugin()
: impl_(std::make_unique<HumanTeleportPluginPrivate>())
{
}

HumanTeleportPlugin::~HumanTeleportPlugin()
{
}

void HumanTeleportPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->model_ = model;

  impl_->home_pose = model->WorldPose();

  impl_->teleport_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
    "/ariac_human/teleport",
    std::bind(
      &HumanTeleportPluginPrivate::TeleportHuman, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

void HumanTeleportPluginPrivate::TeleportHuman(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res) 
{
  res->success = false;
  
  RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Teleporting human to orign");

  model_->SetWorldPose(home_pose);

  res->success = true;
  return;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HumanTeleportPlugin)
}  // namespace ariac_plugins
