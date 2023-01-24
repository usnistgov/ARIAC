#include <ariac_plugins/gripper_color_plugin.hpp>

#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GripperColorPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::rendering::VisualPtr visual_;

  std::map<std::string, ignition::math::Color> colors_;

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr gripper_state_sub_;

  void OnUpdate();

  void GripperStateCallback(const ariac_msgs::msg::VacuumGripperState::SharedPtr msg_);

};

GripperColorPlugin::GripperColorPlugin()
: impl_(std::make_unique<GripperColorPluginPrivate>())
{
}

GripperColorPlugin::~GripperColorPlugin()
{
}

void GripperColorPlugin::Load(gazebo::rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  std::string name = sdf->GetElement("robot_name")->Get<std::string>();

  impl_->visual_ = visual;

  ignition::math::Color blue;
  blue.R(0.0); blue.G(0.18); blue.B(0.7); blue.A(1.0);
  impl_->colors_.insert({"part_gripper", blue});

  ignition::math::Color purple;
  purple.R(0.4); purple.G(0.0); purple.B(0.4); purple.A(1.0);
  impl_->colors_.insert({"tray_gripper", purple});

  impl_->gripper_state_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/" + name + "_gripper_state", 10, 
    std::bind(&GripperColorPluginPrivate::GripperStateCallback, impl_.get(), std::placeholders::_1));

  // impl_->update_connection_ = gazebo::event::Events::ConnectPreRender(
  //   std::bind(&GripperColorPluginPrivate::OnUpdate, impl_.get()));
}

void GripperColorPluginPrivate::GripperStateCallback(
  const ariac_msgs::msg::VacuumGripperState::SharedPtr msg_) 
{
  ignition::math::Color color = colors_[msg_->type];
  
  visual_->SetDiffuse(color);
  visual_->SetAmbient(color);
  visual_->SetTransparency(0);
}

void GripperColorPluginPrivate::OnUpdate()
{
}


// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GripperColorPlugin)
}  // namespace ariac_plugins
