/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>

#include <ariac_plugins/agv_tray_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class AGVTrayPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  bool locked_;
  bool tray_attached_;
  bool sensor_attached_;
  bool in_contact_;
  std::string agv_number_;
  gazebo::physics::LinkPtr agv_tray_link_;

  gazebo::transport::SubscriberPtr contact_sub_;
  gazebo::transport::NodePtr gznode_;



  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr lock_tray_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_tray_service_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr kit_tray_joint_;
  gazebo::physics::JointPtr sensor_joint_;
  gazebo::physics::CollisionPtr model_collision_;
  std::map<std::string, gazebo::physics::CollisionPtr> collisions_;


  bool CheckModelContact(ConstContactsPtr&);
  void AttachJoint();
  void DetachJoint();

  /// Callback for enable service
  void LockTray(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr);

  void UnlockTray(
    std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr);
};

AGVTrayPlugin::AGVTrayPlugin()
: impl_(std::make_unique<AGVTrayPluginPrivate>())
{
}

AGVTrayPlugin::~AGVTrayPlugin()
{
}

void AGVTrayPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->model_ = model;
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->agv_number_ = sdf->GetElement("agv_number")->Get<std::string>();

  impl_->sensor_attached_ = false;

  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  gazebo::physics::WorldPtr world = impl_->model_->GetWorld();
  impl_->kit_tray_joint_ = world->Physics()->CreateJoint("fixed", impl_->model_);
  impl_->kit_tray_joint_->SetName(impl_->agv_number_ + "_kit_tray_joint");

  impl_->sensor_joint_ = world->Physics()->CreateJoint("fixed", impl_->model_);
  impl_->sensor_joint_->SetName(impl_->agv_number_ + "_sensor_joint");

  // Initialize a gazebo node and subscribe to the contacts for the vacuum gripper
  impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

  // Get gripper link
  std::string link_name = sdf->GetElement("agv_tray_link")->Get<std::string>();
  impl_->agv_tray_link_ = impl_->model_->GetLink(link_name);
  
  std::string topic = "/gazebo/world/ariac_robots/" + link_name + "/bumper/contacts";
  impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &AGVTrayPlugin::OnContact, this);

  // Register services
  impl_->lock_tray_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
      "/ariac/" + impl_->agv_number_ + "_lock_tray", 
      std::bind(
      &AGVTrayPluginPrivate::LockTray, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  impl_->unlock_tray_service_ = impl_->ros_node_->create_service<std_srvs::srv::Trigger>(
    "/ariac/" + impl_->agv_number_ + "_unlock_tray", 
    std::bind(
    &AGVTrayPluginPrivate::UnlockTray, impl_.get(),
    std::placeholders::_1, std::placeholders::_2));



  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AGVTrayPlugin::OnUpdate, this));
}

void AGVTrayPlugin::OnUpdate()
{
  // If gripper is enabled and in contact with gripable model attach joint
  if (impl_->locked_ && !impl_->tray_attached_ && impl_->in_contact_) {
    impl_->AttachJoint();
  }

  // If part attached and gripper is disabled remove joint
  if (impl_->tray_attached_ && !impl_->locked_){
    impl_->DetachJoint();
  }

  if (!impl_->sensor_attached_) {
    std::string num(1, impl_->agv_number_.back());
    std::string sensor_name = "agv_tray_sensor_" + num;
    gazebo::physics::ModelPtr sensor;
    sensor = impl_->model_->GetWorld()->ModelByName(sensor_name);
    // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Sensor name: " << sensor_name);
    if (sensor != NULL){
      // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Attaching sensor to agv");

      impl_->sensor_joint_->Load(impl_->agv_tray_link_, sensor->GetLink(), ignition::math::Pose3d());
      impl_->sensor_joint_->Init();

      impl_->sensor_attached_ = true;
    }
  }

}

void AGVTrayPlugin::OnContact(ConstContactsPtr& _msg){
  if (impl_->locked_) {
    impl_->in_contact_ = impl_->CheckModelContact(_msg);
  }
}

void AGVTrayPluginPrivate::AttachJoint(){
  RCLCPP_INFO(ros_node_->get_logger(), "Locking Tray");
  kit_tray_joint_->Load(agv_tray_link_, model_collision_->GetLink(), ignition::math::Pose3d());
  kit_tray_joint_->Init();

  tray_attached_ = true;
}

void AGVTrayPluginPrivate::DetachJoint(){
  RCLCPP_INFO(ros_node_->get_logger(), "Unlocking Tray");
  kit_tray_joint_->Detach();
  tray_attached_ = false;
}

bool AGVTrayPluginPrivate::CheckModelContact(ConstContactsPtr& msg){
  std::string model_in_contact;

  for (int i = 0; i < msg->contact_size(); ++i) {
    // Find out which contact is the agv
    if (msg->contact(i).collision1().find("agv") != std::string::npos) {
      model_in_contact = msg->contact(i).collision2();
    }
    else if (msg->contact(i).collision2().find("agv") != std::string::npos){
      model_in_contact = msg->contact(i).collision1();
    }
    else {
      continue;
    }

    // Check if model is a kit_tray
    if (model_in_contact.find("kit_tray") != std::string::npos){
      model_collision_ = boost::dynamic_pointer_cast<gazebo::physics::Collision>(
        model_->GetWorld()->EntityByName(model_in_contact));
      return true;
    }
  }

  return false;

}

void AGVTrayPluginPrivate::LockTray(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  res->success = false;
  
  if (!locked_) {
    locked_ = true;
    res->success = true;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Tray is already locked");
  }
}

void AGVTrayPluginPrivate::UnlockTray(
  std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  res->success = false;
  
  if (locked_) {
    locked_ = false;
    res->success = true;
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Tray is not yet locked");
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AGVTrayPlugin)
}  // namespace ariac_plugins
