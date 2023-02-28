/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>
#include <ariac_plugins/assembly_state_publisher.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/msgs.hh>

#include <map>
#include <memory>

namespace ariac_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class AssemblyStatePublisherPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<ariac_msgs::msg::AssemblyState>::SharedPtr status_pub_;
    ariac_msgs::msg::AssemblyState status_msg_;

    gazebo::physics::ModelPtr model_;
    bool battery_attached_;
    bool pump_attached_;
    bool regulator_attached_;
    bool sensor_attached_;

    gazebo::transport::NodePtr gznode_;
    gazebo::transport::SubscriberPtr battery_sub_;
    gazebo::transport::SubscriberPtr pump_sub_;
    gazebo::transport::SubscriberPtr regulator_sub_;
    gazebo::transport::SubscriberPtr sensor_sub_;

    gazebo::common::Timer timer_;
    int rate_;

    void PublishState();
  };

  AssemblyStatePublisher::AssemblyStatePublisher()
      : impl_(std::make_unique<AssemblyStatePublisherPrivate>())
  {
  }

  AssemblyStatePublisher::~AssemblyStatePublisher()
  {
  }

  void AssemblyStatePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    impl_->model_ = model;
    std::string model_name = model->GetName();
    std::string link_name = "insert_link";

    /* Causes AGV controllers to fail to load? */
    sdf::ElementPtr ns(new sdf::Element);
    sdf->SetParent(sdf->FindElement("ros"));
    ns->SetName("namespace");
    ns->AddValue("string", model_name, "1");
    sdf->FindElement("ros")->InsertElement(ns);

    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    std::string bat_topic = "/gazebo/world/" + model_name + "/battery_contact/battery_attached";
    std::string pum_topic = "/gazebo/world/" + model_name + "/pump_contact/pump_attached";
    std::string reg_topic = "/gazebo/world/" + model_name + "/regulator_contact/regulator_attached";
    std::string sen_topic = "/gazebo/world/" + model_name + "/sensor_contact/sensor_attached";

    // Initialize a gazebo node and subscribe to each assembly topic
    impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

    impl_->battery_sub_ = impl_->gznode_->Subscribe(bat_topic, &AssemblyStatePublisher::UpdateBattery, this);
    impl_->pump_sub_ = impl_->gznode_->Subscribe(pum_topic, &AssemblyStatePublisher::UpdatePump, this);
    impl_->regulator_sub_ = impl_->gznode_->Subscribe(reg_topic, &AssemblyStatePublisher::UpdateRegulator, this);
    impl_->sensor_sub_ = impl_->gznode_->Subscribe(sen_topic, &AssemblyStatePublisher::UpdateSensor, this);

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());
    impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::AssemblyState>("/ariac/" + model_name + "_assembly_state", pub_qos);

    impl_->rate_ = 30;

    impl_->battery_attached_ = false;
    impl_->pump_attached_ = false;
    impl_->regulator_attached_ = false;
    impl_->sensor_attached_ = false;

    impl_->timer_.Reset();
    impl_->timer_.Start();

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&AssemblyStatePublisher::OnUpdate, this));
  }

  void AssemblyStatePublisher::OnUpdate()
  {
    auto elapsed = impl_->timer_.GetElapsed();
    if (elapsed.Double() > (1. / impl_->rate_))
    {
      impl_->PublishState();
      impl_->timer_.Reset();
      impl_->timer_.Start();
    }
  }

  void AssemblyStatePublisher::UpdateBattery(ConstPosePtr &_msg)
  {
    auto pose = gazebo::msgs::ConvertIgn(*_msg);
    if (pose == ignition::math::Pose3d())
    {
      impl_->battery_attached_ = false;
    }
    else
    {
      impl_->battery_attached_ = true;
    }
    return;
  }
  void AssemblyStatePublisher::UpdatePump(ConstPosePtr &_msg)
  {
    // gzerr << _msg->DebugString() << std::endl;
    auto pose = gazebo::msgs::ConvertIgn(*_msg);
    if (pose == ignition::math::Pose3d())
    {
      impl_->pump_attached_ = false;
    }
    else
    {
      impl_->pump_attached_ = true;
    }
    return;
  }
  void AssemblyStatePublisher::UpdateRegulator(ConstPosePtr &_msg)
  {
    auto pose = gazebo::msgs::ConvertIgn(*_msg);
    if (pose == ignition::math::Pose3d())
    {
      impl_->regulator_attached_ = false;
    }
    else
    {
      impl_->regulator_attached_ = true;
    }
    return;
  }
  void AssemblyStatePublisher::UpdateSensor(ConstPosePtr &_msg)
  {
    auto pose = gazebo::msgs::ConvertIgn(*_msg);
    if (pose == ignition::math::Pose3d())
    {
      impl_->sensor_attached_ = false;
    }
    else
    {
      impl_->sensor_attached_ = true;
    }
    return;
  }

  void AssemblyStatePublisherPrivate::PublishState()
  {
    status_msg_.battery_attached = battery_attached_;
    status_msg_.pump_attached = pump_attached_;
    status_msg_.regulator_attached = regulator_attached_;
    status_msg_.sensor_attached = sensor_attached_;
    status_pub_->publish(status_msg_);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AssemblyStatePublisher)
} // namespace ariac_plugins
