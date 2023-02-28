/*
This software was developed by employees of the National Institute of Standards and Technology (NIST), an agency of the Federal Government. Pursuant to title 17 United States Code Section 105, works of NIST employees are not subject to copyright protection in the United States and are considered to be in the public domain. Permission to freely use, copy, modify, and distribute this software and its documentation without fee is hereby granted, provided that this notice and disclaimer of warranty appears in all copies.

The software is provided 'as is' without any warranty of any kind, either expressed, implied, or statutory, including, but not limited to, any warranty that the software will conform to specifications, any implied warranties of merchantability, fitness for a particular purpose, and freedom from infringement, and any warranty that the documentation will conform to the software, or any warranty that the software will be error free. In no event shall NIST be liable for any damages, including, but not limited to, direct, indirect, special or consequential damages, arising out of, resulting from, or in any way connected with this software, whether or not based upon warranty, contract, tort, or otherwise, whether or not injury was sustained by persons or property or otherwise, and whether or not loss was sustained from, or arose out of the results of, or use of, the software or services provided hereunder.

Distributions of NIST software should also include copyright and licensing statements of any third-party software that are legally bundled with the code in compliance with the conditions of those licenses.
*/

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <ariac_plugins/agv_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/msg/agv_status.hpp>

#include <memory>

namespace ariac_plugins
{
/// Class to hold private data members (PIMPL pattern)
class AGVPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr agv_joint_;

  std::string agv_number_;
  double kitting_location_ = 0.0;
  double front_assembly_station_ = 5.6;
  double back_assembly_station_ = 10.6;
  double warehouse_location_ = 17;
  double max_velocity_ = 2.0;
  double goal_position_;

  // Velocity publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  std_msgs::msg::Float64MultiArray velocity_msg_;

  // AGV Status publisher
  rclcpp::Publisher<ariac_msgs::msg::AGVStatus>::SharedPtr status_pub_;
  ariac_msgs::msg::AGVStatus status_msg_;

  rclcpp::Time last_publish_time_;
  int update_ns_;
  bool first_publish_;

  // Move service
  rclcpp::Service<ariac_msgs::srv::MoveAGV>::SharedPtr move_service_;

  void PublishVelocity(double vel);
  void MoveAGV( 
    ariac_msgs::srv::MoveAGV::Request::SharedPtr req,
    ariac_msgs::srv::MoveAGV::Response::SharedPtr res);
  void MoveToGoal(double goal);
  void PublishStatus();
  void OnUpdate();

};

AGVPlugin::AGVPlugin()
: impl_(std::make_unique<AGVPluginPrivate>())
{
}

AGVPlugin::~AGVPlugin()
{
}

void AGVPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->model_ = model;

  impl_->agv_number_ = sdf->GetElement("agv_number")->Get<std::string>();

  impl_->agv_joint_ = model->GetJoint(impl_->agv_number_ + "_joint");
  if (!impl_->agv_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "AGV joint not found, unable to start agv plugin");
    return;
  }

  // Register velocity publisher
  std::string vel_topic = "/" + impl_->agv_number_ + "_controller/commands";
  impl_->velocity_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(vel_topic, 10);

  // Register status publisher
  std::string status_topic = "/ariac/" + impl_->agv_number_ + "_status";
  impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::AGVStatus>(status_topic, 10);

  double publish_rate = 10;
  impl_->update_ns_ = int((1/publish_rate) * 1e9);
  impl_->first_publish_ = true;

  // Register move service
  impl_->move_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::MoveAGV>(
      "/ariac/move_"+impl_->agv_number_, 
      std::bind(
      &AGVPluginPrivate::MoveAGV, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AGVPluginPrivate::OnUpdate, impl_.get()));
}

void AGVPluginPrivate::OnUpdate()
{
  // Publish status at rate
  rclcpp::Time now = ros_node_->get_clock()->now();
  if (first_publish_) {
    PublishStatus();
    last_publish_time_ = now;
    first_publish_ = false;
  } else if (now - last_publish_time_ >= rclcpp::Duration(0, update_ns_)) {
    PublishStatus();
    last_publish_time_ = now;
  }
}


void AGVPluginPrivate::PublishVelocity(double vel)
{
  // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Publishing velocity: " << vel);
  velocity_msg_.data.clear();
  velocity_msg_.data.push_back(vel);
  velocity_pub_->publish(velocity_msg_);
}


void AGVPluginPrivate::MoveAGV(
  ariac_msgs::srv::MoveAGV::Request::SharedPtr req,
  ariac_msgs::srv::MoveAGV::Response::SharedPtr res)
{
  res->success = false;

  if (req->location == ariac_msgs::srv::MoveAGV::Request::KITTING){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to kitting station");

    this->MoveToGoal(kitting_location_);    

    res->success = true;
    return;
  }
  else if (req->location == ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to front assembly station");

    this->MoveToGoal(front_assembly_station_);   

    res->success = true;
    return;
  }
  else if (req->location == ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " to back assembly station");

    this->MoveToGoal(back_assembly_station_);   

    res->success = true;
    return;
  }
  else if (req->location == ariac_msgs::srv::MoveAGV::Request::WAREHOUSE){
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Moving " << agv_number_ << " warehouse");

    this->MoveToGoal(warehouse_location_);   

    res->success = true;
    return;
  }

  else {
    RCLCPP_WARN(ros_node_->get_logger(), "Location not recognized");
  }
  
}

void AGVPluginPrivate::MoveToGoal(double goal){
  int direction = 1;
  if (agv_joint_->Position(0) > goal)
  {
    direction = -1;
  }
  
  double velocity = 0;
  double acceleration = 2;
  double stop_distance = 0.9;
  rclcpp::Time start = ros_node_->get_clock()->now();
  if (direction > 0){
    while(agv_joint_->Position(0) < goal - stop_distance){
      if (velocity >= max_velocity_){
        velocity = max_velocity_;
      } else {
        rclcpp::Duration dur = ros_node_->get_clock()->now() - start;
        double time = dur.nanoseconds() / 1e9;
        velocity = time * acceleration;
      }
      this->PublishVelocity(velocity);
    }

    double maximum_velocity = velocity;

    start = ros_node_->get_clock()->now();
    while(agv_joint_->Position(0) < goal){
      if (velocity <= 0){
        break;
      } else {
        rclcpp::Duration dur = ros_node_->get_clock()->now() - start;
        double time = dur.nanoseconds() / 1e9;
        velocity = maximum_velocity - (time * acceleration);
      }
      this->PublishVelocity(velocity);
    }

    this->PublishVelocity(0.0);
  }
  else {
    while(agv_joint_->Position(0) > goal + stop_distance){
      if (std::abs(velocity) >= max_velocity_){
        velocity = -max_velocity_;
      } else {
        rclcpp::Duration dur = ros_node_->get_clock()->now() - start;
        double time = dur.nanoseconds() / 1e9;
        velocity = time * -acceleration;
      }
      this->PublishVelocity(velocity);
    }

    double maximum_velocity = velocity;

    start = ros_node_->get_clock()->now();
    while(agv_joint_->Position(0) > goal){
      if (velocity >= 0){
        break;
      } else {
        rclcpp::Duration dur = ros_node_->get_clock()->now() - start;
        double time = dur.nanoseconds() / 1e9;
        velocity = maximum_velocity + (time * acceleration);
      }
      this->PublishVelocity(velocity);
    }

    this->PublishVelocity(0.0);
  }
}

void AGVPluginPrivate::PublishStatus(){
  double error_margin = 0.05; // m

  if (std::abs(agv_joint_->Position(0) - kitting_location_) <= error_margin)
    status_msg_.location = ariac_msgs::msg::AGVStatus::KITTING;
  else if (std::abs(agv_joint_->Position(0) - front_assembly_station_) <= error_margin)
    status_msg_.location = ariac_msgs::msg::AGVStatus::ASSEMBLY_FRONT;
  else if (std::abs(agv_joint_->Position(0) - back_assembly_station_) <= error_margin)
    status_msg_.location = ariac_msgs::msg::AGVStatus::ASSEMBLY_BACK;
  else if (std::abs(agv_joint_->Position(0) - warehouse_location_) <= error_margin)
    status_msg_.location = ariac_msgs::msg::AGVStatus::WAREHOUSE;
  else
    status_msg_.location = ariac_msgs::msg::AGVStatus::UNKNOWN;

  status_msg_.position = agv_joint_->Position(0);
  status_msg_.velocity = agv_joint_->GetVelocity(0);

  status_pub_->publish(status_msg_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AGVPlugin)
}  // namespace ariac_plugins
