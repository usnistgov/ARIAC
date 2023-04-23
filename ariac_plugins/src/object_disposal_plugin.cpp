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

#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/delete_model.hpp>
#include <ariac_plugins/object_disposal_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <sdf/sdf.hh>

#include <map>
#include <memory>

namespace ariac_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class ObjectDisposalPluginPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;

    gazebo::transport::SubscriberPtr contact_sub_;
    gazebo::transport::NodePtr gznode_;

    std::vector<std::string> part_types_;

    gazebo::physics::ModelPtr model_;

    std::vector<std::string> deleted_models_;

    /// Penalty publisher
    // bool publish_penalty_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr penalty_pub_;

    /// Service to delete model
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_entity_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteModel>::SharedPtr delete_model_client_;
  };

  ObjectDisposalPlugin::ObjectDisposalPlugin()
      : impl_(std::make_unique<ObjectDisposalPluginPrivate>())
  {
  }

  ObjectDisposalPlugin::~ObjectDisposalPlugin()
  {
  }

  void ObjectDisposalPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // Create a GazeboRos node instead of a common ROS node.
    // Pass it SDF parameters so common options like namespace and remapping
    // can be handled.
    impl_->model_ = model;

    // Put ros node into namespace based on model name to avoid collisions
    std::string model_name = model->GetName();
    sdf::ElementPtr ns(new sdf::Element);
    sdf->SetParent(sdf->FindElement("ros"));
    ns->SetName("namespace");
    ns->AddValue("string", model_name, "1");
    sdf->FindElement("ros")->InsertElement(ns);

    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    impl_->part_types_ = {"battery", "regulator", "pump", "sensor"};

    impl_->deleted_models_ = {};

    // Initialize a gazebo node and subscribe to the contacts for the vacuum gripper
    impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

    // Get contact topic
    std::string link_name = sdf->GetElement("link_name")->Get<std::string>();
    // impl_->publish_penalty_ = sdf->GetElement("publish_penalty")->Get<bool>();

    std::string topic = "/gazebo/world/" + model_name + "/" + link_name + "/bumper/contacts";

    // Register client
    impl_->delete_entity_client_ = impl_->ros_node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    impl_->delete_model_client_ = impl_->ros_node_->create_client<gazebo_msgs::srv::DeleteModel>("/delete_model");

    // Create penalty publisher
    // impl_->penalty_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::String>("/ariac/penalty", 10);

    impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &ObjectDisposalPlugin::OnContact, this);
  }

  void ObjectDisposalPlugin::OnContact(ConstContactsPtr &_msg)
  {
    using namespace std::chrono_literals;
    std::string model_in_contact;

    for (int i = 0; i < _msg->contact_size(); ++i)
    {
      // Find out which model is in contact with the floor
      if (_msg->contact(i).collision1().find(impl_->model_->GetName()) != std::string::npos)
      {
        model_in_contact = _msg->contact(i).collision2();
      }
      else if (_msg->contact(i).collision2().find(impl_->model_->GetName()) != std::string::npos)
      {
        model_in_contact = _msg->contact(i).collision1();
      }
      else
      {
        continue;
      }
    }

    if (model_in_contact.empty())
      return;

    auto model_name = model_in_contact.substr(0, model_in_contact.find("::"));
    // Check model was not already deleted
    if (std::find(impl_->deleted_models_.begin(), impl_->deleted_models_.end(), model_name) == impl_->deleted_models_.end())
    {
      if (!model_name.empty())
        // std::cout << "model: " << model_name << std::endl;
        // check model is a part
        for (std::string &type : impl_->part_types_)
        {
          if (model_name.find(type) != std::string::npos)
          {
            RCLCPP_WARN(impl_->ros_node_->get_logger(), "Part %s in contact with floor, teleporting", model_name.c_str());
           
            // if (!impl_->delete_entity_client_->wait_for_service(5s))
            // {
            //   RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Unable to find localize_part service. Start vision_node first.");
            //   return;
            // }

            // impl_->model_->SetWorldPose({-20, 13, 0.5, 0, 0, 0}, true, true);
            impl_->model_->SetAutoDisable(true);
            impl_->model_->SetCollideMode("none");
            impl_->deleted_models_.push_back(model_name);

            // auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
            // request->name = model_name;
            // auto future = impl_->delete_entity_client_->async_send_request(request);

            // if (rclcpp::spin_until_future_complete(impl_->ros_node_->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
            // {
            //   RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Failed to receive LocalizePart service response");
            //   return;
            // }

            // auto response = future.get();
            // if (!response->success)
            // {
            //   RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Part deletion failed");
            //   return;
            // }

            // impl_->deleted_models_.push_back(model_name);

            // RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "Response: "<< response->success);
            break;
          }
        }
    }

    // Get model name
    // auto model_name = model_in_contact.substr(0, model_in_contact.find("::"));
    // // Check model was not already deleted
    // if (std::find(impl_->deleted_models_.begin(), impl_->deleted_models_.end(), model_name) == impl_->deleted_models_.end())
    // {
    //   if (!model_name.empty())
    //     std::cout << "part_name: " << model_name << std::endl;
    //   // check model is a part
    //   for (std::string &type : impl_->part_types_)
    //   {
    //     if (model_name.find(type) != std::string::npos)
    //     {

    //       auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    //       request->name = model_name;
    //       auto future = impl_->client_->async_send_request(request);
    //       // auto status = future.wait_for(2s);
    //       auto response = future.get();
    //       std::cout << "Response: " << response->success << std::endl;
    //       impl_->deleted_models_.push_back(model_name);

    //       // if (std::future_status::ready)
    //       // {
    //       //   auto response = future.get();
    //       //   std::cout << "Response: " << response->success << std::endl;
    //       //   impl_->deleted_models_.push_back(model_name);
    //       // }
    //       // else
    //       // {
    //       //   std::cout << "Service call timed out" << std::endl;
    //       // }
    //       break;
    //     }
    //   }
    // }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)
} // namespace ariac_plugins
