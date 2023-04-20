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

#include <ariac_plugins/object_disposal_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

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
    bool publish_penalty_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr penalty_pub_;

    /// Service to delete model
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_model_client_;
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
    impl_->publish_penalty_ = sdf->GetElement("publish_penalty")->Get<bool>();

    std::string topic = "/gazebo/world/" + model_name + "/" + link_name + "/bumper/contacts";

    // Register client
    impl_->delete_model_client_ = impl_->ros_node_->create_client<gazebo_msgs::srv::DeleteEntity>(
        "/delete_entity");

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
      // Find out which contact is the agv
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

      // Check if model is a part
      for (std::string &type : impl_->part_types_)
      {
        std::cout << "model_in_contact: " << model_in_contact << std::endl;
        if (model_in_contact.find(type) != std::string::npos)
        {
          auto part_name = model_in_contact.substr(0, model_in_contact.find("::"));

          if (std::find(impl_->deleted_models_.begin(), impl_->deleted_models_.end(), part_name) == impl_->deleted_models_.end())
          {
            impl_->deleted_models_.push_back(part_name);
            gazebo_msgs::srv::DeleteEntity::Request::SharedPtr request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
            request->name = part_name;
            std::cout << "Request name: " << request->name << std::endl;

            // if (impl_->publish_penalty_) {
            //   std_msgs::msg::String msg;
            //   msg.data = request->name;
            //   impl_->penalty_pub_->publish(msg);
            // }
            if (impl_->delete_model_client_->wait_for_service(2s))
            {

              using ResponseFutT = decltype(impl_->delete_model_client_)::element_type::SharedFuture;
              std::cout << "Before service call" << std::endl;
              impl_->delete_model_client_->async_send_request(request, [this](ResponseFutT fut){
              if (fut.valid() && fut.get()->success){
                std::cout << "------- Deleted model"<< std::endl;
              }
              else{
                std::cout << "Deletion request in gazebo failed" << std::endl;
                } });
              std::cout << "After service call" << std::endl;
            }
            else
            {
              std::cout << "----- Did not find service" << impl_->delete_model_client_->get_service_name() << ", skipping" << std::endl;
            }

              // std::cout << "Before deleting model: " << request->name << std::endl;
              // impl_->delete_model_client_->async_send_request(request);
              // std::cout << "After deleting model: " << request->name << std::endl;
              break;
            }
        }
      }
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)
} // namespace ariac_plugins
