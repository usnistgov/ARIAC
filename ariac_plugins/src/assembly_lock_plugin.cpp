// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math.hh>
#include <ariac_plugins/assembly_lock_plugin.hpp>

#include <map>
#include <memory>

namespace ariac_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class AssemblyLockPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    bool part_attached_;
    bool in_contact_;
    gazebo::physics::LinkPtr assembly_surface_link_;

    gazebo::transport::NodePtr gznode_;
    gazebo::transport::SubscriberPtr contact_sub_;
    gazebo::transport::PublisherPtr attach_pub_;
    gazebo::common::Timer timer_;
    int rate_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr assembly_joint_;
    gazebo::physics::CollisionPtr model_collision_;
    gazebo::physics::ModelPtr model_to_attach_;
    std::map<std::string, gazebo::physics::CollisionPtr> collisions_;

    std::string assembly_part_type_;

    bool CheckModelContact(ConstContactsPtr &);
    void AttachJoint();
    void PublishState();
  };

  AssemblyLock::AssemblyLock()
      : impl_(std::make_unique<AssemblyLockPrivate>())
  {
  }

  AssemblyLock::~AssemblyLock()
  {
  }

  void AssemblyLock::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    impl_->model_ = model;
    gazebo::physics::WorldPtr world = impl_->model_->GetWorld();

    impl_->assembly_joint_ = world->Physics()->CreateJoint("fixed", impl_->model_);
    impl_->assembly_joint_->SetName("assembly_fixed_joints");

    // Initialize a gazebo node and subscribe to the contacts for the assembly surface
    impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

    std::string part_type = sdf->GetElement("assembly_part_type")->Get<std::string>();
    std::string link_name = part_type + "_contact";

    std::string model_name = model->GetName();
    impl_->assembly_surface_link_ = impl_->model_->GetLink(link_name);
    impl_->assembly_part_type_ = part_type;
    impl_->part_attached_ = false;

    std::string topic = "/gazebo/world/" + model_name + "/" + link_name + "/" + part_type + "_contact_sensor/contacts";
    impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &AssemblyLock::OnContact, this);

    std::string attach_topic = "/gazebo/world/" + model_name + "/" + link_name + "/" + part_type + "_attached";
    impl_->rate_ = 5;
    impl_->attach_pub_ = impl_->gznode_->Advertise<gazebo::msgs::Pose>(attach_topic, 100, impl_->rate_);

    impl_->timer_.Reset();
    impl_->timer_.Start();

    // Create a connection so the OnUpdate function is called at every simulation
    // iteration. Remove this call, the connection and the callback if not needed.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&AssemblyLock::OnUpdate, this));
  }

  void AssemblyLock::OnUpdate()
  {
    if (!impl_->part_attached_ && impl_->in_contact_)
    {
      impl_->AttachJoint();
    }

    auto elapsed = impl_->timer_.GetElapsed();
    if (elapsed.Double() > (1. / impl_->rate_))
    {
      impl_->PublishState();
      impl_->timer_.Reset();
      impl_->timer_.Start();
    }
  }

  void AssemblyLock::OnContact(ConstContactsPtr &_msg)
  {
    impl_->in_contact_ = impl_->CheckModelContact(_msg);
  }

  void AssemblyLockPrivate::AttachJoint()
  {
    gzdbg << "Attaching Part" << std::endl;
    assembly_joint_->Load(assembly_surface_link_, model_collision_->GetLink(), ignition::math::Pose3d());
    assembly_joint_->Init();

    part_attached_ = true;
  }

  bool AssemblyLockPrivate::CheckModelContact(ConstContactsPtr &msg)
  {
    std::string part_in_contact;
    int min_contacts = 4;

    for (int i = 0; i < msg->contact_size(); ++i)
    {
      // Find out which contact is the plugin's link
      if (msg->contact(i).collision1().find("insert") != std::string::npos)
      {
        part_in_contact = msg->contact(i).collision2();
      }
      else if (msg->contact(i).collision2().find("insert") != std::string::npos)
      {
        part_in_contact = msg->contact(i).collision1();
      }
      else
      {
        continue;
      }

      // Ignore if part is incorrect type
      if (part_in_contact.find(assembly_part_type_) >= std::string::npos)
      {
        gzdbg << "incorrect part type" << std::endl;
        continue;
      }

      // Check number of contacts
      if (!msg->contact(i).position_size() > min_contacts)
      {
        gzdbg << "not enough contacts" << std::endl;
        continue;
      }
      model_collision_ = boost::dynamic_pointer_cast<gazebo::physics::Collision>(model_->GetWorld()->EntityByName(part_in_contact));
      model_to_attach_ = model_collision_->GetModel();
      return true;
      /* Seems to cause issues for sensor insertion
      //Check normals
      std::vector<bool> aligned;
      for (int j = 0; j < msg->contact(i).normal_size(); ++j){
        ignition::math::Vector3d contact_normal = gazebo::msgs::ConvertIgn(msg->contact(i).normal(j));
        ignition::math::Vector3d assembly_normal = assembly_surface_link_->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0, 0, 1));

        double alignment = assembly_normal.Dot(contact_normal);

        // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Alignment: " << alignment);

        if (std::abs(alignment) > 0.95) {
          aligned.push_back(true);
        }
        else{
          aligned.push_back(false);
        }
      }

      if (std::all_of(aligned.begin(), aligned.end(), [](bool v) { return v; })){
        return true;
      }
    */
    }

    return false;
  }

  void AssemblyLockPrivate::PublishState()
  {
    auto msg = gazebo::msgs::Pose();
    if (part_attached_)
    {
      msg = gazebo::msgs::Convert(model_to_attach_->RelativePose());
    }
    else
    {
      auto blank_pose = ignition::math::Pose3d();
      msg = gazebo::msgs::Convert(blank_pose);
    }
    attach_pub_->Publish(msg);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AssemblyLock)
} // namespace ariac_plugins
