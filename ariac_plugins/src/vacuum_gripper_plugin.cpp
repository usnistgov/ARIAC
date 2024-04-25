#include <ariac_plugins/vacuum_gripper_plugin.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/change_gripper.hpp>

#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/trial.hpp>

#include <ariac_plugins/ariac_common.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <memory>

namespace ariac_plugins
{
  /// Class to hold private data members (PIMPL pattern)
  class VacuumGripperPluginPrivate
  {
  public:
    /// Connection to world update event. Callback is called while this is alive.
    gazebo::event::ConnectionPtr update_connection_;

    /// Node for ROS communication.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<ariac_msgs::msg::VacuumGripperState>::SharedPtr status_pub_;
    ariac_msgs::msg::VacuumGripperState status_msg_;

    std::string robot_name_;
    bool enabled_;
    bool model_attached_;
    bool in_contact_with_part_;
    bool in_contact_with_tray_;

    gazebo::physics::LinkPtr gripper_link_;

    gazebo::transport::SubscriberPtr contact_sub_;
    gazebo::transport::NodePtr gznode_;

    /// Service for enabling the vacuum gripper
    rclcpp::Service<ariac_msgs::srv::VacuumGripperControl>::SharedPtr enable_service_;

    rclcpp::Service<ariac_msgs::srv::ChangeGripper>::SharedPtr change_service_;

    std::map<int, std::string> gripper_types_;
    int current_gripper_type_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr picked_part_joint_;
    gazebo::physics::CollisionPtr model_collision_;
    std::map<std::string, gazebo::physics::CollisionPtr> collisions_;

    std::vector<std::string> pickable_part_types_ = {"battery", "regulator", "pump", "sensor"};
    std::vector<std::string> pickable_part_colors_ = {"red", "orange", "green", "blue", "purple"};

    rclcpp::Time last_publish_time_;
    int update_ns_;
    bool first_publish_;

    // Drop challenges
    std::vector<ariac_msgs::msg::DroppedPartChallenge> drop_challenges_;
    std::string part_in_contact_;
    std::string attached_part_;
    std::map<std::string, int> pick_counts_;
    rclcpp::Time drop_time_;
    bool drop_challenge_active_ = false;

    // Subscriptions
    rclcpp::Subscription<ariac_msgs::msg::Trial>::SharedPtr trial_config_sub_;

    void OnUpdate();

    void OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg);

    void OnContact(ConstContactsPtr &_msg);

    bool CheckModelContact(ConstContactsPtr &, std::string);
    void AttachJoint();
    void DetachJoint();
    void PublishState();

    void HandleDropChallenge();

    double Distance(geometry_msgs::msg::Point, geometry_msgs::msg::Point);

    /// Callback for enable service
    void EnableGripper(
        ariac_msgs::srv::VacuumGripperControl::Request::SharedPtr,
        ariac_msgs::srv::VacuumGripperControl::Response::SharedPtr);

    /// Callback for gripper change service
    void ChangeGripper(
        ariac_msgs::srv::ChangeGripper::Request::SharedPtr,
        ariac_msgs::srv::ChangeGripper::Response::SharedPtr);
  };

  VacuumGripperPlugin::VacuumGripperPlugin()
      : impl_(std::make_unique<VacuumGripperPluginPrivate>())
  {
  }

  VacuumGripperPlugin::~VacuumGripperPlugin()
  {
  }

  void VacuumGripperPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    impl_->model_ = model;
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    impl_->robot_name_ = sdf->GetElement("robot_name")->Get<std::string>();

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());
    impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::VacuumGripperState>("/ariac/" + impl_->robot_name_ + "_gripper_state", pub_qos);

    gazebo::physics::WorldPtr world = impl_->model_->GetWorld();
    impl_->picked_part_joint_ = world->Physics()->CreateJoint("fixed", impl_->model_);
    impl_->picked_part_joint_->SetName("picked_part_fixed_joints");

    // Initialize a gazebo node and subscribe to the contacts for the vacuum gripper
    impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gznode_->Init(impl_->model_->GetWorld()->Name());

    // Get gripper link
    std::string link_name = sdf->GetElement("gripper_link")->Get<std::string>();
    impl_->gripper_link_ = impl_->model_->GetLink(link_name);

    std::string topic = "/gazebo/world/ariac_robots/" + link_name + "/bumper/contacts";
    impl_->contact_sub_ = impl_->gznode_->Subscribe(topic, &VacuumGripperPluginPrivate::OnContact, impl_.get());

    impl_->gripper_types_ = {
        {ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER, "part_gripper"},
        {ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER, "tray_gripper"},
    };

    impl_->current_gripper_type_ = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;

    double publish_rate = 10;
    impl_->update_ns_ = int((1 / publish_rate) * 1e9);
    impl_->first_publish_ = true;

    // Connect Subscribers
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();  

    impl_->trial_config_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Trial>(  
        "/ariac/trial_config", qos.get_subscription_qos("/ariac/trial_config", qos_profile),  
        std::bind(&VacuumGripperPluginPrivate::OnTrialCallback, impl_.get(), std::placeholders::_1));

    // Register enable service
    impl_->enable_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::VacuumGripperControl>(
        "/ariac/" + impl_->robot_name_ + "_enable_gripper",
        std::bind(
            &VacuumGripperPluginPrivate::EnableGripper, impl_.get(),
            std::placeholders::_1, std::placeholders::_2));

    // Register change service
    impl_->change_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::ChangeGripper>(
        "/ariac/" + impl_->robot_name_ + "_change_gripper",
        std::bind(
            &VacuumGripperPluginPrivate::ChangeGripper, impl_.get(),
            std::placeholders::_1, std::placeholders::_2));

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&VacuumGripperPluginPrivate::OnUpdate, impl_.get()));
  }

  void VacuumGripperPluginPrivate::OnTrialCallback(const ariac_msgs::msg::Trial::SharedPtr _msg)
  {
    if (_msg->challenges.size() > 0)
    {
      for (auto challenge : _msg->challenges)
      {
        if (challenge.type == ariac_msgs::msg::Challenge::DROPPED_PART)
        {
          drop_challenges_.push_back(challenge.dropped_part_challenge);
        }
      }
    }
  }

  void VacuumGripperPluginPrivate::OnUpdate()
  {
    if (current_gripper_type_ == ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER)
    {
      // If gripper is enabled and in contact with gripable model attach joint
      if (enabled_ && !model_attached_ && in_contact_with_part_)
      {
        AttachJoint();
      }
    }
    else if (current_gripper_type_ == ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER)
    {
      // If gripper is enabled and in contact with tray attach joint
      if (enabled_ && !model_attached_ && in_contact_with_tray_)
      {
        AttachJoint();
      }
    }

    // If model attached and gripper is disabled remove joint
    if (model_attached_ && !enabled_)
    {
      DetachJoint();
    }

    // Check drop part challenge
    if (drop_challenge_active_)
    {
      if (ros_node_->now() >= drop_time_)
      {
        DetachJoint();
        drop_challenge_active_ = false;
      }
    }

    // Publish status at rate
    rclcpp::Time now = ros_node_->get_clock()->now();
    if (first_publish_)
    {
      PublishState();
      last_publish_time_ = now;
      first_publish_ = false;
    }
    else if (now - last_publish_time_ >= rclcpp::Duration(0, update_ns_))
    {
      PublishState();
      last_publish_time_ = now;
    }
  }

  void VacuumGripperPluginPrivate::OnContact(ConstContactsPtr &_msg)
  {
    if (enabled_)
    {
      in_contact_with_part_ = CheckModelContact(_msg, "part");
      in_contact_with_tray_ = CheckModelContact(_msg, "tray");
    }
  }

  void VacuumGripperPluginPrivate::AttachJoint()
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Picking object");
    picked_part_joint_->Load(gripper_link_, model_collision_->GetLink(), ignition::math::Pose3d());
    picked_part_joint_->Init();

    model_attached_ = true;

    // Update pick counts map
    if (current_gripper_type_ == ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER)
    {
      if (pick_counts_.find(part_in_contact_) == pick_counts_.end())
      {
        pick_counts_[part_in_contact_] = 1;
      }
      else
      {
        pick_counts_[part_in_contact_]++;
      }

      attached_part_ = part_in_contact_;

      HandleDropChallenge();
    }
  }

  void VacuumGripperPluginPrivate::HandleDropChallenge()
  {
    for (auto challenge : drop_challenges_)
    {
      if (robot_name_ != challenge.robot)
      {
        continue;
      }

      std::string part_to_drop;
      part_to_drop = ariac_common::ConvertPartColorToString(challenge.part_to_drop.color) + "_" + ariac_common::ConvertPartTypeToString(challenge.part_to_drop.type);

      if (attached_part_ == part_to_drop)
      {
        if (pick_counts_[attached_part_] == challenge.drop_after_num + 1)
        {
          drop_time_ = ros_node_->now() + rclcpp::Duration::from_seconds(challenge.drop_after_time);
          drop_challenge_active_ = true;
        }
      }
    }
  }

  void VacuumGripperPluginPrivate::DetachJoint()
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Dropping object");
    picked_part_joint_->Detach();
    model_attached_ = false;

    attached_part_ = "";
  }

  bool VacuumGripperPluginPrivate::CheckModelContact(ConstContactsPtr &msg, std::string contact_type)
  {
    std::string model_in_contact;

    int min_contacts = 2;

    for (int i = 0; i < msg->contact_size(); ++i)
    {
      // Find out which collision is the robot
      if (msg->contact(i).collision1().find("ariac_robots") != std::string::npos)
      {
        model_in_contact = msg->contact(i).collision2();
      }
      else if (msg->contact(i).collision2().find("ariac_robots") != std::string::npos)
      {
        model_in_contact = msg->contact(i).collision1();
      }
      else
      {
        continue;
      }

      if (contact_type == "part")
      {
        // Check if part is in the pickable_list
        bool is_part = false;

        for (auto &type : pickable_part_types_)
        {
          if (model_in_contact.find(type) != std::string::npos)
          {
            for (auto &color : pickable_part_colors_)
            {
              if (model_in_contact.find(color) != std::string::npos)
              {
                is_part = true;
                part_in_contact_ = color + "_" + type;
                break;
              }
            }
          }
        }
        // Ignore contact otherwise
        if (!is_part)
        {
          continue;
        }
      }
      else if (contact_type == "tray")
      {
        if (model_in_contact.find("kit_tray") == std::string::npos)
        {
          continue;
        }
      }

      model_collision_ = boost::dynamic_pointer_cast<gazebo::physics::Collision>(model_->GetWorld()->EntityByName(model_in_contact));

      // Check number of contacts
      if (!msg->contact(i).position_size() > min_contacts)
      {
        // RCLCPP_INFO(ros_node_->get_logger(), "Not enough contacts");
        continue;
      }

      // Check normals
      std::vector<bool> aligned;
      for (int j = 0; j < msg->contact(i).normal_size(); ++j)
      {
        ignition::math::Vector3d contact_normal = gazebo::msgs::ConvertIgn(msg->contact(i).normal(j));
        ignition::math::Vector3d gripper_normal = gripper_link_->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0, 0, 1));

        double alignment = gripper_normal.Dot(contact_normal);

        if (std::abs(alignment) > 0.95)
        {
          aligned.push_back(true);
        }
        else
        {
          aligned.push_back(false);
          // RCLCPP_INFO(ros_node_->get_logger(), "Not aligned");
        }
      }

      if (std::all_of(aligned.begin(), aligned.end(), [](bool v)
                      { return v; }))
      {
        return true;
      }
    }

    return false;
  }

  void VacuumGripperPluginPrivate::EnableGripper(
      ariac_msgs::srv::VacuumGripperControl::Request::SharedPtr req,
      ariac_msgs::srv::VacuumGripperControl::Response::SharedPtr res)
  {
    res->success = false;
    if (req->enable)
    {
      if (!enabled_)
      {
        enabled_ = true;
        res->success = true;
      }
      else
      {
        RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already enabled");
      }
    }
    else
    {
      if (enabled_)
      {
        enabled_ = false;
        res->success = true;
      }
      else
      {
        RCLCPP_WARN(ros_node_->get_logger(), "Gripper is already off");
      }
    }
  }

  void VacuumGripperPluginPrivate::ChangeGripper(
      ariac_msgs::srv::ChangeGripper::Request::SharedPtr req,
      ariac_msgs::srv::ChangeGripper::Response::SharedPtr res)
  {
    if (current_gripper_type_ == req->gripper_type)
    {
      res->success = false;
      res->message = "Requested gripper type is already attached";
      return;
    }

    double threshold = 0.01;

    geometry_msgs::msg::Point kts1_parts;
    kts1_parts.x = -1.100;
    kts1_parts.y = -5.380;
    kts1_parts.z = 0.675;

    geometry_msgs::msg::Point kts2_parts;
    kts2_parts.x = -1.500;
    kts2_parts.y = 5.380;
    kts2_parts.z = 0.675;

    geometry_msgs::msg::Point kts1_trays;
    kts1_trays.x = -1.500;
    kts1_trays.y = -5.380;
    kts1_trays.z = 0.675;

    geometry_msgs::msg::Point kts2_trays;
    kts2_trays.x = -1.100;
    kts2_trays.y = 5.380;
    kts2_trays.z = 0.675;

    // Check that link is in the correct location
    geometry_msgs::msg::Point gripper_position;
    auto link_pose = gripper_link_->WorldPose();
    gripper_position.x = link_pose.X();
    gripper_position.y = link_pose.Y();
    gripper_position.z = link_pose.Z();

    if (req->gripper_type == ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER)
    {
      if (Distance(kts1_parts, gripper_position) <= threshold || Distance(kts2_parts, gripper_position) <= threshold)
      {
        current_gripper_type_ = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
        res->success = true;
      }
      else
      {
        res->success = false;
        res->message = "Gripper is not in correct position for tool change";
      }
    }
    else if (req->gripper_type == ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER)
    {
      if (Distance(kts1_trays, gripper_position) <= threshold || Distance(kts2_trays, gripper_position) <= threshold)
      {
        current_gripper_type_ = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
        res->success = true;
      }
      else
      {
        res->success = false;
        res->message = "Gripper is not in correct position for tool change";
      }
    }
  }

  double VacuumGripperPluginPrivate::Distance(geometry_msgs::msg::Point pt1, geometry_msgs::msg::Point pt2)
  {
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
  }

  void VacuumGripperPluginPrivate::PublishState()
  {
    status_msg_.attached = model_attached_;
    status_msg_.enabled = enabled_;
    status_msg_.type = gripper_types_[current_gripper_type_];

    status_pub_->publish(status_msg_);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(VacuumGripperPlugin)
} // namespace ariac_plugins
