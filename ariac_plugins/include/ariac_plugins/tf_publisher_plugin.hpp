#ifndef ARIAC_PLUGINS__TF_PUBLISHER_PLUGIN_HPP_
#define ARIAC_PLUGINS__TF_PUBLISHER_PLUGIN_HPP_

// Gazebo 
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/boolean.pb.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace ariac_plugins{
  enum class SensorStatus {
    NOT_SENSOR,
    WAIT_NEEDED,
    READY_TO_SET,
    SET
  };
  class TFPublisherPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate
  {
    public:
      ~TFPublisherPlugin();

      void Configure (const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void PostUpdate(
        const gz::sim::UpdateInfo &_info, 
        const gz::sim::EntityComponentManager &_ecm) override;
    
    private:
      TransformStamped gz_pose_to_transform(gz::math::Pose3d, std::string, std::string);

      // GZ
      gz::sim::Model model;
      gz::sim::Link base_link;

      // ROS
      rclcpp::Node::SharedPtr ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
      std::thread thread_executor_spin;

      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
      TransformStamped base_link_transform;
      
      std::vector<TransformStamped> child_transforms;

      // Variables
      bool is_static = false;

      SensorStatus sensor_status = SensorStatus::NOT_SENSOR;

      std::string base_frame_name;
  };
}

#endif // ARIAC_PLUGINS__TF_PUBLISHER_PLUGIN_HPP_