#ifndef RAY_SENSOR_PLUGIN_HPP
#define RAY_SENSOR_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>

#include <ariac_interfaces/msg/break_beam_status.hpp>
#include <ariac_interfaces/msg/distance_sensor.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>


namespace ariac_plugins{

/// Plugin for the Ray sensors
class RaySensorPlugin : 
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
  public:
    ~RaySensorPlugin();

    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;

  private:
    void point_cloud_gz_cb(const gz::msgs::PointCloudPacked &_gz_msg);
    void laser_scan_gz_cb(const gz::msgs::LaserScan &_gz_msg);

    std::shared_ptr<gz::transport::Node> gz_node;      

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;

    std::string sensor_type;
    std::string sensor_name;

    ariac_interfaces::msg::BreakBeamStatus status_msg;
    rclcpp::Publisher<ariac_interfaces::msg::BreakBeamStatus>::SharedPtr status_pub;
    rclcpp::Publisher<ariac_interfaces::msg::BreakBeamStatus>::SharedPtr change_pub;
    rclcpp::Publisher<ariac_interfaces::msg::DistanceSensor>::SharedPtr distance_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
};
}


#endif