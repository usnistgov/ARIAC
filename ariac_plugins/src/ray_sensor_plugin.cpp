#include "ariac_plugins/ray_sensor_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::RaySensorPlugin,
  gz::sim::System,
  ariac_plugins::RaySensorPlugin::ISystemConfigure
)

using namespace ariac_plugins;

RaySensorPlugin::~RaySensorPlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void RaySensorPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_event_mgr)
{
  // Read sdf tags
  sensor_type = _sdf->Get<std::string>("sensor_type");
  sensor_name = _sdf->Get<std::string>("sensor_name");
  std::string sensor_topic = sensor_name + "_gz_topic";

  // Set up ROS
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  ros_node = rclcpp::Node::make_shared(sensor_name + "_ray_plugin");

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);

  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);
  auto spin = [this]() {
    while (rclcpp::ok()) {
      executor->spin_once();
    }
  };
  thread_executor_spin = std::thread(spin);

  gz_node = std::make_shared<gz::transport::Node>();
  
  if (sensor_type == "break_beam") { 

    status_pub = ros_node->create_publisher<ariac_interfaces::msg::BreakBeamStatus>(sensor_name + "/status", rclcpp::SensorDataQoS());
    change_pub = ros_node->create_publisher<ariac_interfaces::msg::BreakBeamStatus>(sensor_name + "/change", rclcpp::SensorDataQoS());
    
    gz_node->Subscribe(sensor_topic, &RaySensorPlugin::laser_scan_gz_cb, this);

  } else if (sensor_type == "distance"){
    
    distance_pub = ros_node->create_publisher<ariac_interfaces::msg::DistanceSensor>(sensor_name + "/distance", 10);
    
    gz_node->Subscribe(sensor_topic, &RaySensorPlugin::laser_scan_gz_cb, this);

  } else if (sensor_type == "lidar") {      
    
    point_cloud_pub = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(sensor_name + "/scan", 10);
  
    gz_node->Subscribe(sensor_topic+"/points", &RaySensorPlugin::point_cloud_gz_cb, this);

  }
}

void RaySensorPlugin::point_cloud_gz_cb(const gz::msgs::PointCloudPacked &_gz_msg) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
  ros_msg.header.frame_id = sensor_name + "_frame";
  point_cloud_pub->publish(ros_msg);
}

void RaySensorPlugin::laser_scan_gz_cb(const gz::msgs::LaserScan &_gz_msg) {
  if (sensor_type == "break_beam") {
    sensor_msgs::msg::LaserScan ls;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);
    status_msg.header.frame_id = sensor_name + "_frame";
    status_msg.header.stamp = ls.header.stamp;

    bool object_detected = false;
    bool publish_change = false;

    for(float distance : ls.ranges) {
      if (distance > 0.0 && distance < 1.0) {
        object_detected = true;
        break;
      }
    }

    if (status_msg.object_detected != object_detected) {
      publish_change = true;
    }

    status_msg.object_detected = object_detected;

    if (publish_change) {
      change_pub->publish(status_msg);
    }
    status_pub->publish(status_msg);
  } else if (sensor_type == "distance") {
    sensor_msgs::msg::LaserScan ls;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);
    
    ariac_interfaces::msg::DistanceSensor distance_msg;
    
    distance_msg.header.frame_id = sensor_name + "_frame";
    distance_msg.header.stamp.sec = _gz_msg.header().stamp().sec();
    distance_msg.header.stamp.nanosec = _gz_msg.header().stamp().nsec();

    distance_msg.distance = ls.ranges[0];
    distance_pub->publish(distance_msg);
  }
}


