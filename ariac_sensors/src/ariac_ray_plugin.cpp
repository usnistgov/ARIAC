#include <ariac_sensors/ariac_ray_plugin.hpp>

#include <boost/make_shared.hpp>
#include <gazebo/transport/transport.hh>

#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/sensors.hpp>

#include <string>
#include <algorithm>
#include <limits>
#include <memory>

namespace ariac_sensors
{

class AriacRayPluginPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  ariac_msgs::msg::BreakBeamStatus status_msg_;
  rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr change_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_pub_;

  std::string sensor_name_;

  /// TF frame output is published in
  std::string frame_name_;

  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  void ReadLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  std::string sensor_topic_;

  std::string sensor_type_;

  ariac_msgs::msg::Sensors sensor_health_;

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo subscribe to parent sensor's laser scan
  gazebo::transport::SubscriberPtr laser_scan_sub_;

  /// Sensor Health Subscription
  rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;

  /// Publish a sensor_msgs/Range message from a gazebo laser scan
  void PublishBreakBeamStatus(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/Range message from a gazebo laser scan
  void PublishRange(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  void PublishLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/PointCloud message from a gazebo laser scan
  void PublishPointCloud(ConstLaserScanStampedPtr & _msg);
};

AriacRayPlugin::AriacRayPlugin()
: impl_(std::make_unique<AriacRayPluginPrivate>())
{
}

AriacRayPlugin::~AriacRayPlugin()
{
  // Must release subscriber and then call fini on node to remove it from topic manager.
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void AriacRayPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sensor_type_ = _sdf->Get<std::string>("sensor_type");
  impl_->sensor_name_ = _sdf->Get<std::string>("sensor_name");

  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  if (impl_->sensor_type_ == "break_beam") {

    impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
      "ariac/sensors/" + impl_->sensor_name_ + "/status", rclcpp::SensorDataQoS());
    impl_->change_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
      "ariac/sensors/" + impl_->sensor_name_ + "/change", rclcpp::SensorDataQoS());

  } else if (impl_->sensor_type_ == "proximity") {
    
    impl_->range_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>(
      "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());

  } else if (impl_->sensor_type_ == "laser_profiler") {
    
    impl_->laser_scan_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());

  } else if (impl_->sensor_type_ == "lidar") {
    
    impl_->point_cloud_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud>(
      "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());

  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Sensor type not valid");
    return;
  }

  // Subscribe to sensor health topic
  impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10, 
    std::bind(&AriacRayPlugin::SensorHealthCallback, this, std::placeholders::_1));

  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  impl_->sensor_topic_ = _sensor->Topic();
  impl_->SubscribeGazeboLaserScan();
}

void AriacRayPluginPrivate::SubscribeGazeboLaserScan()
{
  laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &AriacRayPluginPrivate::ReadLaserScan, this);
}

void AriacRayPluginPrivate::ReadLaserScan(ConstLaserScanStampedPtr & _msg)
{
  if (sensor_type_ == "break_beam") {
    PublishBreakBeamStatus(_msg);
  } else if (sensor_type_ == "proximity") {
    PublishRange(_msg);
  } else if (sensor_type_ == "laser_profiler") {
    PublishLaserScan(_msg);
  } else if (sensor_type_ == "lidar") {
    PublishPointCloud(_msg);
  }
}

void AriacRayPluginPrivate::PublishBreakBeamStatus(ConstLaserScanStampedPtr & _msg)
{
  if (!sensor_health_.break_beam) {
    return;
  }

  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg, 0.0);
  status_msg_.header.frame_id = frame_name_;
  status_msg_.header.stamp = ls.header.stamp;

  bool object_detected = false;
  bool publish_change = false;

  for(float distance : ls.ranges) {
    if (distance > 0.0 && distance < 1.0) {
        object_detected = true;
        break;
    }
  }

  if (status_msg_.object_detected != object_detected) {
    publish_change = true;
  }

  status_msg_.object_detected = object_detected;

  // Publish output
  if (publish_change) {
    change_pub_->publish(status_msg_);
  }
  status_pub_->publish(status_msg_);
}

void AriacRayPluginPrivate::PublishRange(ConstLaserScanStampedPtr & _msg)
{
  if (!sensor_health_.proximity) {
    return;
  }

  // Convert Laser scan to range
  auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);
  range_msg.header.frame_id = frame_name_;
  range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_pub_->publish(range_msg);
}

void AriacRayPluginPrivate::PublishLaserScan(ConstLaserScanStampedPtr & _msg)
{
  if (!sensor_health_.laser_profiler) {
    return;
  }

  // Convert Laser scan to ROS LaserScan
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg, 0.0);
  ls.header.frame_id = frame_name_;
  laser_scan_pub_->publish(ls);
}

void AriacRayPluginPrivate::PublishPointCloud(ConstLaserScanStampedPtr & _msg)
{
  if (!sensor_health_.lidar) {
    return;
  }

  // Convert Laser scan to PointCloud
  auto pc = gazebo_ros::Convert<sensor_msgs::msg::PointCloud>(*_msg, 0.0);
  pc.header.frame_id = frame_name_;
  point_cloud_pub_->publish(pc);
}

void AriacRayPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg){
  impl_->sensor_health_ = *msg;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(AriacRayPlugin)

}  // namespace ariac_sensors