#include <ariac_sensors/ariac_logical_camera_plugin.hpp>

#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>

#include <memory>

namespace ariac_sensors
{

class AriacLogicalCameraPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  
  /// Publish for logical camera message
  rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_pub_;
  rclcpp::Publisher<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr basic_pub_;

  /// LogicalCameraImage message modified each update
  ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr advanced_image_msg_;
  ariac_msgs::msg::BasicLogicalCameraImage::SharedPtr basic_image_msg_;
  
  /// AriacLogicalCameraPlugin sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  
  /// List of models that the logical camera will publish
  std::vector<std::string> parts_to_publish_;
  std::vector<std::string> colors_;

  std::string camera_name_;
  std::string sensor_type_;

  std::map<std::string, int> part_types_;
  std::map<std::string, int> part_colors_;

  /// Sensor Health Subscription
  ariac_msgs::msg::Sensors sensor_health_;
  rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();
};

AriacLogicalCameraPlugin::AriacLogicalCameraPlugin()
: impl_(std::make_unique<AriacLogicalCameraPluginPrivate>())
{
}

AriacLogicalCameraPlugin::~AriacLogicalCameraPlugin()
{
}

void AriacLogicalCameraPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

   // Set list of models to publish
  impl_->parts_to_publish_ = {"pump", "battery", "regulator", "sensor"};
  impl_->colors_ = {"red", "green", "blue", "orange", "purple"};

  impl_->part_types_ = {
    {"battery", ariac_msgs::msg::Part::BATTERY},
    {"pump", ariac_msgs::msg::Part::PUMP},
    {"regulator", ariac_msgs::msg::Part::REGULATOR},
    {"sensor", ariac_msgs::msg::Part::SENSOR},
  };

  impl_->part_colors_ = {
    {"red", ariac_msgs::msg::Part::RED},
    {"green", ariac_msgs::msg::Part::GREEN},
    {"blue", ariac_msgs::msg::Part::BLUE},
    {"purple", ariac_msgs::msg::Part::PURPLE},
    {"orange", ariac_msgs::msg::Part::ORANGE},
  };

  impl_->camera_name_ = _sdf->Get<std::string>("camera_name");
  impl_->sensor_type_ = _sdf->Get<std::string>("sensor_type");

  if (impl_->sensor_type_ == "basic") {
    impl_->basic_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BasicLogicalCameraImage>(
      "ariac/sensors/" + impl_->camera_name_ + "/image", rclcpp::SensorDataQoS());

    impl_->basic_image_msg_ = std::make_shared<ariac_msgs::msg::BasicLogicalCameraImage>();

  } else if (impl_->sensor_type_ == "advanced") {
    impl_->advanced_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "ariac/sensors/" + impl_->camera_name_ + "/image", rclcpp::SensorDataQoS());

    impl_->advanced_image_msg_ = std::make_shared<ariac_msgs::msg::AdvancedLogicalCameraImage>();
  }

  // Subscribe to sensor health topic
  impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10, 
    std::bind(&AriacLogicalCameraPlugin::SensorHealthCallback, this, std::placeholders::_1));

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&AriacLogicalCameraPluginPrivate::OnUpdate, impl_.get()));
}

void AriacLogicalCameraPluginPrivate::OnUpdate()
{
  if (!sensor_health_.logical_camera) {
    return;
  }

  const auto & image = this->sensor_->Image();

  geometry_msgs::msg::Pose sensor_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  std::vector<ariac_msgs::msg::PartPose> parts;
  std::vector<ariac_msgs::msg::KitTrayPose> trays;

  for (int i = 0; i < image.model_size(); i++) {

    const auto & lc_model = image.model(i);
    std::string name = lc_model.name();

    if (name.find("kit_tray") != std::string::npos) {
        ariac_msgs::msg::KitTrayPose kit_tray;

        std::string id_string = name.substr(9, 2);
        kit_tray.id = std::stoi(id_string);
        kit_tray.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

        trays.push_back(kit_tray);
        continue;
    }

    for(std::string part_type : parts_to_publish_){
      if (name.find(part_type) != std::string::npos) {
        ariac_msgs::msg::PartPose part;

        part.part.type = part_types_[part_type];
        
        for(std::string color : colors_){
          if (name.find(color) != std::string::npos) {
            part.part.color = part_colors_[color];
          }
        }

        part.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

        parts.push_back(part);

        break;
      }
    }
  }

  if (sensor_type_ == "basic") {
    basic_image_msg_->sensor_pose = sensor_pose;

    basic_image_msg_->part_poses.clear();
    basic_image_msg_->tray_poses.clear();

    for (ariac_msgs::msg::PartPose &part : parts) {
      basic_image_msg_->part_poses.push_back(part.pose);
    }

    for (ariac_msgs::msg::KitTrayPose &tray : trays) {
      basic_image_msg_->tray_poses.push_back(tray.pose);
    }

    basic_pub_->publish(*basic_image_msg_);

  } else if (sensor_type_ == "advanced") {
    advanced_image_msg_->sensor_pose = sensor_pose;

    advanced_image_msg_->part_poses = parts;
    advanced_image_msg_->tray_poses = trays;

    advanced_pub_->publish(*advanced_image_msg_);
  }
}

void AriacLogicalCameraPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg){
  impl_->sensor_health_ = *msg;
}

GZ_REGISTER_SENSOR_PLUGIN(AriacLogicalCameraPlugin)

}  // namespace ariac_sensors
