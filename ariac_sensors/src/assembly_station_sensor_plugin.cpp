#include <ariac_sensors/assembly_station_sensor_plugin.hpp>

#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <ariac_msgs/srv/score_task.hpp>

#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/trial.hpp>

#include <memory>

namespace ariac_sensors
{

class AssemblyStationSensorPluginPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  
  /// AssemblyStationSensorPlugin sensor this plugin is attached to
  gazebo::sensors::LogicalCameraSensorPtr sensor_;
  
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;
  
  /// List of models that the logical camera will publish
  std::vector<std::string> parts_to_publish_;
  std::vector<std::string> colors_;

  std::map<std::string, int> part_types_;
  std::map<std::string, int> part_colors_;

  std::vector<ariac_msgs::msg::Order> orders_;

  std::string sensor_num_;

  // Trial msg subscriber
  rclcpp::Subscription<ariac_msgs::msg::Trial>::SharedPtr trial_sub_;

  // Task Scoring Service
  rclcpp::Service<ariac_msgs::srv::ScoreTask>::SharedPtr score_task_service_;

  /// Publish latest logical camera data to ROS
  void OnUpdate();

  void TrialConfigCallback(const ariac_msgs::msg::Trial::SharedPtr msg);

  void ScoreTask(
    ariac_msgs::srv::ScoreTask::Request::SharedPtr,
    ariac_msgs::srv::ScoreTask::Response::SharedPtr
  );
};

AssemblyStationSensorPlugin::AssemblyStationSensorPlugin()
: impl_(std::make_unique<AssemblyStationSensorPluginPrivate>())
{
}

AssemblyStationSensorPlugin::~AssemblyStationSensorPlugin()
{
}

void AssemblyStationSensorPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  if (!_sdf->HasElement("sensor_num")) {
    impl_->sensor_num_ = "0";
  } else {
    impl_->sensor_num_ = _sdf->Get<std::string>("sensor_num");
  }

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

  // Subscribe to trial topic
  impl_->trial_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Trial>("/ariac/trial_config", 10, 
    std::bind(&AssemblyStationSensorPluginPrivate::TrialConfigCallback, this->impl_.get(), std::placeholders::_1));

  // Register Score Task Service
  impl_->score_task_service_ = impl_->ros_node_->create_service<ariac_msgs::srv::ScoreTask>(
  "/ariac/score_task_assembly_station" + impl_->sensor_num_, 
  std::bind(
    &AssemblyStationSensorPluginPrivate::ScoreTask, this->impl_.get(),
    std::placeholders::_1, std::placeholders::_2));

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&AssemblyStationSensorPluginPrivate::OnUpdate, impl_.get()));
}

void AssemblyStationSensorPluginPrivate::OnUpdate()
{
  const auto & image = this->sensor_->Image();

  geometry_msgs::msg::Pose sensor_pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(
    gazebo::msgs::ConvertIgn(image.pose()));

  std::vector<ariac_msgs::msg::PartPose> parts;

  for (int i = 0; i < image.model_size(); i++) {

    const auto & lc_model = image.model(i);
    std::string name = lc_model.name();

    for(const auto &type : part_types_) {
      if (name.find(type.first) != std::string::npos) {
        ariac_msgs::msg::PartPose part;

        part.part.type = type.second;
        
        for(const auto &color : part_colors_) {
          if (name.find(color.first) != std::string::npos) {
            part.part.color = color.second;
          }
        }

        part.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(gazebo::msgs::ConvertIgn(lc_model.pose()));

        parts.push_back(part);

        break;
      }
    }
  }
}

void AssemblyStationSensorPluginPrivate::TrialConfigCallback(const ariac_msgs::msg::Trial::SharedPtr msg)
{
  orders_ = msg->orders;
}

void AssemblyStationSensorPluginPrivate::ScoreTask(
  ariac_msgs::srv::ScoreTask::Request::SharedPtr request,
  ariac_msgs::srv::ScoreTask::Response::SharedPtr response)
{
  // Verify requested order is correct for this station 
  bool valid_id = false;
  ariac_msgs::msg::Order req_order;
  for (const ariac_msgs::msg::Order order: orders_){
    if (request->order_id == order.id){
      req_order = order;
      valid_id = true;
      break;
    }    
  }
  
  if (!valid_id) {
    RCLCPP_INFO(ros_node_->get_logger(), "ID is not valid");
    return;
  }

  if (req_order.type == ariac_msgs::msg::Order::ASSEMBLY) {
    // Check to see if the correct station
    if (req_order.assembly_task.station != std::stoi(sensor_num_)) {
      RCLCPP_INFO(ros_node_->get_logger(), "Wrong station for this order");
      return;
    }
    
    for (const auto &part : req_order.assembly_task.parts){
      
    }

  } else if (req_order.type == ariac_msgs::msg::Order::COMBINED){
    // Check to see if the correct station
    if (req_order.combined_task.station != std::stoi(sensor_num_)) {
      RCLCPP_INFO(ros_node_->get_logger(), "Wrong station for this order");
      return;
    }
  } else if (req_order.type == ariac_msgs::msg::Order::KITTING) {
    RCLCPP_INFO(ros_node_->get_logger(), "Unable to score kitting tasks");
    return;
  }


  
}

GZ_REGISTER_SENSOR_PLUGIN(AssemblyStationSensorPlugin)

}  // namespace ariac_sensors
