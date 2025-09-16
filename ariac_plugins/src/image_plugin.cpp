#include "ariac_plugins/image_plugin.hpp"

GZ_ADD_PLUGIN(
  ariac_plugins::ImagePlugin,
  gz::sim::System,
  ariac_plugins::ImagePlugin::ISystemConfigure
)

using namespace ariac_plugins;

ImagePlugin::~ImagePlugin()
{
  executor->cancel();
  thread_executor_spin.join();
}

void ImagePlugin::Configure (
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_event_mgr) 
{
  // Read sdf tags
  sensor_name = _sdf->Get<std::string>("sensor_name");

  // Setup ROS
  if (!rclcpp::ok()){
    rclcpp::init(0, nullptr);
  }
  
  ros_node = rclcpp::Node::make_shared(_sdf->Get<std::string>("sensor_name")+"_plugin");

  rclcpp::Parameter sim_time("use_sim_time", true);
  ros_node->set_parameter(sim_time);
  
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor->spin_once();
    }
  };
  
  thread_executor_spin = std::thread(spin);

  image_pub = ros_node->create_publisher<sensor_msgs::msg::Image>(sensor_name + "/image", 10);
  camera_info_pub = ros_node->create_publisher<sensor_msgs::msg::CameraInfo>(sensor_name + "/info", 10);

  // GZ node
  gz_node = std::make_shared<gz::transport::Node>();
  gz_node->Subscribe(sensor_name + "_gz_topic", &ImagePlugin::OnNewImageFrame, this);
  gz_node->Subscribe(sensor_name + "_gz_info_topic", &ImagePlugin::FillCameraInfoMsg, this);
}


void ImagePlugin::OnNewImageFrame(const gz::msgs::Image & _gz_msg)
{
  sensor_msgs::msg::Image image;

  ros_gz_bridge::convert_gz_to_ros(_gz_msg, image);
  image.header.frame_id = sensor_name + "_frame";
  image_pub->publish(image);

  camera_info_msg.header.stamp.sec = _gz_msg.header().stamp().sec();
  camera_info_msg.header.stamp.nanosec = _gz_msg.header().stamp().nsec();      
  camera_info_pub->publish(camera_info_msg);
}

void ImagePlugin::FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg) {
  
  camera_info_msg.header.frame_id = sensor_name + "_frame";;
  camera_info_msg.height = _info_msg.height();
  camera_info_msg.width = _info_msg.width();

  if (_info_msg.distortion().model() == 0) {
    camera_info_msg.distortion_model = "plumb_bob";
  } else if (_info_msg.distortion().model() == 1) {
    camera_info_msg.distortion_model = "rational_polynomial";
  } else if (_info_msg.distortion().model() == 2) {
    camera_info_msg.distortion_model = "equidistant";
  }
  camera_info_msg.d.resize(5);

  for(int i = 0; i <=4; i++){
    camera_info_msg.d[i] = _info_msg.distortion().k(i);
  }

  for(int i = 0; i <= 8; i++){
    camera_info_msg.k[i] = _info_msg.intrinsics().k(i);
    camera_info_msg.r[i] = _info_msg.rectification_matrix(i);
  }

  for(int i = 0; i <= 11; i++){
    camera_info_msg.p[i] = _info_msg.projection().p(i);
  }

  gz_node->Unsubscribe(sensor_name + "_gz_info_topic");
}