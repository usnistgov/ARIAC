#include <ariac_sensors/ariac_camera_plugin.hpp>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include "gazebo/rendering/DepthCamera.hh"
#include <gazebo/rendering/Distortion.hh>

#include <rclcpp/rclcpp.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <string>
#include <algorithm>
#include <limits>
#include <memory>

#define FLOAT_SIZE sizeof(float)

namespace ariac_sensors
{

class AriacCameraPluginPrivate
{
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  std::string camera_type_;

  gazebo::sensors::CameraSensorPtr camera_parent_sensor_;

  gazebo::sensors::DepthCameraSensorPtr depth_parent_sensor_;

  gazebo::rendering::CameraPtr camera_;
  gazebo::rendering::DepthCameraPtr depth_camera_;

  gazebo::event::ConnectionPtr newImageFrameConnection_;
  gazebo::event::ConnectionPtr newDepthFrameConnection_;

  unsigned int width_, height_, depth_;
  std::string format_;

  std::string camera_name_;
  std::string frame_name_;

  image_transport::Publisher image_pub_;

  image_transport::Publisher depth_image_pub_;

  double min_depth_;
  double max_depth_;

  sensor_msgs::msg::Image image_msg_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::mutex image_mutex_;

  bool publish_sensor_data_;

  void FillCameraInfoMsg();

};

AriacCameraPlugin::AriacCameraPlugin()
: impl_(std::make_unique<AriacCameraPluginPrivate>())
{
}

AriacCameraPlugin::~AriacCameraPlugin()
{
}

void AriacCameraPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->camera_type_ = _sdf->Get<std::string>("camera_type");

  impl_->camera_name_ = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->publish_sensor_data_ = true;
  impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 10, 
    std::bind(&AriacCameraPlugin::SensorHealthCallback, this, std::placeholders::_1));

  const std::string camera_topic = "ariac/sensors/" + impl_->camera_name_ + "/rgb_image";

  impl_->image_pub_ = image_transport::create_publisher(
        impl_->ros_node_.get(), camera_topic, qos.get_publisher_qos(
          camera_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile());
  
  if (impl_->camera_type_ == "rgb"){
    impl_->camera_parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor);
    impl_->camera_ = impl_->camera_parent_sensor_->Camera();

    impl_->width_ = impl_->camera_->ImageWidth();
    impl_->height_ = impl_->camera_->ImageHeight();
    impl_->depth_ = impl_->camera_->ImageDepth();
    impl_->format_ = impl_->camera_->ImageFormat();

    impl_->newImageFrameConnection_ = impl_->camera_->ConnectNewImageFrame(
      std::bind(&AriacCameraPlugin::OnNewImageFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)
    );

    impl_->camera_parent_sensor_->SetActive(true);

  } else if (impl_->camera_type_ == "rgbd")
  {
    impl_->depth_parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(_sensor);
    impl_->depth_camera_ = impl_->depth_parent_sensor_->DepthCamera();

    impl_->width_ = impl_->depth_camera_->ImageWidth();
    impl_->height_ = impl_->depth_camera_->ImageHeight();
    impl_->depth_ = impl_->depth_camera_->ImageDepth();
    impl_->format_ = impl_->depth_camera_->ImageFormat();

    impl_->min_depth_ = _sdf->Get<double>("min_depth", 0.4).first;
    impl_->max_depth_ =
      _sdf->Get<double>("max_depth", std::numeric_limits<float>::infinity()).first;

    impl_->newImageFrameConnection_ = impl_->depth_camera_->ConnectNewImageFrame(
      std::bind(&AriacCameraPlugin::OnNewImageFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)
    );

    impl_->newDepthFrameConnection_ = impl_->depth_camera_->ConnectNewDepthFrame(
      std::bind(&AriacCameraPlugin::OnNewDepthFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5)
    );

    impl_->depth_parent_sensor_->SetActive(true);

    const std::string depth_image_topic = "ariac/sensors/" + impl_->camera_name_ + "/depth_image";

    impl_->depth_image_pub_ = image_transport::create_publisher(
          impl_->ros_node_.get(), depth_image_topic, qos.get_publisher_qos(
            depth_image_topic, rclcpp::SensorDataQoS().reliable()).get_rmw_qos_profile());
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Camera type not recognized");
    return;
  }

  const std::string camera_info_topic = "ariac/sensors/" + impl_->camera_name_ + "/camera_info";

  impl_->camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, qos.get_publisher_qos(
        camera_info_topic, rclcpp::SensorDataQoS().reliable()));

  impl_->FillCameraInfoMsg();
  impl_->camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        impl_->ros_node_.get(), impl_->camera_name_);
  
  impl_->camera_info_manager_->setCameraInfo(impl_->camera_info_msg_);

}

void AriacCameraPlugin::OnNewImageFrame(
  const unsigned char *_image,
  unsigned int _width, unsigned int _height,
  unsigned int _depth, const std::string &_format)
{
  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  gazebo::common::Time sensor_update_time;

  if (impl_->camera_type_ == "rgb") {
    sensor_update_time = impl_->camera_parent_sensor_->LastMeasurementTime();
  } else if (impl_->camera_type_ == "rgbd") {
    sensor_update_time = impl_->depth_parent_sensor_->LastMeasurementTime();
  }
  
  auto camera_info_msg = impl_->camera_info_manager_->getCameraInfo();

  camera_info_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_update_time);

  impl_->camera_info_pub_->publish(camera_info_msg);

  // Publish image
  impl_->image_msg_.header.frame_id = impl_->frame_name_;
  impl_->image_msg_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_update_time);

  // Copy from src to image_msg
  uint32_t img_step_ = 3;
  std::string img_encoding_ = sensor_msgs::image_encodings::RGB8;

  sensor_msgs::fillImage(
    impl_->image_msg_, img_encoding_, _height, _width,
    img_step_ * _width, reinterpret_cast<const void *>(_image));

  if (impl_->publish_sensor_data_) {
    impl_->image_pub_.publish(impl_->image_msg_);
  }
}

void AriacCameraPlugin::OnNewDepthFrame(
  const float *_image,
  unsigned int _width, unsigned int _height,
  unsigned int _depth, const std::string &_format)
{
  std::lock_guard<std::mutex> image_lock(impl_->image_mutex_);

  gazebo::common::Time sensor_update_time;

  sensor_update_time = impl_->depth_parent_sensor_->LastMeasurementTime();

  sensor_msgs::msg::Image image_msg;
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.header.frame_id = impl_->frame_name_;
  image_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sensor_update_time);
  image_msg.width = _width;
  image_msg.height = _height;
  image_msg.step = FLOAT_SIZE * _width;
  image_msg.data.resize(_width * _height * FLOAT_SIZE);
  image_msg.is_bigendian = 0;

  int index = 0;

  float pos_inf = std::numeric_limits<float>::infinity();
  float neg_inf = -pos_inf;

  // Copy from src to image_msg
  for (uint32_t j = 0; j < _height; j++) {
    for (uint32_t i = 0; i < _width; i++) {
      index = i + j * _width;
      float depth = _image[index];
      if (impl_->min_depth_ < depth && depth < impl_->max_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &depth, FLOAT_SIZE);
      } else if (depth <= impl_->min_depth_) {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &neg_inf, FLOAT_SIZE);
      } else {
        std::memcpy(&image_msg.data[index * FLOAT_SIZE], &pos_inf, FLOAT_SIZE);
      }
    }
  }
  
  if (impl_->publish_sensor_data_) {
    impl_->depth_image_pub_.publish(image_msg);
  }
  
}

void AriacCameraPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg){
  impl_->publish_sensor_data_ = msg->camera;
}

void AriacCameraPluginPrivate::FillCameraInfoMsg(){
  camera_info_msg_.header.frame_id = frame_name_;
  camera_info_msg_.height = height_;
  camera_info_msg_.width = width_;
  camera_info_msg_.distortion_model = "plumb_bob";
  camera_info_msg_.d.resize(5);

  double cx = (static_cast<double>(width_) + 1.0) / 2.0;
  double cy = (static_cast<double>(height_) + 1.0) / 2.0;
  double focal_length;

  bool border_crop = true;
  auto hack_baseline = 0.0;

  // Get distortion from camera
  double distortion_k1{0.0};
  double distortion_k2{0.0};
  double distortion_k3{0.0};
  double distortion_t1{0.0};
  double distortion_t2{0.0};

  if (camera_type_ == "rgb") {
    double focal_length =
      (static_cast<double>(width_)) / (2.0 * tan(camera_->HFOV().Radian() / 2.0));

    if (camera_->LensDistortion()) {
      camera_->LensDistortion()->SetCrop(border_crop);

      distortion_k1 = camera_->LensDistortion()->K1();
      distortion_k2 = camera_->LensDistortion()->K2();
      distortion_k3 = camera_->LensDistortion()->K3();
      distortion_t1 = camera_->LensDistortion()->P1();
      distortion_t2 = camera_->LensDistortion()->P2();
    }
  } else if (camera_type_ == "rgbd") {
    focal_length =
      (static_cast<double>(width_)) / (2.0 * tan(depth_camera_->HFOV().Radian() / 2.0));

    if (depth_camera_->LensDistortion()) {
      depth_camera_->LensDistortion()->SetCrop(border_crop);

      distortion_k1 = depth_camera_->LensDistortion()->K1();
      distortion_k2 = depth_camera_->LensDistortion()->K2();
      distortion_k3 = depth_camera_->LensDistortion()->K3();
      distortion_t1 = depth_camera_->LensDistortion()->P1();
      distortion_t2 = depth_camera_->LensDistortion()->P2();
    }
  }

  // D = {k1, k2, t1, t2, k3}, as specified in:
  // - sensor_msgs/CameraInfo: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  // - OpenCV: http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  camera_info_msg_.d[0] = distortion_k1;
  camera_info_msg_.d[1] = distortion_k2;
  camera_info_msg_.d[2] = distortion_t1;
  camera_info_msg_.d[3] = distortion_t2;
  camera_info_msg_.d[4] = distortion_k3;

  // Original camera matrix
  camera_info_msg_.k[0] = focal_length;
  camera_info_msg_.k[1] = 0.0;
  camera_info_msg_.k[2] = cx;
  camera_info_msg_.k[3] = 0.0;
  camera_info_msg_.k[4] = focal_length;
  camera_info_msg_.k[5] = cy;
  camera_info_msg_.k[6] = 0.0;
  camera_info_msg_.k[7] = 0.0;
  camera_info_msg_.k[8] = 1.0;

  // rectification
  camera_info_msg_.r[0] = 1.0;
  camera_info_msg_.r[1] = 0.0;
  camera_info_msg_.r[2] = 0.0;
  camera_info_msg_.r[3] = 0.0;
  camera_info_msg_.r[4] = 1.0;
  camera_info_msg_.r[5] = 0.0;
  camera_info_msg_.r[6] = 0.0;
  camera_info_msg_.r[7] = 0.0;
  camera_info_msg_.r[8] = 1.0;

  // camera_ projection matrix (same as camera_ matrix due
  // to lack of distortion/rectification) (is this generated?)
  camera_info_msg_.p[0] = focal_length;
  camera_info_msg_.p[1] = 0.0;
  camera_info_msg_.p[2] = cx;
  camera_info_msg_.p[3] = -focal_length * hack_baseline;
  camera_info_msg_.p[4] = 0.0;
  camera_info_msg_.p[5] = focal_length;
  camera_info_msg_.p[6] = cy;
  camera_info_msg_.p[7] = 0.0;
  camera_info_msg_.p[8] = 0.0;
  camera_info_msg_.p[9] = 0.0;
  camera_info_msg_.p[10] = 1.0;
  camera_info_msg_.p[11] = 0.0;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(AriacCameraPlugin)

}  // namespace ariac_sensors