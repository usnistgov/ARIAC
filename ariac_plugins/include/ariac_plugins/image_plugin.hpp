#ifndef ARIAC_PLUGINS__IMAGE_PLUGIN_HPP
#define ARIAC_PLUGINS__IMAGE_PLUGIN_HPP

#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <gz/sim/Sensor.hh>
#include <ros_gz_bridge/convert.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <gz/plugin/Register.hh>

namespace ariac_plugins{

// Forward declaration of a class to hold the member data about the plugin
class ImagePlugin : 
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
  public:
    ~ImagePlugin();

    void Configure (
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_event_mgr) override;

  private:
    void OnNewImageFrame(const gz::msgs::Image & _gz_msg);
    void FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg);
  
    std::string sensor_name;

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread thread_executor_spin;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    sensor_msgs::msg::CameraInfo camera_info_msg;    
    
    std::shared_ptr<gz::transport::Node> gz_node;
};
}

#endif