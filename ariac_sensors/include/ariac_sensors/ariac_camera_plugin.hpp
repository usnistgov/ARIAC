#ifndef ARIAC_CAMERA_PLUGIN_HPP_
#define ARIAC_CAMERA_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <ariac_msgs/msg/sensors.hpp>

#include <memory>

namespace ariac_sensors
{

class AriacCameraPluginPrivate;


class AriacCameraPlugin : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  AriacCameraPlugin();

  /// \brief Destructor
  virtual ~AriacCameraPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  void OnNewImageFrame(const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth, const std::string &_format);
  
  void OnNewDepthFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

  void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);

private:
  std::unique_ptr<AriacCameraPluginPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // ARIAC_CAMERA_PLUGIN_HPP_