#ifndef ARIAC_RAY_PLUGIN_HPP_
#define ARIAC_RAY_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <ariac_msgs/msg/sensors.hpp>

#include <memory>

namespace ariac_sensors
{

class AriacRayPluginPrivate;


class AriacRayPlugin : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  AriacRayPlugin();

  /// \brief Destructor
  virtual ~AriacRayPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);

private:
  std::unique_ptr<AriacRayPluginPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // ARIAC_RAY_PLUGIN_HPP_