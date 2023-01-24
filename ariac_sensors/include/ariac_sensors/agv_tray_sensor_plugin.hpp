#ifndef AGV_TRAY_SENSOR_PLUGIN_HPP_
#define AGV_TRAY_SENSOR_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace ariac_sensors
{

class AGVTraySensorPluginPrivate;

class AGVTraySensorPlugin : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  AGVTraySensorPlugin();
  /// Destructor.
  virtual ~AGVTraySensorPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<AGVTraySensorPluginPrivate> impl_;
};

}  // namespace ariac_plugins

#endif  // AGV_TRAY_SENSOR_PLUGIN_HPP_
