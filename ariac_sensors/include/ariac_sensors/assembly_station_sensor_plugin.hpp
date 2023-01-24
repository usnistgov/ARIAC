#ifndef ASSEMBLY_STATION_SENSOR_PLUGIN_HPP_
#define ASSEMBLY_STATION_SENSOR_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <ariac_msgs/msg/sensors.hpp>

#include <memory>

namespace ariac_sensors
{

class AssemblyStationSensorPluginPrivate;

/// Plugin to attach to a gazebo AssemblyStationSensorPlugin sensor and publish ROS message of output
class AssemblyStationSensorPlugin : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  AssemblyStationSensorPlugin();
  /// Destructor.
  virtual ~AssemblyStationSensorPlugin();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<AssemblyStationSensorPluginPrivate> impl_;
};

}  // namespace ariac_sensors

#endif  // ASSEMBLY_STATION_SENSOR_PLUGIN_HPP_
