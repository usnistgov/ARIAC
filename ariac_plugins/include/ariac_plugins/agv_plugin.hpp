#ifndef ARIAC_PLUGINS__AGV_PLUGIN_HPP_
#define ARIAC_PLUGINS__AGV_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

// For std::unique_ptr, could be removed
#include <memory>

namespace ariac_plugins
{
// Forward declaration of private data class.
class AGVPluginPrivate;

/// Example ROS-powered Gazebo plugin with some useful boilerplate.
/// \details This is a `ModelPlugin`, but it could be any supported Gazebo plugin type, such as
/// System, Visual, GUI, World, Sensor, etc.
class AGVPlugin : public gazebo::ModelPlugin
{
public:
  /// Constructor
  AGVPlugin();

  /// Destructor
  virtual ~AGVPlugin();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<AGVPluginPrivate> impl_;
};
}  // namespace ariac_plugins

#endif  // ARIAC_PLUGINS__AGV_PLUGIN_HPP_