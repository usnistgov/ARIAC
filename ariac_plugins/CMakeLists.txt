cmake_minimum_required(VERSION 3.8)
project(ariac_plugins)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  gz-plugin2
  gz-cmake3
  gz-sim8
  gz-sensors8
  rclcpp
  rclcpp_action
  ros_gz_bridge
  tf2_ros
  ariac_interfaces
  geometry_msgs
  controller_manager_msgs
  angles
  rclpy
  sensor_msgs
  yaml-cpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# PathVelocityPlanner
add_library(PathVelocityPlanner SHARED src/velocity_planner.cpp)
target_include_directories(PathVelocityPlanner PUBLIC include)


# AGV Motion Plugin
add_library(AgvMotionPlugin SHARED src/agv_motion_plugin.cpp)
target_include_directories(AgvMotionPlugin PUBLIC include)
ament_target_dependencies(AgvMotionPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(AgvMotionPlugin PathVelocityPlanner)


ament_export_libraries(
  AgvMotionPlugin
  PathVelocityPlanner
)

install(
  TARGETS
    PathVelocityPlanner
  DESTINATION 
    lib
)

install(
  TARGETS
    AgvMotionPlugin
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
