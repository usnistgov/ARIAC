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

# AGV Tray Interface Plugin
add_library(AgvTrayInterfacePlugin SHARED src/agv_tray_interface_plugin.cpp)
target_include_directories(AgvTrayInterfacePlugin PUBLIC include)

ament_target_dependencies(AgvTrayInterfacePlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Cheat Tools Plugin
add_library(CheatToolsPlugin SHARED src/cheat_tools_plugin.cpp)
target_include_directories(CheatToolsPlugin PUBLIC include)

ament_target_dependencies(CheatToolsPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Contact Plugin
add_library(ContactPlugin SHARED src/contact_plugin.cpp)
target_include_directories(ContactPlugin PUBLIC include)

ament_target_dependencies(ContactPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Kit Tray Plugin
add_library(KitTrayPlugin SHARED src/kit_tray_plugin.cpp)
target_include_directories(KitTrayPlugin PUBLIC include)

ament_target_dependencies(KitTrayPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


ament_export_libraries(
  AgvMotionPlugin
  AgvTrayInterfacePlugin
  CheatToolsPlugin
  ContactPlugin
  KitTrayPlugin
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
    AgvTrayInterfacePlugin
    CheatToolsPlugin
    ContactPlugin
    KitTrayPlugin
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
