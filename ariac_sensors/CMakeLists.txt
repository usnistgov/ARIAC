cmake_minimum_required(VERSION 3.8)
project(ariac_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  camera_info_manager
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  image_transport
  cv_bridge
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  orocos_kdl
  trajectory_msgs
  ariac_msgs
  tf2_kdl
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Ariac Ray Plugin
add_library(AriacRayPlugin SHARED
  src/ariac_ray_plugin.cpp
)
target_include_directories(AriacRayPlugin PUBLIC include)
ament_target_dependencies(AriacRayPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(AriacRayPlugin)

# Ariac Camera Plugin
add_library(AriacCameraPlugin SHARED
  src/ariac_camera_plugin.cpp
)
target_include_directories(AriacCameraPlugin PUBLIC include)
ament_target_dependencies(AriacCameraPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(AriacCameraPlugin)

# Ariac Logical Camera Plugin
add_library(AriacLogicalCameraPlugin SHARED
  src/ariac_logical_camera_plugin.cpp
)
target_include_directories(AriacLogicalCameraPlugin PUBLIC include)
ament_target_dependencies(AriacLogicalCameraPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(AriacLogicalCameraPlugin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    AriacRayPlugin
    AriacCameraPlugin
    AriacLogicalCameraPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)