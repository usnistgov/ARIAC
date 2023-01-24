cmake_minimum_required(VERSION 3.8)
project(ariac_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Ariac Ray Plugin
add_library(AriacRayPlugin SHARED
  src/ariac_ray_plugin.cpp
)
target_include_directories(AriacRayPlugin PUBLIC include)
ament_target_dependencies(AriacRayPlugin
  "gazebo_ros"
  "sensor_msgs"
  "ariac_msgs"
)
ament_export_libraries(AriacRayPlugin)

# Ariac Camera Plugin
add_library(AriacCameraPlugin SHARED
  src/ariac_camera_plugin.cpp
)
target_include_directories(AriacCameraPlugin PUBLIC include)
ament_target_dependencies(AriacCameraPlugin
  "gazebo_ros"
  "sensor_msgs"
  "ariac_msgs"
  "camera_info_manager"
  "image_transport"
)
ament_export_libraries(AriacCameraPlugin)

# Ariac Logical Camera Plugin
add_library(AriacLogicalCameraPlugin SHARED
  src/ariac_logical_camera_plugin.cpp
)
target_include_directories(AriacLogicalCameraPlugin PUBLIC include)
ament_target_dependencies(AriacLogicalCameraPlugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(AriacLogicalCameraPlugin)

# AGV Tray Sensor Plugin
add_library(AGVTraySensorPlugin SHARED
  src/agv_tray_sensor_plugin.cpp
)
target_include_directories(AGVTraySensorPlugin PUBLIC include)
ament_target_dependencies(AGVTraySensorPlugin
  "orocos_kdl"
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(AGVTraySensorPlugin)

# Assembly Station Sensor Plugin
add_library(AssemblyStationSensorPlugin SHARED
  src/assembly_station_sensor_plugin.cpp
)
target_include_directories(AssemblyStationSensorPlugin PUBLIC include)
ament_target_dependencies(AssemblyStationSensorPlugin
  "gazebo_ros"
  "ariac_msgs"
)
ament_export_libraries(AssemblyStationSensorPlugin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    AGVTraySensorPlugin
    AssemblyStationSensorPlugin
    AriacRayPlugin
    AriacCameraPlugin
    AriacLogicalCameraPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)