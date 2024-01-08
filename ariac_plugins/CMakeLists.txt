cmake_minimum_required(VERSION 3.8)
project(ariac_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  gazebo_msgs
  gazebo_ros
  rclcpp
  ariac_msgs
  geometry_msgs
  std_msgs
  std_srvs
  controller_manager_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_kdl
  orocos_kdl
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Vacuum Gripper
add_library(VacuumGripperPlugin SHARED
  src/vacuum_gripper_plugin.cpp
)
target_include_directories(VacuumGripperPlugin PUBLIC include)
ament_target_dependencies(VacuumGripperPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(VacuumGripperPlugin)

# Conveyor Belt Plugin
add_library(conveyor_belt_plugin SHARED
  src/conveyor_belt_plugin.cpp
)
target_include_directories(conveyor_belt_plugin PUBLIC include)
ament_target_dependencies(conveyor_belt_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(conveyor_belt_plugin)

# AGV Plugin
add_library(agv_plugin SHARED
  src/agv_plugin.cpp
)
target_include_directories(agv_plugin PUBLIC include)
ament_target_dependencies(agv_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(agv_plugin)

# Assembly Lock Plugin
add_library(assembly_lock_plugin SHARED
  src/assembly_lock_plugin.cpp
)
target_include_directories(assembly_lock_plugin PUBLIC include)
ament_target_dependencies(assembly_lock_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(assembly_lock_plugin)

# Assembly State Publisher
add_library(assembly_state_publisher SHARED
  src/assembly_state_publisher.cpp
)
target_include_directories(assembly_state_publisher PUBLIC include)
ament_target_dependencies(assembly_state_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(assembly_state_publisher)

# Gripper Change Plugin
add_library(GripperColorPlugin SHARED
  src/gripper_color_plugin.cpp
)
target_include_directories(GripperColorPlugin PUBLIC include)
ament_target_dependencies(GripperColorPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(GripperColorPlugin)

# Disable Shadows Plugin
add_library(disable_shadows_plugin SHARED
  src/disable_shadows_plugin.cpp
)
target_include_directories(disable_shadows_plugin PUBLIC include)
ament_target_dependencies(disable_shadows_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(disable_shadows_plugin)

# Object Disposal Plugin
add_library(ObjectDisposalPlugin SHARED
  src/object_disposal_plugin.cpp
)
target_include_directories(ObjectDisposalPlugin PUBLIC include)
ament_target_dependencies(ObjectDisposalPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(ObjectDisposalPlugin)

# Tray Plugin
add_library(AGVTrayPlugin SHARED
  src/agv_tray_plugin.cpp
)
target_include_directories(AGVTrayPlugin PUBLIC include)
ament_target_dependencies(AGVTrayPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(AGVTrayPlugin)


# Task Manager Plugin
add_library(TaskManagerPlugin SHARED
  src/task_manager_plugin.cpp
)
target_include_directories(TaskManagerPlugin PUBLIC include)
ament_target_dependencies(TaskManagerPlugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(TaskManagerPlugin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    VacuumGripperPlugin
    conveyor_belt_plugin
    agv_plugin
    assembly_lock_plugin
    assembly_state_publisher
    GripperColorPlugin
    disable_shadows_plugin
    ObjectDisposalPlugin
    AGVTrayPlugin
    TaskManagerPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
