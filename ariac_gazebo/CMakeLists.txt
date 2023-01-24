cmake_minimum_required(VERSION 3.5)
project(ariac_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(
  include/ariac_gazebo
  ${GAZEBO_INCLUDE_DIRS}
)

# Create Cpp executable
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Cpp executables
install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Plugins

# add_library(SideContactPlugin SHARED src/SideContactPlugin.cc)

# target_include_directories(SideContactPlugin PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# add_library(ObjectDisposalPlugin SHARED src/ObjectDisposalPlugin.cc)

# target_include_directories(ObjectDisposalPlugin PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# install(TARGETS
#   SideContactPlugin
#   ObjectDisposalPlugin
#   DESTINATION share/${PROJECT_NAME})


# set(side_contact_plugin_name SideContactPlugin)
# add_library(${side_contact_plugin_name} src/SideContactPlugin.cc)
# target_link_libraries(${side_contact_plugin_name}
#   ${GAZEBO_LIBRARIES}
# )
# install(TARGETS ${side_contact_plugin_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# set(object_disposal_plugin_name ObjectDisposalPlugin)
# add_library(${object_disposal_plugin_name} src/ObjectDisposalPlugin.cc)
# target_link_libraries(${object_disposal_plugin_name}
#   ${GAZEBO_LIBRARIES}
#   ${side_contact_plugin_name}
# )
# install(TARGETS ${object_disposal_plugin_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/environment_startup_node.py
  nodes/robot_controller_switcher_node.py
  nodes/sensor_tf_broadcaster.py
  nodes/object_tf_broadcaster.py
  nodes/part_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()