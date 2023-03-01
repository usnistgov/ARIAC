cmake_minimum_required(VERSION 3.8)
project(ariac_human)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(rosbridge_server REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY 
  agent
  launch
  config
  map
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Agent 
install(PROGRAMS
  agent/gradlew
  DESTINATION share/${PROJECT_NAME}/agent
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/human_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
