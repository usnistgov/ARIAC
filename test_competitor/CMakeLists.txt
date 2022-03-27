cmake_minimum_required(VERSION 3.0.2)
project(test_competitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Part.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectObjects.srv
  MoveGantry.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

catkin_install_python(
  PROGRAMS
  nodes/object_detector_node.py
  nodes/competitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

