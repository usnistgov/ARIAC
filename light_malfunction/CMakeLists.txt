cmake_minimum_required(VERSION 3.8)
project(Light_Malfunction_Challenge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(include/light_malfunction)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(light_malfunction_plugin SHARED src/light_malfunction_plugin.cpp)
target_link_libraries(light_malfunction_plugin ${GAZEBO_LIBRARIES})
