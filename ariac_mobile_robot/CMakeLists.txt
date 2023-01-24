cmake_minimum_required(VERSION 3.8)
project(ariac_mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

install(DIRECTORY launch models urdf
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
