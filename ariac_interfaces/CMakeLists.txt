cmake_minimum_required(VERSION 3.8)
project(ariac_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)

set(action_files
  "action/MoveAgv.action"
)

set(msg_files
  "msg/AgvStations.msg"
  "msg/AgvStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}

  DEPENDENCIES
  builtin_interfaces
  geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
