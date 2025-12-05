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
  "action/GripperCommand.action"
  "action/MoveAgv.action"
)

set(msg_files
  "msg/AgvStations.msg"
  "msg/AgvStatus.msg"
  "msg/AgvTrayStatus.msg"
  "msg/BreakBeamStatus.msg"
  "msg/DistanceSensor.msg"
  "msg/CellDefect.msg"
  "msg/CellFeederStatus.msg"
  "msg/CellTypes.msg"
  "msg/CompetitionStates.msg"
  "msg/CompetitionStatus.msg"
  "msg/CompetitionTime.msg"
  "msg/ConveyorStatus.msg"
  "msg/HighPriorityOrder.msg"
  "msg/InspectionReport.msg"
  "msg/OperationStates.msg"
  "msg/ToolChangerStatus.msg"
  "msg/VacuumTools.msg"
  "msg/VoltageReading.msg"
)

set(srv_files
  "srv/AttachTool.srv"
  "srv/BidirectionalConveyorControl.srv"
  "srv/CheckKitQuality.srv"
  "srv/ControlCellFeeder.srv"
  "srv/ConveyorControl.srv"
  "srv/EndCompetition.srv"
  "srv/SubmitHighPriorityOrder.srv"
  "srv/SubmitInspectionReport.srv"
  "srv/Trigger.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
