cmake_minimum_required(VERSION 3.8)
project(ariac_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AdvancedLogicalCameraImage.msg"
  "msg/AGVStatus.msg"
  "msg/AssemblyPart.msg"
  "msg/AssemblyStationState.msg"
  "msg/AssemblyTask.msg"
  "msg/AssemblyState.msg"
  "msg/BasicLogicalCameraImage.msg"
  "msg/BinInfo.msg"
  "msg/BinParts.msg"
  "msg/BreakBeamStatus.msg"
  "msg/Condition.msg"
  "msg/Challenge.msg"
  "msg/CombinedTask.msg"
  "msg/CompetitionState.msg"
  "msg/ConveyorBeltState.msg"
  "msg/ConveyorParts.msg"
  "msg/DroppedPartChallenge.msg"
  "msg/FaultyPartChallenge.msg"
  "msg/HumanChallenge.msg"
  "msg/HumanState.msg"
  "msg/KittingPart.msg"
  "msg/KittingTask.msg"
  "msg/KitTrayPose.msg"
  "msg/Order.msg"
  "msg/OrderCondition.msg"
  "msg/PartLot.msg"
  "msg/Part.msg"
  "msg/PartPlaceCondition.msg"
  "msg/PartPose.msg"
  "msg/Parts.msg"
  "msg/QualityIssue.msg"
  "msg/RobotMalfunctionChallenge.msg"
  "msg/Robots.msg"
  "msg/SensorBlackoutChallenge.msg"
  "msg/Sensors.msg"
  "msg/SubmissionCondition.msg"
  "msg/TimeCondition.msg"
  "msg/Trial.msg"
  "msg/VacuumGripperState.msg"
)

set(srv_files
  "srv/ChangeGripper.srv"
  "srv/ChangeGripperColor.srv"
  "srv/ConveyorBeltControl.srv"
  "srv/MoveAGV.srv"
  "srv/ScoreTask.srv"
  "srv/SubmitOrder.srv"
  "srv/VacuumGripperControl.srv"
  "srv/PerformQualityCheck.srv"
  "srv/GetPreAssemblyPoses.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs

  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
