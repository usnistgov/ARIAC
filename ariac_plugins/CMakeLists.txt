cmake_minimum_required(VERSION 3.8)
project(ariac_plugins)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  gz-plugin2
  gz-cmake3
  gz-sim8
  gz-sensors8
  rclcpp
  rclcpp_action
  ros_gz_bridge
  tf2_ros
  ariac_interfaces
  geometry_msgs
  controller_manager_msgs
  angles
  rclpy
  sensor_msgs
  yaml-cpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# PathVelocityPlanner
add_library(PathVelocityPlanner SHARED src/velocity_planner.cpp)
target_include_directories(PathVelocityPlanner PUBLIC include)

# Database Manager
find_package(SQLite3 REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(DatabaseManager SHARED src/db_manager.cpp)
target_include_directories(DatabaseManager 
  PUBLIC 
    include
    ${SQLite3_INCLUDE_DIRS})

target_link_libraries(DatabaseManager
  PUBLIC
    sqlite3
    OpenSSL::SSL
    OpenSSL::Crypto
)

# AGV Motion Plugin
add_library(AgvMotionPlugin SHARED src/agv_motion_plugin.cpp)
target_include_directories(AgvMotionPlugin PUBLIC include)
ament_target_dependencies(AgvMotionPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(AgvMotionPlugin PathVelocityPlanner)

# AGV Tray Interface Plugin
add_library(AgvTrayInterfacePlugin SHARED src/agv_tray_interface_plugin.cpp)
target_include_directories(AgvTrayInterfacePlugin PUBLIC include)

ament_target_dependencies(AgvTrayInterfacePlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Bottom Shell Plugin
add_library(BottomShellPlugin SHARED src/bottom_shell_plugin.cpp)
target_include_directories(BottomShellPlugin PUBLIC include)

ament_target_dependencies(BottomShellPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Cell Feed Plugin
add_library(CellFeedPlugin SHARED src/cell_feed_plugin.cpp)
target_include_directories(CellFeedPlugin PUBLIC include)

ament_target_dependencies(CellFeedPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(CellFeedPlugin
  yaml-cpp
)

# Cheat Tools Plugin
add_library(CheatToolsPlugin SHARED src/cheat_tools_plugin.cpp)
target_include_directories(CheatToolsPlugin PUBLIC include)

ament_target_dependencies(CheatToolsPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Competition Manager Plugin
add_library(CompetitionManagerPlugin SHARED src/competition_manager_plugin.cpp)
target_include_directories(CompetitionManagerPlugin PUBLIC include)
ament_target_dependencies(CompetitionManagerPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(CompetitionManagerPlugin
  DatabaseManager
  yaml-cpp
)

# Contact Plugin
add_library(ContactPlugin SHARED src/contact_plugin.cpp)
target_include_directories(ContactPlugin PUBLIC include)

ament_target_dependencies(ContactPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Entity Disposal Plugin
add_library(EntityDisposalPlugin SHARED src/entity_disposal_plugin.cpp)
target_include_directories(EntityDisposalPlugin PUBLIC include)

ament_target_dependencies(EntityDisposalPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Image Plugin
add_library(ImagePlugin SHARED src/image_plugin.cpp)
target_include_directories(ImagePlugin PUBLIC include)

ament_target_dependencies(ImagePlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Kit Tray Plugin
add_library(KitTrayPlugin SHARED src/kit_tray_plugin.cpp)
target_include_directories(KitTrayPlugin PUBLIC include)

ament_target_dependencies(KitTrayPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Linear Conveyor Plugin
add_library(LinearConveyorPlugin SHARED src/linear_conveyor_plugin.cpp)
target_include_directories(LinearConveyorPlugin PUBLIC include)

ament_target_dependencies(LinearConveyorPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Physical Inspection Plugin
add_library(PhysicalInspectionPlugin SHARED src/physical_inspection_plugin.cpp)
target_include_directories(PhysicalInspectionPlugin PUBLIC include)

ament_target_dependencies(PhysicalInspectionPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(PhysicalInspectionPlugin
  yaml-cpp
)

# Ray Sensor Plugin
add_library(RaySensorPlugin SHARED src/ray_sensor_plugin.cpp)
target_include_directories(RaySensorPlugin PUBLIC include)

ament_target_dependencies(RaySensorPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Robot Collision Plugin
add_library(RobotCollisionPlugin SHARED src/robot_collision_plugin.cpp)
target_include_directories(RobotCollisionPlugin PUBLIC include)

ament_target_dependencies(RobotCollisionPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Shell Insertion Plugin
add_library(ShellInsertionPlugin SHARED src/shell_insertion_plugin.cpp)
target_include_directories(ShellInsertionPlugin PUBLIC include)

ament_target_dependencies(ShellInsertionPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Status Light Plugin
add_library(StatusLightPlugin SHARED src/status_light_plugin.cpp)
target_include_directories(StatusLightPlugin PUBLIC include)

ament_target_dependencies(StatusLightPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# TF Publisher Plugin
add_library(TFPublisherPlugin SHARED src/tf_publisher_plugin.cpp)
target_include_directories(TFPublisherPlugin PUBLIC include)

ament_target_dependencies(TFPublisherPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Tool Changer Plugin
add_library(ToolChangerPlugin SHARED src/tool_changer_plugin.cpp)
target_include_directories(ToolChangerPlugin PUBLIC include)

ament_target_dependencies(ToolChangerPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Tool Stand Plugin
add_library(ToolStandPlugin SHARED src/tool_stand_plugin.cpp)
target_include_directories(ToolStandPlugin PUBLIC include)

ament_target_dependencies(ToolStandPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Top Shell Plugin
add_library(TopShellPlugin SHARED src/top_shell_plugin.cpp)
target_include_directories(TopShellPlugin PUBLIC include)

ament_target_dependencies(TopShellPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Vacuum Tool Plugin
add_library(VacuumToolPlugin SHARED src/vacuum_tool_plugin.cpp)
target_include_directories(VacuumToolPlugin PUBLIC include)

ament_target_dependencies(VacuumToolPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Voltage Tester Plugin
add_library(VoltageTesterPlugin SHARED src/voltage_tester_plugin.cpp)
target_include_directories(VoltageTesterPlugin PUBLIC include)

ament_target_dependencies(VoltageTesterPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Welder Plugin
add_library(WelderPlugin SHARED src/welder_plugin.cpp)
target_include_directories(WelderPlugin PUBLIC include)

ament_target_dependencies(WelderPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


ament_export_libraries(
  AgvMotionPlugin
  AgvTrayInterfacePlugin
  BottomShellPlugin
  CellFeedPlugin
  CheatToolsPlugin
  CompetitionManagerPlugin
  ContactPlugin
  DatabaseManager
  EntityDisposalPlugin
  ImagePlugin
  KitTrayPlugin
  LinearConveyorPlugin
  PathVelocityPlanner
  PhysicalInspectionPlugin
  RaySensorPlugin
  RobotCollisionPlugin
  ShellInsertionPlugin
  StatusLightPlugin
  TFPublisherPlugin
  ToolChangerPlugin
  ToolStandPlugin
  TopShellPlugin
  VacuumToolPlugin
  VoltageTesterPlugin
  WelderPlugin
)

install(
  TARGETS
    PathVelocityPlanner
    DatabaseManager
  DESTINATION 
    lib
)

install(
  TARGETS
    AgvMotionPlugin
    AgvTrayInterfacePlugin
    BottomShellPlugin
    CellFeedPlugin
    CheatToolsPlugin
    CompetitionManagerPlugin
    ContactPlugin
    EntityDisposalPlugin
    ImagePlugin
    KitTrayPlugin
    LinearConveyorPlugin
    PhysicalInspectionPlugin
    RaySensorPlugin
    RobotCollisionPlugin
    ShellInsertionPlugin
    StatusLightPlugin
    TFPublisherPlugin
    ToolChangerPlugin
    ToolStandPlugin
    TopShellPlugin
    VacuumToolPlugin
    VoltageTesterPlugin
    WelderPlugin
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
