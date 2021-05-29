cmake_minimum_required(VERSION 2.8.3)
project(nist_gear)

# This package was developed with gazebo7, but now uses gazebo8.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  std_srvs
  tf
  geometry_msgs
  message_generation
)

find_package(gazebo REQUIRED)

########################
## Message generation ##
########################

add_message_files(DIRECTORY msg
  FILES
  RobotHealth.msg
  ConveyorBeltState.msg
  DetectedProduct.msg
  KittingShipment.msg
  AssemblyShipment.msg
  Product.msg
  DetectedKittingShipment.msg
  DetectedAssemblyShipment.msg
  LogicalCameraImage.msg
  Model.msg
  DropProducts.msg
  DropProduct.msg
  Order.msg
  PopulationState.msg
  Proximity.msg
  StorageUnit.msg
  VacuumGripperState.msg
)

add_service_files(DIRECTORY srv
  FILES
  AGVControl.srv
  AGVToAssemblyStation.srv
  AGVToKittingStation.srv
  DroneControl.srv
  DetectKittingShipment.srv
  DetectAssemblyShipment.srv
  AssemblyStationSubmitShipment.srv
  ConveyorBeltControl.srv
  GetMaterialLocations.srv
  PopulationControl.srv
  SubmitShipment.srv
  SubmitTray.srv
  VacuumGripperControl.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_srvs
  tf
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/VacuumGripperPlugin.cc)
target_link_libraries(${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${vacuum_gripper_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  

# Create the libROSVacuumGripperPlugin.so library.
set(ros_vacuum_gripper_plugin_name ROSVacuumGripperPlugin)
add_library(${ros_vacuum_gripper_plugin_name} src/ROSVacuumGripperPlugin.cc)
target_link_libraries(${ros_vacuum_gripper_plugin_name}
  ${catkin_LIBRARIES}
  ${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_vacuum_gripper_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Create the libSensorBlackoutPlugin.so library.
set(sensor_blackout_plugin_name SensorBlackoutPlugin)
add_library(${sensor_blackout_plugin_name} src/SensorBlackoutPlugin.cc)
target_link_libraries(${sensor_blackout_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${sensor_blackout_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Create the libToggleVisualPlugin.so library.
set(toggle_visual_plugin_name ToggleVisualPlugin)
add_library(${toggle_visual_plugin_name} src/ToggleVisualPlugin.cc)
target_link_libraries(${toggle_visual_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${toggle_visual_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libLogPlaybackPlugin.so library.
set(logplayback_plugin_name LogPlaybackPlugin)
add_library(${logplayback_plugin_name} src/LogPlaybackPlugin.cc)
target_link_libraries(${logplayback_plugin_name} ${GAZEBO_LIBRARIES})
install(TARGETS ${logplayback_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Create the libSideContactPlugin.so library.
set(side_contact_plugin_name SideContactPlugin)
add_library(${side_contact_plugin_name} src/SideContactPlugin.cc)
target_link_libraries(${side_contact_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${side_contact_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libURDFContactPlugin.so library.
# set(urdf_contact_plugin_name URDFContactPlugin)
# add_library(${urdf_contact_plugin_name} src/URDFContactPlugin.cc)
# target_link_libraries(${urdf_contact_plugin_name}
#   ${GAZEBO_LIBRARIES}
# )
# install(TARGETS ${urdf_contact_plugin_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Create the libROSAGVPlugin.so library.
set(ros_agv_plugin_name ROSAGVPlugin)
add_library(${ros_agv_plugin_name} src/ROSAGVPlugin.cc)
target_link_libraries(${ros_agv_plugin_name} ${GAZEBO_LIBRARIES})
add_dependencies(${ros_agv_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_agv_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSPersonPlugin.so library.
# set(ros_person_plugin_name ROSPersonPlugin)
# add_library(${ros_person_plugin_name} src/ROSPersonPlugin.cc)
# target_link_libraries(${ros_person_plugin_name} ${GAZEBO_LIBRARIES})
# add_dependencies(${ros_person_plugin_name}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# install(TARGETS ${ros_person_plugin_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Create the libConveyorBeltPlugin.so library.
set(conveyor_belt_plugin_name ConveyorBeltPlugin)
add_library(${conveyor_belt_plugin_name} src/ConveyorBeltPlugin.cc)
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
install(TARGETS ${conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSConveyorBeltPlugin.so library.
set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)
add_library(${ros_conveyor_belt_plugin_name} src/ROSConveyorBeltPlugin.cc)
target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libConveyorCongestionPlugin.so library.
set(conveyor_congestion_plugin ConveyorCongestionPlugin)
add_library(${conveyor_congestion_plugin} src/ConveyorCongestionPlugin.cc)
target_link_libraries(${conveyor_congestion_plugin}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${conveyor_congestion_plugin}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${conveyor_congestion_plugin}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libProximityRayPlugin.so library.
set(proximity_ray_plugin_name ProximityRayPlugin)
add_library(${proximity_ray_plugin_name} src/ProximityRayPlugin.cc)
target_link_libraries(${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSProximityRayPlugin.so library.
set(ros_proximity_ray_plugin_name ROSProximityRayPlugin)
add_library(${ros_proximity_ray_plugin_name} src/ROSProximityRayPlugin.cc)
target_link_libraries(${ros_proximity_ray_plugin_name}
  ${proximity_ray_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_proximity_ray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_proximity_ray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libPopulationPlugin.so library.
set(population_plugin_name PopulationPlugin)
add_library(${population_plugin_name} src/PopulationPlugin.cc)
target_link_libraries(${population_plugin_name}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS ${population_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSPopulationPlugin.so library.
set(ros_population_plugin_name ROSPopulationPlugin)
add_library(${ros_population_plugin_name} src/ROSPopulationPlugin.cc)
target_link_libraries(${ros_population_plugin_name}
  ${catkin_LIBRARIES}
  ${population_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${ros_population_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_population_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAriacTaskManagerPlugin.so library.
set(task_plugin_name ROSAriacTaskManagerPlugin)
add_library(${task_plugin_name}
  src/ROSAriacTaskManagerPlugin.cc
)
target_link_libraries(${task_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  AriacScorer
)
add_dependencies(${task_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${task_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSLaserPlugin.so library.
set(ros_laser_plugin_name ROSLaserPlugin)
add_library(${ros_laser_plugin_name} src/ROSLaserPlugin.cc)
target_link_libraries(${ros_laser_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  RayPlugin
)
add_dependencies(${ros_laser_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_laser_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSLogicalCameraPlugin.so library.
set(ros_logical_camera_plugin_name ROSLogicalCameraPlugin)
add_library(${ros_logical_camera_plugin_name} src/ROSLogicalCameraPlugin.cc)
target_link_libraries(${ros_logical_camera_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${ros_logical_camera_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_logical_camera_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libObjectDisposalPlugin.so library.
set(object_disposal_plugin_name ObjectDisposalPlugin)
add_library(${object_disposal_plugin_name} src/ObjectDisposalPlugin.cc)
target_link_libraries(${object_disposal_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
install(TARGETS ${object_disposal_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSAriacKitTrayPlugin.so library.
set(kit_tray_plugin_name ROSAriacKitTrayPlugin)
add_library(${kit_tray_plugin_name} src/ROSAriacKitTrayPlugin.cc)
target_link_libraries(${kit_tray_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${side_contact_plugin_name}
)
add_dependencies(${kit_tray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${kit_tray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSAriacBriefcasePlugin.so library.
set(briefcase_plugin_name ROSAriacBriefcasePlugin)
add_library(${briefcase_plugin_name} src/ROSAriacBriefcasePlugin.cc)
target_link_libraries(${briefcase_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${side_contact_plugin_name}
)
add_dependencies(${briefcase_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${briefcase_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




# Create the libROSAriacGantryTrayPlugin.so library.
set(gantry_tray_plugin_name ROSAriacGantryTrayPlugin)
add_library(${gantry_tray_plugin_name} src/ROSAriacGantryTrayPlugin.cc)
target_link_libraries(${gantry_tray_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${side_contact_plugin_name}
)
add_dependencies(${gantry_tray_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${gantry_tray_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libAssemblyPlugin.so library.
set(assembly_plugin_name AssemblyPlugin)
add_library(${assembly_plugin_name} src/AssemblyPlugin.cc)
target_link_libraries(${assembly_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${assembly_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${assembly_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# Create the libAriacScorer.so library.
set(target_name AriacScorer)
add_library(${target_name} src/AriacScorer.cpp)
target_link_libraries(${target_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_dependencies(${target_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(tf2_relay src/tf2_relay.cpp)
target_link_libraries(tf2_relay
  ${catkin_LIBRARIES}
)
target_include_directories(tf2_relay
  PRIVATE
  ${catkin_INCLUDE_DIRS}
)
install(TARGETS tf2_relay
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Install ##
#############

# message("CATKIN_PACKAGE_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}")
# message("CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}")
# message("CATKIN_PACKAGE_SHARE_DESTINATION" ${CMAKE_CURRENT_SOURCE_DIR})

install(PROGRAMS script/gear.py script/gazebo_unpauser.py script/startup.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the rviz files
install(DIRECTORY rviz/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

# Configure the setup script
catkin_add_env_hooks(99_nist_gear_setup
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  SHELLS sh)

################
## Unit tests ##
################
#temporarily disabled
#if (CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(test_ariac_scorer test/test_ariac_scorer.cpp)
#  target_link_libraries(test_ariac_scorer
#    AriacScorer ignition-math4::ignition-math4)
#endif()
