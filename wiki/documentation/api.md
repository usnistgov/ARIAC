Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

- [Wiki | Documentation |API](#wiki--documentation-api)
  - [Sensors](#sensors)
  - [TF frames](#tf-frames)
  - [Robot Status and Control](#robot-status-and-control)
  - [Process Management](#process-management)
  - [Cheats](#cheats)

# Wiki | Documentation |API

* GEAR provides a ROS interface to the competitors participating in ARIAC. This interface can be used by competitors to control all available actuators, read sensor information, and send/receive notifications.
* During the competition, it is against the rules to control the ARIAC simulation using anything other than the interface specified below.
* **Teams are not permitted to use any of the topics or services in the `/gazebo/` namespace prefix to control the Gazebo simulation or get information about the simulation state. These interfaces will be blocked during the Finals.**



## Sensors

In the following, `{sensor_name}` is replaced with the name you give the sensor in the user configuration file (see [Configuration of the environment and trials](configuration_spec.md)). Since the sensor names are unique, it ensures that all sensors publish data to unique topics.

- Breakbeam 
  - type: `break_beam`
  - topic: `/ariac/{sensor_name}`
    - msg: [nist_gear/Proximity.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg) 
  - topic: `/ariac/{sensor_name}_change`
    - msg: [nist_gear/Proximity.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg) 
- Proximity 
  - type: `proximity_sensor`
  - topic: `/ariac/{sensor_name}`
    - msg: [sensor_msgs/Range.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)    
- Laser Profiler
  - type: `laser_profiler`
  - topic: `/ariac/{sensor_name}`
    - msg: [sensor_msgs/LaserScan.msg](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html) 
- Depth Camera
  - type: `depth_camera`
  - topic: `/ariac/{sensor_name}/depth/camera_info`
    - msg: [sensor_msgs/CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)
  - topic: `/ariac/{sensor_name}/depth/image_raw`
    - msg: [sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html) 
  - topic: `/ariac/gantry/gantry_bin_camera/color/image_raw`
    - msg: [sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html) 
  - topic: `/ariac/gantry/gantry_tray_camera/color/image_raw`
    - msg: [sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html) 
- Logical Camera
  - type: `logical_camera`
  - topic: `/ariac/{sensor_name}`
    - msg: [nist_gear/LogicalCameraImage](../../nist_gear/msg/LogicalCameraImage.msg)
- Quality Control Sensor
  - type: Not customizable
  - topic: `/ariac/quality_control_sensor_N`, where `N` is a number between 1 and 4.
    - msg: [nist_gear/LogicalCameraImage](../../nist_gear/msg/LogicalCameraImage.msg)


## TF frames

TF frames for static key points of the workcell are published by the ARIAC simulation. Dynamic TF frames for the robot and faulty products are also published by the simulation. The following frames are published on the global `/tf` and `/tf_static` topics.

- World
  - Origin of the workcell
  - frame: `world`
  - type: static
- Bin frame x 8
  - frame: `bin{N}_frame`, where `N` is a number between 1 and 8.
  - type: static
- Robot link frames
  - frame: {link_name}, 18 frames for the gantry robot and 12 frames for the kitting robot.
  - type: static and dynamic
- Each part and each movable tray detected by a logical camera has a frame, which is a child frame of the logical camera frame.
  - frame: The name of the frame is a concatenation of the logical camera name and the name of the part or movable tray detected. 
  - type: dynamic.
- Each faulty part detected by a quality control sensor has its own frame, which is a child of the quality control sensor frame.
  - frame: `quality_control_sensor_{N}_{anonymize_mode_name}_frame`, where `N` is a number between 1 and 4.
- The frame of a fixed kit tray attach to an AGV is published.
  - frame: `kit_tray_{N}` where `N` is a number between 1 and 4.
  - type: dynamic
- Briefcases located at assembly stations.
  - frame: `briefcase_{N}`, where `N` is a number between 1 and 4.
  - type: static
- Tables where movable trays are located
  - frame: `tray_table1_frame` and `tray_table2_frame`.
  - type: static


## Robot Status and Control
Topic and services that can be used to control the robot actuators are described in this section


- Command the arm of the gantry robot.
  - topic: `/ariac/gantry/gantry_arm_controller/command`
    - msg: [trajectory_msgs/JointTrajectory.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
- Command the gantry torso, the small rail, and the long rail.
  - topic: `/ariac/gantry/gantry_controller/command`
    - msg: [trajectory_msgs/JointTrajectory.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
- Command the arm of the kitting robot.
  - topic: `/ariac/kitting/kitting_arm_controller/command`
    - msg: [trajectory_msgs/JointTrajectory.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
- Get the joint states of the gantry robot.
  - topic: `/ariac/gantry/joint_states`
    - type: [sensor_msgs/JointState.msg](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- Get the joint states of the kitting robot.
  - topic: `/ariac/kitting/joint_states`
    - msg: [sensor_msgs/JointState.msg](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
- Get the state of the controllers for the gantry robot.
  - topic: `/ariac/gantry/gantry_controller/state`
    - msg: [control_msgs/JointTrajectoryControllerState.msg](https://docs.ros.org/en/noetic/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
  - topic: `/ariac/gantry/gantry_arm_controller/state`
    - msg: [control_msgs/JointTrajectoryControllerState.msg](https://docs.ros.org/en/noetic/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
- Get the state of the controllers for the kitting arm.
  - topic: `/ariac/kitting/kitting_arm_controller/state`
    - msg: [control_msgs/JointTrajectoryControllerState.msg](https://docs.ros.org/en/noetic/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
- Get the state of the conveyor belt.
  - topic: `/ariac/conveyor/state`
    - msg: [nist_gear/ConveyorBeltState.msg](../../nist_gear/msg/ConveyorBeltState.msg)
- Get the state of the gantry arm gripper.
  - topic: `/ariac/gantry/arm/gripper/state`
    - msg: [nist_gear/VacuumGripperState.msg](../../nist_gear/msg/VacuumGripperState.msg) 
- Get the state of the kitting arm gripper.
  - topic: `/ariac/kitting/arm/gripper/state`
    - msg: [nist_gear/VacuumGripperState.msg](../../nist_gear/msg/VacuumGripperState.msg) 
- Activate/deactivate the gantry arm gripper.
  - service: `ariac/gantry/arm/gripper/control`
    - srv: [nist_gear/VacuumGripperControl.srv](../../nist_gear/srv/VacuumGripperControl.srv)
- Activate/deactivate the kitting arm gripper.
  - service: `ariac/kitting/arm/gripper/control`
    - srv: [nist_gear/VacuumGripperControl.srv](../../nist_gear/srv/VacuumGripperControl.srv)
- Get the type of gripper attached to the gantry arm.
  - topic: `/ariac/gantry/arm/gripper/type`
    - msg: [std_msgs/String.msg](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) 
- Change the gripper for the gantry arm.
  - service: `/ariac/gantry/arm/gripper/change`
    - srv: [nist_gear/ChangeGripper.srv](../../nist_gear/srv/ChangeGripper.srv) 
- Change the health status of both robots.
  - topic: `echo /ariac/robot_health `
    - msg: [nist_gear/RobotHealth.msg](../../nist_gear/msg/RobotHealth.msg) 


## Process Management

The topics and services describe in this section should be used to interact with the GEAR interface.

- Simulation time
  - topic: `/clock`
    - msg: [rosgraph_msgs/Clock.msg](http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html)
- Start the competition.
  - service: `/ariac/start_competition`
    - srv: [std_srvs/Trigger.srv](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)
- End the competition.
  - service: `/ariac/end_competition`
    - srv: [std_srvs/Trigger.srv](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)
- Get the state of the competition (init, ready, go, end_game, or done).
  - topic: `/ariac/competition_state`
    - msg: [std_msgs/String.msg](http://docs.ros.org/api/std_msgs/html/msg/String.html)
- Get the current order to be completed.
  - topic: `/ariac/orders`
    - msg: [nist_gear/Orders](../../nist_gear/msg/Orders.msg)
- Submit a kitting shipment.
  - service: `/ariac/agv{N}/submit_shipment {assembly_station} {shipment_type}`
    - srv: [nist_gear/SubmitKittingShipment.srv](../../nist_gear/srv/SubmitKittingShipment.srv)
- Submit an assembly shipment.
  - service: `/ariac/as{N}/submit_assembly_shipment`
    - srv: [nist_gear/AssemblyStationSubmitShipment.srv](../../nist_gear/srv/AssemblyStationSubmitShipment.srv)
- Get the current station of the AGV. This will return `ks1`, `as2`. etc.
  - topic: `rostopic echo /ariac/agv{N}/station`
    - msg: [std_msgs/String.msg](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) 
- Get the current state of the AGV. This will report whether the AGV is ready or not to be submitted.
  - topic: `rostopic echo /ariac/agv{N}/state`
    - msg: [std_msgs/String.msg](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) 
- Move an AGV to a station without submitting a shipment.
  - service: `/ariac/agv{N}/move_to_station {station_name}`.
    - **Note**: To move an AGV to its kitting station, `{station_name}` is `ks`
    - srv: [nist_gear/srv/MoveToStation.srv](../../nist_gear/srv/MoveToStation.srv) 
- Lock/unlock any movable tray located on an AGV (where `N` can take a value between 1 and 4).
  - service to lock: `rosservice call /ariac/kit_tray_{N}/lock`
    - Example: `rosservice call /ariac/kit_tray_1/lock` locks the movable tray located on AGV1.
  - service to unlock: `rosservice call /ariac/kit_tray_{N}/unlock`
    - Example: `rosservice call /ariac/kit_tray_1/unlock` unlocks the movable tray located on AGV1.
- Get the parts connected (assembled)to a briefcase (where `N` can take a value between 1 and 4).
  - service: `rosservice call /ariac/briefcase_{N}/get_assembled_parts`
    - srv: [nist_gear/DetectConnectedPartsToBriefcase](../../nist_gear/srv/nist_gear/DetectConnectedPartsToBriefcase.srv)

## Cheats

These are only provided for debugging/development purposes and their use is not permitted during the competition trials.
- Get the content of an AGV.
  - service: `ariac/agv{N}/content`
- Get the current completion score in a trial.
  - topic: `/ariac/current_score`
- Get the state of each kit being built on each movable tray.
  - topic: `/ariac/trays`
- Get the state of each assembly being performed an each assembly station.
  - topic: `/ariac/briefcases`
- Get the storage locations for a specific part or movable tray mode.
  - service: `/ariac/material_locations {model}`
```bash
  rosservice call /ariac/material_locations 'assembly_pump_red'

  storage_units: 
  - 
    unit_id: "bin2"

```
- Control the speed of the conveyor belt.
  - service: `/ariac/conveyor/control {power}` where `power` can be 0 or a value in the range of 50 to 100, with 100 representing full speed.
- Get the content of a kit on a fixed kit tray.
  - service: `/ariac/kit_tray_{N}/get_content`. This will report everything detected on a fixed kit tray, including parts inside a movable kit tray located on the fixed kit tray.
- Get the content of assembly in a specific briefcase.
  - service: `/ariac/briefcase_{N}/get_content`.
  

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---
