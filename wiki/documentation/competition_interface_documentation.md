Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------

- [Wiki | Documentation |Competition Interface](#wiki--documentation-competition-interface)
  - [Sensors](#sensors)
  - [TF frames](#tf-frames)
  - [Actuators](#actuators)
  - [Process management](#process-management)
  - [Cheats](#cheats)

# Wiki | Documentation |Competition Interface

GEAR provides a [ROS](http://www.ros.org/) interface to the teams participating in ARIAC.
This interface can be used by teams to control all available actuators, read sensor information and send/receive notifications.

During the competition, it is against the rules to control the ARIAC simulation using anything other than the interface specified below.
**Teams are not permitted to use any of the topics or services in the `/gazebo/` namespace prefix to control the Gazebo simulation or get information about the simulation state. These interfaces will be blocked during the Finals.**

## Sensors


- In the following table:
  - {name} is replaced with the name you give the sensor in the config file (see [Configuration of the environment and trials](configuration_spec.md)). Since the sensor names are unique, it ensures that all sensors publish data to unique topics.
  - **M**=Message



<table>
   <tr>
     <th>Topic</th>
     <th>Description</th>
     <th>Message</th>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}</li></ul></td>
     <td width="30%"><b>M</b>: break beam's output</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg">nist_gear/Proximity.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}_change</li></ul></td>
     <td width="30%"><b>M</b>: break beam's output (output changes only)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg">nist_gear/Proximity.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}</li></ul></td>
     <td width="30%"><b>M</b>: proximity sensor's output</td>
     <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/Range.html">sensor_msgs/Range.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}</li></ul></td>
     <td width="30%"><b>M</b>: laser profiler's output</td>
     <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html">sensor_msgs/LaserScan.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}</li></ul></td>
     <td width="30%"><b>M</b>: depth camera's output</td>
     <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html">sensor_msgs/PointCloud.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/{name}</li></ul></td>
     <td width="30%"><b>M</b>: logical camera's output</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/LogicalCameraImage.msg">nist_gear/LogicalCameraImage.msg</a></td>
   </tr>
</table>

## TF frames

- TF frames for static key points of the workcell are published by the ARIAC simulation.
- Dynamic TF frames for the robot and faulty products are also published by the simulation.

The following frames are published on the global `/tf` and `/tf_static` topics.

|Description|TF frame|Type|
|----------|-----------|---|
| Origin of the workcell | world | static |
| Bin storage units | `bin{N}_frame`, where N=1..8 | static |
| Robot links | `{link_name}`, `shoulder_link` | dynamic |
| Products detected by quality control sensors | `quality_control_sensor_{N}_{anonymize_mode_name}_frame`, where N=1..4, e.g. `quality_control_sensor_1_model_1_frame` | dynamic |
| Trays where products are placed | `kit_tray_{N}`, where N=1..4, e.g. `kit_tray_1` | dynamic |
| Briefcases where products are placed | `briefcase_{N}`, where N=1..4, e.g. `briefcase_1` | dynamic |

## Actuators

- In the following Tables:
  - **M**=Message
  - **T**=Topic




<table>
 <tr>
   <th>Action</th>
   <th>Description</th>
   <th>Message</th>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/gantry_arm_controller/command
</li></ul></td>
   <td width="30%"><b>T</b>: command the gantry arm</td>
   <td width="30%"><a href="http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html">trajectory_msgs/JointTrajectory.msg</a></td>
 </tr>
  <tr>
   <td width="40%"><ul><li>/ariac/gantry/gantry_controller/command
</li></ul></td>
   <td width="30%"><b>T</b>: command the gantry torso only</td>
   <td width="30%"><a href="http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html">trajectory_msgs/JointTrajectory.msg</a></td>
 </tr>
   <tr>
   <td width="40%"><ul><li>/ariac/kitting/kitting_arm_controller/command
</li></ul></td>
   <td width="30%"><b>T</b>: command the kitting robot</td>
   <td width="30%"><a href="http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html">trajectory_msgs/JointTrajectory.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/joint_states</li></ul></td>
   <td width="30%"><b>T</b>: robot joint states</td>
   <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html">sensor_msgs/JointState</a></td>
 </tr>
  <tr>
   <td width="40%"><ul><li>/ariac/kitting/joint_states</li></ul></td>
   <td width="30%"><b>T</b>: robot joint states</td>
   <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html">sensor_msgs/JointState</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/gantry_controller/state</li></ul></td>
   <td width="30%"><b>T</b>: gantry torso controller's state</td>
   <td width="30%"><a href="http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html">control_msgs/JointTrajectoryControllerState.msg</a></td>
 </tr>
  <tr>
   <td width="40%"><ul><li>/ariac/gantry/gantry_arm_controller/state</li></ul></td>
   <td width="30%"><b>T</b>: gantry arm controller's state</td>
   <td width="30%"><a href="http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html">control_msgs/JointTrajectoryControllerState.msg</a></td>
 </tr>
   <tr>
   <td width="40%"><ul><li>/ariac/kitting/kitting_arm_controller/state</li></ul></td>
   <td width="30%"><b>T</b>: kitting arm controller's state</td>
   <td width="30%"><a href="http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html">control_msgs/JointTrajectoryControllerState.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/conveyor/state</li></ul></td>
   <td width="30%"><b>T</b>: conveyor belt's state</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/ConveyorBeltState.msg">nist_gear/ConveyorBeltState.msg</a></td>
 </tr>
  <tr>
   <td width="40%"><ul><li>/ariac/gantry/arm/gripper/state</li></ul></td>
   <td width="30%"><b>T</b>: gripper's state of the gantry robot</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/VacuumGripperState.msg">nist_gear/VacuumGripperState.msg</a></td>
 </tr>
   <tr>
   <td width="40%"><ul><li>/ariac/kitting/arm/gripper/state</li></ul></td>
   <td width="30%"><b>T</b>: gripper's state of the kitting robot</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/VacuumGripperState.msg">nist_gear/VacuumGripperState.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/arm/gripper/control
</li><li>/ariac/kitting/arm/gripper/control
</li></ul></td>
   <td width="30%"><b>S</b>: enable/disable gripper's suction</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/VacuumGripperControl.srv">nist_gear/VacuumGripperControl.srv</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry_tray/lock_models</li><li>/ariac/gantry_tray/unlock_models
</li></ul></td>
   <td width="30%"><b>S</b>: lock/unlock parts on the gantry tray</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/VacuumGripperControl.srv">std_srvs/Trigger.srv</a></td>
 </tr>
  <tr>
   <td width="40%"><ul><li>/ariac/robot_health</li></ul></td>
   <td width="30%"><b>T</b>: get health status of each robot</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/VacuumGripperControl.srv">nist_gear/RobotHealth.msg</a></td>
 </tr>
</table>


## Process management


<table>
   <tr>
     <th>Topic</th>
     <th>Description</th>
     <th>Message</th>
   </tr>
   <tr>
     <td width="40%"><ul><li>/clock</li></ul></td>
     <td width="30%"><b>T</b>: <a href="http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic">simulation time</a></td>
     <td width="30%"><a href="http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html">rosgraph_msgs/Clock.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/start_competition</li></ul></td>
     <td width="30%"><b>S</b>: start the competition</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_srvs/html/srv/Trigger.html">std_srvs/Trigger.srv</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/end_competition</li></ul></td>
     <td width="30%"><b>S</b>: end the competition</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_srvs/html/srv/Trigger.html">std_srvs/Trigger.srv</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/competition_state</li></ul></td>
     <td width="30%"><b>T</b>: state of the competition (init, ready, go, end_game, done)</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_msgs/html/msg/String.html">std_msgs/String.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/orders</li></ul></td>
     <td width="30%"><b>T</b>: new order to be completed</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Order.msg">nist_gear/Order.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/quality_control_sensor_{N}</li></ul></td>
     <td width="30%"><b>T</b>: output of quality control sensor {N} (N=1,2), model names anonymized</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/LogicalCameraImage.msg">nist_gear/LogicalCameraImage.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/agv1/submit_shipment
</li></ul></td>
     <td width="30%"><b>S</b>: submit shipment for AGV{N} (N=1,2,3,4)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/AGVControl.srv">nist_gear/AGVToAssemblyStation.srv</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/agv{N}/state</li></ul></td>
     <td width="30%"><b>T</b>: state of AGV {N} (N=1,2,3,4)</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_msgs/html/msg/String.html">std_msgs/String.msg</a></td>
   </tr>
      <tr>
     <td width="40%"><ul><li>/ariac/as{N}/submit_shipment
</li></ul></td>
     <td width="30%"><b>S</b>: submit shipment for assembly station {N} (N=1,2,3,4)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/AGVControl.srv">nist_gear/AssemblyStationSubmitShipment.srv</a></td>
   </tr>
</table>  


## Cheats

These are only provided for debugging/development purposes and their use is not permitted during the competition trials.


<table>
   <tr>
     <th>Topic</th>
     <th>Description</th>
     <th>Message</th>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/current_score</li></ul></td>
     <td width="30%"><b>M</b>: current completion score</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_msgs/html/msg/Float32.html">std_msgs/Float32.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/trays</li></ul></td>
     <td width="30%"><b>M</b>: state of the kit being built on each tray</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/TrayContents.msg">nist_gear/TrayContents.msg </a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/material_locations</li></ul></td>
     <td width="30%"><b>S</b>: query storage locations for a material (e.g. disk_part, pulley_part)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/GetMaterialLocations.srv">nist_gear/GetMaterialLocations.srv</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/conveyor/control</li></ul></td>
     <td width="30%"><b>S</b>: modify power of the conveyor belt</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/ConveyorBeltControl.srv">nist_gear/ConveyorBeltControl.srv </a></td>
   </tr>
         <tr>
     <td width="40%"><ul><li>/ariac/kit_tray_{N}/get_content
</li></ul></td>
     <td width="30%"><b>S</b>: get product information on kit tray {N} (N=1,2,3,4)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/ConveyorBeltControl.srv">nist_gear/DetectKittingShipment.srv </a></td>
   </tr>
         <tr>
     <td width="40%"><ul><li>/ariac/briefcase_{N}/get_content
</li></ul></td>
     <td width="30%"><b>S</b>: get product information in briefcase {N} (N=1,2,3,4)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/ConveyorBeltControl.srv">nist_gear/DetectAssemblyShipment.srv </a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/kit_tray_{N}/clear_tray</li></ul></td>
     <td width="30%"><b>S</b>: clear the contents of tray {N} without the AGV moving</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_srvs/html/srv/Trigger.html">std_srvs/Trigger.srv</a></td>
   </tr>
      <tr>
     <td width="40%"><ul><li>/ariac/briefcase_{N}/clear_briefcase</li></ul></td>
     <td width="30%"><b>S</b>: clear the contents of briefcase {N}</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_srvs/html/srv/Trigger.html">std_srvs/Trigger.srv</a></td>
   </tr>
</table>  


Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
