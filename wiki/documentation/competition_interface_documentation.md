-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------
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
* TF frames for static key points of the workcell are published by the ARIAC simulation.
* Dynamic TF frames for the robot and faulty products are also published by the simulation.
* Other dynamic TF frames can be accessed through logical cameras.

The following frames are published on the global `/tf` and `/tf_static` topics.

|Description|TF frame|Type|
|----------|-----------|---|
| Origin of the workcell | world | static |
| Each sensor | `{sensor_name}_frame`, e.g. `logical_camera_1_frame` | static |
| Bin storage units | `bin{N}_frame`, where N=1..6 | static |
| Robot links | `{link_name}`, `left_arm_wrist1_link` | dynamic |
| Products detected by logical cameras | `{logical_camera_name}_{product_name}_frame`, e.g. `logical_camera_1_piston_rod_part_1_frame` | dynamic |
| Products detected by quality control sensors | `quality_control_sensor_{N}_{anonymize_mode_name}_frame`, where N=1..2, e.g. `quality_control_sensor_1_model_1_frame` | dynamic |
| Trays where products are placed | `kit_tray_{N}`, where N=1..2, e.g. `kit_tray_1` | dynamic |

## Actuators

- In the following table:
  - {gc}=gantry_controller
  - {rc}=right_arm_controller
  - {lc}=left_arm_controller
  - **M**=Message
  - **S**=Service



<table>
 <tr>
   <th>Topic</th>
   <th>Description</th>
   <th>Message</th>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/{gc}/command</li><li>/ariac/gantry/{rc}/command</li><li>/ariac/gantry/{lc}/command</li></ul></td>
   <td width="30%"><b>M</b>: command robot/arms to move</td>
   <td width="30%"><a href="http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html">trajectory_msgs/JointTrajectory.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/joint_states</li></ul></td>
   <td width="30%"><b>M</b>: robot joint states</td>
   <td width="30%"><a href="http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html">sensor_msgs/JointState</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/gantry/{gc}/state</li><li>/ariac/gantry/{rc}/state</li><li>/ariac/gantry/{lc}/state</li></ul></td>
   <td width="30%"><b>M</b>: robot controller's state</td>
   <td width="30%"><a href="http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html">control_msgs/JointTrajectoryControllerState.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/conveyor/state</li></ul></td>
   <td width="30%"><b>M</b>: conveyor belt's state</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/ConveyorBeltState.msg">nist_gear/ConveyorBeltState.msg</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/right_arm/gripper/control</li><li>/ariac/left_arm/gripper/control</li></ul></td>
   <td width="30%"><b>S</b>: enable/disable gripper's suction</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/VacuumGripperControl.srv">nist_gear/VacuumGripperControl.srv</a></td>
 </tr>
 <tr>
   <td width="40%"><ul><li>/ariac/right_arm/gripper/state</li><li>/ariac/left_arm/gripper/state</li></ul></td>
   <td width="30%"><b>M</b>: gripper's state</td>
   <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/VacuumGripperState.msg">nist_gear/VacuumGripperState.msg</a></td>
 </tr>
</table>


## Process management

- In the following table:
  - **M**=Message
  - **S**=Service

<table>
   <tr>
     <th>Topic</th>
     <th>Description</th>
     <th>Message</th>
   </tr>
   <tr>
     <td width="40%"><ul><li>/clock</li></ul></td>
     <td width="30%"><b>M</b>: <a href="http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic">simulation time</a></td>
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
     <td width="30%"><b>M</b>: state of the competition (init, ready, go, end_game, done)</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_msgs/html/msg/String.html">std_msgs/String.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/orders</li></ul></td>
     <td width="30%"><b>M</b>: new order to be completed</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Order.msg">nist_gear/Order.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/quality_control_sensor_{N}</li></ul></td>
     <td width="30%"><b>M</b>: output of quality control sensor {N} (N=1,2), model names anonymized</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/LogicalCameraImage.msg">nist_gear/LogicalCameraImage.msg</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/agv{N}</li></ul></td>
     <td width="30%"><b>S</b>: notify AGV{N} that the kit is ready (N=1,2)</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/AGVControl.srv">nist_gear/AGVControl.srv</a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/agv{N}/state</li></ul></td>
     <td width="30%"><b>M</b>: state of AGV {N} (N=1,2)</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_msgs/html/msg/String.html">std_msgs/String.msg</a></td>
   </tr>
</table>  


## Cheats

These are only provided for debugging/development purposes and their use is not permitted during the competition trials.

- In the following table:
  - **M**=Message
  - **S**=Service

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
     <td width="40%"><ul><li>/ariac/submit_shipment</li></ul></td>
     <td width="30%"><b>S</b>: submit a tray for evaluation without the AGV moving</td>
     <td width="30%"><a href="https://github.com/usnistgov/ARIAC/blob/master/nist_gear/srv/SubmitShipment.srv">nist_gear/SubmitShipment.srv </a></td>
   </tr>
   <tr>
     <td width="40%"><ul><li>/ariac/kit_tray_{N}/clear_tray</li></ul></td>
     <td width="30%"><b>S</b>: clear the contents of tray {N} without the AGV moving</td>
     <td width="30%"><a href="http://docs.ros.org/api/std_srvs/html/srv/Trigger.html">std_srvs/Trigger.srv</a></td>
   </tr>
</table>  
<!---See [the developer tips page](https://bitbucket.org/osrf/ariac/wiki/2019/developer_tips#markdown-header-running-ariac-with-cheats-enabled) for how to enable cheats.-->



-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
