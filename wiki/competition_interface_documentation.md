-------------------------------------------------
- Wiki | [Home](../README.md) | [Documentation](documentation.md) | [Tutorials](tutorials.md) | [Qualifiers](qualifier.md) | [Finals](finals.md) | [News](updates.md)
-------------------------------------------------

# Wiki | ARIAC Competition Interface

GEAR provides a [ROS](http://www.ros.org/) interface to the teams participating in ARIAC.
This interface can be used by teams to control all available actuators, read sensor information and send/receive notifications.

During the competition, it is against the rules to control the ARIAC simulation using anything other than the interface specified below.
**Teams are not permitted to use any of the topics or services in the `/gazebo/` namespace prefix to control the Gazebo simulation or get information about the simulation state. These interfaces will be blocked during the Finals.**

***Coming soon: hands-on tutorial on interfacing with GEAR through command line tools***

***Coming soon: tutorial on interfacing with GEAR through a ROS node***


## Sensors

|Topic name|Message/Service|Description|Message definition|
|----------|-----------|------------------|---------------|
|/ariac/{name}  | Message|  Break beam's output    |  [nist_gear/Proximity.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg)  |
|/ariac/{name}_change  | Message|  Break beam's output (output changes only)    |  [nist_gear/Proximity.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/Proximity.msg)  |
|/ariac/{name}  | Message|  Proximity sensor's output    |  [sensor_msgs/Range.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)  |
|/ariac/{name}  | Message|  Laser profiler's output    |  [sensor_msgs/LaserScan.msg](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)  |
|/ariac/{name}  | Message|  Depth camera's output    |  [sensor_msgs/PointCloud.msg](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html)  |
|/ariac/{name}  | Message|  Logical camera's output    |  [nist_gear/LogicalCameraImage.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/LogicalCameraImage.msg)  |


Note: The string `{name}` is replaced with the name you give the sensor in the config file. See [Configuration of the environment and trials](configuration_spec.md). Since the sensor names are unique, it ensures that all sensors publish data to unique topics.


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
| Arm links | `arm{N}_{link_name}`, where N=1..2 e.g. `arm1_wrist1_link` | dynamic |
| Products detected by logical cameras | `{logical_camera_name}_{product_name}_frame`, e.g. `logical_camera_1_piston_rod_part_1_frame` | dynamic |
| Products detected by quality control sensors | `quality_control_sensor_{N}_{anonymize_mode_name}_frame`, where N=1..2, e.g. `quality_control_sensor_1_model_1_frame` | dynamic |
| Trays where products are placed | `kit_tray_{N}`, where N=1..2, e.g. `kit_tray_1` | dynamic |

 
Additionally arm TF frames without the `arm{N}_` prefix are published on topics in the arm namespace.

* `/ariac/arm{N}/tf` where N=1..2
* `/ariac/arm{N}/tf_static` where N=1..2


|Description|TF frame|Type|
|----------|-----------|---|
| Origin of the workcell | world | static |
| Arm links | `{link_name}`, e.g. `wrist1_link` | dynamic |


## Actuators

|Topic name|Message/Service|Description|Message definition|
|----------|-----------|------------------|---------------|
|/ariac/arm{N}/arm/command  |   Message    | Command arm to move | [trajectory_msgs/JointTrajectory.msg](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
|/ariac/arm{N}/joint_states  |   Message    | Arm joint states | [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
|/ariac/arm{N}/arm/state  |   Message    | Arm Controller's state | [control_msgs/JointTrajectoryControllerState.msg](http://docs.ros.org/api/control_msgs/html/msg/JointTrajectoryControllerState.html)
| /ariac/conveyor/state  |   Topic    |  Conveyor belt's state        | [nist_gear/ConveyorBeltState.msg ](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/ConveyorBeltState.msg)          |
|/ariac/arm{N}/gripper/control  |   Service    |  Enable/disable gripper's suction| [nist_gear/VacuumGripperControl.srv](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/VacuumGripperControl.srv)|
|/ariac/arm{N}/gripper/state  |   Message    | Gripper's state| [osrf_gear/VacuumGripperState.msg](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/msg/VacuumGripperState.msg)|

## Process management

|Topic name                |Message/Service|Description          |Message definition|
|--------------------------|---------------|---------------------|------------------|
|/clock  |Message |[Simulation time](http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)      |  [rosgraph_msgs/Clock.msg](http://docs.ros.org/api/rosgraph_msgs/html/msg/Clock.html)       |
|/ariac/start_competition  |Service        |Start the competition|  [std_srvs/Trigger.srv](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)       |
|/ariac/end_competition  |Service        |End the competition early|  [std_srvs/Trigger.srv](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)       |
|/ariac/competition_state  |Message        |State of the competition (`init`, `ready`, `go`, `end_game`, `done`)|  [std_msgs/String.msg](http://docs.ros.org/api/std_msgs/html/msg/String.html)       |
|/ariac/orders  |   Message     |New order to be completed       |  [osrf_gear/Order.msg](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/msg/Order.msg)|
|/ariac/quality_control_sensor_{N}  | Message|  Output of quality control sensor {N} (N=1,2), model names anonymized    |  [osrf_gear/LogicalCameraImage.msg](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/msg/LogicalCameraImage.msg)  |
|/ariac/agv{N}  |Service        |Notify AGV{N} that the kit is ready (N=1,2)|  [osrf_gear/AGVControl.srv](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/AGVControl.srv)       |
|/ariac/agv{N}/state  |Message        |State of AGV {N} (N=1,2)|  [std_msgs/String.msg](http://docs.ros.org/api/std_msgs/html/msg/String.html)       |

## Cheats

These are only provided for debugging/development purposes and their use is not permitted during the competition trials. 
<!---See [the developer tips page](https://bitbucket.org/osrf/ariac/wiki/2019/developer_tips#markdown-header-running-ariac-with-cheats-enabled) for how to enable cheats.-->

| Topic name | Message/Service | Description | Message definition |
| ---------- |----------- | ------------------ | --------------- |
| /ariac/current_score  |   Topic    |  Current completion score        |   [std_msgs/Float32.msg](http://docs.ros.org/api/std_msgs/html/msg/Float32.html)      |
| /ariac/trays  |   Message    | State of the kit being built on each tray        | [osrf_gear/TrayContents.msg ](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/msg/TrayContents.msg)          |
|/ariac/material_locations |Service        |Query storage locations for a material (e.g. disk_part, pulley_part)|  [osrf_gear/GetMaterialLocations.srv](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/GetMaterialLocations.srv)       |
| /ariac/conveyor/control  |   Service    |  Modify power of the conveyor belt        | [osrf_gear/ConveyorBeltControl.srv ](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/ConveyorBeltControl.srv)          |
| /ariac/submit_shipment  |   Service    |  Submit a tray for evaluation without the AGV moving        | [osrf_gear/SubmitShipment.srv ](https://github.com/usnistgov/ARIAC/tree/master/nist_gear/srv/SubmitShipment.srv)          |
| /ariac/kit_tray_{N}/clear_tray  |   Service    |  Clear the contents of tray {N} without the AGV moving        | [std_srvs/Trigger.srv ](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)          |

-------------------------------------------------
- Wiki | [Home](../README.md) | [Documentation](documentation.md) | [Tutorials](tutorials.md) | [Qualifiers](qualifier.md) | [Finals](finals.md) | [News](updates.md)
-------------------------------------------------
