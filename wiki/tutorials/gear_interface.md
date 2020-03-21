-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Tutorials | Interacting with GEAR #
```diff
- This page is under construction and will be updated shortly
```



The purpose of this tutorial is to introduce you to the [competition interface](../documentation/competition_interface_documentation.md) that is used to interact with GEAR, the software used by teams participating in the Agile Robotics for Industrial Automation Competition (ARIAC).

# Running GEAR #

* After building GEAR from SOURCE, you will have to source the setup file from your catkin workspace.

```bash
# Path is where you built and installed gear
source ~/ariac_ws/devel/setup.bash
```
* To launch GEAR with a sample work cell that has some sensors and parts in various locations, run:

```bash
roslaunch nist_gear sample_environment.launch
```

# Starting and Stopping the Competition #

## Starting ##

* When gear is started, various competition elements will be in an inactive state.
* A service must be called to activate these components.
* Run the following command to manually start the competition:

```bash
rosservice call /ariac/start_competition
```

## Trial End ##
* When all orders have been filled, or the time limit for the trial has been exhausted, the competition state published on `/ariac/competition_state` will change to `done`.

* To check the competition state, run:

```bash
rostopic echo /ariac/competition_state -n 1
```

* If you wish to end the trial early (e.g. you detected a fault in your system and wish to terminate), run:

```bash
rosservice call /ariac/end_competition
```

# Fulfilling Orders #

* Orders for kits will be published during the competitions.
* An order is composed of products to place onto a tray.
* Teams earn points by completing orders quickly.

## Receiving Orders ##

* Teams must subscribe to this topic to receive the initial order, as well as any future order updates.
* To see the last order that was published, run:

```bash
rostopic echo /ariac/orders
```

* Note the competition must be started before an order will be published.
* You must add your sensors to detect and classify products in the storage bins or conveyor belt.
* The most recently received order is the highest priority.

## Delivering Orders ##

* There are two AGVs carrying trays that kits can be assembled on.
* When a kit has been completed, the AGV must be commanded to deliver the kit so it can be scored.

* The service `/ariac/agv[N}` where N is `1` or `2` is used for this purpose.
* The type of kit that is on the tray must be specified.
* This is included in the original order.
* Call this service to submit a tray for evaluation:

```bash
rosservice call /ariac/agv1 "kit_type: order_0_kit_0"
```

* If multiple trays need to be submitted, the AGV will return an empty tray after the submitted tray has been evaluated.

* During the final competition the order may indicate that it must be delivered to a particular AGV.
* Orders delivered on the wrong AGV will be counted as a zero score.


## Faulty Products ##
* There are quality control sensors above each AGV that publish the pose of faulty parts that they see on the tray. The quality control sensors:
  * have an equivalent interface to logical camera sensors
  * publish tf frames of faulty parts
  * are positioned above each AGV in pre-defined locations
  * users cannot specify the locations of these sensors
  * report faulty parts only once they are in the tray of an AGV
  * do not report any information about non-faulty parts

* As an example (this command will not work during the qualifier or finals), spawn a known faulty part on AGV 1's tray:

```
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/piston_rod_part_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model piston_rod_part_red_5
```

![ariac_faulty_part_labeled.png](https://bitbucket.org/repo/pB4bBb/images/4190081571-ariac_faulty_part_labeled.png)

* Then run this command to see the quality control sensor's output.

```bash
rostopic echo /ariac/quality_control_sensor_1
```

* You should see that the part spawned has been detected as faulty.

```
models:
  -
    type: "model"
    pose:
      position:
        x: 0.743197471473
        y: -0.100148694292
        z: 0.453218022501
      orientation:
        x: -0.496443448488
        y: 0.50485071322
        z: 0.496120280969
        w: -0.502527936164
pose:
  position:
    x: 0.3
    y: 3.5
    z: 1.5
  orientation:
    x: 0.501601833862
    y: 0.499997434122
    z: -0.499997434122
    w: 0.498398166138
```

# Controlling the Arms #

There are two UR10 arms in the simulation. The UR10 simulation and control code is provided by [ROS Industrial's universal_robot ROS packages](https://github.com/ros-industrial/universal_robot).
The control parameters have been modified for use in the ARIAC simulation.

## Controlling a Vacuum Gripper ##

Each arm has a simulated pneumatic gripper attached to the arm's end effector.
Teams can enable or disable the suction of the gripper.
When the suction is enabled and the gripper is making contact with a product, the contacting product will be attached to the gripper.
At any point, teams will also be able to disable the suction, causing the detachment of the object if it was previously attached.
To enable `arm1`'s gripper suction from the command line, run:

```bash
rosservice call /ariac/arm1/gripper/control "enable: true"
```

The gripper periodically publishes its internal state on topic `/ariac/armN/gripper/state`.
Subscribe to this topic to introspect the gripper's status.
You can check whether the suction is enabled/disabled or whether there is an object attached to the gripper. Execute the following command to display `arm1`'s gripper state:

```bash
rostopic echo /ariac/arm1/gripper/state -n 1
```

Disable the suction again for now with

```bash
rosservice call /ariac/arm1/gripper/control "enable: false"
```

## Controlling Arm Joints ##

Each arm has its own command topic for controlling the joints of the arm on `/ariac/armN/arm/command` where `N` is `1` or `2`.
The topic uses [trajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html) messages.
The arm's controller will try to match the commanded states.
The arm is controlled by an instance of [ros_controllers/joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller).

### Command Line ###

Run this command to move `arm1` over a gasket part in the sample environment.

```bash
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 2}, \
        positions: [0.15, 3.14,  -1.570,  2.14, 3.1, -1.59, 0.126]}, \
{time_from_start: {secs: 4}, \
        positions: [-0.35, 3.14,  -0.6,  2.3, 3.0, -1.59, 0.126]}, \
{time_from_start: {secs: 6}, \
        positions: [-0.35, 3.14,  -0.5,  2.3, 3.05, -1.59, 0.126]}, \
]}" -1
```

You should see the arm move to the specified joint positions.
To get the current joint positions of the arm, run:

```bash
rostopic echo /ariac/arm1/joint_states -n 1
```

The `/ariac/armN/joint_states` topic uses the [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html) message which contains joint positions, velocities, efforts, and the name of the joints.


Now, enable the gripper on arm1.

```bash
rosservice call /ariac/arm1/gripper/control "enable: true"
```

The gripper state should now show it has attached to an object.

```
$ rostopic echo -n 1 /ariac/arm1/gripper/state
enabled: True
attached: True
---
```

Note, you could have enabled the gripper at the beginning.
It will attach to the first product it contacts.


Move the part over `AGV1`'s tray
```
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 2}, \
        positions: [0.0, 3.14,  -1.570,  2.14, 3.27, -1.51, 0.0]}, \
{time_from_start: {secs: 5}, \
        positions: [1.0, 1.85,  0,  -0.38, 1.57, -1.51, 0.00]}, \
{time_from_start: {secs: 7}, \
        positions: [1.0, 1.507,  0,  -0.38, 0.38, -1.51, 0.00]}, \
{time_from_start: {secs: 10}, \
        positions: [1.18, 1.507,  0.38,  -0.38, 1.55, 1.75, 0.127]}, \
]}" -1
```

Disable the gripper to drop the object

```bash
rosservice call /ariac/arm1/gripper/control "enable: false"
```

Return the arm to the starting position.

```
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 5}, \
        positions: [0.0, 3.14,  -1.570,  2.14, 3.27, -1.51, 0.0]}, \
]}" -1

```


### rqt GUI ###

To control an arm from a GUI, run:

```
rqt robot_description:=/ariac/arm1/robot_description
```

Open a joint controller plugin with `Plugins` -> `Robot Tools` -> `Joint trajectory controller`.
Select `/ariac/arm1/controller_manager` and the `arm` controller.
You should be able to adjust the arm by modifying the joint values.

![ariac_joint_controller_gui.png](https://bitbucket.org/repo/pB4bBb/images/940260870-ariac_joint_controller_gui.png)

**Be aware that enabling the joint trajectory controller in rqt can conflict with other joint trajectories that the arm might be receiving from, for example, the command-line.**
Disable the controller in rqt if you wish to send trajectories from the command-line.

Repeat the procedure replacing `arm1` with `arm2` to control the second arm.

### MoveIt ###

See the [ARIAC 2019 MoveIt tutorial](moveit_interface.md).

# Visualization in RViz #

## TF frames ##

GEAR publishes various TF frames of various poses in the world.
These frames may be useful when developing the robot control algorithm.

To view them, use the provided rviz config
```
rosrun rviz rviz -d `catkin_find osrf_gear --share --first-only`/rviz/ariac.rviz
```

**Note that GEAR uses tf2_msgs and not the deprecated tf_msgs. Accordingly, you should use the tf2 package instead of tf.**

## Robot Model ##

Using the RobotModel display, one arm at a time can be visualized in rviz.
It requires remapping the `tf` topics and `robot_description` parameter.

Arm 1
```
rosrun rviz rviz /tf:=/ariac/arm1/tf /tf_static:=/ariac/arm1/tf_static robot_description:=/ariac/arm1/robot_description
```

Arm 2
```
rosrun rviz rviz /tf:=/ariac/arm2/tf /tf_static:=/ariac/arm2/tf_static robot_description:=/ariac/arm2/robot_description
```


# Troubleshooting Interfaces #

There are a few interfaces that will not be available during the competition but can be useful during development.
**These interfaces will not be available during the competition.**

## Controlling the Conveyor belt ##

The service `/ariac/conveyor/control` can be used to modify the power of the conveyor belt or stop it.
The power can be 0 or a value in the range of 50 to 100, with 100 representing full speed.

You can start the conveyor belt with

```
rosservice call /ariac/conveyor/control "power: 100"
```

## Viewing Tray Contents ##

The `/ariac/trays` topic can be used during development for seeing the pose of products in the shipping boxes in the same frame as that which will be used for shipment evaluation.

```
$ rostopic echo /ariac/trays
tray: "agv1::kit_tray_1::kit_tray_1::tray"
objects: []
---
tray: "agv2::kit_tray_2::kit_tray_2::tray"
objects: []
---
```

## Submitting Trays without delivery ##

The `/ariac/submit_tray` service can be used during development for submitting kits for evaluation without them being ready for delivery.

```
$ rosservice call /ariac/submit_tray "tray_id: 'agv1::kit_tray_1::kit_tray_1::tray'
kit_type: 'order_0_shipment_0'"
success: True
inspection_result: 0.0
```

## Querying Locations of Products ##
To determine where in the workcell products may be found, you can use the [osrf_gear/GetMaterialLocations](https://bitbucket.org/osrf/ariac/raw/master/osrf_gear/srv/GetMaterialLocations.srv) service.
The bins where products are found will change between trials.

An example service call and response is:

```
$ rosservice call /ariac/material_locations "material_type: piston_rod_part"
storage_units:
  -
    unit_id: "bin2"
  -
    unit_id: "belt"
```

which suggests that products of type piston_rod_part may be found in bin2 or on the conveyor belt.
The TF frame bin2_frame can be used to know the location of bin2 with respect to the origin of the "world":

```
$ rosrun tf tf_echo /world /bin2_frame
At time 0.000
- Translation: [-0.300, -1.150, 0.720]
- Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
            in RPY (radian) [0.000, -0.000, 3.142]
            in RPY (degree) [0.000, -0.000, 180.000]
```

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
