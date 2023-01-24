# Agility Challenges

There are eight possible agility challenges in ARIAC 2023. A description of each challenge is provided below. Besides the `Human Operator` challenge, all other challenges can occur multiple times in a trial. 
<!-- The number of times a challenge can occur is specified in the trial configuration file. The `Human Operator` challenge can only occur once in a trial. The `Human Operator` challenge is triggered when the `Human Operator` field in the trial configuration file is set to `true`. -->

*Note*: A trial may have only some challenges, may not have any challenge at all, or may have all the challenges.

## Faulty Parts

Faulty parts are parts that are not in good condition. They are not suitable for use in the competition. If an order is submitted with faulty parts, these parts are not considered for scoring. Faulty parts are identified by quality control sensors, which are attached to AGVs.

### Faulty Parts Example

Parts are set to faulty through the `faulty_part` challenge in the trial configuration file. Only the first parts placed in a tray are faulty. In the example below, the first parts placed in quadrants 1 and 2 in the tray required by order `MMB30H56` are always faulty. If these parts are removed and replaced with new parts, the new parts are not faulty.

```yaml
challenges:
  - faulty_part:
    order_id: 'MMB30H56'
    quadrant1: true
    quadrant2: true
```

## Flipped Parts

The environment can be started with parts that are flipped. Flipped parts are parts that are upside down. When a part is spawned as flipped, competitors will need to flip those parts again so they end up with the correct orientation. If an order is submitted with flipped parts, these parts are not considered for scoring. Flipped parts are identified by quality control sensors, which are attached to AGVs.

Flipped parts apply to a specific part type and color in a specific bin or on the conveyor belt. To set parts as flipped, the `flipped` field in the trial configuration file must be set as `true` for the corresponding part.

### Flipped Parts Example

The example below describes all purple regulators as flipped in `bin3`.

```yaml
bin3:
  - type: 'regulator'
    color: 'purple'
    slots: [2, 3]
    rotation: 'pi/6'
    flipped: true
```

The example below describes all orange batteries as flipped on the conveyor belt.

```yaml
conveyor_belt: 
  active: true
  spawn_rate: 3.0 
  order: 'sequential' 
  parts_to_spawn:
    - type: 'battery'
      color: 'orange'
      number: 5
      offset: 0.5 # between -1 and 1
      flipped: true
      rotation: 'pi/6'
```

## Faulty Gripper

The faulty gripper challenge simulates a faulty gripper which can drop a part after the part has been picked up. The gripper can drop a part at any time during the trial. The gripper can drop a part that is in the gripper's grasp even if the gripper is not moving. 

The goal of this challenge is to test the ability of the competitors' control system to pick a part of the same type and color again after the gripper has dropped a part. The control system may try to pick the part again from where it was dropped or pick up a part from a different location.

### Faulty Gripper Example

The example below describes a faulty gripper occuring 5 seconds after the ceiling robot has picked up a second red pump.

```yaml
challenges:
  - dropped_part:
    robot: 'ceiling_robot'
    type: 'pump'
    color: 'red'
    drop_after: 1
    delay: 5
```

## Robot Malfunction

The robot malfunction challenge simulates a robot malfunction. The robot can malfunction in some conditions (time, part placement, or submission) during the trial. The robot can malfunction even if it is not moving. When a robot malfunctions, it stops moving and cannot be controlled by the competitors' control system. The robot will remain in the same position until the malfunction is resolved. To specify how long a robot malfunctions, a time duration of the malfunction is specified in the trial configuration file.

The goal of this challenge is to test the ability of the competitors' control system to use the other robot to complete the tasks that was being performed by the robot which is malfunctioning. 

It can happen that both robots malfunction at the same time. In this case, competitors's control system must wait until the malfunction is resolved before continuing with the trial.

### Robot Malfunction Example

The robot malfunction challenge is specified in the trial configuration file using the following fields:

* `duration`: The duration of the robot malfunction in seconds.
* `robots_to_disable`: A list of robots that malfunction. It can be either `floor_robot` or `ceiling_robot` or both.
* Conditions that can trigger the robot malfunction:
  * `part_place_condition`: The challenge starts when a part of a specific type and color is placed on a specific AGV.
  * `time_condition`: The challenge starts after a specific time.
  * `submission_condition`: The challenge starts when a specific order is submitted.

Robot malfunctions can occur multiple times in the same trial. The example below shows a robot malfunction challenge occurring four times.

```yaml
challenges:
  - robot_malfunction:
      duration: 20.0
      robots_to_disable: ['floor_robot']
      time_condition: 10.0
  - robot_malfunction:
      duration: 20.0
      robots_to_disable: ['floor_robot']
      time_condition: 225.0
  - robot_malfunction:
      duration: 25.0
      robots_to_disable: ['ceiling_robot']
      submission_condition:
        order_id: 'MMB30H58'
  - robot_malfunction:
      duration: 5.0
      robots_to_disable: ['floor_robot','ceiling_robot']
      part_place_condition:
        color: 'green'
        type: 'sensor'
        agv: 4
```

## Sensor Blackout

The sensor blackout challenge simulates a sensor blackout. The sensor can black out in some conditions (time, part placement, or submission) during the trial. When a sensor blacks out, it stops publishing data. The sensor will remain in the same state until the sensor blackout is resolved (after a duration). To specify how long a sensor blacks out, a time duration  is specified in the trial configuration file. Sensor blackouts can occur on any sensor type and multiple times during the same challenge. 

The goal of this challenge is to test the ability of the competitors' control system to use the other sensors or use a stored world model to continue the tasks that were being performed before the blackout.

### Sensor Blackout Example

The sensor blackout challenge is specified in the trial configuration file using the following fields:

* `duration`: The duration of the sensor blackout in seconds.
* `sensors_to_disable`: A list of sensor types that are disabled:
  * 'break_beam'
  * 'proximity'
  * 'laser_profiler'
  * 'lidar'
  * 'camera'
  * 'logical_camera'
* Conditions that can trigger the sensor blackout:
  * `part_place_condition`: The challenge starts when a part of a specific type and color is placed on a specific AGV.
  * `time_condition`: The challenge starts after a specific time.
  * `submission_condition`: The challenge starts when a specific order is submitted.

The sensor blackout challenge can occur multiple times in the same trial. The example below shows the challenge occurring twice in the same trial.

```yaml
challenges:
  - sensor_blackout:
      duration: 25.0
      sensors_to_disable: ['break_beam']
      time_condition: 20
  - sensor_blackout:
      duration: 5.0
      sensors_to_disable: ['lidar', 'logical_camera']
      submission_condition:
        order_id: 'MMB30H57'
```

## High-priority Orders

The high-priority orders challenge simulates an order that must be completed before a low-priority order. The high-priority order must be completed and  submitted before the low-priority order.

The goal of this challenge is to test the ability of the competitors' control system to prioritize  high-priority orders over low-priority orders. This may require switching from kitting to assembly or vice versa. This may also require switching from one kitting task to another kitting task or switching from one assembly task to another assembly task.

### High-priority Orders Example

To specify a high-priority order, the `priority` field is set to `true` in the order configuration file. The example below shows a high priority order with the order ID `MMB30H57` and a low priority order with the order ID `MMB30H58`.

```yaml
orders:
  - id: 'MMB30H58'
    type: 'kitting'
    announcement:
      time_condition: 0
    priority: false
    kitting_task:
      agv_number: 2
      tray_id: 2
      destination: 'warehouse'
      products:
        - type: 'battery'
          color: 'blue'
          quadrant: 1
  - id: 'MMB30H57'
    type: 'kitting'
    announcement:
      time_condition: 44.5
    priority: true
    kitting_task:
      agv_number: 3
      tray_id: 5
      destination: 'warehouse'
      products:
        - type: 'sensor'
          color: 'orange'
          quadrant: 4
```

## Insufficient Parts

The insufficient parts challenge simulates a situation where the competitors' control system does not have enough parts to complete an order. This challenge is set up by not providing enough parts in the workcell. The competitors' control system must be able to detect that it does not have enough parts to complete the order and submit incomplete orders.

### Insufficient Parts Example

There is no specific field in the trial configuration file to specify this challenge. The example below shows a trial configuration file where the competitors' control system does not have enough parts to complete the order with the order ID `MMB30H58`: `bin1` has only two `battery` parts of color `blue` but  order `MMB30H58` requires 4.

```yaml
parts: 
  bins: 
    bin1: 
      - type: 'pump'
        color: 'red'
        slots: [1, 2, 3]
        rotation: 'pi/6'
        flipped: false
      - type: 'battery'
        color: 'blue'
        slots: [4, 5]
        rotation: 'pi/2'
        flipped: false
orders:
  - id: 'MMB30H58'
    type: 'kitting'
    announcement:
      time_condition: 0
    priority: false
    kitting_task:
      agv_number: 2
      tray_id: 2
      destination: 'warehouse'
      products:
        - type: 'battery'
          color: 'blue'
          quadrant: 1
        - type: 'battery'
          color: 'blue'
          quadrant: 2
        - type: 'battery'
          color: 'blue'
          quadrant: 3
        - type: 'battery'
          color: 'blue'
          quadrant: 4
```

## Human Operator

The human operator challenge consists of a simulated human operator navigating the workcell. The simulated human operator will have one of the three following behaviors in a
given trial and the selected behavior will stay the same during the trial.

* **Indifferent**: The human operator follows a scripted path, regardless of the location of the robots in the environment.
* **Antagonistic**: During an arbitrary period of time, the human operator purposefully moves towards the ceiling robot to interfere with the robot’s current task.
* **Helpful**: The human operator will stop moving once the ceiling robot is at a certain distance away from him.

The goal of this challenge is to test the ability of the competitors' control system to avoid collisions with the human operator. The pose of the human operator is published to a Topic and this information can also be retrieved from the `/tf` Topic.

### Human Operator Example

The human operator challenge is specified in the trial configuration file using the following fields:

* `behavior`: The behavior of the human operator:
  * `'indifferent'`
  * `'antagonistic'`
  * `'helpful'`
* Conditions that can trigger the human operator behavior:
  * `part_place_condition`: The challenge starts when a part of a specific type and color is placed on a specific AGV.
  * `time_condition`: The challenge starts after a specific time.
  * `submission_condition`: The challenge starts when a specific order is submitted.

  Below is an example of the human operator challenge with the behavior set to `'antagonistic'` and the challenge starting when the order with the order ID `MMB30H57` is submitted.

```yaml
challenges:
  - human_operator:
      behavior: 'antagonistic'
      submission_condition:
        order_id: 'MMB30H57'
```
