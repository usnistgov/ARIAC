
========
Agility Challenges
========



There are 8 possible :term:`agility challenges<Agility Challenge>` in ARIAC 2023. A description of each challenge is provided below. Besides the :ref:`target to human operator` challenge, all other challenges can occur multiple times in a trial. 

.. note::
  A trial may consist of some of the challenges described in this page, may consist of no  challenge at all, or may consist of all the challenges.

.. _target to faulty part:

Faulty Parts
================

Faulty parts are parts that are not in good condition. They are not suitable for use in the competition. If an order is submitted with faulty parts, these parts are not considered for scoring. Faulty parts are identified by quality control sensors, which are attached to AGVs.

  The goal of this challenge is to test the ability of the :term:`CCS<Competitor Control System (CCS)>` to 1) correctly use the quality check sensor to detect faulty parts and 2) replace them with new parts.


Faulty Parts Example
----------------------------

The faulty parts challenge is set with the field ``faulty_part`` under the ``challenges`` field  in the trial configuration file. As mentioned earlier, only the first part placed in a quadrant is faulty. In the example below, any first part placed in  quadrants 1 and 2 in the kitting tray required by order ``MMB30H56`` is faulty. If these parts are removed and replaced with new parts, the new parts are set to non-faulty.

.. code-block:: yaml

  challenges:
    - faulty_part:
      order_id: 'MMB30H56'
      quadrant1: true
      quadrant2: true


Detecting Faulty Parts
----------------------------

.. important::
  The quality control sensor located above each AGV is capable of detecting faulty parts. A quality check can be performed by calling the ``/ariac/perform_quality_check`` service with an order ID argument (see structure of the :ref:`service file<perform-quality-check-srv>`). When a faulty part is detected, the CCS has to discard the part and replace it with a new part. The new part will automatically be set to non-faulty by the :term:`AM<ARIAC Manager (AM)>`.

  .. caution::
    This service can be called only once for each order ID. It is suggested to call this service after the order is completed but before it is submitted.

  .. code-block:: bash
    :caption: Message for the quality check service.
    :name: perform-quality-check-srv

    # PerformQualityCheck.srv
    string order_id
    ---
    bool valid_id
    bool all_passed
    bool incorrect_tray
    ariac_msgs/QualityIssue quadrant1
    ariac_msgs/QualityIssue quadrant2
    ariac_msgs/QualityIssue quadrant3
    ariac_msgs/QualityIssue quadrant4

  * The service returns a Boolean value for the field ``valid_id`` indicating whether or not the order ID is valid. An order ID is not valid if the order ID does not exist or if the quality check was already called for this order ID.

  * The field ``all_passed`` is set to ``true`` only if:

    * All parts in the kitting tray are NOT faulty.
    * All parts are present in the kitting tray (no empty quadrant).
    * All parts have the correct orientation (no flipped part).
    * All parts are of the correct type.
    * All parts are of the correct color.

  * The field ``incorrect_tray`` informs on whether or not the kitting task was performed in the correct kitting tray.
  * Information for each quadrant is reported as a ``ariac_msgs/msg/QualityIssue`` :ref:`message<quality-issue-msg>`.


  .. code-block:: bash
    :caption: Quality information for one quadrant.
    :name: quality-issue-msg

    # QualityIssue.msg
    bool all_passed           # True if everything is correct in the quadrant
    bool missing_part         # True if a part is missing in the quadrant
    bool flipped_part         # True if a part is flipped in the quadrant
    bool faulty_part          # True if a part is faulty in the quadrant
    bool incorrect_part_type  # True if a part has the wrong type in the quadrant
    bool incorrect_part_color # True if a part has the wrong color in the quadrant



.. _target to flipped part:

Flipped Parts
================

The environment can be started with parts that are flipped. Flipped parts are parts that are upside down. When a part is spawned as flipped, the CCS is required to flip this part again so it ends up with the correct orientation. If an order is submitted with flipped parts, these parts are not considered for scoring. 







Flipped Parts Example
----------------------------

Flipped parts apply to a specific part type and color in a specific bin or on the conveyor belt. To set parts as flipped, the ``flipped`` field in the trial configuration file must be set as ``true`` for the corresponding parts.

The example :ref:`below<flipped-parts-in-bin>` describes all purple regulators as flipped in ``bin3``. 

.. code-block:: yaml
  :caption: Setting flipped parts in a bin.
  :name: flipped-parts-in-bin

  bin3:
    - type: 'regulator'
      color: 'purple'
      slots: [2, 3]
      rotation: 'pi/6'
      flipped: true

The example :ref:`below<flipped-parts-on-conveyor-belt>` describes all orange batteries as flipped on the conveyor belt.

.. code-block:: yaml
  :caption: Setting flipped parts on the conveyor belt.
  :name: flipped-parts-on-conveyor-belt
  
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


Detecting Flipped Parts
----------------------------

.. important::
  Flipped parts detection is performed similarly to faulty parts detection. The quality control sensor located above each AGV is capable of detecting flipped parts. See the :ref:`target to faulty part` section for more information on how to perform a quality check.




.. _target to faulty gripper:

Faulty Gripper
================

The faulty gripper challenge simulates a faulty gripper which can drop a part after the part has been picked up. The gripper can drop a part at any time during the trial. The gripper can drop a part that is in the gripper's grasp even if the gripper or robot is not moving. 

The goal of this challenge is to test the ability of the competitors' control system to pick a part of the same type and color again after the gripper has dropped a part. The control system may try to pick the part again from where it was dropped or pick up a part from a different location.

Faulty Gripper Example
----------------------------

The example below describes a faulty gripper occuring 5 seconds after the ceiling robot has picked up a second red pump.

.. code-block:: yaml
  
    challenges:
      - dropped_part:
        robot: 'ceiling_robot'
        type: 'pump'
        color: 'red'
        drop_after: 5
        delay: 5


Detecting Faulty Gripper
----------------------------

.. important::
  To detect a faulty gripper the CCS needs a subscriber to the topic ``/ariac/{robot}_gripper_state``. This topic publishes messages of type ``ariac_msgs/msg/VacuumGripperState``, which has the structure :ref:`below<vacuum-gripper-state-yaml>`. The field ``attached`` can be checked in this challenge to know if the gripper is holding an object. 
  
  .. code-block:: bash
    :caption: VacuumGripperState.msg message file.
    :name: vacuum-gripper-state-yaml
    
    # VacuumGripperState.msg
    bool enabled  # is the succion enabled?
    bool attached # is an object attached to the gripper?
    string type   # type of the gripper attached to the arm




.. _target to robot malfunction:

Robot Malfunction
==================

The robot malfunction challenge simulates a robot malfunction. The robot can malfunction in some conditions (time, part placement, or submission) during the trial. The robot can malfunction even if it is not moving. When a robot malfunctions, it stops moving and cannot be controlled by the competitors' control system. The robot will remain in the same position until the malfunction is resolved. To specify how long a robot malfunctions, a time duration of the malfunction is specified in the trial configuration file.

  The goal of this challenge is to test the ability of the competitors' control system to use the other robot to complete the tasks that was being performed by the robot which is malfunctioning. 

.. note::
  It can happen that both robots malfunction at the same time. In this case, competitors's control system must wait until the malfunction is resolved before continuing with the trial.




Robot Malfunction Example
----------------------------

The robot malfunction challenge is specified with ``robot_malfunction`` as a subfield of ``challenges`` in the trial configuration file. The relevant fields for this agility challenge are listed below.

* ``duration``: The duration of the robot malfunction in seconds.
* ``robots_to_disable``: A list of robots that malfunction. It can be either ``'floor_robot'`` or ``'ceiling_robot'`` or both.
* :ref:`Conditions<target to conditions>` that can trigger the robot malfunction.

..
  * ``part_place_condition``: The challenge starts when a part of a specific type and color is placed on a specific AGV.
  * ``time_condition``: The challenge starts after a specific time.
  * ``submission_condition``: The challenge starts when a specific order is submitted.

Robot malfunctions can occur multiple times in the same trial. The example :ref:`below<robot-malfunction-yaml>` shows a robot malfunction challenge occurring 4 times in the same trial.


.. code-block:: yaml
  :caption: Example of multiple occurrences of the robot malfunction challenge in the same trial.
  :name: robot-malfunction-yaml
  
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

Detecting Robot Malfunctions
-----------------------------

.. important::
  To detect a robot malfunction, the CCS needs a subscriber to the topic ``/ariac/robot_health``. The message type for this topic is :ref:`ariac_msgs/msg/Robots<robots-health>` . The message contains Boolean-type fields which provide information on the health of the robots. The ``floor_robot`` field is ``true`` if the floor robot is healthy and ``false`` if it is malfunctioning. The ``ceiling_robot`` field is ``true`` if the ceiling robot is healthy and ``false`` if it is malfunctioning.

  .. code-block:: bash
    :caption: Robots.msg message file.
    :name: robots-health
    
    # Robots.msg
    bool floor_robot
    bool ceiling_robot


.. _target to sensor blackout:

Sensor Blackout
================

The sensor blackout challenge simulates a situation where some sensors stop reporting data during X seconds. The goal of this challenge is to test the ability of the CCS to use an internal world model to continue the tasks that were being performed before the blackout.

.. note::
  This challenge has been modified from previous ARIAC iterations. In previous iterations, the sensor blackout challenge affected all sensor types at once. In this iteration, the sensor blackout can be customized to affect only selected sensor types.
  

The sensor blackout challenge is triggered based on :ref:`conditions<target to conditions>`. When a sensor type blacks out, all sensors of this type stop publishing data on their respective topics. Once the challenge is resolved (after a duration), these sensors will start publishing  again. 

Sensor Blackout Example
---------------------------


The sensor blackout challenge is specified with ``sensor_blackout`` as a subfield of ``challenges`` in the trial configuration file. The relevant fields for this agility challenge are listed below.

* `duration`: The duration of the sensor blackout in seconds.
* `sensors_to_disable`: A list of sensor types to disable:

  * ``'break_beam'``
  * ``'proximity'``
  * ``'laser_profiler'``
  * ``'lidar'``
  * ``'camera'``
  * ``'logical_camera'``
* :ref:`Conditions<target to conditions>` to trigger the challenge.


The sensor blackout challenge can occur multiple times in the same trial. The example :ref:`below<sensor-blackout-yaml>` shows the challenge occurring twice in the same trial. One  occurrence of the challenge disables the break beam sensor type for 25 seconds when the competition time reaches 20 seconds. The other occurrence of the challenge disables the lidar and logical camera sensor types for 15 seconds when an order is submitted. 



.. code-block:: yaml
  :caption: Example of multiple occurrences of the sensor blackout challenge in the same trial.
  :name: sensor-blackout-yaml
  :emphasize-lines: 2,6

  challenges:
    - sensor_blackout:
        duration: 25.0
        sensors_to_disable: ['break_beam']
        time_condition: 20
    - sensor_blackout:
        duration: 15.0
        sensors_to_disable: ['lidar', 'logical_camera']
        submission_condition:
          order_id: 'MMB30H57'


Detecting Sensor Blackouts
-----------------------------

.. important::
  To detect a sensor blackout the CCS needs a subscriber to the topic ``/ariac/sensor_health``. The message type for this topic is :ref:`ariac_msgs/msg/Sensors<sensors-health>` . The message contains Boolean-type fields which provide information on the health of each sensor type. A ``true`` value indicates that all sensors for a sensor type are healthy (they are publishing) and a ``false`` value indicates that all sensors for a sensor type are malfunctioning (they are not publishing).

  .. code-block:: bash
    :caption: Sensors.msg message file.
    :name: sensors-health
    
    # Sensors.msg
    bool break_beam
    bool proximity
    bool laser_profiler
    bool lidar
    bool camera
    bool logical_camera


High-priority Orders
=====================

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

.. _target to human operator:

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
