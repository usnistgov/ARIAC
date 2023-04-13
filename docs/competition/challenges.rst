

.. _AGILITY_CHALLENGES:

==================
Agility Challenges
==================



There are 8 possible :term:`agility challenges<Agility Challenge>` in ARIAC 2023. 
A description of each challenge is provided below. Besides the :ref:`HUMANS` challenge, 
all other challenges can occur multiple times in a trial. 

.. note::
  A trial may consist of some of the challenges described in this page, may consist of no 
  challenge at all, or may consist of all the challenges.

.. _FAULTY_PARTS:

Faulty Parts
================

Faulty parts are parts that are not in good condition. They are not suitable for use in the competition. 
If an order is submitted with faulty parts, these parts are not considered for scoring. 
Faulty parts are identified by quality control sensors, which are attached 
to AGVs.

  The goal of this challenge is to test the ability of the CCS  to:

  #. Correctly use the quality check sensor to detect faulty parts. 
  #. Replace them with new parts.


Setup
----------------------------

The faulty parts challenge is set with the field :yamlname:`faulty_part` under the :yamlname:`challenges` field 
in the trial configuration file. 
Only the first part placed in a quadrant is faulty. 
In the example below, any first part placed in  quadrants 1 and 2 in the kitting tray required by 
order :yaml:`'MMB30H56'` is faulty. 
If these parts are removed and replaced with new parts, the new parts will be non-faulty.

.. code-block:: yaml

  challenges:
    - faulty_part:
        order_id: 'MMB30H56'
        quadrant1: true
        quadrant2: true


Detection
----------------------------


The quality control sensor located above an AGV is capable of 
detecting faulty parts. 
A quality check can be performed by calling the service :rosservice:`/ariac/perform_quality_check` (:term:`ariac_msgs/srv/PerformQualityCheck`). 
The argument passed to this service call is an order ID. 
When a faulty part is detected, the CCS has to discard the 
part and replace it with a new part. 
The new part will automatically be set to non-faulty by the AM.



More information on the fields of the service message is provided as follows:

  * The service returns a Boolean value for the field :yamlname:`valid_id` indicating whether or not the order ID is valid. An order ID is not valid if the order ID does not exist or if the quality check was already called 
  for this order ID.

  * The field :yamlname:`all_passed` is set to :yaml:`true` only if:

    * All parts in the kitting tray are NOT faulty.
    * All parts are present in the kitting tray (no empty quadrant).
    * All parts have the correct orientation (no flipped part).
    * All parts are of the correct type.
    * All parts are of the correct color.

  * The field :yamlname:`incorrect_tray` informs on whether or not the kitting task was performed in the correct kitting tray.
  * Information for each quadrant is reported as a :term:`ariac_msgs/msg/QualityIssue`.



.. _FLIPPED_PARTS:

Flipped Parts
================

The environment can be started with parts that are flipped. Flipped parts are parts that are upside down. When a part is spawned as flipped, the CCS is required to flip this part again so it ends up with the correct orientation. If an order is submitted with flipped parts, these parts are not considered for scoring. 

  The goal of this challenge is to evaluate the approach used by the CCS to flip a part. 

.. attention::
  Competitors should keep in mind that one of the two robots can malfunction at any point during the trial.
  This means that the CCS should be able to handle the case where 
  one of the robots is not available to flip a part.



Setup
----------------------------

Flipped parts apply to a specific part type and color in a specific bin or on the conveyor belt. 
To set parts as flipped, the :yamlname:`flipped` field in the trial configuration file must be set 
as :yaml:`true` for the corresponding parts. :numref:`flipped-parts-in-bin` describes all purple 
regulators as flipped in :yamlname:`bin3`. :numref:`flipped-parts-on-conveyor-belt` describes all 
orange batteries as flipped on the conveyor belt.

.. code-block:: yaml
  :caption: Setting flipped parts in a bin.
  :name: flipped-parts-in-bin

  bin3:
    - type: 'regulator'
      color: 'purple'
      slots: [2, 3]
      rotation: 'pi/6'
      flipped: true



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


Detection
----------------------------


Flipped parts detection is performed similarly to faulty parts detection. 
A quality check informs whether or not a part is flipped. See the :ref:`FAULTY_PARTS` section for more information on how to perform a quality check.




.. _target to faulty gripper:

Faulty Gripper
================

The faulty gripper challenge simulates a faulty gripper which can drop a part after the part has been picked up. The gripper can drop a part at any time during the trial. The gripper can drop a part that is in the gripper's grasp even if the gripper or robot is not moving. 

  The goal of this challenge is to test the ability of the CCS to: 
  
  #. Recognize that the part has dropped from the gripper. 
  #. Pick a part of the same type and color.

Setup
----------------------------

The faulty gripper challenge can be set up in the trial configuration file with the field :yamlname:`dropped_part` under the :yamlname:`challenges` field. :numref:`faulty-gripper-setup` describes a faulty gripper occuring 5 seconds after the ceiling robot has picked up a second red pump (specified with the :yamlname:`drop_after` field). Multiple occurrences of this challenge may be set up in the trial configuration file as seen in :numref:`multiple-faulty-gripper-setup`.


.. code-block:: yaml
  :caption: Setting up the faulty gripper challenge.
  :name: faulty-gripper-setup

    challenges:
      - dropped_part:
          robot: 'ceiling_robot'
          type: 'pump'
          color: 'red'
          drop_after: 1
          delay: 5



.. code-block:: yaml
  :caption: Multiple occurences of the faulty gripper challenge.
  :name: multiple-faulty-gripper-setup

    challenges:
      - dropped_part:
          robot: 'ceiling_robot'
          type: 'pump'
          color: 'red'
          drop_after: 1
          delay: 5
      - dropped_part:
          robot: 'floor_robot'
          type: 'battery'
          color: 'green'
          drop_after: 1
          delay: 3
      - dropped_part:
          robot: 'floor_robot'
          type: 'regulator'
          color: 'orange'
          drop_after: 2
          delay: 15

.. note::
    The gripper can drop a part even if the robot is not moving.


Detection
----------------------------


To detect a faulty gripper the CCS needs a subscriber to the topic :topic:`/ariac/{robot}_gripper_state` (:term:`ariac_msgs/msg/VacuumGripperState`). Checking the :yamlname:`attached` field of the message will inform whether or not the gripper is holding a part. If the gripper is not holding a part, the CCS can assume that the gripper has dropped the part.


.. _target to robot malfunction:

Robot Malfunction
==================

The robot malfunction challenge simulates a robot malfunction. The robot can malfunction under some :ref:`conditions <CONDITIONS>` during the trial. The robot can malfunction even if it is not moving. When a robot malfunctions, it stops moving and cannot be controlled by the CCS. The robot will remain in the same position until the malfunction is resolved. To specify how long a robot malfunctions, a time duration of the malfunction is specified in the trial configuration file.

  The goal of this challenge is to test the ability of the CCS to use the other robot to complete the tasks that was being performed by the robot which is malfunctioning. 

.. note::
  It can happen that both robots malfunction at the same time. 
  In this case, the CCS must wait until the malfunction is resolved before continuing with the trial.



Setup
----------------------------

The robot malfunction challenge is specified with the field :yamlname:`robot_malfunction` as a subfield of :yamlname:`challenges` in the trial configuration file. The relevant fields for this agility challenge are listed below.
  
  * :yamlname:`duration`: The duration of the robot malfunction in seconds.
  * :yamlname:`robots_to_disable`: A list of robots that malfunction. It can be either :yaml:`'floor_robot'` or :yaml:`'ceiling_robot'` or both.
  * :ref:`One condition <CONDITIONS>` that can trigger the robot malfunction.

Robot malfunctions can occur multiple times in the same trial. :numref:`robot-malfunction-yaml` shows a robot malfunction challenge occurring 4 times under different conditions in the same trial.


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

Detection
-----------------------------


To detect a robot malfunction, the CCS needs a subscriber to the topic :topic:`/ariac/robot_health` (:term:`ariac_msgs/msg/Robots`). The message contains Boolean-type fields which provide information on the health of the robots. A value of :yaml:`true` indicates that the robot is healthy and can be controlled by the CCS. A value of :yaml:`false` indicates that the robot is malfunctioning and cannot be controlled by the CCS.



.. _target to sensor blackout:

Sensor Blackout
================

The sensor blackout challenge simulates a situation where some sensors stop reporting data for :math:`x` seconds. 

  The goal of this challenge is to test the ability of the CCS to use an internal world model to continue the tasks that were being performed before the blackout.

The sensor blackout challenge is triggered based on :ref:`conditions <CONDITIONS>`. When a *sensor type* is disabled, all sensors of this type stop publishing data on their respective topics. Once the challenge is resolved (after a duration), these sensors will start publishing  again. 



Setup
---------------------------


The subfield :yamlname:`sensor_blackout` of :yamlname:`challenges` is used to describe a sensor blackout challenge.
The relevant fields for this agility challenge are listed below.
  
  * :yamlname:`duration`: The duration of the sensor blackout in seconds.
  * :yamlname:`sensors_to_disable`: A list of sensor types to disable:

    * :yaml:`'break_beam'`
    * :yaml:`'proximity'`
    * :yaml:`'laser_profiler'`
    * :yaml:`'lidar'`
    * :yaml:`'camera'`
    * :yaml:`'logical_camera'`
  * :ref:`One condition <CONDITIONS>` to trigger the challenge.


The sensor blackout challenge can occur multiple times in the same trial.
:numref:`sensor-blackout-yaml` shows the challenge occurring twice in the same trial. 
One  occurrence of the challenge disables the break beam sensor type for 25 seconds when the 
competition time reaches 20 seconds. The other occurrence of the challenge disables the lidar 
and logical camera sensor types for 15 seconds when an order is submitted. 



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


Detection
-----------------------------


To detect a sensor blackout the CCS needs a subscriber to 
the topic :topic:`/ariac/sensor_health` (:term:`ariac_msgs/msg/Sensors`). 
The message contains Boolean-type fields which provide information on the health of each sensor type. 
A :yaml:`true` value indicates that all sensors of a type are healthy (they are publishing to topics) 
and a :yaml:`false` value indicates that all sensors of a type are malfunctioning 
(they are not publishing to topics).



High-priority Orders
=====================

The high-priority orders challenge simulates an order that must be completed before a regular-priority order. The high-priority order must be completed and  submitted before the regular-priority order.

  The goal of this challenge is to test the ability of the CCS to prioritize  high-priority orders over regular-priority orders. This requires the CCS to  be able to detect when a high-priority order is announced and to switch task.


.. warning::
  A high-priority order can be announced in one of the two following :ref:`conditions <CONDITIONS>`: Time or part placement. The submission condition is not used to announce a high-priority order.

.. note::
  A high-priority order will only be announced when only regular-priority orders have been announced. A high-priority order will not be announced if there is already a high-priority order in the queue.


Setup
-----------------------------

To specify a high-priority order, the :yamlname:`priority` field is set to :yaml:`true` in the order description. :numref:`high-priority-order-yaml` shows a high-priority order for order :yaml:`'MMB30H57'` and a regular-priority order for order :yaml:`'MMB30H58'`.


.. code-block:: yaml
  :caption: Example of a high-priority order for order :yaml:`'MMB30H58'`.
  :name: high-priority-order-yaml

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


Detection
-------------------------------


To find out out the priority of an order, the CCS is required to parse messages published to the topic :topic:`/ariac/orders` (:term:`ariac_msgs/msg/Order`). For a high-priority order, the value for the field :yamlname:`priority` is set to :yaml:`true`. For a regular-priority order, the value for the field :yamlname:`priority` is set to :yaml:`false`.



Insufficient Parts
===================

The insufficient parts challenge simulates a situation where the workcell does not contain enough parts to complete one or multiple orders. 

  The goal of this challenge is to test whether or not the CCS is capable of identifying insufficient parts to complete one or multiple orders. When an insufficient parts challenge takes place, the CCS must submit incomplete orders.

Setup
-----------------------------

There is no specific field in the trial configuration file to specify this challenge.  :numref:`insufficient-parts-yaml` shows a trial configuration file where the workcell does not have enough parts to complete order :yaml:`'MMB30H58'`. The order requires 4 blue batteries but the whole workcell has only 2 blue batteries (located in bin1).

.. code-block:: yaml
  :caption: Example of insufficient parts challenge.
  :name: insufficient-parts-yaml

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




Detection
-------------------------------


To figure out if the insufficient parts challenge is part of a trial, the CCS can rely on two important topics to retrieve part type, color, and quantity from bins and the conveyor belt.

Bins
^^^^^

The topic :topic:`/ariac/bin_parts` (:term:`ariac_msgs/msg/BinParts`) outputs for each bin: The type, the color, and the quantity of parts. An  output from :console:`ros2 topic echo /ariac/bin_parts` is provided in  :numref:`bin-parts-outputs`. The output shows that bin1 contains 3 red pumps and 2 blue batteries.

  .. code-block:: console
    :class: no-copybutton
    :caption: Message published on the topic :topic:`/ariac/bin_parts`.
    :name: bin-parts-outputs

    ---
    bins:
    - bin_number: 1
      parts:
      - part:
          color: 0
          type: 11
        quantity: 3
      - part:
          color: 2
          type: 10
        quantity: 2
    ---

  .. note::
    Bins that do not contain parts are not included in the message.

Conveyor Belt
^^^^^^^^^^^^^^^
The topic :topic:`/ariac/conveyor_parts` (:term:`ariac_msgs/msg/ConveyorParts`) outputs information on parts that are expected to spawn on the conveyor belt. An output from :console:`ros2 topic echo /ariac/conveyor_parts` is provided in  :numref:`conveyor-parts-outputs`. The message shows that 2 red batteries,  2 green sensors, 3 blue regulators, and 1 orange pump will spawn on the conveyor belt.


  .. code-block:: console
    :class: no-copybutton
    :caption: Message published on the topic :topic:`/ariac/conveyor_parts`.
    :name: conveyor-parts-outputs

    ---
    parts:
    - part:
        color: 0
        type: 10
      quantity: 2
    - part:
        color: 1
        type: 12
      quantity: 2
    - part:
        color: 2
        type: 13
      quantity: 3
    - part:
        color: 3
        type: 11
      quantity: 1
    ---

.. _HUMANS:

Human
==============


The human challenge consists of a simulated human navigating the workcell. 

  The goal of this challenge is to test whether or not the CCS is capable of ensuring the safety of humans on the shop floor. 
  The ceiling robot has to keep a safe distance from the human at any time. 
  If the ceiling robot gets too close to the human, the human will be considered to be in danger and two events happen: 
  
  #. The human is teleported to a safe location.
  #. The ceiling robot's controllers are deactivated for 15 seconds, which is a penalty given to the CCS. 


When the human challenge is used in a trial, the simulated human is assigned one of the following behaviors: 

- **Indifferent**: The human operator follows a scripted path, regardless of the location of the robots in the environment.
- **Antagonistic**: During an arbitrary period of time, the human operator purposefully moves towards the ceiling robot to interfere with the robot's current task.
- **Helpful**: The human operator will stop moving once the ceiling robot is at a certain distance away from him.

.. note::
  The behavior of a human does not change within a trial, e.g., if the human is assigned the behavior :yaml:`'helpful'`, the human will always be helpful throughout the trial. 

  The human in the environment will go to each assembly station in the following order:

  #. Assembly station 4
  #. Assembly station 2
  #. Assembly station 1
  #. Assembly station 3

  When the human reaches assembly station 3, he will repeat the process from the beginning.

.. note::
  The penalty given to the CCS for the human challenge is based on the distance between the human and the ceiling robot. The formula for the distance is:

  .. math::

    d_{min} = k_H(t_1 + t_2)+ k_{R}t_{1} + B + \delta

  where :math:`t_1=1` is the maximum time between the actuation of the sensing function and the output signal switching devices to the off state, :math:`t_2=1.5` is the maximum response time of the machine (i.e., the time required to stop the machine), :math:`\delta=1` is an additional distance, based on the expected intrusion toward the critical zone prior to actuation of the protective equipment, :math:`k_H` is the speed of the intruding human, :math:`k_R` is the speed of the robot, and :math:`B=0` is the Euclidean distance required to bring the robot to a safe, controlled stop.


Setup
---------------------------


The subfield :yamlname:`human` of :yamlname:`challenges` is used to describe a human challenge. The relevant fields for this agility challenge are listed below.
  
  * :yamlname:`behavior`: The behavior of the human operator. The possible values are:

    - :yaml:`'indifferent'`
    - :yaml:`'antagonistic'`
    - :yaml:`'helpful'`
  * :ref:`One condition <CONDITIONS>` to trigger the challenge.


.. code-block:: yaml
  :caption: Human challenge setup in a trial file.
  :name: human-yaml

  challenges:
    - human:
        behavior: 'antagonistic'
        time_condition: 10 # starts 10 s after the start of the competition


Detection
-----------------------------

The pose of the human is published to the topic :topic:`/ariac_human/state` (:term:`ariac_msgs/msg/HumanState`).
An output from :console:`ros2 topic echo /ariac_human/state` is provided in  :numref:`human-state-outputs`.

  .. code-block:: console
    :class: no-copybutton
    :caption: Message published on the topic :topic:`/ariac_human/state`.
    :name: human-state-outputs

    ---
    human_position:
      x: -14.993921250341705
      y: -9.99998557033615
      z: 0.010023161632176515
    robot_position:
      x: -7.0000003262450905
      y: 8.445047061655941e-08
      z: 0.7000000000000002
    human_velocity:
      x: 5.6589307392557084e-05
      y: -1.1679465760540981e-06
      z: 2.8776304097214153e-05
    robot_velocity:
      x: -9.607729520546026e-10
      y: 1.325746825962516e-10
      z: 0.0
    ---

