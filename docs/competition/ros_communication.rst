.. _COMMUNICATIONS:

ROS Communication Overview
==========================

Topics
------


.. list-table:: List of topics with message types.
   :widths: 25 25 25 50
   :header-rows: 1
   :name: communications-topics

   * - Topic Name
     - Message Type
     - Message Definition
     - Description 
   * - ``/ariac/orders`` 
     - ``ariac_msgs/msg/Order``
     - :ref:`Order.msg <OrderMsg>`
     - Orders that the CCS should submit
   * - ``/ariac/competition_state``
     - ``ariac_msgs/msg/CompetitionState`` 
     - :ref:`CompatitionState.msg <CompetitionStateMsg>`
     - Current state of the competition 
   * - ``/ariac/bin_parts``
     - ``ariac_msgs/msg/BinParts`` 
     - :ref:`BinParts.msg <BinPartsMsg>`
     - Part information in each bin at program start-up 
   * - ``/ariac/conveyor_parts``
     - ``ariac_msgs/msg/ConveyorParts`` 
     - :ref:`ConveyorParts.msg <ConveyorPartsMsg>`
     - Parts that will come on the conveyor belt 

.. .. list-table:: List of topics with message types.
..    :widths: 25 25 25 50
..    :header-rows: 1
..    :name: communications-topics

..    * - Topic Name
..      - Message Type
..      - Message Definition
..      - Description 
  ..  * - ``/ariac/orders`` 
  ..    - ``ariac_msgs/msg/Order``
  ..    - :ref:`Order <OrderMsg>`
  ..    - Orders that the CCS should submit
  ..  * - ``/ariac/competition_state``
  ..    - ``ariac_msgs/msg/CompetitionState`` 
  ..    - Current state of the competition 
  ..    - t
  ..  * - ``/ariac/bin_parts``
  ..    - ``ariac_msgs/msg/BinParts`` 
  ..    - Parts in each bin at program start-up 
     - t
   * - ``/ariac/conveyor_parts``
     - ``ariac_msgs/msg/ConveyorParts`` 
     - Parts that will come on the conveyor belt 
     - t
   * - ``/ariac/agv{n}_status``
     - ``ariac_msgs/msg/AGVStatus``
     - State of the AGV {n} (location, position, velocity)
     - t
   * - ``/ariac/{robot}_gripper_state``
     - ``ariac_msgs/msg/VacuumGripperState``
     - State of {robot}'s gripper (enabled, attached, type)
     - t
   * - ``/ariac/conveyor_state``
     - ``ariac_msgs/msg/ConveyorBeltState``
     - State of the conveyor (enabled, power)
     - t
   * - ``/ariac/robot_health``
     - ``ariac_msgs/msg/Robots``
     - Health of the robots
     - t
   * - ``/ariac/sensor_health```
     - ``ariac_msgs/msg/Sensors``
     - Health of the sensors
     - t
   * - ``/ariac_human/state```
     - ``ariac_msgs/msg/HumanState``
     - Position and velocity of the human and the ceiling robot
     - t

Message Definitions
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash
    :caption: Order.msg
    :name: OrderMsg

    uint8 KITTING=0
    uint8 ASSEMBLY=1
    uint8 COMBINED=2

    string id
    uint8 type
    bool priority
    ariac_msgs/KittingTask kitting_task 
    ariac_msgs/AssemblyTask assembly_task
    ariac_msgs/CombinedTask combined_task

.. code-block:: bash
    :caption: CompetitionState.msg
    :name: CompetitionStateMsg

    uint8 IDLE=0    # competition cannot be started yet by the competitor
    uint8 READY=1   # competition can be started by the competitor
    uint8 STARTED=2 # competition has been started
    uint8 ORDER_ANNOUNCEMENTS_DONE=3 # all order announcements have been announced
    uint8 ENDED=4   # competition has ended

    uint8 competition_state # IDLE, READY, STARTED, ORDER_ANNOUNCEMENTS_DONE, ENDED

.. code-block:: bash
    :caption: BinParts.msg
    :name: BinPartsMsg

    ariac_msgs/BinInfo[] bins

.. code-block:: bash
    :caption: BinInfo.msg
    :name: BinInfoMsg

    uint8 BIN1=1
    uint8 BIN2=2
    uint8 BIN3=3
    uint8 BIN4=4
    uint8 BIN5=5
    uint8 BIN6=6
    uint8 BIN7=7
    uint8 BIN8=8

    uint8 bin_number
    ariac_msgs/PartLot[] parts

.. code-block:: bash
    :caption: PartLot.msg
    :name: PartLotMsg

    ariac_msgs/Part part
    uint8 quantity

.. code-block:: bash
    :caption: Part.msg
    :name: PartMsg

    # Constants for part color
    uint8 RED=0
    uint8 GREEN=1
    uint8 BLUE=2
    uint8 ORANGE=3
    uint8 PURPLE=4

    # Constants for part type
    uint8 BATTERY=10
    uint8 PUMP=11
    uint8 SENSOR=12
    uint8 REGULATOR=13

    uint8 color # RED, GREEN, BLUE, ORANGE, PURPLE
    uint8 type # BATTERY, PUMP, SENSOR, REGULATOR

.. code-block:: bash
    :caption: ConveyorParts.msg
    :name: ConveyorPartsMsg

    ariac_msgs/PartLot[] parts


.. code-block:: bash
    :caption: HumanState.msg
    :name: HumanStateMsg

    geometry_msgs/Point human_position
    geometry_msgs/Point robot_position
    geometry_msgs/Vector3 human_velocity
    geometry_msgs/Vector3 robot_velocity


..
    List of topics with the message type and a brief description.

    | Topic Name                     | MSG type                            | Description                                          |
    | ---                            | ---                                 | ---                                                  | 
    | `/ariac/orders`                | `ariac_msgs/msg/Order`              | Orders that the competitors should submit            |
    | `/ariac/competition_state`     | `ariac_msgs/msg/CompetitionState`   | Current state of the competition                     | 
    | `/ariac/bin_parts`             | `ariac_msgs/msg/BinParts`           | Parts in each bin at program start-up                |
    | `/ariac/conveyor_parts`        | `ariac_msgs/msg/ConveyorParts`      | Parts that will come on the conveyor belt            |
    | `/ariac/agv{n}_status`         | `ariac_msgs/msg/AGVStatus`          | State of the AGV {n} (location, position, velocity)  |
    | `/ariac/{robot}_gripper_state` | `ariac_msgs/msg/VacuumGripperState` | State of {robot}'s gripper (enabled, attached, type) |
    | `/ariac/conveyor_state`        | `ariac_msgs/msg/ConveyorBeltState`  | State of the conveyor (enabled, power)               |
    | `/ariac/robot_health`          | `ariac_msgs/msg/Robots`             | Health of the robots                                 |
    | `/ariac/sensor_health`         | `ariac_msgs/msg/Sensors`            | Health of the sensors                                |

Services
--------

.. list-table:: List of services with service types.
   :widths: 25 25 50
   :header-rows: 1
   :name: communications-services

   * - Service Name
     - Service type
     - Description  
   * - ``/ariac/start_competition``
     - ``std_srvs/srv/Trigger``
     - Start the competition   
   * - ``/ariac/end_competition``
     - ``std_srvs/srv/Trigger``
     - End the competition
   * - ``/ariac/submit_order``
     - ``ariac_msgs/srv/SubmitOrder``
     - Submit an order with the requested ``order_id`` 
   * - ``/ariac/perform_quality_check``
     - ``ariac_msgs/srv/PerformQualityCheck``
     - Check the quality of a kitting order with the requested ``order_id``
   * - ``/ariac/move_agv{n}``  
     - ``ariac_msgs/srv/MoveAGV``
     - Move the AGV {n} to the requested location  
   * - ``/ariac/agv{n}_lock_tray``  
     - ``std_srvs/srv/Trigger``
     - Lock a kit tray to AGV {n} 
   * - ``/ariac/agv{n}_unlock_tray``` 
     - ``std_srvs/srv/Trigger``
     - Unlock a kit tray to AGV {n} 
   * - ``/ariac/{robot}_enable_gripper``
     - ``ariac_msgs/srv/VacuumGripperControl``
     - Set the state of {robot}'s gripper to the request state
   * - ``/ariac/{robot}_change_gripper``
     - ``ariac_msgs/srv/ChangeGripper`` 
     - Change the type of {robot}'s gripper to the request type

..
    List of service with the service type and a brief description.

    | Service Name                    | SRV type                              | Description                                                        |
    | ---                             | ---                                   | ---                                                                | 
    | `/ariac/start_competition`      | `std_srvs/srv/Trigger`                | Start the competition                                              |
    | `/ariac/end_competition`        | `std_srvs/srv/Trigger`                | End the competition                                                | 
    | `/ariac/submit_order`           | `ariac_msgs/srv/SubmitOrder`          | Submit an order with the requested `order_id`                      |
    | `/ariac/perform_quality_check`  | `ariac_msgs/srv/PerformQualityCheck`  | Check the quality of a kitting order with the requested `order_id` |
    | `/ariac/move_agv{n}`            | `ariac_msgs/srv/MoveAGV`              | Move the AGV {n} to the requested location                         |
    | `/ariac/agv{n}_lock_tray`       | `std_srvs/srv/Trigger`                | Lock a kit tray to AGV {n}                                         |
    | `/ariac/agv{n}_unlock_tray`     | `std_srvs/srv/Trigger`                | Unlock a kit tray to AGV {n}                                       |
    | `/ariac/{robot}_enable_gripper` | `ariac_msgs/srv/VacuumGripperControl` | Set the state of {robot}'s gripper to the request state            |
    | `/ariac/{robot}_change_gripper` | `ariac_msgs/srv/ChangeGripper`        | Change the type of {robot}'s gripper to the request type           |

Sensor Topics
-------------

.. list-table:: List of sensor topics with message types.
   :widths: 25 50 50
   :header-rows: 1
   :name: communications-sensor-topics

   * - Sensor Type
     - Topic
     - Message  
   * - 'break_beam'
     - ``/ariac/sensors/{sensor_name}/status`` ``/ariac/sensors/{sensor_name}/status``
     - ``ariac_msgs/BreakBeamStatus`` ``ariac_msgs/BreakBeamStatus``
   * - 'proximity`
     - ``/ariac/sensors/{sensor_name}/scan``
     - ``sensor_msgs/Range``
   * - 'laser_profiler'
     - ``/ariac/sensors/{sensor_name}/scan`` 
     - ``sensor_msgs/LaserScan`` 
   * - 'lidar'
     - ``/ariac/sensors/{sensor_name}/scan``	
     - ``sensor_msgs/PointCloud``
   * - 'rgb_camera'
     - ``/ariac/sensors/{sensor_name}/rgb_image``
     - ``sensor_msgs/Image sensor_msgs/Image``
   * - 'rgbd_camera'
     - ``/ariac/sensors/{sensor_name}/rgb_image`` ``/ariac/sensors/{sensor_name}/depth_image``
     - ``sensor_msgs/Image``
   * - 'basic_logical_camera'
     - ``/ariac/sensors/{sensor_name}/image``
     - ``ariac_msgs/BasicLogicalCameraImage``
   * - 'advanced_logical_camera'
     - ``/ariac/sensors/{sensor_name}/image``
     - ``ariac_msgs/AdvancedLogicalCameraImage``

..
    List of sensor topics and their msg types:

    | Sensor Type               | Topic name(s)                                                                       |	MSG type                                              |
    | ---                       | ---                                                                                 | ---                                                   |
    | `break_beam`              | `/ariac/sensors/{sensor_name}/status` `/ariac/sensors/{sensor_name}/status`         | ariac_msgs/BreakBeamStatus ariac_msgs/BreakBeamStatus |
    | `proximity`               | `/ariac/sensors/{sensor_name}/scan`                                                 |	sensor_msgs/Range                                     |
    | `laser_profiler`          | `/ariac/sensors/{sensor_name}/scan`                                                 |	sensor_msgs/LaserScan                                 |
    | `lidar`	                  | `/ariac/sensors/{sensor_name}/scan`	                                                | sensor_msgs/PointCloud                                |
    | `rgb_camera`              | `/ariac/sensors/{sensor_name}/rgb_image`                                            |	sensor_msgs/Image sensor_msgs/Image                   |
    | `rgbd_camera`             | `/ariac/sensors/{sensor_name}/rgb_image` `/ariac/sensors/{sensor_name}/depth_image` | sensor_msgs/Image                                     |
    | `basic_logical_camera`    | `/ariac/sensors/{sensor_name}/image`                                                | ariac_msgs/BasicLogicalCameraImage                    |
    | `advanced_logical_camera` | `/ariac/sensors/{sensor_name}/image`                                                | ariac_msgs/AdvancedLogicalCameraImage                 |