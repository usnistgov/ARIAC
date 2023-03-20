.. _COMMUNICATIONS:

ROS Communication Overview
==========================

This section shows the ROS topics and services that are used to communicate between the CCS and the ARIAC system. The definition of each message and service type is also provided.

Topics
------


.. list-table:: List of topics with message types.
   :widths: 50 100 25 50
   :header-rows: 1
   :name: communications-topics

   * - Topic Name
     - Message Type
     - Message Definition
     - Description 
   * - ``/ariac/orders`` 
     - ``ariac_msgs/msg/Order``
     - `Order.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/Order.msg>`_
     - Orders that the CCS should submit
   * - ``/ariac/competition_state``
     - ``ariac_msgs/msg/CompetitionState`` 
     - `CompetitionState.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/CompetitionState.msg>`_
     - Current state of the competition 
   * - ``/ariac/bin_parts``
     - ``ariac_msgs/msg/BinParts`` 
     - `BinParts.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/BinParts.msg>`_
     - Part information in each bin at program start-up 
   * - ``/ariac/conveyor_parts``
     - ``ariac_msgs/msg/ConveyorParts`` 
     - `ConveyorParts.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/ConveyorParts.msg>`_
     - Parts that will come on the conveyor belt 
   * - ``/ariac/agv{n}_status``
     - ``ariac_msgs/msg/AGVStatus``
     - `AGVStatus.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/AGVStatus.msg>`_
     - State of the AGV {n} (location, position, velocity)
   * - ``/ariac/{robot}_gripper_state``
     - ``ariac_msgs/msg/VacuumGripperState``
     - `VacuumGripperState.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/VacuumGripperState.msg>`_
     - State of {robot}'s gripper (enabled, attached, type)
   * - ``/ariac/conveyor_state``
     - ``ariac_msgs/msg/ConveyorBeltState``
     - `ConveyorBeltState.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/ConveyorBeltState.msg>`_
     - State of the conveyor (enabled, power)
   * - ``/ariac/robot_health``
     - ``ariac_msgs/msg/Robots``
     - `Robots.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/Robots.msg>`_
     - Health of the robots (enabled or disabled)
   * - ``/ariac/sensor_health```
     - ``ariac_msgs/msg/Sensors``
     - `Sensors.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/Sensors.msg>`_
     - Health of the sensors (enabled or disabled)
   * - ``/ariac_human/state```
     - ``ariac_msgs/msg/HumanState``
     - `Humanstate.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/HumanState.msg>`_
     - Position and velocity of the human and the ceiling robot

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
   :widths: 100 60 25 50
   :header-rows: 1
   :name: communications-services

   * - Service Name
     - Service type
     - Service Definition
     - Description  
   * - ``/ariac/start_competition``
     - ``std_srvs/srv/Trigger``
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Start the competition   
   * - ``/ariac/end_competition``
     - ``std_srvs/srv/Trigger``
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - End the competition
   * - ``/ariac/submit_order``
     - ``ariac_msgs/srv/SubmitOrder``
     - `SubmitOrder.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/SubmitOrder.srv>`_
     - Submit an order with the requested ``order_id`` 
   * - ``/ariac/perform_quality_check``
     - ``ariac_msgs/srv/PerformQualityCheck``
     - `PerformQualityCheck.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/PerformQualityCheck.srv>`_
     - Check the quality of a kitting order with the requested ``order_id``
   * - ``/ariac/get_pre_assembly_poses``
     - ``ariac_msgs/srv/GetPreAssemblyPoses``
     - `GetPreAssemblyPoses.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/GetPreAssemblyPoses.srv>`_
     - Get the pose of parts on the AGVs prior to assembly for an assembly or combined order with ``order_id``
   * - ``/ariac/move_agv{n}``  
     - ``ariac_msgs/srv/MoveAGV``
     - `MoveAGV.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/MoveAGV.srv>`_
     - Move the AGV {n} to the requested location  
   * - ``/ariac/agv{n}_lock_tray``  
     - ``std_srvs/srv/Trigger``
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Lock a kit tray to AGV {n} 
   * - ``/ariac/agv{n}_unlock_tray``` 
     - ``std_srvs/srv/Trigger``
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Unlock a kit tray to AGV {n} 
   * - ``/ariac/{robot}_enable_gripper``
     - ``ariac_msgs/srv/VacuumGripperControl``
     - `VacuumGripperControl.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/VacuumGripperControl.srv>`_
     - Set the state of {robot}'s gripper to the request state
   * - ``/ariac/{robot}_change_gripper``
     - ``ariac_msgs/srv/ChangeGripper`` 
     - `ChangeGripper.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/ChangeGripper.srv>`_
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
     - Topic Name
     - Message Type 
   * - break_beam
     - ``/ariac/sensors/{sensor_name}/change`` ``/ariac/sensors/{sensor_name}/status``
     - ``ariac_msgs/BreakBeamStatus`` ``ariac_msgs/BreakBeamStatus``
   * - proximity
     - ``/ariac/sensors/{sensor_name}/scan``
     - ``sensor_msgs/Range``
   * - laser_profiler
     - ``/ariac/sensors/{sensor_name}/scan`` 
     - ``sensor_msgs/LaserScan`` 
   * - lidar
     - ``/ariac/sensors/{sensor_name}/scan``	
     - ``sensor_msgs/PointCloud``
   * - rgb_camera
     - ``/ariac/sensors/{sensor_name}/rgb_image``
     - ``sensor_msgs/Image sensor_msgs/Image``
   * - rgbd_camera
     - ``/ariac/sensors/{sensor_name}/rgb_image`` ``/ariac/sensors/{sensor_name}/depth_image``
     - ``sensor_msgs/Image``
   * - basic_logical_camera
     - ``/ariac/sensors/{sensor_name}/image``
     - ``ariac_msgs/BasicLogicalCameraImage``
   * - advanced_logical_camera
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