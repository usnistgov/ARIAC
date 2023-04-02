.. _COMMUNICATIONS:


ROS Communication Overview
==========================

This section shows the ROS topics and services that are used to communicate between the CCS and the ARIAC system. The definition of each message and service type is also provided.

Topics
------


.. list-table:: List of topics.
   :widths: auto
   :header-rows: 1
   :name: communications-topics

   * - Topic Name
     - Message Type
     - Message Definition
     - Description 
   * - :topic:`/ariac/orders` 
     - :gray:`ariac_msgs/msg/Order`
     - :term:`Order.msg`
     - Orders that the CCS should submit
   * - :topic:`/ariac/competition_state`
     - :gray:`ariac_msgs/msg/CompetitionState` 
     - :term:`CompetitionState.msg`
     - Current state of the competition 
   * - :topic:`/ariac/bin_parts`
     - :gray:`ariac_msgs/msg/BinParts` 
     - :term:`BinParts.msg`
     - Part information in each bin at program start-up 
   * - :topic:`/ariac/conveyor_parts`
     - :gray:`ariac_msgs/msg/ConveyorParts`
     - :term:`ConveyorParts.msg`
     - Parts that will come on the conveyor belt 
   * - :topic:`/ariac/agv{n}_status`
     - :gray:`ariac_msgs/msg/AGVStatus`
     - :term:`AGVStatus.msg`
     - State of the AGV {n} (location, position, velocity)
   * - :topic:`/ariac/{robot}_gripper_state`
     - :gray:`ariac_msgs/msg/VacuumGripperState`
     - :term:`VacuumGripperState.msg`
     - State of {robot}'s gripper (enabled, attached, type)
   * - :topic:`/ariac/conveyor_state`
     - :gray:`ariac_msgs/msg/ConveyorBeltState`
     - :term:`ConveyorBeltState.msg`
     - State of the conveyor (enabled, power)
   * - :topic:`/ariac/robot_health`
     - :gray:`ariac_msgs/msg/Robots`
     - :term:`Robots.msg`
     - Health of the robots (enabled or disabled)
   * - :topic:`/ariac/sensor_health`
     - :gray:`ariac_msgs/msg/Sensors`
     - :term:`Sensors.msg`
     - Health of the sensors (enabled or disabled)
   * - :topic:`/ariac_human/state`
     - :gray:`ariac_msgs/msg/HumanState`
     - :term:`Humanstate.msg`
     - Position and velocity of the human and the ceiling robot
                              |

Message Definitions
^^^^^^^^^^^^^^^^^^^


.. glossary::
    :sorted:

    Order.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY=1
        uint8 COMBINED=2

        string id
        uint8 type # KITTING, ASSEMBLY, or COMBINED
        bool priority
        ariac_msgs/KittingTask kitting_task 
        ariac_msgs/AssemblyTask assembly_task
        ariac_msgs/CombinedTask combined_task

    CompetitionState.msg
      .. code-block:: text
        
        uint8 IDLE=0   
        uint8 READY=1  
        uint8 STARTED=2 
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 
        uint8 ENDED=4 

        uint8 competition_state

    BinParts.msg
      .. code-block:: text
        
        ariac_msgs/BinInfo[] bins


    ConveyorParts.msg
      .. code-block:: text
        
        ariac_msgs/PartLot[] parts

    AGVStatus.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

    VacuumGripperState.msg
      .. code-block:: text

        bool enabled # is the suction enabled?
        bool attached # is an object attached to the gripper?
        string type # type of the gripper

    ConveyorBeltState.msg
      .. code-block:: text

        float64 power
        bool enabled  

    Robots.msg
      .. code-block:: text

        bool floor_robot
        bool ceiling_robot

    Sensors.msg
      .. code-block:: text

        bool break_beam
        bool proximity
        bool laser_profiler
        bool lidar
        bool camera
        bool logical_camera

    HumanState.msg
      .. code-block:: text

        geometry_msgs/Point human_position
        geometry_msgs/Point robot_position
        geometry_msgs/Vector3 human_velocity
        geometry_msgs/Vector3 robot_velocity

    Part.msg
      .. code-block:: text
        
        # part color
        uint8 RED=0
        uint8 GREEN=1
        uint8 BLUE=2
        uint8 ORANGE=3
        uint8 PURPLE=4

        # part type
        uint8 BATTERY=10
        uint8 PUMP=11
        uint8 SENSOR=12
        uint8 REGULATOR=13

        uint8 color
        uint8 type

    PartPose.msg
      .. code-block:: text
        
        ariac_msgs/Part part
        geometry_msgs/Pose pose

    AdvancedLogicalCameraImage.msg
      .. code-block:: text
        
        ariac_msgs/PartPose[] part_poses
        ariac_msgs/KitTrayPose[] tray_poses
        geometry_msgs/Pose sensor_pose

Sensor Topics
-------------

.. list-table:: List of sensor topics.
   :widths: auto
   :header-rows: 1
   :name: communications-sensor-topics

   * - Sensor Type
     - Topic Name
     - Message Type 
     - Message Definition
   * - break_beam
     - :topic:`/ariac/sensors/{sensor_name}/change`
     - :gray:`ariac_msgs/BreakBeamStatus`
     - `BreakBeamStatus.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/BreakBeamStatus.msg>`_
   * - 
     - :topic:`/ariac/sensors/{sensor_name}/status`
     - :gray:`ariac_msgs/BreakBeamStatus`
     - `BreakBeamStatus.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/BreakBeamStatus.msg>`_
   * - proximity
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :gray:`sensor_msgs/Range`
     - `Range.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/Range.html>`_
   * - laser_profiler
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :gray:`sensor_msgs/LaserScan` 
     - `LaserScan.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/LaserScan.html>`_
   * - lidar
     - :topic:`/ariac/sensors/{sensor_name}/scan`	
     - :gray:`sensor_msgs/PointCloud`
     - `PointCloud.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud.html>`_
   * - rgb_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image`
     - :gray:`sensor_msgs/Image`
     - `Image.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/Image.html>`_
   * - rgbd_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image`
     - :gray:`sensor_msgs/Image`
     - `Image.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/Image.html>`_
   * - 
     - :topic:`/ariac/sensors/{sensor_name}/depth_image`
     - :gray:`sensor_msgs/Image`
     - `Image.msg <https://docs.ros2.org/galactic/api/sensor_msgs/msg/Image.html>`_
   * - basic_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :gray:`ariac_msgs/BasicLogicalCameraImage`
     - `BasicLogicalCameraImage.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/BasicLogicalCameraImage.msg>`_
   * - advanced_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :gray:`ariac_msgs/AdvancedLogicalCameraImage`
     - `AdvancedLogicalCameraImage.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/AdvancedLogicalCameraImage.msg>`_



Services
--------

.. list-table:: List of services.
   :widths: auto
   :header-rows: 1
   :name: communications-services

   * - Service Name
     - Service type
     - Service Definition
     - Description  
   * - :rosservice:`/ariac/start_competition`
     - :gray:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Start the competition   
   * - :rosservice:`/ariac/end_competition`
     - :gray:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - End the competition
   * - :rosservice:`/ariac/submit_order`
     - :gray:`ariac_msgs/srv/SubmitOrder`
     - `SubmitOrder.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/SubmitOrder.srv>`_
     - Submit an order with the requested **order_id**
   * - :rosservice:`/ariac/perform_quality_check`
     - :gray:`ariac_msgs/srv/PerformQualityCheck`
     - `PerformQualityCheck.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/PerformQualityCheck.srv>`_
     - Check the quality of a kitting order with the requested **order_id**
   * - :rosservice:`/ariac/get_pre_assembly_poses`
     - :gray:`ariac_msgs/srv/GetPreAssemblyPoses`
     - `GetPreAssemblyPoses.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/GetPreAssemblyPoses.srv>`_
     - Get the pose of parts on the AGVs prior to assembly for an assembly or combined order with **order_id**
   
       .. _moveAGV:
   * - :rosservice:`/ariac/move_agv{n}` 
     - :gray:`ariac_msgs/srv/MoveAGV`
     - `MoveAGV.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/MoveAGV.srv>`_
     - Move the AGV {n} to the requested location  
   * - :rosservice:`/ariac/agv{n}_lock_tray` 
     - :gray:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Lock a kit tray to AGV {n} 
   * - :rosservice:`/ariac/agv{n}_unlock_tray`
     - :gray:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Unlock a kit tray to AGV {n} 
   * - :rosservice:`/ariac/{robot}_enable_gripper`
     - :gray:`ariac_msgs/srv/VacuumGripperControl`
     - `VacuumGripperControl.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/VacuumGripperControl.srv>`_
     - Set the state of {robot}'s gripper to the request state
   * - :rosservice:`/ariac/{robot}_change_gripper`
     - :gray:`ariac_msgs/srv/ChangeGripper` 
     - `ChangeGripper.srv <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/srv/ChangeGripper.srv>`_
     - Change the type of {robot}'s gripper to the request type


Message and Service Definitions
-------------------------------


.. glossary::
    :sorted:

    Order.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY=1
        uint8 COMBINED=2

        string id
        uint8 type # KITTING, ASSEMBLY, or COMBINED
        bool priority
        ariac_msgs/KittingTask kitting_task 
        ariac_msgs/AssemblyTask assembly_task
        ariac_msgs/CombinedTask combined_task

    CompetitionState.msg
      .. code-block:: text
        
        uint8 IDLE=0   
        uint8 READY=1  
        uint8 STARTED=2 
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 
        uint8 ENDED=4 

        uint8 competition_state

    BinParts.msg
      .. code-block:: text
        
        ariac_msgs/BinInfo[] bins


    ConveyorParts.msg
      .. code-block:: text
        
        ariac_msgs/PartLot[] parts

    AGVStatus.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

    VacuumGripperState.msg
      .. code-block:: text

        bool enabled # is the suction enabled?
        bool attached # is an object attached to the gripper?
        string type # type of the gripper

    ConveyorBeltState.msg
      .. code-block:: text

        float64 power
        bool enabled  

    Robots.msg
      .. code-block:: text

        bool floor_robot
        bool ceiling_robot

    Sensors.msg
      .. code-block:: text

        bool break_beam
        bool proximity
        bool laser_profiler
        bool lidar
        bool camera
        bool logical_camera

    HumanState.msg
      .. code-block:: text

        geometry_msgs/Point human_position
        geometry_msgs/Point robot_position
        geometry_msgs/Vector3 human_velocity
        geometry_msgs/Vector3 robot_velocity

    AGVStatus.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

    AGVStatus.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

    AGVStatus.msg
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity


    


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