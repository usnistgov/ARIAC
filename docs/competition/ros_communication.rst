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
     - Description 
   * - :topic:`/ariac/orders` 
     - :term:`ariac_msgs/msg/Order`
     - Orders that the CCS should submit
   * - :topic:`/ariac/competition_state`
     - :term:`ariac_msgs/msg/CompetitionState`
     - Current state of the competition 
   * - :topic:`/ariac/bin_parts`
     - :term:`ariac_msgs/msg/BinParts`
     - Part information in each bin at program start-up 
   * - :topic:`/ariac/conveyor_parts`
     - :term:`ariac_msgs/msg/ConveyorParts`
     - Parts that will come on the conveyor belt 
   * - :topic:`/ariac/agv{n}_status`
     - :term:`ariac_msgs/msg/AGVStatus`
     - State of the AGV {n} (location, position, velocity)
   * - :topic:`/ariac/{robot}_gripper_state`
     - :term:`ariac_msgs/msg/VacuumGripperState`
     - State of {robot}'s gripper (enabled, attached, type)
   * - :topic:`/ariac/conveyor_state`
     - :term:`ariac_msgs/msg/ConveyorBeltState`
     - State of the conveyor (enabled, power)
   * - :topic:`/ariac/robot_health`
     - :term:`ariac_msgs/msg/Robots`
     - Health of the robots (enabled or disabled)
   * - :topic:`/ariac/sensor_health`
     - :term:`ariac_msgs/msg/Sensors`
     - Health of the sensors (enabled or disabled)
   * - :topic:`/ariac_human/state`
     - :term:`ariac_msgs/msg/HumanState`
     - Position and velocity of the human and the ceiling robot
                              |



Sensor Topics
-------------

.. list-table:: List of sensor topics.
   :widths: auto
   :header-rows: 1
   :name: communications-sensor-topics

   * - Sensor Type
     - Topic Name
     - Message Type 
   * - break_beam
     - :topic:`/ariac/sensors/{sensor_name}/change`
     - :term:`ariac_msgs/msg/BreakBeamStatus`
   * - 
     - :topic:`/ariac/sensors/{sensor_name}/status`
     - :term:`ariac_msgs/msg/BreakBeamStatus`
   * - proximity
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :term:`sensor_msgs/msg/Range`
   * - laser_profiler
     - :topic:`/ariac/sensors/{sensor_name}/scan`
     - :term:`sensor_msgs/msg/LaserScan`
   * - lidar
     - :topic:`/ariac/sensors/{sensor_name}/scan`	
     - :term:`sensor_msgs/msg/PointCloud`
   * - rgb_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image`
     - :term:`sensor_msgs/msg/Image`
   * - rgbd_camera
     - :topic:`/ariac/sensors/{sensor_name}/rgb_image`
     - :term:`sensor_msgs/msg/Image`
   * - 
     - :topic:`/ariac/sensors/{sensor_name}/depth_image`
     - :term:`sensor_msgs/msg/Image`
   * - basic_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :term:`ariac_msgs/msg/BasicLogicalCameraImage`
   * - advanced_logical_camera
     - :topic:`/ariac/sensors/{sensor_name}/image`
     - :term:`ariac_msgs/msg/AdvancedLogicalCameraImage`



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
     - :term:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - Start the competition   
   * - :rosservice:`/ariac/end_competition`
     - :term:`std_srvs/srv/Trigger`
     - `Trigger.srv <https://docs.ros2.org/galactic/api/std_srvs/srv/Trigger.html>`_
     - End the competition
   * - :rosservice:`/ariac/submit_order`
     - :term:`ariac_msgs/srv/SubmitOrder`
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



Message Definitions
-------------------


.. glossary::
    :sorted:

    ariac_msgs/msg/Order
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

    ariac_msgs/msg/CompetitionState
      .. code-block:: text
        
        uint8 IDLE=0   
        uint8 READY=1  
        uint8 STARTED=2 
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 
        uint8 ENDED=4 

        uint8 competition_state

    ariac_msgs/msg/BinParts
      .. code-block:: text
        
        ariac_msgs/BinInfo[] bins


    ariac_msgs/msg/ConveyorParts
      .. code-block:: text
        
        ariac_msgs/PartLot[] parts

    ariac_msgs/msg/AGVStatus
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3
        uint8 UNKNOWN=99

        int8 location
        float64 position
        float64 velocity

    ariac_msgs/msg/VacuumGripperState
      .. code-block:: text

        bool enabled # is the suction enabled?
        bool attached # is an object attached to the gripper?
        string type # type of the gripper

    ariac_msgs/msg/ConveyorBeltState
      .. code-block:: text

        float64 power
        bool enabled  

    ariac_msgs/msg/Robots
      .. code-block:: text

        bool floor_robot
        bool ceiling_robot

    ariac_msgs/msg/Sensors
      .. code-block:: text

        bool break_beam
        bool proximity
        bool laser_profiler
        bool lidar
        bool camera
        bool logical_camera

    ariac_msgs/msg/HumanState
      .. code-block:: text

        geometry_msgs/Point human_position
        geometry_msgs/Point robot_position
        geometry_msgs/Vector3 human_velocity
        geometry_msgs/Vector3 robot_velocity

    ariac_msgs/msg/Part
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

    ariac_msgs/msg/PartPose
      .. code-block:: text
        
        ariac_msgs/Part part
        geometry_msgs/Pose pose

    ariac_msgs/msg/AdvancedLogicalCameraImage
      .. code-block:: text
        
        ariac_msgs/PartPose[] part_poses
        ariac_msgs/KitTrayPose[] tray_poses
        geometry_msgs/Pose sensor_pose

    ariac_msgs/msg/BreakBeamStatus
      .. code-block:: text
        
        std_msgs/Header header
        bool object_detected

    sensor_msgs/msg/Range
      .. code-block:: text
        
        uint8 ULTRASOUND=0
        uint8 INFRARED=1
        std_msgs/msg/Header header
        uint8 radiation_type
        float field_of_view
        float min_range
        float max_range
        float range

    sensor_msgs/msg/LaserScan
      .. code-block:: text
        
        std_msgs/msg/Header header
        float angle_min
        float angle_max
        float angle_increment
        float time_increment
        float scan_time
        float range_min
        float range_max
        float[] ranges
        float[] intensities

    sensor_msgs/msg/PointCloud
      .. code-block:: text
        
        std_msgs/msg/Header header
        geometry_msgs/msg/Point32[] points
        sensor_msgs/msg/ChannelFloat32[] channels

    sensor_msgs/msg/Image
      .. code-block:: text
        
        std_msgs/msg/Header header
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data

    ariac_msgs/msg/BasicLogicalCameraImage
      .. code-block:: text
        
        geometry_msgs/Pose[] part_poses
        geometry_msgs/Pose[] tray_poses
        geometry_msgs/Pose sensor_pose


Service Definitions
-------------------


.. glossary::
    :sorted:

    std_srvs/srv/Trigger
      .. code-block:: text

        ---
        boolean success
        string message

      - ``success``: True if the service call was successful, False otherwise
      - ``message``: A message describing the result of the service call

    ariac_msgs/srv/SubmitOrder
      .. code-block:: text

        string order_id
        ---
        bool success
        string message

      - ``order_id``: The ID of the order to be submitted
      - ``success``: True if the order was submitted successfully, False otherwise
      - ``message``: A message describing the result of the service call