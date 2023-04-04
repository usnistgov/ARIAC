.. _COMMUNICATIONS:


ROS Communication Overview
==========================

This section shows the ROS topics and services that are used to communicate between the :term:`Competitor Control System (CCS)` and the ARIAC system. The definition of each message and service type is also provided.

Topics
------


.. list-table:: List of topics.
   :widths: auto
   :header-rows: 1
   :name: communications-topics

   * - Topic Name
     - Message Definition
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
     - State of the AGV ``{n}`` (location, position, velocity)
   * - :topic:`/ariac/{robot}_gripper_state`
     - :term:`ariac_msgs/msg/VacuumGripperState`
     - State of ``{robot}``'s gripper (enabled, attached, type)
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





Sensor Topics
-------------

.. list-table:: List of sensor topics.
   :widths: auto
   :header-rows: 1
   :name: communications-sensor-topics

   * - Sensor Type
     - Topic Name
     - Message Definition 
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
     - Service Definition
     - Description  
   * - :rosservice:`/ariac/start_competition`
     - :term:`std_srvs/srv/Trigger`
     - Start the competition   
   * - :rosservice:`/ariac/end_competition`
     - :term:`std_srvs/srv/Trigger`
     - End the competition
   * - :rosservice:`/ariac/submit_order`
     - :term:`ariac_msgs/srv/SubmitOrder`
     - Submit an order with the requested **order_id**
   * - :rosservice:`/ariac/perform_quality_check`
     - :term:`ariac_msgs/srv/PerformQualityCheck`
     - Check the quality of a kitting order with the requested **order_id**
   * - :rosservice:`/ariac/get_pre_assembly_poses`
     - :term:`ariac_msgs/srv/GetPreAssemblyPoses`
     - Get the pose of parts on the AGVs prior to assembly for an assembly or combined order with **order_id**
   * - :rosservice:`/ariac/move_agv{n}` 
     - :term:`ariac_msgs/srv/MoveAGV`
     - Move the AGV ``{n}`` to the requested location  
   * - :rosservice:`/ariac/agv{n}_lock_tray` 
     - :term:`std_srvs/srv/Trigger`
     - Lock a kit tray to AGV ``{n}`` 
   * - :rosservice:`/ariac/agv{n}_unlock_tray`
     - :term:`std_srvs/srv/Trigger`
     - Unlock a kit tray to AGV ``{n}`` 
   * - :rosservice:`/ariac/{robot}_enable_gripper`
     - :term:`ariac_msgs/srv/VacuumGripperControl`
     - Set the state of ``{robot}``'s gripper to the request state
   * - :rosservice:`/ariac/{robot}_change_gripper`
     - :term:`ariac_msgs/srv/ChangeGripper`
     - Change the type of ``{robot}``'s gripper to the request type



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

      - ``id``: The unique identifier for the order
      - ``type``: The type of order. One of the following:

        - ``KITTING``: A kitting order
        - ``ASSEMBLY``: An assembly order
        - ``COMBINED``: A combined order
      - ``priority``: Whether the order is a priority order
      - ``kitting_task``: The kitting task for the order
      - ``assembly_task``: The assembly task for the order
      - ``combined_task``: The combined task for the order

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/KittingTask`
        - :term:`ariac_msgs/msg/AssemblyTask`
        - :term:`ariac_msgs/msg/CombinedTask`

    ariac_msgs/msg/KittingTask
      .. code-block:: text

        uint8 KITTING=0
        uint8 ASSEMBLY_FRONT=1
        uint8 ASSEMBLY_BACK=2
        uint8 WAREHOUSE=3

        uint8 agv_number
        int8 tray_id
        uint8 destination
        ariac_msgs/KittingPart[] parts

      - ``agv_number``: The AGV number to deliver the kit to (1, 2, 3, or 4)
      - ``tray_id``: The tray number to deliver the kit to (1, 2, 3, 4, 5, or 6)
      - ``destination``: The destination of the kit.  One of the following values:

        - ``KITTING``: The kit is to be delivered to the kitting station
        - ``ASSEMBLY_FRONT``: The kit is to be delivered to the front assembly station (``as1`` or ``as3`` depending on the AGV number)
        - ``ASSEMBLY_BACK``: The kit is to be delivered to the back assembly station (``as2`` or ``as4`` depending on the AGV number)
        - ``WAREHOUSE``: The kit is to be delivered to the warehouse

      - ``parts``: The parts to be placed in the kit

      .. seealso:: :term:`ariac_msgs/msg/KittingPart`


    ariac_msgs/msg/AssemblyTask
      .. code-block:: text

        uint8 AS1=1
        uint8 AS2=2
        uint8 AS3=3
        uint8 AS4=4

        uint8[] agv_numbers
        uint8 station
        ariac_msgs/AssemblyPart[] parts

      - ``agv_numbers``: The AGVs which contain parts for assembly
      - ``station``: The assembly station to assemble the parts at.  One of the following values:

        - ``AS1``: The front assembly station for AGV 1 and 2
        - ``AS2``: The back assembly station for AGV 1 and 2
        - ``AS3``: The front assembly station for AGV 3 and 4
        - ``AS4``: The back assembly station for AGV 3 and 4
      - ``parts``: The parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/AssemblyPart`

    ariac_msgs/msg/CombinedTask
      .. code-block:: text

        uint8 AS1=1
        uint8 AS2=2
        uint8 AS3=3
        uint8 AS4=4

        uint8 station
        ariac_msgs/AssemblyPart[] parts

      - ``station``: The assembly station to assemble the parts at.  One of the following values:

        - ``AS1``: The front assembly station for AGV 1 and 2
        - ``AS2``: The back assembly station for AGV 1 and 2
        - ``AS3``: The front assembly station for AGV 3 and 4
        - ``AS4``: The back assembly station for AGV 3 and 4
      - ``parts``: The parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/AssemblyPart`

    ariac_msgs/msg/AssemblyPart
      .. code-block:: text

        ariac_msgs/Part part
        geometry_msgs/PoseStamped assembled_pose
        geometry_msgs/Vector3 install_direction

      - ``part``: The part to be assembled
      - ``assembled_pose``: The pose of the part in the assembly station
      - ``install_direction``: The direction the part should be installed in the assembly station

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/Part`
        - `geometry_msgs/msg/PoseStamped <https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html>`_
        - `geometry_msgs/msg/Vector3 <https://docs.ros2.org/latest/api/geometry_msgs/msg/Vector3.html>`_

    ariac_msgs/msg/KittingPart
      .. code-block:: text

        uint8 QUADRANT1=1
        uint8 QUADRANT2=2
        uint8 QUADRANT3=3
        uint8 QUADRANT4=4

        ariac_msgs/Part part
        uint8 quadrant

      - ``part``: The part to be placed in the kit
      - ``quadrant``: The quadrant of the kit to place the part in.  One of the following values:

        - ``QUADRANT1``: The first quadrant of the kit
        - ``QUADRANT2``: The second quadrant of the kit
        - ``QUADRANT3``: The third quadrant of the kit
        - ``QUADRANT4``: The fourth quadrant of the kit


    ariac_msgs/msg/CompetitionState
      .. code-block:: text
        
        uint8 IDLE=0   
        uint8 READY=1  
        uint8 STARTED=2 
        uint8 ORDER_ANNOUNCEMENTS_DONE=3 
        uint8 ENDED=4 

        uint8 competition_state

      - ``competition_state``: The current state of the competition.  One of the following values:

        - ``IDLE``: The competition is idle
        - ``READY``: The competition is ready to start
        - ``STARTED``: The competition has started
        - ``ORDER_ANNOUNCEMENTS_DONE``: The competition has started and all orders have been announced
        - ``ENDED``: The competition has ended

    ariac_msgs/msg/BinParts
      .. code-block:: text
        
        ariac_msgs/BinInfo[] bins

      - ``bins``: List of bins and their contents

      .. seealso:: :term:`ariac_msgs/msg/BinInfo`

    ariac_msgs/msg/BinInfo
      .. code-block:: text

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

      - ``bin_number``: The bin number.  One of the following values:
        
          - ``BIN1``: The first bin
          - ``BIN2``: The second bin
          - ``BIN3``: The third bin
          - ``BIN4``: The fourth bin
          - ``BIN5``: The fifth bin
          - ``BIN6``: The sixth bin
          - ``BIN7``: The seventh bin
          - ``BIN8``: The eighth bin
      - ``parts``: The parts in the bin

      .. seealso:: :term:`ariac_msgs/msg/PartLot`

    ariac_msgs/msg/PartLot
      .. code-block:: text

        ariac_msgs/Part part
        uint8 quantity

      - ``part``: The part
      - ``quantity``: The quantity of the part

      .. seealso:: :term:`ariac_msgs/msg/Part`

    ariac_msgs/msg/ConveyorParts
      .. code-block:: text
        
        ariac_msgs/PartLot[] parts

      - ``parts``: The parts on the conveyor

      .. seealso:: :term:`ariac_msgs/msg/PartLot`

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

      - ``location``: The location of the AGV.  One of the following values:
        
          - ``KITTING``: The AGV is at the kitting station
          - ``ASSEMBLY_FRONT``: The AGV is at the front assembly station (``AS1`` or ``AS3`` )
          - ``ASSEMBLY_BACK``: The AGV is at the back assembly station (``AS2`` or ``AS4`` )
          - ``WAREHOUSE``: The AGV is at the warehouse
          - ``UNKNOWN``: The AGV is at an unknown location

      - ``position``: The current position of the AGV in the workcell
      - ``velocity``: The current velocity of the AGV

    ariac_msgs/msg/VacuumGripperState
      .. code-block:: text

        bool enabled 
        bool attached 
        string type 

      - ``enabled``: Is the suction enabled?
      - ``attached``: Is an object attached to the gripper?
      - ``type``: The type of the gripper

    ariac_msgs/msg/ConveyorBeltState
      .. code-block:: text

        float64 power
        bool enabled  

      - ``power``: The power of the conveyor belt
      - ``enabled``: Is the conveyor belt enabled?

    ariac_msgs/msg/Robots
      .. code-block:: text

        bool floor_robot
        bool ceiling_robot

      - ``floor_robot``: Is the floor robot enabled?
      - ``ceiling_robot``: Is the ceiling robot enabled?

    ariac_msgs/msg/Sensors
      .. code-block:: text

        bool break_beam
        bool proximity
        bool laser_profiler
        bool lidar
        bool camera
        bool logical_camera

      - ``break_beam``: Is the break beam sensor type enabled?
      - ``proximity``: Is the proximity sensor type enabled?
      - ``laser_profiler``: Is the laser profiler type enabled?
      - ``lidar``: Is the lidar type enabled?
      - ``camera``: Is the camera type enabled?
      - ``logical_camera``: Is the logical camera type enabled?

    ariac_msgs/msg/HumanState
      .. code-block:: text

        geometry_msgs/Point human_position
        geometry_msgs/Point robot_position
        geometry_msgs/Vector3 human_velocity
        geometry_msgs/Vector3 robot_velocity

      - ``human_position``: The position of the human in the workcell
      - ``robot_position``: The position of the ceiling robot in the workcell
      - ``human_velocity``: The velocity of the human in the workcell
      - ``robot_velocity``: The velocity of the ceiling robot in the workcell

      .. seealso:: 
        
        - `geometry_msgs/msg/Point <https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html>`_
        - `geometry_msgs/msg/Vector3 <https://docs.ros2.org/latest/api/geometry_msgs/msg/Vector3.html>`_

    ariac_msgs/msg/Part
      .. code-block:: text
        
        uint8 RED=0
        uint8 GREEN=1
        uint8 BLUE=2
        uint8 ORANGE=3
        uint8 PURPLE=4

        uint8 BATTERY=10
        uint8 PUMP=11
        uint8 SENSOR=12
        uint8 REGULATOR=13

        uint8 color
        uint8 type

      - ``color``: The color of the part.  One of the following values:
        
          - ``RED``: The part is red
          - ``GREEN``: The part is green
          - ``BLUE``: The part is blue
          - ``ORANGE``: The part is orange
          - ``PURPLE``: The part is purple
      - ``type``: The type of the part.  One of the following values:
        
          - ``BATTERY``: The part is a battery
          - ``PUMP``: The part is a pump
          - ``SENSOR``: The part is a sensor
          - ``REGULATOR``: The part is a regulator


    ariac_msgs/msg/PartPose
      .. code-block:: text
        
        ariac_msgs/Part part
        geometry_msgs/Pose pose

      - ``part``: The part
      - ``pose``: The pose of the part

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/Part`
        - `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/AdvancedLogicalCameraImage
      .. code-block:: text
        
        ariac_msgs/PartPose[] part_poses
        ariac_msgs/KitTrayPose[] tray_poses
        geometry_msgs/Pose sensor_pose

      - ``part_poses``: The parts in the camera's field of view
      - ``tray_poses``: The kit trays in the camera's field of view
      - ``sensor_pose``: The pose of the camera in the world frame

      .. seealso:: 
        
        - :term:`ariac_msgs/msg/PartPose`
        - :term:`ariac_msgs/msg/KitTrayPose`
        - `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/KitTrayPose
      .. code-block:: text
        
        int8 id
        geometry_msgs/Pose pose

      - ``id``: The ID of the kit tray
      - ``pose``: The pose of the kit tray

      .. seealso:: `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/BreakBeamStatus
      .. code-block:: text
        
        std_msgs/Header header
        bool object_detected

      - ``header``: The header of the message
      - ``object_detected``: Is an object detected?

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

      .. seealso:: `sensor_msgs/Range <https://docs.ros2.org/latest/api/sensor_msgs/msg/Range.html>`_

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

      .. seealso:: `sensor_msgs/LaserScan <https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html>`_

    sensor_msgs/msg/PointCloud
      .. code-block:: text
        
        std_msgs/msg/Header header
        geometry_msgs/msg/Point32[] points
        sensor_msgs/msg/ChannelFloat32[] channels

      .. seealso:: `sensor_msgs/PointCloud <https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html>`_

    sensor_msgs/msg/Image
      .. code-block:: text
        
        std_msgs/msg/Header header
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data

      .. seealso:: `sensor_msgs/Image <https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html>`_

    ariac_msgs/msg/BasicLogicalCameraImage
      .. code-block:: text
        
        geometry_msgs/Pose[] part_poses
        geometry_msgs/Pose[] tray_poses
        geometry_msgs/Pose sensor_pose

      - ``part_poses``: The poses of the parts in the camera's field of view
      - ``tray_poses``: The poses of the kit trays in the camera's field of view
      - ``sensor_pose``: The pose of the camera in the world frame

      .. seealso:: `geometry_msgs/Pose <https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html>`_

    ariac_msgs/msg/QualityIssue
      .. code-block:: text
        
        bool all_passed
        bool missing_part
        bool flipped_part
        bool faulty_part
        bool incorrect_part_type
        bool incorrect_part_color

      - ``all_passed``: True if all parts passed the quality check, False otherwise
      - ``missing_part``: True if a part is missing, False otherwise
      - ``flipped_part``: True if a part is flipped, False otherwise
      - ``faulty_part``: True if a part is faulty, False otherwise
      - ``incorrect_part_type``: True if a part has the wrong type, False otherwise
      - ``incorrect_part_color``: True if a part has the wrong color, False otherwise


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

    ariac_msgs/srv/PerformQualityCheck
      .. code-block:: text

        string order_id
        ---
        bool valid_id
        bool all_passed
        bool incorrect_tray
        ariac_msgs/QualityIssue quadrant1
        ariac_msgs/QualityIssue quadrant2
        ariac_msgs/QualityIssue quadrant3
        ariac_msgs/QualityIssue quadrant4

      - ``order_id``: The ID of the order to be submitted
      - ``valid_id``: True if the order ID is valid, False otherwise
      - ``all_passed``: True if all parts in the order passed the quality check, False otherwise
      - ``incorrect_tray``: True if the detected tray does not have the correct ID for the order, False otherwise
      - ``quadrant1``: The quality issue for the first quadrant
      - ``quadrant2``: The quality issue for the second quadrant
      - ``quadrant3``: The quality issue for the third quadrant
      - ``quadrant4``: The quality issue for the fourth quadrant

      .. seealso:: :term:`ariac_msgs/msg/QualityIssue`

    ariac_msgs/srv/GetPreAssemblyPoses
      .. code-block:: text

        string order_id
        ---
        bool valid_id
        bool agv_at_station
        ariac_msgs/PartPose[] parts

      - ``order_id``: The ID of the order to be submitted
      - ``valid_id``: True if the order ID is valid, False otherwise
      - ``agv_at_station``: True if the AGV is at the station, False otherwise
      - ``parts``: The list of parts to be assembled

      .. seealso:: :term:`ariac_msgs/msg/PartPose`

    ariac_msgs/srv/MoveAGV
      .. code-block:: text

        int8 KITTING=0
        int8 ASSEMBLY_FRONT=1
        int8 ASSEMBLY_BACK=2 
        int8 WAREHOUSE=3 

        int8 location
        ---
        bool success
        string message

      - ``location``: The location to move the AGV to. One of the following values:

        - ``KITTING``: Kitting station
        - ``ASSEMBLY_FRONT``: Assembly station front (``AS1`` or ``AS3`` depending on the AGV ID)
        - ``ASSEMBLY_BACK``: Assembly station back  (``AS2`` or ``AS4`` depending on the AGV ID)
        - ``WAREHOUSE``: Warehouse
      - ``success``: True if the AGV was moved successfully, False otherwise
      - ``message``: A message describing the result of the service call

    ariac_msgs/srv/VacuumGripperControl
      .. code-block:: text

        bool enable
        ---
        bool success

      - ``enable``: True to enable the vacuum gripper, False to disable it
      - ``success``: True if the vacuum gripper was enabled/disabled successfully, False otherwise

    ariac_msgs/srv/ChangeGripper
      .. code-block:: text

        uint8 PART_GRIPPER=1
        uint8 TRAY_GRIPPER=2

        uint8 gripper_type

        ---
        bool success
        string message

      - ``gripper_type``: The type of gripper to change to. One of the following values:

        - ``PART_GRIPPER``: Part gripper
        - ``TRAY_GRIPPER``: Tray gripper
      - ``success``: True if the gripper was changed successfully, False otherwise
      - ``message``: A message describing the result of the service call

