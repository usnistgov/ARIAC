
.. _TUTORIAL_3:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash

=========================================================
Tutorial 3: Read Data from an Advanced Logical Camera
=========================================================

.. admonition:: Tutorial 3
  :class: attention
  :name: tutorial_3

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>`
  - **Source Code**: `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_3 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_3>`_ 
  - **Switch Branch**:

    .. code-block:: bash
        
            cd ~/ariac_ws/ariac_tutorials
            git checkout tutorial_3

This tutorial covers the following topics:

  - Add a camera to the environment,
  - Receive messages from the camera, 
  - Store camera data internally as an instance of a class,
  - Display the stored data on the standard output.


Package Structure
--------------------------------------------

Updates and additions that are specific to :inline-tutorial:`tutorial 3`  are highlighted in the tree below.


.. code-block:: text
    :emphasize-lines: 2, 5, 8, 9, 13
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    │   └── sensors.yaml
    ├── ariac_tutorials
    │   ├── __init__.py
    │   ├── utils.py
    │   └── competition_interface.py
    └── nodes
        ├── tutorial_1.py
        ├── tutorial_2.py
        └── tutorial_3.py


CMakelists.txt
--------------------------------------------

.. code-block:: cmake
    :emphasize-lines: 28

    cmake_minimum_required(VERSION 3.8)
    project(ariac_tutorials)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(ariac_msgs REQUIRED)
    find_package(orocos_kdl REQUIRED)

    # Install the config directory to the package share directory
    install(DIRECTORY 
    config
    DESTINATION share/${PROJECT_NAME}
    )

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
    scripts/tutorial_1.py
    scripts/tutorial_2.py
    scripts/tutorial_3.py
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()



Sensor Configuration File
-----------------------------------

Reuse the same sensor configuration file created in :ref:`Tutorial 2 <TUTORIAL2>`.

Add a Logical Camera
^^^^^^^^^^^^^^^^^^^^^^^^

Add an advanced logical camera to  :inline-file:`sensors.yaml` (see lines 8-13 in :numref:`sensors-camera-yaml`). 

.. code-block:: yaml
    :caption: sensors.yaml
    :name: sensors-camera-yaml
    :emphasize-lines: 8, 9, 10, 11, 12, 13
    :linenos:
    
    sensors:
      breakbeam_0:
        type: break_beam
        visualize_fov: true
        pose:
          xyz: [-0.36, 3.5, 0.88]
          rpy: [0, 0, pi]
      advanced_camera_0:
        type: advanced_logical_camera
        visualize_fov: true
        pose:
          xyz: [-2.286, 2.96, 1.8]
          rpy: [pi, pi/2, 0]




Test the Sensor Configuration
--------------------------------------------

To test  the camera was correctly added to the environment, run the following commands:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorials competitor_pkg:=ariac_tutorials


You should see the camera above bins 1-4 as shown in the figure below.

.. figure:: ../images/tutorial3/advanced_camera_0.jpg
   :align: center

    


Competition Interface
--------------------------------

The competition interface for :inline-tutorial:`tutorial 3` is shown in :numref:`competitioninterface-tutorial3`.

.. code-block:: python
    :caption: competition_interface.py
    :name: competitioninterface-tutorial3
    :linenos:

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Pose
    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        BreakBeamStatus as BreakBeamStatusMsg,
        AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
        Part as PartMsg,
        PartPose as PartPoseMsg
    )

    from std_srvs.srv import Trigger

    from ariac_tutorials.utils import (
        multiply_pose,
        AdvancedLogicalCameraImage
    )


    class CompetitionInterface(Node):
        '''
        Class for a competition interface node.

        Args:
            Node (rclpy.node.Node): Parent class for ROS nodes

        Raises:
            KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
        '''
        _competition_states = {
            CompetitionStateMsg.IDLE: 'idle',
            CompetitionStateMsg.READY: 'ready',
            CompetitionStateMsg.STARTED: 'started',
            CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
            CompetitionStateMsg.ENDED: 'ended',
        }
        '''Dictionary for converting CompetitionState constants to strings'''
        
        _part_colors = {
            PartMsg.RED: 'red',
            PartMsg.BLUE: 'blue',
            PartMsg.GREEN: 'green',
            PartMsg.ORANGE: 'orange',
            PartMsg.PURPLE: 'purple',
        }
        '''Dictionary for converting Part color constants to strings'''

        _part_colors_emoji = {
            PartMsg.RED: '🟥',
            PartMsg.BLUE: '🟦',
            PartMsg.GREEN: '🟩',
            PartMsg.ORANGE: '🟧',
            PartMsg.PURPLE: '🟪',
        }
        '''Dictionary for converting Part color constants to emojis'''

        _part_types = {
            PartMsg.BATTERY: 'battery',
            PartMsg.PUMP: 'pump',
            PartMsg.REGULATOR: 'regulator',
            PartMsg.SENSOR: 'sensor',
        }
        '''Dictionary for converting Part type constants to strings'''
        

        def __init__(self):
            super().__init__('competition_interface')

            sim_time = Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )

            self.set_parameters([sim_time])

            # Service client for starting the competition
            self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

            # Subscriber to the competition state topic
            self._competition_state_sub = self.create_subscription(
                CompetitionStateMsg,
                '/ariac/competition_state',
                self._competition_state_cb,
                10)
            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None

            # Subscriber to the break beam status topic
            self._break_beam0_sub = self.create_subscription(
                BreakBeamStatusMsg,
                '/ariac/sensors/breakbeam_0/status',
                self._breakbeam0_cb,
                qos_profile_sensor_data)
            # Store the number of parts that crossed the beam
            self._conveyor_part_count = 0
            # Store whether the beam is broken
            self._object_detected = False
            
            # Subscriber to the logical camera topic
            self._advanced_camera0_sub = self.create_subscription(
                AdvancedLogicalCameraImageMsg,
                '/ariac/sensors/advanced_camera_0/image',
                self._advanced_camera0_cb,
                qos_profile_sensor_data)
            # Store each camera image as an AdvancedLogicalCameraImage object
            self._camera_image: AdvancedLogicalCameraImage = None
            

        @property
        def camera_image(self):
            return self._camera_image

        @property
        def conveyor_part_count(self):
            return self._conveyor_part_count
    
        def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
            '''Callback for the topic /ariac/sensors/advanced_camera_0/image

            Arguments:
                msg -- AdvancedLogicalCameraImage message
            '''
            self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                            msg.tray_poses,
                                                            msg.sensor_pose)

        def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):
            '''Callback for the topic /ariac/sensors/breakbeam_0/status

            Arguments:
                msg -- BreakBeamStatusMsg message
            '''
            if not self._object_detected and msg.object_detected:
                self._conveyor_part_count += 1

            self._object_detected = msg.object_detected

        def _competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state

            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                self.get_logger().info(
                    f'Competition state is: {CompetitionInterface._competition_states[msg.competition_state]}',
                    throttle_duration_sec=1.0)
            self._competition_state = msg.competition_state

        def start_competition(self):
            '''Function to start the competition.
            '''
            self.get_logger().info('Waiting for competition to be ready')

            if self._competition_state == CompetitionStateMsg.STARTED:
                return
            # Wait for competition to be ready
            while self._competition_state != CompetitionStateMsg.READY:
                try:
                    rclpy.spin_once(self)
                except KeyboardInterrupt:
                    return

            self.get_logger().info('Competition is ready. Starting...')

            # Call ROS service to start competition
            while not self._start_competition_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for /ariac/start_competition to be available...')

            # Create trigger request and call starter service
            request = Trigger.Request()
            future = self._start_competition_client.call_async(request)

            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Started competition.')
            else:
                self.get_logger().info('Unable to start competition')
                
        

        def parse_advanced_camera_image(self):
            '''
            Parse an AdvancedLogicalCameraImage message and return a string representation.
            '''
            output = '\n\n==========================\n'

            sensor_pose: Pose = self._camera_image._sensor_pose

            part_pose: PartPoseMsg

            counter = 1
            for part_pose in self._camera_image._part_poses:
                part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
                part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
                part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
                output += f'Part {counter}: {part_color_emoji} {part_color} {part_type}\n'
                output += '==========================\n'
                output += 'Camera Frame\n'
                output += '==========================\n'
                position = f'x: {part_pose.pose.position.x}\n\t\ty: {part_pose.pose.position.y}\n\t\tz: {part_pose.pose.position.z}'
                orientation = f'x: {part_pose.pose.orientation.x}\n\t\ty: {part_pose.pose.orientation.y}\n\t\tz: {part_pose.pose.orientation.z}\n\t\tw: {part_pose.pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'
                output += 'World Frame\n'
                output += '==========================\n'
                part_world_pose = multiply_pose(sensor_pose, part_pose.pose)
                position = f'x: {part_world_pose.position.x}\n\t\ty: {part_world_pose.position.y}\n\t\tz: {part_world_pose.position.z}'
                orientation = f'x: {part_world_pose.orientation.x}\n\t\ty: {part_world_pose.orientation.y}\n\t\tz: {part_world_pose.orientation.z}\n\t\tw: {part_world_pose.orientation.w}'

                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += '==========================\n'

                counter += 1

            return output
    


Code Explanation
^^^^^^^^^^^^^^^^^

The competition interface from :ref:`Tutorial 2 <TUTORIAL2>` was augmented with the components described below.


- The :inline-python:`Pose` module is needed to compute and display the pose of the parts detected by the camera.

    .. code-block:: python
        :lineno-start: 5

        from geometry_msgs.msg import Pose

- :inline-python:`AdvancedLogicalCameraImage`: Message class that stores the part poses and sensor pose of the advanced logical camera (see `AdvancedLogicalCameraImage.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/AdvancedLogicalCameraImage.msg>`_ )
- :inline-python:`Part`: Message class that stores the part type and color (see `Part.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/Part.msg>`_ )
- :inline-python:`PartPose`: Message class that stores a :inline-python:`Part` and its :inline-python:`Pose`  (see `PartPose.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/PartPose.msg>`_)
- **Note**: These message classes are imported as aliases since the package consists of Python classes with the same name.

    .. code-block:: python
        :lineno-start: 6
        :emphasize-lines: 4-6

        from ariac_msgs.msg import (
            CompetitionState as CompetitionStateMsg,
            BreakBeamStatus as BreakBeamStatusMsg,
            AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
            Part as PartMsg,
            PartPose as PartPoseMsg
            )


- The module :inline-python:`utils` contains reusable functions and classes.
- The function :inline-python:`multiply_pose()` is used to compute the pose of the parts detected by the camera in the world frame.
- The class :inline-python:`AdvancedLogicalCameraImage` is a Python class which is used to store the message published on the camera topic. Although a class is not strictly necessary, it makes the code more readable and easier to maintain.

    .. code-block:: python
        :lineno-start: 16

        from ariac_tutorials.utils import (
            multiply_pose,
            AdvancedLogicalCameraImage
        )
  
- Class Variables

    - :inline-python:`_part_colors` and :inline-python:`_part_types` are dictionaries that map the integer values of the part color and type to their string representations. :inline-python:`_part_colors_emoji` is a dictionary that maps the integer values of the part color to their emoji representations. These dictionaries are mainly used to display the part color and type in a human-readable format.

- Instance Variables

    - :inline-python:`_camera_image` is an object of the class :inline-python:`AdvancedLogicalCameraImage` that stores the latest message published on the camera topic.

- Instance Methods

    - :inline-python:`advanced_camera0_cb()`: Callback for the camera topic which stores the latest published message in  :inline-python:`_camera_image`.
    - :inline-python:`parse_advanced_camera_image()` parses the message stored in :inline-python:`_camera_image` and returns a string representation of the message. This method is used to display the part color, type, and pose in a human-readable format. The output is printed in the following format:
    
        - Emoji for the part color using the class attribute :inline-python:`part_colors_emoji_`.
        - Part color using the class attribute :inline-python:`part_colors_`.
        - Part type using the class attribute :inline-python:`part_types_`.
        - Part pose in the camera frame: This is the pose returned by the camera.
        - Part pose in the world frame: This is calculated by multiplying the camera pose with the part pose in the camera frame. This multiplication is done using the method :inline-python:`multiply_pose()`.





Overview of the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_3.py
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface


    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        while rclpy.ok():
        try:
        rclpy.spin_once(interface)

        if interface.camera_image is not None:
            interface.get_logger().info(interface.parse_advanced_camera_image(), throttle_duration_sec=2.0)
        except KeyboardInterrupt:
        break

        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
    main()


Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

This executable does the following:

    - Creates an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Starts the competition.
    - Logs the content of :inline-python:`_camera_image` every 2 seconds.



Run the Executable
--------------------------------

- In *terminal 1*, run the following commands:


    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_3.py


    The node will wait until the competition is ready.


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials


    Once the environment is loaded and the competition state is ready, the interface node running in *terminal 1* will start the competition and the sensor will start publishing data.
    Each part detected by the camera will be logged to the terminal.

Outputs
--------------------------------


.. code-block:: console
    :caption: Terminal outputs
    :class: no-copybutton
    
    ==========================
    Part 1: 🟪 Purple Pump
    ==========================
    Camera Frame
    ==========================
        Position:
            x: 1.0772143770406752
            y: 0.5150000388121461
            z: -0.2060067933778063
        Orientation:
            x: -0.0006855918720226918
            y: -0.7063449441335629
            z: -0.0006911150034743035
            w: 0.7078671289308405
    ==========================
    World Frame
    ==========================
        Position:
            x: -2.0799998435394826
            y: 2.4450000325688257
            z: 0.7227843196083803
        Orientation:
            x: -0.0010731836296401
            y: -0.0009734789503818064
            z: 0.9999989503002881
            w: 3.7353182917545933e-06
    ==========================
    Part 2: 🟪 Purple Pump
    ==========================
    Camera Frame
    ==========================
        Position:
            x: 1.0774243270564583
            y: 0.15500079119043203
            z: -0.20600655688080022
        Orientation:
            x: 0.0003549575317311197
            y: -0.7072292680009703
            z: 0.00035219184924200627
            w: 0.7069840963196159
    ==========================
    World Frame
    ==========================
        Position:
            x: -2.080000206072213
            y: 2.8049992801904398
            z: 0.7225743696009308
        Orientation:
            x: 0.0001765258688373336
            y: 0.0005000301498669066
            z: 0.9999998594026379
            w: 1.7808459680221148e-06