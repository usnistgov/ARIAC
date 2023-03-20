
.. _TUTORIAL_2:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash

=========================================================
Tutorial 2: Read Data from a Break Beam Sensor
=========================================================

.. admonition:: Source Code for Tutorial 2
  :class: attention
  :name: tutorial_2
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_2 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_2>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_2


This tutorial covers the following steps:

  - Create a configuration file for sensors and cameras, 
  - Read data published by the sensor and log the outputs.

The final state of the package :inline-file:`ariac_tutorials` for :inline-tutorial:`tutorial 2` is as follows:

.. code-block:: text
    :emphasize-lines: 4,5,11
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    │   └── sensors.yaml
    ├── ariac_tutorials
    │   ├── __init__.py
    │   └── competition_interface.py
    └── nodes
        ├── tutorial_1.py
        └── tutorial_2.py


Sensor Configuration File
-----------------------------------
A sensor configuration file for a given package must be created in the folder :inline-file:`config` in the package. The file must be added to the ``CMakeLists.txt`` file in the package to allow the competition software to find the file.
To learn more about sensor configuration files, see `https://ariac.readthedocs.io/en/latest/competition/trials.html#sensor-configuration-file  <https://ariac.readthedocs.io/en/latest/competition/trials.html#sensor-configuration-file>`_.

Add a Break Beam Sensor
^^^^^^^^^^^^^^^^^^^^^^^^

Add a break beam sensor to  :inline-file:`sensors.yaml` as seen in :numref:`sensors-yaml-break-beam`. 

.. code-block:: yaml
    :caption: Add a break beam sensor in sensors.yaml
    :name: sensors-yaml-break-beam

    
    sensors:
      breakbeam_0:
        type: break_beam
        visualize_fov: true
        pose:
          xyz: [-0.36, 3.5, 0.88]
          rpy: [0, 0, pi]




Overview of CMakelists.txt
--------------------------------------------

To allow for the competition software to be able to find the sensor configuration, it must be added to the share directory of the package. 
To do this, add the following lines to the :inline-file:`CMakeLists.txt` file in the :inline-file:`ariac_tutorials` package.

.. code-block:: cmake
    :emphasize-lines: 15-18, 26

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

    # Install the config directory to the package share directory
    install(DIRECTORY 
    config
    DESTINATION share/${PROJECT_NAME}
    )

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
    nodes/tutorial_1.py
    nodes/tutorial_2.py
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()


Test the Sensor Configuration
--------------------------------------------

To test  the sensor was correctly added to the environment, run the following commands:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorials competitor_pkg:=ariac_tutorials


.. admonition:: Attention
  :class: warning
  
  By default, the command :inline-bash:`ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorials competitor_pkg:=ariac_tutorials` uses the sensor configuration file :inline-file:`ariac_tutorials/config/sensors.yaml`. If you want to use a different sensor configuration file, you need to use the argument :inline-bash:`sensor_config:={name_of_sensor_config}`. For example, to use the sensor configuration file :inline-file:`sensors_test.yaml`, you would use the command :inline-bash:`ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorials competitor_pkg:=ariac_tutorials sensor_config:=sensors_test`.



You should see a break beam sensor on the right side of the conveyor belt, as shown in the figure.


.. figure:: ../images/tutorial_2_image1.png
    :align: center
    :alt: Break beam sensor in Gazebo




Overview of the Competition Interface
--------------------------------------------

The competition interface for :inline-tutorial:`tutorial 2` is shown in :numref:`competitioninterface-tutorial2`.

.. code-block:: python
    :caption: competition_interface.py
    :name: competitioninterface-tutorial2
    :linenos:

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter import Parameter

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
        BreakBeamStatus as BreakBeamStatusMsg,
    )

    from std_srvs.srv import Trigger


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

        @property
        def conveyor_part_count(self):
            '''Number of parts that crossed the beam.'''
            return self._conveyor_part_count

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



        



Code Explained
^^^^^^^^^^^^^^^^^^^^^^^
- Imports:

    - :inline-python:`ariac_msgs.msg`: The ROS2 message API for the ARIAC messages.

        - :inline-python:`BreakBeamStatus`: ROS message for the break beam status, used to subscribe to the break beam status topic. The message is defined in  `ariac_msgs/msg/BreakBeamStatus.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/BreakBeamStatus.msg>`_ ).
    - :inline-python:`from rclpy.qos import qos_profile_sensor_data`: ROS 2 Quality of Service API. This is used to set the QoS profile for the floor robot gripper state subscriber.

- Intance Variables

    - :inline-python:`_break_beam0_sub`: Subscriber to the break beam status topic. 
    - :inline-python:`_conveyor_part_count`: Variable to store the number of parts on the conveyor belt that cross the beam.
    - :inline-python:`_object_detected`: Variable to store whether the beam is broken.

- Instance Methods

    - :inline-python:`_breakbeam0_cb()`: Callback function for the break beam status topic. It increments the variable :inline-python:`_part_count` if the beam is broken and the variable :inline-python:`_object_detected` is :inline-python:`False`. It also sets the variable :inline-python:`_object_detected` to :inline-python:`True` if the beam is broken.
    

Overview of the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_2.py
    
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
                interface.get_logger().info(f'Part Count: {interface.conveyor_part_count}', throttle_duration_sec=2.0)
            except KeyboardInterrupt:
                break

        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

 This executable creates an instance of the interface, starts the competition and logs the :inline-python:`conveyor_part_count` variable every 2 seconds. 


Run the Executable
--------------------------------

- In *terminal 1*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_2.py


    The node will wait until the competition is ready. In a second terminal, run the following:


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials


    Once the environment is loaded and the competition state is ready, the interface node running in *terminal 2* will start the competition and the sensor will start publishing data. In *terminal 1* you should see the the part count output increasing as parts on the conveyor break the sensor beam, as shown in the figure below.

    .. figure:: ../images/tutorial_2_image2.png
        :align: center


Outputs
--------------------------------


.. code-block:: console
    :class: no-copybutton
    :caption: Outputs:  *terminal 1*
    
    [INFO] [1679030246.597452729] [competition_interface]: Part Count: 0
    [INFO] [1679030248.597506278] [competition_interface]: Part Count: 0
    [INFO] [1679030250.598559700] [competition_interface]: Part Count: 0
    [INFO] [1679030252.599054150] [competition_interface]: Part Count: 0
    [INFO] [1679030254.600060902] [competition_interface]: Part Count: 0
    [INFO] [1679030256.600613831] [competition_interface]: Part Count: 0
    [INFO] [1679030258.601208258] [competition_interface]: Part Count: 0
    [INFO] [1679030260.602070416] [competition_interface]: Part Count: 1
    [INFO] [1679030262.602922331] [competition_interface]: Part Count: 1
    [INFO] [1679030264.603971647] [competition_interface]: Part Count: 1
    [INFO] [1679030266.604177567] [competition_interface]: Part Count: 2
    [INFO] [1679030268.605299171] [competition_interface]: Part Count: 2
    [INFO] [1679030270.605708942] [competition_interface]: Part Count: 3
    [INFO] [1679030272.606264426] [competition_interface]: Part Count: 3
    [INFO] [1679030274.606734362] [competition_interface]: Part Count: 3
    [INFO] [1679030276.607208635] [competition_interface]: Part Count: 4
    [INFO] [1679030278.608460268] [competition_interface]: Part Count: 4
    [INFO] [1679030280.608596068] [competition_interface]: Part Count: 4
