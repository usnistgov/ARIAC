
.. _TUTORIAL_2:

=========================================================
Tutorial 2: Reading Data from a Break Beam Sensor
=========================================================

.. note::
  **Prerequisites**: Tutorial 1 should be completed before starting this tutorial.


This tutorial covers the following steps:

  - Create a configuration file for sensors and cameras, 
  - Add functionality to the competition interface to read data from a sensor and output that data to the terminal.

The package ``competition_tutorials`` is expected to have the following structure for tutorial 2:

.. code-block:: bash
    
    competition_tutorials
    ├── CMakeLists.txt                  (updated)
    ├── package.xml
    ├── config
    │   ├── sensors.yaml                (new)
    ├── competition_tutorials
    │   ├── __init__.py
    │   └── competition_interface.py    (new)
    └── src
        ├── start_competition.py        (from tutorial 1)
        └── read_break_beam_sensor.py   (new)


Create a Sensor Configuration File
-----------------------------------

Create a `sensor configuration file  <https://ariac.readthedocs.io/en/latest/competition/trials.html#sensor-configuration-file>`_ in the package ``competition_tutorials``.

.. code-block:: bash

  cd ~/ariac_ws/src/competition_tutorials
  mkdir config
  touch config/sensors.yaml


Add a break beam sensor to  ``sensors.yaml`` as seen in :numref:`sensors-yaml-break-beam`. 

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




Verify the Sensor is Added to the Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To allow for the competition software to be able to find the sensor configuration it must be added to the share directory of the package. To do this, add the following lines to the ``CMakeLists.txt`` file in the ``competition_tutorials`` package.

.. code-block:: cmake

    install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
    )


To test  the camera was correctly added to the environment, run the following commands:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see a break beam sensor on the right side of the conveyor belt, as shown in the figure below.

.. _fig-break-beam-sensor:
.. figure:: ../images/tutorial_2_image1.png
   :align: center

    


Competition Interface
--------------------------------


The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial2`.  Contents specific to this tutorial are highlighted in yellow.

.. code-block:: python
    :caption: Competition interface for tutorial 2
    :name: competitioninterface-tutorial2
    :emphasize-lines: 8, 57-61, 63, 65, 70-72, 74-83
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
                self.competition_state_cb,
                10)
            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None
            
            # Subscriber to the break beam status topic
            self._break_beam0_sub = self.create_subscription(
                BreakBeamStatusMsg,
                '/ariac/sensors/breakbeam_0/status',
                self.breakbeam0_cb,
                qos_profile_sensor_data)
            # Store the number of parts that crossed the beam
            self._part_count = 0
            # Store whether the beam is broken
            self._object_detected = False
            
            

        @property
        def part_count(self):
            '''Number of parts that crossed the beam.'''
            return self._part_count
        
        def breakbeam0_cb(self, msg: BreakBeamStatusMsg):
            '''Callback for the topic /ariac/sensors/breakbeam_0/status

            Arguments:
                msg -- BreakBeamStatusMsg message
            '''
            if not self._object_detected and msg.object_detected:
                self._part_count += 1

            self._object_detected = msg.object_detected

        def competition_state_cb(self, msg: CompetitionStateMsg):
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

- Imports

    - ``from rclpy.qos import qos_profile_sensor_data``: ROS 2 Quality of Service API. This is used to set the QoS profile for the floor robot gripper state subscriber.
    - ``from ariac_msgs.msg import BreakBeamStatus as BreakBeamStatusMsg``: ROS message for the break beam status, used to subscribe to the break beam status topic.
  
- Class Variables

    - ``_break_beam0_sub``: Subscriber to the break beam status topic. 
    - ``_part_count``: Variable to store the number of parts that crossed the beam.
    - ``_object_detected``: Variable to store whether the beam is broken.

- Class Methods

    - ``breakbeam0_cb()``: Callback function for the break beam status topic. It increments the variable ``_part_count`` if the beam is broken and the variable ``_object_detected`` is ``False``. It also sets the variable ``_object_detected`` to ``True`` if the beam is broken.
    

Create the Executable
--------------------------------

To test this tutorial, create a new file ``read_break_beam_sensor.py`` in ``competition_tutorials/src``:

.. code-block:: bash

    cd ~/ariac_ws/src/competition_tutorials/src
    touch read_break_beam_sensor.py
    chmod +x read_break_beam_sensor.py


Copy the following code in the file ``read_break_beam_sensor.py``:


.. code-block:: python
    :caption: read_break_beam_sensor.py
    
    #!/usr/bin/env python3

    import rclpy
    from competition_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        while rclpy.ok():
            try:
                rclpy.spin_once(interface)
                interface.get_logger().info(f'Part Count: {interface.part_count}', throttle_duration_sec=2.0)
            except KeyboardInterrupt:
                break

        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

 This executable creates an instance of the interface, starts the competition and logs the ``part_count`` variable every 2 seconds. 

Update CMakelists.txt
^^^^^^^^^^^^^^^^^^^^^^

Update ``CMakeLists.txt`` to add ``read_break_beam_sensor.py`` as an executable.

.. code-block:: cmake

  # Install Python executables
  install(PROGRAMS
    src/start_competition.py
    src/read_break_beam_sensor.py
    DESTINATION lib/${PROJECT_NAME}
  )


Run the Executable
--------------------------------

Next, build the package and run the executable.


.. code-block:: bash
    :caption: Terminal 1

    cd ~/ariac_ws
    colcon build
    . install/setup.bash
    ros2 run competition_tutorials read_break_beam_sensor.py


The node will wait until the competition is ready. In a second terminal, run the following:

.. code-block:: bash
    :caption: Terminal 2

    cd ~/ariac_ws
    . install/setup.bash
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial


Once the environment is loaded and the competition state is ready, the interface node running in Terminal 2 will start the competition and the sensor will start publishing data. You should see the the part count output increasing as parts on the conveyor break the sensor beam, as shown in the figure below.

.. figure:: ../images/tutorial_2_image2.png
   :align: center


Outputs
--------------------------------


.. code-block:: text
    :caption: Terminal outputs
    
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
