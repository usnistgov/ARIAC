
.. _TUTORIAL_1:

=========================================================
Tutorial 1: Creating a Competition Package
=========================================================

.. admonition:: Source Code for Tutorial 1
  :class: tip
  :name: tutorial_1
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_1






This tutorial details the steps necessary to create a competition package that is able to interface with the ARIAC competition. 
This competition package will use a python node to listen to the competition state and call a ROS service to start the competition when ready.

The structure of the package ``ariac_tutorials`` for **tutorial 1**  is as follows:

.. code-block:: text
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── ariac_tutorials
    │   ├── __init__.py
    │   └── competition_interface.py
    └── src
        └── tutorial_1.py

To create the package: 

    - Navigate to ``~/ariac_ws/src``, which was created from `the installation directions <https://ariac.readthedocs.io/en/latest/getting_started/installation.html>`_. 
    - Run the package creation command.

.. code-block:: bash
    
    cd ~/ariac_ws/src
    ros2 pkg create competition_tutorials --build-type ament_cmake


Update CMakelists.txt
--------------------------------

Update ``CMakeLists.txt`` as follows:

.. code-block:: cmake
    
    cmake_minimum_required(VERSION 3.8)
    project(competition_tutorials)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(ament_cmake_python REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(ariac_msgs REQUIRED)

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
    src/start_competition.py
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()


Update package.xml
--------------------------------

In ``package.xml``, update the maintainer and license information. Make sure to add the following dependencies:

.. code-block:: xml
    
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>competition_tutorials</name>
    <version>0.0.0</version>
    <description>ROS2 pkg for ARIAC tutorials</description>
    <maintainer email="{your_email}">{your name}</maintainer>
    <license>Apache License 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>rclpy</depend>
    <depend>ariac_msgs</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>


Create Python Package
--------------------------------

Create a python package with the same name as the ros2 package. This python package will include all the python source code for your software. 

.. code-block:: bash
    
    cd ~/ariac_ws/src/competition_tutorials
    mkdir competition_tutorials
    touch competition_tutorials/__init__.py
    touch competition_tutorials/competition_interface.py


Competition Interface
----------------------

The competition interface used in this tutorial is shown in :numref:`competitioninterface-tutorial1`.


.. code-block:: python
    :caption: Competition interface for tutorial 1
    :name: competitioninterface-tutorial1
    
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from ariac_msgs.msg import (
        CompetitionState as CompetitionStateMsg,
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
            # Subscriber to the logical camera topic

        def competition_state_cb(self, msg: CompetitionStateMsg):
            '''Callback for the topic /ariac/competition_state

            Arguments:
                msg -- CompetitionState message
            '''
            # Log if competition state has changed
            if self._competition_state != msg.competition_state:
                self.get_logger().info(
                    f'Competition state is: \
                    {CompetitionInterface._competition_states[msg.competition_state]}',
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


The class ``CompetitionInterface`` consists of the following:

    - ``_competition_states``: A dictionary for converting CompetitionState constants to strings for logging purposes.
    - ``__init__()``: The constructor for the class. 

        - ``_start_competition_client`` is a client for the service ``/ariac/start_competition``.
        - ``_competition_state_sub`` is a subscriber for the topic ``/ariac/competition_state``.
        - ``_competition_state`` is a variable to store the state of the competition.
    - ``competition_state_cb()``: Callback for the topic ``/ariac/competition_state``. This method stores the competition state in the variable ``_competition_state``.
    - ``start_competition()``: Method to start the competition. This method waits for the competition to be ready by checking the value of ``_competition_state`` and then calls the service ``/ariac/start_competition`` through the client ``_start_competition_client``.



Create the Executable
--------------------------------

To test this tutorial, create a new file ``start_competition.py`` in ``competition_tutorials/src``:

.. code-block:: bash

    cd ~/ariac_ws/src/competition_tutorials/src
    touch start_competition.py
    chmod +x start_competition.py


Copy the following code in the file ``start_competition.py``:


.. code-block:: python
    :caption: start_competition.py
    
    #!/usr/bin/env python3

    import rclpy
    from competition_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()
        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()


This executable creates an instance of the ``CompetitionInterface`` class from ``competition_tutorials/competition_interface.py`` and calls the ``start_competition`` method.



Run the Executable
--------------------------------

Next, build the package and run the executable:


.. code-block:: bash
    :caption: Terminal 1

    cd ~/ariac_ws
    colcon build
    . install/setup.bash
    ros2 run competition_tutorials start_competition.py

You should see this output:

.. code-block::
    
    [INFO] [1679025057.998334513] [competition_interface]: Waiting for competition to be ready


The node will wait until the competition is ready. Do the following in a new terminal:

.. code-block:: bash
    :caption: Terminal 2

    cd ~/ariac_ws
    . install/setup.bash
    ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial


This should start gazebo. Once the environment is loaded and the competition state is ready, the interface node running in Terminal 1 will start the competition. This will activate all sensors, enable the robot controllers, and start the conveyor belt. 


Outputs
--------------------------------

.. code-block:: console
    
    [INFO] [1679025057.998334513] [competition_interface]: Waiting for competition to be ready
    [INFO] [1679025079.463133489] [competition_interface]: Competition state is: idle
    [INFO] [1679025085.587755650] [competition_interface]: Competition state is: ready
    [INFO] [1679025085.588245939] [competition_interface]: Competition is ready. Starting...
    [INFO] [1679025085.590775613] [competition_interface]: Started competition.



