
.. _TUTORIAL_1:


.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

=========================================================
Tutorial 1: Creating a Competition Package
=========================================================

.. admonition:: Source Code for Tutorial 1
  :class: attention
  :name: tutorial_1
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_1



This tutorial details the steps necessary to create a competition package that is able to interface with the ARIAC competition. 
This competition package will use a python node to listen to the competition state and call a ROS service to start the competition when ready.

The final state of the package :inline-file:`ariac_tutorials` for :inline-tutorial:`tutorial 1`  is as follows:

.. code-block:: text
    :class: no-copybutton
    :emphasize-lines: 8
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── ariac_tutorials
    │   ├── __init__.py
    │   └── competition_interface.py
    └── nodes
        └── tutorial_1.py


Overview of CMakelists.txt
--------------------------------

.. code-block:: cmake
    :emphasize-lines: 19
    
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

    # Install Python modules
    ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

    # Install Python executables
    install(PROGRAMS
    nodes/tutorial_1.py
    DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()


Overview of package.xml
--------------------------------

.. admonition:: package.xml
  :class: attention

  
  In ``package.xml``, update the maintainer and license information.

.. code-block:: xml
  :emphasize-lines: 7
    
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>ariac_tutorials</name>
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



Overview of the Competition Interface
--------------------------------------------

The competition interface for :inline-tutorial:`tutorial 1` is shown in :numref:`competitioninterface-tutorial1`.


.. code-block:: python
    :caption: competition_interface.py
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
                self._competition_state_cb,
                10)
            # Store the state of the competition
            self._competition_state: CompetitionStateMsg = None
            # Subscriber to the logical camera topic

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

        - :inline-python:`CompetitionState`: The competition state message (defined in  `ariac_msgs/msg/CompetitionState.msg <https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_msgs/msg/CompetitionState.msg>`_ ).
    - :inline-python:`std_srvs.srv`: The ROS2 service API for the standard services.

- Class Variables

    -  :inline-python:`self._competition_states`: A dictionary for converting CompetitionState constants to strings for logging purposes.

- Instance Variables

    - :inline-python:`self._start_competition_client` is a client for the service ``/ariac/start_competition``.
    - :inline-python:`self._competition_state_sub` is a subscriber for the topic ``/ariac/competition_state``.
    - :inline-python:`self._competition_state` is a variable to store the current competition state.

- Class Methods

    - :inline-python:`_competition_state_cb()`: Callback for the topic ``/ariac/competition_state``. This method stores the competition state in the variable :inline-python:`self._competition_state`.
    - :inline-python:`start_competition()`: Method to start the competition. This method waits for the competition to be ready by checking the value of :inline-python:`self._competition_state` and then calls the service ``/ariac/start_competition`` through the client :inline-python:`self._start_competition_client`.



Overview of the Executable
--------------------------------



.. code-block:: python
    :caption: tutorial_1.py
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface

    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()
        interface.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()


Code Explained
^^^^^^^^^^^^^^^^^^^^^^^


This executable does the following:

    - Creates an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Starts the competition.



Run the Executable
--------------------------------


- In *terminal 1*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_1.py


    You should see this output:

    .. code-block:: console
        
        [INFO] [1679025057.998334513] [competition_interface]: Waiting for competition to be ready


    The node waits until the competition is ready. 

- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials


    This should start gazebo. Once the environment is loaded and the competition state is ready, the interface node running in *terminal 1* will start the competition. This will activate all sensors, enable the robot controllers, and start the conveyor belt. Orders will also be announced.


Outputs
--------------------------------

.. code-block:: console
    :caption: terminal 1
    :class: no-copybutton
    
    [INFO] [1679025057.998334513] [competition_interface]: Waiting for competition to be ready
    [INFO] [1679025079.463133489] [competition_interface]: Competition state is: idle
    [INFO] [1679025085.587755650] [competition_interface]: Competition state is: ready
    [INFO] [1679025085.588245939] [competition_interface]: Competition is ready. Starting...
    [INFO] [1679025085.590775613] [competition_interface]: Started competition.

.. code-block:: console
    :caption: terminal 2
    :class: no-copybutton
    
    [gzserver-1] [INFO] [1679262580.420282809] [task_manager]: Announcing order
    [gzserver-1] [INFO] [1679262580.420560785] [task_manager]: 
    [gzserver-1] =================
    [gzserver-1] Announcing Order 2IZJP127
    [gzserver-1] =================
    [gzserver-1] Type: Assembly
    [gzserver-1] Priority: 0
    [gzserver-1]    Assembly Task
    [gzserver-1]    ================
    [gzserver-1]    AGV: [1,2]
    [gzserver-1]    Station: as1
    [gzserver-1]    ================
    [gzserver-1]    Products: 
    [gzserver-1]    ------
    [gzserver-1]    Part: [regulator,red]
    [gzserver-1]    Assembled Pose: [0.175,-0.223,0.215][0.5,-0.5,-0.5,0.5]
    [gzserver-1]    Assembled Direction: [0,0,-1]
    [gzserver-1]    ------
    [gzserver-1]    Part: [battery,red]
    [gzserver-1]    Assembled Pose: [-0.15,0.035,0.043][0,0,0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,1,0]
    [gzserver-1]    ------
    [gzserver-1]    Part: [pump,red]
    [gzserver-1]    Assembled Pose: [0.14,0,0.02][0,0,-0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,0,-1]
    [gzserver-1]    ------
    [gzserver-1]    Part: [sensor,red]
    [gzserver-1]    Assembled Pose: [-0.1,0.395,0.045][0,0,-0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,-1,0]
    [gzserver-1] 
    [gzserver-1] [INFO] [1679262580.421094684] [task_manager]: Announcing order
    [gzserver-1] [INFO] [1679262580.421236972] [task_manager]: 
    [gzserver-1] =================
    [gzserver-1] Announcing Order 2IZJP320
    [gzserver-1] =================
    [gzserver-1] Type: Combined
    [gzserver-1] Priority: 0
    [gzserver-1]    Combined Task
    [gzserver-1]    ================
    [gzserver-1]    Station: as3
    [gzserver-1]    ================
    [gzserver-1]    Products: 
    [gzserver-1]    ------
    [gzserver-1]    Part: [regulator,orange]
    [gzserver-1]    Assembled Pose: [0.175,-0.223,0.215][0.5,-0.5,-0.5,0.5]
    [gzserver-1]    Assembled Direction: [0,0,-1]
    [gzserver-1]    ------
    [gzserver-1]    Part: [battery,orange]
    [gzserver-1]    Assembled Pose: [-0.15,0.035,0.043][0,0,0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,1,0]
    [gzserver-1]    ------
    [gzserver-1]    Part: [pump,orange]
    [gzserver-1]    Assembled Pose: [0.14,0,0.02][0,0,-0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,0,-1]
    [gzserver-1]    ------
    [gzserver-1]    Part: [sensor,orange]
    [gzserver-1]    Assembled Pose: [-0.1,0.395,0.045][0,0,-0.707107,0.707107]
    [gzserver-1]    Assembled Direction: [0,-1,0]
    [gzserver-1] 
    [gzserver-1] [INFO] [1679262580.421869143] [task_manager]: Announcing order
    [gzserver-1] [INFO] [1679262580.421926369] [task_manager]: 
    [gzserver-1] =================
    [gzserver-1] Announcing Order MMB30H56
    [gzserver-1] =================
    [gzserver-1] Type: Kitting
    [gzserver-1] Priority: 0
    [gzserver-1]    Kitting Task
    [gzserver-1]    ================
    [gzserver-1]    AGV: 1
    [gzserver-1]    Tray ID: 3
    [gzserver-1]    Destination: warehouse
    [gzserver-1]    ================
    [gzserver-1]    Products: 
    [gzserver-1]    ------
    [gzserver-1]    Part: [battery,blue]
    [gzserver-1]    Quadrant: 3
    [gzserver-1]    ------
    [gzserver-1]    Part: [pump,purple]
    [gzserver-1]    Quadrant: 1
    [gzserver-1] 
    [gzserver-1] [INFO] [1679262585.260788577] [task_manager]: Announcing order
    [gzserver-1] [INFO] [1679262585.260829987] [task_manager]: 
    [gzserver-1] =================
    [gzserver-1] Announcing Order MMB30H57
    [gzserver-1] =================
    [gzserver-1] Type: Kitting
    [gzserver-1] Priority: 0
    [gzserver-1]    Kitting Task
    [gzserver-1]    ================
    [gzserver-1]    AGV: 2
    [gzserver-1]    Tray ID: 5
    [gzserver-1]    Destination: warehouse
    [gzserver-1]    ================
    [gzserver-1]    Products: 
    [gzserver-1]    ------
    [gzserver-1]    Part: [regulator,orange]
    [gzserver-1]    Quadrant: 2
    [gzserver-1] 
    [gzserver-1] [INFO] [1679262593.872668975] [task_manager]: Announcing order
    [gzserver-1] [INFO] [1679262593.872722245] [task_manager]: 
    [gzserver-1] =================
    [gzserver-1] Announcing Order MMB30H58
    [gzserver-1] =================
    [gzserver-1] Type: Kitting
    [gzserver-1] Priority: 0
    [gzserver-1]    Kitting Task
    [gzserver-1]    ================
    [gzserver-1]    AGV: 3
    [gzserver-1]    Tray ID: 8
    [gzserver-1]    Destination: warehouse
    [gzserver-1]    ================
    [gzserver-1]    Products: 
    [gzserver-1]    ------
    [gzserver-1]    Part: [sensor,green]
    [gzserver-1]    Quadrant: 4
    [gzserver-1] 
    [gzserver-1] [INFO] [1679262593.874780073] [task_manager]: All orders have been announced.




