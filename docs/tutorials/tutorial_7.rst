
.. _TUTORIAL_7:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: inline-bash(code)
    :language: bash


Tutorial 7: Moving Robots with MoveIt
--------------------------------

.. admonition:: Source Code for Tutorial 7
  :class: attention
  :name: tutorial_7
  
  `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_7 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_7>`_ 

  .. code-block:: bash
    
        cd ~/ariac_ws/ariac_tutorials
        git checkout tutorial_7


This tutorial shows how to move the robots through service calls using the following steps:

  - Create a C++ class for interfacing with MoveIt (:inline-file:`src/robot_commander.cpp`).

    - Create a MoveGroupInterface object for each robot.
    - Create a service server for each robot. Service requests sent to these service servers will be used to move the robots to their home position.
  - Create a Python method for each robot that will be used to move the robot to its home position. Each one of these methods consists of a service client.
  - Call these methods from the main function to move the robots to their home positions.


Updates and additions that are specific to :inline-tutorial:`tutorial 7`  are highlighted in the tree below.

.. code-block:: text
    :emphasize-lines: 2, 6-7, 12-13, 21
    :class: no-copybutton
    
    ariac_tutorials
    ├── CMakeLists.txt
    ├── package.xml
    ├── config
    │   └── sensors.yaml
    ├── launch
    │   └── robot_commander.launch.py
    ├── ariac_tutorials
    │   ├── __init__.py
    │   ├── utils.py
    │   └── competition_interface.py
    ├── src
    │   └── robot_commander.cpp
    └── scripts
        ├── tutorial_1.py
        ├── tutorial_2.py
        ├── tutorial_3.py
        ├── tutorial_4.py
        ├── tutorial_5.py
        ├── tutorial_6.py
        └── tutorial_7.py


Overview of  CMakeLists.txt
--------------------------------

.. code-block:: cmake
    :emphasize-lines: 14, 33, 40, 45-46, 48-51, 53-55
    :linenos:

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
    find_package(moveit_ros_planning_interface REQUIRED)

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
    scripts/tutorial_4.py
    scripts/tutorial_5.py
    scripts/tutorial_6.py
    scripts/tutorial_7.py
    DESTINATION lib/${PROJECT_NAME}
    )

    # Install the config directory to the package share directory
    install(DIRECTORY 
    config
    launch
    DESTINATION share/${PROJECT_NAME}
    )

    # Install C++ executables
    add_executable(robot_commander 
    src/robot_commander.cpp)

    ament_target_dependencies(robot_commander 
    rclcpp
    moveit_ros_planning_interface 
    ariac_msgs)

    install(TARGETS
    robot_commander
    DESTINATION lib/${PROJECT_NAME})


    ament_package()



Competition Interface
--------------------------------

The competition interface from :ref:`Tutorial 6 <Tutorial6>` was augmented with the components described below.

.. code-block:: python

    # Service client for moving the floor robot to the home position
    self._move_floor_robot_home = self.create_client(
        Trigger, '/competitor/move_floor_robot_home')

    # Service client for moving the ceiling robot to the home position
    self._move_ceiling_robot_home = self.create_client(
        Trigger, '/competitor/move_ceiling_robot_home')

.. compound::
    
    Two service clients were added to the competition interface. Calls to these service clients will be used to move the robots to their home positions.


.. code-block:: python

    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._move_floor_robot_home.call_async(request)

        elif robot_name == 'ceiling_robot':
            if not self._move_ceiling_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return
            future = self._move_ceiling_robot_home.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)

This public method is used in the main function to move each robot to its home position. The method takes the name of the robot to move as an argument. The method then calls the appropriate service client to move the robot to its home position.


Create the Executable
--------------------------------

.. code-block:: python
    :caption: tutorial_7.py
    
    #!/usr/bin/env python3

    import rclpy
    from ariac_tutorials.competition_interface import CompetitionInterface


    def main(args=None):
        rclpy.init(args=args)
        interface = CompetitionInterface()
        interface.start_competition()

        interface.move_robot_home("floor_robot")
        interface.move_robot_home("ceiling_robot")

        interface.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

Code Explained
^^^^^^^^^^^^^^^^^^^^^^^

This executable does the following:

    - Create an instance of the class :inline-python:`CompetitionInterface` as a ROS node.
    - Start the competition.
    - The call to :inline-python:`move_robot_home("floor_robot")` sends a service request to ``/competitor/move_floor_robot_home``
    - The call to :inline-python:`move_robot_home("ceiling_robot")` sends a service request to ``/competitor/move_ceiling_robot_home``



Run the Executable
--------------------------------

- In *terminal 1*, run the following commands:


    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorials


    This command starts the environment.


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_tutorials robot_commander.launch.py

    The launch command starts the robot commander node and move it.

- In *terminal 3*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_7.py

    The last command starts the competition interface node and sends the service requests to move the robots to their home positions.

Outputs
--------------------------------

The output of the above commands show both robots moving to their home positions in Gazebo.


