.. _INSTALLATION:


Installing ARIAC
===========================

.. admonition:: Requirements
  :class: attention

    ARIAC 2023 is built for ROS2 Galactic running on Ubuntu 20.04 (Focal Fossa). 

    - See the `ROS2 Galactic installation instructions <https://docs.ros.org/en/galactic/Installation.html>`_ for more information.


Cloning the ARIAC Repository
----------------------------

- Create a new ROS2 workspace:

    .. code-block:: bash
        :class: highlight

        source /opt/ros/galactic/setup.bash
        mkdir -p ~/ariac_ws/src
        cd ~/ariac_ws


- Clone the ARIAC repository:

    .. code-block:: bash
        :class: highlight
        
        git clone https://github.com/usnistgov/ARIAC.git src/ariac

    
    **Note:** Always use the ``ariac2023`` branch.


- Install the dependencies:

    .. code-block:: bash
        :class: highlight

        sudo apt install python3-rosdep
        sudo apt install openjdk-17-jdk
        sudo rosdep init
        rosdep update --include-eol-distros
        rosdep install --from-paths src -y --ignore-src

    .. note::

        - :file:`openjdk-17-jdk` is required to install the Java 17 JDK for the human challenge.
        - If you are using a virtual machine, you may need to install the :file:`python3-vcstool` package.


Building the ARIAC Package
--------------------------

- Build the ARIAC package:

    .. code-block:: bash
        :class: highlight

        sudo apt install python3-colcon-common-extensions
        colcon build --packages-select ariac

- Source the workspace:

    .. code-block:: bash
        :class: highlight

        source install/setup.bash

    **Note:** You may want to add the following line to your :file:`~/.bashrc` file: :bash:`source ~/ariac_ws/install/setup.bash`

Starting the ARIAC Simulator
----------------------------

There are mainly two ways to start the ARIAC simulator.

Default Configuration
~~~~~~~~~~~~~~~~~~~~~

The following command starts ARIAC with the default configuration:

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py

    - The default trial file is :file:`kitting.yaml`, located in `ariac_gazebo/config/trials/ <https://github.com/usnistgov/ARIAC/tree/ariac2023/ariac_gazebo/config/trials>`_

        - **Note:** All trial files must be placed in this folder.
    - The default sensor configuration is :file:`sensors.yaml`, located in `test_competitor/config/ <https://github.com/usnistgov/ARIAC/tree/ariac2023/test_competitor/config>`_

Custom Configuration
~~~~~~~~~~~~~~~~~~~~

- To start ARIAC with a different trial, use the following command:

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=<trial_file>

    Replace :samp:`{<trial_file>}` with the name of a trial file (without the ``.yaml`` extension). **Reminder:** This trial file **MUST** be placed in :file:`ariac_gazebo/config/trials/`.
    
    **Example:** To start ARIAC with :file:`assembly.yaml` trial file, run the following command:

        .. code-block:: console
            :class: highlight

            ros2 launch ariac_gazebo ariac.launch.py trial_name:=assembly

- Competitors will need to create their own competitor package and use their own sensor configuration file.

        - To create a new competitor package, see :ref:`tutorial 1 <TUTORIAL1>`.
        - To use a custom sensor configuration file, create a directory named :file:`config` in your competitor package and place your sensor configuration file in that directory. 

            - Below is an example of competitor package structure with a custom sensor configuration file named :file:`my_sensors.yaml`.

            .. code-block:: text
                :class: no-copybutton
                
                my_competitor_pkg
                ├── CMakeLists.txt
                ├── package.xml
                └── config
                    └── my_sensors.yaml

        - Make sure to edit :file:`CMakelists.txt` in your competitor package to include the :file:`config` directory.

            .. code-block:: cmake

                install(DIRECTORY config
                    DESTINATION share/${PROJECT_NAME}/
                )

        - Start ARIAC with a custom trial and with a custom sensor configuration file by running the following command:

            .. code-block:: console
                :class: highlight

                ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=<package> sensor_config:=<sensor_file> trial_name:=<trial_file>

            **Example:** To start ARIAC with :file:`assembly.yaml` using :file:`my_sensors.yaml` sensor configuration file (located in :file:`my_competitor_pkg/config`), run the following command:

                .. code-block:: console
                    :class: highlight

                    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=my_competitor_pkg sensor_config:=my_sensors trial_name:=assembly


Moving the Robots
-----------------

To verify that the robots can be controlled properly you will need three terminals:

- *terminal 1*: Start the environment.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py


- *terminal 2*: Start the moveit node.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

- *terminal 3*: Start the moveit test node.

    .. code-block:: console
        :class: highlight

        ros2 launch test_competitor moveit_test.launch.py


This should start the competition and move each of the robots to the home position. It will also open an RVIZ window showing the robot's planning scene. 


Running the Test Competitor
---------------------------

A test competitor has been created to demonstrate how to complete some of the basic functions (no challenges) of working with the ARIAC environment.
The test competitor has been tested with ``kitting.yaml``, ``assembly.yaml``, ``combined.yaml``, :class: :file:`kitting_assembly.yaml`, and :file:`kitting_combined.yaml`.
There is no guarantee that the test competitor will work with other trials as the goal of the test competitor is to demonstrate how to interface with the ARIAC environment.


The test competitor is located in the `test_competitor <https://github.com/usnistgov/ARIAC/tree/ariac2023/test_competitor>`_ package. To run the test competitor, use the following commands:

- *terminal 1*: Start the environment.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_gazebo ariac.launch.py trial_name:=<trial_file>


- *terminal 2*: Start the MoveIt node.

    .. code-block:: console
        :class: highlight

        ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py

- *terminal 3*: Start the competitor node.

    .. code-block:: console
        :class: highlight

        ros2 launch test_competitor competitor.launch.py

The test competitor will start the competition, subscribe to camera and orders topics, and complete orders. 
