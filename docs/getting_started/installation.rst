
.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-cpp(code)
    :language: cpp

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

.. role:: bash(code)
    :language: bash

.. role:: inline-xml(code)
    :language: xml

.. role:: inline-yaml(code)
    :language: yaml

.. role:: underlined
    :class: underlined



Installing ARIAC
===========================

.. admonition:: Requirements
  :class: attention

    ARIAC 2023 is built for ROS2 Galactic running on Ubuntu 20.04 (Focal Fossa). It is not compatible with ROS2 Foxy or Ubuntu 18.04.

    - See the `ROS2 Galactic installation instructions <https://docs.ros.org/en/galactic/Installation.html>`_ for more information.


Cloning the ARIAC Repository
----------------------------

- Create a new ROS2 workspace:

    .. code-block:: bash

        source /opt/ros/galactic/setup.bash
        mkdir -p ~/ariac_ws/src
        cd ~/ariac_ws


- Clone the ARIAC repository:

    .. code-block:: bash
        
        git clone https://github.com/usnistgov/ARIAC.git src/ariac

    **Note:** Always use the ``ariac2023`` branch.


- Install the dependencies:

    .. code-block:: bash

        sudo apt install python3-rosdep
        sudo rosdep init
        rosdep update --include-eol-distros
        rosdep install --from-paths src -y --ignore-src

    **Note:** If you are using a virtual machine, you may need to install the ``python3-vcstool`` package.


Building the ARIAC Package
--------------------------

- Build the ARIAC package:

    .. code-block:: bash

        sudo apt install python3-colcon-common-extensions
        colcon build --packages-select ariac

- Source the workspace:

    .. code-block:: bash

        source install/setup.bash

    **Note:** You may need to add the following line to your ``~/.bashrc`` file: ``source ~/ariac_ws/install/setup.bash``

Starting the ARIAC Simulator
----------------------------

There are mainly two ways to start the ARIAC simulator.

Default Configuration
~~~~~~~~~~~~~~~~~~~~~

The following command starts ARIAC with the default configuration:

    .. code-block:: bash

        ros2 launch ariac ariac.launch.py

    - The default trial file is ``kitting.yaml``, located in ``~/ariac_ws/src/ariac/ariac_gazebo/config/trials/``

        - **Note:** All trial files must be placed in this folder.
    - The default sensor configuration is ``sensors.yaml``, located in ``~/ariac_ws/src/ariac/test_competitor/config/``

Custom Configuration
~~~~~~~~~~~~~~~~~~~~

To start ARIAC with a different trial, use the following command:

    .. code-block:: bash

        ros2 launch ariac ariac.launch.py trial_name:=<trial_file>

    Replace ``<trial_file>`` with the name of a trial file (without the ``.yaml`` extension). This trial file must be located in ``~/ariac_ws/src/ariac/ariac_gazebo/config/trials/``.
    
    **Example:** To start ARIAC with ``assembly.yaml`` trial file, run the following command:

        .. code-block:: bash

            ros2 launch ariac ariac.launch.py trial_name:=assembly

Competitors will need to create their own competitor package and use their own sensor configuration file.

        - To create a new competitor package, see :ref:`tutorial 1 <TUTORIAL1>`.
        - To use a custom sensor configuration file, create a directory named ``config`` in your competitor package and place your sensor configuration file in that directory. 
        - Make sure to edit ``CMakelists.txt`` in your competitor package to include the ``config`` directory.

            .. code-block:: cmake

                install(DIRECTORY config
                    DESTINATION share/${PROJECT_NAME}/
                )

        - Start ARIAC with a custom trial and with a custom sensor configuration file by running the following command:

            .. code-block:: bash

                ros2 launch ariac ariac.launch.py competitor_pkg:=<package> sensor_config:=<sensor_file> trial_name:=<trial_file>

            **Example:** To start ARIAC with ``assembly.yaml`` using ``my_sensors.yaml`` sensor configuration file (located in ``my_competitor_pkg/config``), run the following command:

                .. code-block:: bash

                    ros2 launch ariac ariac.launch.py competitor_pkg:=my_competitor_pkg sensor_config:=my_sensors trial_name:=assembly

             