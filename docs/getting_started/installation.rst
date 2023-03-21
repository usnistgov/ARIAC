
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

- Start the ARIAC simulator:

    .. code-block:: bash

        ros2 launch ariac ariac.launch.py

    **Note:** This command starts ARIAC with the default configuration:

    - The default trial file is ``kitting.yaml``, located in ``~/ariac_ws/src/ariac/ariac_gazebo/config/trials/kitting.yaml``.
    - The default sensor configuration is ``sensors.yaml``, located in ``~/ariac_ws/src/ariac/test_competitor/config/sensors.yaml``.


    .. admonition:: Custom Configuration
        :class: danger

        To start ARIAC with a custom configuration, you must specify the ``trial_file`` and ``sensor_config`` arguments. For example:

        

    