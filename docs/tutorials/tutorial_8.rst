
.. _TUTORIAL8:

*******************************************
Tutorial 8: Move Robots with ROS2 Actions
*******************************************

.. admonition:: Tutorial 8
  :class: attention
  :name: tutorial_8

  - **Prerequisites:** :ref:`Introduction to Tutorials <TUTORIALS>` and :ref:`Tutorial 7 <TUTORIAL7>`
  - **Source Code**: `https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_8 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_8>`_ 
  - **Switch Branch**:

    .. code-block:: bash
        
            cd ~/ariac_ws/src/ariac_tutorials
            git switch tutorial_8


The code used in this tutorial is provided as a reference for how to use ROS2 actions to command robots. :topic:`MoveIt is not used in this tutorial`.
The main goal of this tutorial is to show how to use ROS2 actions to move both robots simultaneously.
Competitors interested in using ROS2 actions to command robots are encouraged to use this code as a starting point.
Competitors are encouraged to learn more about ROS2 actions by reading the `ROS2 actions documentation <https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html>`_.



Package Structure
=================

Updates and additions that are specific to :tuto:`Tutorial 8`  are highlighted in the tree below.

.. code-block:: text
    :emphasize-lines: 2, 11, 22
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
    |   ├── robot_commander.py
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
        ├── tutorial_7.py
        └── tutorial_8.py




Run the Executable
==================

- In *terminal 1*, run the following commands:


    .. code-block:: bash

        cd ~/ariac_ws
        colcon build
        . install/setup.bash
        ros2 run ariac_tutorials tutorial_8.py


    The node will wait until the competition is ready.


- In *terminal 2*, run the following commands:

    .. code-block:: bash

        cd ~/ariac_ws
        . install/setup.bash
        ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials


Outputs
=======

Both robots simultaneously move to their respective home positions.


