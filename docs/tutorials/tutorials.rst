.. _TUTORIALS:

=========================================================
Introduction to Tutorials
=========================================================

This section contains multiple tutorials on how to achieve basic tasks in ARIAC. Some competitors may find these tutorials useful as a starting point for their own code.
Some of these tutorials use both Python and C++ code. The Python code provides the interface to the ROS and ARIAC systems, while the C++ code provides the interface for robot controls using MoveIt.

The code for tutorials can be found in a `GitHub repository <https://github.com/jaybrecht/ariac_tutorials>`_ which is separate from the official NIST ARIAC repository. The code for each tutorial is located in a separate branch of the repository. The branch name is the same as the tutorial name.

A brief description of each tutorial is provided below.

- :tuto:`Tutorial 1:` :doc:`Create a Competition Package <tutorial_1>`
    
    - Demonstrates how to create a package and start the competition based on the state of the competition
- :tuto:`Tutorial 2:` :doc:`Read Data from a Break Beam Sensor <tutorial_2>`
    
    - Demonstrates how to add a sensor to the sensor configuration file. The sensor is then used to keep track of the number of parts that are spawned on the conveyor belt.
- :tuto:`Tutorial 3:` :doc:`Read Data from an Advanced Logical Camera <tutorial_3>`
    
    - Demonstrates how to add a logical camera to the sensor configuration file. The tutorial then retrieves information about parts detected by the camera and print the result in the terminal.
- :tuto:`Tutorial 4:` :doc:`Read an Order <tutorial_4>`
    
    - Demonstrates how to retrieve published orders and print the result in the terminal. Multiple OOP Python classes are used to store each order and its contents. 
- :tuto:`Tutorial 5:` :doc:`Move AGVs to Stations <tutorial_5>`
    
    - Demonstrates how to retrieve information from published assembly tasks and move the AGVs to the correct station. Service calls to lock and move AGVs are used in this tutorial.
- :tuto:`Tutorial 6:` :doc:`Enable/Disable a Gripper <tutorial_6>`
    
    - Demonstrates using service calls to activate/deactivate the gripper of the floor robot. 
- :tuto:`Tutorial 7:` :doc:`Move Robots with MoveIt <tutorial_7>`
    
    - Demonstrates how to use custom services to move the floor and the ceiling robots to their respective home positions. MoveIt is used in this tutorial. The service clients are implemented in Python while the service servers are hosted in C++.
- :tuto:`Tutorial 8:` :doc:`Move Robots with ROS2 Actions <tutorial_8>`

    - Demonstrates how to control both robots using ROS2 Actions.
.. important::

  Tutorials are meant to be followed in order as each tutorial uses the code from the previous tutorial to which new functionalities and files are added. This allows code from previous tutorials to be used in later tutorials. 

.. todo::

  - **Prerequisites:** :ref:`Installing ARIAC <INSTALLATION>`.
  - Clone the package :file:`ariac_tutorials` in the workspace :file:`~/ariac_ws` by running the following commands in the terminal:

    .. code-block:: bash
    
        cd ~/ariac_ws/src
        git clone https://github.com/jaybrecht/ariac_tutorials
        cd ..
        rosdep install --from-paths src -y --ignore-src
        colcon build
        source install/setup.bash