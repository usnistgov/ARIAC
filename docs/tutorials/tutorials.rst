.. _TUTORIALS:

.. only:: builder_html or readthedocs

.. role:: inline-python(code)
    :language: python

.. role:: inline-file(file)

.. role:: inline-tutorial(file)

=========================================================
Introduction to Tutorials
=========================================================

This section contains multiple tutorials on how to achieve basic tasks in ARIAC using Python. Some competitors may find these tutorials useful as a starting point for their own code.
Some of these tutorials use both Python and C++ code. The Python code provides the interface to the ROS and ARIAC systems, while the C++ code provides the interface for robot controls with and without Moveit.

The code for tutorials can be found in a Github repository which is separated from the official NIST ARIAC repository. 
The Github repository can be found here: `ariac_tutorials <https://github.com/jaybrecht/ariac_tutorials>`_
The code for each tutorial is located in a separate branch of the repository. The branch name is the same as the tutorial name.

A brief description of each tutorial is provided below.

- :inline-tutorial:`Tutorial 1` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_1>`_ ): This tutorial shows how to start the competition based on the state of the competition.
- :inline-tutorial:`Tutorial 2` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_2 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_2>`_ ): This tutorial shows how to add a sensor to the sensor configuration file. The sensor is then used to keep track of the number of parts that are spawned on the conveyor belt.
- :inline-tutorial:`Tutorial 3` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_3 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_3>`_ ): This tutorial shows how to add a logical camera to the sensor configuration file. The tutorial then retrieves information about parts detected by the camera and print the result in the terminal.
- :inline-tutorial:`Tutorial 4` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_4>`_ ): This tutorial shows how to retrieve published orders and print the result in the terminal. Multiple OOP Python classes are used to store each order and its contents.
- :inline-tutorial:`Tutorial 5` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_5 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_5>`_ ): This tutorial retrieves information from published assembly tasks and moves the AGVs to the correct station. Service calls to lock and move AGVs are used in this tutorial.
- :inline-tutorial:`Tutorial 6` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_6 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_6>`_ ): This tutorial uses service calls to activate/deactivate the gripper of the floor robot.
- :inline-tutorial:`Tutorial 7` (`https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_7 <https://github.com/jaybrecht/ariac_tutorials/tree/tutorial_7>`_ ): This tutorial shows how to use custom services to move the floor and the ceiling robots to their respective home positions. MoveIt is used in this tutorial. The service clients are implemented in Python while the service servers are hosted in C++. 

.. important::

  Each tutorial uses the code from the previous tutorial to which new functionalities and files are added. This means that code from previous tutorials can be used in later tutorials.


.. todo::

  Clone the package :inline-file:`ariac_tutorials` in the workspace :inline-file:`~/ariac_ws` by running the following command in the terminal:

    .. code-block:: bash
    
        cd ~/ariac_ws/src
        git clone https://github.com/jaybrecht/ariac_tutorials
        cd ..
        colcon build
        . install/setup.bash