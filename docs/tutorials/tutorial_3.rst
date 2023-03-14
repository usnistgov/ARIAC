
.. _TUTORIAL_3:

======================================
Tutorial 3: Reading Data from a Camera
======================================

.. note::
  Prerequisites: :ref:`TUTORIAL_1` and :ref:`TUTORIAL_2`


In this tutorial you will learn how to: 1) Read data from a camera, 2) store the data internally in Python classes, and 3) display the data on the standard output.

We will reuse the ``sensors.yaml`` file from tutorial 2 to add an advanced logical camera to the environment. 
To start, create a [custom sensor configuration](../competition/sensors.md). Navigate to ``ariac_ws/src/competition_tutorials`` and run the following command:

Add a new entry to  ```sensors.yaml```:

.. code-block:: yaml
    :linenos:
    
    sensors:
        breakbeam_0:
            type: break_beam
            visualize_fov: true
            pose:
                xyz: [-0.36, 3.5, 0.88]
                rpy: [0, 0, pi]
        advanced_camera_0:
            type: advanced_logical_camera
            visualize_fov: true
            pose:
                xyz: [-2.286, 2.96, 1.8]
                rpy: [pi, pi/2, 0]



This will add an advanced logical camera above bins 1, 2, 3, and 4.


To test that this worked, build the workspace. In ``ariac_ws`` run

.. code-block:: bash

  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see the camera as shown in :numref:`fig_tutorial_3_image1`:


Retrieving Camera Messages
----------------------------

The camera which was added to 11sensors.yaml`` is publishing messages to the topic ``/ariac/advanced_camera_0``. Topics for sensors and cameras are dynamically generated based on the name of the sensor in the ``sensors.yaml`` file. The topic name is the name of the sensor/camera prefixed with ``/ariac/``. For example, the topic for the advanced logical camera is ``/ariac/advanced_camera_0``.
To read these messages, we will create a subscriber in the ``competition_interface.py`` file.
