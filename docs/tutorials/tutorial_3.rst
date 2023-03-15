
.. _TUTORIAL_3:

======================================
Tutorial #3: Reading Data from a Camera
======================================

.. note::
  Prerequisites: Complete tutorials 1 and 2.


In this tutorial you will learn how to:
  - Read data from a camera, 
  - Store the data internally in Python classes,
  - Display the data on the standard output.


Add a Camera to the Environment
--------------------------------

Add a an advanced logical camera to  ``sensors.yaml`` as shown on lines 8-13 in :numref:`sensors-yaml`. The camera will be placed above bins 1, 2, 3, and 4. 

.. code-block:: yaml
    :caption: sensors.yaml
    :name: sensors-yaml
    :emphasize-lines: 8, 9, 10, 11, 12, 13
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


Testing the Camera
^^^^^^^^^^^^^^^^^^

To test  the camera was correctly added to the environment, do the following:

.. code-block:: bash
  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see the camera as shown in :numref:`fig_tutorial_3_image1`:


Retrieving Camera Messages
----------------------------

The camera which was added to 11sensors.yaml`` is publishing messages to the topic ``/ariac/advanced_camera_0``. Topics for sensors and cameras are dynamically generated based on the name of the sensor in the ``sensors.yaml`` file. The topic name is the name of the sensor/camera prefixed with ``/ariac/``. For example, the topic for the advanced logical camera is ``/ariac/advanced_camera_0``.
To read these messages, we will create a subscriber in the ``competition_interface.py`` file.
