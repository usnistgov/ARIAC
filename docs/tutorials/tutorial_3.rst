
.. _TUTORIAL_3:

======================================
Tutorial #3: Reading Data from a Camera
======================================

.. note::
  Prerequisites: Complete tutorials 1 and 2.


In this tutorial you will learn how to:
  - Receive messages from a camera, 
  - Store the data internally in Python classes,
  - Display the data on the standard output.


Add a Camera to the Environment
--------------------------------

Add a an advanced logical camera to  ``sensors.yaml`` as shown on lines 8-13 in :numref:`sensors-yaml`. 

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




Testing the Camera
^^^^^^^^^^^^^^^^^^

To test  the camera was correctly added to the environment, do the following:

.. code-block:: bash

  cd ~/ariac_ws
  colcon build
  . install/setup.bash
  ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=competition_tutorials


You should see the camera above bins 1-4 as shown in the figure below:

.. figure:: ../images/tutorial3/advanced_camera_0.jpg
   :scale: 70 %
   :align: center
   :figclass: align-center
   :class: with-shadow

Receiving Messages from a Camera
---------------------------------

The camera which was added to ``sensors.yaml`` is publishing messages to the topic ``/ariac/advanced_camera_0``. Topics for sensors and cameras are dynamically generated based on the name used in ``sensors.yaml`` file. The topic name is the name of the sensor/camera prefixed with ``/ariac/``. For example, the topic for ``advanced_camera_0`` is ``/ariac/advanced_camera_0``.
To read messages published on the topic ``/ariac/advanced_camera_0``, create a subscriber in the ``competition_interface.py`` file.
