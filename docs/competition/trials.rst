Configuration Files
==============================

Configuration files are described in the YAML format. YAML is a human-readable data serialization format. It is commonly used for configuration files and in applications where data is being stored or transmitted. For more information on YAML, see the `YAML website <http://yaml.org/>`_.

ARIAC consists of two main configuration files, which are described below in the following subsections.

Sensor Configuration File
--------------------------------

The sensor configuration file describes the location of the sensors in the workcell. One example of sensor configuration file (`sensors.yaml`) is provided in the `test_competitor <https://github.com/usnistgov/ARIAC/tree/ariac2023/test_competitor/config>`_ package.

Below is a description of the different fields in the sensor configuration file. This file contains 4 sensors which have to be described under the field :yamlname:`sensors` and each sensor consists of:

#. A name (e.g. :yaml:`right_bins_camera`). This name has to be unique among all sensors in the same configuration file.
#. A type (e.g. :yaml:`advanced_logical_camera`). This type has to be one of the types defined in the :yamlname:`sensor_types` field.

    * :yaml:`break_beam`
    * :yaml:`proximity`
    * :yaml:`laser_profiler`
    * :yaml:`lidar`
    * :yaml:`rgb_camera`
    * :yaml:`rgbd_camera`
    * :yaml:`basic_logical_camera`
    * :yaml:`advanced_logical_camera`
#. A pose (defined in the world frame):

    * Position (e.g. :yaml:`xyz: [-2.286, 2.96, 1.8]`).
    * Orientation (e.g. :yaml:`rpy: [pi, pi/2, 0]`). The orientation is defined using the `roll-pitch-yaw <https://en.wikipedia.org/wiki/Euler_angles>`_ convention. The orientation is defined in radians and can be defined using floating-point values or with the :yaml:`pi` constant (:yaml:`pi`, :yaml:`pi/2`, :yaml:`pi/4`, etc.).

:numref:`sensor-configuration-file` shows an example of a sensor configuration file. The field :yamlname:`visualize_fov` is optional and can be used to visualize the field of view of the sensor. The field :yamlname:`visualize_fov` can be set to :yaml:`true` or :yaml:`false`. If the field :yamlname:`visualize_fov` is not defined, the field of view will not be visualized.

.. code-block:: yaml
      :caption: An example of a sensor configuration file.
      :name: sensor-configuration-file

      sensors:
        breakbeam_0:
            type: break_beam
            visualize_fov: true
            pose:
                xyz: [-0.35, 3, 0.95]
                rpy: [0, 0, pi]

        proximity_sensor_0:
            type: proximity
            visualize_fov: true
            pose:
                xyz: [-0.573, 2.84, 1]
                rpy: [pi/2, pi/6, pi/2]

        laser_profiler_0:
            type: laser_profiler
            visualize_fov: true
            pose:
                xyz: [-0.573, 1.486, 1.526]
                rpy: [pi/2, pi/2, 0]

        lidar_0:
            type: lidar
            visualize_fov: false
            pose:
                xyz: [-2.286, -2.96, 1.8]
                rpy: [pi, pi/2, 0]

        rgb_camera_0:
            type: rgb_camera
            visualize_fov: false
            pose:
                xyz: [-2.286, 2.96, 1.8]
                rpy: [pi, pi/2, 0]

        rgbd_camera_0:
            type: rgbd_camera
            visualize_fov: false
            pose:
                xyz: [-2.286, 4.96, 1.8]
                rpy: [pi, pi/2, 0]

        basic_logical_camera_0:
            visualize_fov: false
            type: basic_logical_camera
            pose:
                xyz: [-2.286, 2.96, 1.8]
                rpy: [pi, pi/2, 0]

        advanced_logical_camera_0:
            visualize_fov: false
            type: advanced_logical_camera
            pose:
                xyz: [-2.286, -2.96, 1.8]
                rpy: [pi, pi/2, 0]