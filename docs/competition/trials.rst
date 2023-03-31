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
