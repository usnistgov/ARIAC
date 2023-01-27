# Configuration Files

Configuration files are described in the YAML format. YAML is a human-readable data serialization format. It is commonly used for configuration files and in applications where data is being stored or transmitted. For more information on YAML, see the [YAML website](http://yaml.org/).

ARIAC consists of two main configuration files, which are described below in the following subsections.

## Sensor Configuration File

The sensor configuration file describes the location of the sensors on the competition field. One example of sensor configuration file is provided in the [test_competitor](../../test_competitor/config/sensors.yaml) package.

Below is a description of the different fields in the sensor configuration file. This file contains 4 sensors which have to be described under the field `sensors` and each sensor consists of:

1. A name (e.g. `right_bins_camera`). This name has to be unique among all sensors in the same configuration file.
2. A type (e.g. `advanced_logical_camera`). This type has to be one of the following types described in the section [Sensors](sensors.md).

   * `break_beam`
   * `proximity`
   * `laser_profiler`
   * `lidar`
   * `rgb_camera`
   * `rgbd_camera`
   * `basic_logical_camera`
   * `advanced_logical_camera`
3. A pose (defined in the world frame):
    * Position (e.g. `xyz: [-2.286, 2.96, 1.8]`).
    * Orientation (e.g. `rpy: [pi, pi/2, 0]`). The orientation is defined using the [roll-pitch-yaw](https://en.wikipedia.org/wiki/Euler_angles) convention. The orientation is defined in radians and can be defined using floating-point values or with the following constants:
      * `pi`: 3.141592653589793
      * `pi/2`: 1.5707963267948966
      * `pi/4`: 0.7853981633974483
      * etc

    ```yaml
    sensors:
      right_bins_camera:
          type: advanced_logical_camera
          pose:
          xyz: [-2.286, 2.96, 1.8]
          rpy: [pi, pi/2, 0]

      left_bins_camera:
          type: advanced_logical_camera
          pose:
          xyz: [-2.286, -2.96, 1.8]
          rpy: [pi, pi/2, 0]

      kts1_camera:
          type: advanced_logical_camera
          pose:
          xyz: [-1.3, -5.8, 1.8]
          rpy: [pi, pi/2, pi/2]
      
      kts2_camera:
          type: advanced_logical_camera
          pose:
          xyz: [-1.3, 5.8, 1.8]
          rpy: [pi, pi/2, -pi/2]
    ```

### Placing Sensors in the Environment

To add sensors in the environment, one can start the simulation environment and use Gazebo's GUI to add sensors. The sensors can be added by clicking on the `Insert` button and then selecting the desired sensor type. The sensors can be placed in the environment by clicking on the `Move` button and then clicking on the desired location in the environment. The sensors can be rotated by clicking on the `Rotate` button and then clicking on the desired orientation in the environment.  The sensors can be deleted by clicking on the `Delete` button and then clicking on the desired sensor in the environment. Once the sensors are placed in the environment, the sensor configuration file can be updated with the new sensor information.

Another way to place sensors is to add them in the sensor configuration file and then run the simulation environment. The sensors will be added to the environment automatically. They can the been moved and rotated in the environment. Once the sensors are placed in the environment, the sensor configuration file can be updated with the new sensor information.

## Trial Configuration File

Trials are the main way to test your robot's performance. Multiple trials are used during the qualifiers and the finals. The results of each trial are recorded and then used to rank competitors.
