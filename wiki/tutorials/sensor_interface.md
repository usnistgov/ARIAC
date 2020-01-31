-------------------------------------------------
- Go to [Wiki | Home](../README.md)
- Go to [Wiki | Documentation](documentation.md)
- Go to [Wiki | Tutorials](tutorials.md)
- Go to [Wiki | Qualifiers](qualifier.md)
- Go to [Wiki | Finals](finals.md)
- Go to [Wiki | News](updates.md)
-------------------------------------------------

# Overview #

The purpose of this tutorial is to introduce you to the sensors available to you in the Agile Robotics for Industrial Automation Competition (ARIAC) and how to interface with them from the command-line.

<img src="../figures/sensors.png" alt="alt text" width="900" class="center">

# Prerequisites #

You should have already completed the [GEAR interface tutorial](./gear_interface.md).

# Reading Sensor Data #

As described in the competition specifications, there are sensors available for you to place in the environment. How you can select which sensors to use is covered in the competition configuration specifications.

To start with, launch ARIAC with a sample workcell environment configuration that contains an arm and some sensors in various locations:

```bash
roslaunch osrf_gear sample_environment.launch
```

## Break Beam Sensor ##

<img src="../figures/break_beam.png" alt="alt text" width="900" class="center">

This is a simulated photoelectric sensor, such as the Sick W9L-3.
This sensor has a detection range of 1 meter and the binary output will tell you whether there is an object crossing the beam.
There are two ROS topics that show the output of the sensor:

* `/ariac/{sensor_name}`
* `/ariac/{sensor_name}_change`

```
+ Change this once source is pushed
```
An [osrf_gear/Proximity](https://bitbucket.org/osrf/ariac/src/master/osrf_gear/msg/Proximity.msg) message is periodically published on topic `/ariac/{sensor_name}`.
Run this command to see the message on the command line:

```bash
rostopic echo /ariac/break_beam_1
```

Alternatively, you could subscribe to the `/ariac/{sensor_name}_change` which will only show one message per transition from object not detected to object detected or vice verse.
Run this command to see the message on the command line:

```bash
rostopic echo /ariac/break_beam_1_change
```

## Proximity Sensor ##

<img src="../figures/proximity_sensor.png" alt="alt text" width="900" class="center">

This is a simulated ultrasound proximity sensor such as the SU2-A0-0A.
This sensor has a detection range of ~0.15 meters and the output will tell you how far an object is from the sensor.
The ROS topic `/ariac/{sensor_name}` publishes the data as [sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html).
Run this command to see the proximity sensor from the example world on the command line:

```bash
rostopic echo /ariac/proximity_sensor_1
```

The proximity sensor can be visualized in RViz using the **Range** display.
It helps to disable or decrease the marker scale on the **TF** display to see the cone of the range sensor in RViz.

## Laser Profiler ##
<img src="../figures/laser_profiler.png" alt="alt text" width="900" class="center">

This is a simulated 3D laser profiler such as the Cognex DS1300.
The output of the sensor is an array of ranges and intensities.
The size of the array is equal to the number of beams in the sensor.
The maximum range of each beam is ~0.725m.
The output of the sensor is periodically published on the topic `/ariac/{sensor_name}` as a [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html).
Run this command to see the output on the command line:

```bash
rostopic echo /ariac/laser_profiler_1
```

This can be visualized in RViz by adding a **LaserScan** display.

There is an offset between the position of the laser profiler and the origin of the data.
The position of the laser profiler is in a **tf** frame named `{sensor_name}_frame`, while the origin of the data is in a frame named `{sensor_name}_laser_source_frame`.

## Depth Camera ##
<img src="../figures/depth_camera.png" alt="alt text" width="900" class="center">

This is a simulated time-of-flight depth camera such as the Swissranger SR4000.
The output of the sensor is a [sensor_msgs/Pointcloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html).
Because of the large amount of data being published, if you want to echo the sensor message you probably want to use `--noarr` to suppress the data values on the command line:

```bash
rostopic echo /ariac/depth_camera_1 --noarr
```

The output of a depth camera can be visualized in RViz using the **PointCloud** display.

## Logical Camera ##

<img src="../figures/logical_camera.png" alt="alt text" width="900" class="center">

This is a simulated camera with a built-in object classification and localization system.
The sensor reports the position and orientation of the camera in the world, as well as a collection of the objects detected within its frustum.
The camera reports an object's type and pose from the camera reference frame as an [osrf_gear/LogicalCameraImage](https://bitbucket.org/osrf/ariac/src/master/osrf_gear/msg/LogicalCameraImage.msg) message.
In the sample environment there is a logical camera above one of the bins.
Run the following command to see the output of the logical camera:

```
rostopic echo /ariac/logical_camera_1
```

Logical cameras also publish `tf` transforms.
Use the TF2 library to calculate the pose of the products detected by the logical cameras in the `world` frame (see http://wiki.ros.org/tf2).

Here is an example using TF2 command-line tools to get a the pose of a part detected by a logical camera in world frame.
Note that the frame of the detected products is prefixed by the name of the camera that provides the transform, so if multiple cameras see the same product they will publish transforms with different frame names.

```bash
rosrun tf tf_echo world logical_camera_1_gasket_part_1_frame
At time 1120.405
- Translation: [-0.500, 0.282, 0.724]
- Rotation: in Quaternion [-0.003, -0.001, 0.388, 0.922]
            in RPY (radian) [-0.006, -0.000, 0.797]
            in RPY (degree) [-0.352, -0.025, 45.636]
```

For more information on working with TF frames programmatically see [the tf2 tutorials](http://wiki.ros.org/tf2/Tutorials).

**Note that GEAR uses tf2_msgs and not the deprecated tf_msgs. Accordingly, you should use the tf2 package instead of tf.**

-------------------------------------------------
- Go to [Wiki | Home](../README.md)
- Go to [Wiki | Documentation](documentation.md)
- Go to [Wiki | Tutorials](tutorials.md)
- Go to [Wiki | Qualifiers](qualifier.md)
- Go to [Wiki | Finals](finals.md)
- Go to [Wiki | News](updates.md)
-------------------------------------------------
